#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include "foxglove_bridge/iox2_source.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <foxglove/error.hpp>

namespace foxglove_bridge {
namespace {

std::string loadBinaryFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("iox2 source: cannot open schema file: " + path.string());
  }
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

}  // namespace

Iox2Source::Iox2Source(rclcpp::Node& node, const foxglove::Context& context)
    : _node(node) {
  const auto configs = loadParameters();
  if (configs.empty()) {
    RCLCPP_INFO(_node.get_logger(), "iox2 source: no services configured, disabled");
    return;
  }

  setupSubscribers(context, configs);

  const auto pollMs =
    std::chrono::milliseconds(_node.get_parameter("iox2.poll_interval_ms").as_int());
  _pollTimer = _node.create_wall_timer(pollMs, [this]() {
    pollOnce();
  });

  RCLCPP_INFO(_node.get_logger(), "iox2 source: bridging %zu service(s) into the Foxglove server",
              _entries.size());
}

Iox2Source::~Iox2Source() = default;

std::vector<Iox2Source::Config> Iox2Source::loadParameters() {
  _node.declare_parameter("iox2.schema_dir", std::string(""));
  _node.declare_parameter("iox2.poll_interval_ms", 1);
  _node.declare_parameter("iox2.service_names", std::vector<std::string>{});
  _node.declare_parameter("iox2.topics", std::vector<std::string>{});
  _node.declare_parameter("iox2.schema_names", std::vector<std::string>{});
  _node.declare_parameter("iox2.bfbs_files", std::vector<std::string>{});

  _schemaDir = _node.get_parameter("iox2.schema_dir").as_string();
  const auto serviceNames = _node.get_parameter("iox2.service_names").as_string_array();
  const auto topics = _node.get_parameter("iox2.topics").as_string_array();
  const auto schemaNames = _node.get_parameter("iox2.schema_names").as_string_array();
  const auto bfbsFiles = _node.get_parameter("iox2.bfbs_files").as_string_array();

  if (serviceNames.size() != topics.size() || serviceNames.size() != schemaNames.size() ||
      serviceNames.size() != bfbsFiles.size()) {
    throw std::runtime_error(
      "iox2 source: parameters service_names, topics, schema_names and bfbs_files must have equal "
      "length");
  }

  std::vector<Config> configs;
  configs.reserve(serviceNames.size());
  for (size_t i = 0; i < serviceNames.size(); ++i) {
    configs.push_back({serviceNames[i], topics[i], schemaNames[i], bfbsFiles[i]});
  }
  return configs;
}

void Iox2Source::setupSubscribers(const foxglove::Context& context,
                                  const std::vector<Config>& configs) {
  auto nodeName = iox2::NodeName::create("foxglove_bridge_iox2");
  if (!nodeName.has_value()) {
    throw std::runtime_error("iox2 source: failed to create iox2 NodeName");
  }
  auto nodeResult = iox2::NodeBuilder().name(nodeName.value()).create<iox2::ServiceType::Ipc>();
  if (!nodeResult.has_value()) {
    throw std::runtime_error("iox2 source: failed to create iox2 Node");
  }
  _iox2Node = std::make_unique<IpcNode>(std::move(nodeResult.value()));

  _entries.reserve(configs.size());
  for (const auto& cfg : configs) {
    const auto bfbsPath = _schemaDir / cfg.bfbsFile;
    const auto bfbs = loadBinaryFile(bfbsPath);
    const foxglove::Schema schema{cfg.schemaName, "flatbuffer",
                                  reinterpret_cast<const std::byte*>(bfbs.data()), bfbs.size()};

    auto channelResult = foxglove::RawChannel::create(cfg.topic, "flatbuffer", schema, context);
    if (!channelResult.has_value()) {
      throw std::runtime_error("iox2 source: failed to create channel for topic '" + cfg.topic +
                               "': " + foxglove::strerror(channelResult.error()));
    }

    auto name = iox2::ServiceName::create(cfg.serviceName.c_str());
    if (!name.has_value()) {
      throw std::runtime_error("iox2 source: invalid iox2 service name '" + cfg.serviceName + "'");
    }
    auto schemaFile = iox2::bb::FilePath::create(bfbsPath.string().c_str());
    if (!schemaFile.has_value()) {
      throw std::runtime_error("iox2 source: invalid schema file path '" + bfbsPath.string() + "'");
    }

    // The .bfbs is registered as the service's schema: iceoryx2 stores its content when the
    // service is created and rejects any participant whose file differs, so an incompatible
    // publisher fails here instead of producing undecodable channels. A failed open is logged
    // and the service skipped rather than taking down the whole bridge (iceoryx2 logs the
    // detailed cause itself).
    auto service = _iox2Node->service_builder(name.value())
                     .publish_subscribe<Payload>()
                     .flatbuffer_schema_path(schemaFile.value())
                     .open_or_create();
    if (!service.has_value()) {
      RCLCPP_ERROR(_node.get_logger(),
                   "iox2 source: failed to open service '%s' (error %d, likely a schema mismatch "
                   "with the publisher's registered '%s'); skipping this service",
                   cfg.serviceName.c_str(), static_cast<int>(service.error()),
                   cfg.bfbsFile.c_str());
      continue;
    }

    auto subscriber = service.value().subscriber_builder().create();
    if (!subscriber.has_value()) {
      throw std::runtime_error("iox2 source: failed to create subscriber for service '" +
                               cfg.serviceName + "'");
    }

    _entries.push_back(std::unique_ptr<Entry>(
      new Entry{cfg.serviceName, cfg.topic, std::move(channelResult.value()),
                std::move(service.value()), std::move(subscriber.value()), false}));

    RCLCPP_INFO(_node.get_logger(), "iox2 source: '%s' -> Foxglove '%s' (%s)",
                cfg.serviceName.c_str(), cfg.topic.c_str(), cfg.schemaName.c_str());
  }
}

bool Iox2Source::drain(Entry& entry) {
  while (true) {
    auto sample = entry.subscriber.receive();
    if (!sample.has_value()) {
      // Fatal receive error. Returning false makes the caller drop this entry, so this
      // warning fires exactly once per service rather than on every poll.
      RCLCPP_WARN(_node.get_logger(), "iox2 source: receive error on '%s', disabling",
                  entry.serviceName.c_str());
      return false;
    }
    if (!sample->has_value()) {
      break;  // queue drained
    }

    // The serialized FlatBuffer as located by iceoryx2 itself; no framing to validate. Log
    // time is the receive time (std::nullopt lets the SDK stamp it): shared-memory latency is
    // negligible, and message-level timestamps live inside the FlatBuffer.
    const auto bytes = sample->value().payload_bytes();
    const auto err =
      entry.channel.log(reinterpret_cast<const std::byte*>(bytes.data()), bytes.number_of_bytes());
    if (err != foxglove::FoxgloveError::Ok && !entry.warnedLogError) {
      entry.warnedLogError = true;
      RCLCPP_WARN(_node.get_logger(), "iox2 source: log failed on topic '%s': %s",
                  entry.topic.c_str(), foxglove::strerror(err));
    }
  }
  return true;
}

void Iox2Source::pollOnce() {
  for (auto& entry : _entries) {
    if (!entry) {
      continue;
    }
    if (!drain(*entry)) {
      entry.reset();  // fatal receive error: stop polling this service
    }
  }
}

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
