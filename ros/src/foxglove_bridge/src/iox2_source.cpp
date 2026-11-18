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

constexpr auto OPEN_RETRY_INTERVAL = std::chrono::seconds(1);

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
  _openTimer = _node.create_wall_timer(OPEN_RETRY_INTERVAL, [this]() {
    openPending();
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

    if (!iox2::ServiceName::create(cfg.serviceName.c_str()).has_value()) {
      throw std::runtime_error("iox2 source: invalid iox2 service name '" + cfg.serviceName + "'");
    }
    if (!iox2::bb::FilePath::create(bfbsPath.string().c_str()).has_value()) {
      throw std::runtime_error("iox2 source: invalid schema file path '" + bfbsPath.string() + "'");
    }

    _entries.push_back(std::make_unique<Entry>(Entry{cfg.serviceName, cfg.topic, bfbsPath,
                                                     std::move(channelResult.value()), std::nullopt,
                                                     std::nullopt, std::nullopt, false}));
    RCLCPP_INFO(_node.get_logger(), "iox2 source: '%s' -> Foxglove '%s' (%s)",
                cfg.serviceName.c_str(), cfg.topic.c_str(), cfg.schemaName.c_str());
    tryOpen(*_entries.back());
  }
}

void Iox2Source::tryOpen(Entry& entry) {
  const auto name = iox2::ServiceName::create(entry.serviceName.c_str()).value();
  const auto schemaFile = iox2::bb::FilePath::create(entry.bfbsPath.string().c_str()).value();
  // Never create: a service created with iceoryx2's defaults would refuse publishers asking for
  // more. The .bfbs makes iceoryx2 reject a service with a different schema.
  auto service = _iox2Node->service_builder(name)
                   .publish_subscribe<Payload>()
                   .flatbuffer_schema_path(schemaFile)
                   .open();
  if (!service.has_value()) {
    const auto error = service.error();
    if (entry.lastOpenError != error) {
      if (error == iox2::PublishSubscribeOpenError::DoesNotExist) {
        RCLCPP_INFO(_node.get_logger(), "iox2 source: waiting for a publisher to create '%s'",
                    entry.serviceName.c_str());
      } else {
        RCLCPP_ERROR(_node.get_logger(), "iox2 source: cannot open service '%s', retrying: %s",
                     entry.serviceName.c_str(), iox2::bb::into<const char*>(error));
      }
    }
    entry.lastOpenError = error;
    return;
  }

  auto subscriber = service.value().subscriber_builder().create();
  if (!subscriber.has_value()) {
    RCLCPP_ERROR(_node.get_logger(), "iox2 source: cannot subscribe to '%s', retrying",
                 entry.serviceName.c_str());
    return;
  }
  entry.service.emplace(std::move(service.value()));
  entry.subscriber.emplace(std::move(subscriber.value()));
  if (entry.lastOpenError.has_value()) {
    RCLCPP_INFO(_node.get_logger(), "iox2 source: opened '%s'", entry.serviceName.c_str());
  }
  entry.lastOpenError.reset();
}

void Iox2Source::openPending() {
  for (auto& entry : _entries) {
    if (entry && !entry->subscriber.has_value()) {
      tryOpen(*entry);
    }
  }
}

bool Iox2Source::drain(Entry& entry) {
  while (true) {
    auto sample = entry.subscriber->receive();
    if (!sample.has_value()) {
      RCLCPP_WARN(_node.get_logger(), "iox2 source: receive error on '%s', disabling",
                  entry.serviceName.c_str());
      return false;
    }
    if (!sample->has_value()) {
      break;  // queue drained
    }

    // Log time is the receive time; message timestamps live inside the FlatBuffer.
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
    if (!entry || !entry->subscriber.has_value()) {
      continue;
    }
    if (!drain(*entry)) {
      entry.reset();
    }
  }
}

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
