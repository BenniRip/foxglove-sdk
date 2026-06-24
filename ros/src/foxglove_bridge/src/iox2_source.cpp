#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include "foxglove_bridge/iox2_source.hpp"

#include <chrono>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include <iox2/type_name.hpp>

#include <foxglove/error.hpp>

// Decouple the iceoryx2 user-header type identity from the C++ type: resolve the type name at
// runtime from foxglove_bridge::iox2UserHeaderTypeName(), so it can be overridden via a
// parameter to match any publisher's header type name without recompiling. This specialization
// must be visible before the service builder is instantiated for Iox2MessageHeader.
IOX2_DEFINE_TYPE_NAME(foxglove_bridge::Iox2MessageHeader,
                      foxglove_bridge::iox2UserHeaderTypeName());

namespace foxglove_bridge {
namespace {

std::string& userHeaderTypeNameStorage() {
  static std::string storage = "foxglove_bridge::Iox2MessageHeader";
  return storage;
}

std::string loadBinaryFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("iox2 source: cannot open schema file: " + path.string());
  }
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

}  // namespace

const char* iox2UserHeaderTypeName() {
  return userHeaderTypeNameStorage().c_str();
}

void setIox2UserHeaderTypeName(const std::string& name) {
  if (!name.empty()) {
    userHeaderTypeNameStorage() = name;
  }
}

Iox2Source::Iox2Source(rclcpp::Node& node, const foxglove::Context& context)
    : node_(node) {
  const auto configs = loadParameters();
  if (configs.empty()) {
    RCLCPP_INFO(node_.get_logger(), "iox2 source: no services configured, disabled");
    return;
  }

  // Apply the configured iceoryx2 user-header type name before opening any service, so the
  // service builder picks it up via the IOX2_DEFINE_TYPE_NAME specialization above.
  setIox2UserHeaderTypeName(userHeaderTypeName_);

  setupSubscribers(context, configs);

  const auto pollMs =
    std::chrono::milliseconds(node_.get_parameter("iox2.poll_interval_ms").as_int());
  pollTimer_ = node_.create_wall_timer(pollMs, [this]() {
    pollOnce();
  });

  RCLCPP_INFO(node_.get_logger(), "iox2 source: bridging %zu service(s) into the Foxglove server",
              entries_.size());
}

Iox2Source::~Iox2Source() = default;

std::vector<Iox2Source::Config> Iox2Source::loadParameters() {
  node_.declare_parameter("iox2.schema_dir", std::string(""));
  node_.declare_parameter("iox2.poll_interval_ms", 1);
  node_.declare_parameter("iox2.user_header_type_name", std::string(""));
  node_.declare_parameter("iox2.service_names", std::vector<std::string>{});
  node_.declare_parameter("iox2.topics", std::vector<std::string>{});
  node_.declare_parameter("iox2.schema_names", std::vector<std::string>{});
  node_.declare_parameter("iox2.bfbs_files", std::vector<std::string>{});

  schemaDir_ = node_.get_parameter("iox2.schema_dir").as_string();
  userHeaderTypeName_ = node_.get_parameter("iox2.user_header_type_name").as_string();
  const auto serviceNames = node_.get_parameter("iox2.service_names").as_string_array();
  const auto topics = node_.get_parameter("iox2.topics").as_string_array();
  const auto schemaNames = node_.get_parameter("iox2.schema_names").as_string_array();
  const auto bfbsFiles = node_.get_parameter("iox2.bfbs_files").as_string_array();

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
  iox2Node_ = std::make_unique<IpcNode>(std::move(nodeResult.value()));

  entries_.reserve(configs.size());
  for (const auto& cfg : configs) {
    const auto bfbs = loadBinaryFile(schemaDir_ / cfg.bfbsFile);
    const foxglove::Schema schema{cfg.schemaName, "flatbuffer",
                                  reinterpret_cast<const std::byte*>(bfbs.data()), bfbs.size()};

    auto channelResult = foxglove::RawChannel::create(cfg.topic, "flatbuffer", schema, context);
    if (!channelResult.has_value()) {
      throw std::runtime_error("iox2 source: failed to create channel for topic '" + cfg.topic +
                               "': " + foxglove::strerror(channelResult.error()));
    }

    auto serviceName = iox2::ServiceName::create(cfg.serviceName.c_str());
    if (!serviceName.has_value()) {
      throw std::runtime_error("iox2 source: invalid iox2 service name '" + cfg.serviceName + "'");
    }
    auto service = iox2Node_->service_builder(serviceName.value())
                     .template publish_subscribe<iox2::bb::Slice<uint8_t>>()
                     .user_header<MsgHeader>()
                     .open_or_create();
    if (!service.has_value()) {
      throw std::runtime_error("iox2 source: failed to open iox2 service '" + cfg.serviceName +
                               "'");
    }
    auto subscriber = service.value().subscriber_builder().create();
    if (!subscriber.has_value()) {
      throw std::runtime_error("iox2 source: failed to create subscriber for service '" +
                               cfg.serviceName + "'");
    }

    entries_.push_back(Entry{
      cfg.serviceName,
      cfg.topic,
      std::move(channelResult.value()),
      std::make_unique<PubSubService>(std::move(service.value())),
      std::make_unique<Iox2Subscriber>(std::move(subscriber.value())),
    });

    RCLCPP_INFO(node_.get_logger(), "iox2 source: '%s' -> Foxglove '%s' (%s)",
                cfg.serviceName.c_str(), cfg.topic.c_str(), cfg.schemaName.c_str());
  }
}

void Iox2Source::pollOnce() {
  for (auto& entry : entries_) {
    if (!entry.subscriber) {
      continue;
    }
    while (true) {
      auto sample = entry.subscriber->receive();
      if (!sample.has_value()) {
        RCLCPP_WARN_ONCE(node_.get_logger(), "iox2 source: receive error on '%s', disabling",
                         entry.serviceName.c_str());
        entry.subscriber.reset();
        break;
      }
      if (!sample->has_value()) {
        break;  // queue drained
      }

      const auto& header = sample->value().user_header();
      const auto payload = sample->value().payload();
      // The MsgHeader locates the FlatBuffer within the shared-memory slot and carries a
      // publisher-side wall-clock timestamp, so we never have to parse the payload here.
      const auto* data = payload.data() + header.rootOffset;
      const auto err = entry.channel.log(reinterpret_cast<const std::byte*>(data),
                                         header.payloadSize, header.timestampNs);
      if (err != foxglove::FoxgloveError::Ok) {
        RCLCPP_WARN_ONCE(node_.get_logger(), "iox2 source: log failed on topic '%s': %s",
                         entry.topic.c_str(), foxglove::strerror(err));
      }
    }
  }
}

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
