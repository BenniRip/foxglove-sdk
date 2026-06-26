#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include "foxglove_bridge/iox2_source.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>

#include <iox2/type_name.hpp>

#include <foxglove/error.hpp>

// Decouple the iceoryx2 user-header type identity from the C++ type: resolve the type name at
// runtime from foxglove_bridge::iox2UserHeaderTypeName(), so it can be overridden via a
// parameter to match any publisher's header type name without recompiling. The macro wraps the
// expression in a function that iceoryx2 calls at service-build time (it copies the name into a
// fixed-capacity StaticString, so the returned pointer only has to be valid for that call), and
// the value is read per service-build call rather than cached. This specialization must be
// visible before the service builder is instantiated for Iox2MessageHeader.
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

/// Concrete iox2 subscription. Templated on the iceoryx2 user-header type and on whether that
/// header is used to locate the FlatBuffer:
///   - UseHeader == true  (UserHeaderT == Iox2MessageHeader): the header gives the FlatBuffer's
///     offset/size within the slot and the publisher's wall-clock timestamp.
///   - UseHeader == false (UserHeaderT == void): the whole payload slice is the FlatBuffer and
///     the bridge's receive time is used as the log time. This is the path for once iceoryx2
///     ships first-class FlatBuffer support and the Iox2MessageHeader contract is retired.
template <typename UserHeaderT, bool UseHeader>
class TypedSubscription final : public Iox2Subscription {
  using Slice = iox2::bb::Slice<uint8_t>;
  using Service = iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, Slice, UserHeaderT>;
  using Sub = iox2::Subscriber<iox2::ServiceType::Ipc, Slice, UserHeaderT>;

public:
  // service_ is kept alive (and declared before subscriber_, so destroyed after it) because the
  // iceoryx2 PortFactory must outlive the ports it creates.
  TypedSubscription(std::string serviceName, std::string topic, Service service, Sub subscriber)
      : serviceName_(std::move(serviceName))
      , topic_(std::move(topic))
      , service_(std::move(service))
      , subscriber_(std::move(subscriber)) {}

  bool drain(foxglove::RawChannel& channel, const rclcpp::Logger& logger) override {
    while (true) {
      auto sample = subscriber_.receive();
      if (!sample.has_value()) {
        // Fatal receive error. Returning false makes the caller drop this subscription, so this
        // warning fires exactly once per service rather than on every poll.
        RCLCPP_WARN(logger, "iox2 source: receive error on '%s', disabling", serviceName_.c_str());
        return false;
      }
      if (!sample->has_value()) {
        break;  // queue drained
      }

      const auto payload = sample->value().payload();
      const auto* bytes = reinterpret_cast<const std::byte*>(payload.data());
      const uint64_t available = payload.number_of_bytes();

      const std::byte* data = bytes;
      size_t len = static_cast<size_t>(available);
      std::optional<uint64_t> logTime = std::nullopt;

      if constexpr (UseHeader) {
        // The Iox2MessageHeader locates the FlatBuffer within the shared-memory slot and carries
        // the publisher's wall-clock timestamp, so we never have to parse the payload here.
        // rootOffset/payloadSize are written by another process, so validate the range before
        // handing it to log(): a malformed or buggy publisher must not cause an out-of-bounds
        // read out of the shared-memory slot.
        const auto& header = sample->value().user_header();
        const uint64_t end = static_cast<uint64_t>(header.rootOffset) + header.payloadSize;
        if (end > available) {
          if (!warnedBadHeader_) {
            warnedBadHeader_ = true;
            RCLCPP_WARN(logger,
                        "iox2 source: '%s' header range (offset %u + size %u) exceeds payload "
                        "(%lu bytes); dropping message",
                        serviceName_.c_str(), header.rootOffset, header.payloadSize,
                        static_cast<unsigned long>(available));
          }
          continue;
        }
        data = bytes + header.rootOffset;
        len = header.payloadSize;
        logTime = header.timestampNs;
      }

      const auto err = channel.log(data, len, logTime);
      if (err != foxglove::FoxgloveError::Ok && !warnedLogError_) {
        warnedLogError_ = true;
        RCLCPP_WARN(logger, "iox2 source: log failed on topic '%s': %s", topic_.c_str(),
                    foxglove::strerror(err));
      }
    }
    return true;
  }

private:
  std::string serviceName_;
  std::string topic_;
  Service service_;
  Sub subscriber_;
  [[maybe_unused]] bool warnedBadHeader_ = false;  // unused in the headerless instantiation
  bool warnedLogError_ = false;
};

/// Open an iox2 publish-subscribe service and subscriber for one configured entry and wrap them
/// in a TypedSubscription. The `if constexpr` selects the user-header path at compile time so the
/// two distinct iceoryx2 service/subscriber types are both instantiated from one definition.
template <typename UserHeaderT, bool UseHeader>
std::unique_ptr<Iox2Subscription> openSubscription(iox2::Node<iox2::ServiceType::Ipc>& node,
                                                   const std::string& serviceName,
                                                   const std::string& topic) {
  auto name = iox2::ServiceName::create(serviceName.c_str());
  if (!name.has_value()) {
    throw std::runtime_error("iox2 source: invalid iox2 service name '" + serviceName + "'");
  }

  auto service = [&] {
    auto builder = node.service_builder(name.value()).publish_subscribe<iox2::bb::Slice<uint8_t>>();
    if constexpr (UseHeader) {
      return std::move(builder).user_header<UserHeaderT>().open_or_create();
    } else {
      return std::move(builder).open_or_create();
    }
  }();
  if (!service.has_value()) {
    throw std::runtime_error("iox2 source: failed to open iox2 service '" + serviceName + "'");
  }

  auto subscriber = service.value().subscriber_builder().create();
  if (!subscriber.has_value()) {
    throw std::runtime_error("iox2 source: failed to create subscriber for service '" +
                             serviceName + "'");
  }

  return std::make_unique<TypedSubscription<UserHeaderT, UseHeader>>(
    serviceName, topic, std::move(service.value()), std::move(subscriber.value()));
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

  if (useHeader_) {
    // Apply the configured iceoryx2 user-header type name before opening any service. The
    // service builder reads it at open time via the IOX2_DEFINE_TYPE_NAME specialization above,
    // and that value is evaluated per service-build call rather than cached, so this must run
    // before setupSubscribers(). (Headerless mode uses the void header and ignores this name.)
    setIox2UserHeaderTypeName(userHeaderTypeName_);
  }

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
  node_.declare_parameter("iox2.use_message_header", true);
  node_.declare_parameter("iox2.user_header_type_name", std::string(""));
  node_.declare_parameter("iox2.service_names", std::vector<std::string>{});
  node_.declare_parameter("iox2.topics", std::vector<std::string>{});
  node_.declare_parameter("iox2.schema_names", std::vector<std::string>{});
  node_.declare_parameter("iox2.bfbs_files", std::vector<std::string>{});

  schemaDir_ = node_.get_parameter("iox2.schema_dir").as_string();
  useHeader_ = node_.get_parameter("iox2.use_message_header").as_bool();
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

    auto subscription =
      useHeader_
        ? openSubscription<Iox2MessageHeader, true>(*iox2Node_, cfg.serviceName, cfg.topic)
        : openSubscription<void, false>(*iox2Node_, cfg.serviceName, cfg.topic);

    entries_.push_back(Entry{std::move(channelResult.value()), std::move(subscription)});

    RCLCPP_INFO(node_.get_logger(), "iox2 source: '%s' -> Foxglove '%s' (%s)%s",
                cfg.serviceName.c_str(), cfg.topic.c_str(), cfg.schemaName.c_str(),
                useHeader_ ? "" : " [headerless]");
  }
}

void Iox2Source::pollOnce() {
  for (auto& entry : entries_) {
    if (!entry.subscription) {
      continue;
    }
    if (!entry.subscription->drain(entry.channel, node_.get_logger())) {
      entry.subscription.reset();  // fatal receive error: stop polling this service
    }
  }
}

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
