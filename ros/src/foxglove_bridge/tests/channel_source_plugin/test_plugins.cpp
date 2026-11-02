#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove_bridge/channel_source_plugin.hpp>

namespace foxglove_bridge_test_plugins {

constexpr char TEST_PLUGIN_TOPIC[] = "/test_plugin/data";
constexpr char TEST_PLUGIN_MESSAGE[] = R"({"value":42})";
constexpr char TEST_PLUGIN_SCHEMA[] = R"({"type":"object"})";

/// Serves one JSON channel and logs a fixed message while a client is subscribed.
class TestChannelSourcePlugin : public foxglove_bridge::ChannelSourcePlugin {
public:
  void initialize(rclcpp::Node& node, const foxglove::Context& context) override {
    foxglove::Schema schema;
    schema.name = "test_plugin.Data";
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte*>(TEST_PLUGIN_SCHEMA);
    schema.data_len = sizeof(TEST_PLUGIN_SCHEMA) - 1;
    auto channel = foxglove::RawChannel::create(TEST_PLUGIN_TOPIC, "json", schema, context);
    if (!channel.has_value()) {
      throw std::runtime_error(std::string("failed to create channel: ") +
                               foxglove::strerror(channel.error()));
    }
    _channel.emplace(std::move(channel.value()));
    _channelId = _channel->id();

    _timer = node.create_wall_timer(std::chrono::milliseconds(20), [this]() {
      if (_channel->hasSinks()) {
        _channel->log(reinterpret_cast<const std::byte*>(TEST_PLUGIN_MESSAGE),
                      sizeof(TEST_PLUGIN_MESSAGE) - 1);
      }
    });
  }

  void shutdown() override {
    if (_timer) {
      _timer->cancel();
    }
  }

  bool ownsChannel(uint64_t channelId) const override {
    return _channelId.load() == channelId;
  }

private:
  std::optional<foxglove::RawChannel> _channel;
  std::atomic<uint64_t> _channelId{0};
  rclcpp::TimerBase::SharedPtr _timer;
};

/// Claims to be built against an incompatible interface version; must not be initialized.
class WrongApiVersionPlugin : public foxglove_bridge::ChannelSourcePlugin {
public:
  int apiVersion() const override {
    return foxglove_bridge::CHANNEL_SOURCE_PLUGIN_API_VERSION + 1;
  }

  void initialize(rclcpp::Node& /*node*/, const foxglove::Context& context) override {
    auto channel =
      foxglove::RawChannel::create("/test_plugin/wrong_version", "json", std::nullopt, context);
    if (channel.has_value()) {
      _channel.emplace(std::move(channel.value()));
    }
  }

  void shutdown() override {}

  bool ownsChannel(uint64_t /*channelId*/) const override {
    return false;
  }

private:
  std::optional<foxglove::RawChannel> _channel;
};

/// Fails to initialize; the bridge must keep running without it.
class ThrowingPlugin : public foxglove_bridge::ChannelSourcePlugin {
public:
  void initialize(rclcpp::Node& /*node*/, const foxglove::Context& /*context*/) override {
    throw std::runtime_error("initialization failure requested by test");
  }

  void shutdown() override {}

  bool ownsChannel(uint64_t /*channelId*/) const override {
    return false;
  }
};

}  // namespace foxglove_bridge_test_plugins

PLUGINLIB_EXPORT_CLASS(foxglove_bridge_test_plugins::TestChannelSourcePlugin,
                       foxglove_bridge::ChannelSourcePlugin)
PLUGINLIB_EXPORT_CLASS(foxglove_bridge_test_plugins::WrongApiVersionPlugin,
                       foxglove_bridge::ChannelSourcePlugin)
PLUGINLIB_EXPORT_CLASS(foxglove_bridge_test_plugins::ThrowingPlugin,
                       foxglove_bridge::ChannelSourcePlugin)
