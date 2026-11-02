#include <chrono>
#include <cstddef>
#include <cstdint>
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

constexpr char TEST_PLUGIN_MESSAGE[] = R"({"value":42})";
constexpr char TEST_PLUGIN_SCHEMA[] = R"({"type":"object"})";

/// Serves one JSON channel, "/<instance name>/data", and logs a fixed message while a client is
/// subscribed.
class TestChannelSourcePlugin : public foxglove_bridge::ChannelSourcePlugin {
public:
  void initialize(rclcpp::Node& node, const std::string& name,
                  const foxglove::Context& context) override {
    foxglove::Schema schema;
    schema.name = "test_plugin.Data";
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte*>(TEST_PLUGIN_SCHEMA);
    schema.data_len = sizeof(TEST_PLUGIN_SCHEMA) - 1;
    auto channel = foxglove::RawChannel::create("/" + name + "/data", "json", schema, context);
    if (!channel.has_value()) {
      throw std::runtime_error(std::string("failed to create channel: ") +
                               foxglove::strerror(channel.error()));
    }
    _channel.emplace(std::move(channel.value()));
    _name = name;
    _logger = node.get_logger().get_child(name);

    _timer = node.create_wall_timer(std::chrono::milliseconds(20), [this, &node]() {
      if (_channel->hasSinks()) {
        _channel->log(reinterpret_cast<const std::byte*>(TEST_PLUGIN_MESSAGE),
                      sizeof(TEST_PLUGIN_MESSAGE) - 1,
                      static_cast<uint64_t>(node.now().nanoseconds()));
      }
    });
  }

  void shutdown() override {
    if (_timer) {
      _timer->cancel();
    }
    RCLCPP_INFO(_logger, "Test plugin instance '%s' shut down", _name.c_str());
  }

private:
  std::optional<foxglove::RawChannel> _channel;
  rclcpp::TimerBase::SharedPtr _timer;
  std::string _name;
  rclcpp::Logger _logger = rclcpp::get_logger("test_plugin");
};

/// Claims to be built against an incompatible interface version; must not be initialized.
class WrongApiVersionPlugin : public foxglove_bridge::ChannelSourcePlugin {
public:
  int apiVersion() const override {
    return foxglove_bridge::CHANNEL_SOURCE_PLUGIN_API_VERSION + 1;
  }

  void initialize(rclcpp::Node& /*node*/, const std::string& /*name*/,
                  const foxglove::Context& context) override {
    auto channel =
      foxglove::RawChannel::create("/wrong_version/data", "json", std::nullopt, context);
    if (channel.has_value()) {
      _channel.emplace(std::move(channel.value()));
    }
  }

  void shutdown() override {}

private:
  std::optional<foxglove::RawChannel> _channel;
};

/// Fails to initialize; the bridge must keep running without it.
class ThrowingPlugin : public foxglove_bridge::ChannelSourcePlugin {
public:
  void initialize(rclcpp::Node& /*node*/, const std::string& /*name*/,
                  const foxglove::Context& /*context*/) override {
    throw std::runtime_error("initialization failure requested by test");
  }

  void shutdown() override {}
};

}  // namespace foxglove_bridge_test_plugins

PLUGINLIB_EXPORT_CLASS(foxglove_bridge_test_plugins::TestChannelSourcePlugin,
                       foxglove_bridge::ChannelSourcePlugin)
PLUGINLIB_EXPORT_CLASS(foxglove_bridge_test_plugins::WrongApiVersionPlugin,
                       foxglove_bridge::ChannelSourcePlugin)
PLUGINLIB_EXPORT_CLASS(foxglove_bridge_test_plugins::ThrowingPlugin,
                       foxglove_bridge::ChannelSourcePlugin)
