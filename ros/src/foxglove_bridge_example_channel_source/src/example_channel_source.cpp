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

namespace foxglove_bridge_example_channel_source {

constexpr char COUNTER_SCHEMA[] = R"({"type":"object","properties":{"count":{"type":"integer"}}})";

/// Serves a JSON channel with an incrementing counter, published only while a client is
/// subscribed. Parameters, under the instance name:
/// - `<name>.topic`: topic to advertise (default "/<name>/counter")
/// - `<name>.period_ms`: publish period in milliseconds (default 1000)
class ExampleChannelSource : public foxglove_bridge::ChannelSourcePlugin {
public:
  void initialize(rclcpp::Node& node, const std::string& name,
                  const foxglove::Context& context) override {
    const auto topic = node.declare_parameter(name + ".topic", "/" + name + "/counter");
    const auto periodMs = node.declare_parameter(name + ".period_ms", int64_t{1000});

    foxglove::Schema schema;
    schema.name = "example.Counter";
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte*>(COUNTER_SCHEMA);
    schema.data_len = sizeof(COUNTER_SCHEMA) - 1;
    auto channel = foxglove::RawChannel::create(topic, "json", schema, context);
    if (!channel.has_value()) {
      throw std::runtime_error(std::string("failed to create channel: ") +
                               foxglove::strerror(channel.error()));
    }
    _channel.emplace(std::move(channel.value()));
    _clock = node.get_clock();

    _timer = node.create_wall_timer(std::chrono::milliseconds(periodMs), [this]() {
      publish();
    });
    RCLCPP_INFO(node.get_logger().get_child(name), "Serving a counter on %s", topic.c_str());
  }

  void shutdown() override {
    if (_timer) {
      _timer->cancel();
    }
  }

private:
  void publish() {
    // Only produce data while a client is subscribed.
    if (!_channel->hasSinks()) {
      return;
    }
    const auto message = "{\"count\":" + std::to_string(_count++) + "}";
    // The node clock follows use_sim_time, like the timestamps of the bridge's ROS channels.
    _channel->log(reinterpret_cast<const std::byte*>(message.data()), message.size(),
                  static_cast<uint64_t>(_clock->now().nanoseconds()));
  }

  std::optional<foxglove::RawChannel> _channel;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::TimerBase::SharedPtr _timer;
  uint64_t _count = 0;
};

}  // namespace foxglove_bridge_example_channel_source

PLUGINLIB_EXPORT_CLASS(foxglove_bridge_example_channel_source::ExampleChannelSource,
                       foxglove_bridge::ChannelSourcePlugin)
