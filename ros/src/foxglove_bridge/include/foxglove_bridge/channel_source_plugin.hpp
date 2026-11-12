#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <rclcpp/node.hpp>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/schema.hpp>

#ifndef FOXGLOVE_BRIDGE_PLUGIN_ABI_NAMESPACE
#error "Link channel source plugins against the foxglove_bridge::plugin_api CMake target"
#endif

namespace foxglove_bridge {

// Plugins share the bridge's SDK build, which has no stable ABI. The namespace encodes the
// foxglove_bridge version, so a plugin built against another version registers its pluginlib
// factory for a different base class, and the bridge fails to create it instead of constructing it.
inline namespace FOXGLOVE_BRIDGE_PLUGIN_ABI_NAMESPACE {

/// Creates the channels of a channel source plugin instance on the bridge's context.
///
/// A channel is refused, with `FoxgloveError::ValueError`, if it is identical to a channel of a ROS
/// topic or of another plugin instance, since the SDK would otherwise return that channel. The
/// bridge logs the reason. Thread-safe, and valid until the plugin's `shutdown()` returns.
class ChannelSourcePluginChannels {
public:
  ChannelSourcePluginChannels(const ChannelSourcePluginChannels&) = delete;
  ChannelSourcePluginChannels& operator=(const ChannelSourcePluginChannels&) = delete;
  ChannelSourcePluginChannels(ChannelSourcePluginChannels&&) = delete;
  ChannelSourcePluginChannels& operator=(ChannelSourcePluginChannels&&) = delete;

  /// Creates a channel like `foxglove::RawChannel::create`.
  foxglove::FoxgloveResult<foxglove::RawChannel> createRaw(
    std::string_view topic, std::string_view messageEncoding,
    std::optional<foxglove::Schema> schema = std::nullopt,
    std::optional<std::map<std::string, std::string>> metadata = std::nullopt) {
    return create<foxglove::RawChannel>([&](const foxglove::Context& context) {
      return foxglove::RawChannel::create(topic, messageEncoding, std::move(schema), context,
                                          std::move(metadata));
    });
  }

  /// Creates a typed channel of the SDK, such as `foxglove::messages::CompressedImageChannel`.
  template <typename Channel>
  foxglove::FoxgloveResult<Channel> createTyped(std::string_view topic) {
    return create<Channel>([&](const foxglove::Context& context) {
      return Channel::create(topic, context);
    });
  }

protected:
  ChannelSourcePluginChannels() = default;
  ~ChannelSourcePluginChannels() = default;

  /// Calls `createChannel` with the bridge's context and returns whether the created channel may
  /// be used by the plugin instance.
  virtual bool createAndClaim(
    const std::function<std::optional<uint64_t>(const foxglove::Context&)>& createChannel) = 0;

private:
  template <typename Channel, typename CreateFn>
  foxglove::FoxgloveResult<Channel> create(CreateFn&& createFn) {
    std::optional<foxglove::FoxgloveResult<Channel>> result;
    const bool claimed = createAndClaim([&](const foxglove::Context& context) {
      result.emplace(createFn(context));
      return result->has_value() ? std::optional<uint64_t>(result->value().id()) : std::nullopt;
    });
    if (result->has_value() && !claimed) {
      // Dropping the handle does not close the channel, which belongs to its other owner.
      return tl::unexpected(foxglove::FoxgloveError::ValueError);
    }
    return std::move(*result);
  }
};

/// Plugin that serves non-ROS channels over the bridge's server. See the README for usage.
///
/// Experimental: the interface may change in any release, and plugins must be built against the
/// same foxglove_bridge version that loads them.
class ChannelSourcePlugin {
public:
  virtual ~ChannelSourcePlugin() = default;

  ChannelSourcePlugin(const ChannelSourcePlugin&) = delete;
  ChannelSourcePlugin& operator=(const ChannelSourcePlugin&) = delete;
  ChannelSourcePlugin(ChannelSourcePlugin&&) = delete;
  ChannelSourcePlugin& operator=(ChannelSourcePlugin&&) = delete;

  /// Creates channels with `channels`. Called from the bridge node's constructor, before its
  /// server starts. Parameters belong under `<name>.`, and messages should be logged with
  /// `node.now()`.
  ///
  /// Throwing disables the plugin. Close any channels created so far before throwing, since the
  /// context keeps them advertised otherwise.
  virtual void initialize(rclcpp::Node& node, const std::string& name,
                          ChannelSourcePluginChannels& channels) = 0;

  /// Stops all logging, threads and callbacks of the plugin. Called before the server stops, also
  /// when bridge startup fails after the plugin was initialized.
  virtual void shutdown() = 0;

protected:
  ChannelSourcePlugin() = default;
};

}  // namespace FOXGLOVE_BRIDGE_PLUGIN_ABI_NAMESPACE
}  // namespace foxglove_bridge
