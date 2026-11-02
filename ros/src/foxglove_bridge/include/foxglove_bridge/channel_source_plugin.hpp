#pragma once

#include <cstdint>

#include <rclcpp/node.hpp>

#include <foxglove/context.hpp>

namespace foxglove_bridge {

/// Version of the ChannelSourcePlugin interface. Incremented on any change to the class layout
/// or to the semantics the bridge relies on. The bridge refuses to load plugins built against a
/// different version, since they would not be binary compatible.
constexpr int CHANNEL_SOURCE_PLUGIN_API_VERSION = 1;

/// Base class for plugins that serve additional, non-ROS data over the bridge's WebSocket server
/// (and remote access gateway, if enabled).
///
/// Plugins are loaded with pluginlib from the names in the `channel_source_plugins` parameter.
/// A plugin creates `foxglove::RawChannel`s on the context passed to `initialize()` and logs
/// messages to them; the bridge's server delivers them to clients like any ROS topic. Use
/// `RawChannel::hasSinks()` to find out whether any client is currently subscribed to a channel.
///
/// Plugins must be built against the same foxglove_bridge version they are loaded into: they
/// share the bridge's copy of the Foxglove SDK, which they link through the exported
/// `foxglove_bridge::foxglove_bridge_plugin_api` CMake target.
class ChannelSourcePlugin {
public:
  virtual ~ChannelSourcePlugin() = default;

  ChannelSourcePlugin(const ChannelSourcePlugin&) = delete;
  ChannelSourcePlugin& operator=(const ChannelSourcePlugin&) = delete;
  ChannelSourcePlugin(ChannelSourcePlugin&&) = delete;
  ChannelSourcePlugin& operator=(ChannelSourcePlugin&&) = delete;

  /// Returns the interface version this plugin was compiled against. Must stay the first virtual
  /// function after the destructor, so the bridge can call it on plugins of any version.
  virtual int apiVersion() const {
    return CHANNEL_SOURCE_PLUGIN_API_VERSION;
  }

  /// Called once while the bridge node is being constructed, before the WebSocket server starts.
  /// Declare parameters under a plugin-specific prefix on `node`, and create channels on
  /// `context`. `node.shared_from_this()` is not available yet. Timers and subscriptions created
  /// on `node` run on the bridge's executor; plugins that need their own threads own them.
  /// Throwing an exception disables the plugin; the bridge keeps running without it.
  virtual void initialize(rclcpp::Node& node, const foxglove::Context& context) = 0;

  /// Called once when the bridge shuts down, before its server stops. Stop timers and threads
  /// that log to channels; the plugin is destroyed afterwards. If the bridge fails to start after
  /// `initialize()` returned, the plugin is destroyed without this call, so the destructor must
  /// release the same resources.
  virtual void shutdown() = 0;

  /// Returns true if the channel with this ID was created by this plugin. The bridge uses this to
  /// skip its ROS subscription handling for the channel. Called from server threads, concurrently
  /// with the plugin's own work, so implementations must be thread-safe.
  virtual bool ownsChannel(uint64_t channelId) const = 0;

protected:
  ChannelSourcePlugin() = default;
};

}  // namespace foxglove_bridge
