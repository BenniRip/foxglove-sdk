#pragma once

#include <string>

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
/// Plugin instances are named in the `channel_source_plugins` parameter; `<name>.plugin` holds
/// each instance's pluginlib class name. A plugin creates `foxglove::RawChannel`s on the context
/// passed to `initialize()` and logs messages to them; the bridge's server delivers them to
/// clients like any ROS topic. Use `RawChannel::hasSinks()` to find out whether any client is
/// currently subscribed to a channel. The bridge ignores subscriptions to channels that are not
/// its own ROS channels, so plugins need not register theirs.
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
  ///
  /// - Declare parameters on `node` under `<name>.`, so several instances can coexist, and log
  ///   with `node.get_logger().get_child(name)`.
  /// - Create channels on `context`. Pass `node.now()` as the log time of each message, so
  ///   plugin channels line up with the bridge's ROS channels when `use_sim_time` is set.
  /// - `node.shared_from_this()` is not available yet. Timers and subscriptions created on `node`
  ///   run on the bridge's executor; plugins that need their own threads own them.
  ///
  /// Throwing an exception disables the plugin; the bridge keeps running without it.
  virtual void initialize(rclcpp::Node& node, const std::string& name,
                          const foxglove::Context& context) = 0;

  /// Called once when the bridge shuts down, before its server stops. Stop timers and threads
  /// that log to channels; the plugin is destroyed afterwards. If the bridge fails to start after
  /// `initialize()` returned, the plugin is destroyed without this call, so the destructor must
  /// release the same resources.
  virtual void shutdown() = 0;

protected:
  ChannelSourcePlugin() = default;
};

}  // namespace foxglove_bridge
