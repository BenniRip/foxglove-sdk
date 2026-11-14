#pragma once

#include <string>

#include <rclcpp/node.hpp>

#include <foxglove/context.hpp>

#ifndef FOXGLOVE_BRIDGE_PLUGIN_ABI_NAMESPACE
#error "Link channel source plugins against the foxglove_bridge::plugin_api CMake target"
#endif

namespace foxglove_bridge {

// Plugins share the bridge's SDK build, which has no stable ABI. The namespace encodes the
// foxglove_bridge version, so a plugin built against another version registers its pluginlib
// factory for a different base class, and the bridge fails to create it instead of constructing it.
inline namespace FOXGLOVE_BRIDGE_PLUGIN_ABI_NAMESPACE {

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

  /// Creates the plugin's channels on `context`, the bridge's context. Called from the bridge
  /// node's constructor, before its server starts. Parameters belong under `<name>.`, and messages
  /// should be logged with `node.now()`.
  ///
  /// Plugin channels must differ from the channels of ROS topics in topic, encoding, schema or
  /// metadata, since the context would otherwise share them with the bridge.
  ///
  /// Throwing disables the plugin. Close any channels created so far before throwing, since the
  /// context keeps them advertised otherwise.
  virtual void initialize(rclcpp::Node& node, const std::string& name,
                          const foxglove::Context& context) = 0;

  /// Stops all logging, threads and callbacks of the plugin. Called before the server stops, also
  /// when bridge startup fails after the plugin was initialized.
  virtual void shutdown() = 0;

protected:
  ChannelSourcePlugin() = default;
};

}  // namespace FOXGLOVE_BRIDGE_PLUGIN_ABI_NAMESPACE
}  // namespace foxglove_bridge
