#pragma once

#include <string>

#include <rclcpp/node.hpp>

#include <foxglove/context.hpp>

#ifndef FOXGLOVE_BRIDGE_PLUGIN_API_VERSION
#error "Link channel source plugins against the foxglove_bridge::plugin_api CMake target"
#endif

namespace foxglove_bridge {

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

  /// Creates channels on `context`. Called from the bridge node's constructor, before its server
  /// starts. Parameters belong under `<name>.`, and messages should be logged with `node.now()`.
  ///
  /// Throwing disables the plugin. Close any channels created so far before throwing, since the
  /// context keeps them advertised otherwise.
  virtual void initialize(rclcpp::Node& node, const std::string& name,
                          const foxglove::Context& context) = 0;

  /// Stops all logging, threads and callbacks of the plugin. Called before the server stops, also
  /// when bridge startup fails after the plugin was initialized.
  virtual void shutdown() = 0;

  /// The foxglove_bridge version the plugin was built against.
  const char* bridgeVersion() const noexcept {
    return _bridgeVersion;
  }

protected:
  /// `bridgeVersion` is only overridden by tests.
  explicit ChannelSourcePlugin(const char* bridgeVersion = FOXGLOVE_BRIDGE_PLUGIN_API_VERSION)
      : _bridgeVersion(bridgeVersion) {}

private:
  // Must stay the only data member, so that the bridge can read it from plugins built against
  // other versions.
  const char* _bridgeVersion;
};

}  // namespace foxglove_bridge
