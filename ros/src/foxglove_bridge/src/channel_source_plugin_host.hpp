#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <foxglove/context.hpp>
#include <foxglove_bridge/channel_source_plugin.hpp>

namespace foxglove_bridge {

/// Loads the channel source plugin instances of a bridge node.
class ChannelSourcePluginHost {
public:
  ChannelSourcePluginHost(rclcpp::Node& node, const foxglove::Context& context);
  ~ChannelSourcePluginHost();

  ChannelSourcePluginHost(const ChannelSourcePluginHost&) = delete;
  ChannelSourcePluginHost& operator=(const ChannelSourcePluginHost&) = delete;
  ChannelSourcePluginHost(ChannelSourcePluginHost&&) = delete;
  ChannelSourcePluginHost& operator=(ChannelSourcePluginHost&&) = delete;

  /// Loads the named instances. Failing instances are logged and skipped.
  void load(const std::vector<std::string>& instanceNames);

  /// Shuts the loaded plugins down in reverse load order.
  void shutdown();

  /// Destroys the loaded plugins in reverse load order.
  void destroy();

private:
  void loadInstance(const std::string& name);

  rclcpp::Node& _node;
  const foxglove::Context& _context;
  // The loader must outlive the plugins it created, so it is declared first.
  std::unique_ptr<pluginlib::ClassLoader<ChannelSourcePlugin>> _loader;
  // In load order.
  std::vector<pluginlib::UniquePtr<ChannelSourcePlugin>> _plugins;
};

}  // namespace foxglove_bridge
