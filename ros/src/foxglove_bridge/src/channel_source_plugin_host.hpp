#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <foxglove/context.hpp>
#include <foxglove_bridge/channel_source_plugin.hpp>

namespace foxglove_bridge {

/// Loads the channel source plugin instances of a bridge node and tracks which instance owns each
/// plugin channel.
class ChannelSourcePluginHost {
public:
  /// Plugin channels are created while holding `rosChannelsMutex`, which must also guard the
  /// creation of ROS channels. `isRosChannel` is called with it held.
  ChannelSourcePluginHost(rclcpp::Node& node, const foxglove::Context& context,
                          std::mutex& rosChannelsMutex, std::function<bool(uint64_t)> isRosChannel);
  ~ChannelSourcePluginHost();

  ChannelSourcePluginHost(const ChannelSourcePluginHost&) = delete;
  ChannelSourcePluginHost& operator=(const ChannelSourcePluginHost&) = delete;
  ChannelSourcePluginHost(ChannelSourcePluginHost&&) = delete;
  ChannelSourcePluginHost& operator=(ChannelSourcePluginHost&&) = delete;

  /// Loads the named instances. Failing instances are logged and skipped, unless they are
  /// required, in which case this throws.
  void load(const std::vector<std::string>& instanceNames);

  /// Shuts the loaded plugins down in reverse load order.
  void shutdown();

  /// Destroys the loaded plugins in reverse load order.
  void destroy();

  /// Returns the instance that owns a plugin channel, or nullopt for other channels.
  std::optional<std::string> channelOwner(uint64_t channelId);

  /// Whether a plugin channel is being created on this thread. Sinks are notified of a new channel
  /// synchronously, before the channel can be claimed by its plugin instance.
  static bool creatingChannelOnThisThread();

private:
  class InstanceChannels;
  struct Instance {
    // Declared before the plugin, which may use it until it is destroyed.
    std::unique_ptr<InstanceChannels> channels;
    pluginlib::UniquePtr<ChannelSourcePlugin> plugin;
  };

  void loadInstance(const std::string& name);

  /// Creates a channel for the instance `owner` and returns whether the instance may use it, which
  /// it may not if the channel already belongs to a ROS topic or another instance.
  bool createAndClaim(
    const std::string& owner,
    const std::function<std::optional<uint64_t>(const foxglove::Context&)>& createChannel);

  rclcpp::Node& _node;
  const foxglove::Context& _context;
  std::mutex& _rosChannelsMutex;
  std::function<bool(uint64_t)> _isRosChannel;

  // The loader must outlive the plugins it created, so it is declared first.
  std::unique_ptr<pluginlib::ClassLoader<ChannelSourcePlugin>> _loader;
  // In load order.
  std::vector<Instance> _instances;
  // Acquired after _rosChannelsMutex, and never held while creating a channel, since sinks may call
  // channelOwner() synchronously then.
  std::mutex _channelOwnersMutex;
  std::unordered_map<uint64_t, std::string> _channelOwners;
};

}  // namespace foxglove_bridge
