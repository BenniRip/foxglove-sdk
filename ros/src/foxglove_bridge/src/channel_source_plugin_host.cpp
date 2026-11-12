#include "channel_source_plugin_host.hpp"

#include <cinttypes>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <pluginlib/exceptions.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcpputils/scope_exit.hpp>

#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/version.hpp>

namespace foxglove_bridge {
namespace {

thread_local bool t_creatingChannel = false;

std::string rebuildHint(const std::exception& ex) {
  return std::string(ex.what()) + ". If the plugin was built against a foxglove_bridge version " +
         "other than " + FOXGLOVE_BRIDGE_VERSION + ", rebuild it.";
}

}  // namespace

class ChannelSourcePluginHost::InstanceChannels final : public ChannelSourcePluginChannels {
public:
  InstanceChannels(ChannelSourcePluginHost& host, std::string instanceName)
      : _host(host)
      , _instanceName(std::move(instanceName)) {}

  ~InstanceChannels() = default;

protected:
  bool createAndClaim(const std::function<std::optional<uint64_t>(const foxglove::Context&)>&
                        createChannel) override {
    return _host.createAndClaim(_instanceName, createChannel);
  }

private:
  ChannelSourcePluginHost& _host;
  std::string _instanceName;
};

ChannelSourcePluginHost::ChannelSourcePluginHost(rclcpp::Node& node,
                                                 const foxglove::Context& context,
                                                 std::mutex& rosChannelsMutex,
                                                 std::function<bool(uint64_t)> isRosChannel)
    : _node(node)
    , _context(context)
    , _rosChannelsMutex(rosChannelsMutex)
    , _isRosChannel(std::move(isRosChannel)) {}

ChannelSourcePluginHost::~ChannelSourcePluginHost() {
  destroy();
}

void ChannelSourcePluginHost::load(const std::vector<std::string>& instanceNames) {
  std::unordered_set<std::string> seenNames;
  for (const auto& name : instanceNames) {
    if (!seenNames.insert(name).second) {
      RCLCPP_ERROR(_node.get_logger(), "Ignoring duplicate channel source plugin instance '%s'",
                   name.c_str());
      continue;
    }
    // Outside the try block, so that an invalid value fails startup rather than making a required
    // instance optional.
    const std::string requiredParam = name + "." + PARAM_CHANNEL_SOURCE_PLUGIN_REQUIRED;
    auto requiredDescription = rcl_interfaces::msg::ParameterDescriptor{};
    requiredDescription.name = requiredParam;
    requiredDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    requiredDescription.description =
      "Fail bridge startup if the channel source plugin instance '" + name + "' fails to load.";
    requiredDescription.read_only = true;
    const bool required = _node.declare_parameter(requiredParam, false, requiredDescription);

    std::string error;
    try {
      loadInstance(name);
      continue;
    } catch (const std::exception& ex) {
      error = ex.what();
    } catch (...) {
      error = "unknown exception";
    }
    if (required) {
      throw std::runtime_error("Failed to load required channel source plugin instance '" + name +
                               "': " + error);
    }
    RCLCPP_ERROR(_node.get_logger(), "Failed to load channel source plugin instance '%s': %s",
                 name.c_str(), error.c_str());
  }
}

void ChannelSourcePluginHost::loadInstance(const std::string& name) {
  const std::string classParam = name + "." + PARAM_CHANNEL_SOURCE_PLUGIN_CLASS;
  auto classDescription = rcl_interfaces::msg::ParameterDescriptor{};
  classDescription.name = classParam;
  classDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  classDescription.description =
    "pluginlib class name of the channel source plugin instance '" + name + "'.";
  classDescription.read_only = true;
  const auto className = _node.declare_parameter(classParam, std::string(), classDescription);
  if (className.empty()) {
    throw std::runtime_error("parameter '" + classParam + "' is not set");
  }

  if (!_loader) {
    _loader = std::make_unique<pluginlib::ClassLoader<ChannelSourcePlugin>>(
      "foxglove_bridge", "foxglove_bridge::ChannelSourcePlugin");
  }
  if (!_loader->isClassAvailable(className)) {
    throw std::runtime_error("no plugin description declares the class '" + className + "'");
  }
  Instance instance;
  try {
    instance.plugin = _loader->createUniqueInstance(className);
  } catch (const pluginlib::CreateClassException& ex) {
    // A plugin built against another version registers for another base class, see
    // channel_source_plugin.hpp.
    throw std::runtime_error(rebuildHint(ex));
  } catch (const pluginlib::LibraryLoadException& ex) {
    throw std::runtime_error(rebuildHint(ex));
  }
  instance.channels = std::make_unique<InstanceChannels>(*this, name);
  instance.plugin->initialize(_node, name, *instance.channels);
  _instances.push_back(std::move(instance));
  RCLCPP_INFO(_node.get_logger(), "Loaded channel source plugin instance '%s' (%s)", name.c_str(),
              className.c_str());
}

void ChannelSourcePluginHost::shutdown() {
  for (auto it = _instances.rbegin(); it != _instances.rend(); ++it) {
    try {
      it->plugin->shutdown();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(_node.get_logger(), "Channel source plugin failed to shut down: %s", ex.what());
    } catch (...) {
      RCLCPP_ERROR(_node.get_logger(), "Channel source plugin failed to shut down");
    }
  }
}

void ChannelSourcePluginHost::destroy() {
  while (!_instances.empty()) {
    _instances.pop_back();
  }
}

std::optional<std::string> ChannelSourcePluginHost::channelOwner(uint64_t channelId) {
  std::lock_guard<std::mutex> lock(_channelOwnersMutex);
  const auto it = _channelOwners.find(channelId);
  if (it == _channelOwners.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool ChannelSourcePluginHost::creatingChannelOnThisThread() {
  return t_creatingChannel;
}

bool ChannelSourcePluginHost::createAndClaim(
  const std::string& owner,
  const std::function<std::optional<uint64_t>(const foxglove::Context&)>& createChannel) {
  // Keeps ROS channels from being created between creating and claiming the channel.
  std::lock_guard<std::mutex> lock(_rosChannelsMutex);
  std::optional<uint64_t> channelId;
  {
    t_creatingChannel = true;
    auto resetFlag = rcpputils::make_scope_exit([]() {
      t_creatingChannel = false;
    });
    channelId = createChannel(_context);
  }
  if (!channelId.has_value()) {
    return true;
  }
  if (_isRosChannel(*channelId)) {
    RCLCPP_ERROR(_node.get_logger(),
                 "Channel source plugin instance '%s' cannot use channel %" PRIu64
                 ": it is identical to the channel of a ROS topic",
                 owner.c_str(), *channelId);
    return false;
  }
  std::lock_guard<std::mutex> ownersLock(_channelOwnersMutex);
  const auto [it, inserted] = _channelOwners.try_emplace(*channelId, owner);
  if (!inserted && it->second != owner) {
    RCLCPP_ERROR(_node.get_logger(),
                 "Channel source plugin instance '%s' cannot use channel %" PRIu64
                 ": it is identical to a channel of instance '%s'",
                 owner.c_str(), *channelId, it->second.c_str());
    return false;
  }
  return true;
}

}  // namespace foxglove_bridge
