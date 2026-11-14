#include "channel_source_plugin_host.hpp"

#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <pluginlib/exceptions.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/version.hpp>

namespace foxglove_bridge {
namespace {

std::string rebuildHint(const std::exception& ex) {
  return std::string(ex.what()) + ". If the plugin was built against a foxglove_bridge version " +
         "other than " + FOXGLOVE_BRIDGE_VERSION + ", rebuild it.";
}

}  // namespace

ChannelSourcePluginHost::ChannelSourcePluginHost(rclcpp::Node& node,
                                                 const foxglove::Context& context)
    : _node(node)
    , _context(context) {}

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
    try {
      loadInstance(name);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(_node.get_logger(), "Failed to load channel source plugin instance '%s': %s",
                   name.c_str(), ex.what());
    } catch (...) {
      RCLCPP_ERROR(_node.get_logger(),
                   "Failed to load channel source plugin instance '%s': unknown exception",
                   name.c_str());
    }
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
  pluginlib::UniquePtr<ChannelSourcePlugin> plugin;
  try {
    plugin = _loader->createUniqueInstance(className);
  } catch (const pluginlib::CreateClassException& ex) {
    // A plugin built against another version registers for another base class, see
    // channel_source_plugin.hpp.
    throw std::runtime_error(rebuildHint(ex));
  } catch (const pluginlib::LibraryLoadException& ex) {
    throw std::runtime_error(rebuildHint(ex));
  }
  plugin->initialize(_node, name, _context);
  _plugins.push_back(std::move(plugin));
  RCLCPP_INFO(_node.get_logger(), "Loaded channel source plugin instance '%s' (%s)", name.c_str(),
              className.c_str());
}

void ChannelSourcePluginHost::shutdown() {
  for (auto it = _plugins.rbegin(); it != _plugins.rend(); ++it) {
    try {
      (*it)->shutdown();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(_node.get_logger(), "Channel source plugin failed to shut down: %s", ex.what());
    } catch (...) {
      RCLCPP_ERROR(_node.get_logger(), "Channel source plugin failed to shut down");
    }
  }
}

void ChannelSourcePluginHost::destroy() {
  while (!_plugins.empty()) {
    _plugins.pop_back();
  }
}

}  // namespace foxglove_bridge
