#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <foxglove/context.hpp>
#include <foxglove_bridge/channel_source_plugin.hpp>

constexpr char PLUGIN_CLASS[] = "foxglove_bridge_example_channel_source::ExampleChannelSource";

// Loads the plugin from the install space the way foxglove_bridge does, which only works if the
// plugin description is exported and the library links the bridge's plugin API.
TEST(ExampleChannelSourceTest, loadsThroughPluginlibAndInitializes) {
  pluginlib::ClassLoader<foxglove_bridge::ChannelSourcePlugin> loader(
    "foxglove_bridge", "foxglove_bridge::ChannelSourcePlugin");
  const auto classes = loader.getDeclaredClasses();
  ASSERT_NE(std::find(classes.begin(), classes.end(), PLUGIN_CLASS), classes.end());

  auto plugin = loader.createSharedInstance(PLUGIN_CLASS);
  EXPECT_EQ(foxglove_bridge::CHANNEL_SOURCE_PLUGIN_API_VERSION, plugin->apiVersion());

  auto node = std::make_shared<rclcpp::Node>("example_channel_source_test");
  plugin->initialize(*node, "example", foxglove::Context::create());
  EXPECT_TRUE(node->has_parameter("example.topic"));
  plugin->shutdown();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
