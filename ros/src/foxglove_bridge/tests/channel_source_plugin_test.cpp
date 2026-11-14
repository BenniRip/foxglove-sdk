#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
// See smoke_test.cpp for why the no-TLS client config is used.
#include <websocketpp/config/asio_no_tls_client.hpp>

#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/ros2_foxglove_bridge.hpp>

#include "client/test_client.hpp"

namespace {

constexpr char TEST_PLUGIN_TOPIC[] = "/test_plugin/data";
constexpr char TEST_PLUGIN_MESSAGE[] = R"({"value":42})";
constexpr char TEST_PLUGIN_CLASS[] = "foxglove_bridge_test_plugins::TestChannelSourcePlugin";
constexpr char SECOND_INSTANCE_TOPIC[] = "/second_instance/data";
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(10);
constexpr char SYSINFO_TOPIC[] = "/foxglove_bridge/sysinfo";
// Name of the bridge node that the tests connect to, and the prefix of its loggers.
constexpr char BRIDGE_NODE_NAME[] = "foxglove_bridge";

// Set in main once the bridge has started.
std::string g_uri;
std::vector<std::string> g_startupErrors;

std::mutex g_logsMutex;
std::vector<std::string> g_errorLogs;
std::vector<std::string> g_infoLogs;
rcutils_logging_output_handler_t g_defaultLogHandler = nullptr;

bool isBridgeLogger(const char* name) {
  return name != nullptr && std::string(name).rfind(BRIDGE_NODE_NAME, 0) == 0;
}

void captureLogs(const rcutils_log_location_t* location, int severity, const char* name,
                 rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
  if (severity >= RCUTILS_LOG_SEVERITY_INFO) {
    va_list argsCopy;
    va_copy(argsCopy, *args);
    char buffer[1024];
    std::vsnprintf(buffer, sizeof(buffer), format, argsCopy);
    va_end(argsCopy);
    std::lock_guard<std::mutex> lock(g_logsMutex);
    if (severity < RCUTILS_LOG_SEVERITY_ERROR) {
      g_infoLogs.emplace_back(buffer);
    } else if (isBridgeLogger(name)) {
      g_errorLogs.emplace_back(buffer);
    }
  }
  g_defaultLogHandler(location, severity, name, timestamp, format, args);
}

/// Returns and clears the errors logged by the bridge under test.
std::vector<std::string> takeErrorLogs() {
  std::lock_guard<std::mutex> lock(g_logsMutex);
  return std::exchange(g_errorLogs, {});
}

void clearInfoLogs() {
  std::lock_guard<std::mutex> lock(g_logsMutex);
  g_infoLogs.clear();
}

bool contains(const std::vector<std::string>& logs, const std::string& text) {
  return std::any_of(logs.begin(), logs.end(), [&text](const std::string& log) {
    return log.find(text) != std::string::npos;
  });
}

bool infoLogged(const std::string& text) {
  std::lock_guard<std::mutex> lock(g_logsMutex);
  return contains(g_infoLogs, text);
}

/// Returns the port from the bridge's "Server listening" log. The port parameter is read-only, so
/// the bridge cannot report an ephemeral port through it.
int listeningPort() {
  constexpr char prefix[] = "Server listening on port ";
  std::lock_guard<std::mutex> lock(g_logsMutex);
  for (const auto& log : g_infoLogs) {
    if (const auto pos = log.find(prefix); pos != std::string::npos) {
      return std::stoi(log.substr(pos + sizeof(prefix) - 1));
    }
  }
  throw std::runtime_error("the bridge did not log its port");
}

/// Subscribes to and unsubscribes from a topic's channel, and returns the errors logged meanwhile.
std::vector<std::string> errorsWhileSubscribing(const std::string& topic) {
  foxglove::test::Client<websocketpp::config::asio_client> client;
  auto channelFuture = client.waitForChannel(topic);
  if (client.connect(g_uri).wait_for(DEFAULT_TIMEOUT) != std::future_status::ready ||
      channelFuture.wait_for(DEFAULT_TIMEOUT) != std::future_status::ready) {
    return {"could not find channel for " + topic};
  }
  const auto channel = channelFuture.get();
  takeErrorLogs();
  client.subscribe({{1, channel.id}});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  client.unsubscribe({1});
  // Subscriptions are handled asynchronously, so give the bridge time to process them.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return takeErrorLogs();
}

/// Options for a short-lived bridge that loads an instance of the test plugin named `probe`.
rclcpp::NodeOptions probeBridgeOptions(const std::string& nodeName) {
  rclcpp::NodeOptions nodeOptions;
  nodeOptions.arguments({"--ros-args", "-r", "__node:=" + nodeName});
  nodeOptions.append_parameter_override(foxglove_bridge::PARAM_PORT, 0);
  nodeOptions.append_parameter_override(foxglove_bridge::PARAM_SYSINFO, false);
  nodeOptions.append_parameter_override(foxglove_bridge::PARAM_CHANNEL_SOURCE_PLUGINS,
                                        std::vector<std::string>({"probe"}));
  nodeOptions.append_parameter_override("probe.plugin", TEST_PLUGIN_CLASS);
  return nodeOptions;
}

}  // namespace

TEST(ChannelSourcePluginTest, pluginChannelIsAdvertisedAndDeliversMessages) {
  foxglove::test::Client<websocketpp::config::asio_client> client;
  auto channelFuture = client.waitForChannel(TEST_PLUGIN_TOPIC);
  ASSERT_EQ(std::future_status::ready, client.connect(g_uri).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DEFAULT_TIMEOUT));
  const auto channel = channelFuture.get();
  EXPECT_EQ("json", channel.encoding);
  EXPECT_EQ("test_plugin.Data", channel.schemaName);

  const foxglove::test::SubscriptionId subscriptionId = 1;
  auto msgFuture = client.waitForChannelMsg(subscriptionId);
  client.subscribe({{subscriptionId, channel.id}});
  ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(DEFAULT_TIMEOUT));
  const auto msg = msgFuture.get();
  EXPECT_EQ(std::string(TEST_PLUGIN_MESSAGE), std::string(msg.begin(), msg.end()));

  client.unsubscribe({subscriptionId});
}

TEST(ChannelSourcePluginTest, subscribingToPluginChannelLogsNoErrors) {
  const auto errors = errorsWhileSubscribing(TEST_PLUGIN_TOPIC);
  EXPECT_TRUE(errors.empty()) << "first unexpected error: " << errors.front();
}

TEST(ChannelSourcePluginTest, subscribingToSystemInfoChannelLogsNoErrors) {
  // Like plugin channels, the system info channel has no ROS subscription.
  const auto errors = errorsWhileSubscribing(SYSINFO_TOPIC);
  EXPECT_TRUE(errors.empty()) << "first unexpected error: " << errors.front();
}

TEST(ChannelSourcePluginTest, instancesOfTheSamePluginLoadSideBySide) {
  foxglove::test::Client<websocketpp::config::asio_client> client;
  auto channelFuture = client.waitForChannel(SECOND_INSTANCE_TOPIC);
  ASSERT_EQ(std::future_status::ready, client.connect(g_uri).wait_for(DEFAULT_TIMEOUT));
  EXPECT_EQ(std::future_status::ready, channelFuture.wait_for(DEFAULT_TIMEOUT));
}

TEST(ChannelSourcePluginTest, failingInstancesAreSkippedWithAnError) {
  EXPECT_TRUE(contains(g_startupErrors, "instance 'throwing': initialization failure"));
  EXPECT_TRUE(contains(g_startupErrors,
                       "instance 'unknown_class': no plugin description declares "
                       "the class 'foxglove_bridge_test_plugins::DoesNotExist'"));
  EXPECT_FALSE(contains(g_startupErrors, "DoesNotExist'. If the plugin was built against"));
  EXPECT_TRUE(contains(g_startupErrors, "parameter 'no_class.plugin' is not set"));
  EXPECT_TRUE(contains(g_startupErrors, "instance 'version_mismatch'"));
  EXPECT_TRUE(contains(g_startupErrors, "rebuild it"));
  EXPECT_TRUE(contains(g_startupErrors,
                       "Ignoring duplicate channel source plugin instance "
                       "'test_plugin'"));
  EXPECT_EQ(5u, g_startupErrors.size());
}

TEST(ChannelSourcePluginTest, pluginsAreShutDownWithTheBridge) {
  clearInfoLogs();
  {
    foxglove_bridge::FoxgloveBridge bridge(probeBridgeOptions("shutdown_test_bridge"));
    EXPECT_FALSE(infoLogged("Test plugin instance 'probe' shut down"));
  }
  EXPECT_TRUE(infoLogged("Test plugin instance 'probe' shut down"));
}

TEST(ChannelSourcePluginTest, pluginsAreShutDownWhenBridgeStartupFails) {
  clearInfoLogs();
  auto nodeOptions = probeBridgeOptions("failed_startup_test_bridge");
  // Fails after the plugins are loaded, since no certificate is given.
  nodeOptions.append_parameter_override(foxglove_bridge::PARAM_USETLS, true);
  EXPECT_THROW(foxglove_bridge::FoxgloveBridge{nodeOptions}, std::invalid_argument);
  EXPECT_TRUE(infoLogged("Test plugin instance 'probe' shut down"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  g_defaultLogHandler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(captureLogs);

  int testResult = 0;
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions nodeOptions;
    nodeOptions.arguments({"--ros-args", "-r", std::string("__node:=") + BRIDGE_NODE_NAME});
    nodeOptions.append_parameter_override(foxglove_bridge::PARAM_PORT, 0);
    // Only test_plugin and second_instance load, see failingInstancesAreSkippedWithAnError.
    nodeOptions.append_parameter_override(
      foxglove_bridge::PARAM_CHANNEL_SOURCE_PLUGINS,
      std::vector<std::string>({"throwing", "test_plugin", "unknown_class", "no_class",
                                "version_mismatch", "second_instance", "test_plugin"}));
    nodeOptions.append_parameter_override("throwing.plugin",
                                          "foxglove_bridge_test_plugins::ThrowingPlugin");
    nodeOptions.append_parameter_override("test_plugin.plugin", TEST_PLUGIN_CLASS);
    nodeOptions.append_parameter_override("unknown_class.plugin",
                                          "foxglove_bridge_test_plugins::DoesNotExist");
    nodeOptions.append_parameter_override("version_mismatch.plugin",
                                          "foxglove_bridge_test_plugins::VersionMismatchPlugin");
    nodeOptions.append_parameter_override("second_instance.plugin", TEST_PLUGIN_CLASS);
    foxglove_bridge::FoxgloveBridge node(nodeOptions);
    g_startupErrors = takeErrorLogs();
    g_uri = "ws://localhost:" + std::to_string(listeningPort());
    executor.add_node(node.get_node_base_interface());

    std::thread spinnerThread([&executor]() {
      executor.spin();
    });

    testResult = RUN_ALL_TESTS();

    executor.cancel();
    spinnerThread.join();
    executor.remove_node(node.get_node_base_interface());
  }

  // The global context must be shut down before returning, see smoke_test.cpp.
  rclcpp::shutdown();
  return testResult;
}
