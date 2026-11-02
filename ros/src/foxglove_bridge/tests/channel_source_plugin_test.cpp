#include <chrono>
#include <cstdarg>
#include <cstring>
#include <future>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
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
constexpr char WRONG_VERSION_PLUGIN_TOPIC[] = "/test_plugin/wrong_version";
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(10);
// smoke_test uses 8765.
constexpr int64_t PORT = 8766;
const std::string URI = "ws://localhost:" + std::to_string(PORT);

std::mutex g_errorLogsMutex;
std::vector<std::string> g_errorLogs;
rcutils_logging_output_handler_t g_defaultLogHandler = nullptr;

void captureErrorLogs(const rcutils_log_location_t* location, int severity, const char* name,
                      rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
  if (severity >= RCUTILS_LOG_SEVERITY_ERROR) {
    va_list argsCopy;
    va_copy(argsCopy, *args);
    char buffer[1024];
    std::vsnprintf(buffer, sizeof(buffer), format, argsCopy);
    va_end(argsCopy);
    std::lock_guard<std::mutex> lock(g_errorLogsMutex);
    g_errorLogs.emplace_back(buffer);
  }
  g_defaultLogHandler(location, severity, name, timestamp, format, args);
}

std::vector<std::string> takeErrorLogs() {
  std::lock_guard<std::mutex> lock(g_errorLogsMutex);
  return std::exchange(g_errorLogs, {});
}

}  // namespace

TEST(ChannelSourcePluginTest, pluginChannelIsAdvertisedAndDeliversMessages) {
  foxglove::test::Client<websocketpp::config::asio_client> client;
  auto channelFuture = client.waitForChannel(TEST_PLUGIN_TOPIC);
  ASSERT_EQ(std::future_status::ready, client.connect(URI).wait_for(DEFAULT_TIMEOUT));
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
  foxglove::test::Client<websocketpp::config::asio_client> client;
  auto channelFuture = client.waitForChannel(TEST_PLUGIN_TOPIC);
  ASSERT_EQ(std::future_status::ready, client.connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DEFAULT_TIMEOUT));
  const auto channel = channelFuture.get();
  takeErrorLogs();

  const foxglove::test::SubscriptionId subscriptionId = 1;
  auto msgFuture = client.waitForChannelMsg(subscriptionId);
  client.subscribe({{subscriptionId, channel.id}});
  ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(DEFAULT_TIMEOUT));
  client.unsubscribe({subscriptionId});
  // The unsubscribe is handled asynchronously; give the bridge time to process it.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const auto errors = takeErrorLogs();
  EXPECT_TRUE(errors.empty()) << "first unexpected error: " << errors.front();
}

TEST(ChannelSourcePluginTest, incompatiblePluginIsNotInitialized) {
  std::mutex topicsMutex;
  std::set<std::string> advertisedTopics;
  auto testTopicPromise = std::make_shared<std::promise<void>>();
  auto testTopicFuture = testTopicPromise->get_future();

  foxglove::test::Client<websocketpp::config::asio_client> client;
  client.setTextMessageHandler([&, testTopicPromise](const std::string& payload) {
    const auto msg = nlohmann::json::parse(payload);
    if (msg.value("op", "") != "advertise") {
      return;
    }
    std::lock_guard<std::mutex> lock(topicsMutex);
    for (const auto& channel : msg["channels"]) {
      const auto topic = channel["topic"].get<std::string>();
      advertisedTopics.insert(topic);
      if (topic == TEST_PLUGIN_TOPIC) {
        testTopicPromise->set_value();
      }
    }
  });
  ASSERT_EQ(std::future_status::ready, client.connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, testTopicFuture.wait_for(DEFAULT_TIMEOUT));

  std::lock_guard<std::mutex> lock(topicsMutex);
  EXPECT_EQ(0u, advertisedTopics.count(WRONG_VERSION_PLUGIN_TOPIC));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  g_defaultLogHandler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(captureErrorLogs);

  int testResult = 0;
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions nodeOptions;
    nodeOptions.append_parameter_override(foxglove_bridge::PARAM_PORT, PORT);
    // The empty entry mirrors the launch file default; ThrowingPlugin and WrongApiVersionPlugin
    // must be skipped without affecting the working plugin or the bridge.
    nodeOptions.append_parameter_override(
      foxglove_bridge::PARAM_CHANNEL_SOURCE_PLUGINS,
      std::vector<std::string>({"", "foxglove_bridge_test_plugins::ThrowingPlugin",
                                "foxglove_bridge_test_plugins::WrongApiVersionPlugin",
                                "foxglove_bridge_test_plugins::TestChannelSourcePlugin",
                                "foxglove_bridge_test_plugins::DoesNotExist"}));
    foxglove_bridge::FoxgloveBridge node(nodeOptions);
    executor.add_node(node.get_node_base_interface());

    std::thread spinnerThread([&executor]() {
      executor.spin();
    });

    testResult = RUN_ALL_TESTS();

    executor.cancel();
    spinnerThread.join();
    executor.remove_node(node.get_node_base_interface());
  }

  // See smoke_test.cpp: the global context must be shut down before returning.
  rclcpp::shutdown();
  return testResult;
}
