#pragma once

#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <iox2/iceoryx2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove_bridge/iox2_message_header.hpp>

namespace foxglove_bridge {

/// Optional iceoryx2 source for the Foxglove bridge (Product A of the iox2 bridge plan).
///
/// Subscribes to a configured set of iceoryx2 publish-subscribe services that carry
/// FlatBuffer payloads (a raw byte slice plus an Iox2MessageHeader) and republishes each
/// onto a foxglove::RawChannel created on the bridge's existing server Context. Because
/// the channels share the bridge's Context, iox2 and ROS 2 data are served over the same
/// WebSocket, which is the only way the Foxglove app can show both at once (it does not
/// merge multiple live connections client-side).
///
/// Compiled only when the package is built with -DFOXGLOVE_BRIDGE_WITH_IOX2=ON. Without
/// that option the bridge is byte-for-byte the upstream ROS 2 bridge and pulls no iox2
/// dependency.
///
/// Configured via ROS parameters (all under the "iox2." prefix):
///   - iox2.schema_dir (string): directory holding the .bfbs schema files
///   - iox2.service_names, iox2.topics, iox2.schema_names, iox2.bfbs_files
///       (parallel string arrays; one entry per bridged service)
///   - iox2.poll_interval_ms (int, default 1): subscriber poll period
///   - iox2.user_header_type_name (string): override the iceoryx2 user-header type name to
///       match the publisher (empty keeps the default contract name)
/// An empty service_names list disables the source (no-op).
class Iox2Source {
public:
  /// @param node The bridge node, used for parameters, logging and the poll timer.
  /// @param context The bridge's server Context; iox2 channels are created on it so they
  ///                are advertised over the same WebSocket as the ROS channels.
  Iox2Source(rclcpp::Node& node, const foxglove::Context& context);
  ~Iox2Source();

  Iox2Source(const Iox2Source&) = delete;
  Iox2Source& operator=(const Iox2Source&) = delete;
  Iox2Source(Iox2Source&&) = delete;
  Iox2Source& operator=(Iox2Source&&) = delete;

private:
  using MsgHeader = Iox2MessageHeader;
  using IpcNode = iox2::Node<iox2::ServiceType::Ipc>;
  using PubSubService =
    iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, iox2::bb::Slice<uint8_t>, MsgHeader>;
  using Iox2Subscriber =
    iox2::Subscriber<iox2::ServiceType::Ipc, iox2::bb::Slice<uint8_t>, MsgHeader>;

  struct Entry {
    std::string serviceName;
    std::string topic;
    foxglove::RawChannel channel;
    std::unique_ptr<PubSubService> service;
    std::unique_ptr<Iox2Subscriber> subscriber;
  };

  struct Config {
    std::string serviceName;
    std::string topic;
    std::string schemaName;
    std::string bfbsFile;
  };

  std::vector<Config> loadParameters();
  void setupSubscribers(const foxglove::Context& context, const std::vector<Config>& configs);
  void pollOnce();

  rclcpp::Node& node_;
  std::filesystem::path schemaDir_;
  std::string userHeaderTypeName_;
  std::unique_ptr<IpcNode> iox2Node_;
  std::vector<Entry> entries_;
  rclcpp::TimerBase::SharedPtr pollTimer_;
};

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
