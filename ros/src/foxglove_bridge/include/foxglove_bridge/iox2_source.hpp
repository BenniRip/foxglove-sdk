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

/// Type-erased iox2 subscription (internal detail of Iox2Source). The concrete
/// implementations live in iox2_source.cpp: one reads an Iox2MessageHeader to locate the
/// FlatBuffer within the slot and to recover the publisher timestamp; the other (headerless)
/// treats the entire payload slice as the FlatBuffer. Type erasure lets header and headerless
/// services share one entry list even though they instantiate different iceoryx2
/// service/subscriber types.
struct Iox2Subscription {
  Iox2Subscription() = default;
  virtual ~Iox2Subscription() = default;
  Iox2Subscription(const Iox2Subscription&) = delete;
  Iox2Subscription& operator=(const Iox2Subscription&) = delete;

  /// Drains all queued samples into `channel`. Returns false after a fatal receive error,
  /// after which the caller drops this subscription (so the error is logged once per service
  /// rather than every poll).
  virtual bool drain(foxglove::RawChannel& channel, const rclcpp::Logger& logger) = 0;
};

/// Optional iceoryx2 source for the Foxglove bridge.
///
/// Subscribes to a configured set of iceoryx2 publish-subscribe services that carry
/// FlatBuffer payloads (a raw byte slice, optionally with an Iox2MessageHeader; see
/// iox2.use_message_header) and republishes each onto a foxglove::RawChannel created on the
/// bridge's existing server Context. Because
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
///   - iox2.use_message_header (bool, default true): when true, each sample carries an
///       Iox2MessageHeader that locates the FlatBuffer and supplies the publisher timestamp;
///       when false, the whole payload slice is the FlatBuffer and the bridge's receive time
///       is used. The header path is a stopgap and is expected to be retired once iceoryx2
///       ships first-class FlatBuffer support.
/// The iceoryx2 user-header type name is a build-time setting (Iox2MessageHeader::IOX2_TYPE_NAME,
/// overridable via -DFOXGLOVE_BRIDGE_IOX2_HEADER_TYPE_NAME=...), not a ROS parameter. When a
/// configured service already exists at startup, its registered header identity is validated
/// against the compiled-in one and a mismatch is reported naming both sides.
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
  using IpcNode = iox2::Node<iox2::ServiceType::Ipc>;

  struct Entry {
    foxglove::RawChannel channel;
    std::unique_ptr<Iox2Subscription> subscription;
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

  rclcpp::Node& _node;
  std::filesystem::path _schemaDir;
  bool _useHeader = true;
  std::unique_ptr<IpcNode> _iox2Node;
  std::vector<Entry> _entries;
  rclcpp::TimerBase::SharedPtr _pollTimer;
};

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
