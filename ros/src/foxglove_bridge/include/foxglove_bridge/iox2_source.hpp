#pragma once

#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <iox2/iceoryx2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>

#if !IOX2_FEATURE_FLATBUFFERS
#error "FOXGLOVE_BRIDGE_WITH_IOX2 requires iceoryx2 built with IOX2_FEATURE_FLATBUFFERS=ON"
#endif

namespace foxglove_bridge {

/// Marker type for iceoryx2's native FlatBuffer payload. The bridge forwards the serialized
/// bytes without parsing them, so no schema-generated C++ type is needed:
/// iox2::Flatbuffer<T> registers a T-independent service type identity (the constant type
/// name "iox2::Flatbuffer"), T is only used by the typed payload_root() accessor, and the
/// bridge reads payload_bytes() instead. What ties subscriber and publisher together is the
/// schema file registered with the service (see Iox2Source docs below).
struct OpaqueFlatbuffer {};

/// Optional iceoryx2 source for the Foxglove bridge.
///
/// Subscribes to a configured set of iceoryx2 publish-subscribe services carrying native
/// FlatBuffer payloads (iox2::Flatbuffer<T>) and republishes each sample's serialized bytes
/// onto a foxglove::RawChannel created on the bridge's existing server Context. Because the
/// channels share the bridge's Context, iox2 and ROS 2 data are served over the same
/// WebSocket, which is the only way the Foxglove app can show both at once (it does not
/// merge multiple live connections client-side).
///
/// There is no bridge-specific wire contract: no user header, no framing. iceoryx2 itself
/// knows where the FlatBuffer starts and ends, and enforces schema compatibility at open
/// time (the schema file registered by the service creator must be byte-identical to the
/// one this source passes, or the open fails). Log time is the bridge's receive time;
/// message-level timestamps live inside the FlatBuffers themselves, where Foxglove reads
/// them.
///
/// Compiled only when the package is built with -DFOXGLOVE_BRIDGE_WITH_IOX2=ON against an
/// iceoryx2 built with IOX2_FEATURE_FLATBUFFERS=ON. Without the option the bridge is
/// byte-for-byte the upstream ROS 2 bridge and pulls no iox2 dependency.
///
/// Configured via ROS parameters (all under the "iox2." prefix):
///   - iox2.schema_dir (string): directory holding the .bfbs schema files
///   - iox2.service_names, iox2.topics, iox2.schema_names, iox2.bfbs_files
///       (parallel string arrays; one entry per bridged service)
///   - iox2.poll_interval_ms (int, default 1): subscriber poll period
/// Each entry's .bfbs file serves double duty: it is the Foxglove channel schema and the
/// schema file registered with the iceoryx2 service, so publishers must register the same
/// file. An empty service_names list disables the source (no-op).
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
  using Payload = iox2::Flatbuffer<OpaqueFlatbuffer>;
  using Service = iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, Payload, void>;
  using Subscriber = iox2::Subscriber<iox2::ServiceType::Ipc, Payload, void>;

  struct Entry {
    std::string serviceName;
    std::string topic;
    foxglove::RawChannel channel;
    // The iceoryx2 PortFactory must outlive the ports it creates, so the service is stored
    // alongside its subscriber (and declared before it, so destroyed after it).
    Service service;
    Subscriber subscriber;
    bool warnedLogError = false;
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
  /// Drains all queued samples of `entry` into its channel. Returns false after a fatal
  /// receive error, after which the caller drops the entry (so the error is logged once per
  /// service rather than every poll).
  bool drain(Entry& entry);

  rclcpp::Node& _node;
  std::filesystem::path _schemaDir;
  std::unique_ptr<IpcNode> _iox2Node;
  std::vector<std::unique_ptr<Entry>> _entries;
  rclcpp::TimerBase::SharedPtr _pollTimer;
};

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
