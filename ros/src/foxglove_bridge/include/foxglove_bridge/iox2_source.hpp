#pragma once

#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include <filesystem>
#include <memory>
#include <optional>
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

/// Stand-in for the generated FlatBuffer type: iox2::Flatbuffer<T>'s service type identity
/// does not depend on T, and the bridge only forwards payload_bytes(), so it needs no
/// publisher schema compiled in.
struct OpaqueFlatbuffer {};

/// Republishes iceoryx2 FlatBuffer services onto the bridge's server context, so iox2 and
/// ROS data share one WebSocket (the Foxglove app does not merge live connections).
///
/// Configured via the `iox2.*` parameters, see IOX2_SOURCE.md.
class Iox2Source {
public:
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
    std::filesystem::path bfbsPath;
    foxglove::RawChannel channel;
    // Declared before subscriber: a PortFactory must outlive the ports it creates.
    std::optional<Service> service;
    std::optional<Subscriber> subscriber;
    std::optional<iox2::PublishSubscribeOpenError> lastOpenError;
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
  /// Opens the entry's service if a publisher has created it; never creates one.
  void tryOpen(Entry& entry);
  void openPending();
  void pollOnce();
  /// Returns false on a fatal receive error, so the caller can drop the entry.
  bool drain(Entry& entry);

  rclcpp::Node& _node;
  std::filesystem::path _schemaDir;
  std::unique_ptr<IpcNode> _iox2Node;
  std::vector<std::unique_ptr<Entry>> _entries;
  rclcpp::TimerBase::SharedPtr _pollTimer;
  rclcpp::TimerBase::SharedPtr _openTimer;
};

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
