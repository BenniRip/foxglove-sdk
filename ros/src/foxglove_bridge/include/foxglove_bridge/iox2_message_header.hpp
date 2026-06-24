#pragma once

#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include <cstdint>
#include <string>

namespace foxglove_bridge {

/// Generic user header for FlatBuffer-over-iceoryx2 messages bridged to Foxglove.
///
/// This is the bridge's own, self-contained wire contract: it deliberately has no external
/// dependency, so the bridge stays upstreamable. iceoryx2 keys publish-subscribe services on
/// exact type identity, so any publisher that feeds this bridge must publish a
/// `Slice<uint8_t>` payload with a user header of this exact layout and a matching iceoryx2
/// type name (see below).
///
/// Fields:
///   - rootOffset: byte offset of the FlatBuffer root within the payload slice.
///   - payloadSize: length of the FlatBuffer, in bytes.
///   - timestampNs: publisher wall-clock time (ns since epoch), used as the Foxglove log time
///     so the bridge never has to deserialize the payload to find a timestamp.
///   - sequence: monotonic counter for ordering / drop detection.
struct Iox2MessageHeader {
  uint32_t rootOffset;
  uint32_t payloadSize;
  uint64_t timestampNs;
  uint64_t sequence;
};

/// The iceoryx2 type name used for the user header, decoupled from the C++ type identity.
///
/// iceoryx2 matches services on a type-name string, not on the C++ type. By default this is a
/// stable contract name ("foxglove_bridge::Iox2MessageHeader"), but it is runtime-configurable
/// so the bridge can attach to publishers that use a different header type name without code
/// changes. Set it (once, before opening services) via setIox2UserHeaderTypeName(); the value
/// is wired into iceoryx2 through an IOX2_DEFINE_TYPE_NAME specialization in iox2_source.cpp.
const char* iox2UserHeaderTypeName();

/// Override the iceoryx2 user-header type name. An empty string keeps the current value.
/// Process-global; intended to be called once at startup before any iox2 service is opened.
void setIox2UserHeaderTypeName(const std::string& name);

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
