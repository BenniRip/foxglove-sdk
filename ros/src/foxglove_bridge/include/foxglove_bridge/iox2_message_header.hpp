#pragma once

#ifdef FOXGLOVE_BRIDGE_WITH_IOX2

#include <cstdint>

/// iceoryx2 type name for the user header (see Iox2MessageHeader::IOX2_TYPE_NAME below).
///
/// iceoryx2 keys publish-subscribe services on a type-name string. When a payload/header type
/// exposes a static `IOX2_TYPE_NAME` member, iceoryx2 uses it verbatim; otherwise it derives a
/// name from the C++ type (`"__cxx__abi__" + typeid(T).name()`). Publishers that feed this
/// bridge may use a differently-named C++ header type, so the bridge's header carries an
/// explicit IOX2_TYPE_NAME and we make it overridable at build time. The default is the
/// bridge's own contract name; override it to match the publisher, e.g.:
///
///   -DFOXGLOVE_BRIDGE_IOX2_HEADER_TYPE_NAME=__cxx__abi__N6gravis11iox2_common9MsgHeaderE
///
/// Only the iceoryx2 name plus the header's size/alignment must match the publisher; the C++
/// type itself is private to the bridge, so there is no dependency on the publisher's package.
#ifndef FOXGLOVE_BRIDGE_IOX2_HEADER_TYPE_NAME
#define FOXGLOVE_BRIDGE_IOX2_HEADER_TYPE_NAME "foxglove_bridge::Iox2MessageHeader"
#endif

namespace foxglove_bridge {

/// Generic user header for FlatBuffer-over-iceoryx2 messages bridged to Foxglove.
///
/// This is the bridge's own, self-contained wire contract: it deliberately has no external
/// dependency, so the bridge stays upstreamable. iceoryx2 keys publish-subscribe services on
/// exact type identity, so any publisher that feeds this bridge must publish a
/// `Slice<uint8_t>` payload with a user header of this exact layout and a matching iceoryx2
/// type name (IOX2_TYPE_NAME below).
///
/// Fields:
///   - rootOffset: byte offset of the FlatBuffer root within the payload slice.
///   - payloadSize: length of the FlatBuffer, in bytes.
///   - timestampNs: publisher wall-clock time (ns since epoch), used as the Foxglove log time
///     so the bridge never has to deserialize the payload to find a timestamp.
///   - sequence: monotonic counter for ordering / drop detection.
///
/// IOX2_TYPE_NAME is the iceoryx2 type-name string (a static member iceoryx2 reads to identify
/// the header type); it does not affect the struct's layout. Keep the four data members in sync
/// with the publisher's header: iceoryx2 also matches on size (24 bytes) and alignment (8).
struct Iox2MessageHeader {
  static constexpr const char* IOX2_TYPE_NAME = FOXGLOVE_BRIDGE_IOX2_HEADER_TYPE_NAME;

  uint32_t rootOffset;
  uint32_t payloadSize;
  uint64_t timestampNs;
  uint64_t sequence;
};

}  // namespace foxglove_bridge

#endif  // FOXGLOVE_BRIDGE_WITH_IOX2
