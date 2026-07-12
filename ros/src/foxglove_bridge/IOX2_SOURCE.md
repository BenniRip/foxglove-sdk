# Optional iceoryx2 source

When the bridge is built with `-DFOXGLOVE_BRIDGE_WITH_IOX2=ON`, an iceoryx2 (iox2) source is
compiled in that republishes configured iox2 FlatBuffer services onto the bridge's existing
Foxglove server context. iox2 and ROS 2 data are then served over the **same** WebSocket,
which is the only way the Foxglove app can show both at once (it does not merge multiple
live connections client-side).

When the option is `OFF` (the default), the package is unchanged and pulls no iox2
dependency.

## How it works

The bridge already builds on the Foxglove SDK: it owns a `foxglove::Context` and a
`foxglove::WebSocketServer`, advertises `foxglove::RawChannel`s on that context, and pushes
bytes with `channel.log()`. The iox2 source (`src/iox2_source.cpp`) does the same: it opens
iox2 publish-subscribe subscribers, creates `RawChannel`s on the bridge's `_serverContext`
(encoding `flatbuffer`, schema from a `.bfbs` file), polls the subscribers on a ROS timer,
and logs each payload. The ROS source path is untouched.

### Message header (and how to turn it off)

By default the source uses an iox2 **user header** to locate the FlatBuffer within the
shared-memory slot and to carry a publisher-side wall-clock timestamp, so the bridge never
parses the payload. That header, `foxglove_bridge::Iox2MessageHeader`, is defined **inside
this package** (`include/foxglove_bridge/iox2_message_header.hpp`) with no external
dependency.

This makes the header the bridge's own wire contract: in header mode any publisher that feeds
this bridge must publish a `Slice<uint8_t>` payload with a user header of this exact layout
**and a matching iceoryx2 type name**.

> **The header is a stopgap.** It exists only because today's iceoryx2 has no first-class
> notion of a FlatBuffer payload. Set `iox2.use_message_header: false` to run **headerless**:
> the source subscribes with iox2's default (`void`) user header, treats the **entire payload
> slice** as the FlatBuffer, and uses the bridge's receive time as the log time (there is no
> publisher timestamp without the header). See the future-work section at the end. Header and
> headerless services cannot be mixed in one bridge instance; the toggle is global.

#### Matching the publisher's header type (header mode)

iceoryx2 matches on a type-name *string*, not on the C++ type. When a header type exposes a
static `IOX2_TYPE_NAME` member, iceoryx2 uses it verbatim; otherwise it derives the name from
the C++ type (`"__cxx__abi__" + typeid(T).name()`). `Iox2MessageHeader` carries an
`IOX2_TYPE_NAME` member so the bridge's subscriber can match a publisher whose C++ header type
has a different name, without depending on the publisher's package.

The name is a **build-time** setting (not a ROS parameter, because the typed C++ bindings
resolve it at compile time). It defaults to `foxglove_bridge::Iox2MessageHeader`; override it
to match the publisher:

```bash
colcon build --packages-select foxglove_bridge --cmake-args \
  -DFOXGLOVE_BRIDGE_WITH_IOX2=ON \
  -DFOXGLOVE_BRIDGE_IOX2_HEADER_TYPE_NAME=my_company::iox2::MessageHeader
```

The recommended setup is for the *publisher's* header type to carry its own readable
`IOX2_TYPE_NAME` (as above) and to pass the same string here. Without one, iceoryx2 derives a
compiler-specific mangled name (e.g. `__cxx__abi__N10my_company4iox213MessageHeaderE`) that
would have to be matched verbatim, which is fragile and ABI-dependent; avoid it.

Two caveats on the match:

* **Name alone is not enough.** iceoryx2's type identity is `{size, alignment, type_name}`.
  The override only fixes the *name*; `sizeof`/`alignof` of `Iox2MessageHeader` (24 bytes,
  8-byte aligned) must also equal the publisher's header. Note that iceoryx2 does *not* check
  the field layout: two headers with the same size/alignment/name but different field
  meanings would be accepted and produce garbage, so keep the struct in sync deliberately.
  When the service already exists at startup, the source validates the registered identity
  against the compiled-in one and logs an error naming both sides on a mismatch.
* **Length limit.** iceoryx2 caps type names at 255 characters and truncates silently beyond
  that, which would then fail to match for a non-obvious reason. Keep the name within 255
  characters.

## Building

```bash
colcon build --packages-select foxglove_bridge \
  --cmake-args -DFOXGLOVE_BRIDGE_WITH_IOX2=ON
```

Requirements when `ON`:

* `iceoryx2-cxx` (the iox2 C++ bindings) available to CMake.
* `.bfbs` schema files for the bridged topics (transitional, like the message header; see
  the future-work section).
* Publishers that emit a `Slice<uint8_t>` payload, either with an `Iox2MessageHeader`-compatible
  user header in the default header mode, or a bare FlatBuffer slice when
  `iox2.use_message_header` is `false`.

Export `FOXGLOVE_BRIDGE_WITH_IOX2=ON` in the environment so colcon orders the optional
`package.xml` dependencies before the build:

```bash
export FOXGLOVE_BRIDGE_WITH_IOX2=ON
```

Notes:

* No C++ standard is pinned for the iox2 source: it is written to compile warning-clean at
  C++17 (the bridge's baseline, built with `-Wpedantic -Werror`). iceoryx2-cxx defaults to
  C++17 and propagates its standard via PUBLIC compile features, so CMake raises the
  component's standard automatically if a given iceoryx2-cxx build requires more.
* The integration is deliberately minimal (one source file plus its guarded headers, one
  CMake option, a guarded member, and one guarded construction in the bridge constructor),
  so the default build is unaffected.

## Configuring

iox2 services are configured via ROS parameters under the `iox2.` prefix. See
`config/iox2_services.example.yaml`. An empty `service_names` leaves the source inert.

| Parameter | Type | Meaning |
| :-- | :-- | :-- |
| `iox2.schema_dir` | string | Directory holding the `.bfbs` schema files |
| `iox2.poll_interval_ms` | int | Subscriber poll period (default 1) |
| `iox2.use_message_header` | bool | `true` (default): read an `Iox2MessageHeader` per sample; `false`: whole payload is the FlatBuffer, log time is the bridge's receive time |
| `iox2.service_names` | string[] | iox2 services to subscribe to |
| `iox2.topics` | string[] | Foxglove topics to advertise (parallel to services) |
| `iox2.schema_names` | string[] | Foxglove schema names (parallel) |
| `iox2.bfbs_files` | string[] | `.bfbs` files relative to `schema_dir` (parallel) |

## Limitations

* No MCAP recording from the iox2 source (the bridge does not record).
* TF is not specially forwarded: the ROS bridge already carries `/tf` and `/tf_static` as
  ROS topics, which Foxglove understands natively.
* Only FlatBuffer payloads are supported (the encoding Foxglove can consume without a
  message-definition lookup).

## Future work

Two pieces of configuration exist only because today's iceoryx2 has no first-class notion
of a FlatBuffer payload, and both are expected to disappear if it gains one:

* **The message header.** The `Iox2MessageHeader` exists only to *frame* the FlatBuffer
  (`rootOffset`, `payloadSize`) and to carry a log timestamp. With first-class FlatBuffer
  payloads (the transport itself knowing where the buffer starts and how long it is), the
  bridge no longer needs to read any header content and `iox2.use_message_header: false`
  becomes the natural default; the header contract then disappears from the bridge, and
  user headers become a publisher/consumer concern the bridge is indifferent to.
* **The `.bfbs` schema files.** These are the channel schemas the Foxglove app needs to
  decode the payloads; today they must be shipped to the bridge on disk (`iox2.schema_dir`,
  `iox2.bfbs_files`). If first-class FlatBuffer support registers the schema with the
  service (the natural design), the bridge can obtain it through service discovery instead,
  and the schema parameters disappear as well.

Matching a publisher's header *identity* without compiling it in is also possible today
(iceoryx2's service discovery exposes the registered `{name, size, alignment}`, and its C
API accepts type details as runtime values); it is left out here to keep the change small,
since any header layout change forces a coordinated publisher rebuild anyway. Combined with
the two points above, that would make the bridge fully configuration-light: subscribe to a
service name, discover everything else.
