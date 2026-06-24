# Optional iceoryx2 source (Product A)

This is a Gravis addition to the upstream `foxglove_bridge`. When enabled, an iceoryx2
(iox2) source is compiled into the bridge and republishes configured iox2 FlatBuffer
services onto the bridge's existing Foxglove server context. iox2 and ROS 2 data are then
served over the **same** WebSocket, which is the only way the Foxglove app can show both at
once (it does not merge multiple live connections client-side).

When the option is `OFF` (the default), this package is byte-for-byte the upstream ROS 2
bridge and pulls no iox2 dependency.

## How it works

The bridge already builds on the Foxglove SDK: it owns a `foxglove::Context` and a
`foxglove::WebSocketServer`, advertises `foxglove::RawChannel`s on that context, and pushes
bytes with `channel.log()`. The iox2 source (`src/iox2_source.cpp`) does the same: it opens
iox2 publish-subscribe subscribers, creates `RawChannel`s on the bridge's `_serverContext`
(encoding `flatbuffer`, schema from a `.bfbs` file), polls the subscribers on a ROS timer,
and logs each payload. The ROS source path is untouched.

iox2 keys services on exact type identity, so the source uses a user header to locate the
FlatBuffer and carry a publisher-side wall-clock timestamp (so the bridge never parses the
payload). That header, `foxglove_bridge::Iox2MessageHeader`, is defined **inside this
package** (`include/foxglove_bridge/iox2_message_header.hpp`) with no external/vendor
dependency, so the source stays upstreamable.

This makes the header the bridge's own wire contract: any publisher that feeds this bridge
must publish a `Slice<uint8_t>` payload with a user header of this exact layout **and a
matching iceoryx2 type name**.

iceoryx2 matches on a type-name *string*, not on the C++ type, so the user-header type name
is **runtime-configurable**. The bridge resolves it via an `IOX2_DEFINE_TYPE_NAME`
specialization that calls `iox2UserHeaderTypeName()`, which the `iox2.user_header_type_name`
parameter sets at startup. Leave the parameter empty to use the default contract name
(`foxglove_bridge::Iox2MessageHeader`), or set it to whatever iceoryx2 type name your
publisher uses for its header. This lets the bridge attach to publishers that use a
differently-named header type (e.g. a vendor `MsgHeader`) without any code change; only the
header layout must match.

## Building

```bash
colcon build --packages-select foxglove_bridge \
  --cmake-args -DFOXGLOVE_BRIDGE_WITH_IOX2=ON
```

Requirements when `ON`:

* `iceoryx2-cxx` (the iox2 C++ bindings) available to CMake.
* `.bfbs` schema files for the bridged topics.
* Publishers that emit `Slice<uint8_t>` + an `Iox2MessageHeader`-compatible user header.

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
* This is a maintained Gravis patch on upstream. Keep it minimal (one source file, one
  CMake option, a guarded include/member, and one guarded construction in the bridge
  constructor) so it rebases cleanly.

## Configuring

iox2 services are configured via ROS parameters under the `iox2.` prefix. See
`config/iox2_services.example.yaml`. An empty `service_names` leaves the source inert.

| Parameter | Type | Meaning |
| :-- | :-- | :-- |
| `iox2.schema_dir` | string | Directory holding the `.bfbs` schema files |
| `iox2.poll_interval_ms` | int | Subscriber poll period (default 1) |
| `iox2.user_header_type_name` | string | Override the iceoryx2 user-header type name to match the publisher (empty = default contract name) |
| `iox2.service_names` | string[] | iox2 services to subscribe to |
| `iox2.topics` | string[] | Foxglove topics to advertise (parallel to services) |
| `iox2.schema_names` | string[] | Foxglove schema names (parallel) |
| `iox2.bfbs_files` | string[] | `.bfbs` files relative to `schema_dir` (parallel) |

## Limitations / not in scope

* This is Product A only (iox2 alongside ROS in the ROS 2 bridge). The ROS-free, standalone
  iox2 bridge (Product B) is separate and not part of this change.
* No MCAP recording from the iox2 source (the upstream bridge does not record).
* TF is not specially forwarded: the ROS bridge already carries `/tf` and `/tf_static` as
  ROS topics, which Foxglove understands natively.
