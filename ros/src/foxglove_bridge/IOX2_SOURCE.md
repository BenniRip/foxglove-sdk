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

### No wire contract

The source subscribes to iceoryx2's **native FlatBuffer payloads**
(`iox2::Flatbuffer<T>`, available since iceoryx2 gained first-class FlatBuffer support;
build iceoryx2 with `IOX2_FEATURE_FLATBUFFERS=ON`). iceoryx2 itself knows where each
serialized FlatBuffer starts and ends, so the bridge defines no user header, no framing,
and no header-type matching — it reads `payload_bytes()` and forwards them.

Three properties make this work generically, without compiling in any publisher schema:

* `iox2::Flatbuffer<T>` registers a **T-independent** service type identity (the constant
  type name `"iox2::Flatbuffer"`), so the bridge can open any FlatBuffer service with its
  own opaque marker type.
* Compatibility is enforced through the **schema file** registered with the service:
  iceoryx2 stores the file's content when the service is created and rejects any
  participant whose file differs (byte-identical comparison). The bridge registers each
  entry's `.bfbs` file, so publishers must register the same file — which also guarantees
  the Foxglove channel schema matches the payloads.
* The bridge never parses the payload. Log time is the receive time (shared-memory latency
  is negligible); message-level timestamps live inside the FlatBuffers, where the Foxglove
  app reads them.

## Building

```bash
colcon build --packages-select foxglove_bridge \
  --cmake-args -DFOXGLOVE_BRIDGE_WITH_IOX2=ON
```

Requirements when `ON`:

* `iceoryx2-cxx` built with `IOX2_FEATURE_FLATBUFFERS=ON` (and its `flatbuffers`
  dependency) available to CMake. Building against an iceoryx2 without the feature fails
  with an explicit `#error`.
* `.bfbs` schema files for the bridged topics. Each file is both the Foxglove channel
  schema and the schema registered with the iceoryx2 service, so publishers must point
  iceoryx2 at the same file (e.g. via `flatbuffer_schema_path()`).

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
* The integration is deliberately minimal (one source file plus its guarded header, one
  CMake option, a guarded member, and one guarded construction in the bridge constructor),
  so the default build is unaffected.

## Configuring

iox2 services are configured via ROS parameters under the `iox2.` prefix. See
`config/iox2_services.example.yaml`. An empty `service_names` leaves the source inert.

| Parameter | Type | Meaning |
| :-- | :-- | :-- |
| `iox2.schema_dir` | string | Directory holding the `.bfbs` schema files |
| `iox2.poll_interval_ms` | int | Subscriber poll period (default 1) |
| `iox2.service_names` | string[] | iox2 services to subscribe to |
| `iox2.topics` | string[] | Foxglove topics to advertise (parallel to services) |
| `iox2.schema_names` | string[] | Foxglove schema names (parallel) |
| `iox2.bfbs_files` | string[] | `.bfbs` files relative to `schema_dir` (parallel); also registered as the iceoryx2 service schema |

A service whose schema file does not match the publisher's registered one fails to open;
the source logs the failure and skips that service instead of taking down the bridge.

## Limitations

* No MCAP recording from the iox2 source (the bridge does not record).
* TF is not specially forwarded: the ROS bridge already carries `/tf` and `/tf_static` as
  ROS topics, which Foxglove understands natively.
* Only FlatBuffer payloads are supported (the encoding Foxglove can consume without a
  message-definition lookup).

## Future work

* **Schema discovery.** The `.bfbs` files must currently be shipped to the bridge on disk
  (`iox2.schema_dir`, `iox2.bfbs_files`). iceoryx2 already stores the registered schema
  content with the service, so once its API exposes that content to consumers, the bridge
  could obtain channel schemas through service discovery and the schema parameters would
  disappear, leaving only a service-to-topic mapping.
