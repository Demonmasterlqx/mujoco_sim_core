# mujoco_sim_protocol

`mujoco_sim_protocol` is the repository's only Protocol V4 wire contract. It
contains Protobuf control-plane messages, fixed dense data-plane codecs, Sensor
stream framing, canonical layout fingerprints, and eCAL endpoint naming.

## Contract

- Protobuf package: `mujoco_sim.v4`
- Registry service: `mujoco_sim.v4.<server>.registry`
- Runtime topics: `mujoco_sim/v4/<server>/...`
- Dense framing: `2.0`, magic `MJDATA4\0`, little-endian
- Maximum complete dense frame: 4 MiB
- State/Observation header: 80 bytes
- Command header: 64 bytes

Dense kinds are fixed at Controller State 1, Controller Command 2, and
Observation 3; flags must be zero. State and Observation carry session, epoch,
completed-step tick, layout, sequence, and simulation time. Command carries
only session, epoch, command layout, and strictly increasing session sequence;
it has no tick, simulation time, source-State identity, or source-lag fields.

All multibyte dense-frame fields and IEEE-754 `double` payload scalars are
little-endian. The fixed offsets are:

| Offset | Common field | State / Observation | Command |
| ---: | --- | --- | --- |
| 0 | 8-byte magic | `MJDATA4\0` | `MJDATA4\0` |
| 8 | framing major/minor | `uint16 2`, `uint16 0` | same |
| 12 | kind/flags/header bytes | `uint8`, `0`, `uint16 80` | `uint8`, `0`, `uint16 64` |
| 16 | frame/scalar counts | `uint32`, `uint32` | same |
| 24 | session/epoch | `uint64`, `uint64` | same |
| 40 | kind-specific identity | tick at 40, layout at 48 | layout at 40 |
| 64 | final metadata | sequence at 64, simulation time at 72 | sequence at 56 |

```cpp
mujoco_sim::protocol::DataFrameHeader header;
header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
header.session_id = 4;
header.epoch = 2;
header.layout = {fingerprint_low, fingerprint_high};
header.sequence = 1;

std::string bytes;
mujoco_sim::protocol::DataFrameError error;
std::array<double, 2> commands{0.1, -0.1};
if (!mujoco_sim::protocol::EncodeDataFrame(
        header, commands, &bytes, &error)) {
  // Handle error.message.
}
```

The codec reads individual fields and never casts a packed struct. Parsing
validates magic, version, kind-specific header size, flags, exact frame size,
nonzero sequence, and scalar count before exposing the payload view. The
caller must additionally validate session identity, layout, expected scalar
count, finite values, command ranges, and sequence ordering against its active
session.

The 16-byte layout fingerprint is FNV-1a-128 over canonical ordered resource
metadata, each scalar offset, and the final observation/command scalar counts.
`ValidateLayoutHash()` also requires nonzero resource dimensions, contiguous
offsets, exact counts, and a matching fingerprint. It is an identity checksum,
not a cryptographic digest.

Sensor raw payloads use a separate `MJSTREAM` framing version 1.0 through
`<mujoco_sim/protocol/stream_frame.hpp>`. Its 24-byte little-endian prefix
contains magic, framing version, serialized Protocol V4 `StreamFrameHeader`
size, and payload size. The serialized header is limited to 1 MiB and the raw
payload to 512 MiB; the parser requires both encoded sizes and the Protobuf
`payload_size` field to match the complete frame exactly.

## Endpoints

| Helper | Result for `server=default`, `adapter=arm` |
| --- | --- |
| `RegistryServiceName` | `mujoco_sim.v4.default.registry` |
| `ControllerStateTopicName` | `mujoco_sim/v4/default/controller_state/arm` |
| `ControllerCommandTopicName` | `mujoco_sim/v4/default/controller_command` |
| `ObservationTopicName` | `mujoco_sim/v4/default/observation/arm` |
| `SessionHeartbeatTopicName` | `mujoco_sim/v4/default/session_heartbeat` |
| `ControllerFaultTopicName` | `mujoco_sim/v4/default/controller_fault` |
| `StatusTopicName` | `mujoco_sim/v4/default/status` |
| `SensorStreamTopicName` | `mujoco_sim/v4/default/sensor/<stream>` |

Session heartbeat is used by Observers only. There are no endpoint aliases,
older framing decoders, or version negotiation fallbacks.

Identifiers are 1 to 64 ASCII characters, start with a letter or digit, and
then contain only letters, digits, `_`, or `-`. Controller commands share one
Server topic and are demultiplexed by session identity; State and Observation
topics are per adapter.

## Lifecycle messages

Registration returns an immutable `ResourceLayout`; Controller registration
also returns the complete granted `ControllerRuntimePolicy`. Controllers move
through `PROVISIONAL`, `ACTIVE_NO_COMMAND`, `ONLINE`, `OFFLINE`, and terminal
`ERROR`. A valid non-unspecified `ControllerFault` applies only to the exact
session/epoch and enters `ERROR`; malformed, stale, duplicate, conflicting, or
out-of-range Command frames are rejected without refreshing liveness or
entering `ERROR`.

Observers reuse `AdapterState` but do not use `ACTIVE_NO_COMMAND`. Automatic
expiry, explicit unregister, and replacement publish one `UNREGISTERED`
tombstone before that session disappears from later status messages.

## Build and test

```bash
cmake -S core/mujoco_sim_protocol -B build/mujoco_sim_protocol \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_protocol -j
ctest --test-dir build/mujoco_sim_protocol --output-on-failure
```

Installed consumers use `find_package(mujoco_sim_protocol CONFIG REQUIRED)`,
link `mujoco_sim::protocol`, and include the generated Protobuf API as
`<mujoco_sim/v4/sync.pb.h>`.
