# mujoco_sim_protocol

`mujoco_sim_protocol` is the single Protocol V3 wire contract. It contains
Protobuf messages for the low-frequency control plane, a fixed binary codec for
the high-frequency data plane, Sensor stream framing, and eCAL endpoint naming.

## Contract

- Protobuf package: `mujoco_sim.v3`
- Registry: `mujoco_sim.v3.<server>.registry`
- Runtime topics: `mujoco_sim/v3/<server>/...`
- Framing version: `1.0`
- Maximum Tick, Commit, or Observation frame: 4 MiB

The generated header is `<mujoco_sim/v3/sync.pb.h>`. Protobuf is used for
resource discovery, registration/lifecycle RPCs, session heartbeat, Controller
fault, Server status, and Sensor metadata. Tick, Commit, and Observation are not
Protobuf messages.

## Dense data frames

`<mujoco_sim/protocol/data_frame.hpp>` implements an 80-byte little-endian
header followed by contiguous IEEE-754 `double` values. The header carries kind,
session, epoch, Tick, 128-bit layout fingerprint, Observer sequence, simulation
time, scalar count, and exact frame size. It contains no adapter or resource
names.

```cpp
mujoco_sim::protocol::DataFrameHeader header;
header.kind = mujoco_sim::protocol::DataFrameKind::kCommit;
header.session_id = 4;
header.epoch = 2;
header.tick_id = 10;
header.layout = {fingerprint_low, fingerprint_high};

std::string bytes;
mujoco_sim::protocol::DataFrameError error;
std::array<double, 2> commands{0.1, -0.1};
if (!mujoco_sim::protocol::EncodeDataFrame(
        header, commands, &bytes, &error)) {
  // Handle error.message.
}
```

The codec reads and writes individual fields and never casts a packed struct.
Parsing validates magic, framing version, kind, flags, header size, exact total
size, and scalar count before exposing the payload view. `DecodeDataFrameScalars`
copies into caller-owned reusable storage. A reserved output buffer performs no
steady-state allocation.

Sensor payloads use `<mujoco_sim/protocol/stream_frame.hpp>`: a fixed prefix,
serialized V3 `StreamFrameHeader`, and raw payload bytes.

## Endpoints

| Helper | Result for `server=default`, `adapter=arm` |
| --- | --- |
| `RegistryServiceName` | `mujoco_sim.v3.default.registry` |
| `TickTopicName` | `mujoco_sim/v3/default/tick/arm` |
| `CommitTopicName` | `mujoco_sim/v3/default/commit` |
| `ObservationTopicName` | `mujoco_sim/v3/default/observation/arm` |
| `SessionHeartbeatTopicName` | `mujoco_sim/v3/default/session_heartbeat` |
| `ControllerFaultTopicName` | `mujoco_sim/v3/default/controller_fault` |
| `StatusTopicName` | `mujoco_sim/v3/default/status` |
| `SensorStreamTopicName` | `mujoco_sim/v3/default/sensor/<stream>` |

There are no endpoint aliases or version negotiation fallbacks.

## Build and test

```bash
cmake -S core/mujoco_sim_protocol -B build/mujoco_sim_protocol \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_protocol -j
ctest --test-dir build/mujoco_sim_protocol --output-on-failure
```

`protocol_test` covers fixed golden bytes, malformed fields, unaligned input,
NaN/Inf bit preservation, exact-size rejection, and an arbitrary-input corpus.
`data_frame_allocation_test` verifies zero warmed-buffer allocations across
10,000 encode/parse/decode iterations.
