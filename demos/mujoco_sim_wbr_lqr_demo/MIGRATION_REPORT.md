# WBR LQR Migration Report

## Scope Completed

- Pinned controller repository commit
  `wbr_mujoco@1d895574d91e3b14d13696676d9f91b2b4403ee4` and input/runtime
  reference commit
  `mujoco_interface@a2501454b09038469137deccb1d8c6ee460d1c69`.
- Imported 8 byte-identical runtime `control/*.hpp` headers, 2 MJCF files,
  and all 15 STL meshes.
- Made the model timestep explicit at `0.001` seconds.
- Added Core named-keyframe loading and start the WBR model from `home`.
- Replaced the upstream multi-service/eCAL topology with one
  deterministic `ControllerPipeline::Step` per SDK-delivered State. The SDK
  exposes the latest completed-step State through a depth-1 mailbox, so a slow
  Controller may coalesce intermediate States without blocking the Server.
- Preserved upstream IMU, command fusion, leg solve, odometry, FSM,
  VMC/LQR, motor ordering, signs, gains, and coefficient tables.
- Added a generated YAML/CLI configuration and the executable
  `mujoco-sim-wbr-lqr-adapter`.
- Made all 21 declared state resource names and all 6 command resource names
  configurable. Six semantic motor names preserve control-slot meaning even
  when YAML items and protocol layouts are reordered.
- Added strict layout/value validation, finite float conversion, registered
  actuator range clamping, and clamp statistics. The public integration surface
  uses `StateDecoder` and `WbrController::ComputeCommand()` to match the
  asynchronous State/Command protocol terminology.
- Added deterministic half-open input scripts using the `mujoco_interface`
  input snapshot fields. The shipped stand-up sequence pulses Space and then E.
- Added source-tree and installed-layout server configurations. Relative model
  paths are resolved against the YAML location.
- Added unit, model-contract, headless eCAL, and GUI/Xvfb eCAL tests.

## Quantitative Result

| Item | Count |
|---|---:|
| Imported control headers | 8 |
| Imported MJCF files | 2 |
| Imported mesh files | 15 |
| Semantic motors | 6 |
| Observation resources | 21 |
| Observation scalar values | 28 |
| Command resources | 6 |
| Shipped runtime YAML files | 2 |
| CTest targets in the demo package | 10 |

The direct closed-loop CTest completes 6000 real MuJoCo Steps from `home` and
asserts sustained `NORMAL`, raised base height, bounded pitch, bounded travel,
and the middle leg-length target. The headless eCAL test consumes 6000 delivered
States and asserts the Space/E effects. Separate tests run a 200-Step GUI/Xvfb
loop, capture a nonblank WBR render, and execute the staged adapter/config/model
from a different working directory.

## Deliberate Differences

1. Upstream used independent threads and latest-value message reads. The port
   processes one observation snapshot and produces one command set per
   SDK-delivered State. State delivery follows the granted cadence, and the SDK
   may coalesce intermediate States when the Controller is slower than the
   Server.
2. Upstream optional IMU noise injection lives in its eCAL state I/O and was
   not migrated. Mahony and ground-truth quaternion bypass modes remain
   configurable.
3. Upstream internal limits are `40 Nm` for joint outputs and `5.2 Nm` for
   wheel outputs, while the imported MJCF exposes `20 Nm` and `5.2 Nm`
   actuator ranges. The port preserves equations and clamps joint commands at
   the protocol boundary.
4. Invalid layouts, values, and controller outputs are explicit process errors
   instead of propagating non-finite data.
5. The `mujoco_interface` GUI publishes live input snapshots. Protocol V4 does
   not currently expose that channel, so this demo represents the same fields
   with deterministic configured Controller-tick intervals.

## Migration Friction And Next-Stage Goals

### Resolved: Adapter Callback Coupling

The initial port ran decoder/controller code inside the SDK's eCAL State
callback and forced command creation and publication into one return path.
That made thread ownership, delayed commands, and controller exceptions
awkward.

The SDK now exposes explicit `Activate()`, `WaitForNewState()`, and
`SetCommand()` operations. The WBR pipeline runs only on its business thread,
and failures report a Fault or stop and unregister the local session. A
publisher thread sends new command generations and keepalives while preserving
the same sequence and bytes across send retries.

### P0: Typed Semantic Interface Binding

The protocol and SDK accept flat string lists, so the demo owns a second
semantic mapping layer and manually validates six required slots. The generated
configuration validates shape, but it cannot express that object-sequence
`semantic_name` values must equal one required set.

Target improvement: add reusable typed interface declarations that bind a
semantic key to state/command resources, generate required-set/uniqueness
validation, and flatten them into `ControllerOptions` automatically.

### P1: Cross-Field Configuration Validation

`mujoco_sim_config` handles field-local constraints but application code still
implements cross-field rules: semantic set equality, non-overlapping input
intervals, one-Step Space pulses, and float narrowing.

Target improvement: support generated custom validators or declarative
cross-field constraints while keeping business-specific conversion code
separate from YAML/CLI parsing.

### P1: Shared Launch Configuration

Server and adapter require separate YAML files that duplicate server name,
Step limits, timeouts, and startup ordering. End-to-end scripts currently
coordinate two processes manually.

Target improvement: add a launch/composition manifest that resolves installed
assets, derives per-process configuration, allocates one simulation instance,
and manages shutdown consistently.

### P1: Model/Timing Contract Discovery

The controller assumes `0.001` seconds and required resource dimensions. The
demo verifies these in tests, but registration does not negotiate a control
period, and status does not expose keyframes or model timestep.

Target improvement: expose timestep and available keyframes in resource/model
metadata, allow adapters to declare timing requirements, and reject mismatches
before activation.

### P2: Layout Diagnostics

Protocol V4 identifies the immutable registered layout in every State by its
layout ID. The adapter validates that ID before decoding values, but layout
mismatch diagnostics are still local to the process.

Target improvement: expose layout mismatch counters and the active layout ID in
status diagnostics without expanding the high-frequency State payload.

### P2: Interactive Input Channel

The original controller consumed live keyboard input. The deterministic script
is suitable for tests but does not provide operator control through the server
GUI.

Target improvement: define a separate operator-input topic/API with explicit
timestamp/Step semantics, then choose whether live input replaces or composes
with scripted input.

### P2: Adapter Diagnostics

Clamp counts and FSM state are only process logs. They are not visible through
`mujoco-sim-status`.

Target improvement: add optional controller diagnostic fields or a dedicated
diagnostic topic without placing high-frequency telemetry in the State/Command
data path.

## Verification

The package tests cover:

- upstream golden controller outputs and deterministic reset
- named and fully renamed/reordered resource mapping
- malformed layouts, dimensions, finite ranges, and torque clamping
- generated configuration merge/validation and transactional conversion
- scripted input boundaries and pulse constraints
- Core model loading from keyframe `home`
- 6000-Step MuJoCo closed-loop stand-up and steady balancing
- real eCAL headless and GUI/Xvfb asynchronous execution

All imported provenance and byte-identity details are recorded in
[UPSTREAM.md](UPSTREAM.md).

## Post-Migration Interactive Input Extension

After the migration recorded above, the demo added an optional first-party
standard-input terminal source. Script input remains the default and Protocol
V4 is unchanged. Cin mode maps terminal characters to the existing
`input_snapshot_t` fields. W/A/S/D use bounded configurable Controller-frame
lifetimes (default 200 updates), while Q/E/F/Space retain bounded one-shot
behavior. A subsequent intentional WBR command-mapping patch assigns fixed
signed velocity and yaw-rate references; the original migration evidence and
CTest count reported above remain historical.
