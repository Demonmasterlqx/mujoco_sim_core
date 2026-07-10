# WBR LQR Migration Report

## Scope Completed

- Pinned upstream repository commit
  `6a8b37517803db616d4fb19dcdd5ab448bff7847` and controller submodule commit
  `022d57e7d7ac1fbc7b6fbf3f4162a30bfe42ba5e`.
- Imported 8 byte-identical runtime `control/*.hpp` headers, 2 MJCF files,
  and all 15 STL meshes.
- Made the model timestep explicit at `0.001` seconds.
- Added Core named-keyframe loading and start the WBR model from `home`.
- Replaced the upstream four-thread/shared-memory topology with one
  deterministic `ControllerPipeline::Step` per protocol Tick.
- Preserved upstream IMU, command fusion, dual leg solve, odometry, FSM,
  VMC/LQR, motor ordering, signs, gains, and coefficient tables.
- Added a generated YAML/CLI configuration and the executable
  `mujoco-sim-wbr-lqr-adapter`.
- Made all 21 declared state resource names and all 6 command resource names
  configurable. Six semantic motor names preserve control-slot meaning even
  when YAML items and protocol layouts are reordered.
- Added strict layout/value validation, finite float conversion, registered
  actuator range clamping, and clamp statistics.
- Added deterministic half-open input scripts and translated the upstream
  Space/E/Q sequence.
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
| CTest targets in the demo package | 9 |

The automated headless CTest completes the default 6000 real lockstep Ticks,
and asserts the Space/E/Q command effects at Ticks 1, 2000, and 4000. Separate
tests run a 200-Tick GUI/Xvfb loop, capture a nonblank WBR render, and execute
the staged adapter/config/model from a different working directory.

## Deliberate Differences

1. Upstream used independent threads and latest-value message reads. The port
   processes exactly one observation snapshot and produces one command set per
   simulator Tick.
2. Upstream optional IMU noise injection lived in shared-memory I/O and was not
   migrated. Mahony and ground-truth quaternion bypass modes remain
   configurable.
3. Upstream internal limits are `40 Nm` for joint outputs and `10 Nm` for
   wheel outputs, while the imported MJCF exposes `20 Nm` and `5.2 Nm`
   actuator ranges. The port preserves equations and clamps only at the
   protocol boundary.
4. Invalid layouts, values, and controller outputs are explicit process errors
   instead of propagating non-finite data.
5. Keyboard input is represented by deterministic configured Tick intervals.
   The server GUI does not currently forward interactive keys to adapters.

## Migration Friction And Next-Stage Goals

### P0: Structured Adapter Handler Failure

The SDK callback can only return a command vector. A decoder/controller
exception currently requires `client.Stop()` plus an empty return value, and
the adapter reports the real error only at process exit.

Target improvement: let a Tick handler return either commands or a structured
error. Publish an error Commit when protocol state permits, preserve the
diagnostic in server status, and define whether the server pauses or removes
that controller.

### P0: Typed Semantic Interface Binding

The protocol and SDK accept flat string lists, so the demo owns a second
semantic mapping layer and manually validates six required slots. The generated
configuration validates shape, but it cannot express that object-sequence
`semantic_name` values must equal one required set.

Target improvement: add reusable typed interface declarations that bind a
semantic key to state/command resources, generate required-set/uniqueness
validation, and flatten them into `AdapterOptions` automatically.

### P1: Cross-Field Configuration Validation

`mujoco_sim_config` handles field-local constraints but application code still
implements cross-field rules: semantic set equality, non-overlapping input
intervals, one-Tick Space pulses, and float narrowing.

Target improvement: support generated custom validators or declarative
cross-field constraints while keeping business-specific conversion code
separate from YAML/CLI parsing.

### P1: Shared Launch Configuration

Server and adapter require separate YAML files that duplicate server name,
Tick limits, timeouts, and startup ordering. End-to-end scripts currently
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

### P2: Repeated Layout Transport And Validation

Every Tick carries the full resource layout, and the adapter compares it with
registration on every callback. This is robust but repetitive for static V1
sessions.

Target improvement: identify immutable layouts by epoch/layout ID, send the full
layout at registration or on change, and keep Tick payloads focused on values.

### P2: Interactive Input Channel

The original controller consumed live keyboard input. The deterministic script
is suitable for tests but does not provide operator control through the server
GUI.

Target improvement: define a separate operator-input topic/API with explicit
timestamp/Tick semantics, then choose whether live input replaces or composes
with scripted input.

### P2: Adapter Diagnostics

Clamp counts and FSM state are only process logs. They are not visible through
`mujoco-sim-status`.

Target improvement: add optional controller diagnostic fields or a dedicated
diagnostic topic without placing non-barrier telemetry in Commit.

## Verification

The package tests cover:

- upstream golden controller outputs and deterministic reset
- named and fully renamed/reordered resource mapping
- malformed layouts, dimensions, finite ranges, and torque clamping
- generated configuration merge/validation and transactional conversion
- scripted input boundaries and pulse constraints
- Core model loading from keyframe `home`
- real eCAL headless and GUI/Xvfb lockstep execution

All imported provenance and byte-identity details are recorded in
[UPSTREAM.md](UPSTREAM.md).
