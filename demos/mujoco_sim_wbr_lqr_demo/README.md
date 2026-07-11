# mujoco_sim_wbr_lqr_demo

This package ports the runtime WBR LQR controller from
`CosmosMount/wbr_mujoco` to the repository's V1 lockstep architecture. It
ships the imported model, a synchronous controller library, generated YAML/CLI
configuration, and the executable `mujoco-sim-wbr-lqr-adapter`.

The controller equations and coefficient tables remain in byte-identical
vendored headers. Repository-specific code only adapts model observations,
scripted input, configuration, and actuator commands around that control logic.

## Package Responsibilities

- Register one synchronous adapter with 21 observation resources and six
  actuator command resources.
- Decode named Tick blocks into the upstream `msg_raw_state_t` motor/IMU
  structure.
- Execute the upstream IMU, command fusion, leg solve, odometry, FSM, VMC, and
  LQR stages once per simulator Tick.
- Map six raw torques back to configured actuator resources and clamp them to
  the ranges returned by server registration.
- Load strict generated YAML/CLI configuration for connection settings,
  resource mappings, controller parameters, input scripts, and logging.
- Provide the WBR MJCF, 15 mesh files, a server configuration, and headless/GUI
  end-to-end tests.

The package does not own `mjModel` or `mjData`, call `mj_step()`, or
communicate with eCAL directly. Those responsibilities remain in
`mujoco_sim_core`, `mujoco_sim_server`, and `mujoco_sim_adapter_sdk`.

## Imported Content

The import is pinned in [UPSTREAM.md](UPSTREAM.md):

- 8 runtime `control/*.hpp` headers
- `scene.xml` and `mjmodel.xml`
- 15 STL meshes

The only model behavior change is an explicit `0.001` second MuJoCo timestep.
The server starts the model from keyframe `home`, selected by
`initial_keyframe` in [wbr_server.yaml](config/wbr_server.yaml).

## Resource Configuration

Every declared state and command resource is configurable in
[wbr_lqr.yaml](config/wbr_lqr.yaml). The six `semantic_name` values identify
fixed control slots; YAML item order and concrete MuJoCo resource names do not
define controller semantics.

| Semantic slot | Default position | Default velocity | Default effort | Default command |
|---|---|---|---|---|
| `left_joint_1` | `ljoint1_pos` | `ljoint1_vel` | `ljoint1_tor` | `ljoint1_actuator` |
| `left_joint_4` | `ljoint4_pos` | `ljoint4_vel` | `ljoint4_tor` | `ljoint4_actuator` |
| `left_wheel` | `lwheel_pos` | `lwheel_vel` | `lwheel_tor` | `lwheel_actuator` |
| `right_joint_1` | `rjoint1_pos` | `rjoint1_vel` | `rjoint1_tor` | `rjoint1_actuator` |
| `right_joint_4` | `rjoint4_pos` | `rjoint4_vel` | `rjoint4_tor` | `rjoint4_actuator` |
| `right_wheel` | `rwheel_pos` | `rwheel_vel` | `rwheel_tor` | `rwheel_actuator` |

The IMU defaults are `imu_quat[4]`, `imu_gyro[3]`, and `imu_accel[3]`.
Together with 18 scalar motor states, the adapter registers 21 named
observation blocks containing 28 scalar values.

`TickDecoder` and `TorqueCommandMapper` look up every resource by configured
name. They reject missing, duplicate, unexpected, dimension-mismatched,
non-finite, and float-overflowing data. Layout and Tick ordering may differ.

## Configuration

The build-time schema is
[wbr_lqr.schema.yaml](config/wbr_lqr.schema.yaml). Generated parsing is strict
and transactional:

- Priority is `CLI > YAML > generated defaults`, independent of where
  `--config` appears in argv.
- Unknown/duplicate YAML fields, wrong types, invalid enums, non-finite values,
  unsigned overflow, and schema constraint failures are rejected.
- CLI exposure is opt-in. Resource names and detailed control parameters are
  YAML-only; frequently changed process settings have CLI flags.

Supported adapter CLI overrides are:

```text
--config
--server-name
--controller-id
--timeout-ms
--service-timeout-ms
--ticks
--imu-mode
--log-every-ticks
```

Run `mujoco-sim-wbr-lqr-adapter --help` for generated help. The shipped YAML
also configures Mahony/FSM/PID values and half-open scripted input intervals
`[begin_tick, end_tick)`. A Space transition must be a one-Tick pulse after
Tick 0, with at least one released Tick between pulses. These indices are local
to the adapter session and start at zero on its first handled Tick; they are not
the server-global `Tick.header.tick_id`.

## Build

From the workspace root:

```bash
colcon build --packages-up-to mujoco_sim_wbr_lqr_demo \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

The adapter library depends on `mujoco_sim_adapter_sdk` and
`mujoco_sim_config`. The runnable demo also declares `mujoco_sim_server` as a
runtime dependency because the shipped server YAML is part of the primary
workflow.

The install tree contains:

```text
bin/mujoco-sim-wbr-lqr-adapter
share/mujoco_sim_wbr_lqr_demo/config/wbr_lqr.yaml
share/mujoco_sim_wbr_lqr_demo/config/wbr_server.yaml
share/mujoco_sim_wbr_lqr_demo/mjcf/scene.xml
share/mujoco_sim_wbr_lqr_demo/mjcf/meshes/*
```

The relative `model_path: ../mjcf/scene.xml` in `wbr_server.yaml` is resolved
against that YAML file's directory, so the installed example works from any
process working directory.

## Run Headless

From the workspace root, capture the absolute share path. Then open two
terminals and load the workspace in each one:

```bash
source install/setup.bash
demo_share="$(pwd)/install/mujoco_sim_wbr_lqr_demo/share/mujoco_sim_wbr_lqr_demo"
```

Start the server first:

```bash
mujoco-sim-server \
  --config "$demo_share/config/wbr_server.yaml" \
  --headless \
  --max-ticks 0
```

Then start the adapter while the server is running:

```bash
mujoco-sim-wbr-lqr-adapter \
  --config "$demo_share/config/wbr_lqr.yaml"
```

Both shipped configurations use server name `wbr`. The adapter's `max_ticks`
counts 6000 newly handled `ACTIVE` Ticks. Server `max_ticks`, however, counts
all simulation steps since model load, including autonomous steps before the
adapter registers. The command above therefore overrides the shipped server
value `6000` with `0`; after the adapter automatically unregisters and exits,
stop the server separately. For a bounded server run, provide a larger total
step budget that includes startup time.

The adapter prints periodic FSM/clamp counters and a final summary with handled,
nonzero-command, clamped-command, and scripted Space/E/Q effect counts.

## Run With GUI

Use the same adapter command and start the server with `--gui`:

```bash
mujoco-sim-server \
  --config "$demo_share/config/wbr_server.yaml" \
  --gui \
  --max-ticks 0
```

On a machine without a physical display, use `xvfb-run -a` to exercise the
GUI path. Use `--headless` when rendering is not required.

## Synchronous Control Loop

For adapter-session Tick `n`, `TickController::HandleTick` performs:

```text
named observation blocks
  -> msg_raw_state_t
  -> IMU update
  -> command fusion(input_n, pendulum_{n-1})
  -> left/right leg solve with az=0
  -> odometry
  -> left/right leg solve with odometry az
  -> upstream FSM/VMC/LQR
  -> six raw torques
  -> registered ctrlrange clamp
  -> six named CommandValue entries
```

`pendulum_{-1}` is value-initialized. There are no controller worker threads
or latest-value shared-memory reads: one Tick produces one deterministic
command set before the adapter Commit is published.

The server supports registering, activating, and unregistering this adapter
while simulation is already `RUNNING`. A per-adapter timeout clears its six
actuators and moves only this session to `TIMED_OUT`; the server and other
controllers continue. The SDK handles a `TIMED_OUT` Tick with a non-blocking
try-lock. If the controller callback is free, its valid Commit is recovery
proof only and the server restores the session to `ACTIVE` on the next Tick;
recovery Ticks do not count toward the adapter's 6000-Tick limit.
They do advance the stateful controller pipeline and its session-local input
script index because the handler successfully processed them.

An exception in decoding or control stops the SDK loop and makes the process
exit with code 1. If the server marks this exact session `ERROR` because of a
local command failure, the SDK also stops, prints `lifecycle_warning`, and
attempts to unregister. Normal completion likewise performs best-effort
unregistration. The current SDK callback has no structured handler-error
Commit, which is recorded as a next-stage architecture issue in
[MIGRATION_REPORT.md](MIGRATION_REPORT.md).

## Library API

Downstream CMake consumers can link the reusable controller:

```cmake
find_package(mujoco_sim_wbr_lqr_demo CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE mujoco_sim::wbr_lqr_demo)
```

The package config exports `mujoco_sim_wbr_lqr_demo_MJCF_DIR` for locating
the installed model. Public APIs include `InterfaceConfig`,
`MakeAdapterOptions`, `TickDecoder`, `TorqueCommandMapper`,
`ControllerPipeline`, `InputScript`, and `TickController`.

## Tests

```bash
colcon test --packages-select mujoco_sim_wbr_lqr_demo
colcon test-result --verbose
```

CTest covers:

- pinned upstream control golden values and deterministic pipeline reset
- default and fully renamed/reordered interface decoding and torque mapping
- command range clamping and non-finite rejection
- generated YAML translation, semantic motor validation, and input scripts
- Core loading of the real model from keyframe `home`
- installed CMake target, MJCF directory, all assets, adapter help, and a
  staged-config headless run from a different working directory
- a nonblank PNG captured from the real WBR GLFW render loop
- real eCAL headless and GUI/Xvfb lockstep runs against an unlimited server;
  both verify automatic unregistration, then terminate the server explicitly

See [MIGRATION_REPORT.md](MIGRATION_REPORT.md) for deliberate differences,
verification counts, and prioritized architecture improvements.
