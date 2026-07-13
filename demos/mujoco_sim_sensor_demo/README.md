# mujoco_sim_sensor_demo

All processes use strict Protocol V2.0 endpoints and the unified Adapter
lifecycle. Server, SDK, and demo binaries are rebuilt and deployed together.

`mujoco_sim_sensor_demo` is a pure eCAL and PCL demonstration of synchronous
vehicle control, non-barrier observation, asynchronous camera/LiDAR transport,
and LiDAR/IMU mapping. It does not require a ROS 2 runtime.

## Package Responsibility

The package owns the 12 x 8 x 3 m room and four-wheel skid-steer MJCF, the
perimeter waypoint controller, the PCL ICP mapper, map serialization, four
process entry points, strict Demo configuration, and the process launcher. It
does not implement simulation scheduling, adapter registration, stream
rendering, or eCAL framing; those remain in `mujoco_sim_server`,
`mujoco_sim_adapter_sdk`, `mujoco_sim_sensor`, and `mujoco_sim_protocol`.

The four Demo processes are:

| Executable | Client | Responsibility |
| --- | --- | --- |
| `mujoco-sim-sensor-demo-controller` | `SyncAdapterClient` | Receives wheel position and IMU data in each Tick, computes four wheel velocity commands, and Commit-s each Tick. It participates in the simulation barrier. |
| `mujoco-sim-sensor-demo-state-observer` | `ObserverClient` | Receives decimated wheel joint state and IMU frames for monitoring. It never Commit-s and never joins the barrier. |
| `mujoco-sim-sensor-demo-camera-consumer` | `SensorStreamClient` | Validates RGB8, metric depth32, uint32 segmentation, and optical XYZ frames and writes a real RGB PNG. |
| `mujoco-sim-sensor-demo-mapper` | `ObserverClient` + `SensorStreamClient` | Time-pairs synchronized IMU orientation with asynchronous 16-line LiDAR, runs PCL ICP, and writes map artifacts on shutdown. |

Numeric consumers decode `DEPTH32F`, `SEGMENTATION32U`, and `XYZ32F` from the
protocol-defined little-endian wire order; they do not depend on host
endianness.

`SyncAdapterClient` intentionally keeps the existing small-observation
semantics: the Controller Tick contains all configured joint/IMU observations,
and the Tick is completed only by an explicit Commit. `ObserverClient` provides
the same simulation-step-consistent small data without delaying the simulation.
Images and point clouds are separate asynchronous streams and are never copied
into Tick or ObservationFrame messages.

## Build And Test

From the workspace root:

```bash
colcon build --packages-up-to mujoco_sim_sensor_demo \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mujoco_sim_sensor_demo \
  --event-handlers console_direct+
colcon test-result --verbose
```

PCL is resolved through its system pkg-config files. The package does not use
or hard-code `/opt/MVS`; PCD v0.7 output is written directly, so PCL's optional
OpenNI/`libusb` IO dependency is not loaded by the Demo.

## Start The Complete Demo

After sourcing the colcon install environment, run from the repository root:

```bash
source install/setup.bash
mujoco-sim-sensor-demo-launch \
  --server-config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml \
  --demo-config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml \
  --output-dir /tmp/mujoco_sensor_demo \
  --mode headless
```

Headless mode removes `DISPLAY` and `WAYLAND_DISPLAY` for the Server and uses
automatic EGL surfaceless/pbuffer selection with OSMesa fallback. To open the MuJoCo viewer while keeping all
sensor streams active, change only the launch mode:

```bash
mujoco-sim-sensor-demo-launch \
  --server-config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml \
  --demo-config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml \
  --output-dir /tmp/mujoco_sensor_demo_gui \
  --mode gui
```

The launcher starts the Server first, waits for its registry, starts and waits
for the state observer, camera consumer, and mapper, then starts the Controller.
The normal completion condition is one closed perimeter lap. The Controller
does not depend on Server `max_ticks`; the Server configuration keeps
`max_ticks: 0`. After the lap, the launcher sends `SIGINT` to the three sensor
processes, and to the optional truth recorder used by validation, so each client
can unregister and the mapper can atomically write its outputs. Every child has
a bounded `SIGINT -> SIGTERM -> SIGKILL` shutdown; the launcher continues to
monitor the Server while clients flush, rejects premature or unexpected Server
exit statuses, and cannot wait indefinitely. `SIGINT` or `SIGTERM` sent to the
launcher runs the same bounded cleanup for every child process.

Each process can also be started manually with the same strict config:

```bash
mujoco-sim-server --config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml --headless
mujoco-sim-sensor-demo-state-observer --config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml
mujoco-sim-sensor-demo-camera-consumer --config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml
mujoco-sim-sensor-demo-mapper --config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml
mujoco-sim-sensor-demo-controller --config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml
```

Start the Controller last. Use a unique `--server-name` on all five commands
when multiple demos share one eCAL network.

## Configuration

`config/sensor_demo.schema.yaml` generates the C++20 parser through
`mujoco_sim_config`. YAML is strict: duplicate keys, unknown fields, wrong
types, unsigned overflow, invalid ranges, duplicate resource names, output
filename collisions, and invalid keepalive/timeout relationships are rejected
transactionally. Precedence is `CLI > YAML > generated defaults`, independent
of where `--config` appears.

All declared interfaces are configurable:

- Controller wheel position, IMU quaternion/gyro, and four command names.
- State Observer wheel position/velocity and IMU quaternion/gyro/acceleration
  names, delivery step interval, queue, keepalive, and timeout.
- Mapper IMU observation name and delivery policy, LiDAR stream name, client
  queue, time-pairing tolerance, and all ICP/output settings.
- RGB, depth, segmentation, and camera point-cloud stream names, client queue,
  screenshot filename, and output directory.
- Server camera/LiDAR source names, rates, dimensions, queue modes/depths,
  overflow policy, range, frame IDs, and render backend.

Common launch overrides include `--server-name`, `--output-dir`,
`--service-timeout-ms`, and `--wait-timeout-ms`. Run any process with `--help`
for generated options. `--controller-max-simulation-time-s`,
`--state-max-frames`, `--camera-max-samples`, and `--mapper-max-scans` are
bounded-test controls; zero disables each limit.

The supplied Server catalog uses one shared 320 x 240 camera capture at 8 Hz
for RGB/depth/segmentation/XYZ, plus a 360 x 16, 360-degree LiDAR at 8 Hz.
Camera streams use `latest` depth 1. LiDAR uses bounded FIFO depth 4 with
`drop_oldest`. Changing queue policy changes capture backpressure only as
documented by the Server; it does not add streams to the Controller barrier.

## Outputs

The output directory contains:

- `map.pcd`: binary PCD v0.7 XYZ map.
- `trajectory.csv`: timestamped ICP poses and fitness values.
- `map_top_down.png`: top-down map with the estimated trajectory overlaid.
- `front_camera.png`: validated RGB sensor frame, useful as a render check.
- `logs/`: one log per Server/Demo process when using the launcher.

The mapping algorithm uses only LiDAR and IMU inputs. Ground-truth model state
is not available to the mapper; tests may inspect model truth only after a run
to score loop completion and map quality. The headless E2E gate transforms map
points into world coordinates from the truth XY/yaw sample nearest the mapper's
first accepted LiDAR timestamp, then checks the five asymmetric columns for
surface support, angular coverage, inlier ratio, and residual. Deterministic
scorer tests cover a recorder sample that predates mapper startup and reject Y
reflection, map-only uniform scale, anisotropic scale, and accumulated drift in
addition to trajectory ATE/RPE/scale checks.

The test suite separately captures and decodes a real MuJoCo `simulate` GUI
screenshot under Xvfb, and runs GUI plus offscreen camera streaming together to
verify that viewer and sensor rendering can coexist. Fake-process launcher tests
cover signal-ignoring children, a Server that fails while clients flush, and an
unexpected Server shutdown status. The GUI stream script separately bounds both
camera timeout escalation and Server `SIGINT -> SIGTERM -> SIGKILL` cleanup. The
model contract detects a large static roof at any height above the floor by
geometry and arena coverage rather than by a specific geom name.
