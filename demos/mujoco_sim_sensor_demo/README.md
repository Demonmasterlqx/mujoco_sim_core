# MuJoCo Sensor Demo

This demo exercises the complete Protocol V4 stack with one Server and four
demo clients:

- an asynchronous waypoint Controller using dense State/Command frames;
- a 100 Hz state Observer using dense Observation frames;
- a camera consumer for RGB, depth, segmentation, and camera point clouds;
- a mapper combining ray LiDAR with Observer IMU orientation;

The E2E quality test adds an optional fifth client: an every-step truth
Observer whose data is used only for post-run scoring.

All Controller/Observer layouts are resolved once after registration.
`VehicleDecoder`, `StateDecoder`, `ImuOrientationDecoder`, and
`WheelCommandMapper` cache scalar spans and perform no resource-name lookup per
frame. They reject mismatched dense scalar counts, overlapping command spans,
non-finite values, and invalid simulation timestamps without partially updating
outputs. Sensor payloads retain their separate `MJSTREAM` framing with a
Protocol V4 `StreamFrameHeader`; they are not inserted into State or Observation
frames.

Configuration is generated from
[`config/sensor_demo.schema.yaml`](config/sensor_demo.schema.yaml) and strict.
Controller timing, keepalive, State cadence, and control policies are granted
by the Server; Observer processes use `session_timeout_ms`,
`heartbeat_period_ms`, and an explicit cadence. Removed field names are not
accepted.

From the repository root after sourcing the installed workspace:

```bash
mujoco-sim-sensor-demo-launch \
  --mode gui \
  --server-config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml \
  --demo-config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml
```

The demo server configuration explicitly enables every-step Observer delivery
because the mapper requests it; the optional E2E truth recorder uses the same
cadence. Normal Server defaults keep that mode disabled. The launcher starts
consumers before the Controller, bounds startup and shutdown, writes logs and
generated artifacts beneath `--output-dir`, and defaults to headless mode.

Build and test:

```bash
cmake -S demos/mujoco_sim_sensor_demo -B build/mujoco_sim_sensor_demo \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_sensor_demo -j
ctest --test-dir build/mujoco_sim_sensor_demo --output-on-failure
```

Tests cover algorithms, strict configuration, dense adapters, model contract,
rendering, installed consumption, launcher cleanup, headless quality, and GUI
streaming.
