# MuJoCo Sensor Demo

This demo exercises the complete Protocol V3 stack with five processes:

- a synchronous waypoint Controller using dense Tick/Commit frames;
- a 100 Hz state Observer using dense Observation frames;
- a camera consumer for RGB, depth, segmentation, and camera point clouds;
- a mapper combining ray LiDAR with Observer IMU orientation;
- an optional every-step truth Observer used only by the E2E quality test.

All Controller/Observer layouts are resolved once after activation.
`VehicleDecoder`, `StateDecoder`, `ImuOrientationDecoder`, and
`WheelCommandMapper` cache scalar spans and perform no resource-name lookup per
frame. Sensor payloads remain V3 framed raw data and are not inserted into Tick
or Observation frames.

Configuration is generated from
[`config/sensor_demo.schema.yaml`](config/sensor_demo.schema.yaml) and strict.
Controller timing uses `commit_timeout_ms`, `session_timeout_ms`, and
`heartbeat_period_ms`; Observer processes use `session_timeout_ms`,
`heartbeat_period_ms`, and an explicit cadence. Removed field names are not
accepted.

Run from an installed workspace:

```bash
demos/mujoco_sim_sensor_demo/scripts/launch_sensor_demo.sh \
  --mode gui \
  --server-config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml \
  --demo-config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml
```

The demo server configuration explicitly enables every-step Observer delivery
because the E2E truth recorder requests it. Normal Server defaults keep that
mode disabled.

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
