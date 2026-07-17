# mujoco_sim_sensor

`mujoco_sim_sensor` provides transport-independent offscreen RGB, depth,
segmentation, camera point-cloud, and ray LiDAR capture from isolated MuJoCo
snapshots. It does not own eCAL endpoints or define wire messages.

The installed C++ API exposes `OffscreenRenderer`, `RayLidar`, profile and
descriptor helpers through `<mujoco_sim/sensor/sensor.hpp>`. Camera sources
support `RGB8`, metric `DEPTH32F`, `SEGMENTATION32U`, and camera-frame
`XYZ32F`; all use `ROS_OPTICAL`. Ray LiDAR produces organized `XYZ32F` in
`ROS_BODY`, with misses encoded as NaNs. Camera sources require a named
perspective MuJoCo camera, while LiDAR sources require a named MuJoCo site and
exactly one vertical-angle mode (an explicit angle list or a sampled FOV).

Offscreen rendering selects surfaceless EGL, EGL pbuffer, then OSMesa when the
backend preference is automatic. MuJoCo's OpenGL dispatch is process-global,
so after the first renderer selects EGL or OSMesa, the same process cannot
switch to the other platform family.

The Server maps captured buffers into the independent `MJSTREAM` 1.0 Sensor
stream framing. This stream is not a Protocol V4 `MJDATA4` frame: its header is
bounded to 1 MiB and its payload to 512 MiB. A blocking stream preserves
lossless completed-step ordering and is interruptible by the Server stop token.
Non-blocking streams use bounded latest/FIFO queues.
Subscriber state is sampled by `SensorRuntime` every 200 ms by default, so
capture never performs a subscriber query and a newly connected consumer may
wait up to one sample period for lazy capture to begin.

`Capture()` consumes an isolated `SimulationSnapshot` and caller-supplied
epoch/tick/time/sequence metadata. It returns a Protobuf
`StreamFrameHeader` plus raw payload bytes; transport serialization and queue
ownership remain Server responsibilities. The source must be used with the
same model instance from which it was created.

## Build and test

```bash
cmake -S core/mujoco_sim_sensor -B build/mujoco_sim_sensor \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_sensor -j
ctest --test-dir build/mujoco_sim_sensor --output-on-failure
```

Both EGL and OSMesa development packages are required at configure time.
Installed consumers use `find_package(mujoco_sim_sensor CONFIG REQUIRED)` and
link `mujoco_sim::sensor`.
