# mujoco_sim_sensor

The package is used by the strict Protocol V2.0 Server build and contains only
the V2 stream contract.

`mujoco_sim_sensor` is the transport-free sensor capture package for immutable
`mujoco_sim::core::SimulationSnapshot` inputs. It provides configured camera
and 360-degree ray-lidar sources, protocol stream descriptors, and raw frame
payloads. It contains no eCAL integration, scheduling, queues, worker threads,
or backpressure policy.

The public entry point is `mujoco_sim/sensor/sensor.hpp`. Camera sources select
any combination of `RGB8`, `DEPTH32F`, `SEGMENTATION32U`, and `XYZ32F` streams.
Camera rows are top-left first; depth is metric optical Z; segmentation is zero
for background and MuJoCo geom ID plus one otherwise; XYZ is organized in ROS
optical coordinates. Ray lidar emits organized `XYZ32F` in ROS body coordinates
with NaNs for misses and range-rejected returns. Rays start at column zero on
local +X, advance counterclockwise toward local +Y, and exclude geoms attached
to the lidar site's non-world parent body. A site attached directly to the world
body keeps world geoms visible because they are the environment, not self geometry.

Camera rendering first attempts an EGL surfaceless context, then an EGL pbuffer
on the surfaceless platform, and finally OSMesa. It never opens an X11 or Wayland
display. Orthographic MuJoCo cameras are currently rejected because the protocol
camera-intrinsics descriptor and requested point-cloud convention are pinhole
camera contracts.

MuJoCo stores loaded OpenGL entry points process-wide. A process may create any
number of renderers using its first selected platform family, but it cannot
switch between EGL and OSMesa after the first MuJoCo render context is created.
Automatic selection follows the established family; an explicit conflicting
preference returns an error.

This README intentionally stays package-focused; integration and transport
examples belong in downstream demo and server packages.

## Build and use

This package is a library and cannot be started by itself. From the workspace
root:

```bash
colcon build --packages-up-to mujoco_sim_sensor --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

It requires C++20, `mujoco_sim_core`, `mujoco_sim_protocol`, EGL headers and
OSMesa headers. The renderer loads the selected GL platform at runtime. An
installed CMake consumer uses:

```cmake
find_package(mujoco_sim_sensor CONFIG REQUIRED)
target_link_libraries(my_capture PRIVATE mujoco_sim::sensor)
```

Public headers are `sensor.hpp`, `source_profile.hpp`,
`offscreen_renderer.hpp`, and `ray_lidar.hpp`. A caller first creates a
`SimulationSnapshot`, validates a `CameraProfile` or `RayLidarProfile`, creates
the corresponding capture object, and then supplies updated snapshots plus
monotonic `CaptureMetadata` for each frame. Capture objects remain bound to the
snapshot model used at creation; a snapshot cloned with
`SimulationCore::CloneSnapshot()` is compatible because it shares that model.
`StreamShapeFor()` is the allocation-free shape preflight used by downstream
configuration code. It returns the row stride and payload size while enforcing
the renderer `int` dimension limit and the protocol 512 MiB payload limit.

`mujoco_sim_server` is the production scheduler and eCAL publisher. Its static
YAML catalog supplies source names, shapes, rates, frame IDs, and queue policy;
this library deliberately does not interpret YAML or wall/simulation time.

## Data contracts

| Output | Scalar layout | Coordinates and invalid values |
|---|---|---|
| `RGB8` | packed R,G,B bytes | top-left row first |
| `DEPTH32F` | one float32 metre value per pixel | optical Z, not Euclidean ray distance |
| `SEGMENTATION32U` | one uint32 per pixel | `0` background, otherwise MuJoCo geom ID + 1 |
| camera `XYZ32F` | packed float32 X,Y,Z per pixel | organized ROS optical frame |
| ray LiDAR `XYZ32F` | packed float32 X,Y,Z per ray | organized ROS body frame; NaN triplet for misses |

All multi-byte payload scalars use little-endian wire order. `DEPTH32F` and
both `XYZ32F` variants are IEEE-754 binary32; `SEGMENTATION32U` is uint32.
Camera outputs from one profile share the same capture metadata. LiDAR columns
start at local +X and rotate counterclockwise toward +Y; rows follow configured
vertical angles from minimum to maximum. Stream framing and transport are owned
by `mujoco_sim_protocol` and Server, not this package.

## Tests

```bash
colcon test --packages-select mujoco_sim_sensor
colcon test-result --test-result-base build/mujoco_sim_sensor --verbose
```

Tests run with empty `DISPLAY`/`WAYLAND_DISPLAY` and cover nonblank EGL and
OSMesa capture, concurrent and repeated renderer creation, destroying one EGL
renderer while another remains usable, intrinsics, metric depth, optical-Z
near/far segmentation, point-cloud consistency, numeric little-endian payloads,
known LiDAR distances and NaN misses, vertical ring generation, invalid/overflow
profiles, calibrated principal-point projection, public shape limits, foreign
snapshots, deterministic repeated frames, and an
installed-package consumer.
