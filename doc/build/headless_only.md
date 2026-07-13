# Headless-only build

Configure the server with GUI and sensor streams disabled:

```bash
colcon build --packages-select mujoco_sim_server --cmake-args \
  -DMUJOCO_SIM_BUILD_GUI=OFF \
  -DMUJOCO_SIM_BUILD_SENSOR_STREAMS=OFF
```

This mode defaults to headless, does not find or link GLFW/PNG, and does not link the sensor package, EGL, or OSMesa. Passing `--gui` fails with a build-capability diagnostic. A non-empty `sensor_streams` configuration also fails before runtime startup.

Camera/LiDAR can be retained without GUI by setting sensor streams ON and GUI OFF. MuJoCo selection is independent: `MUJOCO_SIM_USE_BUNDLED_MUJOCO=ON` uses the repository distribution; OFF requires an installed `mujoco` CMake package.

`core/mujoco_sim_server/test/build_matrix.sh` builds, installs, consumes, smoke-tests, and checks dependencies for the supported matrices.
