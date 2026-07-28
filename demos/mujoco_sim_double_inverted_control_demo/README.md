# Double Inverted Pendulum Control Demo

This package runs a two-joint PD Controller through one asynchronous Protocol
V4 Adapter SDK session. The Controller requests the two actuator commands and
six scalar joint position, velocity, and actuator-force observations declared
by the model.

Build the package and its dependencies from the repository root:

```bash
colcon build --packages-up-to mujoco_sim_double_inverted_control_demo \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Start the Server and adapter in separate terminals:

```bash
mujoco-sim-server --config \
  install/mujoco_sim_double_inverted_control_demo/share/\
mujoco_sim_double_inverted_control_demo/config/double_inverted_server.yaml
```

```bash
mujoco-sim-double-inverted-control-demo --config \
  install/mujoco_sim_double_inverted_control_demo/share/\
mujoco_sim_double_inverted_control_demo/config/mujoco_sim_double_inverted_control_demo.yaml
```

Both commands use the `default` Server namespace. The Server starts from the
`random_unstable` keyframe, while the adapter drives both joint references
toward zero. Controller gains and the symmetric torque limit belong to the
adapter YAML; the Server YAML contains only strict Server fields.

The same executable and model are also reused by the dual-adapter demo without
adding another C++ Controller implementation.
