# mujoco_sim_hardware_interface_demo

This ROS 2 Jazzy demo runs a two-wheel differential-drive robot through
`mujoco_sim_hardware_interface`. The MuJoCo Server remains the only physics
owner; `ros2_control_node` loads the Protocol V4 hardware plugin and standard
joint, IMU, and differential-drive controllers.

The URDF intentionally contains no `state_interface` or `command_interface`
elements. Joint, IMU, and grouped sensor interfaces are generated from the
mapping parameters. Debug output includes scalar wheel topics, one grouped IMU
array, one grouped position array, `/clock`, and a direct `world -> mujoco_imu_pose`
transform.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mujoco_sim_hardware_interface_demo diff_drive.launch.py
```

The launcher waits for the controller subscriber and publishes a finite
three-second stamped velocity command. To drive the robot manually instead,
disable the bounded command generator:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mujoco_sim_hardware_interface_demo diff_drive.launch.py auto_command:=false
```

Then, in another terminal:

```bash
ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.4}, angular: {z: 0.2}}}"
```

Useful outputs are `/diff_drive_controller/odom`, `/imu_sensor_broadcaster/imu`,
`/clock`, `/tf`, and the hardware component node's private `debug` topics.

The controller manager intentionally keeps `use_sim_time: false`. Its hardware
`read()` callback produces `/clock`; making that same update loop wait for
simulation time would create a startup cycle. Other consumers can still use the
published simulation clock.

Shutdown is ordered: the Server ignores the launch service's first SIGINT,
`ros2_control_node` stops its controllers and unregisters the Adapter, and only
then does the launch event handler terminate the Server. The E2E test retains
bounded SIGINT, SIGTERM, and SIGKILL fallbacks for unexpected failures.

## Build and automated E2E

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths core demos --symlink-install \
  --packages-up-to mujoco_sim_hardware_interface_demo \
  --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths core demos --packages-select \
  mujoco_sim_hardware_interface mujoco_sim_hardware_interface_demo \
  --event-handlers console_cohesion+
colcon test-result --all --verbose
```

The headless E2E sends a finite-duration `cmd_vel` and checks monotonic `/clock`,
wheel state, forward odometry, the Server command range, direct TF, a three-value
generic sensor array, and the thirteen-value IMU array in the documented order.
