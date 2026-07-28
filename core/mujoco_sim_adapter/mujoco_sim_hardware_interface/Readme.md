# mujoco_sim_hardware_interface

这是面向 Ubuntu 24.04、ROS 2 Jazzy 和 ros2_control 4.x 的 Protocol V4
硬件插件。插件通过
[`mujoco_sim_adapter_sdk`](../mujoco_sim_adapter_sdk/Readme.md) 的
`ControllerClient` 注册观测与执行器资源，不链接或访问 MuJoCo。

插件名为：

```xml
<plugin>mujoco_sim_hardware_interface/MujocoSimSystem</plugin>
```

## 生命周期与数据流

- `on_init()` 严格解析参数、检查资源冲突并生成 ROS state/command interface，不连接
  Server。
- `on_configure()` 创建可选实时 publisher，不注册远端 session。
- `on_activate()` 新建单次使用的 `ControllerClient`，执行 Start、布局校验、零指令限幅
  和 Activate。
- `read()` 非阻塞读取最新完整 State；没有新帧时保持末值并返回 OK。新帧只有在所有值、
  维度和仿真时间均有效后才整体更新 ROS state interface。
- `write()` 收集全部 ROS command，拒绝 NaN/Inf，按 Server 注册范围限幅，然后调用一次
  `SetCommand()`。该调用只更新 SDK depth-1 latest mailbox，不等待传输或仿真步。
- `on_deactivate()` 先停止本地 command/state 通信，再注销并销毁当前 session；注销失败时
  保留已停止的 client 供清理路径重试。下一次成功激活创建新 session，因此支持重复
  activate/deactivate。

插件要求至少配置一个 command actuator。只读传感器进程应使用 SDK
`ObserverClient`，不能注册成 Controller。

`read()` 与 `write()` 由同一个 ros2_control update loop 串行调用；本插件不支持两个
回调并发执行。生命周期转换也必须在 update loop 停止后进行。类本身直接持有 SDK
client、资源集合、句柄缓存与稠密缓冲区，不使用隐藏实现对象。

### 职责边界

`MujocoSimSystem` 只编排 ros2_control 生命周期和 handles、ControllerClient 的
注册/激活/State/Command 事务，以及可选 `/clock`。它不知道各类资源的 debug 消息类型、
Topic、数组顺序或 TF payload。

每个 `ResourceInterface` 派生类完整拥有自己的 ROS 输出：joint 管理各标量 debug
publisher，通用 sensor 管理整组数组 publisher，IMU 管理整组数组 publisher 和可选动态
TF publisher。资源负责创建、预分配消息、发布与销毁这些 publisher。主类只在生命周期
边界要求资源 configure/reset，并在整帧 State 已写入 ROS handles 后发送 commit 事件、在
整帧 Command 已被 SDK 接受并完成限幅值回写后发送 accepted 事件。`StageState()` 与
`StageCommand()` 只校验和填充预分配缓冲区，不允许发布，因此失败帧不会产生任何 debug
或 TF 副作用。

## 公共硬件参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `server_id` | `default` | Protocol V4 Server 标识符 |
| `adapter_id` | 无 | 必填 Controller 标识符 |
| `server_time_out_ms` | `5000` | 正整数，SDK 服务发现与 RPC 超时 |
| `publish_sim_time` | `false` | 每个新 State 通过 realtime_tools 发布 `/clock` |
| `debug` | `false` | 全局调试开关，可由组件局部参数覆盖 |

只有 `debug` 允许作为 joint/sensor 局部参数覆盖硬件级默认值。`server_id`、`adapter_id`、
`server_time_out_ms` 和 `publish_sim_time` 描述单一 System/SDK 会话；若写入组件参数，
`on_init()` 会直接拒绝，避免出现局部配置被静默忽略的情况。

组件配置会继承硬件参数，组件同名参数优先。参数值采用严格格式：布尔值只能是
`true`/`false`，整数不能包含尾随字符或溢出。未知字段会使初始化失败。
每个 sensor 都必须显式配置 `sensor_type`：六轴 IMU 使用 `imu`，当前其他所有 sensor
统一使用 `other`；插件不会再根据其余参数推断类型。

## 自动导出的接口

映射参数是唯一接口来源。Jazzy unlisted-interface API 根据映射自动导出接口；不要再写
`<state_interface>` 或 `<command_interface>`，否则插件会显式拒绝重复契约。

### Joint

```xml
<joint name="left_wheel_joint">
  <param name="actuator_velocity">left_wheel_actuator</param>
  <param name="sensor_position">left_wheel_position</param>
  <param name="sensor_velocity">left_wheel_velocity</param>
</joint>
```

支持以下标量映射，注册资源维度必须为 1：

| 参数 | ROS 接口 |
|---|---|
| `actuator_position` | command `<joint>/position` |
| `actuator_velocity` | command `<joint>/velocity` |
| `actuator_effort` | command `<joint>/effort` |
| `sensor_position` | state `<joint>/position` |
| `sensor_velocity` | state `<joint>/velocity` |
| `sensor_effort` | state `<joint>/effort` |

command interface 在每次激活时初始化为 0；若 Server 返回控制范围，则先限幅再保存和
提交。

### 六轴 IMU 与直接 TF

```xml
<sensor name="imu_sensor">
  <param name="sensor_type">imu</param>
  <param name="target_link">imu_link</param>
  <param name="father_link">world</param>
  <param name="accelerometer">imu_acceleration</param>
  <param name="gyro">imu_angular_velocity</param>
  <param name="orientation">imu_orientation</param>
  <param name="position">imu_position</param>
</sensor>
```

`accelerometer` 和 `gyro` 必填且维度均为 3，导出
`linear_acceleration.x/y/z` 和 `angular_velocity.x/y/z`。`orientation` 与
`position` 必须同时配置或同时省略；其维度分别为 4 和 3。MuJoCo 四元数按
`w,x,y,z` 输入，插件校验并归一化后导出 ROS `x,y,z,w` 接口。配置直接位姿时，插件
还通过 realtime_tools 发布 `father_link -> target_link` 的 `/tf`；六轴数据本身不会被
积分成位姿。

### 通用多维 Sensor

```xml
<sensor name="force_sensor">
  <param name="sensor_type">other</param>
  <param name="sensor_name">contact_force</param>
  <param name="sensor_dim">3</param>
  <param name="sensor_dim_names">x y z</param>
</sensor>
```

`sensor_dim_names` 是空格分隔的非空唯一名称，数量必须等于 `sensor_dim`；插件导出
`force_sensor/x`、`force_sensor/y` 和 `force_sensor/z`。

### 完整 ros2_control 片段

以下片段同时展示 joint、IMU 与通用多维 sensor；其中没有显式
`state_interface`/`command_interface`：

```xml
<ros2_control name="mujoco_sim_diff_drive" type="system">
  <hardware>
    <plugin>mujoco_sim_hardware_interface/MujocoSimSystem</plugin>
    <param name="server_id">ros2_control_demo</param>
    <param name="adapter_id">ros2_control</param>
    <param name="server_time_out_ms">10000</param>
    <param name="publish_sim_time">true</param>
    <param name="debug">true</param>
  </hardware>
  <joint name="left_wheel_joint">
    <param name="actuator_velocity">left_wheel_actuator</param>
    <param name="sensor_position">left_wheel_position</param>
    <param name="sensor_velocity">left_wheel_velocity</param>
  </joint>
  <sensor name="imu_sensor">
    <param name="sensor_type">imu</param>
    <param name="target_link">mujoco_imu_pose</param>
    <param name="father_link">world</param>
    <param name="accelerometer">imu_acceleration</param>
    <param name="gyro">imu_angular_velocity</param>
    <param name="orientation">imu_orientation</param>
    <param name="position">imu_position</param>
  </sensor>
  <sensor name="base_position_sensor">
    <param name="sensor_type">other</param>
    <param name="sensor_name">base_position_debug</param>
    <param name="sensor_dim">3</param>
    <param name="sensor_dim_names">x y z</param>
  </sensor>
</ros2_control>
```

## 调试 Topic

所有调试 publisher 都使用 `realtime_tools::RealtimePublisher` 和 depth-1 SensorData
QoS。publisher 忙时丢弃调试样本，不影响 State/Command：

- joint 标量：`~/debug/joints/<joint>/state/<interface>` 和
  `~/debug/joints/<joint>/command/<interface>`，消息为 `std_msgs/msg/Float64`；
- 通用 sensor：`~/debug/sensors/<sensor>`，一个 `Float64MultiArray`，顺序等于
  `sensor_dim_names`；
- IMU：`~/debug/imus/<imu>`，一个 `Float64MultiArray`，顺序为 acceleration xyz、
  gyro xyz、可选 orientation xyzw、position xyz。

通用 sensor 与 IMU 均是“每个 ROS sensor 组件整组一个数组”，不会为每个维度拆分
调试消息。`/clock` 使用 ClockQoS，由 `MujocoSimSystem` 独占管理；`/tf` 使用动态 TF
QoS，由配置了直接位姿的 `ImuResource` 管理。二者仅在新 State sequence 完整提交后
发布。调试 publisher 忙时直接丢弃该样本。

## `/clock` 启动约束

插件的 `read()` 是 `/clock` 生产者，因此运行它的 controller manager 必须以 wall/steady
clock 驱动，即 `controller_manager.ros__parameters.use_sim_time: false`。如果 controller
manager 自身等待仿真时间，控制循环会等待 `/clock`，而 `/clock` 又等待控制循环，形成
启动环。其他消费仿真时间的节点仍可按需设置 `use_sim_time: true`。

## 完整示例

可运行的双轮差速车示例位于
[`mujoco_sim_hardware_interface_demo`](../../../demos/mujoco_sim_hardware_interface_demo/README.md)，
包含 MJCF、URDF、标准 broadcaster、`diff_drive_controller` 和 launch 文件。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths core demos --symlink-install \
  --packages-up-to mujoco_sim_hardware_interface \
  --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths core demos --packages-select mujoco_sim_hardware_interface \
  --event-handlers console_cohesion+
colcon test-result --all --verbose
cmake --build build/mujoco_sim_hardware_interface \
  --target mujoco_sim_hardware_interface_docs
```

真实 Server、标准 controller、`/clock`、TF、里程计、命令范围和整组数组的有界 E2E
位于 demo 包。运行：

```bash
colcon test --base-paths core demos --packages-select mujoco_sim_hardware_interface_demo \
  --event-handlers console_cohesion+
```
