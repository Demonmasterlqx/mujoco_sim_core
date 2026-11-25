# mujoco_sim_core

## 包文档

| 包 | 作用 | 文档 |
|---|---|---|
| `mujoco_sim_config` | 从 schema 在构建期生成 C++ 配置类型、严格 YAML/CLI 解析器 | [查看](core/mujoco_sim_config/README.zh-CN.md) |
| `mujoco_sim_protocol` | 定义唯一 Protocol V4 控制面、稠密数据面、Sensor framing 和 eCAL 命名规则 | [查看](core/mujoco_sim_protocol/Readme.md) |
| `mujoco_sim_core` | 管理 MuJoCo 状态、资源所有权和异步控制仿真 | [查看](core/mujoco_sim_core/Readme.md) |
| `mujoco_sim_sensor` | 从隔离 Snapshot 捕获 headless camera 与 360 度 ray LiDAR | [查看](core/mujoco_sim_sensor/README.md) |
| `mujoco_sim_server` | 提供 eCAL server、配置解析和图形界面 | [查看](core/mujoco_sim_server/Readme.md) |
| `mujoco_sim_adapter_sdk` | 提供异步 Controller、Observer 与 Sensor Stream C++ Client | [查看](core/mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md) |
| `mujoco_sim_hardware_interface` | 将 Protocol V4 Controller 映射为 ROS 2 Jazzy ros2_control system plugin | [查看](core/mujoco_sim_adapter/mujoco_sim_hardware_interface/Readme.md) |
| `mujoco_sim_double_inverted_control_demo` | 使用独立 Controller adapter 控制双倒立摆 | [查看](demos/mujoco_sim_double_inverted_control_demo/README.md) |
| `mujoco_sim_wbr_lqr_demo` | 迁移 WBR LQR 抡腿机器人控制器，提供可配置接口和完整闭环示例 | [查看](demos/mujoco_sim_wbr_lqr_demo/README.md) |
| `mujoco_sim_dual_adapter_demo` | 在同一个 Server 中同时运行 WBR LQR 与双倒立摆 Controller | [查看](demos/mujoco_sim_dual_adapter_demo/README.md) |
| `mujoco_sim_sensor_demo` | 四进程房间巡航、相机消费和 IMU/LiDAR ICP 建图示例 | [查看](demos/mujoco_sim_sensor_demo/README.md) |
| `mujoco_sim_hardware_interface_demo` | 使用标准 diff_drive/IMU broadcaster 的 ROS 2 差速车闭环示例 | [查看](demos/mujoco_sim_hardware_interface_demo/README.md) |

## 构建方式

使用 colcon 作为构建工具。仿真运行时包为纯 CMake 包；可选的 ROS 2 Control 集成包和
对应 demo 使用 ROS 2 Jazzy ament_cmake：

系统依赖统一通过 apt 安装，禁止使用 rosdep。安装前需要为 Ubuntu 24.04 配置可提供
`ecal` 和 `python3-colcon-common-extensions` 的 apt 软件源。配置生成依赖
Python 3/PyYAML，生成后的运行时依赖 CLI11 和 yaml-cpp；server 图形界面依赖
GLFW 和 libpng，传感器及 demo 还依赖 EGL、OSMesa、Eigen 和 PCL。

```bash
sudo apt update
sudo apt install --yes \
  binutils build-essential clang-format cmake doxygen ecal graphviz libcli11-dev \
  libegl1-mesa-dev libeigen3-dev \
  libglfw3-dev libgtest-dev libosmesa6-dev libpcl-dev libpng-dev \
  libprotobuf-dev libspdlog-dev libyaml-cpp-dev pkg-config protobuf-compiler \
  python3-colcon-common-extensions python3-venv python3-yaml xvfb
```

构建 ROS 2 Control 集成还需先配置 ROS 2 Jazzy apt 软件源，然后安装：

```bash
sudo apt install --yes \
  ros-jazzy-ament-cmake ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller ros-jazzy-hardware-interface \
  ros-jazzy-geometry-msgs \
  ros-jazzy-imu-sensor-broadcaster ros-jazzy-joint-state-broadcaster \
  ros-jazzy-nav-msgs ros-jazzy-pluginlib ros-jazzy-rclpy \
  ros-jazzy-realtime-tools ros-jazzy-robot-state-publisher ros-jazzy-sensor-msgs \
  ros-jazzy-rosgraph-msgs ros-jazzy-std-msgs ros-jazzy-tf2-msgs ros-jazzy-tf2-ros
source /opt/ros/jazzy/setup.bash
```

```bash
colcon list --base-paths core demos --topological-order
colcon build --base-paths core demos --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths core demos --packages-select \
  mujoco_sim_config mujoco_sim_protocol mujoco_sim_core mujoco_sim_sensor \
  mujoco_sim_server mujoco_sim_adapter_sdk mujoco_sim_wbr_lqr_demo \
  mujoco_sim_sensor_demo mujoco_sim_hardware_interface \
  mujoco_sim_hardware_interface_demo --event-handlers console_cohesion+
colcon test-result --all --verbose
```

## 架构设计

当前运行时契约见 [Protocol V4](doc/protocol/v4.md)，线程与边界设计见
[Server runtime](doc/architecture/server_runtime.md)、
[异步 Controller 时序](doc/architecture/async_controller_sequence.md) 和
[Controller 生命周期](doc/architecture/controller_lifecycle.md)。`doc/plans`、
`doc/refactor` 与 `doc/performance/v1.3` 是历史设计和基线证据，不是当前运行说明。

以 `mujoco_sim_core` 为核心，MuJoCo server 掌管整个仿真进程，包括模型加载、仿真进度、仿真时间和 `mj_step()`。

Protocol V4 使用 eCAL 作为 IPC 方案。外部控制器（ROS 2/RTOS/裸机/native C++ 等）
通过 adapter 接入，不直接访问 `mjModel` 或 `mjData`。

Server 按配置持续执行 Step 并发布最新 Controller State。`ControllerClient` 消费 State、
把最新 Command 写入 depth-1 mailbox；Server 在下一 Step 边界使用该 Command，没有新值时
保持末值。Controller 计算、断开或发送失败都不会阻塞仿真。只读小数据使用
`ObserverClient`；RGB、深度、分割和点云使用 `SensorStreamClient` 消费独立异步 raw stream。

各程序的配置类型和解析器由 `mujoco_sim_config` 根据包内 schema 在构建期生成。启动参数
使用 CLI11，配置文件使用 yaml-cpp 解析标准 YAML。配置优先级固定为
“命令行 > YAML > 程序默认值”，与 `--config` 在命令行中的位置无关。YAML 顶层必须是
单个 mapping，未知字段、重复字段、错误类型和整数溢出都会导致启动失败。

```yaml
server_name: default
model_path: robot.xml
initial_keyframe: ""
max_ticks: 0
pacing_mode: realtime
realtime_factor: 1.0
command_timeout_ms: 500
command_keepalive_ms: 100
offline_session_expire_ms: 0
controller_state_every_n_steps: 1
controller_state_period_s: 0.0
controller_state_max_rate_hz: 1000
startup_ctrl_policy: zero
offline_ctrl_policy: hold_last
error_ctrl_policy: zero
release_ctrl_policy: zero
controller_overrides: []
session_timeout_ms: 1000
activation_timeout_ms: 5000
observer_heartbeat_period_ms: 250
status_period_ms: 100
status_min_interval_ms: 20
observer_max_rate_hz: 500
allow_every_step_observer: false
transport_subscriber_sample_period_ms: 1000
sensor_subscriber_sample_period_ms: 200
viewer_sync_period_ms: 16
viewer_control_input_enabled: true
viewer_mouse_force_enabled: true
viewer_pause_run_enabled: true
viewer_reset_enabled: true
viewer_mouse_force_timeout_ms: 100
log_summary_period_ms: 1000
log_level: info
log_file: ""
log_file_max_size_mb: 10
log_file_max_files: 5
headless: false
render_backend: auto
sensor_streams: []
```

YAML 中的相对路径按配置文件所在目录解析，CLI 路径按进程当前目录解析。使用
`mujoco-sim-server --config server.yaml` 加载配置；`--model`、`--initial-keyframe`、
`--server-name`、`--max-ticks` 和 `--activation-timeout-ms` 可以覆盖对应 YAML 字段；
`--log-level`、`--log-file`、`--log-file-max-size-mb`、`--log-file-max-files` 及
`--log-summary-period-ms` 也有 CLI 覆盖。`--viewer-sync-period-ms` 覆盖 GUI 快照采集周期；
取值必须是 `[1, UINT32_MAX]` 内的毫秒数，默认 `16`。
`viewer_pause_run_enabled` 与 `viewer_reset_enabled` 仅可在 YAML 中配置，默认均为 `true`。
headless 模式仍严格解析全部 viewer 字段，但不创建 GUI，也不会产生本地 Pause/Run/Reset
动作。其余未声明 CLI flag 的运行策略必须写入 YAML。
默认启动 MuJoCo 官方
`simulate` 图形界面；远程服务、CI 或无显示环境使用 `--headless`，该模式不会初始化
GLFW viewer。若配置 camera Stream，sensor worker 仍会初始化 EGL 或 OSMesa offscreen
OpenGL context。若 YAML 设置了 `headless: true`，可用 `--gui` 在命令行恢复图形模式。

图形界面读取 Core 生成的隔离快照，支持官方相机旋转、平移、缩放、左右侧栏、
渲染选项、传感器和性能面板。仿真线程以非阻塞方式使用两个固定生产缓冲和 depth-1
latest 邮箱；锁竞争、缓冲耗尽或 GUI 变慢时允许跳过或替换显示帧。独立 GUI sync worker
可以等待 MuJoCo viewer mutex，但仿真步进不会等待它。UI 不持有也不直接修改真实
`mjModel/mjData`；白名单 GUI 输入经独立邮箱交给仿真线程应用，且不改变控制器所有权和
异步 State/Command 语义。

GUI 输入默认开启。Control Slider 与 `Clear all` 会在下一个 Step 边界产生一次性
actuator 写入；同一边界中 GUI 后于 Adapter 写入生效，但下一条 Adapter Command 可以
重新覆盖它。可先双击选择动态刚体，再用 `Ctrl + 右键拖动`施加
平移扰动力或用 `Ctrl + 左键拖动`施加旋转扰动力。松开鼠标会发布清除状态；若 Viewer
停止同步，Server 会按 `viewer_mouse_force_timeout_ms`（默认 `100` ms，最小 `1` ms）在下一
Step 前清除残留力。GUI 输入不取得 Controller 资源所有权，也不改变 Adapter liveness；
将对应输入字段显式设置为 `false` 可恢复纯观察行为。

GUI 的 Pause 只停止 `mj_step()`：RPC、Adapter liveness、状态发布、Sensor/Viewer 管理与
定时器仍会运行，`mujoco-sim-status --watch` 会显示 `PAUSED`，且 tick 和 simulation time
保持不变。Run 从点击时的 steady clock 重新开始 pacing，不会追赶暂停期间的墙钟时间。
Reset 将权威状态恢复到默认初始状态或 `initial_keyframe`，清零 control、鼠标力与旧
Controller Command，但保留当前 Run/Pause 模式、epoch、tick、session、layout 和资源
所有权；下一次 Step 的 tick 继续递增。Reset 后 `ONLINE` Controller 会显示为
`ACTIVE_NO_COMMAND`，并且必须发送 sequence 大于 Reset 前最后值的新 wire Command。
官方 SDK 的 keepalive 会继续以递增 sequence 发布最新值。`ERROR_PAUSED` 不能通过 Run
或 Reset 恢复。将 `viewer_reset_enabled` 设为 `false` 会显式禁用 Reset 按钮并阻止被动
viewer 副本发生本地 Reset。

## Protocol V4 运行示例

在仓库根目录启动 server：

```bash
source install/setup.bash

mujoco-sim-server \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

server 加载模型后立即进入 `RUNNING`，即使没有 adapter 也会按模型 timestep 实时推进。
Controller 集成方式和完整闭环示例见
[`mujoco_sim_wbr_lqr_demo`](demos/mujoco_sim_wbr_lqr_demo/README.md)；Sensor Stream、
Observer 和多进程控制示例见
[`mujoco_sim_sensor_demo`](demos/mujoco_sim_sensor_demo/README.md)。

无头运行只需增加：

```bash
mujoco-sim-server \
  --headless \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

server 使用 Ctrl-C 或关闭 GUI 结束。若配置非零 `max_ticks`，它统计 server 启动以来的
全部 `mj_step()`，包括 adapter 注册前和注销后的步进，因此不能把它当作某个 adapter
的 State 数。

在 server 运行期间，可从另一个终端查询默认 server 的状态和模型资源：

```bash
source install/setup.bash

mujoco-sim-status
mujoco-sim-status --server-name default --watch --interval-ms 1000
mujoco-sim-status --server-name default --json
```

`mujoco-sim-status` 默认收到一条有效 `ServerStatus` 后退出；统一 Adapter 表区分
Controller/Observer，并显示 Controller 的异步会话状态；Sensor Stream 表显示
subscriber、capture/publish/drop/error/blocking 指标。`--watch` 按
指定间隔持续输出，断线后报告 stale 并等待同名 server 恢复。`--json --watch` 输出
NDJSON，诊断信息始终写入 stderr。

Server 默认每 1000 ms 输出一次 `event=server_summary` 基本运行统计。字段
`realtime_factor_rate` 是相邻两次汇总之间的仿真时间增量除以 steady-clock wall time
增量，即 `delta simulation time / delta wall time`；它表示实际仿真推进倍率，不是配置的
目标值；`step_rate_hz` 是同一窗口的 `interval_steps / interval_wall_s`。
`target_realtime_factor` 单独记录 `realtime_factor` 配置；在 `unthrottled` 模式下实际倍率
可以远高于该值。通过 `log_summary_period_ms` 调整周期，设为 `0` 可完全关闭周期汇总。
GUI 模式还输出 `viewer_capture_attempts/submitted/skipped`、
`viewer_pending_replaced`、`viewer_sync_completed/failures`，以及 capture/sync 平均和最大
微秒耗时；headless 模式下这些字段均为零。

server 是唯一物理状态所有者。Controller 激活后从下一次 cadence 到期的 completed Step
接收首帧 State。超过 command timeout 后会进入 `OFFLINE`，但 Server 继续 Step 和发布 State；
下一条有效 Command 会立即恢复 `ONLINE`。非法、stale 或 duplicate Command 仅被拒绝，不刷新
liveness，也不会把会话置为 `ERROR`。

## WBR LQR Demo

WBR demo 使用安装后的相对路径配置，server 会从命名 keyframe `home` 开始。终端 1：

```bash
source install/setup.bash
mujoco-sim-server \
  --config install/mujoco_sim_wbr_lqr_demo/share/mujoco_sim_wbr_lqr_demo/config/wbr_server.yaml \
  --headless \
  --max-ticks 0
```

终端 2：

```bash
source install/setup.bash
mujoco-sim-wbr-lqr-adapter \
  --config install/mujoco_sim_wbr_lqr_demo/share/mujoco_sim_wbr_lqr_demo/config/wbr_lqr.yaml
```

随附 adapter YAML 的 `max_ticks: 0` 表示无限运行；这里也把 server 的全局 `max_ticks`
覆盖为 `0`，因此正常交互运行需由操作者分别结束 adapter 和 server。需要有限验收时，
使用 adapter 的 `--ticks` 设置本地 State 上限，不要把 server 的启动总 step 上限
当作 adapter 预算。21 个 state resource 和 6 个 command resource 均可在 adapter YAML
中改名；控制器仍按六个固定语义槽位执行上游 LQR/FSM 逻辑。
默认脚本会在第 1 个 Controller tick 使能底盘；可继续配置 Q/E/F 的确定性区间来选择
腿长目标。控制主体固定到 `wbr_mujoco@1d895574`，另有下述固定速度/偏航率的显式
本地映射补丁；输入快照语义对齐
`mujoco_interface@a2501454`；包内 6000-Step MuJoCo 闭环测试验证最终持续处于
`NORMAL`、机身抬升、俯仰受控且水平漂移有界。

需要实时键盘控制时，选择标准输入终端模式：

```bash
mujoco-sim-wbr-lqr-adapter \
  --config install/mujoco_sim_wbr_lqr_demo/share/mujoco_sim_wbr_lqr_demo/config/wbr_lqr.yaml \
  --input-mode cin \
  --cin-hold-ticks 200
```

每个 W/A/S/D 字符会作用于可配置的 `input.cin.hold_ticks` 次 Controller update
（默认 200 次，1 kHz 控制周期下约 200 ms），按住时终端自动重复会刷新对应方向。
W/S 分别给出 `+2/-2 m/s`，A/D 分别给出 `+1/-1 rad/ms`（即 `+1000/-1000 rad/s`）；
相反方向同时有效时输入为中性。Q/E/F 调整腿长，Space 切换底盘，X 清除运动计时，
Esc 安全退出。cin 模式要求前台交互式 TTY，输入跟随终端焦点，不读取系统全局键盘事件。

## Dual Adapter Demo

组合 Demo 在一个 Server 中加载 WBR 和双倒立摆，并启动两个资源互不重叠的
Controller adapter。该包不包含新的 C++ 实现：

```bash
source install/setup.bash
mujoco-sim-dual-adapter-demo-launch \
  --mode headless \
  --wbr-ticks 6000 \
  --timeout-s 45 \
  --output-dir /tmp/mujoco_sim_dual_adapter
```

省略 `--wbr-ticks` 和 `--timeout-s` 时持续运行，直到操作者中断；使用 `--mode gui`
可显示两个模型共享的场景。启动器负责建立隔离 Server namespace、启动两个 adapter、
监控提前退出，并按有界顺序停止剩余进程。详细配置和测试方式见
[`mujoco_sim_dual_adapter_demo`](demos/mujoco_sim_dual_adapter_demo/README.md)。

## Sensor / Mapping Demo

该 Demo 启动一个 Server 和四个独立 eCAL 进程，完成房间周界一圈控制、异步状态
Observer、四类 camera 流消费，以及 16 线 LiDAR + IMU 的 PCL ICP 建图：

```bash
source install/setup.bash
mujoco-sim-sensor-demo-launch \
  --server-config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml \
  --demo-config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml \
  --output-dir /tmp/mujoco_sensor_demo \
  --mode headless
```

`--mode gui` 在保持 offscreen sensor streams 的同时打开 MuJoCo viewer。正常完成条件是
Controller 检测到一圈闭环，不依赖 Server `max_ticks`。输出目录包含 `map.pcd`、
`trajectory.csv`、`map_top_down.png`、`front_camera.png` 和各进程日志。
