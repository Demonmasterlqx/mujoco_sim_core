# mujoco_sim_core

## 包文档

| 包 | 作用 | 文档 |
|---|---|---|
| `mujoco_sim_config` | 从 schema 在构建期生成 C++ 配置类型、严格 YAML/CLI 解析器 | [查看](core/mujoco_sim_config/README.md) |
| `mujoco_sim_protocol` | 定义唯一 Protocol V3 控制面、稠密数据面、Sensor framing 和 eCAL 命名规则 | [查看](core/mujoco_sim_protocol/Readme.md) |
| `mujoco_sim_core` | 管理 MuJoCo 状态、资源所有权和 lockstep 仿真 | [查看](core/mujoco_sim_core/Readme.md) |
| `mujoco_sim_sensor` | 从隔离 Snapshot 捕获 headless camera 与 360 度 ray LiDAR | [查看](core/mujoco_sim_sensor/README.md) |
| `mujoco_sim_server` | 提供 eCAL server、配置解析和图形界面 | [查看](core/mujoco_sim_server/Readme.md) |
| `mujoco_sim_adapter_sdk` | 提供同步 Controller、非 Barrier Observer 与异步 Stream C++ Client | [查看](core/mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md) |
| `mujoco_sim_sample_adapter` | 展示固定 command adapter 和端到端联调 | [查看](core/mujoco_sim_adapter/mujoco_sim_sample_adapter/Readme.md) |
| `mujoco_sim_wbr_lqr_demo` | 迁移 WBR LQR 抡腿机器人控制器，提供可配置接口和完整闭环示例 | [查看](demos/mujoco_sim_wbr_lqr_demo/README.md) |
| `mujoco_sim_sensor_demo` | 四进程房间巡航、相机消费和 IMU/LiDAR ICP 建图示例 | [查看](demos/mujoco_sim_sensor_demo/README.md) |

## 构建方式

使用 colcon 作为构建工具。所有包均为纯 CMake 包：

配置生成依赖 Python 3/PyYAML，生成后的运行时依赖 CLI11 和 yaml-cpp；server 图形界面
依赖 GLFW 和 libpng。可通过 rosdep 安装，对应 Ubuntu 包包括 `python3-yaml`、
`libcli11-dev`、`libyaml-cpp-dev`、`libglfw3-dev`、`libpng-dev`、EGL/OSMesa 和 PCL。

```bash
rosdep install --from-paths core demos --ignore-src -r -y
```

```bash
colcon list --topological-order
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mujoco_sim_config mujoco_sim_protocol mujoco_sim_core mujoco_sim_sensor mujoco_sim_server mujoco_sim_adapter_sdk mujoco_sim_sample_adapter mujoco_sim_wbr_lqr_demo mujoco_sim_sensor_demo
colcon test-result --verbose
```

## 架构设计

以 `mujoco_sim_core` 为核心，MuJoCo server 掌管整个仿真进程，包括模型加载、仿真进度、仿真时间和 `mj_step()`。

Protocol V3 使用 eCAL 作为 IPC 方案。外部控制器通过 adapter 接入，不直接访问 `mjModel` 或 `mjData`。

外部控制器（ROS 2/RTOS/裸机/native C++ 等）通过 adapter 接入。

MuJoCo server 会加载模型，然后通过同步 Tick/Commit 协议与各控制器通信。

同步 Controller 的既有行为保持不变：`SyncAdapterClient` 仍在每个 Tick 中原子获得其声明的
关节/IMU 等小 observation，计算后 Commit 才能完成 Barrier。只读小数据使用
`ObserverClient`，它在 completed step 后按步数或仿真时间降采样，不参与 Barrier；RGB、
深度、分割和点云使用 `SensorStreamClient` 消费独立异步 raw stream。

各程序的配置类型和解析器由 `mujoco_sim_config` 根据包内 schema 在构建期生成。启动参数
使用 CLI11，配置文件使用 yaml-cpp 解析标准 YAML。配置优先级固定为
“命令行 > YAML > 程序默认值”，与 `--config` 在命令行中的位置无关。YAML 顶层必须是
单个 mapping，未知字段、重复字段、错误类型和整数溢出都会导致启动失败。

```yaml
server_name: default
model_path: robot.xml
initial_keyframe: ""
max_ticks: 0
tick_timeout_ms: 1000
session_timeout_ms: 1000
activation_timeout_ms: 5000
controller_heartbeat_period_ms: 250
observer_heartbeat_period_ms: 250
tick_retry_initial_ms: 5
tick_retry_backoff_multiplier: 2.0
tick_retry_max_delay_ms: 20
tick_retry_max_attempts: 3
allow_timed_out_adapter_replacement: true
status_period_ms: 100
status_min_interval_ms: 20
observer_max_rate_hz: 500
allow_every_step_observer: false
transport_subscriber_sample_period_ms: 1000
sensor_subscriber_sample_period_ms: 200
log_summary_period_ms: 1000
headless: false
render_backend: auto
sensor_streams: []
```

YAML 中的相对路径按配置文件所在目录解析，CLI 路径按进程当前目录解析。使用
`mujoco-sim-server --config server.yaml` 加载配置；`--model`、`--initial-keyframe`、
`--server-name`、`--max-ticks`、`--tick-timeout-ms`、`--session-timeout-ms`、`--activation-timeout-ms` 和
同名超时 adapter 替换开关可以覆盖对应 YAML 字段。默认启动 MuJoCo 官方
`simulate` 图形界面；远程服务、CI 或无显示环境使用 `--headless`，该模式不会初始化
GLFW viewer。若配置 camera Stream，sensor worker 仍会初始化 EGL 或 OSMesa offscreen
OpenGL context。若 YAML 设置了 `headless: true`，可用 `--gui` 在命令行恢复图形模式。

图形界面读取 Core 生成的隔离快照，支持官方相机旋转、平移、缩放、左右侧栏、
渲染选项、传感器和性能面板。UI 不持有也不会修改真实 `mjModel/mjData`，控制器所有权
和 Tick/Commit Barrier 行为在 GUI 与 headless 模式下完全一致。

## Protocol V3 运行示例

打开两个终端，并确保当前目录都是仓库根目录。终端 1 先启动 server：

```bash
source install/setup.bash

mujoco-sim-server \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

server 加载模型后立即进入 `RUNNING`，即使没有 adapter 也会按模型 timestep 实时推进。
可以在运行期间任意时刻于终端 2 注册并启动 adapter：

```bash
source install/setup.bash

mujoco-sim-sample-adapter \
  --ticks 5 \
  --command-value 0.1
```

无头运行只需增加：

```bash
mujoco-sim-server \
  --headless \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

adapter 处理 5 个 Tick 后会尽力注销并退出，server 继续自主运行，使用 Ctrl-C 或关闭 GUI
结束 server。若配置非零 `max_ticks`，它统计 server 启动以来的全部 `mj_step()`，包括 adapter
注册前、注销后以及空 Barrier 的步进，因此不能把它当作某个 adapter 的处理 Tick 数。

在 server 运行期间，可从第三个终端查询默认 server 的状态和模型资源：

```bash
source install/setup.bash

mujoco-sim-status
mujoco-sim-status --server-name default --watch --interval-ms 1000
mujoco-sim-status --server-name default --json
```

`mujoco-sim-status` 默认收到一条有效 `ServerStatus` 后退出；统一 Adapter 表区分
Controller/Observer，并保留 Controller 的 Commit/Barrier 诊断；Sensor Stream 表显示
subscriber、capture/publish/drop/error/blocking 指标。`--watch` 按
指定间隔持续输出，断线后报告 stale 并等待同名 server 恢复。`--json --watch` 输出
NDJSON，诊断信息始终写入 stderr。

server 是唯一物理状态所有者。每个 Tick 打开时固定该 Tick 的 Barrier 集合；运行期新激活的
adapter 从下一 Tick 加入。某个 adapter 超时或提交非法命令只隔离该 controller、清零其
actuator 并让其余 Barrier 继续，不再把整个 server 置为错误暂停。server 会继续为
`TIMED_OUT` adapter 生成 Tick 并尝试发布；一份合法 Commit 只作为恢复证明，它从下一 Tick
才重新成为 `ACTIVE` Barrier 成员。

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
使用 adapter 的 `--ticks` 设置本地 `ACTIVE` Tick 上限，不要把 server 的启动总 step 上限
当作 adapter 预算。21 个 state resource 和 6 个 command resource 均可在 adapter YAML
中改名；控制器仍按六个固定语义槽位执行上游 LQR/FSM 逻辑。

## Sensor / Mapping Demo

该 Demo 启动一个 Server 和四个独立 eCAL 进程，完成房间周界一圈控制、非 Barrier 状态
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
