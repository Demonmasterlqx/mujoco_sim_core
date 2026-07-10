# mujoco_sim_core

## 包文档

| 包 | 作用 | 文档 |
|---|---|---|
| `mujoco_sim_protocol` | 定义 V1 Protobuf 协议和 eCAL 命名规则 | [查看](core/mujoco_sim_protocol/Readme.md) |
| `mujoco_sim_core` | 管理 MuJoCo 状态、资源所有权和 lockstep 仿真 | [查看](core/mujoco_sim_core/Readme.md) |
| `mujoco_sim_server` | 提供 eCAL server、配置解析和图形界面 | [查看](core/mujoco_sim_server/Readme.md) |
| `mujoco_sim_adapter_sdk` | 提供同步 controller adapter C++ SDK | [查看](core/mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md) |
| `mujoco_sim_sample_adapter` | 展示固定 command adapter 和端到端联调 | [查看](core/mujoco_sim_adapter/mujoco_sim_sample_adapter/Readme.md) |

## 构建方式

使用 colcon 作为构建工具。当前 V1 包均为纯 CMake 包：

server 配置解析和图形界面依赖系统提供的 CLI11、yaml-cpp、GLFW 和
libpng，可通过 rosdep 安装，对应 Ubuntu 包为 `libcli11-dev`、
`libyaml-cpp-dev`、`libglfw3-dev` 和 `libpng-dev`。

```bash
colcon list --topological-order
colcon build --packages-up-to mujoco_sim_server --cmake-args -DBUILD_TESTING=ON
colcon build --packages-up-to mujoco_sim_sample_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mujoco_sim_protocol mujoco_sim_core mujoco_sim_server mujoco_sim_adapter_sdk mujoco_sim_sample_adapter
colcon test-result --verbose
```

## 架构设计

以 `mujoco_sim_core` 为核心，MuJoCo server 掌管整个仿真进程，包括模型加载、仿真进度、仿真时间和 `mj_step()`。

V1 使用 eCAL 作为 IPC 方案。外部控制器通过 adapter 接入，不直接访问 `mjModel` 或 `mjData`。

外部控制器（ROS 2/RTOS/裸机/native C++ 等）通过 adapter 接入。

MuJoCo server 会加载模型，然后通过同步 Tick/Commit 协议与各控制器通信。

启动参数使用 CLI11 解析，配置文件使用 yaml-cpp 解析标准 YAML。配置优先级固定为
“命令行 > YAML > 程序默认值”，与 `--config` 在命令行中的位置无关。YAML 顶层必须是
单个 mapping，未知字段、重复字段、错误类型和整数溢出都会导致启动失败。

```yaml
server_name: default
model_path: robot.xml
max_ticks: 0
tick_timeout_ms: 1000
startup_timeout_ms: 5000
status_period_ms: 100
headless: false
```

使用 `mujoco-sim-server --config server.yaml` 加载配置；`--model`、
`--server-name`、`--max-ticks`、`--tick-timeout-ms` 和
`--startup-timeout-ms` 可以覆盖对应 YAML 字段。默认启动 MuJoCo 官方
`simulate` 图形界面；远程服务、CI 或无显示环境使用 `--headless`，该模式不会初始化
GLFW/OpenGL。若 YAML 设置了 `headless: true`，可用 `--gui` 在命令行恢复图形模式。

图形界面读取 Core 生成的隔离快照，支持官方相机旋转、平移、缩放、左右侧栏、
渲染选项、传感器和性能面板。UI 不持有也不会修改真实 `mjModel/mjData`，控制器所有权
和 Tick/Commit Barrier 行为在 GUI 与 headless 模式下完全一致。

## V1 运行示例

打开两个终端，并确保当前目录都是仓库根目录。终端 1 先启动 server：

```bash
source install/setup.bash

mujoco-sim-server \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 5
```

在默认 5 秒启动超时内，于终端 2 启动 adapter：

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
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 5
```

在 server 运行期间，可从第三个终端查询默认 server 的状态和模型资源：

```bash
source install/setup.bash

mujoco-sim-status
mujoco-sim-status --server-name default --watch --interval-ms 1000
mujoco-sim-status --server-name default --json
```

`mujoco-sim-status` 默认收到一条有效 `ServerStatus` 后退出；`--watch` 按指定间隔持续
输出，断线后报告 stale 并等待同名 server 恢复。`--json --watch` 输出 NDJSON，诊断信息
始终写入 stderr。

server 是唯一物理状态所有者。每个 Tick 中，adapter 接收 observation，提交完整 actuator command；server 收齐 Barrier 后写入 `data->ctrl` 并调用 `mj_step()`。
