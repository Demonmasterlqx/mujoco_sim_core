# mujoco_sim_sample_adapter

`mujoco_sim_sample_adapter` 是
[mujoco_sim_adapter_sdk](../mujoco_sim_adapter_sdk/Readme.md) 的可运行参考实现。它注册
指定 observation/command，对每个 Tick layout 中的全部 command actuator 提交同一个
常量值，用于验证 controller 注册和同步 Tick/Commit 链路。

## 职责与边界

本包当前提供：

- 可执行程序 `mujoco-sim-sample-adapter`，负责解析参数并驱动 SDK 的显式
  `Start()`/`Activate()`/`WaitForTick()`/`Commit()` 生命周期。
- 可复用的参数解析和固定 command 辅助库。
- headless 与 GUI server 的五 Tick 端到端测试。

固定值策略不读取 observation 数值；它仅遍历服务端放入当前 Tick `layout.commands` 的
descriptor，并为每个名称生成一个 `command_value`。本包不加载模型、不运行 server、
不拥有 MuJoCo 状态，也不是通用控制算法或资源发现工具。

## 默认资源与行为

默认资源与仓库中的
[`simple_actuator.xml`](../../mujoco_sim_core/test/testdata/simple_actuator.xml)
匹配：

| 类型 | 默认名称 |
|---|---|
| observations | `hinge_pos`, `hinge_vel` |
| commands | `hinge_motor` |

默认 controller ID 为 `sample_controller`，默认向 `default` server 注册。每个 Tick 对
`hinge_motor` 提交 `0.0`。默认不限制处理 Tick 数，因此不带参数运行时不会自行成功退出。

## 依赖与构建

生产代码要求 CMake 3.22、C++20、`mujoco_sim_adapter_sdk` 和
`mujoco_sim_config`；eCAL、协议、Protobuf、CLI11 和 yaml-cpp 由依赖 target 传递，配置
代码在构建期由 Python/PyYAML 生成。生产程序不直接依赖 MuJoCo。启用测试后还需要 GTest、
`mujoco_sim_server`，以及在无 `DISPLAY` 环境中用于 GUI E2E 的 `xvfb-run`。Server 构建
还需要 CLI11、yaml-cpp、GLFW 和 libpng，详见[仓库 README](../../../Readme.md)。

以下命令均从仓库根目录执行：

```bash
colcon build \
  --packages-up-to mujoco_sim_sample_adapter \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

只构建 sample 库与程序时可将 `BUILD_TESTING` 设为 `OFF`；此时不会配置三个测试。

## 命令行参数

主程序使用 schema 生成的 CLI11 解析器。解析是事务性的，错误不会部分修改
`SampleOptions`；`--help` 文本也由同一 schema 生成。

| 参数 | 默认值 | 当前含义 |
|---|---:|---|
| `-h`, `--help` | - | 输出用法并以 `0` 退出，不连接 server。 |
| `--config PATH` | - | 加载严格 YAML 配置；CLI 显式值覆盖 YAML。 |
| `--server-name NAME` | `default` | registry 和 topic namespace；必须与 server 一致。 |
| `--adapter-id ID` | `sample_controller` | controller 唯一 ID；不能为空。 |
| `--observations a,b` | `hinge_pos,hinge_vel` | 逗号分隔的唯一 observation 名称；空列表允许。 |
| `--commands a,b` | `hinge_motor` | 逗号分隔的唯一 actuator 名称；最终必须至少有一个名称。 |
| `--ticks N` | `0` | 对 N 个已交付 `ACTIVE` Tick 完成本地 Commit 后退出；`0` 表示无限等待。 |
| `--command-value VALUE` | `0.0` | 为 Tick layout 中每个 command 提交的 `double` 标量。 |
| `--timeout-ms N` | `1000` | 注册给 server 的每 Tick Commit 等待上限；`0` 表示使用 server 默认 timeout。 |
| `--service-timeout-ms N` | `5000` | 注册和激活各自等待 registry service 的总上限，单位毫秒。 |

未知/重复参数、缺少值、负无符号数、整数溢出以及 `NaN`/`Inf` command 值都会失败。
`--ticks` 接受 `uint64`，两个 timeout 接受 `uint32`，不会发生静默截断。

`--timeout-ms` 在 controller 激活后约束每个 required Tick 的 Commit 等待时间；
`--service-timeout-ms` 只控制 Adapter 调用 registry service 的等待时间。server 的
`activation_timeout_ms` 约束注册后的 `PROVISIONAL` session 必须多快完成 Activate。

## YAML 配置

YAML 顶层支持 `server_name`、`adapter_id`、`observation_names`、`command_names`、
`max_ticks`、`command_value`、`timeout_ms` 和 `service_timeout_ms`。例如：

```yaml
server_name: demo
adapter_id: sample_controller
observation_names: [hinge_pos, hinge_vel]
command_names: [hinge_motor]
max_ticks: 5
command_value: 0.1
timeout_ms: 1000
service_timeout_ms: 5000
```

加载方式：

```bash
mujoco-sim-sample-adapter --config sample.yaml --ticks 10
```

优先级固定为“CLI > YAML > 默认值”，与 `--config` 的 argv 位置无关。解析器拒绝未知或
重复字段、错误类型、重复资源名、非有限数值和整数溢出。

## 公开 Target 与 API

安装包导出：

| Target/程序 | 用途 |
|---|---|
| `mujoco_sim::sample_adapter` | 公开 C++ 辅助库。 |
| `mujoco_sim::mujoco-sim-sample-adapter` | 安装后导出的 CMake executable target。 |
| `mujoco-sim-sample-adapter` | 安装到 `bin` 的命令。 |

使用辅助库：

```cmake
find_package(mujoco_sim_sample_adapter CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE mujoco_sim::sample_adapter)
```

公开头文件
[`<mujoco_sim/sample_adapter/sample_adapter.hpp>`](include/mujoco_sim/sample_adapter/sample_adapter.hpp)
提供：

| API | 当前行为 |
|---|---|
| `SampleOptions` | 组合 SDK `AdapterOptions`、`max_ticks` 和 `command_value`。 |
| `ParseNameList()` | 拆分逗号列表、去除条目首尾空白并丢弃空条目。 |
| `ConstantCommandsForTick()` | 按 Tick command layout 生成完整的同值 `CommandValue` 列表。 |
| `ParseCommandLine()` | 通过生成式 YAML/CLI 解析器合并配置；兼容保留调用方预置的 `max_ticks`、`command_value` 和 `service_timeout_ms`。 |
| `Usage()` | 返回 schema 生成的完整 CLI11 help。 |

## 单独启动 Adapter

构建并加载环境后，可直接启动 sample 程序：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name default \
  --adapter-id sample_controller \
  --ticks 5 \
  --command-value 0.1
```

这里的“单独启动”只表示直接运行 Adapter 可执行程序；它仍要求同一 eCAL 网络中已有名为
`default` 的 server。若没有可发现的 registry service，注册会在默认 5000 ms 后失败。

## GUI Server 双终端联调

两个终端都从仓库根目录启动并各自执行 `source install/setup.bash`。先启动终端 1；
`--gui` 会明确选择图形模式：

```bash
source install/setup.bash
mujoco-sim-server \
  --gui \
  --server-name gui_demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 0 \
  --tick-timeout-ms 1000
```

在 server 运行期间启动终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name gui_demo \
  --adapter-id gui_sample \
  --ticks 5 \
  --command-value 0.1 \
  --timeout-ms 1000 \
  --service-timeout-ms 5000
```

Adapter 处理并提交 5 个不同的 `ACTIVE` Tick 后显式注销并返回 `0`；server 继续按模型
timestep 自主运行，关闭 viewer 结束进程。

## Headless Server 双终端联调

无显示环境使用 `--headless`。仍然先启动终端 1：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name headless_demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 0 \
  --tick-timeout-ms 1000
```

再启动终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name headless_demo \
  --adapter-id headless_sample \
  --ticks 5 \
  --command-value 0.1 \
  --timeout-ms 1000 \
  --service-timeout-ms 5000
```

GUI 与 headless 模式使用同一 registry 和 Tick/Commit 路径；差别只在 server 是否创建图形
viewer。Adapter 退出后用 Ctrl-C 结束这个无限运行的 headless server。

## 自定义资源示例

测试模型还提供 `hinge_force` observation。下面使用它替换默认 observation 列表，展示
实际的自定义资源注册。终端 1：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name force_demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 0
```

终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name force_demo \
  --adapter-id force_sample \
  --observations hinge_force \
  --commands hinge_motor \
  --ticks 3 \
  --command-value -0.2
```

换用自定义 MJCF 时，将 `--model` 替换为从仓库根目录可达的相对路径，并把
`--observations`/`--commands` 改为该模型实际导出的名称。Sample 不会根据模型自动改写
默认资源。

## 运行前提、退出与失败

成功联调需满足：

- 已构建并加载 install 环境；server 先启动且两端 `server_name` 完全一致。
- `adapter_id` 非空且未注册；资源名存在、列表中无重复项，每个 command actuator
  尚未被其他 controller 认领。
- server 支持运行期注册、激活和注销；注册后必须在 server 的 `activation_timeout_ms` 内
  调用 Activate。新激活的 Adapter 从下一 Tick 加入固定 Barrier。
- `command_value` 必须是有限值，并处于 actuator control range 内。示例模型的
  `hinge_motor` 范围是 `[-1, 1]`。
- 业务循环必须在 controller timeout 内计算并提交完整 command。Sample 自动覆盖 Tick
  layout 中的全部 command，但 server timeout 仍应留出通信和调度余量。

可执行程序的退出行为：

| 返回值 | 条件 |
|---:|---|
| `0` | `-h/--help`；或 `--ticks N` 中的 `N > 0` 且已在本地 Commit N 个 `ACTIVE` Tick。 |
| `1` | CLI 解析失败、SDK `Start()` 失败、激活失败，或 eCAL 在未请求 Stop 时结束。 |

`--ticks` 只统计 SDK 本地接受 Commit 的 `ACTIVE` Tick；这表示 payload 已校验、缓存并尝试
发布，不是 Server 同步确认。`TIMED_OUT` 恢复 Tick 仍会由显式业务循环处理并提交，但不计
入进度。Sample 通过 `WaitForTick()` 接收匹配 session 的 `ERROR` 或
replacement 诊断并以 `1` 退出。正常达到 Tick 数后程序显式注销；析构仍提供尽力注销后备。
注销失败单独打印，但不覆盖已经确定的主返回值。

`--ticks 0` 没有基于 Tick 数的成功退出条件。server 正常达到自己的全局 `max_ticks` 后发布
`STOPPED`，SDK 只对匹配 session 的 `ERROR` 自动停止，因此仍应由进程编排管理这种终止。
server 的 `max_ticks` 从启动后的第一步开始计数，包括 Adapter 接入前的自主 step，不能与
Adapter `--ticks` 简单设成相同值。

## 测试

启用 `BUILD_TESTING` 后，本包注册三个 CTest：

| 测试 | 覆盖范围 |
|---|---|
| `sample_adapter_test` | 名称列表、固定 command、生成式 CLI/YAML、严格校验、优先级和事务性。 |
| `sample_adapter_e2e` | 启动真实无限 headless server 和 Adapter，完成 5 个 Adapter Tick、验证显式注销后终止 server。 |
| `sample_adapter_gui_e2e` | 同一动态生命周期的 GUI 路径；确认注销后终止 server，无显示环境通过 `xvfb-run`。 |

从仓库根目录运行：

```bash
colcon test --packages-select mujoco_sim_sample_adapter
colcon test-result --verbose
```

两个 E2E 测试使用同一个
[`simple_actuator.xml`](../../mujoco_sim_core/test/testdata/simple_actuator.xml)，各自的 CTest
timeout 为 20 秒。
