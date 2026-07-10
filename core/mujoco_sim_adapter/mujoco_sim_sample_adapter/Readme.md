# mujoco_sim_sample_adapter

`mujoco_sim_sample_adapter` 是
[mujoco_sim_adapter_sdk](../mujoco_sim_adapter_sdk/Readme.md) 的可运行参考实现。它注册
指定 observation/command，对每个 Tick layout 中的全部 command actuator 提交同一个
常量值，用于验证 controller 注册和同步 Tick/Commit 链路。

## 职责与边界

本包当前提供：

- 可执行程序 `mujoco-sim-sample-adapter`，负责解析参数并驱动 SDK 的
  `Start()`/`Run()` 生命周期。
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

生产代码要求 CMake 3.22、C++20 和 `mujoco_sim_adapter_sdk`；eCAL、协议与 Protobuf 由
SDK target 传递。生产程序不直接依赖 MuJoCo。启用测试后还需要 GTest、
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

主程序使用默认构造的 `SampleOptions`，因此下表是直接运行可执行程序时的默认值。
解析器实际支持以下全部参数。参数和值必须写成两个 argv，例如 `--server-name demo`；
当前实现不支持 `--server-name=demo` 形式。

| 参数 | 默认值 | 当前含义 |
|---|---:|---|
| `-h`, `--help` | - | 输出用法并以 `0` 退出，不连接 server。 |
| `--server-name NAME` | `default` | registry 和 topic namespace；必须与 server 一致。 |
| `--controller-id ID` | `sample_controller` | controller 唯一 ID；不能为空。 |
| `--observations a,b` | `hinge_pos,hinge_vel` | 逗号分隔的 observation 名称；条目会去除首尾空白，空条目被丢弃，空列表允许。 |
| `--commands a,b` | `hinge_motor` | 逗号分隔的 actuator 名称；处理规则同上，但最终必须至少有一个名称。 |
| `--ticks N` | `0` | 处理 N 个不同的新 Tick 后成功退出；`0` 表示无限等待。 |
| `--command-value VALUE` | `0.0` | 为 Tick layout 中每个 command 提交的 `double` 标量。 |
| `--timeout-ms N` | `1000` | 注册给 server 的每 Tick Commit 等待上限；`0` 表示使用 server 默认 timeout。 |
| `--service-timeout-ms N` | `5000` | 注册和激活各自等待 registry service 的总上限，单位毫秒。 |

`N` 当前通过 `std::stoull` 解析，`VALUE` 通过 `std::stod` 解析。缺少值、无法解析的
数值、未知参数、空 controller ID 或空 command 列表都会导致参数解析失败。当前
`--help` 输出是简略 synopsis，其中未列出两个 timeout 参数，但解析器支持它们。

当前 parser 没有额外拒绝前导负号，`std::stoull` 可能把 `-1` 接受为无符号值；两个
timeout 还会从 `uint64_t` 直接转换为 `uint32_t`，超过 `4294967295` 的值会截断，例如
`4294967296` 会变成 `0`。这不是推荐用法：`--ticks` 应使用 `0..UINT64_MAX`，两个
timeout 应限制在 `0..4294967295`，并且不要传负数。

`--timeout-ms` 与 server 的 `--startup-timeout-ms` 含义不同：前者在 controller 注册后
约束每个 Tick 的 Commit 等待时间；后者约束一个设置了有限 `--max-ticks` 的 server
等待首个 active controller 的时间。`--service-timeout-ms` 只控制 Adapter 调用 registry
service 的等待时间。

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
| `ParseCommandLine()` | 重置 server/controller、资源列表和 `timeout_ms` 后解析 CLI；省略时保留传入对象原有的 `max_ticks`、`command_value` 和 `service_timeout_ms`。 |
| `Usage()` | 返回当前简略命令行 synopsis。 |

## 单独启动 Adapter

构建并加载环境后，可直接启动 sample 程序：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name default \
  --controller-id sample_controller \
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
  --max-ticks 5 \
  --tick-timeout-ms 1000 \
  --startup-timeout-ms 5000
```

在 5 秒 startup timeout 内启动终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name gui_demo \
  --controller-id gui_sample \
  --ticks 5 \
  --command-value 0.1 \
  --timeout-ms 1000 \
  --service-timeout-ms 5000
```

Adapter 处理并提交 5 个不同 Tick 后返回 `0`；server 完成 5 次 `mj_step()` 后也返回 `0`
并关闭 viewer。

## Headless Server 双终端联调

无显示环境使用 `--headless`。仍然先启动终端 1：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name headless_demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 5 \
  --tick-timeout-ms 1000 \
  --startup-timeout-ms 5000
```

再启动终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name headless_demo \
  --controller-id headless_sample \
  --ticks 5 \
  --command-value 0.1 \
  --timeout-ms 1000 \
  --service-timeout-ms 5000
```

GUI 与 headless 模式使用同一 registry 和 Tick/Commit 路径；差别只在 server 是否创建图形
viewer。

## 自定义资源示例

测试模型还提供 `hinge_force` observation。下面使用它替换默认 observation 列表，展示
实际的自定义资源注册。终端 1：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name force_demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 3 \
  --startup-timeout-ms 5000
```

终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name force_demo \
  --controller-id force_sample \
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
- 对有限 `--max-ticks` 的 server，Adapter 必须在 `--startup-timeout-ms` 内完成注册和激活。
- `controller_id` 非空且未注册；资源名存在、列表中无重复项，每个 command actuator
  尚未被其他 controller 认领。
- 注册和激活必须发生在第一个 Tick 打开前；当前 server 不支持运行时新增 controller。
- `command_value` 必须是有限值，并处于 actuator control range 内。示例模型的
  `hinge_motor` 范围是 `[-1, 1]`。
- handler 必须在 controller timeout 内提交完整 command。Sample 自动覆盖 Tick layout
  中的全部 command，但 server timeout 仍应留出通信和调度余量。

可执行程序的退出行为：

| 返回值 | 条件 |
|---:|---|
| `0` | `-h/--help`；或 `--ticks N` 中的 `N > 0` 且已处理 N 个不同 Tick。 |
| `1` | CLI 解析失败、SDK `Start()` 失败、激活失败，或 eCAL 在未请求 Stop 时结束。 |

`--ticks 0` 没有基于 Tick 数的成功退出条件。Sample 也不订阅 server status，server 提前
退出不会可靠地让 Adapter 自动返回；需要自动收尾时，应像上述示例一样让 server
`--max-ticks` 与 Adapter `--ticks` 保持一致，或由外部进程管理终止。注册/启动错误会写入
标准错误；当前 SDK 的激活失败只体现为返回值 `1`。

## 测试

启用 `BUILD_TESTING` 后，本包注册三个 CTest：

| 测试 | 覆盖范围 |
|---|---|
| `sample_adapter_test` | 名称列表解析、按 Tick layout 生成固定 command、CLI 默认值与覆盖。 |
| `sample_adapter_e2e` | 启动真实 headless server 和 Adapter，完成 5 个 lockstep Tick。 |
| `sample_adapter_gui_e2e` | 启动真实 GUI server 和 Adapter，完成 5 个 lockstep Tick；无显示环境通过 `xvfb-run`。 |

从仓库根目录运行：

```bash
colcon test --packages-select mujoco_sim_sample_adapter
colcon test-result --verbose
```

两个 E2E 测试使用同一个
[`simple_actuator.xml`](../../mujoco_sim_core/test/testdata/simple_actuator.xml)，各自的 CTest
timeout 为 20 秒。
