# mujoco_sim_server

`mujoco_sim_server` 是 MuJoCo lockstep 仿真的进程入口和 eCAL 组合层。它加载
MJCF 模型，将无传输依赖的仿真 Core 暴露为 V1 eCAL service/topic，可选运行
MuJoCo `simulate` 图形界面，并安装只读状态查询命令 `mujoco-sim-status`。包版本为
`0.1.0`，协议版本为 `1.0`。

本文所有 shell 命令都假定当前目录是仓库根目录。示例统一使用仓库已有的
[simple_actuator.xml](../mujoco_sim_core/test/testdata/simple_actuator.xml)。

## 职责与边界

本包负责：

- 解析命令行和严格 YAML 配置，构造 `ServerConfig`。
- 初始化 eCAL、加载 MJCF，并注册 controller registry service 和运行时 topic。
- 驱动 Tick/Commit barrier：向每个 active controller 发布 Tick，收齐有效 Commit
  后由 Core 写入 actuator command 并执行一个 `mj_step()`。
- 周期发布 server/controller 状态，处理 Heartbeat，并把运行时错误转换为进程退出码。
- 默认创建基于 GLFW 的 MuJoCo `simulate` 界面；界面读取 Core 的隔离快照，不持有或
  修改真实仿真 `mjModel`/`mjData`。
- 提供 `mujoco-sim-status`，订阅 `ServerStatus`、查询模型资源，并支持单次、watch、
  人类可读和 JSON 输出。

边界如下：

- Protobuf 消息和 eCAL 命名规则由
  [`mujoco_sim_protocol`](../mujoco_sim_protocol/Readme.md) 定义，本包只负责传输和分发。
- 模型状态、资源目录、controller/session/actuator 所有权、Commit 校验和物理步进由
  [`mujoco_sim_core`](../mujoco_sim_core/Readme.md) 实现。
- controller 控制逻辑、重试和命令生成属于 adapter；本包不内置控制策略，也不向
  adapter 暴露原始 MuJoCo 指针。
- GUI 和 `--headless` 使用同一套 Core 与 eCAL 协议；GUI 不改变 barrier 语义。

## 依赖与构建

[package.xml](package.xml) 和 [CMakeLists.txt](CMakeLists.txt) 声明了以下依赖：

| 类别 | 依赖 | 用途 |
|---|---|---|
| 构建 | CMake `>= 3.22`、C++20 | 配置和编译 |
| 仓库内 | `mujoco_sim_core`、`mujoco_sim_protocol` | 仿真 Core、V1 Protobuf 与命名规则 |
| 系统 | CLI11、eCAL、GLFW 3、libpng、yaml-cpp、Threads | CLI、IPC、GUI、PNG 兼容层、YAML、线程 |
| 测试 | GTest、Xvfb、`readelf` | 单元/包消费测试、状态工具依赖检查和无物理显示器的 GUI 测试 |

对应 `package.xml` key 为 `cli11`、`ecal`、`libglfw3-dev`、`libpng-dev`、
`yaml-cpp`，测试还需要 `gtest` 和 `xvfb`。即使运行时始终使用 `--headless`，当前
构建仍会编译 GUI，因此 GLFW 和 libpng 仍是构建依赖。MuJoCo 3.10.0 库及
`simulate` 源码来自 `mujoco_sim_core` 中现有的 Linux/x86 vendored 目录。
安装态 package consumer 使用 `readelf` 检查状态工具没有引入 MuJoCo/GUI 等重依赖；
Ubuntu 可通过 `binutils` 提供该命令，当前 `package.xml` 尚未单独声明这一测试依赖。

已配置 rosdep 的环境可先安装清单依赖：

```bash
rosdep install --from-paths core --ignore-src -r -y
```

构建本包及其仓库内依赖：

```bash
colcon build --packages-up-to mujoco_sim_server --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

`source install/setup.bash` 只影响当前 shell，新终端需要重新执行。

## 启动模式

`model_path` 没有内置默认值，必须由 `--model` 或 YAML 提供。以下三条启动命令是
互斥的替代方案。

### 默认 GUI

不传 `--headless` 时默认创建 MuJoCo `simulate` 窗口：

```bash
source install/setup.bash
mujoco-sim-server \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

### 无头模式

服务器、CI 或无显示环境使用 `--headless`。此模式不初始化 GLFW/OpenGL：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

### 显式 GUI

`--gui` 显式选择图形模式，主要用于覆盖 YAML 中的 `headless: true`：

```bash
source install/setup.bash
mujoco-sim-server \
  --gui \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

GUI 需要有效的 X11/OpenGL 环境。窗口关闭时 server 请求停止并正常返回；没有 active
controller 时模型只显示而不会推进仿真。

### Xvfb

Xvfb 提供虚拟 X server，适合必须覆盖 GUI 路径但没有物理显示器的环境：

```bash
source install/setup.bash
xvfb-run -a mujoco-sim-server \
  --gui \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml
```

Xvfb 仍会初始化并运行 GUI 渲染；如果不需要渲染，应直接使用 `--headless`。当 CMake
配置测试时发现 `DISPLAY` 未设置，会查找 `xvfb-run` 并自动给 GUI 测试添加
`xvfb-run -a`；此时未安装 Xvfb 会导致测试配置失败。

## CLI 参数

语法为 `mujoco-sim-server [OPTIONS]`，完整参数如下：

| 参数 | 值/范围 | 程序默认值 | 行为与约束 |
|---|---|---|---|
| `-h`, `--help` | 无 | - | 将 CLI11 生成的帮助写到 stdout 并以 `0` 退出。 |
| `--config PATH` | 已存在的文件 | 不加载配置文件 | 加载一个 YAML 文件；CLI11 在解析阶段检查文件存在。 |
| `--model PATH` | 字符串 | 空 | MuJoCo MJCF 路径；合并配置后必须非空。文件读取和 MJCF 校验在启动阶段进行。 |
| `--server-name NAME` | 非空字符串 | `default` | eCAL service/topic 的命名空间。 |
| `--max-ticks N` | `uint64`，`0..18446744073709551615` | `0` | 达到 N 个 tick 后成功退出；`0` 表示无限运行。 |
| `--tick-timeout-ms N` | `uint32`，`0..4294967295` | `1000` | controller 注册请求未提供非零 timeout 时使用的 Commit barrier 超时。 |
| `--startup-timeout-ms N` | `uint32`，`0..4294967295` | `5000` | 有界运行等待 active controller 的时间；仅在 `max_ticks > 0` 时触发。 |
| `--headless` | 无 | 未启用 | 将 `headless` 设为 `true`，不创建 viewer。与 `--gui` 互斥。 |
| `--gui` | 无 | 未启用 | 将 `headless` 设为 `false`，可覆盖 YAML。与 `--headless` 互斥。 |

CLI 还执行以下校验：

- 不接受位置参数或未知参数。
- 每个 option/flag 最多出现一次；重复参数会失败。
- `--headless` 与 `--gui` 不能同时出现。
- 无符号整数拒绝负数、非数字和溢出值。
- 最终 `model_path` 和 `server_name` 必须非空。

相对 `--model` 路径按 server 进程的当前工作目录解析，不按 YAML 文件所在目录解析；
本文命令从仓库根目录运行，因此示例模型路径以 `core/` 开头。

## YAML 配置

YAML 顶层支持且只支持以下字段：

| 字段 | 严格类型/范围 | 默认值 | 含义 | 对应 CLI 覆盖 |
|---|---|---|---|---|
| `server_name` | string，最终非空 | `default` | eCAL server 命名空间 | `--server-name` |
| `model_path` | string，最终非空 | 空 | MJCF 模型路径 | `--model` |
| `max_ticks` | unsigned integer (`uint64`)，`0..18446744073709551615` | `0` | 最大 tick 数；`0` 无限 | `--max-ticks` |
| `tick_timeout_ms` | unsigned integer (`uint32`)，`0..4294967295` | `1000` | 默认 Commit barrier 超时 | `--tick-timeout-ms` |
| `startup_timeout_ms` | unsigned integer (`uint32`)，`0..4294967295` | `5000` | 有界运行的 controller 启动超时 | `--startup-timeout-ms` |
| `status_period_ms` | unsigned integer (`uint32`)，`0..4294967295` | `100` | `ServerStatus` 发布周期；`0` 会在每轮 server loop 尝试发布 | 无 |
| `headless` | boolean | `false` | `true` 不创建 viewer，`false` 创建 viewer | `--headless` / `--gui` |

### 严格校验

- 文件最多包含一个 YAML document。空文件或空/null document 合法，表示不修改当前
  默认配置；非空 document 的根节点必须是 mapping。
- key 必须是标量字段名；未知字段和重复字段均失败。
- string 字段必须是 YAML 字符串，mapping、sequence、null、隐式数字或隐式布尔值
  都会失败。值本身类似数字或布尔值时应显式加引号。
- integer 字段必须是未加引号的 YAML 无符号整数；负数、浮点数、字符串和超出目标
  类型范围的值都会失败。实现没有额外的非零下限。
- `headless` 必须是 yaml-cpp 可解析的 YAML boolean；例如 `true`/`false` 合法，
  `"true"` 和 `enabled` 非法。
- YAML 语法错误、多 document、类型错误、未知/重复字段都会中止启动。错误文本包含
  配置路径以及可用的行列号；失败的加载不会部分修改配置。
- 完成 YAML 与 CLI 合并后，空 `model_path` 或空 `server_name` 仍会中止启动。

### 优先级

优先级固定为：**命令行显式值 > YAML > 程序默认值**。

`--config` 在 argv 中出现的位置不影响优先级：程序先解析整条命令行，再加载 YAML，
最后统一应用 CLI override。未显式给出的 CLI 参数不会覆盖 YAML；`--headless` 写入
`true`，`--gui` 写入 `false`。`status_period_ms` 没有 CLI 参数，只能来自 YAML 或
默认值。程序不读取配置环境变量，也不自动查找默认配置文件。

### 完整示例

将以下内容作为仓库根目录下的 `server.yaml`：

```yaml
server_name: demo
model_path: core/mujoco_sim_core/test/testdata/simple_actuator.xml
max_ticks: 5
tick_timeout_ms: 1000
startup_timeout_ms: 5000
status_period_ms: 100
headless: true
```

加载配置：

```bash
source install/setup.bash
mujoco-sim-server --config server.yaml
```

命令行覆盖示例（即使 `--config` 位于最后，结果也相同）：

```bash
source install/setup.bash
mujoco-sim-server \
  --server-name cli_demo \
  --gui \
  --config server.yaml
```

示例设置了 `max_ticks: 5`，因此需要在 5 秒默认启动超时内连接并激活 adapter；完整
双终端命令见下文。

## eCAL service 与 topic

endpoint 名称由 `server_name` 和 controller ID 构造，定义见
[`topics.hpp`](../mujoco_sim_protocol/include/mujoco_sim/protocol/topics.hpp)。载荷是
[`sync.proto`](../mujoco_sim_protocol/proto/mujoco_sim/v1/sync.proto) 中消息的序列化
Protobuf bytes。

### Registry service

service 名称为：

```text
mujoco_sim.<server_name>.registry
```

默认配置对应 `mujoco_sim.default.registry`。该 service 注册四个 method：

| method | request | response | 作用 |
|---|---|---|---|
| `GetAvailableResources` | `GetAvailableResourcesRequest` | `GetAvailableResourcesResponse` | 查询模型中的 observation sensor 和 command actuator。 |
| `RegisterController` | `RegisterControllerRequest` | `RegisterControllerResponse` | 申请资源、检查冲突并取得 session、epoch 和实际 layout。 |
| `ActivateController` | `ActivateControllerRequest` | `ActivateControllerResponse` | 将 provisional controller 激活；运行开始后不支持动态激活。 |
| `UnregisterController` | `UnregisterControllerRequest` | `UnregisterControllerResponse` | 在没有 open tick 时释放 controller 及 actuator 所有权。 |

无法反序列化的 service request 会得到对应 response 类型的 `INVALID_ARGUMENT`；业务
校验失败通过 response 的 `ErrorCode` 返回，通常不会终止 server。

### Runtime topic

下表方向均以 server 为参照：

| 名称模板 | 方向 | Protobuf 消息 | 行为 |
|---|---|---|---|
| `mujoco_sim/<server_name>/tick/<controller_id>` | 发布 | `Tick` | 每个 active controller 独立 topic，携带其 layout 和 observation。 |
| `mujoco_sim/<server_name>/commit` | 订阅 | `AdapterCommit` | 所有 adapter 共用；有效 Commit 参与当前 tick barrier。 |
| `mujoco_sim/<server_name>/heartbeat` | 订阅 | `Heartbeat` | 所有 adapter 共用；更新合法 controller 的 heartbeat。 |
| `mujoco_sim/<server_name>/status` | 发布 | `ServerStatus` | 按 `status_period_ms` 发布状态；达到 `max_ticks` 或受控运行错误退出前再发布一次。 |

server 启动 eCAL 时的进程名为 `mujoco-sim-server`。空或无法解析的 Commit/Heartbeat
topic payload 会被忽略。server 与 adapter 的 `server_name` 必须一致，否则它们使用
不同 endpoint。

正常 lockstep 顺序为：查询/注册资源、激活 controller、server 发布 Tick、每个 active
controller 提交完整且有效的 actuator command、Core 收齐 barrier 后执行一步并发布
下一 Tick。同一 open tick 会重复发布，adapter 可幂等重发相同 Commit。

## 状态查询工具

本包同时安装独立可执行文件 `mujoco-sim-status`。其源码、内部 library 和测试位于
`tools/status/`，目标只链接 CLI11、eCAL、Threads 和 `mujoco_sim_protocol`，不链接
server library、MuJoCo、GLFW、libpng 或 yaml-cpp。它仍属于同一个 ROS/colcon 包，
因此这不是包管理层面的独立轻量包。

默认查询名为 `default` 的 server，收到一条状态后退出：

```bash
source install/setup.bash
mujoco-sim-status
mujoco-sim-status --server-name demo
```

持续监控或输出机器可读数据：

```bash
mujoco-sim-status --server-name demo --watch --interval-ms 1000
mujoco-sim-status --server-name demo --json
mujoco-sim-status --server-name demo --watch --json
```

| 参数 | 默认值 | 行为 |
|---|---:|---|
| `-h`, `--help` | - | 输出 CLI11 帮助并以 `0` 退出。 |
| `--server-name NAME` | `default` | 选择 status topic 和 registry service 的 namespace。 |
| `--timeout-ms N` | `3000` | 首条有效状态的等待上限，也是 watch 的 stale 阈值；必须大于 0。 |
| `--watch` | 关闭 | 持续监控；stale 时提示但不退出，恢复后继续输出。 |
| `--interval-ms N` | `1000` | watch 的最小输出间隔；必须大于 0。 |
| `--json` | 关闭 | 单次输出 JSON；watch 每行输出一个完整 JSON 对象，即 NDJSON。 |

人类模式显示 server state、epoch、tick、协议版本、更新时间、状态消息、controller，
以及 observation/command 的数量、名称和维度。JSON 顶层固定为 `status`、`resources`
和 `resource_error`；资源查询失败不会使状态查询失败，此时 `resources` 为 `null`，具体
原因位于 `resource_error`。stdout 只包含状态结果，stale/recovered 和冲突诊断写入
stderr。

退出码为：成功或正常 Ctrl-C 返回 `0`，CLI/eCAL/格式化错误返回 `1`，在
`--timeout-ms` 内没有收到有效状态返回 `2`。工具锁定首个有效 status publisher；同名
publisher 并存时忽略冲突来源，当前来源 stale 后才切换，并按 host/PID/epoch 关联资源
service 响应。

## 与 sample adapter 双终端联调

先构建 sample adapter 及其依赖：

```bash
colcon build --packages-up-to mujoco_sim_sample_adapter --cmake-args -DBUILD_TESTING=ON
```

打开两个当前目录均为仓库根目录的终端。先启动终端 1，再在 5 秒内启动终端 2。

终端 1，启动有界 headless server：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 5 \
  --startup-timeout-ms 5000 \
  --tick-timeout-ms 1000
```

终端 2，启动
[`mujoco_sim_sample_adapter`](../mujoco_sim_adapter/mujoco_sim_sample_adapter/Readme.md)：

```bash
source install/setup.bash
mujoco-sim-sample-adapter \
  --server-name demo \
  --controller-id sample_controller \
  --observations hinge_pos,hinge_vel \
  --commands hinge_motor \
  --ticks 5 \
  --command-value 0.1 \
  --timeout-ms 1000 \
  --service-timeout-ms 5000
```

`simple_actuator.xml` 确实定义了 `hinge_pos`、`hinge_vel` 和 `hinge_motor`；命令值
`0.1` 位于 actuator 的 `[-1, 1]` control range 内。adapter 处理 5 个 Tick 后退出，
server 达到 `--max-ticks 5` 后也应以 `0` 退出。要联调 GUI，可在终端 1 删除
`--headless`（使用默认 GUI）或替换为 `--gui`；无显示器时在命令前加 `xvfb-run -a`。

## 退出码与失败模式

| 退出码 | 主要含义 | 典型场景 |
|---|---|---|
| `0` | 正常结束 | `--help`；达到非零 `max_ticks`；调用 `Stop()`；GUI 窗口关闭。 |
| `1` | 参数/配置/启动或 eCAL 生命周期失败 | CLI/YAML 错误；缺少模型；MJCF 加载失败；eCAL 或 GLFW 初始化失败；GUI 初始加载失败；未请求 Stop 时 eCAL 不再可用。 |
| `2` | server 已启动后的受控运行错误 | 有界运行在启动期限内没有 active controller；Core 进入 `ERROR_PAUSED`；GUI 快照同步失败。 |

常见运行时失败行为：

- `max_ticks == 0` 时，`startup_timeout_ms` 不会让无 controller 的 server 退出；server
  会保持 IDLE 并等待 controller。`max_ticks > 0` 时超时退出码为 `2`。
- active controller 未在其 barrier timeout 内提交 Commit，会使 Core 进入
  `ERROR_PAUSED`，发布最终状态并以 `2` 退出。
- session/epoch/tick 不匹配、未知或未激活 controller 的 Commit、重复 Commit 内容不
  一致、controller 主动报告 Commit error、遗漏/重复/越权 actuator、非有限值或超出
  control range 的 command 都可能进入 `ERROR_PAUSED` 并以 `2` 退出。
- registry method 的资源冲突、版本不匹配或前置条件失败会返回 Protobuf 错误响应，
  本身不等同于进程失败。
- 默认 GUI 在没有可用 display 时通常在 GLFW 初始化阶段以 `1` 失败；使用
  `--headless` 或 Xvfb。

## 公开 CMake target

安装后使用：

```cmake
find_package(mujoco_sim_server CONFIG REQUIRED)

add_executable(my_server main.cpp)
target_link_libraries(my_server PRIVATE mujoco_sim::server)
```

| 用途 | 本包 build tree | `find_package` 后的导入 target |
|---|---|---|
| server C++ 库 | `mujoco_sim::server` | `mujoco_sim::server` |
| server 可执行文件 | `mujoco-sim-server` | `mujoco_sim::mujoco-sim-server` |
| 状态查询命令 | `mujoco-sim-status` | 不导出 CMake target；安装后直接运行同名命令 |

可链接库公开
[`server_config.hpp`](include/mujoco_sim/server/server_config.hpp) 和
[`ecal_server.hpp`](include/mujoco_sim/server/ecal_server.hpp)，提供配置解析以及
`EcalServer::Start/Run/Stop`。可执行 target 用于 `$<TARGET_FILE:...>`、测试或部署，
不作为链接库使用。安装态 package config 会继续查找 CLI11、eCAL、GLFW、
`mujoco_sim_core`、`mujoco_sim_protocol`、PNG、Threads 和 yaml-cpp。
状态工具内部静态库 `mujoco_sim_status_lib` 仅供本包构建和测试，不安装、不导出。

## 测试

构建并运行本包测试：

```bash
colcon build --packages-up-to mujoco_sim_server --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mujoco_sim_server
colcon test-result --verbose
```

也可从仓库根目录直接运行该包已经配置好的 CTest：

```bash
ctest --test-dir build/mujoco_sim_server --output-on-failure
```

本包注册的测试包括：

- `server_test`：CLI、默认值、严格 YAML、优先级和 endpoint 命名。
- `png_writer_test`：PNG 兼容写出路径。
- `server_help`：帮助输出和退出码。
- `server_package_consumer`：安装、外部 `find_package`、链接、两个安装态命令和状态工具
  动态依赖检查。
- `sim_viewer_test`：viewer 加载、快照同步和关闭。
- `server_gui_startup_timeout`：GUI 模式下有界启动超时返回 `2`。
- `status_tool_test`：状态工具 CLI、publisher/epoch 跟踪、payload 校验及 human/JSON 输出。
- `status_tool_e2e`：真实 server 的单次 human/JSON 查询、watch NDJSON、stale、恢复和
  SIGINT 退出。

覆盖 server 与 sample adapter 的 headless/GUI 端到端 lockstep 测试：

```bash
colcon build --packages-up-to mujoco_sim_sample_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mujoco_sim_sample_adapter
colcon test-result --verbose
```

`sim_viewer_test`、`server_gui_startup_timeout` 和 sample adapter 的 GUI e2e 都需要
display；配置阶段没有 `DISPLAY` 时会自动改用 `xvfb-run -a`。
