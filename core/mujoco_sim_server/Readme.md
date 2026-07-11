# mujoco_sim_server

`mujoco_sim_server` 是 MuJoCo lockstep 仿真的进程入口和 eCAL 组合层。它加载
MJCF 模型，将无传输依赖的仿真 Core 暴露为 V1 eCAL service/topic，可选运行
MuJoCo `simulate` 图形界面，并安装只读状态查询命令 `mujoco-sim-status`。包版本为
`0.1.0`，协议版本为 `1.1`。

本文所有 shell 命令都假定当前目录是仓库根目录。示例统一使用仓库已有的
[simple_actuator.xml](../mujoco_sim_core/test/testdata/simple_actuator.xml)。

## 职责与边界

本包负责：

- 使用 `mujoco_sim_config` 的构建期 schema 生成器解析命令行和严格 YAML，构造
  `ServerConfig`。
- 初始化 eCAL、加载 MJCF，并注册 controller registry service 和运行时 topic。
- 模型加载后立即以 `RUNNING` 驱动仿真；无 adapter 时按模型 timestep 实时步进，有
  adapter 时按每 Tick 固定的 Commit Barrier 推进。
- 支持运行期 register/activate/unregister，以及 adapter timeout、恢复、替换和局部命令
  错误隔离。
- 周期发布 server/controller 状态，处理 Heartbeat，并把运行时错误转换为进程退出码。
- 使用结构化键值日志记录生命周期、协议拒绝、超时和窗口化运行指标；日志默认写 stderr，
  可选写入轮转文件。
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
| 仓库内 | `mujoco_sim_config`、`mujoco_sim_core`、`mujoco_sim_protocol` | 生成配置、仿真 Core、V1 Protobuf 与命名规则 |
| 系统 | Python 3/PyYAML、CLI11、eCAL、GLFW 3、libpng、yaml-cpp、spdlog、Threads | 构建期生成、CLI、IPC、GUI、PNG、YAML、日志、线程 |
| 测试 | GTest、Xvfb、`readelf` | 单元/包消费测试、状态工具依赖检查和无物理显示器的 GUI 测试 |

CLI11、yaml-cpp、Python 和 PyYAML 由 `mujoco_sim_config` 传递；本包直接声明 eCAL、
GLFW、libpng 和 spdlog，测试还需要 GTest 和 Xvfb。即使运行时始终使用 `--headless`，当前
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
controller 时模型仍按 MJCF timestep 实时推进并持续刷新画面。

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
| `--initial-keyframe NAME` | 字符串 | 空 | 模型加载后原子应用具名 MuJoCo keyframe；空值使用模型默认初始状态。 |
| `--server-name NAME` | 非空字符串 | `default` | eCAL service/topic 的命名空间。 |
| `--max-ticks N` | `uint64`，`0..18446744073709551615` | `0` | server 启动后累计完成 N 次 `mj_step()` 即成功退出；包含无 adapter 的 step，`0` 表示无限运行。 |
| `--tick-timeout-ms N` | `uint32`，`0..4294967295` | `1000` | controller 注册请求未提供非零 timeout 时使用的 Commit barrier 超时。 |
| `--startup-timeout-ms N` | `uint32`，`0..4294967295` | `5000` | 已弃用的兼容参数；仍解析并记录，但运行时始终忽略。 |
| `--activation-timeout-ms N` | `uint32`，`1..4294967295` | `5000` | `PROVISIONAL` adapter 完成 Activate 的最长时间；过期后自动移除并释放资源。 |
| `--allow-timed-out-adapter-replacement` | 无 | 启用 | 允许同 `controller_id` 的新注册替换 `TIMED_OUT` session。与 deny flag 互斥。 |
| `--deny-timed-out-adapter-replacement` | 无 | 未启用 | 禁止同名 `TIMED_OUT` 替换；注册返回 `ALREADY_EXISTS`。与 allow flag 互斥。 |
| `--headless` | 无 | 未启用 | 将 `headless` 设为 `true`，不创建 viewer。与 `--gui` 互斥。 |
| `--gui` | 无 | 未启用 | 将 `headless` 设为 `false`，可覆盖 YAML。与 `--headless` 互斥。 |
| `--log-level LEVEL` | `trace/debug/info/warn/error/critical/off` | `info` | 设置 server 最低日志级别。 |
| `--log-file PATH` | path | 空 | 非空时在 stderr 之外写轮转文件；自动创建父目录。 |
| `--log-file-max-size-mb N` | `uint32`，`>= 1` | `10` | 单个日志文件的 MiB 上限。 |
| `--log-file-max-files N` | `uint32`，`1..200000` | `5` | 包含当前文件在内的日志文件总数。 |
| `--log-summary-period-ms N` | `uint32` | `5000` | 运行摘要周期；`0` 关闭周期和退出前摘要。 |

CLI 还执行以下校验：

- 不接受位置参数或未知参数。
- 每个 option/flag 最多出现一次；重复参数会失败。
- `--headless` 与 `--gui` 不能同时出现。
- `--allow-timed-out-adapter-replacement` 与 `--deny-timed-out-adapter-replacement` 不能同时出现。
- 无符号整数拒绝负数、非数字和溢出值。
- `activation_timeout_ms` 必须至少为 `1`。
- 最终 `model_path` 和 `server_name` 必须非空。

相对 `--model` 路径按 server 进程的当前工作目录解析；YAML 中的相对 `model_path` 按
YAML 文件所在目录解析。因此安装态配置可以用相对路径稳定引用同包安装的 MJCF。

## YAML 配置

YAML 顶层支持且只支持以下字段：

| 字段 | 严格类型/范围 | 默认值 | 含义 | 对应 CLI 覆盖 |
|---|---|---|---|---|
| `server_name` | string，最终非空 | `default` | eCAL server 命名空间 | `--server-name` |
| `model_path` | string，最终非空 | 空 | MJCF 模型路径 | `--model` |
| `initial_keyframe` | string | 空 | 初始具名 MuJoCo keyframe；空值不应用 keyframe | `--initial-keyframe` |
| `max_ticks` | unsigned integer (`uint64`)，`0..18446744073709551615` | `0` | 从启动开始的最大总 step 数；`0` 无限 | `--max-ticks` |
| `tick_timeout_ms` | unsigned integer (`uint32`)，`0..4294967295` | `1000` | 默认 Commit barrier 超时 | `--tick-timeout-ms` |
| `startup_timeout_ms` | unsigned integer (`uint32`)，`0..4294967295` | `5000` | 已弃用且忽略，仅保留配置兼容 | `--startup-timeout-ms` |
| `activation_timeout_ms` | unsigned integer (`uint32`)，`1..4294967295` | `5000` | provisional adapter 激活期限 | `--activation-timeout-ms` |
| `allow_timed_out_adapter_replacement` | boolean | `true` | 是否允许同名注册替换 `TIMED_OUT` session | allow/deny flags |
| `status_period_ms` | unsigned integer (`uint32`)，`0..4294967295` | `100` | `ServerStatus` 发布周期；`0` 会在每轮 server loop 尝试发布 | 无 |
| `headless` | boolean | `false` | `true` 不创建 viewer，`false` 创建 viewer | `--headless` / `--gui` |
| `log_level` | enum：`trace/debug/info/warn/error/critical/off` | `info` | 最低日志级别 | `--log-level` |
| `log_file` | path | 空 | 可选轮转日志文件 | `--log-file` |
| `log_file_max_size_mb` | unsigned integer (`uint32`)，`1..4294967295` | `10` | 单文件 MiB 上限 | `--log-file-max-size-mb` |
| `log_file_max_files` | unsigned integer (`uint32`)，`1..200000` | `5` | 当前文件和历史文件总数 | `--log-file-max-files` |
| `log_summary_period_ms` | unsigned integer (`uint32`) | `5000` | 摘要周期，`0` 关闭 | `--log-summary-period-ms` |

### 严格校验

- 文件最多包含一个 YAML document。空文件或空/null document 合法，表示不修改当前
  默认配置；非空 document 的根节点必须是 mapping。
- key 必须是标量字段名；未知字段和重复字段均失败。
- string 字段必须是 YAML 字符串，mapping、sequence、null、隐式数字或隐式布尔值
  都会失败。值本身类似数字或布尔值时应显式加引号。
- integer 字段必须是未加引号的 YAML 无符号整数；负数、浮点数、字符串和超出目标
  类型范围的值都会失败。表中声明非零下限的字段还会执行下限校验。
- `headless` 和 `allow_timed_out_adapter_replacement` 必须是 yaml-cpp 可解析的 YAML
  boolean；例如 `true`/`false` 合法，`"true"` 和 `enabled` 非法。
- YAML 语法错误、多 document、类型错误、未知/重复字段都会中止启动。错误文本包含
  配置路径以及可用的行列号；失败的加载不会部分修改配置。
- YAML 中的相对 path 字段按配置文件目录解析；CLI path 保持相对进程当前目录。
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
initial_keyframe: ""
max_ticks: 5
tick_timeout_ms: 1000
startup_timeout_ms: 5000
activation_timeout_ms: 5000
allow_timed_out_adapter_replacement: true
status_period_ms: 100
headless: true
log_level: info
log_file: logs/server.log
log_file_max_size_mb: 10
log_file_max_files: 5
log_summary_period_ms: 5000
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

示例设置了 `max_ticks: 5`，因此 server 会从模型加载完成开始计数并很快自主完成 5 步；
该值不适合等待人工启动 adapter。双终端联调应使用 `max_ticks: 0`，或为启动前的自主步进
预留足够的总 step 预算。

## 日志

日志采用一行一个事件的键值文本，包含 ISO 时间、level、thread、`component=server` 和
稳定的 `event` 名称。默认彩色输出到 stderr；`log_file` 非空时，同一事件同时写入无
颜色的轮转文件。默认 `10 MiB`、总共 5 个文件，因此最多约占用 50 MiB。显式配置的
文件或父目录无法创建会使 server 启动失败，不会静默退回仅终端输出。
运行期磁盘写入或 flush 失败时，logger 会在 stderr 输出
`event=log_sink_error`，用于监控文件日志健康状态。

```bash
mujoco-sim-server \
  --headless \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --log-level debug \
  --log-file logs/server.log
```

级别约定：`error` 是启动或运行失败；`warn` 是请求/Commit 拒绝、非法 payload、超时或
发布失败；`info` 是启动、模型、endpoint、controller、状态转换、摘要和退出；`debug`
是资源/发布细节；`trace` 才记录每条 Tick、Commit 和 Heartbeat。外部输入引发的同类
重复告警按固定类别最多每秒输出一次，下一条日志的 `suppressed` 字段给出被抑制数量；
用户字符串会转义并截断，协议 payload 和 actuator 数值不会写入日志。

`event=tick_summary` 默认每 5 秒以及退出前输出一次，字段口径为：

- `step_rate_hz`：窗口内完成的 `mj_step` 数除以 wall time。
- `real_time_factor`：窗口仿真时间增量除以 wall time。
- `ticks_opened`：barrier 从关闭变为打开的次数。
- `tick_publish_attempts`、`tick_serialization_failures`、`tick_messages_sent`：Tick
  发布尝试、序列化失败和成功发送消息数；
  `tick_no_subscribers` 统计发送前后至少一次未观察到 subscriber 的 `Send()==false`；
  `tick_send_rejected_with_subscribers` 统计发送前后均观察到 subscriber 但 eCAL 仍返回
  false 的次数。
- `tick_retransmissions`：open barrier 期间重复发送的 Tick 消息数。
- `commits`、`duplicate_commits`、`rejected_commits`、`timeouts`、`recoveries`、
  `replacements`、`local_errors`、`activation_expirations` 和 `malformed_messages`：对应窗口
  计数；摘要还分别列出 provisional/active/timed-out/error controller 数量。
- `barrier_latency_avg_ms`、`barrier_latency_max_ms`：仅统计成功完成的 barrier。

帮助仍只写 stdout；server 结构化日志不写 stdout。`mujoco-sim-status` 是独立 target，
不链接 server logger，其 human/JSON/NDJSON stdout 契约不受影响。eCAL 自身可能输出其
配置加载提示。Heartbeat 的 `trace` 表示 payload 已解析并交给 Core；当前 Core API 不
返回 Heartbeat 语义拒绝原因，因此日志不会声称其已被接受。

## eCAL service 与 topic

endpoint 名称由 `server_name` 和 controller ID 构造，定义见
[`topics.hpp`](../mujoco_sim_protocol/include/mujoco_sim/protocol/topics.hpp)。载荷是
[`sync.proto`](../mujoco_sim_protocol/proto/mujoco_sim/v1/sync.proto) 中消息的序列化
Protobuf bytes。

所有入站 service/topic payload 的 transport 上限为 4 MiB；大于上限的 payload 在
Protobuf 解析前拒绝并进入 malformed 诊断。空的 Commit/Heartbeat topic payload 也按
malformed 处理；service 的空 payload 可作为 protobuf 默认消息进入正常业务校验。

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
| `ActivateController` | `ActivateControllerRequest` | `ActivateControllerResponse` | 运行期将 provisional controller 激活；从下一 Tick 加入 Barrier。 |
| `UnregisterController` | `UnregisterControllerRequest` | `UnregisterControllerResponse` | 运行期清零并释放 controller 及 actuator；可从 open Tick 的 Barrier 移除。 |

无法反序列化的 service request 会得到对应 response 类型的 `INVALID_ARGUMENT`；业务
校验失败通过 response 的 `ErrorCode` 返回，通常不会终止 server。

### Runtime topic

下表方向均以 server 为参照：

| 名称模板 | 方向 | Protobuf 消息 | 行为 |
|---|---|---|---|
| `mujoco_sim/<server_name>/tick/<controller_id>` | 发布 | `Tick` | 每个 `ACTIVE`/`TIMED_OUT` controller 独立 topic，携带其 layout、observation 和状态。 |
| `mujoco_sim/<server_name>/commit` | 订阅 | `AdapterCommit` | 所有 adapter 共用；有效 Commit 参与当前 tick barrier。 |
| `mujoco_sim/<server_name>/heartbeat` | 订阅 | `Heartbeat` | 所有 adapter 共用；更新合法 controller 的 heartbeat。 |
| `mujoco_sim/<server_name>/status` | 发布 | `ServerStatus` | 按 `status_period_ms` 发布状态；达到 `max_ticks` 或受控运行错误退出前再发布一次。 |

server 启动 eCAL 时的进程名为 `mujoco-sim-server`。空或无法解析的 Commit/Heartbeat
topic payload 会记录限流后的 malformed 告警。server 与 adapter 的 `server_name` 必须
一致，否则它们使用不同 endpoint。publisher 尚未发现 subscriber 时 `Send()` 返回
false 属于正常 discovery 状态，只在 trace 记录。eCAL 不返回更具体的失败原因，因此
两次 subscriber 快照都非零但 `Send()` 返回 false 时，日志记录可证实的
`message_send_rejected` 及前后计数，不推断底层传输原因。

### 运行循环与动态生命周期

- 模型加载成功后 Core 立即为 `RUNNING`。没有 active adapter 时，server 打开空 Barrier、
  执行一步，并以模型 `timestep` 为周期安排下一步。每次完成后按“当前 wall time +
  timestep”重新安排，因此调度或 Barrier 延迟不会触发补跑追赶。
- `max_ticks` 比较 server 启动后的全局 Tick/step ID；adapter 注册前和注销后的空 Barrier
  step 也计入。它不是任一 adapter 的 `Run(max_ticks)` 计数。
- 每次打开 Tick 时固定 required Barrier 集合。运行期 register/activate 不修改已打开的
  Barrier，新 `ACTIVE` adapter 从下一 Tick 加入；运行期 unregister 会清零并移除该成员，
  其余 Commit 已齐时可直接完成本 Tick。
- 每个 required adapter 使用其注册的 `timeout_ms`。单个 adapter 超时后转为 `TIMED_OUT`、
  actuator 清零并从 Barrier 移除，server 和其他 adapter 继续 `RUNNING`。当前 Core 已
  issued 的 Tick 保持 immutable，重传不会改写其中的状态或 observation。
- server 继续为 `TIMED_OUT` adapter 生成 Tick 并尝试发布，但 transport 不保证投递成功。
  它提交的完整合法 `COMMIT_OK` 仅证明通信和控制路径恢复，command 不应用到物理状态；
  相同恢复 Commit 的重传幂等成功，该 session 在下一 Tick 才转回 `ACTIVE` 并重新加入
  Barrier。协商 minor 0 时，线上状态降级为旧 SDK 可识别的 `ACTIVE`。
- 默认允许新注册用相同 `controller_id` 替换 `TIMED_OUT` session；旧 actuator 先清零，
  新注册获得新 session 并处于 `PROVISIONAL`。可通过配置关闭替换。未在
  `activation_timeout_ms` 内激活的 provisional 注册会被移除并释放资源。
- 完整但非法的 command、`COMMIT_ERROR` 或同 Tick 不同 payload 的重复 Commit 只把来源
  controller 置为 `ERROR`、清零并移出 Barrier；不会把整个 server 置为
  `ERROR_PAUSED`。身份/version/tick 不匹配的消息只被拒绝，也不会改变 controller 状态。

同一 open Tick 会重复发布；payload 保持不变，adapter 可幂等重发相同 Commit。全局
`ERROR_PAUSED` 只用于 viewer 同步失败等 server 级致命错误。

## 状态查询工具

本包同时安装独立可执行文件 `mujoco-sim-status`。其源码、内部 library 和测试位于
`tools/status/`，目标使用 `mujoco_sim_config` 生成配置并链接 eCAL、Threads 和
`mujoco_sim_protocol`，不链接 server library、Core、MuJoCo、GLFW 或 libpng。它仍属于
同一个 ROS/colcon 包，
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
| `--config PATH` | - | 加载严格 YAML；命令行显式值覆盖 YAML。 |
| `--server-name NAME` | `default` | 选择 status topic 和 registry service 的 namespace。 |
| `--timeout-ms N` | `3000` | 首条有效状态的等待上限，也是 watch 的 stale 阈值；必须大于 0。 |
| `--watch` | 关闭 | 持续监控；stale 时提示但不退出，恢复后继续输出。 |
| `--interval-ms N` | `1000` | watch 的最小输出间隔；必须大于 0。 |
| `--json` | 关闭 | 单次输出 JSON；watch 每行输出一个完整 JSON 对象，即 NDJSON。 |

人类模式显示 server state、epoch、tick、协议版本、更新时间、状态消息、controller，
以及 observation/command 的数量、名称和维度。controller 表列为 `ID`、`State`、
`Session`、`Last heartbeat`、`Last commit`、`State since`、`Timeouts`、`Barrier` 和
`Message`；可直接观察 `TIMED_OUT`/`ERROR`、恢复进度和当前 Barrier 成员。JSON 顶层固定为 `status`、`resources`
和 `resource_error`；资源查询失败不会使状态查询失败，此时 `resources` 为 `null`，具体
原因位于 `resource_error`。stdout 只包含状态结果，stale/recovered 和冲突诊断写入
stderr。

状态 YAML 支持 `server_name`、`timeout_ms`、`interval_ms`、`watch` 和 `json`。其严格
类型、事务性和优先级规则与 server 配置一致。

退出码为：成功或正常 Ctrl-C 返回 `0`，CLI/eCAL/格式化错误返回 `1`，在
`--timeout-ms` 内没有收到有效状态返回 `2`。工具锁定首个有效 status publisher；同名
publisher 并存时忽略冲突来源，当前来源 stale 后才切换，并按 host/PID/epoch 关联资源
service 响应。

## 与 sample adapter 双终端联调

先构建 sample adapter 及其依赖：

```bash
colcon build --packages-up-to mujoco_sim_sample_adapter --cmake-args -DBUILD_TESTING=ON
```

打开两个当前目录均为仓库根目录的终端。server 先启动并自主运行，adapter 可在运行期接入。

终端 1，启动持续运行的 headless server：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name demo \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 0 \
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
`0.1` 位于 actuator 的 `[-1, 1]` control range 内。adapter 处理 5 个 `ACTIVE` Tick 后
尽力注销并退出；server 继续自主运行，使用 Ctrl-C 结束。要联调 GUI，可在终端 1 删除
`--headless`（使用默认 GUI）或替换为 `--gui`；无显示器时在命令前加 `xvfb-run -a`。

## 退出码与失败模式

| 退出码 | 主要含义 | 典型场景 |
|---|---|---|
| `0` | 正常结束 | `--help`；达到非零 `max_ticks`；调用 `Stop()`；GUI 窗口关闭。 |
| `1` | 参数/配置/启动或 eCAL 生命周期失败 | CLI/YAML 错误；缺少模型；MJCF 加载失败；eCAL 或 GLFW 初始化失败；GUI 初始加载失败；未请求 Stop 时 eCAL 不再可用。 |
| `2` | server 已启动后的全局运行错误 | Core 因 server 级致命故障进入 `ERROR_PAUSED`；GUI 快照同步失败。 |

常见运行时失败行为：

- `startup_timeout_ms` 无论 `max_ticks` 是否为零都被忽略；server 不等待首个 adapter，
  而是保持 `RUNNING` 并按 timestep 自主步进。启动时会记录 deprecated warning。
- active controller 未在自己的 Barrier timeout 内提交 Commit，只会转为 `TIMED_OUT`、
  清零并被当前 Barrier 排除；server 不退出。
- 合法恢复 Commit 只安排下一 Tick 恢复；也可以默认允许的同名新 session 替换超时 session。
- 重复 Commit 内容不一致、主动 `COMMIT_ERROR`、遗漏/重复/越权 actuator、非有限值或
  control range 越界只让该 controller 进入 `ERROR`。session/epoch/version/tick 等身份错误
  只拒绝消息。两类错误都不会停止其他 controller 或 server。
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
不作为链接库使用。安装态 package config 会继续查找 `mujoco_sim_config`、eCAL、GLFW、
`mujoco_sim_core`、`mujoco_sim_protocol`、PNG、spdlog 和 Threads；配置包再传递 CLI11 与
yaml-cpp。
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

- `server_test`：生成式 CLI/YAML、日志配置、格式/转义、级别、并发、文件失败、轮转、
  告警限流、payload/send 分类、callback 排空、亚毫秒调度 deadline、全部 Barrier 指标和
  endpoint 命名。
- `png_writer_test`：PNG 兼容写出路径。
- `server_help`：帮助输出和退出码。
- `server_logging_e2e`：真实 eCAL malformed/rejection 流量、stderr/文件双 sink、摘要、
  告警限流、弃用配置诊断、全局 `max_ticks` 退出以及 info 无逐 Tick 噪声。
- `server_lifecycle_e2e`：运行期注册/激活/注销、局部 timeout、恢复、同名替换、局部
  command `ERROR`，并使用双 actuator 验证快速 `ACTIVE` Commit 不会饿死 `TIMED_OUT` Tick，
  且 adapter timeout 不停止 server。
- `server_package_consumer`：安装、外部 `find_package`、链接、两个安装态命令和状态工具
  动态依赖检查。
- `sim_viewer_test`：viewer 加载、快照同步和关闭。
- `server_gui_autonomous_run`：GUI 无 adapter 时自主运行到 `max_ticks` 并返回 `0`。
- `status_tool_test`：状态工具 CLI、publisher/epoch 跟踪、payload 校验，以及包含动态
  controller 列和字段的 human/JSON 输出。
- `status_tool_e2e`：真实 server 的单次 human/JSON 查询、watch NDJSON、stale、恢复和
  SIGINT 退出。

覆盖 server 与 sample adapter 的 headless/GUI 端到端 lockstep 测试：

```bash
colcon build --packages-up-to mujoco_sim_sample_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mujoco_sim_sample_adapter
colcon test-result --verbose
```

`sim_viewer_test`、`server_gui_autonomous_run` 和 sample adapter 的 GUI e2e 都需要
display；配置阶段没有 `DISPLAY` 时会自动改用 `xvfb-run -a`。
