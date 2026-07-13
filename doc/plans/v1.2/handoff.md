# MuJoCo Sim Core V1.2 Handoff

## 1. 文档目的

本文面向下一位维护者，记录截至 2026-07-13 的 Server 单写者事件循环重构状态、关键不变量、
验证结果和后续边界。本轮实施依据：

- [`mujoco_sim_core_maintainability_refactor_plan.md`](./mujoco_sim_core_maintainability_refactor_plan.md)
- [`reduce_wrapper.md`](./reduce_wrapper.md)
- [`../v1.1/handoff.md`](../v1.1/handoff.md)

V1.1 handoff 中关于 Protocol V2、Core Barrier、Tick cache、SDK 生命周期和可选构建的说明仍然
有效，但其中 Server endpoint groups、`SimulationScheduler`、Core state lock 和 completed-step
pipeline 的描述已被本次实现取代。Server 的现行设计以本文和以下文档为准：

- [`../../architecture/server_runtime.md`](../../architecture/server_runtime.md)
- [`../../architecture/locking_model.md`](../../architecture/locking_model.md)
- [`../../../core/mujoco_sim_server/Readme.md`](../../../core/mujoco_sim_server/Readme.md)

本轮不要求向后兼容，只要求当前仓库整体迁移并通过测试。Protocol V2 wire format和 Core 公开
结果类型没有改变。

## 2. 当前完成状态

Server 单写者改造已经完成：

- `ServerApplication` 改为 Pimpl composition root，只负责配置、eCAL、Viewer、Transport、
  SimulationLoop 的构造和生命周期。
- `ServerApplication::Start()` 现在完成初始化并立即启动 simulation thread；`Run()` 在
  headless 模式等待线程，在 GUI 模式于调用线程运行 render loop 后再等待 simulation thread。
- 新增 `SimulationLoop`，独占 `SimulationCore`、调度状态、Sensor runtime、Viewer snapshot、
  runtime metrics 和全部 Core 查询/修改。
- 新增类型化 `ServerEvent` 和有界 `ServerEventQueue`。eCAL callback 只做 payload 校验、一次
  Protobuf 解析和入队，不再直接调用 Core。
- Registry RPC event 使用 promise/future 返回结果；Commit 和 Heartbeat 使用 fire-and-forget
  event。
- Commit、Unregister、Deadline 和 Empty Barrier 的 completed-step 统一进入
  `ApplyEffects()` / `ApplyCompletedStep()`。
- Tick 初发和重传继续复用 Core 的 immutable cached bytes，并在发送前检查 issued identity 是否
  仍为 current。
- Sensor blocking fence 改用 `std::stop_token` 中断，Core/Snapshot 必选参数由 pointer 改为
  reference。
- 删除 `simulation_mutex_`、`step_pipeline_busy_`、旧 Server 主状态锁调度，以及多条重复的
  parse/notify/guarded handler wrapper。
- 删除 endpoint groups 和 `SimulationScheduler` 源码、target 与测试。
- Server 的多个内部 component libraries 合并为一个 `mujoco_sim::server` library；安装态只导出
  该 library 和 `mujoco-sim-server` executable target。

本轮没有实施 `reduce_wrapper.md` 中的 Adapter SDK `ActivateWith`、`CommitWith`、
`UnregisterWith` 等清理。该部分属于后续 SDK Pimpl/Transport 抽取，应独立实施，不能与本次
Server 单写者交付混淆。

## 3. 当前 Server 结构

主要文件：

| 文件 | 职责 |
| --- | --- |
| `src/server_application.cpp` | composition、Start/Run/RequestStop、eCAL ownership 和 teardown |
| `src/simulation_loop.hpp/.cpp` | 唯一 Core owner、事件分派、timer、step effects、发布和指标 |
| `src/server_event.hpp/.cpp` | 类型化 RPC/topic event、payload accounting、RPC cancel |
| `src/server_event_queue.hpp/.cpp` | 有界 MPSC FIFO、batch wait、close 和 queued RPC completion |
| `src/ecal_server_transport.hpp/.cpp` | eCAL service/topic 输入边界及单写者输出 publisher |
| `src/sensor_runtime.hpp/.cpp` | snapshot capture worker 和 stream queue policy |
| `src/sim_viewer.hpp/.cpp` | GUI render 与隔离 snapshot 同步 |
| `src/server_runtime.*` | callback gate、异常边界、send 分类和 runtime metrics |

运行方向：

```text
eCAL callback threads
  -> validate size
  -> parse protobuf once
  -> ServerEventQueue
  -> SimulationLoop thread
  -> SimulationCore
  -> ApplyEffects
  -> Sensor snapshot/fence
  -> Tick/Observation/Status transport output
```

禁止重新让 callback、Sensor worker 或 GUI render thread 直接调用 `SimulationCore`。即使某个 Core
方法看起来只读，例如 `MakeStatus()` 或 resource query，也必须在 SimulationLoop 中执行，以免
重新形成隐含多写者/多读者模型。

## 4. Event queue 契约

固定边界位于 `ServerEventQueue`：

```text
maximum events:         1024
maximum payload bytes:  64 MiB
maximum batch size:     64
protocol payload limit: 4 MiB per callback input
```

必须保持以下行为：

- FIFO 顺序只在成功入队的 event 间保证。
- 每批最多处理 64 个 event，随后必须给 deadline、Tick、status、Viewer 和 metrics timer 执行机会。
- Registry RPC 在队列满时返回 `INTERNAL_ERROR`，消息说明 queue full。
- Server stopping/queue closed 时 Registry RPC 返回 `INTERNAL_ERROR`，不能无限等待 promise。
- Commit 和 Heartbeat 在队列满或关闭时直接丢弃并计入 `queue_rejections`。
- `Close()` 必须原子拒绝新输入，并完成所有 queued RPC promise。
- queue mutex 只保护容器、字节计数、closed 状态和 condition variable；不得在持锁状态下等待
  promise 或处理 Core event。

若后续要增加 event 类型，必须同时补充：

1. `ServerEvent` variant；
2. `EventPayloadBytes()`；
3. RPC cancel 行为；
4. `SimulationLoop::Handle()` overload；
5. queue limit/close 测试。

## 5. 单写者和 step 不变量

SimulationLoop 是模型加载完成后的唯一 Core caller。所有可能完成 `mj_step()` 的入口仍包括：

- normal Commit Barrier；
- deadline/timeout Barrier 缩减；
- unregister；
- local Controller error；
- empty Barrier。

这些入口必须把 `StepCompletedEvent` 交给同一 `ApplyEffects()` 路径。Completed step 的顺序为：

```text
Core returns StepCompletedEvent
  -> SensorRuntime::OnStepCompleted(core)
  -> wait for blocking Sensor jobs with loop stop_token
  -> publish Observer frames
  -> update barrier/step metrics
  -> set next wall-clock Tick deadline
  -> mark status dirty
```

Blocking Sensor stream 会有意暂停唯一 SimulationLoop。暂停期间 Registry RPC、Heartbeat 和 Commit
只能在 event queue 中等待，不能绕过 fence 并发访问 Core。`RequestStop()` 必须能够通过 stop
token 和 Sensor condition 唤醒该 fence。

不要重新引入 `simulation_mutex_` 或 `step_pipeline_busy_`。如果某项功能无法以 event、owning
snapshot 或 immutable effect 表达，应先重新划分所有权，而不是增加跨线程 Core lock。

## 6. Transport 边界

`EcalServerTransport` 保留必要的 wrapper：

- callback active-count gate，确保 teardown 可以停止新 callback 并排空在途 callback；
- transport exception boundary，异常不得穿过 eCAL C++ callback；
- Protobuf parse/serialize boundary；
- eCAL service/subscriber/publisher ownership。

已删除的无语义 wrapper 不应恢复：

- per-message `RegistryEndpoint::Parse()` overload；
- Endpoint 到 ServerApplication 再到通用 parser 的转发链；
- 多个等价 scheduler Notify 方法；
- `StopWithReason()` 到 `RequestStop()` 的单纯转发；
- 分散在不同 step 来源中的 Begin/Finish pipeline 包装。

Transport publisher maps 只由 SimulationLoop 输出路径访问，因此不需要 publisher mutex。输入
callback 捕获 typed handlers，不能捕获或借用 Core。

## 7. 生命周期与关闭顺序

`RequestStop()` 可从任意线程调用且必须幂等。第一次调用：

1. 记录 stop reason；
2. 将正常外部 stop 的 exit code 设为 0，保留已确定的内部错误码 2；
3. 关闭 event queue 并取消 queued RPC；
4. request simulation thread stop token；
5. 唤醒 Sensor blocking wait；
6. 请求 Viewer close。

随后 teardown 顺序必须保持：

```text
join SimulationLoop
  -> stop eCAL inputs
  -> remove callbacks
  -> wait active callbacks idle
  -> reset transport objects
  -> destroy loop/Sensor/Viewer
  -> finalize eCAL when owned by ServerApplication
```

SimulationLoop 退出时，成功路径调用 `core_.MarkStopped(reason)` 并发布最终 `STOPPED` status；错误
路径保持或进入 `ERROR_PAUSED` 并发布最终 status。SDK 依靠该终态唤醒无限等待。

因为 `Start()` 已经启动 simulation thread，测试和嵌入方不能再假设 `Run()` 调用前没有 Tick、
Observation 或 Sensor sample。`server_terminal_state_ecal_test` 已迁移：允许 stop 前收到合法数据，
并额外验证终态后的下一次等待返回 `kStopped`。

## 8. 构建与导出

Server CMake 只构建一个主 library：

```text
build-tree alias:  mujoco_sim::server
installed target:  mujoco_sim::server
executable target: mujoco_sim::mujoco-sim-server
```

以下旧 component target 已删除，不提供兼容 alias：

```text
mujoco_sim::server_config
mujoco_sim::server_runtime
mujoco_sim::server_endpoints
mujoco_sim::server_sensor_runtime
mujoco_sim::server_viewer
```

GUI/Sensor 四种组合均已验证 build、install、consumer 和依赖排除：

| GUI | Sensor | 结果 |
| --- | --- | --- |
| ON | ON | 通过 |
| ON | OFF | 通过 |
| OFF | ON | 通过，headless smoke 通过 |
| OFF | OFF | 通过，headless smoke 和轻量依赖检查通过 |

复现命令：

```bash
bash core/mujoco_sim_server/test/build_matrix.sh \
  "$PWD/install" /tmp/mujoco_sim_server_single_writer_matrix
```

## 9. 验证记录

本轮最终验证：

```text
Server default CTest:              12/12 passed
Adapter SDK CTest:                  7/7 passed
Adapter SDK unit cases:            62/62 passed
Server ASAN/UBSAN CTest:           12/12 passed
ASAN queue + lifecycle rerun:       2/2 passed
strict warnings build:             passed
optional feature build matrix:      4/4 passed
install/package consumer:           passed in every matrix case
git diff --check:                   passed
removed lock/wrapper name scan:     clean in Server sources
```

关键命令：

```bash
colcon build --packages-up-to mujoco_sim_adapter_sdk \
  --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/mujoco_sim_server --output-on-failure
ctest --test-dir build/mujoco_sim_adapter_sdk --output-on-failure
ASAN_OPTIONS=detect_leaks=0 \
  ctest --test-dir build/mujoco_sim_server_asan --output-on-failure
```

TSAN 配置和编译成功，但当前运行环境在进入测试代码前失败：

```text
FATAL: ThreadSanitizer: unexpected memory mapping ...
```

`server_event_queue_test` 和 `server_lifecycle_e2e` 都以相同 runtime 错误退出，没有产生 data race
报告。这是当前 Linux 地址空间/TSAN runtime 限制，不能记作 TSAN 通过，也不能记作代码竞态
失败。换环境后应优先重跑这两项；可尝试 V1.1 handoff 建议的 `setarch x86_64 -R`，但是否允许
关闭 ASLR 取决于宿主权限。

## 10. 测试覆盖

新增 `server_event_queue_test` 覆盖：

- FIFO 和 batch 上限；
- wait 被 producer 唤醒；
- event count 和 payload byte 上限；
- close 后拒绝输入；
- close 完成 queued RPC；
- 多 producer 并发入队。

既有 Server/SDK E2E 继续覆盖：

- register/activate/unregister、timeout、recovery、replacement；
- Commit Barrier 和 local command error 隔离；
- blocking Sensor、Observer 和 stream Tick 对齐；
- Server 最终 `STOPPED` 状态唤醒所有 SDK client role；
- GUI autonomous run；
- status tool human/JSON/watch/stale/recovery；
- 安装态 package consumer。

当前没有独立 transport overload E2E。Queue limit 已被单元测试覆盖，但若要修改 callback 的拒绝
诊断、恶意 malformed traffic 或高负载行为，应新增真实 eCAL transport 测试，而不是只测试
`ServerEventQueue`。

## 11. 当前工作树和提交边界

本 handoff 所在提交包含本次 Server 单写者实现、测试迁移、架构文档，以及用户此前提供并要求
保留的两份 V1.2 计划文档。删除的 Endpoint/Scheduler 文件属于本轮预期变更，不应恢复。

接手后仍应先执行：

```bash
git status --short
git diff --check
```

不要删除或重写 build/install/log 之外无法确认来源的本地改动。计划文档中的历史分析可以提到
旧类名，不应使用全仓字符串替换去清除这些历史记录；只要求现行 Server source 不再依赖旧类。

## 12. 已知限制和后续工作

后续工作不属于本次交付：

1. 按 maintainability plan 继续拆分 `SimulationCore` 内部领域对象和状态机。
2. 独立实施 Adapter SDK Pimpl、Transport、CallbackGate、AdapterSession 和 BlockingMailbox，
   再删除 SDK `XxxWith(lambda)` 测试 wrapper。
3. 将 Sensor queue policy 从 worker 中进一步策略化并增加独立测试。
4. 在支持的环境中重跑 TSAN，并增加 queue overload/malformed/shutdown transport E2E。
5. 继续压低 `simulation_loop.cpp` 文件规模；事件 visitor 已拆为 typed handler，后续可按
   registry/control/timer/output 分类拆文件，但不能拆出第二个 Core owner。
6. 按计划逐步补齐 Core、SDK 和 Protocol 的公开 API/字段契约文档。

特别注意：不要在下一步同时修改线程模型、Core 状态机和 Protocol wire format。单写者边界已经
稳定，后续应以小批次保持当前 12 项 Server 和 7 项 SDK CTest 全绿。

## 13. 建议接手顺序

1. 阅读本文、两份 V1.2 plan、Server runtime 和 locking model。
2. 确认 `SimulationLoop` 是现行代码中唯一 Core owner/caller。
3. 从安装态运行 Server 12 项和 SDK 7 项 CTest。
4. 修改事件类型前先阅读 queue close/RPC promise 测试。
5. 修改 completed-step 行为前先确认五种 step 来源仍统一进入 `ApplyEffects()`。
6. 修改 Sensor blocking 行为前验证 stop token 能中断 fence。
7. 发布前重跑 warnings、ASAN/UBSAN、四种 feature matrix，并在可用环境补跑 TSAN。

后续修改应优先保护四条边界：严格 Protocol V2、单线程访问 Core、所有 step 来源统一后处理、
停止时所有 RPC 和 blocking wait 都可完成。
