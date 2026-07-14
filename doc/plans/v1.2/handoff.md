# MuJoCo Sim Core V1.2 Handoff

## 1. 文档目的

本文面向下一位维护者，记录截至 2026-07-14 的 Server 单写者事件循环与 `SimulationCore`
可维护性重构状态、关键不变量、验证结果和后续边界。本轮实施依据：

- [`mujoco_sim_core_maintainability_refactor_plan.md`](./mujoco_sim_core_maintainability_refactor_plan.md)
- [`reduce_wrapper.md`](./reduce_wrapper.md)
- [`../v1.1/handoff.md`](../v1.1/handoff.md)

V1.1 handoff 中关于 Protocol V2、Core Barrier、Tick cache、SDK 生命周期和可选构建的说明仍然
有效，但其中 Server endpoint groups、`SimulationScheduler`、Core state lock、completed-step
pipeline，以及旧 `SimulationCore` getter/result 类型的描述已被本次实现取代。现行设计以本文和
以下文档为准：

- [`../../architecture/server_runtime.md`](../../architecture/server_runtime.md)
- [`../../architecture/locking_model.md`](../../architecture/locking_model.md)
- [`../../architecture/core_components.md`](../../architecture/core_components.md)
- [`../../architecture/controller_lifecycle.md`](../../architecture/controller_lifecycle.md)
- [`../../architecture/tick_commit_sequence.md`](../../architecture/tick_commit_sequence.md)
- [`../../../core/mujoco_sim_server/Readme.md`](../../../core/mujoco_sim_server/Readme.md)
- [`../../../core/mujoco_sim_core/Readme.md`](../../../core/mujoco_sim_core/Readme.md)

本轮不要求 Core C++ source compatibility，当前仓库调用方已整体迁移。Protocol V2 的字段号、
enum 数值和 wire format 没有改变；`sync.proto` 只新增契约注释。Core 公开结果类型已经统一为
`EngineResult<T>` / `EngineEffects`，旧 getter 和专用 result struct 不再提供兼容层。

## 2. 当前完成状态

Maintainability plan 的阶段 A、B、C、D 已完成。`SimulationCore` 改造包括：

- `SimulationCore` 现在是 move-only、单 Pimpl facade；moved-from 对象只能析构或 move-assign。
- 原 2138 行集中实现已拆成 `ModelRuntime`、`ResourceCatalog`、`AdapterRegistry`、
  `ControllerStateMachine`、`BarrierCoordinator`、`TickStore`、`ObservationScheduler`、
  `SnapshotManager` 和 `StatusProjection` 等真实协作模块。
- 内部模块通过 private 字段和原子操作维护不变量；facade 不再直接修改 registry、barrier 或
  tick 容器，也不再维护 8 个可独立为空的 `unique_ptr`。
- 所有变更操作统一返回 `EngineResult<T>` 或 `EngineEffects`；adapter lifecycle、status dirty 和
  completed step 由同一个 effects channel 交给 `SimulationLoop`。
- Controller/Observer 使用显式状态转换；状态矩阵、非法转换、timeout、replacement、recovery、
  duplicate Commit 和 local error 均有表驱动或 characterization test。
- `LoadModel()` 是事务性的：失败不改变现有模型、session、epoch、Tick 或 snapshot ownership；
  成功替换整个模型 generation。
- Tick handle 保持不可变和单次序列化；Barrier、deferred recovery、actuator zeroing、Observer
  scheduling 和 snapshot generation 不变量保持。
- 公开标量 getter 已收敛为一次一致性 `CoreState` 值快照；资源查询收敛为 `CoreResources`。
- 公开 operational methods 从 29 个降为 20 个，达到计划初始目标；最终 `<=15` 留给后续
  protocol-isolation 阶段。

Server 单写者改造已经完成并迁移到新的 Core effects API：

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

本轮仍没有实施 `reduce_wrapper.md` 中的 Adapter SDK `ActivateWith`、`CommitWith`、
`UnregisterWith` 等清理。该部分属于后续 SDK Pimpl/Transport 抽取，应独立实施，不能与本次
Server 单写者交付混淆。

## 3. 当前 Core 结构

`SimulationCore` 只暴露 model、snapshot、resource、adapter lifecycle、Tick/Commit、deadline、
terminal state 和 projection 操作。主要实现文件：

| 文件 | 职责 |
| --- | --- |
| `src/simulation_core_facade.cpp` | Pimpl 构造/移动、公开查询和 facade 转发 |
| `src/simulation_core.cpp` | snapshot 与少量 Core 入口实现 |
| `src/internal/simulation_core_impl.hpp` | 单一实现对象及内部协作对象 ownership |
| `src/internal/model_runtime.*` | transactional model prepare/commit、MuJoCo state |
| `src/internal/resource_catalog.*` | model-derived observation/command catalog 和 lookup |
| `src/internal/adapter_registry.*` | Controller/Observer session、ownership 和生命周期索引 |
| `src/internal/controller_state_machine.*` | 显式 transition validation 和 state mutation |
| `src/internal/barrier_coordinator.*` | frozen membership、Commit accounting 和 completion |
| `src/internal/tick_store.*` | immutable issued Tick、identity 和 serialization cache |
| `src/internal/observation_scheduler.*` | Observer due calculation 和 completed-step frames |
| `src/internal/snapshot_manager.*` | snapshot create/clone/update 和 generation validation |
| `src/internal/status_projection.*` | wire `ServerStatus` 和 `CoreState` 只读投影 |

`CoreResources` 的两个 span 借用当前 model generation 的 catalog。它们会在下一次成功
`LoadModel()`、owner 被析构或 owner 被 move-assign 时失效；不得跨这些边界保存。`CoreState` 是
拥有数据的值快照，可以安全保存，但只代表调用瞬间。

Core 本身不加锁，也不承诺多线程并发调用。生产环境仍由 `SimulationLoop` 独占全部读写；测试
直接调用 Core 时也应维持单线程所有权。不要为方便查询在 Core 内重新加入 mutex。

## 4. 当前 Server 结构

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

## 5. Event queue 契约

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

## 6. 单写者和 step 不变量

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

## 7. Transport 边界

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

## 8. 生命周期与关闭顺序

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

## 9. 构建与导出

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

## 10. 验证记录

2026-07-13 至 2026-07-14 的最终验证结果：

```text
colcon build:                         9 packages passed
colcon test-result:                  51 tests, 0 failures, 0 skipped
Core GTest cases:                    53 passed
Core quality CTest:                   3/3 passed
Core ASAN/UBSAN CTest:                1/1 passed
Server default CTest:                12/12 passed
Server lifecycle TSAN scope:          3/3 passed
Doxygen warning-as-error:            passed
complexity checker self-tests:        3/3 passed
10,000-step benchmark:               passed
git diff --check:                    passed
```

复杂度门禁当前覆盖 209 个 Core 函数，要求 `CCN <= 15`、`NLOC <= 80`、每个 `.cpp <= 500`
行，并拒绝相对 checked-in baseline 的回退。当前最大实现文件是 `resource_catalog.cpp`，378 行。
baseline key 使用 source-relative path 和完整函数签名，不使用易变的起始行号。

关键复现命令：

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --event-handlers console_cohesion+
colcon test-result --all --verbose

build/quality-venv/bin/python \
  tools/quality/check_simulation_core_complexity.py \
  --lizard build/quality-venv/bin/lizard \
  --baseline tools/quality/simulation_core_complexity_baseline.csv \
  --root .
python3 tools/quality/test_check_simulation_core_complexity.py
```

10,000-step benchmark 的参考结果，不作为跨机器的绝对性能阈值：

```json
{
  "controllers": 8,
  "steps": 10000,
  "serialization_count": 80000,
  "published_messages": 240000,
  "layout_bytes_avoided": 90380000,
  "barrier_p99_us": 1092.51,
  "wall_seconds": 2.68848,
  "cpu_seconds": 2.6521,
  "max_rss_kib": 200896
}
```

TSAN 在本机需通过 `setarch "$(uname -m)" -R` 关闭 ASLR 后运行；suppression 只覆盖调用栈进入
预编译、未插桩 eCAL shared library 的报告。具体命令见 `tools/quality/README.md`。

## 11. 测试覆盖

Core 测试新增或强化覆盖：

- Controller/Observer 全状态转换矩阵和非法 transition；
- transactional model load 和旧 snapshot generation 拒绝；
- register/activate/unregister、activation expiry、timeout、replacement 和 recovery；
- actuator ownership、local error zeroing、deferred zeroing 和 Barrier membership；
- immutable Tick、identity invalidation、duplicate Commit 和 serialization cache；
- Commit、deadline、unregister、local error、empty Barrier 五种 step completion source；
- Observer schedule、status projection、resource catalog 和 `CoreState` counts。

既有 `server_event_queue_test` 继续覆盖：

- FIFO 和 batch 上限；
- wait 被 producer 唤醒；
- event count 和 payload byte 上限；
- close 后拒绝输入；
- close 完成 queued RPC；
- 多 producer 并发入队。

Server/SDK/Demo E2E 继续覆盖：

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

## 12. 当前工作树和提交边界

本 handoff 所在提交包含 `SimulationCore` 模块化、统一 effects API、仓库调用方迁移、测试、
架构文档和质量门禁。它建立在上一提交的 Server 单写者实现之上，不包含新的 Protocol wire
字段，也不包含 Adapter SDK Pimpl 重构。

仓库根目录的未跟踪 `screenshot.png` 是本地 GUI 测试产物，不属于源码提交。build/install/log、
quality venv、sanitizer build tree 和生成的 Doxygen 输出也不得纳入版本控制。

接手后仍应先执行：

```bash
git status --short
git diff --check
```

计划文档中的历史分析会保留旧类名、旧行数和原始建议，用于解释重构动机；不要使用全仓字符串
替换“修正”这些历史数据。判断现状应以本 handoff、architecture 文档和当前源码为准。

## 13. 已知限制和后续工作

后续工作不属于本次交付：

1. Core 公开 operational methods 当前为 20，已达到初始门槛但未达到最终 `<=15`；只有在出现
   清晰的调用聚合边界时再继续收敛，不要为数字增加无语义 wrapper。
2. Core 内部仍使用部分 V2 Protobuf message。后续 protocol-isolation 应先引入稳定领域 command
   和 projection boundary，再考虑拆 proto 文件，不能改变现有字段号或 enum 数值。
3. 独立实施 Adapter SDK Pimpl、Transport、CallbackGate、AdapterSession 和 BlockingMailbox，
   再删除 SDK `XxxWith(lambda)` 测试 wrapper。
4. 将 Sensor queue policy 从 worker 中进一步策略化并增加独立测试。
5. 增加 queue overload/malformed/shutdown transport E2E。
6. 继续压低 `simulation_loop.cpp` 文件规模；事件 visitor 已拆为 typed handler，后续可按
   registry/control/timer/output 分类拆文件，但不能拆出第二个 Core owner。
7. 阶段 F 的 Observer/Sensor/GUI 可选能力隔离和 proto 物理拆分尚未全部完成。

特别注意：`CoreResources` 是 borrowed view，`CoreState` 是 owning value；不能把前者缓存到 model
reload 或 Core replacement 之后。不要在下一步同时修改线程模型、Core 状态机和 Protocol wire
format。单写者边界已经稳定，后续应以小批次保持全仓 51 项 CTest 全绿。

## 14. 建议接手顺序

1. 阅读本文、maintainability plan、Core architecture、Controller lifecycle 和 Tick sequence。
2. 确认 `SimulationLoop` 仍是生产代码中唯一 Core owner/caller。
3. 修改 Core 前先运行 Core GTest、complexity checker self-test 和 baseline gate。
4. 修改 lifecycle 前对照完整状态矩阵；普通拒绝保持 effects 为空，但 Controller-local Commit
   error 必须保留 isolation、zeroing 和可能的 step completion effects。
5. 修改 model/resource/snapshot 前验证 transactional load 和 generation invalidation。
6. 修改 completed-step 行为前确认五种来源仍统一进入 `EngineEffects` / `ApplyEffects()`。
7. 修改事件类型前阅读 queue close/RPC promise 测试；修改 Sensor blocking 前验证 stop token fence。
8. 发布前重跑全仓测试、Doxygen warnings、ASAN/UBSAN、TSAN scope 和性能 benchmark。

后续修改应优先保护六条边界：严格 Protocol V2、单线程访问 Core、transactional model load、
immutable Tick、所有 step 来源统一后处理、停止时所有 RPC 和 blocking wait 都可完成。
