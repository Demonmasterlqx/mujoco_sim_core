# MuJoCo Sim Core V1.3 Handoff

## 1. 文档目的

本文面向下一位维护者，记录截至 2026-07-15 的 Protocol V3、Core/wire 解耦和消息频率优化
最终状态。实现基线提交为：

```text
bc69ea3 feat: complete protocol v3 rate optimization
```

本轮实施依据 [`mujoco_sim_protocol_and_message_rate_optimization_plan.md`](./mujoco_sim_protocol_and_message_rate_optimization_plan.md)。
计划已经完成，不存在待实现阶段或 V2 兼容期。以下文档共同描述现行契约：

- [`../../protocol/v3.md`](../../protocol/v3.md)
- [`../../performance/v1.3/README.md`](../../performance/v1.3/README.md)
- [`../../architecture/core_components.md`](../../architecture/core_components.md)
- [`../../architecture/server_runtime.md`](../../architecture/server_runtime.md)
- [`../../architecture/controller_lifecycle.md`](../../architecture/controller_lifecycle.md)
- [`../../architecture/tick_commit_sequence.md`](../../architecture/tick_commit_sequence.md)

V1.2 handoff 中关于单写者 `SimulationLoop`、有界事件队列、统一 completed-step pipeline 和
可中断 teardown 的不变量仍然有效。其 Protocol V2、Core protobuf 类型、protobuf Tick/Commit、
SDK protobuf API 和旧消息频率说明已经被本文替代。

## 2. 最终状态

V1.3 是一次无兼容层的硬切换：

- 仓库只生成和安装 `mujoco_sim.v3`，只使用 `mujoco_sim/v3/...` topic；
- Tick、Commit 和 Observation 使用统一的稠密二进制 `DataFrame`；
- Core 不再依赖 `mujoco_sim_protocol`、protobuf 或 eCAL；
- Server 是 wire/domain mapper 和消息频率策略的唯一 owner；
- SDK 公开 domain DTO、layout span 和 move-only lease，不公开 generated protobuf；
- Sample Adapter、WBR LQR Demo、Sensor Demo 和 Status Tool 已整体迁移到 V3；
- 正常 Commit 隐式刷新 presence，活跃 Controller 不再发送 per-Tick heartbeat；
- Tick retry、ServerStatus、Observer、subscriber query 和日志均有独立、有界 cadence；
- 编解码、消息频率、重试、生命周期、队列、并发、安装态和 A-D 性能场景均有自动化证据。

不要重新增加 V2 proto、endpoint alias、配置 alias、SDK/Core overload、运行时协商或 fallback。
仓库内旧配置应继续被严格 parser 拒绝。

## 3. 包和依赖边界

现行依赖方向为：

```text
mujoco_sim_protocol ----> mujoco_sim_server <---- mujoco_sim_core
          |                       |
          v                       v
mujoco_sim_adapter_sdk     mujoco_sim_sensor
          |
          +----> sample adapter / demos
```

`mujoco_sim_core` 只拥有领域状态和 MuJoCo 状态。它不能 include 或 link Protocol、protobuf、eCAL，
也不能保存 encoded bytes。`mujoco_sim_protocol` 只拥有 V3 控制面 schema、topic helper、layout hash
和 data-frame codec。Server 同时依赖 Core 与 Protocol，并执行所有双向映射。

生产路径仍由 `SimulationLoop` 独占 `SimulationCore`。Transport callback、Sensor worker、Viewer
thread 和 SDK callback 都不得直接访问 Core，即使调用看起来是只读的也不例外。

## 4. Protocol V3 契约

控制面固定为：

```text
protobuf package: mujoco_sim.v3
protocol major:   3
protocol minor:   0
registry service: mujoco_sim.v3.<server>.registry
```

protobuf 只用于低频控制消息：资源查询、注册、激活、注销、Sensor stream 查询、SessionHeartbeat、
ControllerFault、ServerStatus 以及 Sensor header/status。Tick、Commit 和 Observation 不使用
protobuf，也不携带 adapter/resource 名称。

高频 data frame 使用固定 80-byte little-endian header：

| 字段 | 契约 |
| --- | --- |
| magic | `MJSYNC3\0` |
| framing | major 1, minor 0 |
| kind | Tick 1, Commit 2, Observation 3 |
| flags | 只有 Tick 可使用 `RECOVERY_PROOF` |
| identity | `session_id + epoch + tick_id` |
| layout | 完整 128-bit fingerprint |
| sequence | Observation 从 1 递增；Tick/Commit 为 0 |
| payload | `scalar_count` 个连续 IEEE-754 `double` |

总长度必须精确等于 `80 + scalar_count * 8`，单 frame 上限为 4 MiB。codec 逐字段读写，不能改为
packed struct、native-endian memcpy 或依赖宿主 alignment。解析顺序必须保持：transport size limit、
结构校验、overflow-safe 长度校验、复用 buffer 解码、事件入队、最后由单写者校验领域 identity 和值。

主要实现：

- `core/mujoco_sim_protocol/include/mujoco_sim/protocol/data_frame.hpp`
- `core/mujoco_sim_protocol/src/data_frame.cpp`
- `core/mujoco_sim_protocol/proto/mujoco_sim/v3/sync.proto`
- `core/mujoco_sim_protocol/include/mujoco_sim/protocol/topics.hpp`

## 5. Core 领域边界

Core public boundary 位于 `core/mujoco_sim_core/include/mujoco_sim/core/domain.hpp`。注册、layout、
session、heartbeat、fault、status、Tick、Commit 和 Observation 都使用 Core-owned enum/struct。
generated protobuf enum 和 message 只能出现在 Server mapper 与 SDK transport 内。

注册时按请求资源顺序冻结 dense layout，并预计算：

- observation/command descriptor；
- 每个资源的 `{offset, dimension}`；
- observation/command scalar count；
- 128-bit canonical layout fingerprint；
- command range 校验所需 slot。

Commit 热路径只校验 count、identity、fingerprint、finite、range 和 dense values，不构造资源名
map/set，也不查询 catalog。Duplicate Commit 使用每个 scalar 的 IEEE-754 bit pattern 比较，因此
`+0.0` 与 `-0.0` 不视为相同 payload。

`TickStore` 继续拥有 immutable issued domain handle、`current` 状态和 invalidation。Encoded bytes
移到 `SimulationLoop` 的 `EncodedTickCache`：每个 issue generation 只 encode 一次，初发和 retry
共享完全相同的 bytes；completion、replacement、unregister、model reload 和 stop 都必须清理 cache。

Commit、Deadline、Unregister、LocalError 和 EmptyBarrier 五种 step completion source 仍必须通过
同一个 effects/completed-step pipeline。不要绕过该路径直接调用 `mj_step()`、发布 Observation 或
推进 Sensor fence。

## 6. Server 运行时与消息频率

Transport callback 只执行 size check、parse/decode、结构校验和 enqueue。结构损坏且不能可信提取
identity 的 frame 只计 malformed 后丢弃；结构完整但 session/layout/count/value 非法的 Commit
必须入队，由 Core 按领域规则拒绝或隔离。

默认运行策略如下：

| 项目 | 默认值与行为 |
| --- | --- |
| Controller heartbeat | 250 ms；最近成功 Commit 未超过周期时 suppress |
| Observer heartbeat | 250 ms，约 4 Hz |
| Tick retry | 5/10/20 ms，最多 3 次，不越过 Controller deadline |
| ServerStatus periodic | 100 ms，稳态约 10 Hz |
| lifecycle status min interval | 20 ms，burst 合并上限 50 Hz |
| terminal status | teardown 前立即发布，不受 min interval 限制 |
| Observer SDK default | 显式 `period_s = 0.01`，即 100 Hz |
| Observer maximum | 默认 500 Hz；超限拒绝，不 clamp |
| every-step Observer | 默认禁止，需 Server 和 registration 双方显式开启 |
| transport subscriber sample | 1000 ms |
| Sensor subscriber sample | 200 ms |
| runtime log summary | 1000 ms |

有效 Commit 同时刷新 presence 和当前 Tick 的完成状态；Heartbeat 只刷新 presence，永远不能延长
Barrier deadline、满足 required membership 或完成 recovery proof。Transport `Send()==true` 仍只
表示交给 transport，不代表 Server 已接受 Commit。

每个 Controller/Barrier 有独立 retry state。收到 valid Commit 后立即停止该 Controller retry；
Barrier completion/invalidation 清理全部相关 retry state。不得恢复固定周期广播整个 open Barrier。

ServerStatus 使用 `kNone < kPeriodicOnly < kLifecycle < kTerminal` urgency 合并。正常 step、Commit、
Heartbeat 和 metrics 只能产生 periodic urgency；终态必须先发布再 teardown。

事件队列边界保持为 1024 events、64 MiB payload 和每批 64 events。新增事件必须同步实现 variant、
实际 payload byte accounting、typed handler、close/cancel 行为和 queue limit 测试。

## 7. SDK 契约

Controller 使用 `ControllerLayout`、`TickLease` 和 dense command span；Observer 使用
`ObservationLease`。Lease 是 move-only buffer owner，暴露的 span 只在 lease 生命周期内有效。
一个 client 仍只允许一个 waiter。

Controller 处理规则：

- `WaitForTick()` 返回 Tick identity、simulation time、recovery proof 和 observation span；
- `Commit()` 校验 lease 属于当前 client/session，并校验 exact count、finite 和本地 range；
- `FailTick()` 通过低频 protobuf `ControllerFault` 报告不可生成有效 command 的情况；
- duplicate Tick 尚未提交时不重复交给用户；已有 Commit 时只重发 cached exact bytes；
- 相同 identity 但 payload 不同会使当前 session 进入 terminal protocol error；
- `Stop()` 只停止本地 wait/heartbeat，不伪造远端注销成功；
- `Unregister()` 失败时保留 session identity，允许重试。

Observer mailbox 默认 capacity 1，溢出采用 latest/drop-oldest。Transport sequence gap 和本地 eviction
都计入 dropped count。Observer registration 必须明确 cadence；缺失 cadence 是 wire error。

Heartbeat loop 和 mailbox wait 必须用 condition variable/stop token 唤醒，不允许不可中断 sleep。
Callback teardown 顺序保持 disable、移除 callback、等待其他 active lease、销毁 endpoint；当前 callback
线程停止或销毁最后一个 owner 的测试不能删除。

## 8. Sensor、Demo 与配置

Sensor stream 继续与 Controller data plane 分离。默认 non-blocking latest queue 深度为 1；blocking
stream 仍保持 lossless fence 语义，并且 `RequestStop()` 必须能中断等待。Sensor lazy capture 根据
200 ms subscriber cache 决定是否采集，不得在每次 capture/publish 前后查询 subscriber。

以下内置调用方已迁移到 V3 dense API：

- `mujoco_sim_sample_adapter`
- `mujoco_sim_wbr_lqr_demo`
- `mujoco_sim_sensor_demo`
- Server status tool

配置 source 是各包的 schema YAML；generated DTO 不能手工修改。Server 关键字段位于
`core/mujoco_sim_server/config/server_config.yaml`，包括独立的 commit/session timeout、heartbeat、
retry、status、Observer rate、subscriber sampling 和 summary period。旧字段没有 alias。

## 9. 性能验收与证据

最终授权并实现的 wire-byte 门槛是：

| 指标 | 门槛 | 实测 |
| --- | ---: | ---: |
| Tick bytes | 比 V1.2 至少减少 50% | 50.2% |
| 8-scalar Commit bytes | 比 V1.2 至少减少 25% | 26.2% |
| A-D 每场景总数据面 bytes | 比 V1.2 至少减少 45% | 46.4% 至 48.5% |

保留完整 80-byte identity header 后，8-scalar Commit 固定为 `80 + 8 * 8 = 144` bytes；V1.2
对照为 195 bytes。不要把 Commit 门槛恢复为 50%，该门槛与固定头契约数学上不兼容。

A-D 均使用 GCC 13.3、Release、同一模型、CPU 0、2,000 warmup steps、10,000 measured steps，
每个版本/场景保留五轮原始数据。完整中位数、场景定义和限制见
[`../../performance/v1.3/README.md`](../../performance/v1.3/README.md)。原始证据位于同一目录：

- `v12_scenarios_abc.ndjson` / `v13_scenarios_abc.ndjson`
- `v12_scenario_d.ndjson` / `v13_scenario_d.ndjson`
- `v12_core_benchmark.ndjson` / `v13_core_benchmark.ndjson`
- `v13_codec.json`

场景 D 的 Sensor worker 是 non-blocking latest，仿真会跑赢 100 Hz worker，因此 frame count 随调度
变化且允许 drop。报告中的 bytes、CPU、barrier latency 和 queue watermark 是实测值，不是外推值。

## 10. 最终验证记录

2026-07-15 的最终结果：

```text
colcon packages:                       9 passed
colcon test-result:                   56 tests, 0 errors, 0 failures, 0 skipped
Server optional build matrix:          4 configurations passed
installed package consumers:           passed
Protocol malformed/golden/allocation:  passed
message-rate and retry E2E:             passed
warnings as errors:                     passed
Core/SDK complexity gates:              passed
Doxygen warnings as errors:             passed
ASAN/UBSAN scopes:                       passed
git diff --check:                        passed
production/install V2 scan:              clean
```

TSAN targets 可以构建，但当前宿主运行时报：

```text
FATAL: ThreadSanitizer: unexpected memory mapping
```

因此 TSAN 没有被记为通过；这是明确记录的宿主 memory-mapping 阻塞。应在支持关闭 ASLR 且能正常
建立 TSAN shadow mapping 的宿主机重新运行。不要把该记录改写为 race-free 证明。

## 11. 复现命令

完整构建与测试：

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --event-handlers console_cohesion+
colcon test-result --all --verbose
```

Server 四种可选构建和 installed consumer：

```bash
bash core/mujoco_sim_server/test/build_matrix.sh \
  "$PWD/install" /tmp/mujoco_sim_server_v13_matrix
```

复杂度门禁：

```bash
build/quality-venv/bin/python \
  tools/quality/check_simulation_core_complexity.py \
  --lizard build/quality-venv/bin/lizard \
  --baseline tools/quality/simulation_core_complexity_baseline.csv \
  --root .

build/quality-venv/bin/python \
  tools/quality/check_adapter_sdk_complexity.py \
  --lizard build/quality-venv/bin/lizard \
  --root .
```

TSAN 在合适宿主上的建议入口：

```bash
setarch "$(uname -m)" -R env \
  TSAN_OPTIONS="halt_on_error=1:suppressions=$PWD/tools/quality/tsan.supp" \
  ctest --test-dir build/mujoco_sim_server_tsan --output-on-failure \
    -R 'server_test|server_event_queue_test|server_lifecycle_e2e'
```

性能 target 需要配置 `MUJOCO_SIM_BUILD_PERFORMANCE_BENCHMARK=ON`。正式对比必须继续使用相同
compiler、Release build、CPU affinity、模型、warmup、step count 和五轮顺序运行；跨机器的绝对
RTF 不能直接比较。

## 12. 修改时必须保持的不变量

后续变更至少要守住以下边界：

1. Core 保持 protocol/eCAL/protobuf independent，且只有 SimulationLoop 调用生产 Core。
2. Tick/Commit/Observation 固定头、identity、fingerprint、count 和总长度必须严格校验。
3. 一个 issue generation 只 encode 一次，retry 必须复用 exact bytes。
4. Heartbeat 不能完成或延长 Barrier；正常活跃 Commit 窗口 Controller heartbeat 为 0 Hz。
5. 五类 step source 必须走同一 effects pipeline，Sensor/Observer completed-step 顺序不能分叉。
6. Terminal status 必须在 teardown 前立即发布并唤醒 SDK waiter。
7. Callback 不读 session table、不调用 Core，queue payload 按实际内容而非 capacity 计费。
8. Lease span 不得跨 lease 生命周期保存；callback teardown 不能等待自身。
9. Subscriber query、status、Observer、retry 和 summary 的频率不能重新与 step rate 绑定。
10. 不增加 V2/compat/fallback；新增公开 API 要补 Doxygen、测试和 installed consumer 覆盖。

## 13. 接手后的建议阅读顺序

1. 本文和 V3 protocol 文档；
2. `data_frame.hpp/.cpp` 与 V3 `sync.proto`；
3. Core `domain.hpp`、`simulation_core.hpp` 和 `TickStore`；
4. Server `simulation_loop.*`、`ecal_server_transport.*`、`server_event.*`；
5. SDK `sync_adapter_client.*`、`observer_client.*` 和内部 mailbox/session 类型；
6. `message_rate_e2e`、data-frame tests 和 A-D benchmark；
7. 完整优化计划的第 13 至 16 节验收项。

当前计划没有遗留实现任务。后续若修改 wire format、cadence、queue policy 或 lease ownership，应先
更新契约与基准，再按本 handoff 的验证矩阵完整复跑。
