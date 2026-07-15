# MuJoCo Sim Core V1.3 协议与消息频率优化计划

> 状态：实施计划，基于 2026-07-14 的 V1.2 handoff 与当前主干源码。
> 范围：Protocol、Core protocol-isolation、Server/SDK 高频数据面、消息频率和相关测试。
> 兼容性：**不提供任何 wire、topic、配置或 C++ API 向后兼容；采用一次性 Protocol V3 硬切换。**

本计划的权威输入为 [V1.2 handoff](../v1.2/handoff.md)、
[Server runtime](../../architecture/server_runtime.md)、
[Core components](../../architecture/core_components.md)、
[Controller lifecycle](../../architecture/controller_lifecycle.md) 和
[Tick/Commit sequence](../../architecture/tick_commit_sequence.md)。历史计划只用于理解动机；模块名称、
线程边界和测试基线以这些文档与当前源码为准。

## 1. 决策摘要

V1.3 不在 V2 旁增加可协商的 `FLAT_PROTO`，也不保留 named fallback。最终仓库只有一套协议：

```text
低频控制面：Protocol V3 Protobuf
高频数据面：固定 80-byte header + little-endian IEEE-754 float64 array
Sensor 大数据：保留 header + raw payload framing，但迁移到 V3 类型和 endpoint
```

本轮必须同时完成四项工作：

1. 先将 V2 Protobuf 从 Core 领域模型、Adapter record、Barrier、Tick cache 和 effects 中移出；
2. 用唯一的 V3 稠密数据面替换命名 `Tick`、`AdapterCommit`、`ObservationFrame`；
3. 修正 Heartbeat、Status、Tick 重传、Observer 和 subscriber 查询的高频行为；
4. 一次性迁移 Server、SDK、sample、demos、tools、测试和文档，随后删除全部 V2 产物。

明确不采用以下设计：

- 不做 `DataPlaneEncoding` 能力协商；
- 不同时发布 named/flat 两份数据；
- 不保留 V2 topic、service alias、proto package 或转换 wrapper；
- 不保留接受/返回 Protobuf 的旧 SDK/Core 公共 API；
- 不预留未实现的 encoding enum；
- 不用更大的 timeout 掩盖发送、序列化或调度开销。

## 2. 当前状态与证据

### 2.1 必须保持的 V1.2 架构边界

V1.2 已建立以下不可回退的不变量：

- `SimulationLoop` 是模型加载后唯一的 `SimulationCore` caller；
- eCAL callback 只校验、解析并向有界 `ServerEventQueue` 入队；
- queue 上限为 1024 events、64 MiB payload、每批最多 64 events；
- Commit、Deadline、Unregister、LocalError、EmptyBarrier 五种 step 来源统一经过
  `ApplyEffects()` / `ApplyCompletedStep()`；
- Tick identity 在 Core 中校验，初发和重传复用同一份不可变 wire bytes；
- Sensor blocking fence 可由 `stop_token` 中断；
- SDK 的 `Stop()` 只做本地取消，远端清理由可重试 `Unregister()` 完成；
- callback gate、单 waiter、duplicate Commit 幂等和终态唤醒语义必须保持。

V1.3 只改变领域/协议边界和输出策略，不引入第二个 Core owner，不恢复 Core mutex，也不拆出
可并发修改 Core 的数据面线程。

### 2.2 当前热点的源码依据

| 热点 | 当前位置与行为 | V1.3 方向 |
| --- | --- | --- |
| Core 依赖 wire 类型 | `simulation_core.hpp`、registry、Barrier、TickStore、ObservationScheduler 直接保存 `mujoco_sim::v2` | Core 改为纯领域 command/result/projection |
| Tick 字符串与序列化 | `TickStore::Insert()` 保存完整 `v2::Tick` 并在 Core 内序列化 | Core 只产生稠密 frame；Server codec 序列化并缓存 |
| Commit 字符串校验 | `commit_engine.cpp` 每次构造 name set/map 并查询 catalog | 注册时生成 dense command slots；每次只校验 count/finite/range |
| per-Tick Heartbeat | `sync_adapter_client_receive.cpp` 对新 Tick 和未提交 duplicate Tick 发布 Heartbeat | Commit 隐式刷新 presence；独立低频 keepalive 仅在无 Commit 时发送 |
| Status 绕过周期 | `ApplyCompletedStep()` 每步执行 `status_dirty_ = true` | step/commit/heartbeat 只更新投影；不触发即时全量 Status |
| Tick 无限重传 | `SimulationLoop` 对 open Barrier 每 5 ms 重发 | 有界指数退避，每个 Barrier 默认最多 3 次重传 |
| Observer 默认全速 | SDK 默认 `every_n_steps = 1`，协议未设置也表示每步 | wire 必须显式 cadence；SDK 默认 10 ms，Server 执行频率上限 |
| subscriber 热查询 | Tick 发送前后各 `GetSubscriberCount()`；Sensor 也频繁查询 | 独立低频采样；发送路径只执行 `Send()` |

### 2.3 基准要求

修改前必须保存 V1.2 Release/headless 基准，不能只引用历史 handoff 数字。固定模型、CPU affinity
和 eCAL 配置，至少覆盖：

```text
A. 1 Controller，32 observation scalars，8 command scalars
B. 8 Controllers（现有 performance_8_controller.xml）
C. 1 Controller + 1 Observer（100 Hz）
D. C + 一个 non-blocking Sensor stream
```

每个场景预热 2,000 steps，测量 10,000 steps，记录原始 JSON。指标包括消息数、bytes、p50/p95/p99、
CPU、RSS、RTF、barrier latency、每 step allocation 数以及 queue high watermark。

## 3. 目标与非目标

### 3.1 功能目标

- 注册后 session layout 完全冻结；热路径不携带资源名称；
- Controller Tick、Commit 和 Observer frame 只传定长身份信息与连续 scalar array；
- Core 不 include generated protobuf header，不保存 Protobuf message，不负责 wire serialization；
- Server callback 完成一次有界解析后只向现有事件队列提交领域 event；
- Commit 继续是 Barrier 的唯一成功证明；Heartbeat 永远不能满足当前 Barrier；
- 所有旧的 timeout/recovery、duplicate、zeroing 和五类 completed-step 语义保持；
- 正常运行时消息频率由仿真 Tick、显式 Observer cadence 和低频 timer 决定，不由状态字段变化次数决定。

### 3.2 性能验收目标

参考场景 A 的 Release/headless 验收目标：

| 指标 | 目标 |
| --- | ---: |
| Tick encode p95 | `< 50 us` |
| SDK Tick decode p95 | `< 30 us` |
| Commit encode p95 | `< 30 us` |
| Server Commit decode + validate p95 | `< 30 us` |
| 稳态 codec allocation | buffer 预热后 `0/message` |
| Tick wire bytes | 相同 scalar 数下比 V2 至少减少 50% |
| 8-scalar Commit wire bytes | 比 V2 至少减少 25% |
| A-D 总数据面 wire bytes | 每个场景比 V1.2 至少减少 45% |
| 正常 Commit 活跃时 Controller Heartbeat | `0 Hz` |
| 无 lifecycle 变化时 ServerStatus | 默认 `10 Hz`，不随 step rate 增长 |
| 默认 Observer | `100 Hz`（SDK 显式写入 10 ms cadence） |
| 正常 Tick 初发 | 每 Controller/Barrier 恰好 1 条 |
| Tick 重传 | 每 Controller/Barrier 默认最多 3 条 |
| 高频路径资源名查找 | `0/step` |

绝对延迟用于防止明显回退；最终结论还必须同时报告相对 V1.2 的 CPU、RTF 和 p99。不同机器之间
不直接比较绝对 RTF。

### 3.3 非目标

- 不改变 MuJoCo step 的 wall-clock 调度算法；
- 不并行执行 Controller，也不引入控制流水线；
- 不优化 Viewer rendering 或 Sensor capture kernel；
- 不将 image/point cloud 塞入 Controller Tick；
- 不在缺少 V3 profiling 证据时增加 shared-memory ring buffer；
- 不改变 blocking Sensor 的 lossless 语义，但默认配置继续使用 non-blocking latest；
- 不为旧二进制、旧配置或旧源码调用方提供迁移期运行模式。

## 4. 目标分层与依赖

最终依赖方向为：

```text
                         +-----------------------+
                         | mujoco_sim_protocol   |
                         | V3 proto + data codec |
                         +-----------+-----------+
                                     |
                    +----------------+----------------+
                    |                                 |
          +---------v---------+             +---------v---------+
          | server transport  |             | adapter SDK       |
          | wire <-> domain   |             | wire <-> SDK view |
          +---------+---------+             +-------------------+
                    |
          +---------v---------+
          | SimulationLoop    |  single writer / policy owner
          +---------+---------+
                    |
          +---------v---------+
          | SimulationCore    |  pure domain + MuJoCo, no protobuf/eCAL
          +-------------------+
```

`mujoco_sim_core` 不再依赖 `mujoco_sim_protocol`。Server 同时依赖 Core 和 Protocol，并拥有所有
wire/domain mapper。SDK 依赖 Protocol，但公开 API 不暴露 generated protobuf 类型。

### 4.1 Core 领域类型

在 Core public/internal boundary 引入不依赖 wire 的类型：

```cpp
struct SessionIdentity {
  std::uint64_t session_id;
  std::uint64_t epoch;
};

struct ResourceDescriptor { /* name, dimension, MuJoCo metadata */ };

struct ResourceSpan {
  std::uint32_t offset;
  std::uint32_t dimension;
};

struct AdapterLayout {
  LayoutFingerprint fingerprint;       // 128 bit
  std::vector<ResourceDescriptor> observations;
  std::vector<ResourceDescriptor> commands;
  std::vector<ResourceSpan> observation_spans;
  std::vector<ResourceSpan> command_spans;
  std::uint32_t observation_scalar_count;
  std::uint32_t command_scalar_count;
};

struct ControllerCommit {
  SessionIdentity session;
  std::uint64_t tick_id;
  LayoutFingerprint layout;
  std::vector<double> commands;
};

struct IssuedTick {
  IssuedTickIdentity identity;
  LayoutFingerprint layout;
  double simulation_time_s;
  bool recovery_proof;
  std::vector<double> observations;
  mutable std::atomic<bool> current;
};
```

Registration command/receipt、heartbeat、controller fault、observer delivery 和 status projection 同样使用
领域 enum/struct。Wire enum 只能在 Server mapper 和 SDK transport 内出现。

### 4.2 Serialization cache ownership

Core 的 `TickStore` 继续拥有 issued identity、current/invalidation 和 immutable domain handle，但不再
保存 `serialized_bytes`。SimulationLoop 增加 `EncodedTickCache`：

```text
Core OpenIssuedTicks() returns same immutable handles
  -> cache miss by issue_generation: encode once and store shared bytes
  -> initial publish uses cached bytes
  -> retry looks up the same cache entry and reuses exact bytes
  -> Core invalidation/lifecycle effect erases cache entry
```

验收不变量仍是每个 `issue_generation` 恰好 encode 一次。不能通过重编码后比较内容来证明等价。
`EncodedTickCache` 只由 SimulationLoop 访问，不需要 mutex。

## 5. Protocol V3 控制面

### 5.1 硬切换规则

最终协议标识：

```text
protobuf package: mujoco_sim.v3
protocol major:   3
protocol minor:   0
service:          mujoco_sim.v3.<server>.registry
topics:           mujoco_sim/v3/<server>/...
```

endpoint 已包含 major version，因此 request/header 不再重复携带 `protocol_major/minor`。V3.0 不做
minor negotiation。Server 和 SDK 只编译 V3；连接错误版本表现为 endpoint 不相交，而不是运行时 fallback。

### 5.2 控制消息范围

Protobuf 只承担以下低频消息：

- `GetAvailableResources`；
- `RegisterAdapter`、`ActivateAdapter`、`UnregisterAdapter`；
- `GetSensorStreams`；
- `SessionHeartbeat`；
- `ControllerFault`；
- `ServerStatus` 和 Sensor stream header/status。

删除 Protobuf `Header`、`Tick`、`ObservationBlock`、`ObservationFrame`、`ActuatorCommandBlock` 和
`AdapterCommit`。控制请求使用最小必要身份；service/topic 已确定的 `server_name` 不在 payload 中重复。

### 5.3 注册与布局

`RegisterAdapterRequest` 保留 adapter ID、role、资源名、timeout 和 Observer cadence。V3 变化：

- 不含 protocol version 和 encoding capabilities；
- Observer 必须且只能设置 `every_n_steps` 或 `period_s`；
- Controller 可设置 `commit_timeout_ms`；`0` 使用 Server default；
- `session_timeout_ms` 独立表示没有任何有效 presence 的失联时间。

`RegisterAdapterResponse` 返回：

```proto
message ResourceSlot {
  ResourceDescriptor descriptor = 1;
  uint32 scalar_offset = 2;
}

message ResourceLayout {
  repeated ResourceSlot observations = 1;
  repeated ResourceSlot commands = 2;
  bytes layout_fingerprint = 3;       // exactly 16 bytes
  uint32 observation_scalar_count = 4;
  uint32 command_scalar_count = 5;
}
```

顺序由 Server 按 request 中的名称顺序冻结。SDK 注册时构建 name -> `{offset, dimension}` 索引，之后
热路径只使用 offset/span。layout fingerprint 继续使用显式 endian、length-prefixed canonical encoding；
实现从 Protobuf descriptor 输入改为 Core domain descriptor 输入。

### 5.4 Session identity 简化

session ID 在 Server process 内从不复用，因此 Activate、Unregister、Heartbeat、Fault 和 data frame
使用 `session_id + epoch` 定位。`adapter_id` 和 role 在注册后由 session table 得到，不在每条消息重复。
共享 Commit topic 由 session table O(1) 路由；不再增加语义重复的 `adapter_handle`。

## 6. V3 高频数据面

### 6.1 固定 frame 格式

Tick、Commit 和 Observation 使用同一个 80-byte header。所有整数和 `double` bit pattern 均为
little-endian；payload 是连续 `scalar_count * 8` bytes。

| Offset | Size | 字段 | 约束 |
| ---: | ---: | --- | --- |
| 0 | 8 | magic | ASCII `MJSYNC3\0` |
| 8 | 2 | framing major | `1` |
| 10 | 2 | framing minor | `0` |
| 12 | 1 | kind | `1=Tick, 2=Commit, 3=Observation` |
| 13 | 1 | flags | Tick 可含 `RECOVERY_PROOF`；其余 bit 必须为 0 |
| 14 | 2 | header size | `80` |
| 16 | 4 | frame size | 必须等于 `80 + scalar_count * 8` |
| 20 | 4 | scalar count | 必须等于注册 layout 的精确数量 |
| 24 | 8 | session ID | 非零且精确匹配 |
| 32 | 8 | epoch | 精确匹配当前 model generation |
| 40 | 8 | tick ID | Barrier/frame identity |
| 48 | 8 | layout fingerprint low | 128-bit fingerprint 的低 64 bit |
| 56 | 8 | layout fingerprint high | 128-bit fingerprint 的高 64 bit |
| 64 | 8 | sequence | Observer 从 1 递增；Tick/Commit 为 0 |
| 72 | 8 | simulation time bits | `std::bit_cast<uint64_t>(double)` |

codec 必须以逐字段 read/write 实现，禁止 `reinterpret_cast` packed C struct；禁止依赖宿主 alignment、
padding 或 native endian。编译期断言 `sizeof(double) == 8` 和 `numeric_limits<double>::is_iec559`。

### 6.2 Frame 语义

**Tick**

- payload 是注册 layout 顺序的 pre-step observation scalars；
- `RECOVERY_PROOF` 是 issued frame 的不可变属性，不因重传改变；
- 初发与所有重传必须逐 byte 相同，因此不能在 flags 中标记“这是重传”；
- `sequence == 0`，scalar count 等于 `observation_scalar_count`。

**Commit**

- payload 是 layout 顺序的完整 command scalars；只表达成功 command；
- header identity 和 simulation time 从对应 Tick 复制；
- scalar count 必须等于 `command_scalar_count`；每个值必须 finite 且满足预计算 range；
- duplicate 判断比较 identity 与每个 scalar 的 IEEE-754 bit pattern，不再构造 name map/set；因此
  `+0.0` 与 `-0.0` 不属于相同 duplicate payload。

**Observation**

- payload 是完成 step 后的 observation scalars；
- `sequence` 在 Observer session 内严格递增；
- 不参与 Barrier，不重传；SDK mailbox 使用 latest/drop-oldest 语义。

Controller 无法产生有效 command 时发送低频 `ControllerFault` Protobuf，其中包含 session、epoch、tick、
fault code 和有界诊断文本。有效 Fault 仍进入事件队列并调用 Core 的 local-error transition，可能完成
当前 Barrier。正常 Commit 不携带 status 或 error string。

### 6.3 Codec API 与边界检查

Protocol library 提供：

```cpp
struct DataFrameView {
  DataFrameHeader header;
  std::span<const std::byte> scalar_bytes;
};

bool EncodeDataFrame(const DataFrameHeader&, std::span<const double>,
                     std::string* reusable, CodecError*);
bool ParseDataFrame(std::span<const std::byte>, DataFrameView*, CodecError*);
bool DecodeScalars(const DataFrameView&, std::span<double> reusable,
                   CodecError*);
```

解析顺序固定为：

1. callback 先执行 4 MiB transport payload limit；
2. 检查最小 header、magic、framing version、kind、flags 和 `header_size`；
3. 使用 overflow-safe arithmetic 校验 `frame_size == 80 + count * 8`；
4. 在 4 MiB 上限内把 scalar 解码到复用 vector；
5. 构造只含 header 和 dense values 的 `CommitEvent`，按实际内容计费后入队；
6. SimulationLoop/Core 再根据单写者持有的 session table 校验 session/epoch/tick/layout/count，随后
   校验 finite/range 和 duplicate。

结构损坏且无法可信提取 identity 的 frame 只计 malformed 并丢弃。callback 不得读取 session table、
不得调用 Core，也不得依据 scalar count 或 command value 改变 lifecycle。结构完整但
session/tick/layout/count/command value 非法的 Commit 必须入队并由 Core 按既有规则拒绝或隔离。

### 6.4 Topic

```text
mujoco_sim/v3/<server>/tick/<adapter_id>          DataFrame(kind=Tick)
mujoco_sim/v3/<server>/commit                     DataFrame(kind=Commit)
mujoco_sim/v3/<server>/observation/<adapter_id>   DataFrame(kind=Observation)
mujoco_sim/v3/<server>/session_heartbeat          SessionHeartbeat protobuf
mujoco_sim/v3/<server>/controller_fault           ControllerFault protobuf
mujoco_sim/v3/<server>/status                     ServerStatus protobuf
mujoco_sim/v3/<server>/sensor/<stream_name>       V3 sensor framed payload
```

同一 topic 只有一种 payload 类型。V2 endpoint helper 和字符串常量全部删除。

## 7. Liveness、Barrier 与消息降频

### 7.1 Controller Heartbeat

将“session presence”和“当前 Tick commit deadline”拆开：

- valid Commit 同时更新 `last_presence_at` 和 `last_commit_tick`；
- valid Heartbeat 只更新 `last_presence_at`；
- 当前 Barrier deadline 从 Tick issue time 和 `commit_timeout_ms` 得到；
- Heartbeat **不能**延长 Barrier deadline、满足 required membership 或完成 recovery proof。

SDK Controller 增加可停止的低频 keepalive loop，发送条件为：

```text
now - last_successful_commit_publish >= heartbeat_period
AND now - last_heartbeat_publish >= heartbeat_period
AND session is active/timed-out and not locally stopped
```

默认 `heartbeat_period_ms = 250`，且注册校验要求 `heartbeat_period <= session_timeout / 3`。连续 Commit
期间 Heartbeat 为 0；Server paused 或暂时没有 Tick 时保持 4 Hz presence。

Transport `Send()==true` 只表示已交给 transport，不代表 Server 接受 Commit，这一语义不变。

### 7.2 Observer Heartbeat

Observer 没有 Commit，固定使用低频 heartbeat loop。默认 250 ms，最小允许 100 ms。每次包含最新已消费
sequence，但新 frame 到达不立即触发 Heartbeat。

### 7.3 ServerStatus

Core effects 将 `bool status_changed` 替换为语义化 urgency：

```cpp
enum class StatusUrgency { kNone, kPeriodicOnly, kLifecycle, kTerminal };
```

合并规则取最高 urgency：

- step、正常 Commit、Heartbeat、Tick/sequence/metrics 更新：`kPeriodicOnly`；
- register/activate/timeout/recovery/error/unregister/model generation：`kLifecycle`；
- `STOPPED` / `ERROR_PAUSED` 最终状态：`kTerminal`。

发布策略：

```text
periodic:  默认每 100 ms 发布完整快照
lifecycle: 标记 pending，在 status_min_interval 边界合并发布
terminal:  立即发布一次，不受 min interval 限制，随后才 teardown
```

默认 `status_min_interval_ms = 20`，因此 lifecycle burst 上限为 50 Hz。删除
`ApplyCompletedStep()` 中的 `status_dirty_ = true`。正常 1 kHz、10 秒且无 lifecycle 变化时应发布约
100 条 Status，而不是约 10,000 条。

### 7.4 Tick 重传

每个 open Barrier/Controller 保存独立 retry state：

```cpp
struct TickRetryState {
  std::uint32_t attempts;
  Clock::time_point next_retry;
  std::chrono::milliseconds delay;
};
```

默认策略：初发后 5 ms、10 ms、20 ms 最多重传三次；下一次时间不得越过该 Controller 的 Barrier
deadline。收到 valid Commit 后停止该 Controller 的 retry。Barrier completion/invalidation 删除全部 retry
state。相同 Tick 到达 SDK 时：

- 尚未 Commit：不重复交给用户，也不发送 per-Tick Heartbeat；
- 已有 cached Commit：最多重发完全相同的 cached bytes；
- 相同 identity 但 payload 不同：session terminal protocol error；
- stale Tick：丢弃并计数。

`OpenIssuedTicks()` 仍可返回 current handles，但 retry timer 只发布尚未满足且未耗尽次数的 Controller，
不再每 5 ms 广播整个 open Barrier。

### 7.5 Observer cadence

Wire registration 缺少 cadence 时返回 `INVALID_ARGUMENT`。SDK 的 convenience default 明确写入
`period_s = 0.01`，而不是依赖 wire default。Server 配置：

```text
observer_max_rate_hz: 500
allow_every_step_observer: false
```

超过上限直接拒绝，不静默 clamp。测试/debug 若需要 every-step，必须显式开启 Server 配置并显式注册。
控制器不得通过 ObserverFrame 获取高频闭环输入。

### 7.6 Subscriber 与日志采样

- Tick/Observation/Status 正常发送路径只调用一次 `Send()`；
- subscriber count 由 SimulationLoop 的 1 Hz metrics timer 更新；
- publisher 创建和连续 send failure 可触发一次额外查询，但必须 rate-limit；
- Sensor lazy capture 的 subscriber cache 由 SensorRuntime 每 200 ms 更新，capture/publish 不再前后各查询；
- subscriber 从 0 变为非 0 最多延迟一个采样周期开始 capture；该延迟写入契约与测试；
- per-Tick/per-Commit 日志只允许 `trace`，且 disabled level 前不得构造动态诊断字符串；
- summary 默认 1 秒或更低频，不随消息数触发。

## 8. Server 实现计划

### 8.1 Transport 输入

`EcalServerTransport` 增加 binary Commit callback 和 `ControllerFault` callback。callback 继续使用 V1.2
active-count gate 与 exception boundary。成功解析后提交：

```cpp
CommitEvent { core::ControllerCommit message; }
ControllerFaultEvent { core::ControllerFault fault; }
HeartbeatEvent { core::SessionHeartbeat heartbeat; }
```

`EventPayloadBytes()` 必须按 vector capacity-independent 实际内容计费。新增 event 时同步补齐 variant、
RPC cancel（topic event 无 promise）、typed `Handle()` 和 queue close/limit 测试。

### 8.2 SimulationLoop

SimulationLoop 继续拥有：

- Core 和所有 domain/wire mapper 调用；
- `EncodedTickCache` 与 per-controller `TickRetryState`；
- status urgency、周期和 hard rate cap；
- subscriber/transport metrics timer；
- Sensor blocking fence 和 completed-step 后处理顺序。

`ApplyCompletedStep()` 的顺序不变：Sensor snapshot/fence -> Observer publish -> metrics -> next Tick deadline。
协议改造不能把 encode/send 移到并发访问 Core 的 worker。

### 8.3 Transport 输出

- `PublishTick()` 接收 immutable encoded bytes + issued identity；发送前仍调用
  `core.IsIssuedTickCurrent()`；
- `PublishObservation()` 使用 session reusable buffer 编码一次；
- publisher maps 仍由 SimulationLoop 单线程访问；
- adapter unregister/replacement 时删除 publisher、encoded cache 和 retry state；
- `PublishResult` 不再包含每次发送前后的 subscriber snapshots，只记录 attempted/sent/bytes/error。

## 9. SDK 公共 API

### 9.1 Controller

删除公开 Protobuf `Tick`、`RegisterAdapterResponse` 和 `CommandValue` hot API，改为：

```cpp
class ControllerLayout {
 public:
  ScalarSpan Observation(std::string_view name) const;
  ScalarSpan Command(std::string_view name) const;
  std::size_t observation_scalar_count() const noexcept;
  std::size_t command_scalar_count() const noexcept;
};

class TickLease {
 public:
  std::uint64_t tick_id() const noexcept;
  double simulation_time_s() const noexcept;
  bool recovery_proof() const noexcept;
  std::span<const double> observations() const noexcept;
};

TickWaitResult WaitForTick(std::chrono::milliseconds timeout,
                           TickLease* tick, std::string* error);
bool Commit(const TickLease&, std::span<const double> commands,
            std::string* error);
bool FailTick(const TickLease&, FaultCode, std::string_view message,
              std::string* error);
```

`TickLease` move-only 并拥有/共享 mailbox buffer，span 在 lease 生命周期内有效。Commit 校验 lease 属于
当前 Client/session、command count、finite 和本地已知 range；SDK 仍不能把 transport send 当作 Server ack。

callback 直接 decode 到复用 buffer，避免 Protobuf -> vector -> caller 的多次复制。一个 Client 仍只允许
一个 waiter。duplicate Tick 与 cached Commit 行为保持，但缓存对象改为 encoded V3 bytes。

### 9.2 Observer

Observer 使用 `ObservationLease` 暴露 sequence、tick、simulation time 和 scalar span。默认 mailbox
capacity 为 1，溢出 drop-oldest，并把 transport sequence gap 与本地 eviction 计入 dropped count。
registration info 和 layout 使用 SDK domain DTO，不暴露 proto。

### 9.3 生命周期

Pimpl、`AdapterSession`、`CallbackGate`、`BlockingMailbox` 和 typed Transport 保持。新增 heartbeat loop 必须：

- 使用 condition variable/stop token 唤醒，不能用不可中断 sleep；
- `Stop()` 可立即停止本地 wait/heartbeat，但不伪造远端 Unregister 成功；
- `Unregister()` 失败后 session identity 保留，允许重试；
- callback teardown 先 disable、移除 callback、等待其他 lease，再销毁 endpoint。

## 10. 配置

所有配置只修改 schema source，再由 `mujoco_sim_config` generator 生成；不手改 generated DTO。建议最终字段：

```yaml
tick_timeout_ms: 1000
session_timeout_ms: 1000

controller_heartbeat_period_ms: 250
observer_heartbeat_period_ms: 250

tick_retry_initial_ms: 5
tick_retry_backoff_multiplier: 2.0
tick_retry_max_delay_ms: 20
tick_retry_max_attempts: 3

status_period_ms: 100
status_min_interval_ms: 20

observer_max_rate_hz: 500
allow_every_step_observer: false

transport_subscriber_sample_period_ms: 1000
sensor_subscriber_sample_period_ms: 200
log_summary_period_ms: 1000
```

旧字段不做 alias/deprecation 映射；字段改名时同步迁移仓库内所有 YAML，并让未知旧字段继续被严格 parser
拒绝。校验包括正数、合理上限、retry delay 不超过 timeout、heartbeat 与 session timeout 的比例，以及
`status_min_interval_ms <= status_period_ms`。

## 11. 分阶段实施

每一阶段结束均要求相关 package build/test 通过；Phase 3 的 V3 切换必须作为一个仓库级原子交付，不能
发布只迁移一半的 Server 或 SDK。

### Phase 0：冻结基准与 characterization

1. 保存第 2.3 节四个场景的 V1.2 原始 benchmark；
2. 增加 Heartbeat/Status/retry/subscriber query 的消息计数 characterization；
3. 增加五种 completed-step source 和 shutdown fence 的现状断言；
4. 为 V2 Tick/Commit 记录 bytes 与 allocation baseline。

退出条件：数据可复现，测试能在未优化代码上稳定暴露当前频率。

### Phase 1：Core protocol-isolation

1. 增加 domain identity、resource、registration、commit、heartbeat、fault、status 类型；
2. ResourceCatalog 生成 dense spans、command slots 和 domain layout fingerprint；
3. AdapterRegistry/BarrierCoordinator 不再保存 Protobuf Commit/layout；
4. TickStore/ObservationScheduler 改为 immutable dense domain frames；
5. Server 添加临时的 V2 wire/domain mapper 以保持该阶段可测试；
6. 移除 Core 对 generated protobuf 和 protocol library 的 CMake/package dependency。

临时 mapper 仅用于让 Phase 1 独立验证，不进入 V1.3 最终产物，也不构成兼容设计。

退出条件：扫描 `core/mujoco_sim_core` 不出现 `sync.pb.h`、`mujoco_sim::v2`、protobuf container 或
`Serialize*`；Core 全部状态机/Barrier/benchmark 测试通过。

### Phase 2：消息频率语义

1. 拆分 presence deadline 与 Barrier commit deadline；
2. Commit 更新 presence，Controller heartbeat 改为 suppressed low-rate loop；
3. status effects 改为 urgency，删除 per-step immediate publish；
4. Tick retry 改为 per-controller 有界退避；
5. Observer cadence 变为必填并执行 rate cap；
6. subscriber query 移出 send/capture 热路径；
7. 增加频率 metrics 和测试。

退出条件：第 13.2 节频率测试全部通过，五种 step source、timeout/recovery 和 stop 行为无回退。

### Phase 3：Protocol V3 原子切换

1. 新增 V3 control proto、data frame codec、golden bytes 和 malformed corpus；
2. Server mapper、event、transport、encoded Tick cache 全部切换 V3；
3. SDK 切换 TickLease/ObservationLease/dense Commit API；
4. sample adapter、WBR LQR、Sensor Demo、status tool 和 probes 全量迁移；
5. Sensor frame header、topic 和 discovery 迁移 V3；
6. 删除 V2 proto、generated targets、topic helpers、layout mapper 和旧 SDK API；
7. 更新 architecture、protocol、README、配置示例和 package consumer。

退出条件：全仓只存在 V3 endpoint/wire；所有 build matrix 组合和安装态 consumer 通过。

### Phase 4：复用、benchmark 与清理

1. 为 codec string/vector 预留固定容量并验证稳态零 allocation；
2. 优化 dense observation copy spans，禁止每 Tick MuJoCo ID/name lookup；
3. 输出 V1.2/V1.3 同场景对比报告；
4. 运行 warnings、complexity、Doxygen、ASAN/UBSAN、TSAN scope；
5. 删除临时 instrumentation 和 Phase 1 mapper。

只有 profiling 显示 protobuf control/status 或一次 payload copy 成为主要瓶颈，才另立后续 shared-memory
方案；不在本计划中提前实现。

## 12. 文件级修改清单

| 模块 | 主要修改 |
| --- | --- |
| `mujoco_sim_protocol/proto` | 用 V3 control schema 替换 V2 monolithic sync schema |
| `mujoco_sim_protocol/include/src` | 新增 `data_frame` codec；topics/layout fingerprint 改 V3；stream header 迁移 |
| `mujoco_sim_core/include` | 公共 facade 改 domain command/result/projection，移除 protobuf |
| `mujoco_sim_core/src/internal/resource_catalog.*` | 预计算 observation spans、command slots、scalar counts |
| `adapter_registry.*` / state machines | 保存 domain layout、presence/deadline，不保存 wire message |
| `barrier_coordinator.*` / `commit_engine.cpp` | dense commit、byte-stable duplicate、预计算 range/write plan |
| `tick_store.*` / `observation_scheduler.*` | dense immutable frame、identity/invalidation，不序列化 |
| `status_projection.*` / effects | domain status 与 `StatusUrgency` |
| `server_event.*` / queue | dense Commit、Fault、Heartbeat events 与 payload accounting |
| `ecal_server_transport.*` | V3 codec boundary、单次 Send、低频 subscriber sample |
| `simulation_loop.*` | encoded cache、retry state、status rate cap、metrics timer |
| `sensor_runtime.*` | V3 stream header/topic、subscriber cache sampling |
| `mujoco_sim_adapter_sdk` | V3 Transport、lease/span API、heartbeat suppression、reusable buffers |
| samples/demos/tools | 全量迁移新 SDK DTO、layout/span API 和 V3 endpoint |
| docs/tests/config | 删除 V2 说明，更新架构、配置、golden/E2E/perf gates |

## 13. 测试与验证

### 13.1 Protocol/codec

- 固定 golden bytes 覆盖三种 frame kind、正负/零/无穷/NaN bit pattern 的 round trip；
- magic/version/kind/flags/header size/frame size/count 的逐项拒绝；
- `count * 8` overflow、截断、尾随 bytes、超过 4 MiB；
- unaligned input span 和跨 endian golden fixture；
- exact 16-byte layout fingerprint 与 scalar count；
- fuzz/property test：任意输入不越界、不崩溃、不产生超限 allocation；
- encode 到预留 buffer 后 allocation count 为零。

NaN/Inf 可由通用 codec 无损解析；Controller command 的领域校验必须拒绝它们。

### 13.2 消息频率

在 1 kHz、10 秒稳定运行中验证：

```text
Commit:                约 10,000 / Controller
Controller Heartbeat:  0（允许启动/停止边界最多 2 条）
ServerStatus:           100 ± timer tolerance
Observer default:       1,000 ± phase tolerance
Tick retransmission:    正常路径为 0
subscriber queries:     每 publisher 约 10，不能与 Tick 数成比例
```

停止 Tick 后 Controller heartbeat 恢复约 4 Hz。Lifecycle burst 受 20 ms hard interval 合并；最终
`STOPPED`/`ERROR_PAUSED` 必须立即发布且 SDK waiter 被唤醒。

### 13.3 Core 与故障语义

- 注册 layout offset/count 与 model catalog 一致；
- wrong session/epoch/tick/layout/count；
- missing/extra commands、NaN/Inf、range violation；
- identical duplicate Commit 幂等，conflicting duplicate 触发 local error；
- Tick payload conflict 使 SDK session terminal；
- 首包/Commit 丢失、三次 retry 耗尽、deadline timeout；
- Heartbeat 持续时仍不能绕过当前 Barrier deadline；
- timeout proof/recovery pending/next-Tick recovery；
- replacement、unregister、model reload 清理 encoded cache/retry state；
- Commit、Deadline、Unregister、LocalError、EmptyBarrier 各自恰好完成一次 step；
- actuator zeroing 与 Sensor/Observer completed-step 顺序保持。

### 13.4 Queue、并发和关闭

- binary Commit/Fault/Heartbeat 的 event byte accounting；
- 1024 event、64 MiB、64 batch 三个边界保持；
- malformed callback 不访问 Core，queue full 时 topic event 丢弃并计数；
- Registry RPC close 后完成 promise；
- SDK callback teardown、single waiter、mailbox eviction、heartbeat stop；
- blocking Sensor fence 中 RequestStop 可中断；
- TSAN scope 验证 callback、heartbeat、Stop/Unregister 和 mailbox 竞态。

### 13.5 全仓门禁

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --event-handlers console_cohesion+
colcon test-result --all --verbose

bash core/mujoco_sim_server/test/build_matrix.sh \
  "$PWD/install" /tmp/mujoco_sim_server_v13_matrix

git diff --check
```

此外执行现有 Core/SDK complexity、warnings、Doxygen、ASAN/UBSAN 和可运行环境中的 TSAN gate。
TSAN 若仍因宿主 memory mapping 限制无法运行，必须明确记录为未验证，不能记为通过。

## 14. 可观测性

新增或调整累计 metrics：

```text
tick_frames / tick_bytes / tick_encode_us
commit_frames / commit_bytes / commit_decode_us / commit_validate_us
observation_frames / observation_bytes
controller_heartbeats / observer_heartbeats
status_periodic / status_lifecycle / status_terminal / status_coalesced
tick_retransmissions / retry_exhausted
malformed_data_frames / stale_frames / layout_mismatches
subscriber_count_queries
encoded_tick_cache_hits / misses / entries
event_queue_rejections / high_watermark
```

metrics 记录不得在热路径创建 resource-name label。按 adapter/resource 的高基数明细只在 debug snapshot
按需生成。1 秒 summary 报告 rate、平均 bytes、retry ratio、queue watermark、RTF 和 barrier latency，
不逐 Tick 输出。

## 15. 风险与控制

| 风险 | 控制措施 |
| --- | --- |
| flat array 错位 | 注册返回 offset/dimension；frame 校验 session/epoch/fingerprint/exact count；golden test |
| raw double 跨平台 | 逐字段 little-endian codec、IEC-559 static assert、禁止 packed struct |
| Core 隔离扩大改动面 | Phase 1 先建立 domain boundary，并完整跑状态机/Barrier characterization |
| 减少重传后首包丢失 | 三次有界退避、duplicate 幂等、deadline/timeout/recovery fault injection |
| Heartbeat 错误延长 Barrier | presence 和 commit deadline 分字段、独立测试 |
| Status 降频影响终态 | terminal urgency 无条件立即发送后再 teardown |
| subscriber cache 导致启动延迟 | Sensor 200 ms、其他 1 s 明确上限与连接/断开测试 |
| reusable buffer 生命周期错误 | move-only lease、单 owner mailbox、ASAN/TSAN 和 callback teardown tests |
| V3 全量切换遗漏调用方 | 全仓 V2 扫描、9-package build、build matrix、installed consumer 和全部 E2E |

## 16. 最终验收清单

### 架构

- [ ] `SimulationLoop` 仍是生产路径唯一 Core caller；
- [ ] Core 不依赖 protobuf、protocol 或 eCAL；
- [ ] callback 只 parse/validate/enqueue，不直接调用 Core；
- [ ] 五种 step completion source 仍走同一 effects pipeline；
- [ ] Tick 每 issue generation 只 encode 一次，retry 复用 exact bytes；
- [ ] Stop、RPC、heartbeat 和 blocking wait 均可完成。

### 协议

- [ ] 仓库只生成/安装 `mujoco_sim.v3`；
- [ ] 无 V2 proto、topic、service alias、config alias 或 SDK/Core compatibility overload；
- [ ] Tick/Commit/Observation wire 不含 adapter/resource 名称；
- [ ] 三类 frame 严格校验固定头、identity、fingerprint、count 和总长度；
- [ ] Commit 预计算 slots，热路径无字符串 map/set/catalog lookup；
- [ ] SDK 公开 lease/span/domain DTO，不公开 generated protobuf。

### 频率与性能

- [ ] Commit 活跃时 Controller Heartbeat 为 0 Hz；
- [ ] Observer Heartbeat 默认 4 Hz；
- [ ] ServerStatus 稳态默认 10 Hz，completed step 不绕过周期；
- [ ] terminal status 在 teardown 前立即发布；
- [ ] Tick retry 有 per-controller 退避和最大次数；
- [ ] Observer wire cadence 必填，SDK default 为显式 100 Hz；
- [ ] subscriber query 不在每次 Send/capture 前后执行；
- [ ] codec 预热后稳态零 allocation，latency 达到第 3.2 节门槛；
- [ ] Tick 减少至少 50%、8-scalar Commit 减少至少 25%，A-D 每个场景总数据面减少至少 45%；
- [ ] V1.2/V1.3 benchmark 使用相同模型、build type 和运行条件并保留原始结果。

### 质量

- [ ] Protocol malformed/golden/fuzz、Core lifecycle、Server queue/rate、SDK concurrency 测试通过；
- [ ] 全仓 colcon、四种 Server build matrix、installed consumers 和 E2E 通过；
- [ ] warnings、complexity、Doxygen、ASAN/UBSAN 通过；TSAN 通过或明确记录环境阻塞；
- [ ] `rg` 扫描确认生产代码和安装清单无 V2/compat/fallback 残留；
- [ ] architecture、protocol、README、配置示例与最终实现一致。

## 17. 完成定义

V1.3 的完成标准不是“新增 fast topic 可用”，而是仓库已经只有一个可测试、可安装、可观测的 V3
协议，Core 与 wire 解耦，所有内置调用方完成硬切换，并且消息频率与性能门禁有自动化证据。

在上述清单全部满足前，不删除性能 instrumentation，不宣称协议优化完成；在 V2/flat 双路径、自动
fallback 或旧 SDK overload 仍存在时，也不得将其视为 V1.3 最终状态。
