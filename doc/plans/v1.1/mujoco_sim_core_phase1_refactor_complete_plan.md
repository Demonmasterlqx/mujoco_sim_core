# mujoco_sim_core 第一阶段重构完整方案

> 适用仓库：`https://github.com/Demonmasterlqx/mujoco_sim_core`
> 基线提交：`5a7e8cd3f78406883ea7e72c3c1ad5b27fd467b8`
> 当前协议基线：MuJoCo Simulator Protocol V1.2
> 目标协议：MuJoCo Simulator Protocol V2.0
> 决策：V2.0 是一次性破坏性切换，不提供旧协议、旧 API、旧配置或旧二进制的向后兼容。

---

## 1. 文档目标

本文依据 [`doc/plans/v1/handoff.md`](../v1/handoff.md) 和基线提交的实际代码，定义第一阶段重构的最终实施方案。

本阶段同时解决以下问题：

- `core/mujoco_sim_core/src/simulation_core.cpp` 约 1900 行，模型、资源、Adapter、Barrier、Observer、Snapshot 和状态逻辑集中在单一实现中。
- `core/mujoco_sim_server/src/ecal_server.cpp` 约 1900 行，应用生命周期、eCAL endpoint、调度、GUI、Sensor、日志和指标相互耦合。
- `SimulationCore::OpenTick()` 每次返回完整 Protobuf 副本。
- `EcalServer::PublishTicks()` 每次发布和重传都重新序列化 Tick。
- `SimulationCore::IsTickIssued()` 通过双方 `SerializeAsString()` 做完整 payload 比较。
- 每个 Tick 重复携带完整 `ResourceLayout`，SDK 也对 Tick layout 做完整序列化比较。
- legacy Controller lifecycle 与 role-aware Adapter lifecycle 同时存在，Core、Server、SDK、状态和测试都有双轨逻辑。
- protocol minor 使用 `std::min(client, server)` 协商，Sensor catalog 也接受旧 minor。
- Server 配置仍保留已弃用且无效的 `startup_timeout_ms`。
- `EcalServer` 在 blocking Sensor fence 期间持有 `simulation_mutex_`，阻止其他无需等待 Sensor 的 Core 操作。
- Server 构建无条件查找 GLFW 和 PNG、编译 MuJoCo `simulate`，纯 headless 产物仍携带 GUI 依赖。

本方案不是在原结构上继续增加包装层，而是建立单一、明确的 V2 架构边界。仓库内 Server、SDK、状态工具、Sample Adapter 和 Demo 必须在同一个变更集内完成升级并重新构建。

---

## 2. 当前基线

### 2.1 包与测试

当前 `colcon list --topological-order` 包含 9 个包：

1. `mujoco_sim_config`
2. `mujoco_sim_protocol`
3. `mujoco_sim_core`
4. `mujoco_sim_sensor`
5. `mujoco_sim_server`
6. `mujoco_sim_adapter_sdk`
7. `mujoco_sim_sample_adapter`
8. `mujoco_sim_sensor_demo`
9. `mujoco_sim_wbr_lqr_demo`

基线测试结果：

```text
50 tests, 0 errors, 0 failures, 0 skipped
```

实施前必须重新 clean build/test，并把结果、运行环境和依赖版本记录到：

```text
doc/refactor/phase1_baseline.md
```

### 2.2 当前公开入口

Core 当前公开以下两套生命周期入口：

```cpp
RegisterController(...)
ActivateController(...)
UnregisterController(...)

RegisterAdapter(...)
ActivateAdapter(...)
UnregisterAdapter(...)
```

`RegisterAdapter(role=CONTROLLER)` 内部重新构造 `RegisterControllerRequest` 并调用旧入口。`ServerStatus` 同时输出 `controllers` 和 `adapters`，同一 Controller 状态被重复编码。`SyncAdapterClient` 仍直接使用 Controller 专用 service，而 `ObserverClient` 使用 Adapter service。

这些双轨入口在 V2 中全部删除，不保留 wrapper、alias、重载或转换 service。

### 2.3 当前锁和 step 路径

当前 Server 主要使用：

- `mutex_`：Core 状态、指标、dirty flag 和部分应用状态。
- `simulation_mutex_`：串行化 completed-step 路径，并跨越 blocking Sensor fence。
- `publisher_mutex_`：Publisher map 和 `Send()`。
- `CallbackState`：停止接收 callback 并等待 callback 排空。

Commit、deadline、unregister 和 empty Barrier 都可能完成一次 step，但各路径分别执行 Observation 收集、Sensor enqueue、blocking wait、日志和通知。重构必须把这些路径收敛到同一个 step completion pipeline。

---

## 3. 总体目标

第一阶段完成后必须满足：

1. `SimulationCore` 是薄领域门面，不直接保存全部 Controller、Observer、Barrier、Tick 和 Snapshot 实现细节。
2. `ServerApplication` 是组合根；endpoint、调度、viewer 和 sensor runtime 各自拥有明确职责。
3. 协议只存在 V2.0；所有 endpoint、消息和 SDK 只接受精确 V2.0。
4. Controller 和 Observer 只使用统一 Adapter lifecycle。
5. 每个 Adapter 的每个 Tick 只构建、canonicalize 和序列化一次。
6. Tick 只携带 `layout_hash`，完整 layout 只在注册结果中发送。
7. Tick 发布批次拥有缓存对象的共享生命周期，解锁发布不会产生悬空指针或 `string_view`。
8. 所有 completed-step 来源产生同一种 `StepCompletedEvent`。
9. eCAL `Send()`、日志 I/O 和 blocking Sensor wait 不持有 Core state mutex。
10. GUI、camera stream 和 MuJoCo 来源可以独立配置构建。
11. Sample、WBR、Observer、Sensor Demo 和状态工具全部迁移到 V2 并通过 E2E。
12. 现有仿真状态机和数据语义保持不变。

---

## 4. 非目标

第一阶段不实现：

- ROS 2 Control Adapter。
- InProcess Controller transport。
- transport-neutral Adapter SDK。
- 多 World 或多模型并行实例。
- Reset/Pause/Resume/Step control plane。
- Checkpoint、回滚或持久化。
- 多 Controller actuator 仲裁。
- 向量 command；actuator command 仍为一个标量 `double`。
- 动态 Stream catalog。
- 图像压缩、视频编码或 GPU 直通。
- `SensorStreamClient` 内建跨 stream 时间同步。
- 运行期 V1/V2 桥接、代理、转换器或双协议 endpoint。

---

## 5. 必须保持的运行时不变量

破坏性切换只作用于源码 API、wire、配置和构建接口。以下运行时语义不可改变：

- Server 是唯一真实 `mjModel/mjData` 持有者、唯一 `mjData.ctrl` 写入者和唯一 `mj_step()` 调用者。
- 模型加载是事务性的；加载或 keyframe 初始化失败不得破坏当前有效模型。
- Tick 打开时固定 required 和 recipient 集合；新激活 Controller 从下一 Tick 加入 Barrier。
- ACTIVE Controller 参与 Barrier；TIMED_OUT Controller 只接收恢复 Tick。
- Command 在全部 required Commit 到齐前不得写入 `data->ctrl`。
- 一个 completed Tick 恰好调用一次 `mj_step()`。
- 无 Controller 时 empty Barrier 仍按模型 timestep 推进，且延迟后不补跑。
- timeout、unregister 和 local command error 只隔离对应 Adapter，并可缩小当前 Barrier。
- 恢复 Commit 只证明连接恢复，不应用其中的 command；下一 Tick 才恢复 ACTIVE。
- Observer 永不参与 Barrier；every-N-step 和 simulation-time period 调度保持锁相。
- 所有真正执行 `mj_step()` 的路径都必须生成 Observer frame 并调度 Sensor capture。
- GUI 和 Sensor worker 只使用 Snapshot，不直接持有主 `mjData`。
- Snapshot clone 复制源 snapshot 当时的数据，不回读 Core 的当前数据。
- 单个 Stream/source/Adapter 错误不得终止其他独立资源。
- callback teardown 顺序保持：停止新 callback、排空 callback、停止 worker、销毁 endpoint、最后处理 eCAL runtime。

---

## 6. 目标架构

### 6.1 Core

```text
mujoco_sim_core
├── SimulationCore
├── model
│   ├── ModelState
│   └── SnapshotManager
├── resources
│   └── ResourceCatalog
├── adapters
│   └── AdapterRegistry
├── synchronization
│   ├── BarrierCoordinator
│   └── TickStore
├── observation
│   └── ObservationScheduler
└── status
    └── StatusBuilder
```

内部组件放在 `mujoco_sim::core::detail`，不导出稳定 ABI，不引入不必要的虚函数。`SimulationCore` 负责组合和跨组件事务，不再自己保存所有容器。

### 6.2 Server

```text
mujoco_sim_server
├── ServerApplication
├── SimulationScheduler
├── RegistryEndpoint
├── ControlEndpoint
├── ObservationEndpoint
├── StatusEndpoint
├── SensorRuntime
├── ViewerRuntime
├── ServerLogger
└── RuntimeMetrics
```

```text
main
  └── ServerApplication
        ├── SimulationCore
        ├── SimulationScheduler
        ├── RegistryEndpoint
        ├── ControlEndpoint
        ├── ObservationEndpoint
        ├── StatusEndpoint
        ├── SensorRuntime
        └── ViewerRuntime
```

删除公开 `EcalServer` 类型和 `ecal_server.hpp`。`main`、安装态 package consumer 和测试直接构造 `ServerApplication`。

---

## 7. Protocol V2.0 一次性切换

### 7.1 文件和命名空间

删除：

```text
core/mujoco_sim_protocol/proto/mujoco_sim/v1/sync.proto
```

新增：

```text
core/mujoco_sim_protocol/proto/mujoco_sim/v2/sync.proto
```

Protobuf package 改为：

```proto
package mujoco_sim.v2;
```

C++ 使用 `mujoco_sim::v2`。构建和安装树只生成、导出 V2 header，不生成或安装 V1 header。

Endpoint 名称包含协议 major，避免旧进程误连：

```text
mujoco_sim.v2.<server_name>.registry
mujoco_sim/v2/<server_name>/tick/<adapter_id>
mujoco_sim/v2/<server_name>/commit
mujoco_sim/v2/<server_name>/adapter_heartbeat
mujoco_sim/v2/<server_name>/observation/<adapter_id>
mujoco_sim/v2/<server_name>/sensor/<stream_name>
mujoco_sim/v2/<server_name>/status
```

### 7.2 版本规则

保留 `protocol_major/protocol_minor` 作为消息自描述和拒绝诊断字段，但规则固定为精确匹配：

```text
protocol_major == 2
protocol_minor == 0
```

删除：

- `effective_protocol_major`
- `effective_protocol_minor`
- `std::min(request_minor, server_minor)`
- minimum-minor 分支
- minor 0 的 TIMED_OUT wire state 特例
- client/server 降级或协商测试

任何非 2.0 请求返回 `VERSION_MISMATCH`；任何非 2.0 topic payload 直接拒绝并计入低基数协议错误指标。

### 7.3 统一 Adapter lifecycle

V2 只保留：

```proto
RegisterAdapterRequest / RegisterAdapterResponse
ActivateAdapterRequest / ActivateAdapterResponse
UnregisterAdapterRequest / UnregisterAdapterResponse
```

删除 Controller 专用 lifecycle 消息和 Registry service method。`RegisterAdapterRequest.role` 必须为 `CONTROLLER` 或 `OBSERVER`。

Controller 约束：

- `command_names` 至少一个。
- observation 可为空。
- delivery 不得设置。
- actuator 继续独占。

Observer 约束：

- observation 至少一个。
- command 必须为空。
- delivery 缺省为 `every_n_steps=1`。

`SyncAdapterClient` 继续作为同步 Controller SDK 类型，但其 `registration()` 返回 `RegisterAdapterResponse`，内部只调用统一 Adapter service。

### 7.4 统一身份字段和状态

V2 中统一使用 `adapter_id`：

- `Tick.adapter_id`
- `AdapterCommit.adapter_id`
- `AdapterHeartbeat.adapter_id`
- `AdapterStatus.adapter_id`

删除 Controller 专用 `Heartbeat` 消息和 heartbeat topic。Controller 与 Observer 都发送 `AdapterHeartbeat`，通过 `role` 区分；Controller 使用 Header tick ID 报告最后接收的 Tick，Observer 继续携带最后 observation sequence。

删除 `ControllerStatus` 和 `ServerStatus.controllers`。`ServerStatus.adapters` 是唯一 lifecycle 状态表。`AdapterStatus` 必须覆盖现有诊断信息：

- role、session、state、message。
- state since tick、timeout count。
- last heartbeat tick。
- last delivery tick。
- last commit tick。
- required for barrier。
- Observer sequence 和 dropped frame count。

状态工具只遍历 `adapters`，按 role 展示 Controller 或 Observer，不再合并两张表。

### 7.5 Layout hash

`ResourceLayout` 在注册响应中携带完整 descriptor 和固定 16-byte `layout_hash`：

```proto
message ResourceLayout {
  repeated ResourceDescriptor observations = 1;
  repeated ResourceDescriptor commands = 2;
  bytes layout_hash = 3;  // exactly 16 bytes
}
```

`Tick` 删除完整 layout，只携带相同 hash：

```proto
message Tick {
  Header header = 1;
  string adapter_id = 2;
  bytes layout_hash = 3;  // exactly 16 bytes
  repeated ObservationBlock observations = 4;
  ControllerState controller_state = 5;
}
```

SDK 不向 Tick 补回 layout。业务代码必须从 `client.registration().layout()` 读取布局。

Canonical 输入使用显式 length-prefixed binary encoding，不直接依赖 Protobuf 的普通序列化结果。编码顺序固定为：

1. canonical format version。
2. observation section marker 和数量。
3. 每个 observation descriptor 的全部 wire 字段，按字段号编码。
4. command section marker 和数量。
5. 每个 command descriptor 的全部 wire 字段，按字段号编码。

字符串按 UTF-8 byte length + bytes；整数按固定 little-endian；bool 为单 byte；double 使用 IEEE-754 bit pattern 的 little-endian 表示。descriptor 和资源顺序参与 hash。

首版算法固定为 FNV-1a 128，输出按 big-endian 写入 16 bytes。Protocol 包提供唯一实现和固定测试向量；Core、Server、SDK 和测试不得各自实现 hash。

SDK 收到 Tick 时必须验证：

- hash 长度恰好为 16。
- hash 与 registration layout 完全一致。
- adapter/session/epoch/version 一致。

hash 不匹配是当前 session 的 terminal protocol error，不猜测、不重新发现、不继续 Commit。

---

## 8. Endpoint identifier

统一校验以下字段：

- `server_name`
- `adapter_id`
- `stream_name`

MuJoCo resource name、frame ID 和模型内对象名不套用该规则。

规则：

```regex
^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$
```

Protocol 包增加：

```text
core/mujoco_sim_protocol/include/mujoco_sim/protocol/identifier.hpp
core/mujoco_sim_protocol/src/identifier.cpp
```

```cpp
enum class IdentifierKind {
  kServer,
  kAdapter,
  kStream,
};

bool ValidateIdentifier(std::string_view value,
                        IdentifierKind kind,
                        std::string* error);
```

校验层次：

1. 配置和 SDK 在创建 eCAL 对象前尽早拒绝。
2. RegistryEndpoint 和 SensorRuntime 在 transport 边界拒绝不合法输入。
3. Core 的 AdapterRegistry 做最终领域校验。

不增加放宽规则的配置项或环境变量。

---

## 9. 拆分 SimulationCore

### 9.1 ModelState

职责：

- 独占真实 `mjModel/mjData`。
- 事务式加载 MJCF。
- 应用 `initial_keyframe`、清零 ctrl、执行 `mj_forward()`。
- 执行唯一的 `mj_step()`。
- 暴露 timestep、simulation time 和 model generation。

```cpp
class ModelState {
 public:
  bool Load(const std::string& path,
            const ModelLoadOptions& options,
            std::string* error);

  bool loaded() const noexcept;
  const mjModel* model() const noexcept;
  const mjData* data() const noexcept;
  mjData* mutable_data() noexcept;

  void ApplyActuatorWrites(
      std::span<const ValidatedActuatorWrite> writes);
  void Step();
  void ZeroAllControls();

  double simulation_time() const noexcept;
  double timestep() const noexcept;
  std::uint64_t generation() const noexcept;
};
```

### 9.2 ResourceCatalog

职责：

- 从 `mjModel` 生成 observation/command descriptor。
- 保存名称索引和稳定顺序。
- 校验注册资源并构造 `ResourceLayout`。
- 调用 Protocol canonical hash helper 填充 `layout_hash`。

资源 metadata 继续包含 MuJoCo type/datatype/stage、dimension、interval、delay、frame、unit、object、interpolation 和 ctrlrange。

### 9.3 AdapterRegistry

统一保存 Controller 和 Observer record，使用一个 ID 空间和一个 session allocator。职责包括：

- 完整注册验证后再原子提交。
- actuator ownership。
- PROVISIONAL activation deadline。
- ACTIVE、TIMED_OUT、ERROR、UNREGISTERED 状态转换。
- timed-out session replacement policy。
- heartbeat、recovery 和诊断字段。

Controller 与 Observer 可使用 variant 或按 role 分开的内部 record，但不得重新暴露两套 lifecycle API。

### 9.4 BarrierCoordinator

职责：

- 打开 Tick 时固定 required/recipient identity。
- 保存 pending validated Commit。
- 处理 commit、unregister、deadline 和 local error 对当前 Barrier 的影响。
- Barrier 完成时返回 actuator write plan。

required 集合打开后只能缩小，不能增加。pending command 只保存在 Barrier 中，完成前不得接触 `mjData.ctrl`。

### 9.5 TickStore

不得使用裸指针或只依赖 Core 容器生命周期的 `string_view` 作为解锁后的发布接口。

```cpp
struct IssuedTickIdentity {
  std::string adapter_id;
  std::uint64_t session_id;
  std::uint64_t epoch;
  std::uint64_t tick_id;
  std::uint64_t issue_generation;
};

struct IssuedTick {
  mujoco_sim::v2::Tick message;
  std::string serialized_bytes;
  IssuedTickIdentity identity;
  std::atomic<bool> current{true};
};

using IssuedTickHandle = std::shared_ptr<const IssuedTick>;
```

Core 只公开：

```cpp
std::vector<IssuedTickHandle> OpenIssuedTicks();
bool IsIssuedTickCurrent(const IssuedTickIdentity&) const;
```

删除 `OpenTick()` 和 `IsTickIssued(const Tick&)`。

失效规则：

- Barrier 完成。
- Adapter unregister、replacement 或进入 terminal ERROR。
- epoch/model generation 变化。
- Server stop。

失效时将 record 标记为非 current 并从 TickStore 索引移除；已被发布批次持有的 handle 保持内存有效。在途 Send 可以完成，但 SDK 必须按 session/epoch/tick/hash 丢弃过期消息。

### 9.6 ObservationScheduler

负责 every-N-step、period-s、sequence 和 delivery deadline。所有调度只使用 completed step 和 simulation time；period deadline 按原锁相规则推进。

### 9.7 SnapshotManager

负责 Create、Clone、Update、snapshot owner identity 和 model generation 校验。公开 `SimulationSnapshot` 仍可作为 Core 的受控返回类型，但实现和 MuJoCo deleter 移入该组件。

### 9.8 StatusBuilder

只读取 ModelState、AdapterRegistry、BarrierCoordinator 和 TickStore，构建 V2 `ServerStatus`，不得修改任何状态。

---

## 10. 统一 completed-step pipeline

所有可能完成 Barrier 的 Core 操作返回可选 `StepCompletedEvent`：

```cpp
enum class StepCompletionSource {
  kCommit,
  kDeadline,
  kUnregister,
  kLocalError,
  kEmptyBarrier,
};

struct StepCompletedEvent {
  std::uint64_t completed_tick_id;
  std::uint64_t next_tick_id;
  double simulation_time_s;
  StepCompletionSource source;
  std::vector<mujoco_sim::v2::ObservationFrame> observer_frames;
};
```

固定顺序：

1. Barrier 返回 validated actuator writes。
2. ModelState 批量写 ctrl。
3. ModelState 调用一次 `mj_step()`。
4. 更新 tick ID 和 simulation time。
5. ObservationScheduler 生成 frame。
6. 关闭 Barrier 并失效 TickStore record。
7. 返回 `StepCompletedEvent`。

Server 统一处理事件：

```text
lock core state
  -> 执行 Core 操作并取得 StepCompletedEvent
  -> SensorRuntime 复制本 step 所需 snapshot 并 enqueue
unlock core state
  -> 完成 blocking sensor fence
  -> 发布 ObserverFrame
  -> 更新 metrics/status
  -> 标记 step pipeline 完成
  -> 唤醒 Scheduler 打开下一 Tick
```

删除各 endpoint 中重复的 `CollectObservationFrames()`、`OnStepCompleted()`、blocking wait 和通知分支。

---

## 11. 拆分 Server

### 11.1 ServerApplication

负责：

- 最终配置校验。
- 日志和 eCAL runtime 生命周期。
- 构造 Core、Scheduler、Endpoint、SensorRuntime 和 ViewerRuntime。
- 事务式 Start 和固定 Stop/Shutdown 顺序。
- 进程退出码和 stop reason。

公开接口：

```cpp
class ServerApplication {
 public:
  explicit ServerApplication(ServerConfig config);
  bool Start(std::string* error);
  int Run();
  void RequestStop(std::string reason = "external_stop");
  const ServerConfig& config() const noexcept;
};
```

### 11.2 SimulationScheduler

负责 wall-clock timestep、empty Barrier、Tick 重传、deadline、status deadline、viewer sync deadline、`max_ticks` 和 condition wakeup。

Scheduler 持有显式 `step_pipeline_busy` 状态。只要某个 StepCompletedEvent 尚未完成 Sensor fence 和事件发布，就不得打开下一 Tick；该约束不通过长期持有 Core mutex 实现。

### 11.3 RegistryEndpoint

只注册以下 V2 method：

- `GetAvailableResources`
- `RegisterAdapter`
- `ActivateAdapter`
- `UnregisterAdapter`
- `GetSensorStreams`

同时拥有统一 `AdapterHeartbeat` subscriber。负责 payload 上限、parse/serialize、调用 Core、publisher cleanup 请求和 scheduler notification。

### 11.4 ControlEndpoint

负责：

- `AdapterCommit` subscriber。
- Tick publisher map。
- 初次 Tick 和重传发布。
- payload 上限、parse、日志和 transport metrics。

发布流程：

1. 从 Core 获取 `IssuedTickHandle` 批次。
2. 释放 Core state mutex。
3. 通过 identity 做 O(1) current 检查。
4. 直接发送 `serialized_bytes`。
5. publisher mutex 只保护 publisher map 和同 publisher Send，不与 Core mutex 同时持有。

### 11.5 ObservationEndpoint

负责 Observer publisher、Frame 发布和 Observer 注销后的 publisher 清理。Frame 已在 StepCompletedEvent 中拥有，不回调 Core 收集。

### 11.6 StatusEndpoint

负责 periodic/dirty-triggered `ServerStatus`，合并 SensorRuntime 的 StreamStatus 后在锁外序列化和发送。

### 11.7 ViewerRuntime

负责 viewer snapshot、主线程 RenderLoop、16 ms 同步节拍和 close request，不持有 endpoint，不访问真实主 `mjData`。

---

## 12. 锁模型

### 12.1 锁职责

`core_mutex`：

- 保护全部非线程安全 Core 调用。
- 保护从主状态复制 Snapshot 的短临界区。
- 不保护 eCAL、日志、blocking wait、worker join 或 Protobuf 序列化。

`publisher_mutex`：

- 保护 publisher map。
- 串行化同一 publisher 的创建、Send 和删除。

`application_mutex`：

- 保护 application state、stop reason、dirty flag、scheduler condition 和 `step_pipeline_busy`。

`CallbackState`：

- 拒绝 teardown 后的新 callback。
- 统计并排空 active callback。

### 12.2 禁止组合

- 持有 publisher mutex 时调用 Core。
- 持有 core mutex 时调用 eCAL `Send()`。
- 持有 core mutex 时等待 blocking Sensor。
- 持有 application mutex 时 join worker 或执行 transport teardown。
- Sensor worker 反向调用需获取 core mutex 的 Server 方法。

Endpoint callback 最多在短时间内持有 core mutex；所有外部 I/O 都使用拥有型结果在解锁后执行。

---

## 13. Server 配置清理

删除 `startup_timeout_ms` 的全部痕迹：

- generated schema field。
- `ServerConfig` member。
- `ToGenerated/ToPublic` 转换。
- `--startup-timeout-ms` CLI option。
- deprecated log event。
- Server unit/E2E assertion。
- Sample/WBR 配置和脚本参数。
- README 示例和表格。

严格 YAML 遇到 `startup_timeout_ms` 必须报告 unknown field；CLI 遇到 `--startup-timeout-ms` 必须报告 unknown option。

`activation_timeout_ms` 是 PROVISIONAL Adapter 的唯一激活期限。`tick_timeout_ms` 是 ACTIVE/TIMED_OUT keepalive 和 Barrier deadline 的默认值。

---

## 14. 可选 GUI、Sensor Stream 和 MuJoCo 构建

### 14.1 CMake options

```cmake
option(MUJOCO_SIM_BUILD_GUI
       "Build the MuJoCo simulate-based graphical viewer"
       ON)

option(MUJOCO_SIM_BUILD_SENSOR_STREAMS
       "Build asynchronous camera and LiDAR streams"
       ON)

option(MUJOCO_SIM_USE_BUNDLED_MUJOCO
       "Use the MuJoCo distribution bundled in this repository"
       ON)
```

`MUJOCO_SIM_USE_BUNDLED_MUJOCO` 在 `mujoco_sim_core` 中解析，因为 Core 当前创建和导出 `mujoco::mujoco`。bundled OFF 时使用 `find_package(mujoco CONFIG REQUIRED)`；bundled ON 时继续安装仓库内 library/header，并在安装态 config 中重建 imported target。

### 14.2 Target 拆分

```text
mujoco_sim_server_runtime
mujoco_sim_server_endpoints
mujoco_sim_server_sensor_runtime       # Sensor streams ON
mujoco_sim_server_viewer               # GUI ON
mujoco_sim_server
mujoco-sim-server
```

GUI OFF：

- 不查找 GLFW 或 PNG。
- 不编译 `simulate.cc`、GLFW adapter、viewer 和 PNG bridge。
- `--gui` 返回“该构建未包含 GUI”的明确错误。
- 默认运行模式自动为 headless。

Sensor streams OFF：

- Server 不链接 `mujoco_sim_sensor`、EGL 或 OSMesa。
- 非空 `sensor_streams` 配置在启动前失败。
- 相机和 LiDAR 都随 SensorRuntime 关闭。

必须支持并测试：

```text
GUI ON  + Sensor streams ON
GUI OFF + Sensor streams ON
GUI OFF + Sensor streams OFF
```

GUI ON + Sensor streams OFF 也应正常构建和运行，但不要求单独 CI job。

---

## 15. SDK 和仓库内应用迁移

### 15.1 SyncAdapterClient

- 使用 V2 Adapter lifecycle service。
- `AdapterOptions.controller_id` 改为 `adapter_id`，调用方同步修改。
- `registration()` 返回 V2 `RegisterAdapterResponse`。
- Tick 只校验 `layout_hash`，不比较完整 layout。
- Commit 使用 `AdapterCommit.adapter_id`。
- Status terminal-state 检查只遍历 `ServerStatus.adapters`。
- 已 Commit Tick 的重传继续自动发送缓存 Commit。

### 15.2 ObserverClient

- 切换到 V2 namespace 和 versioned endpoint。
- 删除 effective version 检查，固定验证 2.0。
- 保持 bounded FIFO/drop-oldest、drop count 和 wait wakeup 语义。

### 15.3 SensorStreamClient

- discovery 和 raw frame header 固定验证 V2.0。
- raw framing 自身的 framing version 继续独立存在。
- stream terminal error、borrowed view 和 `Copy()` 语义不变。

### 15.4 Sample Adapter

`ConstantCommandsForTick()` 不再读取 `tick.layout()`。应用启动后从 `client.registration().layout().commands()` 构建稳定 command name 列表，每 Tick 只填充值。

### 15.5 WBR LQR Demo

`TickDecoder` 在构造时验证 registration layout；Decode 时验证 Tick `layout_hash` 与构造时保存的 hash，不再调用 `ValidateTickLayout(tick.layout(), ...)`。控制算法、semantic slot 映射和六电机路径不变。

### 15.6 Status Tool 和 E2E probe

全部改为 V2 endpoint 和统一 AdapterStatus。删除 legacy/generic unregister 对照路径，每个生命周期场景只通过统一 API 验证一次。

---

## 16. 实施顺序

### Step 0：建立基线

记录：

- clean 9-package build/test。
- Sample Adapter 5 Tick。
- WBR headless 摘要。
- Sensor Demo 输出和质量指标。
- 1、2、4、8 Controller Barrier latency。
- Tick 平均/最大 bytes 和每 Tick 序列化次数。
- Server CPU 和 RSS。
- GUI/headless binary 动态依赖和镜像大小。

### Step 1：Protocol V2 原子切换

在同一个可构建变更中完成：

- V2 proto、versioned endpoint 和 topics。
- 统一 Adapter lifecycle 和 AdapterStatus。
- Core/Server/SDK/工具/Demo 的 namespace 与字段迁移。
- 删除 V1 proto、Controller lifecycle、minor 协商和 V1 protocol tests。
- 删除 `startup_timeout_ms`。

该 Step 不允许提交只生成 V2 但仍保留 V1 runtime 的中间状态。

### Step 2：Identifier 和 layout hash

- Protocol helper 和固定 hash test vectors。
- ResourceLayout/Tick hash 生成与验证。
- Config、SDK、Endpoint、Core 分层 identifier 校验。
- Sample/WBR 改用 registration layout。

### Step 3：ModelState、SnapshotManager 和 ResourceCatalog

- 先移动 metadata helper，再抽取 Snapshot。
- 抽取事务式 ModelState。
- 抽取 Catalog 和 layout 构造。
- `SimulationCore::LoadModel()` 改为组合事务。

### Step 4：AdapterRegistry 和 ObservationScheduler

- 迁移统一 Adapter record、session、ownership 和 deadline。
- 迁移 Observer delivery state。
- StatusBuilder 改读 Registry 只读接口。

### Step 5：BarrierCoordinator 和 TickStore

- 迁移 required/recipient/pending Commit。
- 引入 `IssuedTickHandle`、serialized cache 和 generation。
- Server 直接发送缓存 bytes。
- 删除旧 Tick API 和深序列化比较。

### Step 6：拆分 Server

顺序：

1. StatusEndpoint。
2. ObservationEndpoint。
3. RegistryEndpoint。
4. ControlEndpoint。
5. SimulationScheduler。
6. ServerApplication，并删除 EcalServer。

### Step 7：completed-step 和锁边界

- 所有 Core step 来源返回 StepCompletedEvent。
- endpoint 删除重复 post-step 代码。
- blocking Sensor 使用 `step_pipeline_busy` fence。
- Send、Observer publish 和日志移出 Core lock。
- 增加并发、Stop 和 TSAN 验证。

### Step 8：可选构建

- MuJoCo resolver 移入 Core CMake helper。
- 拆 viewer、sensor runtime 和 endpoint targets。
- 增加三类正式构建矩阵。
- 验证安装态 exported target 和动态依赖。

### Step 9：文档和最终验收

- 更新全部 README。
- 发布架构、V2 cutover、锁模型和 headless build 文档。
- 运行全仓普通测试、严格告警、sanitizer 和 E2E。

---

## 17. 测试矩阵

### 17.1 Protocol

- V2 常量和 versioned endpoint 名称。
- 非 2.0 request/header 拒绝。
- V1 service method 未注册、V1 topic helper 不存在。
- V2 schema descriptor 和 serialization conformance fixture。
- canonical encoding 固定 vectors。
- hash 长度、descriptor 字段、资源顺序和 section 顺序变化。
- malformed payload 和 4 MiB protocol payload 上限。
- raw stream prefix/header/payload 上限和 endian round-trip。

不保留 V1.1/V1.2 descriptor、enum number 或 golden bytes 测试。

### 17.2 Core

ModelState：

- default reset、keyframe reset、ctrl 清零。
- keyframe 不存在。
- XML/初始化失败不破坏当前模型。
- generation 只在成功加载后变化。

ResourceCatalog：

- unnamed resource 跳过。
- dimension、unit、frame、object、ctrlrange 和 Sensor metadata。
- 相同 layout hash 稳定。
- 任一字段或顺序变化导致 hash 变化。

AdapterRegistry：

- Controller/Observer ID 冲突。
- actuator 独占和注册失败回滚。
- role-specific 请求约束。
- activation timeout、runtime timeout、replacement on/off。
- session、epoch、heartbeat 和 recovery。

BarrierCoordinator：

- empty、单 Controller、多 Controller。
- Commit 顺序无关。
- duplicate same/mismatch。
- unregister、deadline、local error 完成 Barrier。
- 新 Activate 不进入当前 Barrier。
- recovery proof 不应用 command。

TickStore：

- 每 Adapter/Tick 只序列化一次。
- 初发和重传 bytes 完全相同。
- current identity 检查不序列化 Protobuf。
- invalidation 后索引立即拒绝。
- handle 在 Core 解锁和索引删除后仍可安全读取。
- model reload、replacement、unregister、Stop 后失效。

ObservationScheduler：

- every N、period_s、第一次 delivery 和锁相。
- 每种 StepCompletionSource 都不漏帧。
- sequence 单调。

### 17.3 Server

- Start 事务失败回滚 endpoint。
- 无 Controller 按 timestep 推进且不补跑。
- 有 Controller 等待 Barrier。
- `max_ticks`、Stop 和 eCAL not-ok。
- 所有 StepCompletionSource 进入统一 pipeline。
- Sensor fence 期间 Core mutex 可被无关 heartbeat/status 操作取得。
- eCAL Send 不持有 Core mutex。
- Tick 失效与在途发布并发安全。
- Publisher 无 subscriber、Send rejected 和 transport exception。
- callback drain、endpoint cleanup failure 和 worker teardown。
- Viewer sync failure 和 render exception boundary。

### 17.4 配置

- CLI > YAML > defaults。
- YAML 相对路径按文件目录解析。
- identifier 合法/非法边界。
- `startup_timeout_ms` 为 unknown YAML field。
- `--startup-timeout-ms` 为 unknown CLI option。
- GUI/Sensor Stream build capability 与运行配置一致。

### 17.5 SDK

- V2 registration/activation/unregister。
- registration layout/hash 校验。
- Tick hash mismatch 进入 terminal error。
- outstanding Tick 和重复 Wait 规则。
- Commit command name/dimension/range/finite 校验。
- 重传 Tick 自动重发相同 Commit。
- Stop、ERROR、session replacement 唤醒无限 Wait。
- Observer queue/drop 和 Sensor borrowed view 生命周期。

### 17.6 E2E

- Sample Adapter。
- 双 Controller Barrier。
- runtime register/activate/unregister。
- timeout/recovery 和 replacement。
- Observer every-step/period。
- RGB、depth、segmentation、camera XYZ。
- blocking LiDAR。
- WBR LQR headless/GUI。
- Sensor Demo headless/GUI。
- 无 DISPLAY Camera Stream。
- V2 Status Tool human/JSON/watch。
- eCAL cleanup 和 bounded process shutdown。

---

## 18. 性能验收

控制场景：

```text
8 Controllers
30 observations per Controller
1 kHz model timestep
5 ms Tick retransmission
10 seconds
```

要求：

- 每个 Adapter/Tick 的 Protobuf serialize count 为 1。
- 重传不构造 Tick、不复制完整 layout、不做完整 Tick 深比较。
- V2 Tick 平均 bytes 显著低于基线记录。
- Server CPU 不高于基线。
- Barrier latency P99 相对基线恶化不超过 5%。
- Tick handle invalidation/发布并发无 use-after-free。

并发要求：

- Commit callback 不持有 Core mutex 执行 Send。
- blocking Sensor 不持有 Core mutex 等待。
- Stop/Register/Unregister/Commit/Heartbeat 无死锁。
- 仓库自身纯并发 TSAN 测试无新增竞态。

Headless 要求：

- GUI OFF 不查找或链接 GLFW/PNG。
- GUI OFF + Sensor streams ON 可在 EGL/OSMesa 下运行。
- GUI OFF + Camera OFF 不链接 viewer、Sensor、EGL 或 OSMesa symbols。
- installed package consumer 在每个正式矩阵中通过。

---

## 19. 日志和指标

新增事件：

```text
event=tick_cache_created
event=tick_cache_invalidated
event=layout_hash_validated
event=layout_hash_mismatch
event=blocking_sensor_fence_started
event=blocking_sensor_fence_completed
event=scheduler_step_completed
event=protocol_version_rejected
```

新增指标：

```text
tick_serializations
tick_cache_hits
tick_cache_misses
tick_cached_bytes
layout_bytes_avoided
layout_hash_mismatches
blocking_wait_count
blocking_wait_total_ms
blocking_wait_max_ms
scheduler_wakeups
scheduler_late_wakeups
```

删除 deprecated config 事件和任何协议协商事件。日志不得输出完整 Tick、Observation scalar、Command scalar 或 Raw Sensor payload。

---

## 20. 文档更新

更新：

```text
Readme.md
core/mujoco_sim_core/Readme.md
core/mujoco_sim_protocol/Readme.md
core/mujoco_sim_server/Readme.md
core/mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md
core/mujoco_sim_sensor/README.md
core/mujoco_sim_adapter/mujoco_sim_sample_adapter/Readme.md
demos/mujoco_sim_wbr_lqr_demo/Readme.md
demos/mujoco_sim_sensor_demo/README.md
```

新增：

```text
doc/architecture/core_components.md
doc/architecture/server_runtime.md
doc/architecture/locking_model.md
doc/protocol/v2.md
doc/build/headless_only.md
doc/refactor/phase1_baseline.md
doc/refactor/phase1_v2_cutover.md
```

`phase1_v2_cutover.md` 只说明仓库内组件必须同步重建、旧 endpoint 已删除和配置字段变化；不提供双版本部署步骤或转换层。

---

## 21. PR 拆分

1. `test: capture phase-one behavior and performance baseline`
2. `feat(protocol)!: replace v1 with strict v2 protocol`
3. `refactor(adapter)!: use unified adapter lifecycle`
4. `refactor(status)!: remove duplicated controller status`
5. `refactor(config)!: remove deprecated startup timeout`
6. `feat(protocol): validate endpoint identifiers and layout hashes`
7. `refactor(core): extract model state and snapshots`
8. `refactor(core): extract resource catalog`
9. `refactor(core): extract adapter registry and observation scheduler`
10. `refactor(core): extract barrier coordinator and tick store`
11. `perf(core): publish immutable cached ticks`
12. `refactor(server): extract status and observation endpoints`
13. `refactor(server): extract registry and control endpoints`
14. `refactor(server)!: replace ecal server with server application`
15. `refactor(server): unify completed-step pipeline`
16. `refactor(server): move blocking sensor fence outside core lock`
17. `build(server): split gui and sensor runtime targets`
18. `docs: publish phase-one v2 architecture`

带 `!` 的提交是明确的 breaking change。Protocol V2、Core、Server、SDK、状态工具和仓库内应用的原子切换可合并为一个集成 PR；不能为了缩小 PR 而在主分支保留 V1/V2 双轨实现。

每个合入点必须可独立构建、测试通过、不混入无关格式化，并说明行为不变量验证结果。

---

## 22. 风险与控制

### 22.1 V2 原子切换

风险：跨包类型、endpoint 和字段同时变化，漏改安装态 consumer 或脚本会在运行期静默等待。

控制：

- versioned endpoint 让旧进程无法误连。
- `rg` 检查 `mujoco_sim::v1`、`mujoco_sim/v1` 和 Controller lifecycle symbol 为零。
- 9 包 clean build，不使用旧 build/install 产物。
- 所有真实 eCAL E2E 从新 install tree 运行。

### 22.2 状态机回归

重点检查：

- unregister、deadline、local error 触发 step 的条件。
- recovery pending 是否提前 ACTIVE。
- completed-step 是否漏 Observer/Sensor。
- empty Barrier 是否仍按 timestep 推进。

控制：先建立 characterization tests，再逐组件迁移，每个 step 来源独立断言 event 和 `mj_step()` 次数。

### 22.3 Tick handle 生命周期

风险：Core 解锁后 Tick 被 unregister/commit/replace 失效，发布线程仍在访问缓存。

控制：发布批次使用 `shared_ptr<const IssuedTick>`；索引失效和对象生命周期分离；并发测试覆盖失效前后 Send。

### 22.4 Layout hash

风险：canonical 编码不稳定、跨架构 endian 不一致或漏字段。

控制：Protocol 包唯一实现、显式 endian、全部字段测试、固定 vectors、x86_64/其他可用架构交叉验证。hash 只用于一致性检测，不用于安全认证。

### 22.5 completed-step 并发

风险：移除长时间 `simulation_mutex_` 后下一 Tick 可能在 Sensor fence 完成前打开。

控制：Scheduler 使用显式 `step_pipeline_busy` 状态；只有统一事件处理器清除该状态；所有 open-Tick 路径断言 pipeline idle。

### 22.6 可选构建

风险：exported target 仍泄漏 GLFW/PNG/Sensor 依赖，或 GUI OFF 配置仍接受 `--gui`。

控制：每个正式矩阵运行 configure、build、installed consumer、动态依赖检查和最小 runtime smoke test。

---

## 23. 完成定义

全部满足才算第一阶段完成：

- 仓库不再生成、安装或引用 `mujoco_sim::v1`。
- Registry 不再注册 Controller 专用 lifecycle method。
- Core 不再包含 Controller lifecycle wrapper 或 minor negotiation。
- `ServerStatus` 只输出统一 Adapter 状态。
- `startup_timeout_ms` 和 CLI option 已完全删除。
- `EcalServer` 类型和 header 已删除，公开入口为 `ServerApplication`。
- `SimulationCore` 不直接保存全部 Controller/Observer/Barrier/Tick 容器。
- `OpenTick()` 和 `IsTickIssued(const Tick&)` 已删除。
- Tick 重传只发送缓存 bytes，每 Adapter/Tick 只序列化一次。
- Tick 不携带完整 ResourceLayout，SDK 不补回 layout。
- layout hash 和 endpoint identifier 在所有边界统一验证。
- 所有 step 路径只通过 StepCompletedEvent 进入 post-step pipeline。
- blocking Sensor 不持有 Core mutex 等待。
- eCAL Send 不持有 Core mutex。
- GUI OFF 不依赖 GLFW/PNG；Sensor streams 可独立开关。
- Sample、WBR、Observer、Sensor Demo、Status Tool 全部通过 V2 E2E。
- 9 包 clean build/test、严格告警和对应 sanitizer 门禁通过。
- 性能和并发验收达标。
- README、架构、V2、锁模型和构建文档完整。

---

## 24. 最终推荐接口

```cpp
class SimulationCore {
 public:
  bool LoadModel(const std::string& path,
                 const ModelLoadOptions& options,
                 std::string* error);

  RegisterAdapterResult RegisterAdapter(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  ActivateAdapterResult ActivateAdapter(
      const mujoco_sim::v2::ActivateAdapterRequest& request);
  UnregisterAdapterResult UnregisterAdapter(
      const mujoco_sim::v2::UnregisterAdapterRequest& request);

  std::vector<IssuedTickHandle> OpenIssuedTicks();
  CommitResult SubmitCommit(const mujoco_sim::v2::AdapterCommit& commit);
  DeadlineResult CheckDeadlines(std::chrono::steady_clock::time_point now);
  EmptyBarrierResult CompleteEmptyBarrier();

  void AcceptHeartbeat(const mujoco_sim::v2::AdapterHeartbeat& heartbeat);

  std::unique_ptr<SimulationSnapshot> CreateSnapshot(
      std::string* error) const;
  std::unique_ptr<SimulationSnapshot> CloneSnapshot(
      const SimulationSnapshot& source,
      std::string* error) const;
  bool UpdateSnapshot(SimulationSnapshot* snapshot,
                      std::string* error) const;

  mujoco_sim::v2::ServerStatus MakeStatus() const;
};
```

所有可能 step 的 result 类型都包含：

```cpp
std::optional<StepCompletedEvent> step_completed;
```

```cpp
class SimulationScheduler {
 public:
  bool Start(std::string* error);
  int Run();
  void RequestStop(std::string reason);

  void NotifyLifecycleChanged();
  void NotifyCommitReceived();
  void CompleteStepPipeline(const StepCompletedEvent& event);
};
```

```cpp
class ControlEndpoint {
 public:
  bool Start(std::string* error);
  void Stop();
  void PublishTicks(std::span<const IssuedTickHandle> ticks,
                    PublishReason reason);
};
```

---

## 25. 最终原则

第一阶段建立以下稳定边界：

```text
MuJoCo State
    与
Resource/Adapter State
    与
Barrier/Tick State
    与
Transport Endpoint
    与
Wall-clock Scheduler
    与
Sensor Backpressure
```

V2 之后的新能力必须沿这些边界扩展：新 transport 不修改 Core，新 Sensor 不修改 Barrier，GUI 不影响 headless，协议 payload 优化不进入控制算法。第一阶段不追求更多功能，优先完成单一协议、单一 lifecycle、单一 step pipeline 和可验证的并发所有权。
