# MuJoCo Sim Core V1.1 Handoff

## 1. 文档目的

本文面向下一位维护者，记录截至 2026-07-13 的 Phase 1 / Protocol V2.0 重构状态、关键架构、
运行时不变量、验证结果、当前工作树和后续注意事项。实施要求以
[`mujoco_sim_core_phase1_refactor_complete_plan.md`](./mujoco_sim_core_phase1_refactor_complete_plan.md)
为准；本文侧重于接手后如何理解、验证和继续修改当前代码。

V1.1 仍基于以下 V1 提交开始重构：

```text
5a7e8cd3f78406883ea7e72c3c1ad5b27fd467b8
feat: add sensor observers and streams
```

当前 V1.1 基线是包含本 handoff 的 Phase 1 交付提交。上述提交只是重构起点，不是 V2 实现
提交。需要比较 V1/V2 行为或性能时，以它作为 V1 reference。

## 2. 当前完成状态

Phase 1 已完成一次性破坏性 V2 切换：

- Wire、C++ namespace、service 和 topic 只保留 Protocol V2.0。
- 删除 V1 proto、Controller 专用 lifecycle、minor 协商和旧配置入口，不提供兼容层。
- Controller 与 Observer 统一使用 Adapter registration、activation、heartbeat、status 和
  unregister lifecycle。
- Tick 只携带 16-byte `layout_hash`；完整 `ResourceLayout` 只在注册响应中发送。
- 每个 Adapter/Tick 生成一个 immutable `IssuedTickHandle`，Protobuf 只序列化一次，重传复用
  cached bytes。
- Core 的所有 step 来源统一返回 `StepCompletedEvent`。
- Server 已拆为 `ServerApplication`、endpoint groups、`SimulationScheduler`、SensorRuntime 和
  ViewerRuntime；公开 `EcalServer` 已删除。
- eCAL Send、日志和 blocking Sensor fence 不持有 Core state lock。
- GUI 与 Sensor Stream 可独立关闭，支持纯 headless 安装和 consumer。
- Sample Adapter、WBR LQR Demo、Sensor Demo、Status Tool 和 SDK 均已迁移到 V2。

相关说明：

- [`../../architecture/core_components.md`](../../architecture/core_components.md)
- [`../../architecture/server_runtime.md`](../../architecture/server_runtime.md)
- [`../../architecture/locking_model.md`](../../architecture/locking_model.md)
- [`../../protocol/v2.md`](../../protocol/v2.md)
- [`../../build/headless_only.md`](../../build/headless_only.md)
- [`../../refactor/phase1_v2_cutover.md`](../../refactor/phase1_v2_cutover.md)
- [`../../refactor/phase1_baseline.md`](../../refactor/phase1_baseline.md)

## 3. 包和依赖边界

仓库仍包含 9 个 CMake/colcon 包：

| 包 | 职责 |
| --- | --- |
| `mujoco_sim_config` | schema 驱动的 DTO、CLI11 和严格 YAML 配置 |
| `mujoco_sim_protocol` | V2 protobuf、endpoint 命名、identifier、layout hash、raw stream framing |
| `mujoco_sim_core` | MuJoCo 主状态、Adapter 状态机、Barrier、Tick cache、Snapshot、Observer 调度 |
| `mujoco_sim_sensor` | transport-free camera/LiDAR capture |
| `mujoco_sim_server` | composition root、eCAL endpoint、调度、Sensor、GUI、日志和状态工具 |
| `mujoco_sim_adapter_sdk` | Controller、Observer、Sensor Stream 客户端和共享 eCAL runtime |
| `mujoco_sim_sample_adapter` | 最小 V2 Controller 示例 |
| `mujoco_sim_wbr_lqr_demo` | WBR LQR 同步控制 Demo |
| `mujoco_sim_sensor_demo` | 相机、IMU、LiDAR 和建图 Demo |

依赖方向必须保持：

```text
protocol -> core -> server
    |         |       |
    |         +-> sensor
    |
    +-> adapter_sdk -> sample/demos

config -> server/sample/demos/tools
```

Core 和 Sensor 不得依赖 eCAL。Transport callback 不得直接访问 MuJoCo 主状态。

## 4. Protocol V2.0

V2 是唯一协议，不存在协商或降级：

```text
protobuf package: mujoco_sim.v2
protocol major:   2
protocol minor:   0
```

Endpoint 包含 major version，例如：

```text
mujoco_sim.v2.<server>.registry
mujoco_sim/v2/<server>/tick/<adapter>
mujoco_sim/v2/<server>/commit
mujoco_sim/v2/<server>/adapter_heartbeat
```

`server_name`、`adapter_id` 和 `stream_name` 必须匹配：

```regex
^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$
```

Registration 返回完整 layout 和 canonical hash。Tick 只返回该 hash。SDK 遇到 hash malformed 或
mismatch 时进入当前 session 的 terminal protocol error，不重新发现 layout，也不继续 Commit。

禁止重新增加 V1 schema、namespace、topic helper、service alias、旧配置字段或转换 wrapper。

## 5. Core 状态和 step 边界

`SimulationCore` 是 transport-free public facade。内部 `detail` 组件包括：

- `ModelState`：事务模型加载、真实 `mjModel/mjData`、actuator write 和唯一 `mj_step()`。
- `ResourceCatalog`：稳定资源顺序、descriptor、registration layout 和 layout hash。
- `AdapterRegistry`：Controller/Observer record、session 和 actuator ownership。
- `BarrierCoordinator`：required/recipient/pending Commit、deferred zero 和 actuator write plan。
- `TickStore`：immutable Tick handle、serialized cache、identity index 和 invalidation。
- `SnapshotManager`、`ObservationScheduler`、`StatusBuilder`。

必须保持以下顺序：

```text
Barrier ready
  -> BarrierCoordinator::BuildWritePlan()
  -> ModelState::ApplyActuatorWrites()
  -> ModelState::Step()
  -> increment tick
  -> build Observer frames
  -> invalidate Tick handles
  -> close Barrier
  -> return StepCompletedEvent
```

Timeout、unregister、replacement 和 local command error 在 Tick 打开期间不能直接修改
`mjData.ctrl`。它们把 actuator zero 加入当前 Barrier write plan；只有 Barrier 完成后才应用。
Tick 未打开时可以通过 ModelState 的归零接口立即清理已释放 Controller 的控制量。

目前内部组件仍集中实现于 `simulation_core.cpp`，其中 AdapterRegistry 的部分领域操作仍由
`SimulationCore` 编排并直接操作内部 record。继续拆分时应优先增加窄领域 API 和只读视图，
不要再增加 public map 访问，也不要改变上述 write/step 顺序。

## 6. Tick cache 和发布生命周期

`OpenIssuedTicks()` 返回 `shared_ptr<const IssuedTick>`：

- 一个 Adapter/Tick 只构建并序列化一次。
- 初发和重传必须复用同一 handle 和同一 `serialized_bytes`。
- Core invalidation 会从 identity index 删除记录并把 `current` 置为 false。
- 已经在途的 shared handle 在解锁发布期间仍保持有效。
- completion、model reload、replacement、unregister 和 stop 都必须 invalidation。

Server 发布前必须检查 handle identity/current，但不得通过重新序列化完整 Tick 做等价性比较。

## 7. Server 调度和 completed-step pipeline

`ServerApplication` 是组合根。Core 操作在 state lock 内完成；拿到 `StepCompletedEvent` 后，应用
释放 Core lock，再执行 Snapshot enqueue、blocking Sensor wait、Observer publish、日志和指标。

`SimulationScheduler::WaitUntil()` 有两种语义：

- 调用时 pipeline busy：等待 pipeline completion、generation change 或 deadline。
- 调用时 pipeline idle：只等待 generation change 或 deadline，不能因为 idle predicate 立即返回。

2026-07-13 的代码审查曾发现 idle wait predicate 恒真导致 RunLoop busy-spin；该问题已修复，
并增加 deadline wait 和 notification wakeup 测试。修改 scheduler predicate 时必须同时覆盖 busy
和 idle 两条路径，并在 TSAN 下验证 lost wakeup。

所有真正执行 `mj_step()` 的来源都必须走统一 pipeline：

- normal Commit Barrier；
- deadline 缩小 Barrier；
- unregister；
- local Controller error；
- empty Barrier。

## 8. SDK 生命周期和 teardown

### 8.1 SyncAdapterClient

- `Start()` 收到 `OK` registration 后立即记录 provisional session。
- 后续 identity/layout/hash 校验失败时执行 best-effort Unregister，避免服务端资源泄漏。
- 同时最多一个 outstanding Tick。
- `Commit()` 在本地规范化并缓存 payload，再执行 publish。
- publish 返回 false 时 `Commit()` 必须返回 false 和诊断；cached Commit 保留，调用方可用同一
  Tick 显式重试。
- Server 重传同一 Tick 时 SDK 自动重发完全相同的 cached Commit。

传输发送成功不等于 Server 已接受 Commit；V2 当前没有 Commit ack service。

### 8.2 ObserverClient 和 SensorStreamClient

Callback teardown 使用 shared `CallbackState` lease：先禁止新 callback，再移除 transport
callback，最后等待其他线程上的 active callback 排空。

Sensor Stream callback 允许在当前 callback 中停止或销毁最后一个 client owner。等待条件会
扣除当前线程持有的 lease，避免析构等待自身；lease 随后只访问 shared CallbackState。修改该
逻辑时必须保留以下测试：

- teardown 等待其他线程上的 callback；
- delayed callback 不能访问已销毁 client；
- callback 当前线程可以 Stop；
- callback 当前线程可以销毁 client。

## 9. 可选构建

正式能力开关：

```text
MUJOCO_SIM_BUILD_GUI
MUJOCO_SIM_BUILD_SENSOR_STREAMS
MUJOCO_SIM_USE_BUNDLED_MUJOCO
```

支持矩阵：

| GUI | Sensor Stream | 预期 |
| --- | --- | --- |
| ON | ON | 完整 Server、GUI 和 Sensor |
| OFF | ON | headless Server，保留 EGL/OSMesa camera/LiDAR |
| OFF | OFF | 不查找/链接 GLFW、PNG、Sensor、EGL 或 OSMesa |

使用 `core/mujoco_sim_server/test/build_matrix.sh` 验证 build、install、consumer、headless smoke
和依赖排除。运行配置请求构建时不存在的能力必须 fail-fast，不得静默忽略。

## 10. 验证基线

Phase 1 完整验收记录位于 [`../../refactor/phase1_baseline.md`](../../refactor/phase1_baseline.md)：

```text
clean build: 9 packages
full suite:  51 CTest tests, 0 failures
ASAN/UBSAN: 46 Core + 12 Server tests passed
TSAN:       46 Core + 60 Adapter SDK tests and concurrency tests passed
build matrix: GUI ON/Sensor ON, GUI OFF/Sensor ON, GUI OFF/Sensor OFF passed
```

性能中位数相对 V1 baseline：

| Metric | Change |
| --- | ---: |
| Tick serialization count | -66.7% |
| Average/maximum Tick bytes | -63.1% |
| CPU seconds | -73.4% |
| Barrier P99 | -69.1% |
| Maximum RSS | -4.6% |

2026-07-13 代码审查修复后已重新验证：

```text
Core core_test                         passed
Core warnings build + core_test        passed
Server server_test                     passed
Server simulation_scheduler_test       passed
Adapter SDK adapter_sdk_test           passed
git diff --check                       passed
```

本轮新增测试位于已有 test executables 内，因此不会增加顶层 CTest test 数量。尚未在这些最后修复
之后重新跑完整 9-package clean build、全部 E2E、ASAN/UBSAN 和 TSAN；发布或合并前应重跑。

Adapter SDK 全 target warnings build 曾被
`test/sensor_observer_e2e_probe.cpp` 的未使用 `MethodInfo` helper 阻断；交付提交已删除该 helper
并重新通过对应 warnings build。后续增加 E2E helper 时仍需在全 target `-Werror` 配置下验证，
不能只构建 SDK library 和 unit test。

## 11. 交付提交和本地工作树

包含本 handoff 的提交是 V1.1 Phase 1 交付基线，覆盖 V2 protocol、Core、Server、SDK、Demo、
文档、V1 文件删除、性能 fixture 和审查修复。接手者开始工作前仍应运行：

```bash
git status --short
git diff --check
```

交付提交同时包含此前由用户确认保留的以下改动：

```text
demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml
  headless: false

demos/mujoco_sim_wbr_lqr_demo/test/app_config_test.cpp
```

接手后的新改动不能假定可丢弃。提交前应按 package 分组审查 staged/unstaged/untracked 文件，
并确保没有把本地 build、install、log 或 Demo 输出加入版本控制。

## 12. 建议接手顺序

1. 阅读本 handoff、Phase 1 plan、V2 protocol 和 locking model。
2. 检查工作树并确认用户原有改动仍被保留。
3. 从空 build/install/log 目录运行 9-package clean build 和完整 CTest/E2E。
4. 重跑三个正式 optional-build matrix。
5. 重跑 ASAN/UBSAN；使用 `setarch x86_64 -R` 运行 GCC TSAN。
6. 复核安装树不存在 V1 header/proto/symbol，纯 headless binary 不携带 GUI/Sensor 依赖。
7. 复核运行指标和性能 fixture，再开始下一阶段功能修改。

任何后续改动都应优先保护三条边界：严格 V2、Barrier 完成前不写 ctrl、Core lock 外执行
transport/blocking work。
