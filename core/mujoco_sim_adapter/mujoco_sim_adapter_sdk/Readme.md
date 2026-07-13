# mujoco_sim_adapter_sdk

`mujoco_sim_adapter_sdk` 是 MuJoCo 仿真通信的 C++ SDK，提供三个可共存的客户端：

- `SyncAdapterClient`：同步 Controller，使用 `WaitForTick()`/`Commit()` 参与 Barrier。
- `ObserverClient`：只读 Observer，按步数或仿真时间采样 observation，不提供 Commit API。
- `SensorStreamClient`：发现并消费 RGB、深度、分割和 XYZ 等原始传感器流，不创建 Adapter session。

三个客户端共享进程级 eCAL runtime lease；同一进程中可以同时创建多个不同类型的客户端。
新增 API 不改变同步控制契约：`SyncAdapterClient` 的 `Tick` 仍按 registration layout 携带
关节状态、IMU 等小 observation，控制器仍在同一 Tick 上提交 Commit。

**本包只安装 C++ 库和头文件，不安装独立生产可执行程序。** 可运行参考程序见
[mujoco_sim_sample_adapter](../mujoco_sim_sample_adapter/Readme.md)。

## 职责与边界

`SyncAdapterClient` 负责：

- 使用 `server_name` 定位 registry service，注册、激活和注销 Controller。
- 声明 observation sensor 与 command actuator，保存 Server 返回的 session、epoch 和布局。
- 创建 Controller 专属 Tick 订阅、ServerStatus 订阅和共享 Heartbeat/Commit publisher。
- 在 eCAL callback 中校验、去重并缓存 Tick；用户控制代码只在调用方业务线程运行。
- 通过独立共享 callback state 隔离延迟 eCAL 回调与 Client 生命周期，并在 teardown 等待
  已取得 lease 的 callback；所有 publisher 写入在 SDK 内串行化。
- 自动发送 Heartbeat；Commit 发布后收到相同 Tick 时原样重发缓存 Commit。
- 在 `TIMED_OUT` 期间只缓存最新后续 Tick，避免恢复慢于仿真时产生无界队列。
- 根据 registration layout 在本地校验和规范化 command，再构造协议 Commit。
- 监听匹配 session 的 `ERROR` 和 session replacement，唤醒等待线程并保留诊断。
- 仅在 SDK 自己初始化 eCAL 时，于析构阶段完成对应的 `eCAL::Finalize()`。

SDK 不加载 MuJoCo 模型、不持有 `mjModel/mjData`、不执行 `mj_step()`、不提供 GUI、配置
解析、控制算法、资源自动选择或自动重连。物理状态、资源所有权、Commit Barrier 和最终
协议校验仍由 Server/Core 管理。

Commit 继续使用 eCAL pub/sub，没有同步业务回执。`Commit()` 成功只表示命令通过本地校验、
已缓存并尝试发布，不证明 Server 已接受。Server 未收到 Commit 时会重发 Tick；SDK 利用该
重传自动重发缓存 Commit。

## 依赖与构建

生产库要求 CMake 3.22、C++20、eCAL 和 `mujoco_sim_protocol`。启用测试时还需要 GTest 和
`mujoco_sim_server`，用于运行真实 timeout/recovery E2E；生产 target 不依赖 Server 或
MuJoCo。

从仓库根目录执行：

```bash
colcon build \
  --packages-up-to mujoco_sim_adapter_sdk \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

安装后下游通过 CMake config package 使用：

```cmake
find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
target_link_libraries(my_adapter PRIVATE mujoco_sim::adapter_sdk)
```

公开头文件为：

- [`<mujoco_sim/adapter_sdk/sync_adapter_client.hpp>`](include/mujoco_sim/adapter_sdk/sync_adapter_client.hpp)
- [`<mujoco_sim/adapter_sdk/observer_client.hpp>`](include/mujoco_sim/adapter_sdk/observer_client.hpp)
- [`<mujoco_sim/adapter_sdk/sensor_stream_client.hpp>`](include/mujoco_sim/adapter_sdk/sensor_stream_client.hpp)

公开 target `mujoco_sim::adapter_sdk` 传递 `mujoco_sim::protocol` 和 `eCAL::core`。新增客户端没有
额外的下游依赖，现有 CMake package config 和 target 名称保持不变。

## SyncAdapterClient 公开 API

| API | 行为 |
|---|---|
| `AdapterOptions` | 描述 Server/Controller、资源声明和 service/Commit timeout。 |
| `CommandValue` | 一个具名标量 actuator command。 |
| `TickWaitResult` | `kReceived`、`kTimeout`、`kStopped` 或 `kError`。 |
| `Start()` | 初始化通信、注册 Controller 并创建 pub/sub；状态仍为 `PROVISIONAL`。 |
| `Activate()` | 等待短暂 discovery 窗口后显式激活 Controller。 |
| `WaitForTick()` | 在业务线程阻塞获取一个 Tick；不执行或提交控制算法。 |
| `Commit()` | 校验并提交当前 Tick 的完整 command 集合。 |
| `Stop()` | 本地取消，停止接受新 Tick 并唤醒等待线程；不隐式注销。 |
| `Unregister()` | 幂等注销当前 session；`NOT_FOUND`/`INVALID_SESSION` 视为已完成。 |
| `registration()` | 返回最近一次成功注册的 response，用于读取布局和会话信息。 |
| `lifecycle_warning()` | 返回 Server `ERROR`、session replacement 等线程安全诊断副本。 |

`AdapterOptions`：

| 字段 | 默认值 | 约束与含义 |
|---|---:|---|
| `server_name` | `"default"` | 必须与 Server namespace 一致。 |
| `adapter_id` | 空 | 必填。 |
| `observation_names` | 空列表 | 允许不申请 observation。 |
| `command_names` | 空列表 | 至少一个。 |
| `timeout_ms` | `1000` | 每 Tick Commit deadline；`0` 使用 Server 默认值。 |
| `service_timeout_ms` | `5000` | 每次 registry service 调用总等待上限。 |

核心接口：

```cpp
enum class TickWaitResult {
  kReceived,
  kTimeout,
  kStopped,
  kError,
};

bool Start(std::string* error_message);
bool Activate(std::string* error_message);
TickWaitResult WaitForTick(std::chrono::milliseconds timeout,
                           mujoco_sim::v2::Tick* tick,
                           std::string* error_message);
bool Commit(const mujoco_sim::v2::Tick& tick,
            const std::vector<CommandValue>& commands,
            std::string* error_message);
```

`WaitForTick()` 的 `timeout == 0ms` 是非阻塞检查，
`std::chrono::milliseconds::max()` 表示无限等待。只有 `kReceived` 会修改输出 Tick；超时不
修改它。空输出指针、负 timeout、未 Activate、并发 Wait 或仍有未提交 Tick 均返回
`kError` 和具体消息。

## ObserverClient

`ObserverClient` 使用 V2.0 `RegisterAdapter(role=OBSERVER)`、`ActivateAdapter` 和
`UnregisterAdapter`。它订阅 Observer 专属 observation topic 和 ServerStatus，并由独立线程向
共享 adapter heartbeat topic 周期发布 `AdapterHeartbeat`。Observer 不参与 Commit Barrier，公开
类中也没有 `Commit()` 成员。

`ObserverOptions`：

| 字段 | 默认值 | 约束与含义 |
|---|---:|---|
| `server_name` | `"default"` | Server namespace。 |
| `observer_id` | 空 | 必填，进程不自动生成 ID。 |
| `observation_names` | 空列表 | 至少一个，不能重复。 |
| `timeout_ms` | `1000` | Server 侧 keepalive timeout；`0` 使用 Server 默认值。 |
| `service_timeout_ms` | `5000` | 每次 registry service 调用总等待上限。 |
| `keepalive_period_ms` | `250` | 独立 heartbeat 线程的发布周期，必须大于 `0`；`timeout_ms` 非零时必须满足 `keepalive_period_ms < timeout_ms`。 |
| `every_n_steps` | `1` | 与 `period_s` 二选一；按仿真步数降采样。 |
| `period_s` | 未设置 | 与 `every_n_steps` 二选一；按仿真时间降采样。 |
| `queue_capacity` | `1` | observation FIFO 上限，满时丢弃最旧帧。 |

选择 `period_s` 时需要同时执行 `options.every_n_steps.reset()`。每个合法新 sequence 只入队一次；
identity、协议版本、registration layout、block 维度不匹配以及旧/重复 sequence 均被忽略。
`dropped_frames()` 返回 sequence gap 与本地队列满载造成的累计丢帧数。

```cpp
#include <mujoco_sim/adapter_sdk/observer_client.hpp>

mujoco_sim::adapter_sdk::ObserverOptions options;
options.observer_id = "telemetry";
options.observation_names = {"hinge_pos", "hinge_vel"};
options.queue_capacity = 4;

mujoco_sim::adapter_sdk::ObserverClient observer(options);
std::string error;
if (!observer.Start(&error) || !observer.Activate(&error)) {
  // Handle error.
}
mujoco_sim::v2::ObservationFrame frame;
const auto result = observer.WaitForObservation(
    std::chrono::milliseconds(100), &frame, &error);
```

`WaitForObservation()` 的 timeout 规则与 `WaitForTick()` 相同。一个 Client 同时只允许一个
Wait；超时和错误不会修改输出。`Stop()` 可从其他线程唤醒 Wait 并停止 keepalive，之后仍可调用
`Unregister()`。匹配 session 的 `ERROR`/`TIMED_OUT` 或同 ID replacement 会使 Wait 返回
`kError`，原因可从 `lifecycle_warning()` 获取。

## SensorStreamClient

`SensorStreamClient::Start()` 调用 V2.0 `GetSensorStreams`，确认所有请求的静态 descriptor 后，
为每个 stream 创建 `SensorStreamTopicName` subscriber。它不注册 Adapter、没有 Activate、
Heartbeat、Commit 或 Unregister 阶段。

`SensorStreamOptions` 包含 `server_name`、必填且不重复的 `stream_names`、
`service_timeout_ms`（默认 `5000`）和每个 stream 的 `queue_capacity`（默认 `1`）。Start 会校验
topic、format、width/height、row stride、frame convention、rate 和 payload 上限。每帧再校验
server、epoch、协议版本、stream 名、format、shape 和 payload，旧/重复 sequence 被忽略。

每个 stream 必须独立选择一种模式：

- `SetCallback(name, callback, &error)`：callback 收到只在本次调用期间有效的
  `StreamSampleView`。`header()` 和 `payload()` 都是只读的；需要跨 callback 保存时显式调用
  `Copy()` 得到 `OwnedStreamSample`。同一 stream 的 callback 串行执行，异常会被捕获并写入
  `stream_diagnostic()`。
- `WaitForSample(name, timeout, &sample, &error)`：第一次调用即选择阻塞模式。队列满时丢弃
  最旧 sample，累计值由 `dropped_samples(name)` 返回。不同 stream 可以由不同线程同时等待，
  同一 stream 不允许并发 Wait。

`SensorStreamClient` 还订阅匹配 server/epoch/protocol 的 `ServerStatus`。请求的某个 stream
进入 `STREAM_ERROR` 后，该错误对该 stream 粘性生效：对应 `WaitForSample()` 被唤醒并返回
`kError`，callback 模式可从 `stream_diagnostic()` 读取同一服务端诊断；其他 stream 不受影响。
同 tick 的后续 status 仍会处理，以免遗漏异步 worker 在 step 之间产生的错误，旧 tick status
会被忽略。

`dropped_samples()` 合并 Server frame header 报告的 capture queue drop、transport sequence gap
和本地 client queue drop；没有 terminal error 时，`stream_diagnostic()` 保存最近一次
malformed/mismatch/callback 异常。

模式一旦选定不可切换；callback 模式和 blocking 模式互相拒绝。模式尚未选择时收到的帧先进入
有界队列，选择 callback 时这些旧帧会被清除并计入 drop。`Stop()` 禁止新的 sample delivery、
唤醒全部 Wait，但不等待已经进入的 callback；析构会移除 eCAL callback 并等待这些 callback
返回。因此 callback 捕获的外部对象必须至少存活到 Client 销毁完成，不能在 callback 内销毁
Client 本身。

```cpp
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>

mujoco_sim::adapter_sdk::SensorStreamOptions options;
options.stream_names = {"front_rgb"};
mujoco_sim::adapter_sdk::SensorStreamClient streams(options);
std::string error;
if (streams.Start(&error)) {
  streams.SetCallback(
      "front_rgb",
      [](const mujoco_sim::adapter_sdk::StreamSampleView& view) {
        auto owned = view.Copy();
        // Queue or process owned outside this callback.
      },
      &error);
}
```

## 生命周期

1. 构造 `AdapterOptions` 和不可复制的 `SyncAdapterClient`。
2. `Start(&error)` 注册资源并创建 endpoint；此时通过 `registration().layout()` 初始化
   decoder、控制器和 command mapper，避免准备完成前进入 Commit deadline。
3. `Activate(&error)` 激活当前 session。新 Controller 从 Server 的下一 Tick 加入 Barrier。
4. 业务线程循环调用 `WaitForTick()`、计算 command、调用 `Commit()`。
5. 正常结束或业务错误时显式调用 `Unregister()`；其他线程可用 `Stop()` 取消 Wait。
6. 析构函数会停止并排空 transport callback，再尽力注销，作为遗漏显式清理时的后备。

对象是单次生命周期：Stop/Unregister 后不能重新 Start 或 Activate。`Stop()` 不等待 service
注销，因此实时控制线程可立即退出；`Unregister()` 允许在 Stop 后调用并保持失败可重试。
`Start()`、`Activate()`、`Commit()` 和 `Unregister()` 在内部串行化；`Stop()` 可从其他线程
设置取消标志并唤醒阻塞操作。并发 Activate 不会发出两个请求，Stop 与成功 Activate response
竞争时由 Stop 获胜，预收 Tick 会被清除。

匹配 session 被 Server 标为 `ERROR` 或被同名新 session 替换时，SDK 设置 terminal error、
停止接收并唤醒 Wait。此时 `WaitForTick()` 返回 `kError`，同一原因也可从
`lifecycle_warning()` 获取。

Server 全局进入 `STOPPED` 时，Controller 的 `WaitForTick()` 和 Observer 的
`WaitForObservation()` 都返回 `kStopped`；全局进入 `ERROR_PAUSED` 时，两者都返回
`kError` 并保留 Server 诊断。成功 `Unregister()` 与 `STOPPED` 使用同一个原子终止转换：停止
标志、注册/激活状态和待交付数据在同一锁内更新，因此并发 Wait 不会误报“尚未激活”。

## Tick Mailbox 与重传

SDK 内部维护三个逻辑槽位：

- `queued`：已收到、尚未交给业务线程的最新 Tick。
- `outstanding`：已经由 `WaitForTick()` 返回、尚未成功 Commit 的 Tick。
- `committed`：最近提交的 Tick 和不可变 Commit，用于幂等重发。

规则如下：

- 同一时间只允许一个 outstanding Tick。未提交时再次 Wait 明确报错，不静默丢弃。
- 相同 identity/payload 的 Tick 在 outstanding 阶段不重复交付；在 committed 阶段自动重发
  相同 Commit。timestamp 差异不视为 payload 冲突。
- identity 相同但 observation/layout 等 payload 不同时拒绝并追加去重 lifecycle warning；
  tick ID 小于已知最新值时忽略。
- `TIMED_OUT` Server 会继续推进并发布新 Tick；SDK 的 queued 槽位只保留最大 tick ID。
  outstanding Tick 不会被撤销，提交后下一次 Wait 获取当时缓存的最新值。
- eCAL callback 不调用用户代码，也不会因控制计算耗时阻塞接收线程。
- 每个合法新 Tick或重传 Tick都会自动发送 Heartbeat；Heartbeat 不表示 Commit 已完成。

## Commit 校验

`Commit()` 只接受当前 outstanding Tick，或最近 committed Tick 的幂等重试。SDK 使用注册
layout 执行以下本地校验：

- 每个注册 command 必须且只能出现一次，不允许空名称、未知或遗漏 actuator。
- 当前 `CommandValue` API 只支持 `dimension == 1` 的标量 actuator。
- command 必须为有限 `double`；有 `ctrlrange` 时必须位于闭区间内。
- Tick 必须与 SDK 保存的 server/controller/session/epoch/protocol/layout 和 payload 一致。

命令按 registration layout 顺序编码，因此输入 vector 顺序不影响 wire payload。校验失败
不会清除 outstanding Tick，调用方可修正后重试。首次成功后该 Tick 的 Commit 内容不可变；
相同内容可显式重发，不同内容会在本地拒绝，避免触发 Server 的 duplicate-payload 错误。

SDK 不提供 `CommitError()`。控制算法无法产生合法命令时，应保存业务诊断并 Stop/Unregister；
局部输入错误不需要通过空命令把 Server session 置为 `ERROR`。

## 最小示例

```cpp
#include <chrono>
#include <iostream>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

int main() {
  using namespace std::chrono_literals;
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.adapter_id = "minimal_controller";
  options.observation_names = {"hinge_pos", "hinge_vel"};
  options.command_names = {"hinge_motor"};

  mujoco_sim::adapter_sdk::SyncAdapterClient client(options);
  std::string error;
  if (!client.Start(&error)) {
    std::cerr << error << '\n';
    return 1;
  }
  const auto unregister = [&] {
    std::string unregister_error;
    if (client.Unregister(&unregister_error)) return true;
    std::cerr << "unregister failed: " << unregister_error << '\n';
    return false;
  };

  // Initialize controllers that depend on client.registration().layout() here.
  if (!client.Activate(&error)) {
    std::cerr << error << '\n';
    unregister();
    return 1;
  }

  std::uint64_t active_ticks = 0;
  while (active_ticks < 5) {
    mujoco_sim::v2::Tick tick;
    const auto result = client.WaitForTick(100ms, &tick, &error);
    if (result == mujoco_sim::adapter_sdk::TickWaitResult::kTimeout) continue;
    if (result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
      std::cerr << error << '\n';
      unregister();
      return 1;
    }

    std::vector<mujoco_sim::adapter_sdk::CommandValue> commands;
    for (const auto& descriptor : tick.layout().commands()) {
      commands.push_back({descriptor.name(), 0.1});
    }
    if (!client.Commit(tick, commands, &error)) {
      std::cerr << error << '\n';
      unregister();
      return 1;
    }
    if (tick.controller_state() == mujoco_sim::v2::ACTIVE) ++active_ticks;
  }

  return unregister() ? 0 : 1;
}
```

Server 与 Adapter 的 `server_name` 必须一致，资源必须存在且 actuator 未被其他 Controller
占用。运行期注册、激活、超时恢复和注销均受支持。

## 测试

```bash
colcon test --packages-select mujoco_sim_adapter_sdk
colcon test-result --verbose
```

- `adapter_sdk_test` 覆盖三个 Client。Observer 测试包含无 Commit API、schedule 校验、
  queue/drop、timeout/stop、身份/layout/sequence/minor、status replacement、heartbeat 内容和
  确定性 callback 排空、`STOPPED`/注销与并发 Wait 的原子终止；stream 测试包含 descriptor、
  malformed frame、view copy、callback/wait 排他、queue/drop、同 Tick
  `RUNNING -> STREAM_ERROR`、timeout/stop、异常隔离和 callback UAF state；Sync 测试同样覆盖
  `STOPPED`/注销的原子终止以及原有 Tick/Commit 行为。
- `ecal_runtime_static_lifetime_test` 以进程退出场景验证 static/global Client 持有的共享 eCAL
  runtime 不会遭遇静态析构顺序问题。
- `server_terminal_state_ecal_test` 通过真实 eCAL transport 验证 Server `STOPPED` 与
  `ERROR_PAUSED` 分别映射为 Controller/Observer 的 `kStopped` 与 `kError`。
- `sensor_stream_ecal_test` 建立真实 eCAL registry 与 status topic，验证同 Tick
  `RUNNING -> STREAM_ERROR` 跨传输唤醒正在等待的 `SensorStreamClient`。
- `sensor_runtime_ecal_test` 连接生产 `SensorRuntime` 与真实 eCAL transport，注入运行期
  capture failure，验证服务端生成的 `STREAM_ERROR` 传播到 SDK 且唤醒 Wait。
- `adapter_sdk_lifecycle_e2e` 启动真实 headless Server，故意持有首个 Tick 超过 deadline，
  验证 `TIMED_OUT -> recovery pending -> ACTIVE`、显式 Commit 和最终注销。
- `sensor_observer_e2e` 在同一进程组合共享 eCAL runtime 的 Observer 与 Stream Client，连接
  真实无 DISPLAY Server；销毁首个 runtime owner 后其余 Client 继续通信，并使用独立 gate
  Controller 将正常 Commit、timeout、open-Barrier unregister 和局部 command `ERROR` 每类
  completed-step 固定在唯一 Tick。测试要求 IMU ObservationFrame 与 blocking LiDAR raw frame
  的 Tick 精确匹配，同时覆盖空 Barrier、非空 RGB、注销和 Server 持续运行。
- Sample Adapter 另行覆盖正常 headless/GUI 五 Tick Barrier；WBR Demo 覆盖长时间闭环控制。
