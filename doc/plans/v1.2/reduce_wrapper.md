需要避免，但不是“所有包装器都不要”。

这个仓库真正的问题是：

> **很多包装器没有提升抽象层次，只是把一次调用拆成了多次跳转，并且每一层还偷偷加入锁、日志、异常处理或状态修改。**

这类包装器通常不会造成明显运行性能问题，编译器可能会内联；但会严重增加：

* 阅读调用链的成本；
* 调试时的跳转次数；
* 副作用定位难度；
* 单元测试和真实运行路径不一致的风险；
* 修改一个行为时遗漏某个中间层的概率。

## 一、当前仓库中确实存在包装过度

### 1. Server RPC 调用链过长

以注册 Adapter 为例，目前大致是：

```text
eCAL SetMethodCallback
    ↓
RegistryEndpoint::RegisterMethod()
    ↓
内部 lambda
    ↓
InvokeGuardedServiceCallback()
    ↓
ServerApplication::HandleRegisterAdapter()
    ↓
RegistryEndpoint::Parse()
    ↓
ParseServicePayload()
    ↓
protobuf ParseFromString()
    ↓
SimulationCore::RegisterAdapter()
    ↓
RegisterControlAdapter() / Observer 注册逻辑
```

一次业务操作需要跨越：

* eCAL；
* Endpoint；
* callback guard；
* ServerApplication；
* protobuf parser；
* SimulationCore；
* 内部注册实现。

这里的问题不是函数数量本身，而是**职责分割的位置不合理**。

例如：

```cpp
RegistryEndpoint::Parse(...)
```

实际上 Endpoint 没有提供任何 Registry 特有语义，只是继续调用：

```cpp
ParseServicePayload(bytes, message);
```

这就是典型的无语义包装。

---

### 2. Commit 链路也有类似问题

当前 Commit 路径近似为：

```text
eCAL subscriber callback
    ↓
ControlEndpoint::SubscribeCommits() 的 lambda
    ↓
InvokeGuardedSubscriberCallback()
    ↓
ServerApplication::HandleCommitBytes()
    ↓
ControlEndpoint::ParseCommit()
    ↓
ParseTopicPayload()
    ↓
SimulationCore::SubmitCommit()
    ↓
CompleteBarrier()
    ↓
SensorRuntime::OnStepCompleted()
    ↓
CompleteSensorFence()
    ↓
PublishObservationFrames()
    ↓
FinishStepPipeline()
```

这条链路中更严重的问题是：**包装器不仅转发调用，还在不同层偷偷改变状态**。

例如：

* callback guard 管理活动回调；
* `HandleCommitBytes()` 获取两个 mutex；
* `SubmitCommit()` 可能完成物理 step；
* `BeginStepPipeline()` 改 scheduler 状态；
* `CompleteSensorFence()` 阻塞等待 sensor；
* `FinishStepPipeline()` 再改变 scheduler 状态。

因此开发者看到：

```cpp
core_.SubmitCommit(commit);
```

并不能直观判断它是否会：

* 修改 Controller 状态；
* 完成 Barrier；
* 调用 `mj_step`；
* 生成 observation；
* 导致 sensor pipeline 启动。

这是比函数跳转更多的维护问题。

---

### 3. SDK 中为测试而制造了大量 `XxxWith`

`SyncAdapterClient` 中有非常典型的链路：

```text
Activate()
    ↓
ActivateWith(lambda)
    ↓
CallService()
    ↓
eCAL
```

```text
Commit()
    ↓
CommitWith(lambda)
    ↓
PublishCommit()
    ↓
PublishSerialized(lambda)
    ↓
eCAL::Send()
```

```text
Unregister()
    ↓
UnregisterWith(lambda)
    ↓
CallService()
    ↓
CompleteUnregister()
```

对应位置包括：

* `sync_adapter_client.cpp:268`：`Activate`
* `sync_adapter_client.cpp:280`：`ActivateWith`
* `sync_adapter_client.cpp:537`：`Commit`
* `sync_adapter_client.cpp:550`：`CommitWith`
* `sync_adapter_client.cpp:638`：`Unregister`
* `sync_adapter_client.cpp:650`：`UnregisterWith`
* `sync_adapter_client.cpp:993`：`PublishSerialized`

这种设计大概率是为了测试时注入 lambda，但代价是生产代码调用链被测试接缝污染。

### 更好的做法

不要为每一个函数增加一个 `XxxWith(std::function<...>)`，而是在构造阶段注入一个 Transport 接口：

```cpp
class AdapterTransport {
 public:
  virtual ~AdapterTransport() = default;

  virtual Result<mujoco_sim::v2::RegisterAdapterResponse> Register(
      const mujoco_sim::v2::RegisterAdapterRequest& request) = 0;

  virtual Result<mujoco_sim::v2::ActivateAdapterResponse> Activate(
      const mujoco_sim::v2::ActivateAdapterRequest& request) = 0;

  virtual Result<mujoco_sim::v2::UnregisterAdapterResponse> Unregister(
      const mujoco_sim::v2::UnregisterAdapterRequest& request) = 0;

  virtual bool PublishCommit(
      const mujoco_sim::v2::AdapterCommit& commit) = 0;
};
```

真实实现：

```cpp
class EcalAdapterTransport final : public AdapterTransport {
  // 负责 eCAL、序列化、publisher mutex、服务调用
};
```

测试实现：

```cpp
class FakeAdapterTransport final : public AdapterTransport {
  // 测试直接配置响应
};
```

于是生产代码可以变成：

```cpp
bool SyncAdapterClient::Activate(std::string* error_message) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);

  if (!ValidateCanActivate(error_message)) {
    return false;
  }

  auto response = transport_->Activate(BuildActivateRequest());
  if (!response) {
    *error_message = response.error();
    return false;
  }

  return ApplyActivateResponse(*response, error_message);
}
```

这样可以删除：

* `ActivateWith()`；
* `CommitWith()`；
* `UnregisterWith()`；
* 多个临时 lambda；
* `std::function` 参数；
* 测试路径和生产路径之间的结构差异。

---

## 二、哪些包装器应该删除

判断一个包装器是否应该存在，可以问：

> 它是否增加了一项明确且独立的保证？

合理的保证包括：

* 类型转换；
* 协议序列化；
* 输入验证；
* 线程同步；
* 生命周期管理；
* 异常边界；
* 资源所有权；
* 稳定公共 API；
* 领域规则。

如果什么都没增加，只是换了一个名字继续调用，就应删除。

### 仓库中可以直接处理的例子

#### 1. `RequestStop()` 和 `StopWithReason()`

当前是：

```cpp
void ServerApplication::RequestStop(std::string reason) {
  StopWithReason(reason);
}
```

两者没有明显不同的语义，应合并成：

```cpp
void ServerApplication::RequestStop(std::string reason) {
  bool changed = false;
  // 原 StopWithReason 内容
}
```

除非真的存在一个要求“调用者已经持锁”的版本，那也应该明确命名：

```cpp
RequestStopLocked()
```

而不是泛泛地叫 `StopWithReason()`。

---

#### 2. Scheduler 中两个完全相同的通知函数

当前：

```cpp
void NotifyLifecycleChanged();
void NotifyCommitReceived();
```

实现完全一样，都是：

```cpp
{
  std::lock_guard lock(mutex_);
  ++wake_generation_;
}
condition_.notify_all();
```

这两个接口制造了虚假的语义区分。

短期可以合并：

```cpp
void Notify();
```

更长期在单写者事件循环完成后，整个 `SimulationScheduler` 都可以明显简化，甚至由事件队列自己的 condition variable 替代。

---

#### 3. 多个 `RegistryEndpoint::Parse()` 重载

现在有五个几乎完全一样的函数：

```cpp
bool RegistryEndpoint::Parse(
    const std::string& bytes,
    RegisterAdapterRequest* message) const {
  return ParseServicePayload(bytes, message);
}
```

这些函数不应该逐消息类型存在。

可以在 Transport 内部集中为一个模板：

```cpp
template <typename Message>
Result<Message> ParseProto(std::string_view bytes);
```

更好的方式是让 ServerApplication 根本不接收 bytes，而是接收类型化请求：

```cpp
RegisterAdapterResponse RegistryService::RegisterAdapter(
    const RegisterAdapterRequest& request);
```

字节解析只存在于 eCAL Transport 边界。

---

#### 4. `PublishSerialized(std::function<bool()>)`

当前：

```cpp
bool SyncAdapterClient::PublishSerialized(
    const std::function<bool()>& publish) {
  std::lock_guard publish_lock(publisher_mutex_);
  return publish();
}
```

调用时又构造 lambda：

```cpp
return PublishSerialized([this, &bytes] {
  return commit_publisher_ != nullptr &&
         commit_publisher_->Send(bytes);
});
```

这只是为了复用一个 mutex，没必要引入 `std::function`。

可以直接让 Transport 管理发送锁：

```cpp
bool EcalAdapterTransport::PublishCommit(
    const AdapterCommit& commit) {
  auto bytes = Serialize(commit);
  if (!bytes) {
    return false;
  }

  std::lock_guard lock(publisher_mutex_);
  return commit_publisher_ &&
         commit_publisher_->Send(*bytes);
}
```

---

#### 5. `EnterFatalError()` 调用 `EnterErrorPaused()`

当前：

```cpp
void SimulationCore::EnterFatalError(const std::string& message) {
  EnterErrorPaused(message);
}
```

这个问题不仅是包装多，而是**命名语义不一致**。

“Fatal Error”通常意味着不可恢复，而“Error Paused”通常意味着可能恢复。应该二选一：

* 如果两者行为完全一样，删除其中一个；
* 如果 Fatal 真的是致命错误，就实现不同状态和退出策略；
* 不要保留两个名字表达同一个行为。

---

## 三、哪些包装器应该保留

### 1. eCAL callback 的异常边界

类似：

```cpp
InvokeGuardedSubscriberCallback(...)
RunWithExceptionBoundary(...)
```

这类包装器有合理价值，因为 eCAL callback 不应该允许 C++ 异常逃逸到第三方框架。

但是异常边界只应该存在一次：

```text
eCAL callback
    ↓
唯一 exception boundary
    ↓
类型化消息或事件队列
```

不要在后面的 Application、Core、Publisher 每一层都重复套异常包装。

---

### 2. CallbackLease 和运行时生命周期

`CallbackLease`、`CallbackState` 用于防止对象销毁时仍有 callback 运行，这属于明确的生命周期保证，可以保留。

不过单写者事件循环完成后，这部分可以简化成：

```text
callback 存活
    ↓
解析消息
    ↓
入队
    ↓
立即返回
```

callback 不再长时间持有 `ServerApplication` 或操作 Core，生命周期问题会自然减少。

---

### 3. `EcalRuntime`

全局初始化和引用计数的 eCAL runtime 包装属于资源所有权管理，是合理包装。

它应该有清楚注释：

* 首次 acquire 是否初始化 eCAL；
* 最后一个 handle 销毁时是否 finalize；
* 是否允许跨线程；
* 与 Publisher、Subscriber 的销毁顺序。

---

### 4. `SimulationCore` 的公共 Facade

例如：

```cpp
SimulationCore::MakeStatus()
```

内部转给：

```cpp
status_builder_->Build(...)
```

这种包装可以保留，因为 `SimulationCore` 是公共 API，外部不应该直接依赖内部 `StatusBuilder`。

即使函数只有一行，只要它保护了模块边界，就不是坏包装。

---

### 5. 领域命名包装器

例如：

```cpp
CompleteEmptyBarrier()
```

虽然内部可能只是：

```cpp
return CompleteBarrier(StepCompletionSource::kEmptyBarrier);
```

但它表达了一个明确的领域事件，并固定了完成原因。这类一行函数可以保留。

判断重点不是“是否只有一行”，而是：

> 调用者看到这个名字，是否更容易理解业务含义。

---

## 四、建议将调用层次限制为三个语义层

我建议主调用路径固定为：

```text
Transport
    ↓
Application / Use Case
    ↓
Domain Engine
```

### Transport 层

只负责：

* eCAL；
* protobuf 序列化；
* payload size；
* callback exception boundary；
* 网络发送和接收。

例如：

```cpp
class EcalServerTransport;
```

### Application 层

只负责一次完整用例：

* 注册 Adapter；
* 激活 Adapter；
* 处理 Commit；
* 处理 Deadline；
* 完成 Step 后发布结果。

例如：

```cpp
class RegistryService;
class SimulationLoop;
```

### Domain 层

只负责：

* Controller 状态机；
* 资源所有权；
* Barrier；
* MuJoCo step；
* Observation 构造。

例如：

```cpp
class SimulationEngine;
```

调用链应接近：

```text
EcalServerTransport::OnRegister(bytes)
    ↓ 解析一次
RegistryService::RegisterAdapter(request)
    ↓
SimulationEngine::RegisterAdapter(request)
```

而不是在 Endpoint、Application 和 Core 之间反复解析、序列化、记录状态。

---

## 五、Server 端推荐的目标形态

### 当前

```text
eCAL callback
  → Endpoint lambda
  → callback guard
  → HandleCommitBytes
  → Endpoint ParseCommit
  → generic ParseTopicPayload
  → Core SubmitCommit
  → Scheduler BeginPipeline
  → SensorRuntime
  → Fence
  → Publish
  → Scheduler CompletePipeline
```

### 修改后

```text
eCAL callback
  → EcalTransport 解析
  → ServerEventQueue::Push(CommitReceived)
  → return
```

单写者线程：

```text
SimulationLoop::Handle(CommitReceived)
  → SimulationEngine::SubmitCommit
  → ExecuteStepEffects
```

其中 `SimulationEngine` 返回显式效果：

```cpp
struct EngineEffects {
  bool status_changed = false;
  std::optional<StepCompleted> step;
  std::vector<AdapterTransition> transitions;
  std::vector<TickToPublish> ticks;
  std::vector<ObservationFrame> observations;
};
```

Application 层统一处理：

```cpp
void SimulationLoop::ApplyEffects(EngineEffects effects) {
  PublishObservations(effects.observations);
  ScheduleSensorWork(effects.step);
  PublishStatusIfNeeded(effects.status_changed);
}
```

这样物理 step 的后处理只有一个入口，不再由：

* Commit；
* Deadline；
* Unregister；
* Empty Barrier；

分别复制一套逻辑。

---

## 六、应建立一套包装器准入规则

以后新增包装器时，应至少满足下面一项：

1. 建立模块边界；
2. 隐藏第三方依赖；
3. 管理资源所有权；
4. 保证线程安全；
5. 建立异常边界；
6. 完成数据格式转换；
7. 实现明确领域操作；
8. 提供稳定公共 ABI/API。

以下情况原则上禁止：

```cpp
A() { return B(); }
```

并且：

* A 和 B 处于同一抽象层；
* 参数和返回值完全相同；
* A 没有增加不变量；
* A 只被调用一次；
* 名字没有增加领域意义。

还应避免：

```cpp
Xxx()
XxxWith(std::function<...>)
```

这种测试注入方式。优先使用：

* 构造函数依赖注入；
* interface；
* template policy；
* fake transport；
* fake clock。

---

## 七、推荐实际整改顺序

### 第一批：低风险删除

优先处理：

* 合并 `RequestStop()` / `StopWithReason()`；
* 合并两个 Scheduler Notify；
* 删除五个 Endpoint Parse 重载；
* 集中重复的 `MethodInfo()`；
* 删除简单的枚举名转发函数；
* 检查 `EnterFatalError()` 的真实语义。

这批基本不改变业务行为。

### 第二批：清理 SDK 测试包装器

引入：

```cpp
AdapterTransport
EcalAdapterTransport
FakeAdapterTransport
```

然后删除：

* `ActivateWith()`；
* `CommitWith()`；
* `UnregisterWith()`；
* `PublishSerialized()`；
* 多个 per-call `std::function`。

这是很适合作为第一项正式重构的部分，风险相对可控。

### 第三批：类型化 Endpoint

让 Endpoint 只负责：

```cpp
BindRpc<Request, Response>()
Subscribe<Message>()
Publish<Message>()
```

ServerApplication 不再处理：

```cpp
const std::string& request
const void* data
std::size_t size
```

而是处理类型化消息。

### 第四批：单写者事件循环

这是最终解决“函数调用乱飞”和“状态修改乱飞”的关键。

完成后：

* callback 不再直接操作 Core；
* Deadline 不再和 Commit 并行 step；
* Unregister 不再单独复制 step pipeline；
* Sensor 后处理统一；
* Scheduler 补偿逻辑大幅消失。

## 最终结论

**需要避免的是无语义包装器，而不是所有小函数。**

这个仓库目前应该：

* 保留 Transport、RAII、异常边界、Facade 和领域操作包装；
* 删除同层转发、`XxxWith(lambda)`、重复 Parse、重复 Notify；
* 把测试注入从“函数级 lambda”改成“对象级依赖注入”；
* 将主链路稳定在 `Transport → Application → Domain`；
* 更重要的是让所有状态变化集中到单写者线程，而不是只减少函数数量。

否则即使机械地合并函数，调用链可能变短，但并发和副作用仍然会继续散落。
