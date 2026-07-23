#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/v4/sync.pb.h>

#include "adapter_transport.hpp"
#include "callback_gate.hpp"
#include "lifecycle_operation.hpp"

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Immutable storage owned by one StateLease.
 *        一个 StateLease 所持有的不可变存储。
 *
 * The receive callback completes both members before publishing the shared
 * object. Readers subsequently access it through shared ownership only.
 * 接收回调会先完整写入两个成员，再发布共享对象；此后读取方仅通过共享所有权访问。
 */
struct StateLease::Data {
  /// Parsed Protocol V4 frame metadata. 解析后的 Protocol V4 帧元数据。
  mujoco_sim::protocol::DataFrameHeader header;
  /// Dense observation scalars owned by this object. 本对象持有的稠密观测标量。
  std::vector<double> scalars;
};

/**
 * @brief Implements one single-use Controller client session.
 *        实现一个单次使用的控制器客户端会话。
 *
 * LifecycleOperation serializes Start(), Activate(), ReportFault(), Stop(),
 * and Unregister() without retaining its mutex during transport calls or
 * joins. mutex protects mutable session data, command mailboxes, State data,
 * and terminal diagnostics. stop is published before Stop() waits for an
 * admitted lifecycle operation.
 *
 * LifecycleOperation 会串行化 Start()、Activate()、ReportFault()、Stop() 与
 * Unregister()，但执行传输调用或线程等待时不持有其互斥锁。mutex 保护可变会话数据、
 * 指令邮箱、State 数据和终止诊断。Stop() 等待已准入生命周期操作前会先发布 stop。
 *
 * @par Thread safety / 线程安全
 * Public methods follow the concurrency contract documented by ControllerClient.
 * Transport callbacks enter through CallbackGate and never retain mutex while
 * invoking eCAL. 公共方法遵循 ControllerClient 所记录的并发契约；传输回调通过
 * CallbackGate 准入，调用 eCAL 时不会持有 mutex。
 */
class ControllerClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  /// Grants the test peer controlled access to deterministic injection points.
  /// 允许测试 peer 受控访问确定性的注入点。
  friend class ControllerClientTestPeer;

  /** @brief Creates a non-started implementation. 创建尚未启动的实现。
   * @param options Immutable options copied into the session. 复制到会话中的不可变选项。
   */
  explicit Impl(ControllerOptions options);

  /**
   * @brief Performs bounded local shutdown and best-effort remote cleanup.
   *        执行有界本地关闭并尽力清理远端会话。
   */
  ~Impl();

  /** @brief Validates immutable Controller options. 校验不可变的控制器选项。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true when all options are valid. 所有选项有效时返回 true。
   */
  bool Validate(std::string* error_message);

  /** @brief Builds the registration request without performing I/O. 构造注册请求且不执行 I/O。
   * @return Owning Protocol V4 request. 自有的 Protocol V4 请求。
   */
  mujoco_sim::v4::RegisterAdapterRequest RegistrationRequest() const;

  /** @brief Validates and commits a successful registration response. 校验并提交成功注册响应。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true when the response defines a valid immutable layout. 响应定义有效不可变布局时返回
   * true。
   */
  bool AcceptRegistration(std::string* error_message);

  /** @brief Registers the provisional session and starts receive endpoints.
   * 注册临时会话并启动接收端点。
   * @param error_message Non-null diagnostic output cleared on entry. 入口处清空的非空诊断输出。
   * @return true after local startup commits. 本地启动提交后返回 true。
   * @par Blocking / 阻塞行为 May wait for the bounded registration RPC. 可能等待有界注册 RPC。
   */
  bool Start(std::string* error_message);

  /** @brief Finishes local startup after remote registration. 在远端注册后完成本地启动。
   * @param error_message Non-null startup diagnostic. 非空启动诊断。
   * @return true when reception and local state commit. 接收端和本地状态提交后返回 true。
   */
  bool FinishStart(std::string* error_message);

  /** @brief Attempts to unregister a remotely created session after local failure.
   * 本地失败后尝试注销远端会话。
   * @param error_message Non-null original diagnostic extended on cleanup failure.
   * 非空原始诊断；清理失败时追加内容。
   * @par Blocking / 阻塞行为 May wait for the bounded unregister RPC. 可能等待有界注销 RPC。
   */
  void CompensateRegistration(std::string* error_message);

  /** @brief Cancels local activation and compensates its remote effects.
   * 取消本地激活并补偿其远端影响。
   * @param error_message Non-null activation diagnostic. 非空激活诊断。
   */
  void AbortActivation(std::string* error_message);

  /** @brief Creates State and status subscriptions and enables callbacks. 创建 State
   * 与状态订阅并启用回调。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true when both endpoints are ready. 两个端点均就绪时返回 true。
   */
  bool StartReception(std::string* error_message);

  /**
   * @brief Disables, removes, and drains receive callbacks.
   *        禁用、移除并清空接收回调。
   * @par Blocking / 阻塞行为
   * Waits for callbacks running on other threads, never for the caller's own
   * callback lease. 等待其他线程上的回调，不等待调用线程自身的回调租约。
   */
  void StopReception() noexcept;

  /** @brief Activates the provisional session and starts command publication.
   * 激活临时会话并启动指令发布。
   * @param error_message Non-null diagnostic output cleared on entry. 入口处清空的非空诊断输出。
   * @return true after remote and local activation commit. 远端和本地激活均提交后返回 true。
   */
  bool Activate(std::string* error_message);

  /** @brief Tests the State wait predicate while mutex is held. 持有 mutex 时检查 State 等待谓词。
   * @param require_new Whether the sequence must exceed last_read. 序列是否必须大于 last_read。
   * @return true when a read should finish. 读取应结束时返回 true。
   * @pre mutex is held. 已持有 mutex。
   */
  bool StateReady(bool require_new) const;

  /** @brief Waits for State, stop, or terminal error. 等待 State、停止或终止错误。
   * @param lock Lock owning mutex. 持有 mutex 的锁。
   * @param wait Whether to block for a new sequence. 是否阻塞等待新序列。
   * @param timeout Maximum wait duration. 最大等待时长。
   * @return true when the predicate became ready. 谓词就绪时返回 true。
   */
  bool WaitForState(std::unique_lock<std::mutex>& lock, bool wait,
                    std::chrono::milliseconds timeout);

  /** @brief Implements polling and blocking State reads. 实现轮询及阻塞 State 读取。
   * @param output Non-null transactional lease output. 非空事务性租约输出。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @param wait Whether a newer sequence is required. 是否要求更新的序列。
   * @param timeout Maximum wait duration. 最大等待时长。
   * @return Public State read result. 公开的 State 读取结果。
   */
  StateReadResult Read(StateLease* output, std::string* error_message, bool wait,
                       std::chrono::milliseconds timeout);

  /** @brief Validates finite command values and registered ranges. 校验指令有限性及注册范围。
   * @param values Exact dense command layout. 精确的稠密指令布局。
   * @return true when every scalar is valid. 每个标量均有效时返回 true。
   * @pre mutex is held and values has the registered scalar count. 已持有 mutex，且 values
   * 数量与注册值一致。
   */
  bool CommandsValid(std::span<const double> values) const;

  /** @brief Replaces the depth-one latest-command mailbox. 替换深度为一的最新指令邮箱。
   * @param values Dense command values copied by the call. 本次调用复制的稠密指令值。
   * @param error_message Non-null diagnostic output cleared on entry. 入口处清空的非空诊断输出。
   * @return true when the local mailbox accepts the command. 本地邮箱接受指令时返回 true。
   */
  bool SetCommand(std::span<const double> values, std::string* error_message);

  /** @brief Publishes one validated Controller fault. 发布一条已校验的控制器故障。
   * @param code Public non-unspecified fault code. 公开的非未指定故障码。
   * @param message Fault text of at most 4096 bytes. 最多 4096 字节的故障文本。
   * @param error_message Non-null diagnostic output cleared on entry. 入口处清空的非空诊断输出。
   * @return true when eCAL accepts the encoded payload. eCAL 接受编码 payload 时返回 true。
   */
  bool ReportFault(FaultCode code, std::string_view message, std::string* error_message);

  /** @brief Runs retry and keepalive publication on the owned thread. 在自有线程运行重试与
   * keepalive 发布。
   * @par Thread safety / 线程安全 Called only by publisher. 仅由 publisher 线程调用。
   */
  void PublishLoop();

  /** @brief Thread boundary converting exceptions into terminal errors.
   * 将异常转换为终止错误的线程边界。
   */
  void RunPublishLoop() noexcept;

  /** @brief Records the first terminal error and wakes waiters. 记录首个终止错误并唤醒等待方。
   * @param message Owning contextual diagnostic. 自有的上下文诊断。
   * @par Thread safety / 线程安全 May run on publisher or callback threads.
   * 可在发布或回调线程运行。
   */
  void SetTerminalError(std::string message);

  /** @brief Validates and accepts one borrowed State frame. 校验并接受一帧借用的 State 数据。
   * @param data Callback-owned bytes valid only during this call.
   * 仅本次调用期间有效的回调自有字节。
   * @param size Byte count. 字节数。
   */
  void OnState(const void* data, std::size_t size);

  /** @brief Applies one borrowed ServerStatus update. 应用一条借用的 ServerStatus 更新。
   * @param data Callback-owned bytes valid only during this call.
   * 仅本次调用期间有效的回调自有字节。
   * @param size Byte count. 字节数。
   */
  void OnStatus(const void* data, std::size_t size);

  /** @brief Publishes local cancellation under mutex. 在 mutex 保护下发布本地取消。
   */
  void StopLocal();

  /** @brief Cancels work, joins the publisher, and drains reception.
   * 取消工作、等待发布线程并清空接收。
   */
  void Stop();

  /** @brief Joins the publisher unless called by that same thread.
   * 等待发布线程，除非由该线程自身调用。
   */
  void JoinPublisher();

  /** @brief Performs retryable remote cleanup and then local shutdown.
   * 执行可重试远端清理，再关闭本地工作。
   * @param error_message Non-null diagnostic output cleared on entry. 入口处清空的非空诊断输出。
   * @return true for successful or already-complete cleanup. 清理成功或已完成时返回 true。
   */
  bool Unregister(std::string* error_message);

  /// Monotonic clock used only for local publication deadlines. 仅用于本地发布日期的单调时钟。
  using Clock = std::chrono::steady_clock;
  /// Immutable construction options. 不可变构造选项。
  ControllerOptions options;
  /// Last successful wire registration response. 最近一次成功的线端注册响应。
  mujoco_sim::v4::RegisterAdapterResponse wire;
  /// Public immutable registration projection. 公开的不可变注册投影。
  AdapterRegistration registration;
  /// Command layout identity copied from registration. 从注册响应复制的指令布局标识。
  mujoco_sim::protocol::LayoutFingerprint fingerprint{};
  /// Server-granted command keepalive period. Server 授予的指令 keepalive 周期。
  std::chrono::milliseconds keepalive{100};
  /// Shared transport retained through remote cleanup. 保留到远端清理完成的共享传输。
  std::shared_ptr<internal::AdapterTransport> transport;
  /// Process-wide eCAL runtime lease. 进程级 eCAL 运行时租约。
  std::shared_ptr<void> runtime;
  /// Callback gate specialized for this implementation. 针对此实现的回调门控。
  using CallbackGate = internal::CallbackGate<Impl>;
  /// Shared callback admission state. 共享回调准入状态。
  using CallbackState = CallbackGate::State;
  /// One admitted callback invocation. 一次已准入的回调调用。
  using CallbackLease = CallbackGate::Lease;
  /// State that outlives registered callback functors. 生命周期长于已注册回调函数的状态。
  std::shared_ptr<CallbackState> callback_state;
  /// Controller State subscriber, owned after StartReception. StartReception 后持有的 State
  /// 订阅者。
  std::unique_ptr<eCAL::CSubscriber> state_sub;
  /// Server status subscriber, owned after StartReception. StartReception 后持有的 Server
  /// 状态订阅者。
  std::unique_ptr<eCAL::CSubscriber> status_sub;
  /// Serializes lifecycle operations without retaining a lock during them.
  /// 不在操作期间持锁的生命周期串行器。
  internal::LifecycleOperation lifecycle;
  /// Protects all non-atomic mutable session and mailbox fields below.
  /// 保护下列全部非原子可变会话及邮箱字段。
  std::mutex mutex;
  /// Wakes State readers and the command publisher when protected predicates change.
  /// 受保护谓词变化时唤醒 State 读取方和指令发布方。
  std::condition_variable condition;
  /// Owned command publication thread. 自有的指令发布线程。
  std::thread publisher;
  /// Test-only injection executed immediately before thread creation.
  /// 仅测试使用、在线程创建前立即执行的注入点。
  std::function<void()> before_publisher_start;
  /// Lock-free cancellation flag observed by RPC polling and worker loops. RPC
  /// 轮询与工作循环观察的无锁取消标志。
  std::atomic<bool> stop{false};
  /// Whether provisional local startup committed. 临时本地启动是否已提交。
  bool started = false;
  /// Whether local activation committed. 本地激活是否已提交。
  bool active = false;
  /// Whether a remote session may still exist. 远端会话是否可能仍存在。
  bool remote_registered = false;
  /// Whether remote cleanup completed idempotently. 远端清理是否已幂等完成。
  bool unregistered = false;
  /// Enforces at most one State reader. 保证最多一个 State 读取方。
  bool waiting = false;
  /// Enables missing-session detection only after one matching status.
  /// 仅在出现过匹配状态后启用会话缺失检测。
  bool seen_status = false;
  /// Latest immutable State frame, protected by mutex. mutex 保护的最新不可变 State 帧。
  std::shared_ptr<const StateLease::Data> latest;
  /// Latest dense command generation, protected by mutex. mutex 保护的最新稠密指令代次。
  std::vector<double> latest_command;
  /// Monotonic local mailbox generation. 单调递增的本地邮箱代次。
  std::uint64_t generation = 0;
  /// Next Protocol V4 Command sequence. 下一条 Protocol V4 指令序列。
  std::uint64_t next_sequence = 1;
  /// Last State sequence returned to the reader. 最近返回给读取方的 State 序列。
  std::uint64_t last_read = 0;
  /// Sticky first terminal diagnostic, protected by mutex. mutex 保护的首个粘性终止诊断。
  std::string terminal;
};

}  // namespace mujoco_sim::adapter_sdk
