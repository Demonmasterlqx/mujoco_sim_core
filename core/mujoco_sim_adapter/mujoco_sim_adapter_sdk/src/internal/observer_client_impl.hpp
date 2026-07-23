#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/v4/sync.pb.h>

#include "adapter_session.hpp"
#include "adapter_transport.hpp"
#include "blocking_mailbox.hpp"
#include "callback_gate.hpp"
#include "lifecycle_operation.hpp"

namespace mujoco_sim::adapter_sdk {

/** @brief Immutable storage owned by one ObservationLease.
 *         一个 ObservationLease 所持有的不可变存储。
 *
 * A receive callback completes both fields before publishing shared ownership.
 * 接收回调会在发布共享所有权前完整写入两个字段。
 */
struct ObservationLease::Data {
  /// Parsed Protocol V4 observation frame metadata. 解析后的 Protocol V4 观测帧元数据。
  mujoco_sim::protocol::DataFrameHeader header;
  /// Dense observation scalars owned by this frame. 本帧持有的稠密观测标量。
  std::vector<double> scalars;
};

/**
 * @brief Implements one single-use Observer session and its heartbeat worker.
 *        实现一个单次使用的 Observer 会话及其心跳工作线程。
 *
 * lifecycle_ serializes lifecycle admission without retaining its mutex during
 * RPC, callback teardown, or joins. mutex_ protects session state, mailbox,
 * counters, and diagnostics. heartbeat_thread_mutex_ protects thread-handle
 * ownership only and is never held while joining. Transport callbacks enter
 * through callback_state_. No project lock may be held while calling eCAL,
 * transport methods, removing callbacks, or joining a thread.
 * lifecycle_ 在 RPC、回调拆除或线程等待期间不持有其互斥锁地串行化生命周期准入。
 * mutex_ 保护会话状态、邮箱、计数与诊断。heartbeat_thread_mutex_ 仅保护线程句柄所有权，
 * 等待线程退出时绝不持有。传输回调通过 callback_state_ 准入。调用 eCAL、传输方法、
 * 移除回调或等待线程时不得持有项目锁。
 */
class ObserverClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  /** @brief Creates a non-started implementation and owns its options.
   *         创建未启动实现并持有其配置。
   * @param options Immutable construction options. 不可变构造配置。
   */
  explicit Impl(ObserverOptions options);
  /** @brief Stops workers, drains callbacks, and attempts best-effort cleanup.
   *         停止工作线程、清空回调并尝试尽力清理。
   */
  ~Impl();
  /** @brief Implements provisional registration and local startup.
   *         实现临时注册与本地启动。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return true after registration, reception, and heartbeat startup. 注册、接收与心跳启动后为
   * true。
   */
  bool Start(std::string* error_message);
  /** @brief Implements remote activation and local commit.
   *         实现远端激活与本地提交。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return true when activation commits locally. 激活在本地提交时为 true。
   */
  bool Activate(std::string* error_message);
  /** @brief Implements transactional single-waiter Observation delivery.
   *         实现事务性的单等待方 Observation 投递。
   * @param timeout Non-negative wait duration. 非负等待时长。
   * @param frame Required output modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return Public wait outcome. 公开等待结果。
   */
  ObservationWaitResult WaitForObservation(std::chrono::milliseconds timeout,
                                           ObservationLease* frame, std::string* error_message);
  /** @brief Publishes idempotent lock-free cancellation and wakes local waits.
   *         发布幂等无锁取消状态并唤醒本地等待。
   */
  void Stop();
  /** @brief Performs retryable remote cleanup under lifecycle admission.
   *         在生命周期准入下执行可重试远端清理。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return true when no remote session remains. 远端不再存在会话时为 true。
   */
  bool Unregister(std::string* error_message);
  /** @brief Returns synchronized sequence-gap and eviction count.
   *         返回同步后的序列缺口与淘汰计数。
   * @return Accumulated dropped frames. 累计丢帧数。
   */
  std::uint64_t dropped_frames() const;
  /** @brief Returns a synchronized diagnostic snapshot.
   *         返回同步后的诊断快照。
   * @return Owned warning/error text. 自有警告/错误文本。
   */
  std::string lifecycle_warning() const;

 private:
  /// Grants the public wrapper access to immutable registration state.
  /// 允许公开包装访问不可变注册状态。
  friend class ObserverClient;
  /// Grants focused tests access to deterministic receive/lifecycle hooks.
  /// 允许定向测试访问确定性接收/生命周期钩子。
  friend class ObserverClientTestPeer;

  /** @brief Validates immutable options without I/O.
   *         不执行 I/O 地校验不可变配置。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true when identifiers, timeouts, cadence, and capacity are valid.
   *         标识符、超时、节拍与容量均有效时为 true。
   */
  bool ValidateOptions(std::string* error_message) const;
  /** @brief Validates and projects the successful wire registration.
   *         校验成功的线端注册并生成公开投影。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true for matching identity, role, layout, and provisional state.
   *         身份、角色、布局与临时状态均匹配时为 true。
   */
  bool ValidateRegistration(std::string* error_message);
  /** @brief Clears mailbox, counters, status evidence, and diagnostics for registration.
   *         为新注册清空邮箱、计数、状态证据与诊断。
   * @par Thread safety / 线程安全
   * Acquires mutex_ internally and performs no external calls. 内部获取 mutex_，不调用外部代码。
   */
  void ResetSessionData();
  /** @brief Creates Observation/status subscribers and enables callback admission.
   *         创建 Observation/status 订阅者并启用回调准入。
   * @param error_message Non-null endpoint diagnostic. 非空端点诊断。
   * @return true when both receive endpoints are installed. 两个接收端点均安装后为 true。
   */
  bool StartObservationReception(std::string* error_message);
  /** @brief Starts the heartbeat worker and checks cancellation.
   *         启动心跳工作线程并检查取消状态。
   * @param error_message Non-null startup diagnostic. 非空启动诊断。
   * @return true when the worker is running and startup remains admitted.
   *         工作线程运行且启动仍获准时为 true。
   */
  bool StartHeartbeat(std::string* error_message);
  /** @brief Cancels and drains partial startup, then attempts compensating cleanup.
   *         取消并清理部分启动，再尝试补偿清理。
   * @pre The caller owns lifecycle admission and no project mutex.
   * 调用方持有生命周期准入且未持项目锁。
   */
  void AbortStart();

  /**
   * @brief Rolls back a remotely accepted activation that cannot be committed locally.
   *        回滚已被远端接受、但无法在本地提交的激活操作。
   *
   * Local cancellation is published first, then heartbeat and callbacks are
   * drained before remote unregister. Cleanup failure preserves session identity.
   * 先发布本地取消，再等待心跳并清空回调，最后注销远端会话。清理失败会保留会话标识。
   *
   * @param error_message Non-null activation diagnostic extended on cleanup failure.
   *                      非空激活诊断；清理失败时追加内容。
   * @pre Caller owns lifecycle admission and no project mutex. 调用方持有生命周期准入且未持项目锁。
   * @par Blocking / 阻塞行为
   * May join heartbeat, drain callbacks, and perform bounded unregister RPC.
   * 可能等待心跳、清空回调并执行有界注销 RPC。
   */
  void AbortActivation(std::string* error_message);
  /** @brief Sends registration and records remote cleanup ownership.
   *         发送注册并记录远端清理所有权。
   * @param error_message Non-null RPC/response diagnostic. 非空 RPC/响应诊断。
   * @return true after a valid registration is projected. 有效注册完成投影后为 true。
   */
  bool Register(std::string* error_message);
  /** @brief Commits local startup unless cancellation won the race.
   *         提交本地启动，除非取消在竞争中先发生。
   * @param error_message Non-null startup diagnostic. 非空启动诊断。
   * @return true after provisional local commit. 本地临时状态提交后为 true。
   */
  bool FinishStart(std::string* error_message);
  /** @brief Waits for activation delay while observing cancellation.
   *         在观察取消状态的同时等待激活延迟。
   * @param delay Non-negative local delay. 非负本地延迟。
   * @return false when cancellation occurs. 发生取消时为 false。
   */
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  /** @brief Performs one retryable unregister attempt for owned session identity.
   *         对持有的会话标识执行一次可重试注销尝试。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return true when no remote cleanup remains. 不再需要远端清理时为 true。
   */
  bool UnregisterSession(std::string* error_message);
  /** @brief Interprets unregister response and commits successful cleanup.
   *         解释注销响应并提交成功清理。
   * @param response Borrowed RPC response. 借用的 RPC 响应。
   * @param error_message Non-null response diagnostic. 非空响应诊断。
   * @return true for OK, NOT_FOUND, or INVALID_SESSION. 对 OK、NOT_FOUND 或 INVALID_SESSION 为
   * true。
   */
  bool CompleteUnregister(const mujoco_sim::v4::UnregisterAdapterResponse& response,
                          std::string* error_message);
  /** @brief Releases session ownership and clears queued frames under mutex_.
   *         在 mutex_ 下释放会话所有权并清空排队帧。
   * @pre mutex_ is held. 已持有 mutex_。
   */
  void MarkStoppedLocked();
  /** @brief Atomically commits normal stop and wakes observation/heartbeat waits.
   *         原子提交正常停止并唤醒观测/心跳等待。
   */
  void TransitionToStopped();
  /** @brief Parses borrowed eCAL bytes and validates frame identity/layout.
   *         解析借用的 eCAL 字节并校验帧身份/布局。
   * @param data Callback-owned bytes valid only for this call. 仅本次调用有效的回调自有字节。
   * @param size Byte count. 字节数。
   */
  void HandleObservationBytes(const void* data, std::size_t size);
  /** @brief Parses borrowed ServerStatus and converges terminal state.
   *         解析借用 ServerStatus 并收敛终止状态。
   * @param data Callback-owned bytes valid only for this call. 仅本次调用有效的回调自有字节。
   * @param size Byte count. 字节数。
   */
  void HandleStatusBytes(const void* data, std::size_t size);
  /** @brief Enqueues one owned, validated, monotonically newer frame.
   *         将一帧自有、已校验且序列更新的数据入队。
   * @param frame Shared immutable frame transferred into the bounded mailbox.
   *              转入有界邮箱的共享不可变帧。
   */
  void HandleObservation(std::shared_ptr<const ObservationLease::Data> frame);
  /** @brief Records the first terminal error, cancels work, and wakes waiters.
   *         记录首个终止错误、取消工作并唤醒等待方。
   * @param error Non-empty contextual diagnostic. 非空上下文诊断。
   * @par Thread safety / 线程安全
   * May run on receive or heartbeat threads. 可在接收或心跳线程运行。
   */
  void SetTerminalError(const std::string& error);

  /** @brief Records a non-terminal lifecycle diagnostic without duplication.
   *         记录非终止生命周期诊断并避免重复。
   * @param warning Non-empty contextual diagnostic. 非空上下文诊断。
   * @par Thread safety / 线程安全
   * Acquires mutex_; may run on heartbeat/teardown threads. 获取 mutex_；可在心跳/拆除线程运行。
   */
  void RecordLifecycleWarning(const std::string& warning);
  /** @brief Disables reception, removes callbacks, and drains other-thread callbacks.
   *         禁用接收、移除回调并等待其他线程上的回调退出。
   *
   * eCAL teardown exceptions become non-terminal warnings. No client mutex is
   * held while invoking eCAL or waiting. eCAL 拆除异常会成为非终止警告；调用 eCAL
   * 或等待时不持有客户端互斥锁。
   */
  void StopObservationReception() noexcept;
  /** @brief Creates the owned heartbeat thread unless cancellation is set.
   *         在未取消时创建自有心跳线程。
   * @return true when a thread exists; false when cancelled. 线程存在时为 true，已取消时为 false。
   * @par Thread safety / 线程安全
   * Serializes handle ownership with heartbeat_thread_mutex_. 使用 heartbeat_thread_mutex_
   * 串行化句柄所有权。
   */
  bool StartHeartbeatThread();
  /** @brief Transfers out and joins the heartbeat thread without holding its mutex.
   *         在不持有线程互斥锁的情况下转移并等待心跳线程退出。
   */
  void JoinHeartbeatThread();
  /** @brief Heartbeat thread boundary converting exceptions to terminal errors.
   *         将异常转换为终止错误的心跳线程边界。
   */
  void RunHeartbeatLoop() noexcept;
  /** @brief Publishes heartbeat cadence until cancellation or terminal transport error.
   *         按节拍发布心跳，直到取消或终止传输错误。
   * @par Thread safety / 线程安全
   * Called only by heartbeat_thread_; releases mutex_ for transport publication.
   * 仅由 heartbeat_thread_ 调用；执行传输发布时释放 mutex_。
   */
  void HeartbeatLoop();
  /** @brief Builds heartbeat identity and last-seen sequence without I/O.
   *         不执行 I/O 地构造心跳身份与最近序列。
   * @return Owning Protocol V4 heartbeat. 自有 Protocol V4 心跳。
   * @pre mutex_ is held and a session is owned. 已持有 mutex_ 且持有会话。
   */
  mujoco_sim::v4::SessionHeartbeat BuildHeartbeat() const;

  /// Callback admission gate specialized for this implementation. 针对此实现的回调准入门控。
  using CallbackGate = internal::CallbackGate<Impl>;
  /// Shared callback admission/accounting state. 共享回调准入/计数状态。
  using CallbackState = CallbackGate::State;
  /// One admitted callback invocation. 一次已准入回调调用。
  using CallbackLease = CallbackGate::Lease;

  /// Immutable construction options. 不可变构造配置。
  ObserverOptions options_;
  /// Last successful wire registration response; immutable after Start().
  /// 最近一次成功的线端注册响应；Start() 后不可变。
  mujoco_sim::v4::RegisterAdapterResponse wire_registration_;
  /// Public registration projection; immutable after Start(). Start() 后不可变的公开注册投影。
  AdapterRegistration registration_;
  /// Dense layout identity used by callback validation; immutable after Start().
  /// 回调校验使用的稠密布局标识；Start() 后不可变。
  mujoco_sim::protocol::LayoutFingerprint layout_fingerprint_{};
  /// Shared RPC/publication transport retained through cleanup. 保留至清理完成的共享 RPC/发布传输。
  std::shared_ptr<internal::AdapterTransport> transport_;
  /// Owned Observation subscriber; removed before destruction. 自有 Observation
  /// 订阅者；析构前移除。
  std::unique_ptr<eCAL::CSubscriber> observation_subscriber_;
  /// Owned Server status subscriber; removed before destruction. 自有服务器状态订阅者；析构前移除。
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  /// Admission state outliving registered callback functors. 生命周期长于已注册回调函数的准入状态。
  std::shared_ptr<CallbackState> callback_state_;
  /// Serializes lifecycle calls without retaining a lock during their work.
  /// 在工作期间不持锁地串行化生命周期调用。
  internal::LifecycleOperation lifecycle_;
  /// Protects heartbeat_thread_ ownership only; never nested with mutex_.
  /// 仅保护 heartbeat_thread_ 所有权；绝不与 mutex_ 嵌套持有。
  std::mutex heartbeat_thread_mutex_;
  /// Protects session_, mailbox, counters, monitor flag, and diagnostics below.
  /// 保护下列 session_、邮箱、计数、监控标志与诊断。
  mutable std::mutex mutex_;
  /// Wakes observation/activation waits when mailbox, stop, or error predicates change.
  /// 邮箱、停止或错误谓词变化时唤醒观测/激活等待。
  std::condition_variable condition_;
  /// Wakes heartbeat cadence early for cancellation/session release.
  /// 取消或会话释放时提前唤醒心跳节拍等待。
  std::condition_variable heartbeat_condition_;
  /// Owned heartbeat worker; handle ownership is protected by heartbeat_thread_mutex_.
  /// 自有心跳线程；句柄所有权由 heartbeat_thread_mutex_ 保护。
  std::thread heartbeat_thread_;
  /// Lock-free cancellation visible to RPC polling, callbacks, and heartbeat.
  /// RPC 轮询、回调与心跳可见的无锁取消状态。
  std::atomic<bool> stop_requested_{false};
  /// Process-wide eCAL runtime lease released after endpoints/transport. 在端点/传输后释放的进程级
  /// eCAL 租约。
  std::shared_ptr<void> ecal_runtime_;
  /// Local test-adjustable delay before activation RPC; defaults to 100 ms.
  /// 激活 RPC 前可由测试调整的本地延迟；默认为 100 ms。
  std::chrono::milliseconds activation_delay_{100};
  /// Whether waits should treat eCAL runtime shutdown as normal stop; protected by mutex_.
  /// 等待是否应将 eCAL 运行时关闭视为正常停止；由 mutex_ 保护。
  bool monitor_ecal_ = false;
  /// Lifecycle phase and remote cleanup ownership, protected by mutex_. 由 mutex_
  /// 保护的生命周期阶段与远端清理所有权。
  internal::AdapterSession session_;
  /// Bounded owning frame mailbox, accessed only under mutex_. 仅在 mutex_ 下访问的有界自有帧邮箱。
  internal::BlockingMailbox<std::shared_ptr<const ObservationLease::Data>> observations_;
  /// Last accepted frame sequence, protected by mutex_. 由 mutex_ 保护的最近接收帧序列。
  std::uint64_t last_sequence_ = 0;
  /// Last accepted completed-step tick, protected by mutex_. 由 mutex_ 保护的最近已完成步 tick。
  std::uint64_t last_tick_id_ = 0;
  /// Sequence gaps plus local evictions, protected by mutex_. 由 mutex_
  /// 保护的序列缺口与本地淘汰总数。
  std::uint64_t dropped_frames_ = 0;
  /// Exact-session status evidence enabling later disappearance detection; protected by mutex_.
  /// 启用后续消失检测的精确会话状态证据；由 mutex_ 保护。
  bool seen_status_ = false;
  /// Sticky first terminal diagnostic, protected by mutex_. 由 mutex_ 保护的首个粘性终止诊断。
  std::string terminal_error_;
  /// Accumulated non-terminal warnings including terminal context, protected by mutex_.
  /// 包含终止上下文的累计非终止警告；由 mutex_ 保护。
  std::string lifecycle_warning_;
};

}  // namespace mujoco_sim::adapter_sdk
