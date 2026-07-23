#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Configures Observer registration, cadence, heartbeat, and buffering.
 *        配置 Observer 注册、节拍、心跳与缓冲行为。
 */
struct ObserverOptions {
  /// Server identifier used to derive endpoints; defaults to `"default"` and
  /// must satisfy Protocol V4 Server identifier rules.
  /// 用于派生端点的服务器标识符；默认为 `"default"`，且须符合 Protocol V4 服务器标识符规则。
  std::string server_name = "default";
  /// Non-empty Observer identifier, unique within the selected Server and valid
  /// under Protocol V4 Adapter identifier rules.
  /// 非空 Observer 标识符；在所选服务器内唯一，并符合 Protocol V4 Adapter 标识符规则。
  std::string observer_id;
  /// Non-empty observation resource list in requested dense layout order.
  /// 非空观测资源列表，顺序即请求的稠密布局顺序。
  std::vector<std::string> observation_names;
  /// Positive Server-side liveness timeout in milliseconds; defaults to 1000 ms.
  /// 正数的服务器端存活超时，单位为毫秒；默认为 1000 ms。
  std::uint32_t session_timeout_ms = 1000;
  /// Positive discovery and RPC timeout in milliseconds; defaults to 5000 ms.
  /// 正数的发现与 RPC 超时，单位为毫秒；默认为 5000 ms。
  std::uint32_t service_timeout_ms = 5000;
  /// Heartbeat period in milliseconds; defaults to 250 ms, must be at least
  /// 100 ms, and must not exceed one third of session_timeout_ms.
  /// 心跳周期，单位为毫秒；默认为 250 ms，至少为 100 ms，且不得超过
  /// session_timeout_ms 的三分之一。
  std::uint32_t heartbeat_period_ms = 250;
  /// Optional step cadence; when selected it must be greater than zero and
  /// period_s must be empty. 可选的步数节拍；选用时必须大于零，且 period_s 必须为空。
  std::optional<std::uint32_t> every_n_steps;
  /// Optional simulation-time cadence in seconds; defaults to 0.01 s, must be
  /// finite and positive, and must be empty when every_n_steps is selected.
  /// 可选的仿真时间节拍，单位为秒；默认为 0.01 s，必须有限且为正，并在选用
  /// every_n_steps 时置空。
  std::optional<double> period_s = 0.01;
  /// Positive bounded frame queue capacity; defaults to one newest frame.
  /// 正数的有界帧队列容量；默认为仅保留最新一帧。
  std::size_t queue_capacity = 1;
};

/** @brief Result of waiting for an Observer frame. 等待 Observer 帧的结果。 */
enum class ObservationWaitResult {
  /// A frame lease was written to the output. 已向输出写入帧租约。
  kReceived,
  /// The deadline elapsed; output is unchanged and diagnostic is empty.
  /// 截止期限已到；输出不变且诊断为空。
  kTimeout,
  /// Local cancellation, successful cleanup, Server stop, or eCAL shutdown
  /// ended the wait; output is unchanged and diagnostic is empty.
  /// 本地取消、成功清理、服务器停止或 eCAL 关闭终止等待；输出不变且诊断为空。
  kStopped,
  /// Invalid use or a sticky remote/lifecycle error ended the wait; output is
  /// unchanged and diagnostic is non-empty. 无效调用或粘性远端/生命周期错误终止等待；
  /// 输出不变且诊断非空。
  kError,
};

/**
 * @brief Owns one immutable Observation frame.
 *        持有一帧不可变 Observation 数据。
 *
 * The move-only lease owns shared immutable storage and may outlive the client.
 * Its span remains valid until destruction, replacement, or moving from it.
 * Default-constructed and moved-from leases are empty and return zero/empty values.
 * 该仅移动租约持有共享不可变存储，可晚于客户端析构。其 span 在析构、替换或被移走前
 * 有效。默认构造或被移走的租约为空，访问器返回零值或空值。
 */
class ObservationLease {
 public:
  /** @brief Private immutable frame storage. 私有不可变帧存储。 */
  struct Data;
  /** @brief Creates an empty lease. 创建空租约。 */
  ObservationLease();
  /** @brief Releases the owned frame, if any. 释放持有的帧（如有）。 */
  ~ObservationLease();
  /** @brief Copying is disabled because transfer is explicit. 禁止复制，所有权须显式转移。 */
  ObservationLease(const ObservationLease&) = delete;
  /** @brief Copy assignment is disabled. 禁止复制赋值。 */
  ObservationLease& operator=(const ObservationLease&) = delete;
  /** @brief Transfers the frame and leaves the source empty. 转移帧并使源租约为空。 */
  ObservationLease(ObservationLease&&) noexcept;
  /** @brief Replaces this frame by transfer and leaves the source empty.
   * 通过转移替换当前帧并使源租约为空。 */
  ObservationLease& operator=(ObservationLease&&) noexcept;

  /** @brief Returns the session-local frame sequence, or zero when empty.
   *         返回会话内帧序列号；租约为空时返回零。
   * @return Monotonic message sequence. 单调递增消息序列号。
   */
  std::uint64_t sequence() const noexcept;
  /** @brief Returns the completed-step tick, or zero when empty.
   *         返回已完成仿真步 tick；租约为空时返回零。
   * @return Server tick identifier, distinct from sequence. 服务器 tick 标识符，与序列号不同。
   */
  std::uint64_t tick_id() const noexcept;
  /** @brief Returns completed-step simulation time in seconds, or zero when empty.
   *         返回已完成仿真步的仿真时间（秒）；租约为空时返回零。
   * @return Simulation time in seconds. 仿真时间，单位为秒。
   */
  double simulation_time_s() const noexcept;
  /** @brief Borrows dense observations in registered layout order.
   *         借用按注册布局顺序排列的稠密观测值。
   * @return Span valid only while this non-moved lease retains its frame.
   *         仅在本租约未被移走且仍持有帧期间有效的 span。
   */
  std::span<const double> observations() const noexcept;
  /** @brief Reports whether this lease owns a frame. 返回本租约是否持有帧。
   * @return true for a received frame; otherwise false. 持有已接收帧时为 true，否则为 false。
   */
  explicit operator bool() const noexcept;

 private:
  /// Shared immutable storage that may outlive the client. 可晚于客户端析构的共享不可变存储。
  std::shared_ptr<const Data> data_;
  /// Grants the client permission to commit a received frame. 允许客户端提交已接收帧。
  friend class ObserverClient;
};

/**
 * @brief Single-session read-only Protocol V4 Observer client.
 *        单会话只读 Protocol V4 Observer 客户端。
 *
 * The client is single-use. Lifecycle calls are serialized without holding a
 * project lock across RPC, callback teardown, or thread joins. At most one
 * observation wait may run, while Stop() may concurrently cancel it. Receive
 * callbacks execute on eCAL threads and never expose borrowed payloads.
 * 客户端仅可使用一次。生命周期调用会串行化，执行 RPC、回调拆除或线程等待时不持有
 * 项目锁。最多只能有一个观测等待进行，Stop() 可并发取消该等待。接收回调在 eCAL
 * 线程执行，且不会向外暴露借用 payload。
 */
class ObserverClient {
 public:
  /** @brief Creates a non-started client and takes ownership of its options.
   *         创建未启动客户端并取得配置所有权。
   * @param options Registration, cadence, and transport options validated by Start().
   *                由 Start() 校验的注册、节拍与传输配置。
   */
  explicit ObserverClient(ObserverOptions options);
  /**
   * @brief Cancels local work, joins heartbeat, drains callbacks, and attempts
   *        best-effort remote cleanup.
   *        取消本地工作、等待心跳退出、清空回调并尝试尽力清理远端会话。
   * @par Blocking / 阻塞行为
   * May wait for lifecycle work, heartbeat/callback completion, and bounded RPC.
   * 可能等待生命周期工作、心跳/回调完成及有界 RPC。
   */
  ~ObserverClient();

  /** @brief Clients are neither copyable nor movable. 客户端不可复制或移动。 */
  ObserverClient(const ObserverClient&) = delete;
  /** @brief Clients are neither copyable nor movable. 客户端不可复制或移动。 */
  ObserverClient& operator=(const ObserverClient&) = delete;

  /**
   * @brief Registers a provisional Observer and starts reception and heartbeat.
   *        注册临时 Observer 并启动接收与心跳。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需诊断输出；有效入口会先清空，成功时保持为空。
   * @return true after registration and local infrastructure start; otherwise false.
   *         注册及本地基础设施启动完成时返回 true，否则返回 false。
   * @pre This single-use client has not previously started or stopped.
   *      此单次使用客户端此前未启动或停止。
   * @par Blocking / 阻塞行为
   * Performs bounded discovery/RPC and endpoint/thread creation. 执行有界发现/RPC 及端点/线程创建。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Activates the provisional Observer session.
   *        激活临时 Observer 会话。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when remote activation commits locally; otherwise false.
   *         远端激活成功在本地提交时返回 true，否则返回 false。
   * @pre Start() succeeded and the client is neither active nor stopped.
   *      Start() 已成功，且客户端尚未激活或停止。
   * @post Failed local commit triggers compensating unregister; failed cleanup
   *       preserves the session for Unregister() retry. 本地提交失败会触发补偿注销；
   *       清理失败会保留会话供 Unregister() 重试。
   */
  bool Activate(std::string* error_message);

  /**
   * @brief Waits for one queued Observation frame.
   *        等待一帧排队的 Observation 数据。
   * @param timeout Non-negative wall-clock duration; `milliseconds::max()` waits
   *        until frame, cancellation, Server stop, or error. 非负墙钟等待时长；
   *        `milliseconds::max()` 会等待至帧、取消、服务器停止或错误发生。
   * @param frame Required output, modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required diagnostic output; cleared on valid entry.
   *                      必需诊断输出；有效入口会先清空。
   * @return Wait outcome; kReceived, kTimeout, and kStopped leave the diagnostic empty.
   *         等待结果；kReceived、kTimeout 与 kStopped 均保持诊断为空。
   * @pre Activate() succeeded, unless a terminal status is already pending.
   *      Activate() 已成功，除非已有待报告的终止状态。
   * @par Thread safety / 线程安全
   * At most one waiter is allowed; Stop() may cancel it concurrently.
   * 最多允许一个等待方；Stop() 可并发取消等待。
   */
  ObservationWaitResult WaitForObservation(std::chrono::milliseconds timeout,
                                           ObservationLease* frame, std::string* error_message);

  /**
   * @brief Permanently cancels local waits and heartbeat without remote cleanup.
   *        永久取消本地等待与心跳，但不清理远端会话。
   *
   * The call is idempotent, non-blocking apart from short critical sections,
   * and may run concurrently with a wait or lifecycle operation.
   * 本调用幂等，除短临界区外不阻塞，可与等待或生命周期操作并发执行。
   */
  void Stop();

  /**
   * @brief Removes the remote session and completes local stop state.
   *        删除远端会话并完成本地停止状态。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when no remote session remains; false when cleanup may be retried.
   *         远端不再存在会话时返回 true；清理可重试时返回 false。
   * @post Failed transport/RPC cleanup preserves session identity for retry.
   *       传输/RPC 清理失败会保留会话标识以供重试。
   * @par Blocking / 阻塞行为
   * Performs at most one bounded cleanup RPC. 最多执行一次有界清理 RPC。
   */
  bool Unregister(std::string* error_message);

  /** @brief Returns sequence gaps plus local bounded-queue evictions.
   *         返回序列缺口与本地有界队列淘汰数量之和。
   * @return Accumulated dropped frame count for the current session. 当前会话累计丢帧数。
   * @par Thread safety / 线程安全
   * Safe during reception. 可在接收期间安全调用。
   */
  std::uint64_t dropped_frames() const;
  /** @brief Returns accumulated non-terminal warnings or the sticky terminal error.
   *         返回累计的非终止警告或粘性终止错误。
   * @return Owned diagnostic snapshot; empty when none has occurred. 自有诊断快照；无诊断时为空。
   * @par Thread safety / 线程安全
   * Safe during heartbeat and reception. 可在心跳与接收期间安全调用。
   */
  std::string lifecycle_warning() const;

  /** @brief Returns the last successful registration projection.
   *         返回最近一次成功注册的公开投影。
   * @return Reference valid until client destruction. 在客户端析构前有效的引用。
   * @pre Start() completed successfully; do not race this access with Start().
   *      Start() 已成功完成；不得与 Start() 并发访问。
   */
  const AdapterRegistration& registration() const;
  /** @brief Returns the immutable dense observation layout.
   *         返回不可变稠密观测布局。
   * @return Reference valid until client destruction. 在客户端析构前有效的引用。
   * @pre Start() completed successfully; do not race this access with Start().
   *      Start() 已成功完成；不得与 Start() 并发访问。
   */
  const ControllerLayout& layout() const;

 private:
  /// Grants focused tests access to lifecycle and receive hooks.
  /// 允许定向测试访问生命周期与接收钩子。
  friend class ObserverClientTestPeer;
  /** @brief Private implementation type; not part of the installed ABI surface.
   *         私有实现类型；不属于已安装 ABI 接口。
   */
  class Impl;
  /// Shared implementation retained by admitted callbacks during teardown.
  /// 拆除期间由已准入回调共同持有的实现。
  std::shared_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::adapter_sdk
