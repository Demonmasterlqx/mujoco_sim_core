#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Configures Controller registration, resources, and RPC discovery.
 *        配置控制器注册、资源与 RPC 发现行为。
 */
struct ControllerOptions {
  /// Server identifier used to derive endpoints; defaults to `"default"` and
  /// must satisfy the Protocol V4 Server identifier rules.
  /// 用于派生端点的服务器标识符；默认为 `"default"`，且必须符合 Protocol V4
  /// 服务器标识符规则。
  std::string server_name = "default";
  /// Non-empty Controller identifier, unique within the selected Server and
  /// valid under the Protocol V4 Adapter identifier rules.
  /// 非空控制器标识符；在所选服务器内唯一，并符合 Protocol V4 Adapter 标识符规则。
  std::string adapter_id;
  /// Observation resource names requested in dense registration order; an
  /// empty list is permitted and each name must identify a Server resource.
  /// 按稠密注册顺序请求的观测资源名称；允许为空，每个名称必须标识服务器资源。
  std::vector<std::string> observation_names;
  /// Command resource names requested in dense registration order; at least
  /// one command is required and each name must identify a Server actuator.
  /// 按稠密注册顺序请求的指令资源名称；至少需要一个指令，每个名称必须标识服务器执行器。
  std::vector<std::string> command_names;
  /// Positive discovery and RPC timeout in milliseconds; defaults to 5000 ms.
  /// 正数的发现与 RPC 超时时间，单位为毫秒；默认为 5000 ms。
  std::uint32_t service_timeout_ms = 5000;
};

/**
 * @brief Owns one immutable Controller State frame.
 *        持有一帧不可变的控制器 State 数据。
 *
 * A lease is move-only. Its scalar span and metadata remain valid until the
 * lease is destroyed, reset by move assignment, or moved from. A moved-from or
 * default-constructed lease is empty and returns zero/empty accessor values.
 * The owned buffer may be retained across client calls and threads.
 *
 * 租约只能移动。其标量 span 与元数据在租约析构、被移动赋值重置或被移走前保持有效。
 * 被移走或默认构造的租约为空，访问器返回零值或空值。该自有缓冲区可跨客户端调用和线程保留。
 */
class StateLease {
 public:
  /** @brief Private immutable frame storage. 私有不可变帧存储。 */
  struct Data;

  /** @brief Creates an empty lease. 创建空租约。 */
  StateLease();
  /** @brief Releases the owned frame, if any. 释放持有的帧（如有）。 */
  ~StateLease();
  /** @brief Copying is disabled because lease transfer is explicit. 禁止复制，租约须显式转移。 */
  StateLease(const StateLease&) = delete;
  /** @brief Copy assignment is disabled. 禁止复制赋值。 */
  StateLease& operator=(const StateLease&) = delete;
  /** @brief Transfers frame ownership and leaves the source empty. 转移帧所有权并使源租约为空。 */
  StateLease(StateLease&&) noexcept;
  /** @brief Replaces this frame by transfer and leaves the source empty.
   * 通过转移替换当前帧并使源租约为空。 */
  StateLease& operator=(StateLease&&) noexcept;

  /** @brief Returns the Protocol V4 State message sequence, or zero when empty.
   *         返回 Protocol V4 State 消息序列号；租约为空时返回零。
   * @return Monotonic session-local sequence. 会话内单调递增序列号。
   */
  std::uint64_t sequence() const noexcept;
  /** @brief Returns the completed simulation-step tick, or zero when empty.
   *         返回已完成仿真步的 tick；租约为空时返回零。
   * @return Server tick identifier, distinct from the message sequence.
   *         服务器 tick 标识符，与消息序列号不同。
   */
  std::uint64_t tick_id() const noexcept;
  /** @brief Returns completed-step simulation time in seconds, or zero when empty.
   *         返回已完成仿真步的仿真时间（秒）；租约为空时返回零。
   * @return Simulation time in seconds. 仿真时间，单位为秒。
   */
  double simulation_time_s() const noexcept;
  /** @brief Borrows the dense observation scalars in registered layout order.
   *         借用按注册布局顺序排列的稠密观测标量。
   * @return Span valid only while this non-moved lease retains its frame.
   *         仅在本租约未被移走且仍持有该帧期间有效的 span。
   */
  std::span<const double> observations() const noexcept;
  /** @brief Reports whether this lease owns a frame. 返回本租约是否持有帧。
   * @return true for a received frame; otherwise false. 持有已接收帧时为 true，否则为 false。
   */
  explicit operator bool() const noexcept;

 private:
  /// Shared immutable storage; retained leases may safely outlive the client.
  /// 共享不可变存储；被保留的租约可安全地晚于客户端析构。
  std::shared_ptr<const Data> data_;
  /// Grants the client permission to commit a received frame. 允许客户端提交已接收帧。
  friend class ControllerClient;
};

/**
 * @brief Result of polling or waiting for Controller State.
 *        轮询或等待控制器 State 的结果。
 */
enum class StateReadResult {
  /// A State lease was written to the output. 已向输出写入 State 租约。
  kReceived,
  /// No eligible State arrived before the deadline; output is unchanged.
  /// 截止期限前没有符合条件的 State；输出保持不变。
  kTimeout,
  /// Local cancellation, successful cleanup, or eCAL shutdown ended the read;
  /// output is unchanged. 本地取消、成功清理或 eCAL 关闭终止了读取；输出保持不变。
  kStopped,
  /// Invalid use or a sticky lifecycle/transport error ended the read; output
  /// is unchanged and the diagnostic is non-empty. 无效调用或粘性生命周期/传输错误
  /// 终止了读取；输出保持不变，且诊断非空。
  kError,
};

/**
 * @brief Single-session asynchronous Protocol V4 Controller client.
 *        单会话异步 Protocol V4 控制器客户端。
 *
 * The client is single-use: Start() and Activate() each succeed at most once,
 * and Stop() or successful Unregister() permanently prevents restart. One
 * thread may read State while one control thread calls SetCommand(); Stop() may
 * run concurrently to cancel both. Lifecycle operations are serialized and do
 * not retain project locks while performing RPC, callback teardown, or thread
 * joins. Concurrent State reads and concurrent SetCommand() calls are invalid.
 *
 * 客户端仅可使用一次：Start() 与 Activate() 各最多成功一次，Stop() 或成功的
 * Unregister() 会永久禁止重启。一个线程可读取 State，同时一个控制线程调用
 * SetCommand()；Stop() 可并发执行以取消两者。生命周期操作会串行化，执行 RPC、回调
 * 拆除或线程等待时不持有项目锁。不允许并发读取 State，也不允许并发调用 SetCommand()。
 */
class ControllerClient {
 public:
  /**
   * @brief Creates a non-started client and takes ownership of its options.
   *        创建未启动客户端并取得配置所有权。
   * @param options Registration and transport options validated by Start().
   *                由 Start() 校验的注册与传输配置。
   */
  explicit ControllerClient(ControllerOptions options);
  /**
   * @brief Cancels local work, drains callbacks and the publisher, then attempts
   *        best-effort remote cleanup.
   *        取消本地工作、清空回调并等待发布线程后，尝试尽力清理远端会话。
   * @par Blocking / 阻塞行为
   * May wait for an in-flight lifecycle operation, callback, publisher exit,
   * and bounded cleanup RPC. 可能等待正在执行的生命周期操作、回调、发布线程退出及有界清理 RPC。
   */
  ~ControllerClient();

  /** @brief Clients are neither copyable nor movable. 客户端不可复制或移动。 */
  ControllerClient(const ControllerClient&) = delete;
  /** @brief Clients are neither copyable nor movable. 客户端不可复制或移动。 */
  ControllerClient& operator=(const ControllerClient&) = delete;

  /**
   * @brief Registers a provisional Controller and starts State/status reception.
   *        注册临时控制器并启动 State/status 接收。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需的诊断输出；有效入口会先清空，成功时保持为空。
   * @return true on complete local and remote startup; otherwise false with a
   *         contextual diagnostic. 本地与远端启动均完成时返回 true，否则返回 false
   * 并给出上下文诊断。
   * @pre This single-use client has not previously started or stopped.
   *      此单次使用客户端此前未启动或停止。
   * @par Blocking / 阻塞行为
   * Performs bounded discovery/RPC and endpoint creation. 执行有界发现/RPC 与端点创建。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Activates the provisional session and starts Command publication.
   *        激活临时会话并启动 Command 发布。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需的诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when remote activation and local publisher startup both commit;
   *         otherwise false. 远端激活与本地发布线程启动均提交时返回 true，否则返回 false。
   * @pre Start() succeeded and the client is neither active nor stopped.
   *      Start() 已成功，且客户端尚未激活或停止。
   * @post A locally uncommittable remote activation triggers compensating
   *       unregister; failed cleanup remains retryable through Unregister().
   *       无法在本地提交的远端激活会触发补偿注销；清理失败仍可通过 Unregister() 重试。
   * @par Blocking / 阻塞行为
   * Performs a bounded RPC and may drain local resources during compensation.
   * 执行有界 RPC，补偿期间可能等待本地资源退出。
   */
  bool Activate(std::string* error_message);

  /**
   * @brief Polls the latest accepted State without waiting for a newer sequence.
   *        轮询最新已接收 State，不等待更新序列。
   * @param state Required output, modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required diagnostic output; cleared on valid entry.
   *                      必需的诊断输出；有效入口会先清空。
   * @return Read outcome; kTimeout, kStopped, and kReceived leave the diagnostic empty.
   *         读取结果；kTimeout、kStopped 与 kReceived 均保持诊断为空。
   * @par Blocking / 阻塞行为
   * Non-blocking apart from a short state mutex critical section. 除短暂状态锁临界区外不阻塞。
   */
  StateReadResult ReadLatestState(StateLease* state, std::string* error_message);

  /**
   * @brief Waits for a State sequence newer than the last successful read.
   *        等待比上次成功读取更新的 State 序列。
   * @param timeout Non-negative wall-clock duration; `milliseconds::max()` waits
   *        without a deadline until data, cancellation, or error. 非负墙钟等待时长；
   *        `milliseconds::max()` 会无限等待，直至数据、取消或错误发生。
   * @param state Required output, modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required diagnostic output; cleared on valid entry.
   *                      必需的诊断输出；有效入口会先清空。
   * @return Read outcome; kTimeout, kStopped, and kReceived leave the diagnostic empty.
   *         读取结果；kTimeout、kStopped 与 kReceived 均保持诊断为空。
   * @par Thread safety / 线程安全
   * At most one State read may be in progress; Stop() may cancel it concurrently.
   * 最多只能有一个 State 读取正在进行；Stop() 可并发取消该读取。
   */
  StateReadResult WaitForNewState(std::chrono::milliseconds timeout, StateLease* state,
                                  std::string* error_message);

  /**
   * @brief Replaces the depth-one latest Command mailbox.
   *        替换深度为一的最新 Command 邮箱。
   * @param values Dense command scalars in layout order, with exact scalar
   *        count, finite values, and inclusive registered ranges. 按布局顺序排列的稠密
   *        指令标量；数量必须精确、值必须有限且位于注册的闭区间内。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需的诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when the local mailbox accepts the command; otherwise false.
   *         本地邮箱接受指令时返回 true，否则返回 false。
   * @pre Activate() succeeded. Activate() 已成功。
   * @par Blocking / 阻塞行为
   * Does not wait for transport publication or a simulation step. 不等待传输发布或仿真步完成。
   */
  bool SetCommand(std::span<const double> values, std::string* error_message);

  /**
   * @brief Publishes one terminal Controller fault for the active session.
   *        为活动会话发布一个终止性控制器故障。
   * @param code Supported Protocol V4 fault category. 支持的 Protocol V4 故障类别。
   * @param message Diagnostic text of at most 4096 bytes, borrowed for this call.
   *                最多 4096 字节的诊断文本，仅在本次调用期间借用。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需的诊断输出；有效入口会先清空，成功时保持为空。
   * @return true only when eCAL accepts the serialized fault; otherwise false.
   *         仅当 eCAL 接受序列化故障时返回 true，否则返回 false。
   * @par Blocking / 阻塞行为
   * Serializes with lifecycle operations but does not wait for Server handling.
   * 与生命周期操作串行化，但不等待服务器处理。
   */
  bool ReportFault(FaultCode code, std::string_view message, std::string* error_message);

  /**
   * @brief Permanently cancels local reads and publication without remote cleanup.
   *        永久取消本地读取和发布，但不清理远端会话。
   *
   * This call is idempotent and may run concurrently with reads or lifecycle
   * work. It publishes cancellation before waiting for lifecycle admission.
   * 本调用幂等，可与读取或生命周期工作并发执行；等待生命周期准入前会先发布取消状态。
   */
  void Stop();

  /**
   * @brief Removes the remote session and completes local shutdown.
   *        删除远端会话并完成本地关闭。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需的诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when no remote session remains; false when cleanup may be retried.
   *         远端不再存在会话时返回 true；清理可重试时返回 false。
   * @post Failed transport/RPC cleanup preserves session identity for a later retry.
   *       传输/RPC 清理失败会保留会话标识以供后续重试。
   * @par Blocking / 阻塞行为
   * May perform a bounded RPC, drain callbacks, and join the publisher thread.
   * 可能执行有界 RPC、清空回调并等待发布线程退出。
   */
  bool Unregister(std::string* error_message);

  /**
   * @brief Returns the last successful registration projection.
   *        返回最近一次成功注册的公开投影。
   * @return Reference valid until client destruction. 在客户端析构前有效的引用。
   * @pre Start() completed successfully; do not race this access with Start().
   *      Start() 已成功完成；不得与 Start() 并发访问。
   */
  const AdapterRegistration& registration() const;
  /**
   * @brief Returns the immutable dense layout from registration.
   *        返回注册得到的不可变稠密布局。
   * @return Reference valid until client destruction. 在客户端析构前有效的引用。
   * @pre Start() completed successfully; do not race this access with Start().
   *      Start() 已成功完成；不得与 Start() 并发访问。
   */
  const ControllerLayout& layout() const;

 private:
  /** @brief Private implementation type; not part of the installed ABI surface.
   *         私有实现类型；不属于已安装 ABI 接口。
   */
  class Impl;
  /// Shared implementation retained by admitted callbacks during teardown.
  /// 拆除期间由已准入回调共同持有的实现。
  std::shared_ptr<Impl> impl_;
  /// Grants focused tests access to internal lifecycle hooks. 允许定向测试访问内部生命周期钩子。
  friend class ControllerClientTestPeer;
};

}  // namespace mujoco_sim::adapter_sdk
