#pragma once

#include <cstdint>

namespace mujoco_sim::adapter_sdk::internal {

/** @brief Local Adapter lifecycle phases. 本地 Adapter 生命周期阶段。 */
enum class ClientPhase {
  /// No remote registration has been attempted. 尚未尝试远端注册。
  kConstructed,
  /// Registration succeeded and local startup is being committed. 注册成功，正在提交本地启动。
  kStarting,
  /// Local startup completed; remote activation is still pending. 本地启动完成，仍待远端激活。
  kProvisional,
  /// Activation RPC is in flight. 激活 RPC 正在执行。
  kActivating,
  /// Remote and local activation both committed. 远端与本地激活均已提交。
  kActive,
  /// Local cancellation or successful cleanup is terminal. 本地取消或成功清理形成终止状态。
  kStopped,
  /// A sticky lifecycle or remote error is terminal. 粘性生命周期或远端错误形成终止状态。
  kTerminalError,
};

/** @brief Identifies one remote Adapter session and model epoch.
 *         标识一个远端 Adapter 会话与模型代次。
 */
struct SessionIdentity {
  /// Nonzero Server-assigned session ID after registration. 注册后由服务器分配的非零会话 ID。
  std::uint64_t session_id = 0;
  /// Nonzero model epoch that owns the session layout. 持有会话布局的非零模型 epoch。
  std::uint64_t epoch = 0;
};

/**
 * @brief Maintains local lifecycle phase separately from remote-session ownership.
 *        分别维护本地生命周期阶段与远端会话所有权。
 *
 * Callers serialize every access with their client state mutex. Remote session
 * ownership intentionally survives Stop() and Fail(), so Unregister remains
 * retryable after local or terminal outcomes. No member performs I/O or blocks.
 * 调用方必须使用客户端状态互斥锁串行化所有访问。远端会话所有权会有意跨越 Stop() 与
 * Fail() 保留，使本地停止或终止错误后仍可重试 Unregister。所有成员均不执行 I/O 或阻塞。
 *
 * @par Thread safety / 线程安全
 * Not internally synchronized; caller holds the owning client state mutex.
 * 内部不提供同步；调用方须持有所属客户端状态互斥锁。
 */
class AdapterSession {
 public:
  /** @brief Returns the current local phase. 返回当前本地阶段。
   * @return Current phase. 当前阶段。
   */
  ClientPhase phase() const { return phase_; }
  /** @brief Reports whether local startup has passed registration.
   *         返回本地启动是否已通过注册阶段。
   * @return true in provisional, activating, or active phase. 在临时、激活中或活动阶段为 true。
   */
  bool started() const {
    return phase_ == ClientPhase::kProvisional || phase_ == ClientPhase::kActivating ||
           phase_ == ClientPhase::kActive;
  }
  /** @brief Reports whether activation committed. 返回激活是否已提交。 */
  bool active() const { return phase_ == ClientPhase::kActive; }
  /** @brief Reports whether receive callbacks may accept session data.
   *         返回接收回调是否可接受会话数据。
   * @return true during activation or after commit. 激活期间或提交后为 true。
   */
  bool accepting_data() const { return phase_ == ClientPhase::kActivating || active(); }
  /** @brief Reports whether remote cleanup is still owned. 返回是否仍持有远端清理责任。 */
  bool has_session() const { return has_session_; }
  /** @brief Borrows the current identity. 借用当前会话标识。
   * @return Reference valid while the owning session object exists and caller lock is held.
   *         所属会话对象存在且调用方持锁期间有效的引用。
   * @pre has_session() is true. has_session() 为 true。
   */
  const SessionIdentity& identity() const { return identity_; }
  /** @brief Reports a normal terminal local stop. 返回是否为正常本地终止。 */
  bool stopped() const { return phase_ == ClientPhase::kStopped; }
  /** @brief Reports a sticky terminal error. 返回是否为粘性终止错误。 */
  bool terminal_error() const { return phase_ == ClientPhase::kTerminalError; }

  /**
   * @brief Records remote registration and enters local startup.
   *        记录远端注册并进入本地启动阶段。
   * @param identity Nonzero remote identity owned until ReleaseSession().
   *                 在 ReleaseSession() 前持有的非零远端标识。
   * @return false if an existing local stop/error prevents startup; true otherwise.
   *         既有本地停止/错误阻止启动时返回 false，否则返回 true。
   * @post Remote ownership is retained even when the return value is false.
   *       即使返回 false，仍保留远端所有权以便补偿清理。
   */
  bool Registered(SessionIdentity identity) {
    identity_ = identity;
    has_session_ = true;
    if (phase_ == ClientPhase::kStopped || terminal_error()) return false;
    phase_ = ClientPhase::kStarting;
    return true;
  }
  /** @brief Commits local startup and enters provisional phase.
   *         提交本地启动并进入临时阶段。
   * @return true only from the starting phase. 仅从启动中阶段转移时返回 true。
   */
  bool StartCompleted() {
    if (phase_ != ClientPhase::kStarting) return false;
    phase_ = ClientPhase::kProvisional;
    return true;
  }
  /** @brief Reserves the provisional session for activation.
   *         预留临时会话以执行激活。
   * @return true only for an owned provisional session. 仅持有临时会话时返回 true。
   */
  bool BeginActivation() {
    if (phase_ != ClientPhase::kProvisional || !has_session()) return false;
    phase_ = ClientPhase::kActivating;
    return true;
  }
  /** @brief Rolls an unsuccessful activation back to provisional.
   *         将未成功的激活回退到临时阶段。
   * @return true only while activation is in flight. 仅激活正在执行时返回 true。
   */
  bool ActivationRejected() {
    if (phase_ != ClientPhase::kActivating) return false;
    phase_ = ClientPhase::kProvisional;
    return true;
  }
  /** @brief Commits activation. 提交激活。
   * @return true only while activation is in flight. 仅激活正在执行时返回 true。
   */
  bool Activated() {
    if (phase_ != ClientPhase::kActivating) return false;
    phase_ = ClientPhase::kActive;
    return true;
  }
  /** @brief Enters normal stopped phase unless a terminal error is already sticky.
   *         进入正常停止阶段；若已有粘性终止错误则保留该错误阶段。
   */
  void Stop() {
    if (!terminal_error()) phase_ = ClientPhase::kStopped;
  }
  /** @brief Makes the terminal-error phase sticky. 将终止错误阶段设为粘性状态。 */
  void Fail() { phase_ = ClientPhase::kTerminalError; }
  /** @brief Releases remote cleanup ownership and records a normal stop.
   *         释放远端清理所有权并记录正常停止。
   * @post has_session() is false. has_session() 为 false。
   */
  void ReleaseSession() {
    has_session_ = false;
    Stop();
  }

 private:
  /// Local lifecycle phase, protected by the owning client mutex.
  /// 本地生命周期阶段，由所属客户端互斥锁保护。
  ClientPhase phase_ = ClientPhase::kConstructed;
  /// Last registered identity, meaningful while has_session_ is true.
  /// 最近注册的标识，仅在 has_session_ 为 true 时有意义。
  SessionIdentity identity_;
  /// Remote cleanup ownership, independent of terminal local phase.
  /// 远端清理所有权，与本地终止阶段相互独立。
  bool has_session_ = false;
};

}  // namespace mujoco_sim::adapter_sdk::internal
