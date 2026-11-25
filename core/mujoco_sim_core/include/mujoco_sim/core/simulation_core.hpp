#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <mujoco_sim/core/domain.hpp>
#include <mujoco_sim/core/interactive_input.hpp>

struct mjData_;
struct mjModel_;

namespace mujoco_sim::core {

namespace detail {
class SnapshotManager;
}

struct CoreOptions {
  struct ControllerOverride {
    std::string adapter_id;
    std::uint32_t command_timeout_ms = 500;
    std::uint32_t command_keepalive_ms = 100;
    std::uint32_t offline_session_expire_ms = 0;
    ObservationDelivery state_delivery;
    ControlPolicy startup_policy = ControlPolicy::kZero;
    ControlPolicy offline_policy = ControlPolicy::kHoldLast;
    ControlPolicy error_policy = ControlPolicy::kZero;
    ControlPolicy release_policy = ControlPolicy::kZero;
  };
  std::string server_name = "default";
  std::uint32_t command_timeout_ms = 500;
  std::uint32_t command_keepalive_ms = 100;
  std::uint32_t observer_session_timeout_ms = 1000;
  std::uint32_t activation_timeout_ms = 5000;
  std::uint32_t offline_session_expire_ms = 0;
  ObservationDelivery controller_state_delivery = [] {
    ObservationDelivery d;
    d.set_every_n_steps(1);
    return d;
  }();
  double controller_state_max_rate_hz = 1000.0;
  ControlPolicy startup_policy = ControlPolicy::kZero;
  ControlPolicy offline_policy = ControlPolicy::kHoldLast;
  ControlPolicy error_policy = ControlPolicy::kZero;
  ControlPolicy release_policy = ControlPolicy::kZero;
  std::vector<ControllerOverride> controller_overrides;
};
struct ModelLoadOptions {
  std::string initial_keyframe;
};
/**
 * @brief Identifies one owned lifecycle effect emitted by SimulationCore.
 *
 * SimulationLoop consumes these values on the Core owner thread to update
 * transport resources, structured logs, and lifecycle status. Values do not
 * borrow Core storage. `kSimulationReset` is emitted only for an ONLINE
 * Controller that Reset returns to ACTIVE_NO_COMMAND.
 *
 * 标识 SimulationCore 产生的一项自有生命周期效果。
 *
 * SimulationLoop 在 Core 所有者线程上消费这些值，用于更新传输资源、结构化日志与
 * 生命周期状态。值不借用 Core 存储。`kSimulationReset` 仅在 Reset 将 ONLINE
 * Controller 退回 ACTIVE_NO_COMMAND 时产生。
 */
enum class AdapterChangeKind {
  kRegistered,
  kActivated,
  kUnregistered,
  kActivationExpired,
  kTimedOut,
  kRecovered,
  kLocalError,
  kServerFailed,
  kReplaced,
  /** An ONLINE Controller was returned to command-waiting by simulation Reset.
   * 仿真 Reset 使 ONLINE Controller 返回等待 command 的状态。 */
  kSimulationReset
};
struct AdapterChange {
  AdapterChangeKind kind = AdapterChangeKind::kRegistered;
  std::string adapter_id;
  AdapterRole role = ADAPTER_ROLE_UNSPECIFIED;
  std::uint64_t session_id = 0;
  std::optional<std::uint64_t> previous_session_id;
  AdapterState old_state = ADAPTER_STATE_UNSPECIFIED, new_state = ADAPTER_STATE_UNSPECIFIED;
  std::string message;
  std::uint64_t tick_id = 0;
};
struct StepCompletedEvent {
  std::uint64_t completed_tick_id = 0, next_tick_id = 0;
  double simulation_time_s = 0;
  std::vector<ControllerStateFrame> controller_states;
  std::vector<ObservationFrame> observer_frames;
};
struct EngineEffects {
  std::vector<AdapterChange> adapter_changes;
  std::optional<StepCompletedEvent> step_completed;
  enum class StatusUrgency {
    kNone,
    kPeriodicOnly,
    kLifecycle,
    kTerminal
  };
  StatusUrgency status_urgency = StatusUrgency::kNone;
};
template <class Response>
struct EngineResult {
  Response response;
  EngineEffects effects;
};
struct ModelLoadReceipt {
  bool loaded = false;
  std::string message;
};
struct CommandReceipt {
  ErrorCode code = OK;
  std::string message;
};
/**
 * @brief Reports whether a local simulation lifecycle action was applied.
 *
 * `applied` is true for accepted idempotent requests. `state_changed` only
 * reports a ServerState transition; Reset can therefore be applied without a
 * state change. The message owns a diagnostic suitable for structured logs.
 *
 * 报告本地仿真生命周期动作是否已应用。
 *
 * 对已接受的幂等请求，`applied` 也为 true。`state_changed` 只表示 ServerState
 * 转换，因此 Reset 可以成功但不改变状态。message 自有可用于结构化日志的诊断文本。
 */
struct SimulationActionReceipt {
  /** Whether the request was accepted and applied. 请求是否被接受并应用。 */
  bool applied = false;
  /** Whether the action changed ServerState. 动作是否改变了 ServerState。 */
  bool state_changed = false;
  /** Owned action result diagnostic. 自有的动作结果诊断。 */
  std::string message;
};
struct CoreResources {
  std::span<const ResourceDescriptor> observations, commands;
};
struct CoreState {
  std::uint64_t epoch = 0, current_tick_id = 0;
  double simulation_time_s = 0, timestep_s = 0;
  ServerState server_state = SERVER_STATE_UNSPECIFIED;
  std::size_t active_controller_count = 0;
  std::array<std::size_t, 7> controller_counts{}, observer_counts{};
  std::size_t controller_count(AdapterState s) const noexcept {
    return controller_counts[static_cast<std::size_t>(s)];
  }
  std::size_t observer_count(AdapterState s) const noexcept {
    return observer_counts[static_cast<std::size_t>(s)];
  }
};

class SimulationSnapshot {
 public:
  ~SimulationSnapshot();
  SimulationSnapshot(const SimulationSnapshot&) = delete;
  SimulationSnapshot& operator=(const SimulationSnapshot&) = delete;
  SimulationSnapshot(SimulationSnapshot&&) noexcept;
  SimulationSnapshot& operator=(SimulationSnapshot&&) noexcept;
  const mjModel_* model() const;
  mjData_* data() const;

 private:
  struct Impl;
  SimulationSnapshot();
  std::unique_ptr<Impl> impl_;
  friend class SimulationCore;
  friend class detail::SnapshotManager;
};

class SimulationCore {
 public:
  explicit SimulationCore(CoreOptions options = {});
  ~SimulationCore();
  SimulationCore(const SimulationCore&) = delete;
  SimulationCore& operator=(const SimulationCore&) = delete;
  SimulationCore(SimulationCore&&) noexcept;
  SimulationCore& operator=(SimulationCore&&) noexcept;
  EngineResult<ModelLoadReceipt> LoadModel(const std::string&,
                                           const ModelLoadOptions& options = {});
  bool IsModelLoaded() const;
  std::unique_ptr<SimulationSnapshot> CreateSnapshot(std::string*) const;
  std::unique_ptr<SimulationSnapshot> CloneSnapshot(const SimulationSnapshot&, std::string*) const;
  bool UpdateSnapshot(SimulationSnapshot&, std::string*) const;
  CoreResources resources() const;
  GetAvailableResourcesResponse GetAvailableResources(const GetAvailableResourcesRequest&) const;
  EngineResult<RegisterAdapterResponse> RegisterAdapter(
      const RegisterAdapterRequest&,
      std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  EngineResult<ActivateAdapterResponse> ActivateAdapter(
      const ActivateAdapterRequest&,
      std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  EngineResult<UnregisterAdapterResponse> UnregisterAdapter(const UnregisterAdapterRequest&);
  EngineResult<CommandReceipt> SubmitCommand(const ControllerCommand&,
                                             std::chrono::steady_clock::time_point now);
  EngineResult<CommandReceipt> SubmitFault(const ControllerFault&);
  EngineEffects AcceptHeartbeat(const AdapterHeartbeat&, std::chrono::steady_clock::time_point now);
  /**
   * @brief Advances one physics step without new interactive input.
   * @return Effects and immutable outputs for the completed Step, or no
   *         completed event when the Core is not running.
   *         已完成 Step 的效果与不可变输出；Core 未运行时不包含完成事件。
   *
   * 在没有新交互输入的情况下推进一个物理步。
   */
  EngineEffects Step();

  /**
   * @brief Applies borrowed interactive input and advances one physics step.
   *
   * Pending Controller and policy writes are applied first, followed by the
   * one-shot actuator writes and optional complete wrench replacement. Invalid
   * interactive entries are ignored. This function neither waits for nor
   * changes any Adapter session.
   *
   * 应用借用的交互输入并推进一个物理步。
   *
   * 首先应用待处理的 Controller 与策略写入，然后应用一次性 actuator 写入以及可选的
   * 完整扰动力替换。非法交互条目会被忽略。本函数不会等待或改变任何 Adapter 会话。
   *
   * @param input Values borrowed only for this call. 仅在本次调用期间借用的输入值。
   * @return Effects and immutable outputs for the completed Step, or no
   *         completed event when the Core is not running.
   *         已完成 Step 的效果与不可变输出；Core 未运行时不包含完成事件。
   * @par Thread safety / 线程安全
   * Call only from the SimulationCore owner thread.
   * 仅可由 SimulationCore 所有者线程调用。
   */
  EngineEffects Step(const InteractiveStepInput& input);
  /**
   * @brief Enters or leaves the normal manual PAUSED state.
   *
   * Only RUNNING and PAUSED accept this action. Idempotent requests are
   * accepted without emitting lifecycle effects. ERROR_PAUSED, IDLE, STOPPED,
   * and an unloaded model reject the request.
   *
   * 进入或离开正常的手动 PAUSED 状态。
   *
   * 只有 RUNNING 和 PAUSED 接受本动作。幂等请求会被接受但不产生生命周期效果。
   * ERROR_PAUSED、IDLE、STOPPED 以及未加载模型时会拒绝请求。
   *
   * @param paused true to pause physics; false to resume physics.
   *               true 表示暂停物理步进，false 表示恢复物理步进。
   * @return Owned receipt and lifecycle status effects. 自有回执与生命周期状态效果。
   * @par Thread safety / 线程安全
   * Call only from the SimulationCore owner thread. 仅可由 SimulationCore 所有者线程调用。
   * @par Blocking / 阻塞
   * Does not block or execute a physics step. 不阻塞，也不执行物理步进。
   */
  EngineResult<SimulationActionReceipt> SetPaused(bool paused);

  /**
   * @brief Resets authoritative MuJoCo data to the configured initial state.
   *
   * Reset is accepted only in RUNNING or PAUSED. It preserves epoch, tick,
   * sessions, layouts, ownership, ServerState, and sequence counters. Cached
   * Controller commands and interactive inputs in Core are cleared. The
   * supplied steady-clock time restarts command liveness for Controllers that
   * are waiting for a post-reset command.
   *
   * 将权威 MuJoCo data 重置为配置的初始状态。
   *
   * Reset 仅在 RUNNING 或 PAUSED 中被接受。它保留 epoch、tick、session、layout、
   * 所有权、ServerState 与序列计数器，并清除 Core 中缓存的 Controller command 和交互
   * 输入。传入的 steady-clock 时间用于重新开始等待 Reset 后 command 的存活性计时。
   *
   * @param now Explicit steady-clock action time. 显式的 steady-clock 动作时间。
   * @return Owned receipt and lifecycle effects. 自有回执与生命周期效果。
   * @par Thread safety / 线程安全
   * Call only from the SimulationCore owner thread. 仅可由 SimulationCore 所有者线程调用。
   * @par Blocking / 阻塞
   * Performs only local MuJoCo reset/forward work and never waits for transport
   * or worker threads. 仅执行本地 MuJoCo reset/forward，不等待传输或 worker 线程。
   */
  EngineResult<SimulationActionReceipt> Reset(std::chrono::steady_clock::time_point now);
  EngineEffects CheckDeadlines(std::chrono::steady_clock::time_point now);
  EngineEffects MarkFailed(const std::string&);
  EngineEffects MarkStopped(const std::string&);
  ServerStatus MakeStatus() const;
  CoreState state() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::core
