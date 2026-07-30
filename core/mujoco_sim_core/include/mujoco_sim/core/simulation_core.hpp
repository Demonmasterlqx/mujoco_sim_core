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
enum class AdapterChangeKind {
  kRegistered,
  kActivated,
  kUnregistered,
  kActivationExpired,
  kTimedOut,
  kRecovered,
  kLocalError,
  kServerFailed,
  kReplaced
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
