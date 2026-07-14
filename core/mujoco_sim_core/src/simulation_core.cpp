#include "internal/simulation_core_impl.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace mujoco_sim::core {

namespace {

std::size_t CountForState(const std::array<std::size_t, 6>& counts,
                          mujoco_sim::v2::ControllerState state) noexcept {
  const int index = static_cast<int>(state);
  if (index < 0 || index >= static_cast<int>(counts.size())) {
    return 0;
  }
  return counts[static_cast<std::size_t>(index)];
}

void CountControllers(
    const std::vector<internal::ControllerRecord*>& controllers,
    CoreState* result) {
  for (const internal::ControllerRecord* controller : controllers) {
    const auto index =
        static_cast<std::size_t>(internal::WireState(controller->phase));
    ++result->controller_counts[index];
    if (controller->phase == internal::ControllerPhase::kActive) {
      ++result->active_controller_count;
    }
  }
}

void CountObservers(
    const std::vector<internal::ObserverRecord*>& observers,
    CoreState* result) {
  for (const internal::ObserverRecord* observer : observers) {
    const auto index =
        static_cast<std::size_t>(internal::WireState(observer->phase));
    ++result->observer_counts[index];
  }
}

}  // namespace

std::size_t CoreState::controller_count(
    mujoco_sim::v2::ControllerState state) const noexcept {
  return CountForState(controller_counts, state);
}

std::size_t CoreState::observer_count(
    mujoco_sim::v2::ControllerState state) const noexcept {
  return CountForState(observer_counts, state);
}
namespace {

using mujoco_sim::v2::ERROR_PAUSED;
using mujoco_sim::v2::OBSERVER;
using mujoco_sim::v2::PROVISIONAL;
using mujoco_sim::v2::TIMED_OUT;
using mujoco_sim::v2::UNREGISTERED;
using internal::ControllerPhase;
using internal::ControllerStateMachine;
using internal::ObserverPhase;
using internal::ObserverStateMachine;
using internal::WireState;

}  // namespace

EngineEffects SimulationCore::Impl::DoCheckDeadlines(
    std::chrono::steady_clock::time_point now) {
  EngineEffects effects;
  ExpireProvisionalControllers(now, &effects.adapter_changes);
  UpdateObserverDeadlines(now, &effects.adapter_changes);
  effects.step_completed =
      UpdateBarrierDeadlines(now, &effects.adapter_changes);
  effects.status_changed = !effects.adapter_changes.empty() ||
                           effects.step_completed.has_value();
  return effects;
}

void SimulationCore::Impl::ExpireProvisionalControllers(
    std::chrono::steady_clock::time_point now,
    std::vector<AdapterChange>* changes) {
  std::vector<std::string> expired_controllers;
  for (Controller* controller : adapter_registry_->Controllers()) {
    if (controller->phase == ControllerPhase::kProvisional &&
        now - controller->registered_at >
            std::chrono::milliseconds(options_.activation_timeout_ms)) {
      changes->push_back(
          {AdapterChangeKind::kActivationExpired, controller->id,
           mujoco_sim::v2::CONTROLLER, controller->session_id, std::nullopt,
           PROVISIONAL, UNREGISTERED, "controller activation timed out",
           current_tick_id_});
      ReleaseControllerResources(*controller);
      expired_controllers.push_back(controller->id);
    }
  }
  for (const auto& id : expired_controllers) adapter_registry_->EraseController(id);
}

void SimulationCore::Impl::UpdateObserverDeadlines(
    std::chrono::steady_clock::time_point now,
    std::vector<AdapterChange>* changes) {
  std::vector<std::string> expired_observers;
  for (Observer* observer : adapter_registry_->Observers()) {
    if (observer->phase == ObserverPhase::kProvisional &&
        now - observer->registered_at >
            std::chrono::milliseconds(options_.activation_timeout_ms)) {
      changes->push_back(
          {AdapterChangeKind::kActivationExpired, observer->id, OBSERVER,
           observer->session_id, std::nullopt, PROVISIONAL, UNREGISTERED,
           "observer activation timed out", current_tick_id_});
      expired_observers.push_back(observer->id);
      continue;
    }
    if (observer->phase == ObserverPhase::kActive &&
        now - observer->last_heartbeat_at >
            std::chrono::milliseconds(observer->timeout_ms)) {
      const auto old_state = WireState(observer->phase);
      ObserverStateMachine::MarkTimedOut(
          observer, current_tick_id_, "observer keepalive timed out");
      changes->push_back(
          {AdapterChangeKind::kTimedOut, observer->id, OBSERVER,
           observer->session_id, std::nullopt, old_state, TIMED_OUT,
           observer->message, current_tick_id_});
    }
  }
  for (const auto& id : expired_observers) adapter_registry_->EraseObserver(id);
}

std::optional<StepCompletedEvent> SimulationCore::Impl::UpdateBarrierDeadlines(
    std::chrono::steady_clock::time_point now,
    std::vector<AdapterChange>* changes) {
  if (!tick_open_ || server_state_ == ERROR_PAUSED) return std::nullopt;
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now - tick_started_at_)
                              .count();
  for (Controller* controller : adapter_registry_->Controllers()) {
    if (!barrier_coordinator_->IsRequired(controller->id) ||
        barrier_coordinator_->HasPendingCommit(controller->id) ||
        elapsed_ms <= controller->timeout_ms) {
      continue;
    }
    const auto old_state = WireState(controller->phase);
    ControllerStateMachine::MarkTimedOut(
        controller, current_tick_id_,
        "controller timed out waiting for Commit");
    ZeroControllerCommands(*controller);
    barrier_coordinator_->RemoveRequiredController(controller->id);
    changes->push_back(
        {AdapterChangeKind::kTimedOut, controller->id,
         mujoco_sim::v2::CONTROLLER, controller->session_id, std::nullopt,
         old_state, TIMED_OUT, controller->message, current_tick_id_});
  }
  return CompleteBarrier(StepCompletionSource::kDeadline);
}

mujoco_sim::v2::ServerStatus SimulationCore::Impl::MakeStatus() const {
  return status_projection_->Build(MakeHeader(0), server_state_, status_message_,
                                   *adapter_registry_, *barrier_coordinator_);
}

void SimulationCore::Impl::ReleaseControllerResources(
    const Controller& controller) {
  for (const auto& command : controller.layout.commands()) {
    adapter_registry_->ReleaseActuator(command.name(), controller.id);
  }
}

void SimulationCore::Impl::EnterErrorPaused(const std::string& message) {
  server_state_ = ERROR_PAUSED;
  status_message_ = message;
  tick_open_ = false;
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  for (Controller* controller : adapter_registry_->Controllers()) {
    if (controller->phase == ControllerPhase::kActive) {
      ControllerStateMachine::MarkError(controller, current_tick_id_, message);
    }
  }
  for (Observer* observer : adapter_registry_->Observers()) {
    if (observer->phase == ObserverPhase::kActive) {
      ObserverStateMachine::MarkError(observer, current_tick_id_, message);
    }
  }
}

void SimulationCore::Impl::EnterFatalError(const std::string& message) {
  EnterErrorPaused(message);
}

void SimulationCore::Impl::MarkStopped(const std::string& message) {
  server_state_ = mujoco_sim::v2::STOPPED;
  status_message_ = message;
  tick_open_ = false;
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
}

std::uint64_t SimulationCore::Impl::epoch() const { return epoch_; }

std::uint64_t SimulationCore::Impl::current_tick_id() const {
  return current_tick_id_;
}

double SimulationCore::Impl::simulation_time() const {
  return model_state_->simulation_time();
}

double SimulationCore::Impl::timestep() const {
  return model_state_->timestep();
}

mujoco_sim::v2::ServerState SimulationCore::Impl::server_state() const {
  return server_state_;
}

bool SimulationCore::Impl::tick_open() const { return tick_open_; }

CoreState SimulationCore::Impl::state() const {
  CoreState result;
  result.epoch = epoch();
  result.current_tick_id = current_tick_id();
  result.simulation_time_s = simulation_time();
  result.timestep_s = timestep();
  result.server_state = server_state();
  result.tick_open = tick_open();
  CountControllers(adapter_registry_->Controllers(), &result);
  CountObservers(adapter_registry_->Observers(), &result);
  return result;
}

std::optional<AdapterInfo> SimulationCore::Impl::FindAdapterInfo(
    const std::string& adapter_id,
    std::optional<mujoco_sim::v2::AdapterRole> role) const {
  if (!role || *role == mujoco_sim::v2::CONTROLLER) {
    const Controller* found = adapter_registry_->FindController(adapter_id);
    if (found != nullptr) {
      return AdapterInfo{adapter_id, mujoco_sim::v2::CONTROLLER,
                         found->session_id, WireState(found->phase)};
    }
  }
  if (!role || *role == OBSERVER) {
    const Observer* found = adapter_registry_->FindObserver(adapter_id);
    if (found != nullptr) {
      return AdapterInfo{adapter_id, OBSERVER, found->session_id,
                         WireState(found->phase)};
    }
  }
  return std::nullopt;
}

std::vector<AdapterInfo> SimulationCore::Impl::AdapterInfos() const {
  std::vector<AdapterInfo> infos;
  infos.reserve(adapter_registry_->ControllerCount() +
                adapter_registry_->ObserverCount());
  for (const Controller* controller : adapter_registry_->Controllers()) {
    infos.push_back({controller->id, mujoco_sim::v2::CONTROLLER,
                     controller->session_id, WireState(controller->phase)});
  }
  for (const Observer* observer : adapter_registry_->Observers()) {
    infos.push_back({observer->id, OBSERVER, observer->session_id,
                     WireState(observer->phase)});
  }
  return infos;
}

std::vector<AdapterInfo>
SimulationCore::Impl::RecoveryPendingControllers() const {
  std::vector<AdapterInfo> infos;
  for (const Controller* controller : adapter_registry_->Controllers()) {
    if (controller->phase == ControllerPhase::kRecoveryPending) {
      infos.push_back({controller->id, mujoco_sim::v2::CONTROLLER,
                       controller->session_id, TIMED_OUT});
    }
  }
  return infos;
}

std::uint64_t SimulationCore::Impl::ControllerHeartbeatTick(
    const std::string& adapter_id) const {
  const Controller* found = adapter_registry_->FindController(adapter_id);
  return found == nullptr ? 0 : found->last_heartbeat_tick;
}

std::chrono::steady_clock::time_point
SimulationCore::Impl::ObserverHeartbeatTime(
    const std::string& adapter_id) const {
  const Observer* found = adapter_registry_->FindObserver(adapter_id);
  return found == nullptr ? std::chrono::steady_clock::time_point{}
                          : found->last_heartbeat_at;
}

}  // namespace mujoco_sim::core
