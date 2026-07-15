#include "controller_state_machine.hpp"

#include "adapter_registry.hpp"

namespace mujoco_sim::core::internal {

mujoco_sim::core::ControllerState WireState(ControllerPhase phase) noexcept {
  switch (phase) {
    case ControllerPhase::kProvisional:
      return mujoco_sim::core::PROVISIONAL;
    case ControllerPhase::kActive:
      return mujoco_sim::core::ACTIVE;
    case ControllerPhase::kTimedOutAwaitingProof:
    case ControllerPhase::kRecoveryPending:
      return mujoco_sim::core::TIMED_OUT;
    case ControllerPhase::kError:
      return mujoco_sim::core::ERROR;
  }
  return mujoco_sim::core::CONTROLLER_STATE_UNSPECIFIED;
}

mujoco_sim::core::ControllerState WireState(ObserverPhase phase) noexcept {
  switch (phase) {
    case ObserverPhase::kProvisional:
      return mujoco_sim::core::PROVISIONAL;
    case ObserverPhase::kActive:
      return mujoco_sim::core::ACTIVE;
    case ObserverPhase::kTimedOut:
      return mujoco_sim::core::TIMED_OUT;
    case ObserverPhase::kError:
      return mujoco_sim::core::ERROR;
  }
  return mujoco_sim::core::CONTROLLER_STATE_UNSPECIFIED;
}

bool ControllerStateMachine::CanActivate(ControllerPhase phase) noexcept {
  return phase == ControllerPhase::kProvisional;
}

bool ControllerStateMachine::AcceptsBarrierCommit(
    ControllerPhase phase) noexcept {
  return phase == ControllerPhase::kActive;
}

bool ControllerStateMachine::AcceptsRecoveryCommit(
    ControllerPhase phase) noexcept {
  return phase == ControllerPhase::kTimedOutAwaitingProof ||
         phase == ControllerPhase::kRecoveryPending;
}

bool ControllerStateMachine::IsTickRecipient(ControllerPhase phase) noexcept {
  return phase == ControllerPhase::kActive ||
         phase == ControllerPhase::kTimedOutAwaitingProof ||
         phase == ControllerPhase::kRecoveryPending;
}

bool ControllerStateMachine::Activate(ControllerRecord* controller,
                                      std::uint64_t tick_id) {
  if (!CanActivate(controller->phase)) return false;
  controller->phase = ControllerPhase::kActive;
  controller->state_since_tick = tick_id;
  controller->last_presence_at = std::chrono::steady_clock::now();
  controller->message.clear();
  return true;
}

bool ControllerStateMachine::AcceptRecoveryProof(
    ControllerRecord* controller,
    const mujoco_sim::core::AdapterCommit& commit) {
  if (!AcceptsRecoveryCommit(controller->phase)) return false;
  controller->phase = ControllerPhase::kRecoveryPending;
  controller->recovery_proof = commit;
  controller->last_commit = commit;
  controller->message = "valid recovery commit received; activation pending";
  return true;
}

bool ControllerStateMachine::ActivateRecovered(ControllerRecord* controller,
                                                std::uint64_t tick_id) {
  if (controller->phase != ControllerPhase::kRecoveryPending) return false;
  controller->phase = ControllerPhase::kActive;
  controller->state_since_tick = tick_id;
  controller->last_presence_at = std::chrono::steady_clock::now();
  controller->message.clear();
  return true;
}

bool ControllerStateMachine::MarkTimedOut(ControllerRecord* controller,
                                          std::uint64_t tick_id,
                                          const std::string& message) {
  if (controller->phase != ControllerPhase::kActive) return false;
  controller->phase = ControllerPhase::kTimedOutAwaitingProof;
  controller->state_since_tick = tick_id;
  controller->message = message;
  controller->timed_out_since_tick = tick_id;
  controller->recovery_proof.reset();
  ++controller->timeout_count;
  return true;
}

bool ControllerStateMachine::MarkError(ControllerRecord* controller,
                                       std::uint64_t tick_id,
                                       const std::string& message) {
  if (controller->phase == ControllerPhase::kError) return false;
  controller->phase = ControllerPhase::kError;
  controller->state_since_tick = tick_id;
  controller->message = message;
  return true;
}

bool ObserverStateMachine::Activate(ObserverRecord* observer,
                                    std::uint64_t tick_id) {
  if (observer->phase != ObserverPhase::kProvisional) return false;
  observer->phase = ObserverPhase::kActive;
  observer->state_since_tick = tick_id;
  observer->activation_tick = tick_id;
  observer->last_heartbeat_at = std::chrono::steady_clock::now();
  observer->message.clear();
  return true;
}

bool ObserverStateMachine::MarkTimedOut(ObserverRecord* observer,
                                        std::uint64_t tick_id,
                                        const std::string& message) {
  if (observer->phase != ObserverPhase::kActive) return false;
  observer->phase = ObserverPhase::kTimedOut;
  observer->state_since_tick = tick_id;
  observer->message = message;
  ++observer->timeout_count;
  return true;
}

bool ObserverStateMachine::MarkError(ObserverRecord* observer,
                                     std::uint64_t tick_id,
                                     const std::string& message) {
  if (observer->phase == ObserverPhase::kError) return false;
  observer->phase = ObserverPhase::kError;
  observer->state_since_tick = tick_id;
  observer->message = message;
  return true;
}

}  // namespace mujoco_sim::core::internal
