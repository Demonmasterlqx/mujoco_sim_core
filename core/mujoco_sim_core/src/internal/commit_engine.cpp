#include "simulation_core_impl.hpp"

#include <cmath>
#include <bit>
#include <string>
#include <utility>
#include <vector>


namespace mujoco_sim::core {
namespace {

using mujoco_sim::core::ACTIVE;
using mujoco_sim::core::ERROR;
using mujoco_sim::core::ERROR_PAUSED;
using mujoco_sim::core::FAILED_PRECONDITION;
using mujoco_sim::core::INVALID_ARGUMENT;
using mujoco_sim::core::INVALID_COMMAND;
using mujoco_sim::core::INVALID_EPOCH;
using mujoco_sim::core::INVALID_SESSION;
using mujoco_sim::core::INVALID_TICK;
using mujoco_sim::core::TIMED_OUT;
using mujoco_sim::core::VERSION_MISMATCH;
using internal::ControllerStateMachine;

bool SameCommandPayload(const std::vector<double>& lhs,
                        const std::vector<double>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (std::bit_cast<std::uint64_t>(lhs[index]) !=
        std::bit_cast<std::uint64_t>(rhs[index])) {
      return false;
    }
  }
  return true;
}

bool SameCommitPayload(const mujoco_sim::core::AdapterCommit& lhs,
                       const mujoco_sim::core::AdapterCommit& rhs) {
  return SameCommandPayload(lhs.commands, rhs.commands);
}


}  // namespace

EngineResult<CommitReceipt> SimulationCore::Impl::DoSubmitCommit(
    const mujoco_sim::core::AdapterCommit& commit) {
  EngineResult<CommitReceipt> result;
  if (server_state_ == ERROR_PAUSED) {
    result.response.code = FAILED_PRECONDITION;
    result.response.message = "server is error-paused";
    return result;
  }
  Controller* controller = adapter_registry_->FindControllerBySession(
      commit.identity.session.session_id);
  if (controller == nullptr) {
    result.response.code = INVALID_SESSION;
    result.response.message = "controller session is not registered";
    return result;
  }
  if (!ValidateCommitEnvelope(*controller, commit, &result)) return result;

  if (ControllerStateMachine::AcceptsRecoveryCommit(controller->phase)) {
    return HandleRecoveryCommit(controller, commit);
  }
  return HandleActiveCommit(controller, commit);
}

EngineResult<CommitReceipt> SimulationCore::Impl::DoSubmitFault(
    const ControllerFault& fault) {
  EngineResult<CommitReceipt> result;
  Controller* controller = adapter_registry_->FindControllerBySession(
      fault.identity.session.session_id);
  if (controller == nullptr) {
    result.response.code = INVALID_SESSION;
    result.response.message = "controller session is not registered";
    return result;
  }
  if (fault.identity.session.epoch != epoch_) {
    result.response.code = INVALID_EPOCH;
    result.response.message = "epoch mismatch";
    return result;
  }
  if (fault.identity.tick_id != current_tick_id_ ||
      fault.identity.layout != controller->layout.fingerprint) {
    result.response.code = INVALID_TICK;
    result.response.message = "fault identity mismatch";
    return result;
  }
  result.effects.step_completed = EnterControllerError(controller, fault.message);
  result.response.code = INVALID_COMMAND;
  result.response.message = controller->message;
  RecordCommitChange(&result, AdapterChangeKind::kLocalError, *controller,
                     ACTIVE, ERROR);
  return result;
}

bool SimulationCore::Impl::ValidateCommitEnvelope(
    const Controller& controller,
    const mujoco_sim::core::AdapterCommit& commit,
    EngineResult<CommitReceipt>* result) const {
  if (commit.identity.session.session_id != controller.session_id) {
    result->response.code = INVALID_SESSION;
    result->response.message = "session mismatch";
  } else if (commit.identity.session.epoch != epoch_) {
    result->response.code = INVALID_EPOCH;
    result->response.message = "epoch mismatch";
  } else if (commit.identity.layout != controller.layout.fingerprint) {
    result->response.code = INVALID_ARGUMENT;
    result->response.message = "layout fingerprint mismatch";
  } else {
    return true;
  }
  return false;
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandleRecoveryCommit(
    Controller* controller, const mujoco_sim::core::AdapterCommit& commit) {
  EngineResult<CommitReceipt> result;
  const std::uint64_t tick_id = commit.identity.tick_id;
  if (controller->recovery_proof &&
      tick_id == controller->recovery_proof->identity.tick_id) {
    if (SameCommitPayload(commit, *controller->recovery_proof)) {
      controller->last_presence_at = std::chrono::steady_clock::now();
      result.response.duplicate = true;
      result.response.message = "duplicate recovery commit ignored";
      return result;
    }
    result.effects.step_completed = EnterControllerError(
        controller, "duplicate recovery commit payload mismatch");
    result.response.code = INVALID_COMMAND;
    result.response.message = controller->message;
    RecordCommitChange(&result, AdapterChangeKind::kLocalError, *controller,
                       TIMED_OUT, ERROR);
    return result;
  }
  if (tick_id < controller->timed_out_since_tick ||
      !controller->last_tick_sent || tick_id > *controller->last_tick_sent ||
      (controller->recovery_proof &&
       tick_id <= controller->recovery_proof->identity.tick_id)) {
    result.response.code = INVALID_TICK;
    result.response.message =
        "commit does not match a new Tick issued for recovery";
    return result;
  }
  if (!ValidateCommands(*controller, commit, &result.response.message)) {
    result.response.code = INVALID_COMMAND;
    return result;
  }
  ControllerStateMachine::AcceptRecoveryProof(controller, commit);
  controller->last_presence_at = std::chrono::steady_clock::now();
  result.response.message = controller->message;
  RecordCommitChange(&result, AdapterChangeKind::kRecoveryPending,
                     *controller, TIMED_OUT, TIMED_OUT);
  result.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return result;
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandleActiveCommit(
    Controller* controller, const mujoco_sim::core::AdapterCommit& commit) {
  if (!ControllerStateMachine::AcceptsBarrierCommit(controller->phase)) {
    CommitReceipt receipt;
    receipt.code = FAILED_PRECONDITION;
    receipt.message = "controller is not active";
    return {std::move(receipt), {}};
  }
  if (commit.identity.tick_id < current_tick_id_) {
    return HandleStaleCommit(controller, commit);
  }
  if (auto error = ValidateCurrentCommitContext(*controller, commit)) {
    return {std::move(*error), {}};
  }
  const auto* pending =
      barrier_coordinator_->FindPendingCommit(controller->id);
  if (pending != nullptr) {
    return HandlePendingCommit(controller, commit, *pending);
  }
  return AcceptActiveCommit(controller, commit);
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandleStaleCommit(
    Controller* controller, const mujoco_sim::core::AdapterCommit& commit) {
  if (!controller->last_commit ||
      commit.identity.tick_id != controller->last_commit->identity.tick_id) {
    CommitReceipt receipt;
    receipt.code = INVALID_TICK;
    receipt.message = "stale commit cannot be verified";
    return {std::move(receipt), {}};
  }
  if (!SameCommitPayload(commit, *controller->last_commit)) {
    return RejectActiveCommit(
        controller, "stale duplicate commit payload mismatch");
  }
  controller->last_presence_at = std::chrono::steady_clock::now();
  CommitReceipt receipt;
  receipt.duplicate = true;
  receipt.message = "stale duplicate commit ignored";
  return {std::move(receipt), {}};
}

std::optional<CommitReceipt>
SimulationCore::Impl::ValidateCurrentCommitContext(
    const Controller& controller,
    const mujoco_sim::core::AdapterCommit& commit) const {
  CommitReceipt error;
  if (commit.identity.tick_id > current_tick_id_) {
    error.code = INVALID_TICK;
    error.message = "tick mismatch";
  } else if (!tick_open_) {
    error.code = FAILED_PRECONDITION;
    error.message = "no tick is open";
  } else if (!barrier_coordinator_->IsRequired(controller.id)) {
    error.code = FAILED_PRECONDITION;
    error.message = "controller joins the barrier on the next Tick";
  } else if (commit.identity.session.session_id != controller.session_id ||
             commit.identity.session.epoch != epoch_ ||
             commit.identity.tick_id != current_tick_id_) {
    error.code = INVALID_TICK;
    error.message = "tick identity mismatch";
  } else {
    return std::nullopt;
  }
  return error;
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandlePendingCommit(
    Controller* controller, const mujoco_sim::core::AdapterCommit& commit,
    const mujoco_sim::core::AdapterCommit& pending) {
  if (!SameCommitPayload(commit, pending)) {
    return RejectActiveCommit(controller, "duplicate commit payload mismatch");
  }
  controller->last_presence_at = std::chrono::steady_clock::now();
  CommitReceipt receipt;
  receipt.duplicate = true;
  receipt.message = "duplicate commit ignored";
  return {std::move(receipt), {}};
}

EngineResult<CommitReceipt> SimulationCore::Impl::RejectActiveCommit(
    Controller* controller, const std::string& message) {
  EngineResult<CommitReceipt> result;
  result.effects.step_completed = EnterControllerError(controller, message);
  result.response.code = INVALID_COMMAND;
  result.response.message = controller->message;
  RecordCommitChange(&result, AdapterChangeKind::kLocalError, *controller,
                     ACTIVE, ERROR);
  return result;
}

EngineResult<CommitReceipt> SimulationCore::Impl::AcceptActiveCommit(
    Controller* controller, const mujoco_sim::core::AdapterCommit& commit) {
  std::string message;
  if (!ValidateCommands(*controller, commit, &message)) {
    return RejectActiveCommit(controller, message);
  }

  EngineResult<CommitReceipt> result;
  barrier_coordinator_->AcceptCommit(controller->id, commit,
                                     controller->layout);
  controller->last_commit = commit;
  controller->last_presence_at = std::chrono::steady_clock::now();
  if (!barrier_coordinator_->Ready()) {
    result.response.message = "commit accepted";
  } else {
    result.effects.step_completed =
        CompleteBarrier(StepCompletionSource::kCommit);
    result.response.message = result.effects.step_completed
                                  ? "barrier complete"
                                  : "commit accepted";
  }
  result.effects.status_urgency =
      EngineEffects::StatusUrgency::kPeriodicOnly;
  return result;
}


bool SimulationCore::Impl::ValidateCommands(
    const Controller& controller,
    const mujoco_sim::core::AdapterCommit& commit,
    std::string* message) const {
  if (commit.commands.size() != controller.layout.command_scalar_count) {
    *message = "commit scalar count does not match layout";
    return false;
  }
  for (std::size_t resource = 0;
       resource < controller.layout.commands_.size(); ++resource) {
    const auto& descriptor = controller.layout.commands_[resource];
    const auto& span = controller.layout.command_spans[resource];
    for (std::uint32_t offset = 0; offset < span.dimension; ++offset) {
      const double value = commit.commands[span.offset + offset];
      if (!std::isfinite(value)) {
        *message = "actuator command is not finite: " + descriptor.name();
        return false;
      }
      if (descriptor.has_ctrl_range() &&
          (value < descriptor.ctrl_min() || value > descriptor.ctrl_max())) {
        *message = "actuator command is outside ctrl range: " +
                   descriptor.name();
        return false;
      }
    }
  }
  return true;
}

void SimulationCore::Impl::ZeroControllerCommands(const Controller& controller) {
  std::vector<int> actuator_ids;
  actuator_ids.reserve(controller.layout.commands_size());
  for (const auto& command : controller.layout.commands()) {
    actuator_ids.push_back(command.mujoco_id());
  }
  if (tick_open_) {
    barrier_coordinator_->QueueZeroActuators(actuator_ids);
  } else {
    model_state_->ZeroActuators(actuator_ids);
  }
}

std::optional<StepCompletedEvent> SimulationCore::Impl::EnterControllerError(
    Controller* controller, const std::string& message) {
  ControllerStateMachine::MarkError(controller, current_tick_id_, message);
  ZeroControllerCommands(*controller);
  InvalidateIssuedTick(controller->id);
  barrier_coordinator_->RemoveController(controller->id);
  return CompleteBarrier(StepCompletionSource::kLocalError);
}

void SimulationCore::Impl::RecordCommitChange(
    EngineResult<CommitReceipt>* result, AdapterChangeKind kind,
    const Controller& controller,
    mujoco_sim::core::ControllerState old_state,
    mujoco_sim::core::ControllerState new_state) const {
  result->effects.adapter_changes.push_back(
      {kind, controller.id, mujoco_sim::core::CONTROLLER,
       controller.session_id, std::nullopt, old_state, new_state,
       result->response.message, current_tick_id_});
  result->effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
}


}  // namespace mujoco_sim::core
