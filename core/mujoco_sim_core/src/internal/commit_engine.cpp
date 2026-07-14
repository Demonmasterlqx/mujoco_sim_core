#include "simulation_core_impl.hpp"

#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v2::ACTIVE;
using mujoco_sim::v2::ERROR;
using mujoco_sim::v2::ERROR_PAUSED;
using mujoco_sim::v2::FAILED_PRECONDITION;
using mujoco_sim::v2::INVALID_ARGUMENT;
using mujoco_sim::v2::INVALID_COMMAND;
using mujoco_sim::v2::INVALID_EPOCH;
using mujoco_sim::v2::INVALID_SESSION;
using mujoco_sim::v2::INVALID_TICK;
using mujoco_sim::v2::NOT_FOUND;
using mujoco_sim::v2::TIMED_OUT;
using mujoco_sim::v2::VERSION_MISMATCH;
using internal::ControllerStateMachine;

bool SameCommandPayload(
    const google::protobuf::RepeatedPtrField<
        mujoco_sim::v2::ActuatorCommandBlock>& lhs,
    const google::protobuf::RepeatedPtrField<
        mujoco_sim::v2::ActuatorCommandBlock>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  std::map<std::string, std::vector<double>> lhs_by_name;
  for (const auto& command : lhs) {
    lhs_by_name.emplace(command.name(),
                        std::vector<double>(command.values().begin(),
                                            command.values().end()));
  }
  std::map<std::string, std::vector<double>> rhs_by_name;
  for (const auto& command : rhs) {
    rhs_by_name.emplace(command.name(),
                        std::vector<double>(command.values().begin(),
                                            command.values().end()));
  }
  return lhs_by_name == rhs_by_name;
}

bool SameCommitPayload(const mujoco_sim::v2::AdapterCommit& lhs,
                       const mujoco_sim::v2::AdapterCommit& rhs) {
  return lhs.status() == rhs.status() &&
         lhs.error_message() == rhs.error_message() &&
         SameCommandPayload(lhs.commands(), rhs.commands());
}


}  // namespace

EngineResult<CommitReceipt> SimulationCore::Impl::DoSubmitCommit(
    const mujoco_sim::v2::AdapterCommit& commit) {
  EngineResult<CommitReceipt> result;
  if (server_state_ == ERROR_PAUSED) {
    result.response.code = FAILED_PRECONDITION;
    result.response.message = "server is error-paused";
    return result;
  }
  Controller* controller =
      adapter_registry_->FindController(commit.adapter_id());
  if (controller == nullptr) {
    result.response.code = NOT_FOUND;
    result.response.message = "controller is not registered";
    return result;
  }
  if (!ValidateCommitEnvelope(*controller, commit, &result)) return result;

  if (ControllerStateMachine::AcceptsRecoveryCommit(controller->phase)) {
    return HandleRecoveryCommit(controller, commit);
  }
  return HandleActiveCommit(controller, commit);
}

bool SimulationCore::Impl::ValidateCommitEnvelope(
    const Controller& controller,
    const mujoco_sim::v2::AdapterCommit& commit,
    EngineResult<CommitReceipt>* result) const {
  const auto& header = commit.header();
  if (header.server_name() != options_.server_name) {
    result->response.code = INVALID_ARGUMENT;
    result->response.message = "server name mismatch";
  } else if (header.session_id() != controller.session_id) {
    result->response.code = INVALID_SESSION;
    result->response.message = "session mismatch";
  } else if (header.epoch() != epoch_) {
    result->response.code = INVALID_EPOCH;
    result->response.message = "epoch mismatch";
  } else if (header.protocol_major() !=
             mujoco_sim::protocol::kProtocolMajor) {
    result->response.code = VERSION_MISMATCH;
    result->response.message = "protocol major mismatch";
  } else if (header.protocol_minor() != controller.protocol_minor) {
    result->response.code = VERSION_MISMATCH;
    result->response.message = "protocol minor mismatch";
  } else {
    return true;
  }
  return false;
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandleRecoveryCommit(
    Controller* controller, const mujoco_sim::v2::AdapterCommit& commit) {
  EngineResult<CommitReceipt> result;
  const std::uint64_t tick_id = commit.header().tick_id();
  if (controller->recovery_proof &&
      tick_id == controller->recovery_proof->header().tick_id()) {
    if (SameCommitPayload(commit, *controller->recovery_proof)) {
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
       tick_id <= controller->recovery_proof->header().tick_id())) {
    result.response.code = INVALID_TICK;
    result.response.message =
        "commit does not match a new Tick issued for recovery";
    return result;
  }
  if (commit.status() != mujoco_sim::v2::COMMIT_OK) {
    result.response.code = INVALID_COMMAND;
    result.response.message =
        "timed-out controller did not report a valid commit";
    return result;
  }
  if (!ValidateCommands(*controller, commit, &result.response.message)) {
    result.response.code = INVALID_COMMAND;
    return result;
  }
  ControllerStateMachine::AcceptRecoveryProof(controller, commit);
  result.response.message = controller->message;
  RecordCommitChange(&result, AdapterChangeKind::kRecoveryPending,
                     *controller, TIMED_OUT, TIMED_OUT);
  result.effects.status_changed = true;
  return result;
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandleActiveCommit(
    Controller* controller, const mujoco_sim::v2::AdapterCommit& commit) {
  if (!ControllerStateMachine::AcceptsBarrierCommit(controller->phase)) {
    CommitReceipt receipt;
    receipt.code = FAILED_PRECONDITION;
    receipt.message = "controller is not active";
    return {std::move(receipt), {}};
  }
  if (commit.header().tick_id() < current_tick_id_) {
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
    Controller* controller, const mujoco_sim::v2::AdapterCommit& commit) {
  if (!controller->last_commit ||
      commit.header().tick_id() !=
          controller->last_commit->header().tick_id()) {
    CommitReceipt receipt;
    receipt.code = INVALID_TICK;
    receipt.message = "stale commit cannot be verified";
    return {std::move(receipt), {}};
  }
  if (!SameCommitPayload(commit, *controller->last_commit)) {
    return RejectActiveCommit(
        controller, "stale duplicate commit payload mismatch");
  }
  CommitReceipt receipt;
  receipt.duplicate = true;
  receipt.message = "stale duplicate commit ignored";
  return {std::move(receipt), {}};
}

std::optional<CommitReceipt>
SimulationCore::Impl::ValidateCurrentCommitContext(
    const Controller& controller,
    const mujoco_sim::v2::AdapterCommit& commit) const {
  CommitReceipt error;
  if (commit.header().tick_id() > current_tick_id_) {
    error.code = INVALID_TICK;
    error.message = "tick mismatch";
  } else if (!tick_open_) {
    error.code = FAILED_PRECONDITION;
    error.message = "no tick is open";
  } else if (!barrier_coordinator_->IsRequired(controller.id)) {
    error.code = FAILED_PRECONDITION;
    error.message = "controller joins the barrier on the next Tick";
  } else if (!ValidateCommonHeader(commit.header(), controller,
                                   &error.message)) {
    error.code = error.message == "tick mismatch" ? INVALID_TICK
                                                   : INVALID_ARGUMENT;
  } else {
    return std::nullopt;
  }
  return error;
}

EngineResult<CommitReceipt> SimulationCore::Impl::HandlePendingCommit(
    Controller* controller, const mujoco_sim::v2::AdapterCommit& commit,
    const mujoco_sim::v2::AdapterCommit& pending) {
  if (!SameCommitPayload(commit, pending)) {
    return RejectActiveCommit(controller, "duplicate commit payload mismatch");
  }
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
    Controller* controller, const mujoco_sim::v2::AdapterCommit& commit) {
  if (commit.status() != mujoco_sim::v2::COMMIT_OK) {
    return RejectActiveCommit(
        controller,
        "controller reported commit error: " + commit.error_message());
  }
  std::string message;
  if (!ValidateCommands(*controller, commit, &message)) {
    return RejectActiveCommit(controller, message);
  }

  EngineResult<CommitReceipt> result;
  barrier_coordinator_->AcceptCommit(controller->id, commit);
  controller->last_commit = commit;
  if (!barrier_coordinator_->Ready()) {
    result.response.message = "commit accepted";
  } else {
    result.effects.step_completed =
        CompleteBarrier(StepCompletionSource::kCommit);
    result.response.message = result.effects.step_completed
                                  ? "barrier complete"
                                  : "commit accepted";
  }
  result.effects.status_changed = true;
  return result;
}


bool SimulationCore::Impl::ValidateCommands(
    const Controller& controller,
    const mujoco_sim::v2::AdapterCommit& commit,
    std::string* message) const {
  std::set<std::string> expected;
  for (const auto& command : controller.layout.commands()) {
    expected.insert(command.name());
  }
  std::set<std::string> seen;
  for (const auto& command : commit.commands()) {
    if (!expected.contains(command.name())) {
      *message = "controller submitted unowned actuator: " + command.name();
      return false;
    }
    if (!seen.insert(command.name()).second) {
      *message = "duplicate actuator command: " + command.name();
      return false;
    }
    if (command.values_size() != 1) {
      *message = "actuator command must contain exactly one value: " +
                 command.name();
      return false;
    }
    const double value = command.values(0);
    if (!std::isfinite(value)) {
      *message = "actuator command is not finite: " + command.name();
      return false;
    }
    const auto* descriptor = resource_catalog_->FindCommand(command.name());
    if (descriptor == nullptr) {
      *message = "controller submitted unknown actuator: " + command.name();
      return false;
    }
    if (descriptor->has_ctrl_range() &&
        (value < descriptor->ctrl_min() || value > descriptor->ctrl_max())) {
      *message = "actuator command is outside ctrl range: " + command.name();
      return false;
    }
  }
  if (seen != expected) {
    *message = "commit does not include all claimed actuators";
    return false;
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
    mujoco_sim::v2::ControllerState old_state,
    mujoco_sim::v2::ControllerState new_state) const {
  result->effects.adapter_changes.push_back(
      {kind, controller.id, mujoco_sim::v2::CONTROLLER,
       controller.session_id, std::nullopt, old_state, new_state,
       result->response.message, current_tick_id_});
  result->effects.status_changed = true;
}


}  // namespace mujoco_sim::core
