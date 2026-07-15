#include "simulation_core_impl.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <utility>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::core::ACTIVE;
using mujoco_sim::core::ALREADY_EXISTS;
using mujoco_sim::core::ERROR_PAUSED;
using mujoco_sim::core::FAILED_PRECONDITION;
using mujoco_sim::core::INVALID_ARGUMENT;
using mujoco_sim::core::INVALID_EPOCH;
using mujoco_sim::core::INVALID_SESSION;
using mujoco_sim::core::NOT_FOUND;
using mujoco_sim::core::OBSERVER;
using mujoco_sim::core::OK;
using mujoco_sim::core::PROVISIONAL;
using mujoco_sim::core::RESOURCE_CONFLICT;
using mujoco_sim::core::TIMED_OUT;
using mujoco_sim::core::UNREGISTERED;
using mujoco_sim::core::VERSION_MISMATCH;
using internal::ControllerStateMachine;
using internal::ObserverPhase;
using internal::ObserverStateMachine;
using internal::WireState;

template <typename Response>
Response ErrorResponse(mujoco_sim::core::ErrorCode code,
                       const std::string& message) {
  Response response;
  response.set_code(code);
  response.set_message(message);
  return response;
}


}  // namespace

mujoco_sim::core::ActivateAdapterResponse
SimulationCore::Impl::ActivateControlAdapter(
    const mujoco_sim::core::ActivateAdapterRequest& request) {
  Controller* controller =
      adapter_registry_->FindController(request.adapter_id());
  if (controller == nullptr) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        NOT_FOUND, "controller is not registered");
  }
  if (request.session_id() != controller->session_id) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::core::STOPPED) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  if (!ControllerStateMachine::CanActivate(controller->phase)) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "controller is not provisional");
  }
  ControllerStateMachine::Activate(controller, current_tick_id_);
  mujoco_sim::core::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

EngineResult<mujoco_sim::core::UnregisterAdapterResponse>
SimulationCore::Impl::UnregisterControlAdapter(
    const mujoco_sim::core::UnregisterAdapterRequest& request) {
  Controller* controller =
      adapter_registry_->FindController(request.adapter_id());
  if (controller == nullptr) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                NOT_FOUND, "controller is not registered"),
            {}};
  }
  if (request.session_id() != controller->session_id) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                INVALID_SESSION, "session mismatch"),
            {}};
  }
  if (request.epoch() != epoch_) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                INVALID_EPOCH, "epoch mismatch"),
            {}};
  }
  const std::string controller_id = controller->id;
  ZeroControllerCommands(*controller);
  ReleaseControllerResources(*controller);
  InvalidateIssuedTick(controller_id);
  barrier_coordinator_->RemoveController(controller_id);
  adapter_registry_->EraseController(controller_id);
  auto step_completed = CompleteBarrier(StepCompletionSource::kUnregister);

  mujoco_sim::core::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  EngineEffects effects;
  effects.step_completed = std::move(step_completed);
  return {std::move(response), std::move(effects)};
}

mujoco_sim::core::ActivateAdapterResponse SimulationCore::Impl::DoActivateAdapter(
    const mujoco_sim::core::ActivateAdapterRequest& request) {
  if (request.role() == mujoco_sim::core::CONTROLLER) {
    return ActivateControlAdapter(request);
  }
  if (request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  Observer* observer = adapter_registry_->FindObserver(request.adapter_id());
  if (observer == nullptr) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        NOT_FOUND, "observer is not registered");
  }
  if (request.session_id() != observer->session_id) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (observer->phase != ObserverPhase::kProvisional) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "observer is not provisional");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::core::STOPPED) {
    return ErrorResponse<mujoco_sim::core::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  ObserverStateMachine::Activate(observer, current_tick_id_);
  mujoco_sim::core::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

EngineResult<mujoco_sim::core::UnregisterAdapterResponse>
SimulationCore::Impl::DoUnregisterAdapter(
    const mujoco_sim::core::UnregisterAdapterRequest& request) {
  if (request.role() == mujoco_sim::core::CONTROLLER) {
    return UnregisterControlAdapter(request);
  }
  if (request.role() != OBSERVER) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                INVALID_ARGUMENT,
                "adapter role must be CONTROLLER or OBSERVER"),
            {}};
  }
  const Observer* observer =
      adapter_registry_->FindObserver(request.adapter_id());
  if (observer == nullptr) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                NOT_FOUND, "observer is not registered"),
            {}};
  }
  if (request.session_id() != observer->session_id) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                INVALID_SESSION, "session mismatch"),
            {}};
  }
  if (request.epoch() != epoch_) {
    return {ErrorResponse<mujoco_sim::core::UnregisterAdapterResponse>(
                INVALID_EPOCH, "epoch mismatch"),
            {}};
  }
  adapter_registry_->EraseObserver(request.adapter_id());
  mujoco_sim::core::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return {std::move(response), {}};
}

void SimulationCore::Impl::DoAcceptHeartbeat(
    const mujoco_sim::core::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  if (adapter_registry_->FindControllerBySession(
          heartbeat.session.session_id) != nullptr) {
    AcceptControllerHeartbeat(heartbeat, now);
    return;
  }
  AcceptObserverHeartbeat(heartbeat, now);
}

void SimulationCore::Impl::AcceptControllerHeartbeat(
    const mujoco_sim::core::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  Controller* controller = adapter_registry_->FindControllerBySession(
      heartbeat.session.session_id);
  if (controller == nullptr) return;
  if (heartbeat.session.epoch == epoch_ && controller->last_tick_sent &&
      heartbeat.last_seen <= *controller->last_tick_sent) {
    controller->last_presence_at = now;
    controller->last_heartbeat_tick =
        std::max(controller->last_heartbeat_tick.value_or(0),
                 heartbeat.last_seen);
  }
}

void SimulationCore::Impl::AcceptObserverHeartbeat(
    const mujoco_sim::core::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  Observer* observer = adapter_registry_->FindObserverBySession(
      heartbeat.session.session_id);
  if (observer == nullptr) return;
  if (observer->phase == ObserverPhase::kActive &&
      heartbeat.session.epoch == epoch_ && heartbeat.last_seen <= observer->sequence) {
    observer->last_heartbeat_at = now;
  }
}


}  // namespace mujoco_sim::core
