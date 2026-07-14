#include "simulation_core_impl.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <utility>

#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v2::ACTIVE;
using mujoco_sim::v2::ALREADY_EXISTS;
using mujoco_sim::v2::ERROR_PAUSED;
using mujoco_sim::v2::FAILED_PRECONDITION;
using mujoco_sim::v2::INVALID_ARGUMENT;
using mujoco_sim::v2::INVALID_EPOCH;
using mujoco_sim::v2::INVALID_SESSION;
using mujoco_sim::v2::NOT_FOUND;
using mujoco_sim::v2::OBSERVER;
using mujoco_sim::v2::OK;
using mujoco_sim::v2::PROVISIONAL;
using mujoco_sim::v2::RESOURCE_CONFLICT;
using mujoco_sim::v2::TIMED_OUT;
using mujoco_sim::v2::UNREGISTERED;
using mujoco_sim::v2::VERSION_MISMATCH;
using internal::ControllerStateMachine;
using internal::ObserverPhase;
using internal::ObserverStateMachine;
using internal::WireState;

template <typename Response>
Response ErrorResponse(mujoco_sim::v2::ErrorCode code,
                       const std::string& message) {
  Response response;
  response.set_code(code);
  response.set_message(message);
  return response;
}


}  // namespace

mujoco_sim::v2::ActivateAdapterResponse
SimulationCore::Impl::ActivateControlAdapter(
    const mujoco_sim::v2::ActivateAdapterRequest& request) {
  Controller* controller =
      adapter_registry_->FindController(request.adapter_id());
  if (controller == nullptr) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        NOT_FOUND, "controller is not registered");
  }
  if (request.session_id() != controller->session_id) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  if (!ControllerStateMachine::CanActivate(controller->phase)) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "controller is not provisional");
  }
  ControllerStateMachine::Activate(controller, current_tick_id_);
  mujoco_sim::v2::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

EngineResult<mujoco_sim::v2::UnregisterAdapterResponse>
SimulationCore::Impl::UnregisterControlAdapter(
    const mujoco_sim::v2::UnregisterAdapterRequest& request) {
  Controller* controller =
      adapter_registry_->FindController(request.adapter_id());
  if (controller == nullptr) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                NOT_FOUND, "controller is not registered"),
            {}};
  }
  if (request.session_id() != controller->session_id) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_SESSION, "session mismatch"),
            {}};
  }
  if (request.epoch() != epoch_) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
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

  mujoco_sim::v2::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  EngineEffects effects;
  effects.step_completed = std::move(step_completed);
  return {std::move(response), std::move(effects)};
}

mujoco_sim::v2::ActivateAdapterResponse SimulationCore::Impl::DoActivateAdapter(
    const mujoco_sim::v2::ActivateAdapterRequest& request) {
  if (request.role() == mujoco_sim::v2::CONTROLLER) {
    return ActivateControlAdapter(request);
  }
  if (request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  Observer* observer = adapter_registry_->FindObserver(request.adapter_id());
  if (observer == nullptr) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        NOT_FOUND, "observer is not registered");
  }
  if (request.session_id() != observer->session_id) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (observer->phase != ObserverPhase::kProvisional) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "observer is not provisional");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  ObserverStateMachine::Activate(observer, current_tick_id_);
  mujoco_sim::v2::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

EngineResult<mujoco_sim::v2::UnregisterAdapterResponse>
SimulationCore::Impl::DoUnregisterAdapter(
    const mujoco_sim::v2::UnregisterAdapterRequest& request) {
  if (request.role() == mujoco_sim::v2::CONTROLLER) {
    return UnregisterControlAdapter(request);
  }
  if (request.role() != OBSERVER) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_ARGUMENT,
                "adapter role must be CONTROLLER or OBSERVER"),
            {}};
  }
  const Observer* observer =
      adapter_registry_->FindObserver(request.adapter_id());
  if (observer == nullptr) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                NOT_FOUND, "observer is not registered"),
            {}};
  }
  if (request.session_id() != observer->session_id) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_SESSION, "session mismatch"),
            {}};
  }
  if (request.epoch() != epoch_) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_EPOCH, "epoch mismatch"),
            {}};
  }
  adapter_registry_->EraseObserver(request.adapter_id());
  mujoco_sim::v2::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return {std::move(response), {}};
}

void SimulationCore::Impl::DoAcceptHeartbeat(
    const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  if (heartbeat.role() == mujoco_sim::v2::CONTROLLER) {
    AcceptControllerHeartbeat(heartbeat);
    return;
  }
  if (heartbeat.role() == OBSERVER) AcceptObserverHeartbeat(heartbeat, now);
}

void SimulationCore::Impl::AcceptControllerHeartbeat(
    const mujoco_sim::v2::AdapterHeartbeat& heartbeat) {
  Controller* controller =
      adapter_registry_->FindController(heartbeat.adapter_id());
  if (controller == nullptr) return;
  const auto& header = heartbeat.header();
  if (header.server_name() == options_.server_name &&
      header.session_id() == controller->session_id &&
      header.epoch() == epoch_ &&
      header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
      header.protocol_minor() == mujoco_sim::protocol::kProtocolMinor &&
      controller->last_tick_sent &&
      header.tick_id() <= *controller->last_tick_sent) {
    controller->last_heartbeat_tick =
        std::max(controller->last_heartbeat_tick, header.tick_id());
  }
}

void SimulationCore::Impl::AcceptObserverHeartbeat(
    const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  Observer* observer = adapter_registry_->FindObserver(heartbeat.adapter_id());
  if (observer == nullptr) return;
  const auto& header = heartbeat.header();
  if (observer->phase == ObserverPhase::kActive &&
      header.server_name() == options_.server_name &&
      header.session_id() == observer->session_id && header.epoch() == epoch_ &&
      header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
      header.protocol_minor() == mujoco_sim::protocol::kProtocolMinor &&
      heartbeat.last_observation_sequence() <= observer->sequence) {
    observer->last_heartbeat_at = now;
  }
}


}  // namespace mujoco_sim::core
