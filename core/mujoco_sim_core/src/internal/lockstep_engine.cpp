#include "simulation_core_impl.hpp"

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::core::ERROR_PAUSED;
using mujoco_sim::core::RUNNING;
using internal::ControllerPhase;
using internal::ControllerStateMachine;
using internal::ObserverPhase;
using internal::WireState;

}  // namespace

std::vector<IssuedTickHandle> SimulationCore::Impl::DoOpenIssuedTicks() {
  std::vector<IssuedTickHandle> ticks;
  if (!IsModelLoaded() || server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::core::STOPPED) {
    return ticks;
  }
  if (!tick_open_ && !OpenNewBarrier()) return ticks;
  server_state_ = RUNNING;
  for (const auto& adapter_id : barrier_coordinator_->RecipientIds()) {
    const Controller* controller =
        adapter_registry_->FindController(adapter_id);
    if (controller == nullptr ||
        !ControllerStateMachine::IsTickRecipient(controller->phase)) {
      continue;
    }
    const auto tick = tick_store_->Find(adapter_id);
    if (tick) ticks.push_back(tick);
  }
  return ticks;
}

bool SimulationCore::Impl::OpenNewBarrier() {
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  for (Controller* controller : adapter_registry_->Controllers()) {
    if (controller->phase == ControllerPhase::kRecoveryPending) {
      ControllerStateMachine::ActivateRecovered(controller, current_tick_id_);
    }
    if (controller->phase == ControllerPhase::kActive) {
      barrier_coordinator_->AddRequiredController(controller->id);
    } else if (ControllerStateMachine::IsTickRecipient(controller->phase)) {
      barrier_coordinator_->AddRecipient(controller->id);
    }
  }
  for (const auto& adapter_id : barrier_coordinator_->RecipientIds()) {
    Controller* controller = adapter_registry_->FindController(adapter_id);
    if (controller == nullptr) continue;
    controller->last_tick_sent = current_tick_id_;
    const auto issued = tick_store_->Insert(
        adapter_id, BuildTickFor(*controller), controller->session_id, epoch_,
        current_tick_id_);
    if (!issued) {
      EnterErrorPaused("failed to create issued Tick");
      return false;
    }
  }
  tick_started_at_ = std::chrono::steady_clock::now();
  tick_open_ = true;
  return true;
}

bool SimulationCore::Impl::IsIssuedTickCurrent(
    const IssuedTickIdentity& identity) const {
  return tick_open_ && tick_store_->IsCurrent(identity);
}

void SimulationCore::Impl::InvalidateIssuedTick(const std::string& adapter_id) {
  tick_store_->Invalidate(adapter_id);
}

void SimulationCore::Impl::InvalidateAllIssuedTicks() {
  tick_store_->InvalidateAll();
}

mujoco_sim::core::Tick SimulationCore::Impl::BuildTickFor(
    const Controller& controller) const {
  mujoco_sim::core::Tick tick;
  tick.identity.session = {controller.session_id, epoch_};
  tick.identity.tick_id = current_tick_id_;
  tick.identity.layout = controller.layout.fingerprint;
  tick.identity.simulation_time_s = model_state_->simulation_time();
  tick.recovery_proof = controller.phase != ControllerPhase::kActive;
  tick.observations.reserve(controller.layout.observation_scalar_count);

  for (const auto& observation : controller.layout.observations()) {
    const int sensor_id = observation.mujoco_id();
    const int address = model_state_->model_ptr()->sensor_adr[sensor_id];
    const int dimension = model_state_->model_ptr()->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      tick.observations.push_back(static_cast<double>(
          model_state_->data_ptr()->sensordata[address + i]));
    }
  }

  return tick;
}

std::vector<mujoco_sim::core::ObservationFrame>
SimulationCore::Impl::BuildObservationFramesForCompletedStep() {
  std::vector<mujoco_sim::core::ObservationFrame> frames;
  for (Observer* observer : adapter_registry_->Observers()) {
    if (observer->phase != ObserverPhase::kActive ||
        current_tick_id_ <= observer->activation_tick) {
      continue;
    }

    if (!observation_scheduler_->Due(*observer, current_tick_id_, timestep())) {
      continue;
    }

    frames.push_back(BuildObservationFrame(observer));
    observation_scheduler_->MarkDelivered(observer, current_tick_id_,
                                           timestep());
  }
  return frames;
}

mujoco_sim::core::ObservationFrame SimulationCore::Impl::BuildObservationFrame(
    Observer* observer) const {
  mujoco_sim::core::ObservationFrame frame;
  frame.identity.session = {observer->session_id, epoch_};
  frame.identity.tick_id = current_tick_id_;
  frame.identity.layout = observer->layout.fingerprint;
  frame.identity.simulation_time_s = model_state_->simulation_time();
  frame.sequence = ++observer->sequence;
  frame.observations.reserve(observer->layout.observation_scalar_count);
  for (const auto& observation : observer->layout.observations()) {
    const int sensor_id = observation.mujoco_id();
    const int address = model_state_->model_ptr()->sensor_adr[sensor_id];
    const int dimension = model_state_->model_ptr()->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      frame.observations.push_back(static_cast<double>(
          model_state_->data_ptr()->sensordata[address + i]));
    }
  }
  return frame;
}

std::optional<StepCompletedEvent> SimulationCore::Impl::CompleteBarrier(
    StepCompletionSource source) {
  if (!tick_open_ || !barrier_coordinator_->Ready()) {
    return std::nullopt;
  }
  model_state_->ApplyActuatorWrites(barrier_coordinator_->BuildWritePlan());
  model_state_->Step();
  ++current_tick_id_;
  StepCompletedEvent event;
  event.completed_tick_id = current_tick_id_ - 1;
  event.next_tick_id = current_tick_id_;
  event.simulation_time_s = model_state_->simulation_time();
  event.source = source;
  event.observer_frames = BuildObservationFramesForCompletedStep();
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  tick_open_ = false;
  return event;
}

std::optional<StepCompletedEvent>
SimulationCore::Impl::DoCompleteEmptyBarrier() {
  return CompleteBarrier(StepCompletionSource::kEmptyBarrier);
}


}  // namespace mujoco_sim::core
