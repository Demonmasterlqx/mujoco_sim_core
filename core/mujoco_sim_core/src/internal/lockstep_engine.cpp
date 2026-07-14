#include "simulation_core_impl.hpp"

#include <chrono>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v2::ERROR_PAUSED;
using mujoco_sim::v2::RUNNING;
using internal::ControllerPhase;
using internal::ControllerStateMachine;
using internal::ObserverPhase;
using internal::WireState;

std::uint64_t NowUnixNanos() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

}  // namespace

std::vector<IssuedTickHandle> SimulationCore::Impl::DoOpenIssuedTicks() {
  std::vector<IssuedTickHandle> ticks;
  if (!IsModelLoaded() || server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
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
        adapter_id, BuildTickFor(*controller),
        controller->layout.ByteSizeLong(), controller->session_id, epoch_,
        current_tick_id_);
    if (!issued) {
      EnterErrorPaused("failed to serialize issued Tick");
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

mujoco_sim::v2::Tick SimulationCore::Impl::BuildTickFor(
    const Controller& controller) const {
  mujoco_sim::v2::Tick tick;
  *tick.mutable_header() = MakeHeader(controller.session_id);
  tick.set_adapter_id(controller.id);
  tick.set_controller_state(WireState(controller.phase));
  tick.set_layout_hash(controller.layout.layout_hash());

  for (const auto& observation : controller.layout.observations()) {
    auto* block = tick.add_observations();
    block->set_name(observation.name());
    const int sensor_id = observation.mujoco_id();
    const int address = model_state_->model_ptr()->sensor_adr[sensor_id];
    const int dimension = model_state_->model_ptr()->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      block->add_values(static_cast<double>(
          model_state_->data_ptr()->sensordata[address + i]));
    }
  }

  return tick;
}

std::vector<mujoco_sim::v2::ObservationFrame>
SimulationCore::Impl::BuildObservationFramesForCompletedStep() {
  std::vector<mujoco_sim::v2::ObservationFrame> frames;
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

mujoco_sim::v2::ObservationFrame SimulationCore::Impl::BuildObservationFrame(
    Observer* observer) const {
  mujoco_sim::v2::ObservationFrame frame;
  *frame.mutable_header() = MakeHeader(observer->session_id);
  frame.mutable_header()->set_protocol_minor(observer->protocol_minor);
  frame.set_adapter_id(observer->id);
  frame.set_sequence(++observer->sequence);
  for (const auto& observation : observer->layout.observations()) {
    auto* block = frame.add_observations();
    block->set_name(observation.name());
    const int sensor_id = observation.mujoco_id();
    const int address = model_state_->model_ptr()->sensor_adr[sensor_id];
    const int dimension = model_state_->model_ptr()->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      block->add_values(static_cast<double>(
          model_state_->data_ptr()->sensordata[address + i]));
    }
  }
  return frame;
}

mujoco_sim::v2::Header SimulationCore::Impl::MakeHeader(
    std::uint64_t session_id) const {
  mujoco_sim::v2::Header header;
  header.set_server_name(options_.server_name);
  header.set_session_id(session_id);
  header.set_epoch(epoch_);
  header.set_tick_id(current_tick_id_);
  header.set_timestamp_unix_nanos(NowUnixNanos());
  header.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  header.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  header.set_simulation_time_s(model_state_->simulation_time());
  return header;
}

bool SimulationCore::Impl::ValidateCommonHeader(
    const mujoco_sim::v2::Header& header, const Controller& controller,
    std::string* message) const {
  if (header.server_name() != options_.server_name) {
    *message = "server name mismatch";
    return false;
  }
  if (header.session_id() != controller.session_id) {
    *message = "session mismatch";
    return false;
  }
  if (header.epoch() != epoch_) {
    *message = "epoch mismatch";
    return false;
  }
  if (header.tick_id() != current_tick_id_) {
    *message = "tick mismatch";
    return false;
  }
  return true;
}

std::optional<StepCompletedEvent> SimulationCore::Impl::CompleteBarrier(
    StepCompletionSource source) {
  if (!tick_open_ || !barrier_coordinator_->Ready()) {
    return std::nullopt;
  }
  model_state_->ApplyActuatorWrites(
      barrier_coordinator_->BuildWritePlan(*resource_catalog_));
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
