#include <mujoco_sim/core/simulation_core.hpp>

#include <utility>

#include "internal/simulation_core_impl.hpp"

namespace mujoco_sim::core {

EngineResult<SimulationActionReceipt> SimulationCore::SetPaused(bool paused) {
  EngineResult<SimulationActionReceipt> out;
  if (!IsModelLoaded()) {
    out.response.message = "model is not loaded";
    return out;
  }
  if (impl_->server_state != RUNNING && impl_->server_state != PAUSED) {
    out.response.message = "server state does not allow manual pause control";
    return out;
  }

  const ServerState target = paused ? PAUSED : RUNNING;
  out.response.applied = true;
  if (impl_->server_state == target) {
    out.response.message = "simulation state already matches request";
    return out;
  }

  impl_->server_state = target;
  impl_->status_message = paused ? "paused by local GUI" : "";
  out.response.state_changed = true;
  out.response.message = paused ? "simulation paused" : "simulation resumed";
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}

EngineResult<SimulationActionReceipt> SimulationCore::Reset(
    std::chrono::steady_clock::time_point now) {
  EngineResult<SimulationActionReceipt> out;
  if (!IsModelLoaded()) {
    out.response.message = "model is not loaded";
    return out;
  }
  if (impl_->server_state != RUNNING && impl_->server_state != PAUSED) {
    out.response.message = "server state does not allow simulation reset";
    return out;
  }

  impl_->model->ResetToInitialState();
  impl_->queued_policy_writes.clear();
  for (auto& [id, controller] : impl_->controllers) {
    if (controller.state == PROVISIONAL) continue;
    controller.last_command.clear();
    controller.command_dirty = false;
    if (controller.state == ERROR) continue;

    controller.activation_tick = impl_->current_tick;
    controller.next_period = 1;
    controller.pending_policy = ControlPolicy::kZero;
    if (controller.state == OFFLINE) continue;

    controller.last_command_at = now;
    controller.message = "waiting for command after simulation reset";
    if (controller.state == ONLINE) {
      const auto old_state = controller.state;
      controller.state = ACTIVE_NO_COMMAND;
      controller.state_since_tick = impl_->current_tick;
      out.effects.adapter_changes.push_back(
          {AdapterChangeKind::kSimulationReset, controller.id, CONTROLLER, controller.session,
           std::nullopt, old_state, controller.state, "waiting for command after simulation reset",
           impl_->current_tick});
    }
  }

  for (auto& [id, observer] : impl_->observers) {
    observer.activation_tick = impl_->current_tick;
    observer.last_delivery_tick.reset();
    observer.next_period = 1;
  }

  out.response.applied = true;
  out.response.message = "simulation reset to configured initial state";
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}

}  // namespace mujoco_sim::core
