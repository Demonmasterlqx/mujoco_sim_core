#include "status_projection.hpp"

#include "adapter_registry.hpp"
#include "barrier_coordinator.hpp"
#include "controller_state_machine.hpp"

namespace mujoco_sim::core::internal {

mujoco_sim::core::ServerStatus StatusProjection::Build(
    std::uint64_t epoch, std::uint64_t tick_id, double simulation_time_s,
    mujoco_sim::core::ServerState server_state, const std::string& message,
    const AdapterRegistry& registry,
    const BarrierCoordinator& barrier) const {
  mujoco_sim::core::ServerStatus status;
  status.epoch = epoch;
  status.tick_id = tick_id;
  status.simulation_time_s = simulation_time_s;
  status.state = server_state;
  status.message = message;
  for (const ControllerRecord* controller : registry.Controllers()) {
    auto& adapter = status.adapters.emplace_back();
    adapter.adapter_id = controller->id;
    adapter.role = mujoco_sim::core::CONTROLLER;
    adapter.session_id = controller->session_id;
    adapter.state = WireState(controller->phase);
    adapter.message = controller->message;
    adapter.state_since_tick_id = controller->state_since_tick;
    adapter.timeout_count = controller->timeout_count;
    adapter.last_heartbeat_tick_id = controller->last_heartbeat_tick;
    if (controller->last_tick_sent) {
      adapter.last_delivery_tick_id = *controller->last_tick_sent;
    }
    if (controller->last_commit) {
      adapter.last_commit_tick_id = controller->last_commit->identity.tick_id;
    }
    adapter.required_for_barrier = barrier.IsRequired(controller->id);
  }
  for (const ObserverRecord* observer : registry.Observers()) {
    auto& adapter = status.adapters.emplace_back();
    adapter.adapter_id = observer->id;
    adapter.role = mujoco_sim::core::OBSERVER;
    adapter.session_id = observer->session_id;
    adapter.state = WireState(observer->phase);
    adapter.message = observer->message;
    adapter.state_since_tick_id = observer->state_since_tick;
    adapter.timeout_count = observer->timeout_count;
    if (observer->last_delivery_tick) {
      adapter.last_delivery_tick_id = *observer->last_delivery_tick;
    }
    adapter.last_observation_sequence = observer->sequence;
  }
  return status;
}

}  // namespace mujoco_sim::core::internal
