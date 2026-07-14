#include "status_projection.hpp"

#include "adapter_registry.hpp"
#include "barrier_coordinator.hpp"
#include "controller_state_machine.hpp"

namespace mujoco_sim::core::internal {

mujoco_sim::v2::ServerStatus StatusProjection::Build(
    const mujoco_sim::v2::Header& header,
    mujoco_sim::v2::ServerState server_state, const std::string& message,
    const AdapterRegistry& registry,
    const BarrierCoordinator& barrier) const {
  mujoco_sim::v2::ServerStatus status;
  *status.mutable_header() = header;
  status.set_state(server_state);
  status.set_message(message);
  for (const ControllerRecord* controller : registry.Controllers()) {
    auto* adapter = status.add_adapters();
    adapter->set_adapter_id(controller->id);
    adapter->set_role(mujoco_sim::v2::CONTROLLER);
    adapter->set_session_id(controller->session_id);
    adapter->set_state(WireState(controller->phase));
    adapter->set_message(controller->message);
    adapter->set_state_since_tick_id(controller->state_since_tick);
    adapter->set_timeout_count(controller->timeout_count);
    adapter->set_last_heartbeat_tick_id(controller->last_heartbeat_tick);
    if (controller->last_tick_sent) {
      adapter->set_last_delivery_tick_id(*controller->last_tick_sent);
    }
    if (controller->last_commit) {
      adapter->set_last_commit_tick_id(
          controller->last_commit->header().tick_id());
    }
    adapter->set_required_for_barrier(barrier.IsRequired(controller->id));
  }
  for (const ObserverRecord* observer : registry.Observers()) {
    auto* adapter = status.add_adapters();
    adapter->set_adapter_id(observer->id);
    adapter->set_role(mujoco_sim::v2::OBSERVER);
    adapter->set_session_id(observer->session_id);
    adapter->set_state(WireState(observer->phase));
    adapter->set_message(observer->message);
    adapter->set_state_since_tick_id(observer->state_since_tick);
    adapter->set_timeout_count(observer->timeout_count);
    if (observer->last_delivery_tick) {
      adapter->set_last_delivery_tick_id(*observer->last_delivery_tick);
    }
    adapter->set_last_observation_sequence(observer->sequence);
  }
  return status;
}

}  // namespace mujoco_sim::core::internal
