#include "adapter_registration.hpp"

#include <utility>
#include <vector>

namespace mujoco_sim::server {

namespace {

auto SetupError(std::string message) {
  mujoco_sim::core::EngineResult<mujoco_sim::core::RegisterAdapterResponse> result;
  result.response.set_code(mujoco_sim::core::INTERNAL_ERROR);
  result.response.set_message(message.empty() ? "failed to initialize adapter runtime resources"
                                              : std::move(message));
  return result;
}

}  // namespace

mujoco_sim::core::EngineResult<mujoco_sim::core::RegisterAdapterResponse> RegisterAdapterAtomically(
    mujoco_sim::core::SimulationCore* core, ControllerCommandInbox* command_inbox,
    const mujoco_sim::core::RegisterAdapterRequest& request,
    std::chrono::steady_clock::time_point now, const PreparePublisher& prepare_publisher,
    const RollbackPublisher& rollback_publisher) {
  if (request.role() != mujoco_sim::core::CONTROLLER &&
      request.role() != mujoco_sim::core::OBSERVER) {
    auto result = SetupError("unsupported adapter role");
    result.response.set_code(mujoco_sim::core::INVALID_ARGUMENT);
    return result;
  }
  auto publisher = prepare_publisher(request.adapter_id(), request.role());
  if (!publisher.ready) {
    return SetupError(std::move(publisher.error));
  }

  auto reservation =
      command_inbox->PrepareRegistration(request.role() == mujoco_sim::core::CONTROLLER);
  if (!reservation) {
    if (publisher.created) {
      rollback_publisher(request.adapter_id(), request.role());
    }
    return SetupError("controller command inbox is closed");
  }

  auto result = core->RegisterAdapter(request, now);
  if (result.response.code() != mujoco_sim::core::OK) {
    if (publisher.created) {
      rollback_publisher(request.adapter_id(), request.role());
    }
    return result;
  }

  std::vector<std::uint64_t> replaced_sessions;
  for (const auto& change : result.effects.adapter_changes) {
    if (change.new_state == mujoco_sim::core::UNREGISTERED) {
      replaced_sessions.push_back(change.session_id);
    }
  }
  if (!reservation.Commit(result.response.session_id(), result.response.epoch(),
                          replaced_sessions)) {
    // Commit is allocation-free and Close() cannot run while the reservation
    // is held. Reaching this branch indicates an internal invariant failure.
    return SetupError("failed to commit adapter runtime resources");
  }
  return result;
}

}  // namespace mujoco_sim::server
