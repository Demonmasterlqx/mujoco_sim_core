#include "viewer_input_state.hpp"

#include <algorithm>

namespace mujoco_sim::server {

ViewerInputState::ViewerInputState(std::chrono::milliseconds wrench_timeout)
    : wrench_timeout_(wrench_timeout) {}

void ViewerInputState::Merge(const SimViewer::ViewerInputBatch& input) {
  for (const auto& write : input.actuator_writes) {
    const auto found = std::ranges::find(pending_control_writes_, write.actuator_id,
                                         &mujoco_sim::core::InteractiveActuatorWrite::actuator_id);
    if (found == pending_control_writes_.end()) {
      pending_control_writes_.push_back(write);
    } else {
      found->value = write.value;
    }
  }
  if (input.wrench_frame) {
    active_body_wrenches_ = input.wrench_frame->body_wrenches;
    last_wrench_update_ = input.wrench_frame->generated_at;
    wrench_state_received_ = true;
    wrench_dirty_ = true;
    wrench_timed_out_ = false;
  }
  if (input.simulation_actions.running.has_value()) {
    pending_running_ = input.simulation_actions.running;
  }
  reset_pending_ = reset_pending_ || input.simulation_actions.reset_requested;
}

ViewerInputState::PreparedStep ViewerInputState::PrepareStep(
    std::chrono::steady_clock::time_point now) {
  PreparedStep prepared;
  if (wrench_state_received_ && !active_body_wrenches_.empty() && !wrench_timed_out_ &&
      now > last_wrench_update_ && now - last_wrench_update_ > wrench_timeout_) {
    prepared.wrench_expired = true;
    prepared.expired_body_count = active_body_wrenches_.size();
    prepared.wrench_age =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_wrench_update_);
    active_body_wrenches_.clear();
    wrench_dirty_ = true;
    wrench_timed_out_ = true;
  }
  prepared.input = {
      .actuator_writes = pending_control_writes_,
      .body_wrenches = active_body_wrenches_,
      .replace_body_wrenches = wrench_dirty_,
  };
  return prepared;
}

void ViewerInputState::CommitCompletedStep() noexcept {
  pending_control_writes_.clear();
  wrench_dirty_ = false;
}

ViewerInputState::PendingSimulationActions ViewerInputState::TakeSimulationActions() noexcept {
  PendingSimulationActions output{
      .running = pending_running_,
      .reset_requested = reset_pending_,
  };
  pending_running_.reset();
  reset_pending_ = false;
  return output;
}

void ViewerInputState::ResetAfterSimulationReset() noexcept {
  pending_control_writes_.clear();
  active_body_wrenches_.clear();
  last_wrench_update_ = {};
  wrench_state_received_ = false;
  wrench_dirty_ = false;
  wrench_timed_out_ = false;
}

}  // namespace mujoco_sim::server
