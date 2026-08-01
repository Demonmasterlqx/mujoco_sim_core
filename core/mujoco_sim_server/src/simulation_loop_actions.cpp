#include "simulation_loop.hpp"

#include <spdlog/logger.h>

#include <chrono>
#include <utility>

#include "sensor_runtime.hpp"
#include "server_logging.hpp"
#include "sim_viewer.hpp"

namespace mujoco_sim::server {

void SimulationLoop::HandleViewerSimulationActions(std::chrono::steady_clock::time_point now) {
  auto actions = viewer_input_state_.TakeSimulationActions();
  if (actions.reset_requested) {
    auto result = core_.Reset(now);
    ApplyEffects(std::move(result.effects));
    if (viewer_ != nullptr) viewer_->CompleteSimulationResetAction();
    if (result.response.applied) {
      viewer_input_state_.ResetAfterSimulationReset();
      sensor_runtime_->OnSimulationReset(core_.state());
      ResetRuntimeAnchorsAfterSimulationReset(now);
      const auto state = core_.state();
      logger_->info(
          "event=simulation_reset source=local_gui tick_id={} simulation_time_s={} "
          "server_state={}",
          state.current_tick_id, state.simulation_time_s, static_cast<int>(state.server_state));
    } else {
      logger_->warn("event=simulation_reset_rejected source=local_gui reason={}",
                    LogValue(result.response.message));
    }
  }

  if (!actions.running.has_value()) return;
  const auto before = core_.state().server_state;
  auto result = core_.SetPaused(!*actions.running);
  ApplyEffects(std::move(result.effects));
  const auto after = core_.state().server_state;
  if (result.response.applied && before == mujoco_sim::core::PAUSED &&
      after == mujoco_sim::core::RUNNING) {
    next_step_ = now;
  }
  if (result.response.state_changed) {
    logger_->info("event={} source=local_gui tick_id={} simulation_time_s={}",
                  after == mujoco_sim::core::PAUSED ? "simulation_paused" : "simulation_resumed",
                  core_.state().current_tick_id, core_.state().simulation_time_s);
  } else if (!result.response.applied) {
    logger_->warn("event=simulation_pause_control_rejected source=local_gui reason={}",
                  LogValue(result.response.message));
    if (viewer_ != nullptr) {
      viewer_->RequestRunningIndicator(after == mujoco_sim::core::RUNNING);
    }
  }
}

void SimulationLoop::ResetRuntimeAnchorsAfterSimulationReset(
    std::chrono::steady_clock::time_point now) {
  const auto state = core_.state();
  summary_wall_anchor_ = now;
  summary_simulation_anchor_ = state.simulation_time_s;
  summary_tick_anchor_ = state.current_tick_id;
  next_step_ = now;
  next_viewer_capture_ = now;
  next_summary_ = config_.log_summary_period_ms
                      ? now + std::chrono::milliseconds(config_.log_summary_period_ms)
                      : std::chrono::steady_clock::time_point::max();
}

}  // namespace mujoco_sim::server
