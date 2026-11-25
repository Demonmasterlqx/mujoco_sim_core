#include <mujoco_sim/core/simulation_core.hpp>

#include <vector>

int main() {
  mujoco_sim::core::SimulationCore core;
  const std::vector<mujoco_sim::core::InteractiveActuatorWrite> writes = {
      {.actuator_id = 0, .value = 0.0},
  };
  const auto effects = core.Step({.actuator_writes = writes});
  const auto pause = core.SetPaused(true);
  const auto reset = core.Reset({});
  const bool paused_value_is_stable = static_cast<int>(mujoco_sim::core::PAUSED) == 5;
  return core.IsModelLoaded() || effects.step_completed.has_value() || pause.response.applied ||
                 reset.response.applied || !paused_value_is_stable
             ? 1
             : 0;
}
