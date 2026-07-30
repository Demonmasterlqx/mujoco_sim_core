#include <mujoco_sim/core/simulation_core.hpp>

#include <vector>

int main() {
  mujoco_sim::core::SimulationCore core;
  const std::vector<mujoco_sim::core::InteractiveActuatorWrite> writes = {
      {.actuator_id = 0, .value = 0.0},
  };
  const auto effects = core.Step({.actuator_writes = writes});
  return core.IsModelLoaded() || effects.step_completed.has_value() ? 1 : 0;
}
