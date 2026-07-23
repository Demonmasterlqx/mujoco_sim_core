#include <mujoco_sim/core/simulation_core.hpp>

int main() {
  mujoco_sim::core::SimulationCore core;
  return core.IsModelLoaded() ? 1 : 0;
}
