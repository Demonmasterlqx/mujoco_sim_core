#include <mujoco_sim/wbr_lqr_demo/controller.hpp>

int main() {
  mujoco_sim::wbr_lqr_demo::AdapterConfig config;
  const auto options = mujoco_sim::wbr_lqr_demo::MakeControllerOptions(config);
  return options.observation_names.size() == 21 && options.command_names.size() == 6 ? 0 : 1;
}
