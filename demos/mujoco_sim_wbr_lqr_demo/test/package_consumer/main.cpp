#include <mujoco_sim/wbr_lqr_demo/tick_controller.hpp>

int main() {
  mujoco_sim::wbr_lqr_demo::AdapterConfig config;
  const auto options = mujoco_sim::wbr_lqr_demo::MakeAdapterOptions(config);
  return options.observation_names.size() == 21 &&
                 options.command_names.size() == 6
             ? 0
             : 1;
}
