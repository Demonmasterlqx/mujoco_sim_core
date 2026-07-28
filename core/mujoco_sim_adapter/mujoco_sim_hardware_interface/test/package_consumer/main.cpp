#include <mujoco_sim_hardware_interface/mujoco_sim_system.hpp>

#include <memory>
#include <type_traits>

#include <hardware_interface/system_interface.hpp>

int main() {
  static_assert(std::is_base_of_v<hardware_interface::SystemInterface,
                                  mujoco_sim::hardware::MujocoSimSystem>);
  static_assert(!std::is_copy_constructible_v<mujoco_sim::hardware::MujocoSimSystem>);
  auto system = std::make_unique<mujoco_sim::hardware::MujocoSimSystem>();
  return system == nullptr ? 1 : 0;
}
