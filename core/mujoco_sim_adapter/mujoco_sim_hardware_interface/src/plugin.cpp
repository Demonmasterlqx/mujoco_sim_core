#include <mujoco_sim_hardware_interface/mujoco_sim_system.hpp>

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mujoco_sim::hardware::MujocoSimSystem, hardware_interface::SystemInterface)
