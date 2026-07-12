#pragma once

namespace mujoco_sim::sensor_demo {

void InstallProcessSignalHandlers();
bool ProcessStopRequested() noexcept;

}  // namespace mujoco_sim::sensor_demo
