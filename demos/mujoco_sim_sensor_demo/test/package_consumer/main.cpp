#include <string>

#include <mujoco_sim/sensor_demo/imu_synchronizer.hpp>
#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>

int main() {
  std::string error;
  mujoco_sim::sensor_demo::ImuSynchronizer synchronizer(2);
  if (!synchronizer.Add(
          {0.0, Eigen::Quaterniond::Identity()}, &error)) {
    return 1;
  }
  mujoco_sim::sensor_demo::WaypointController controller;
  return controller.config().waypoints.size() == 4 ? 0 : 2;
}
