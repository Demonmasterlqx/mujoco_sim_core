#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace mujoco_sim::sensor_demo {

struct PlanarPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
};

struct WaypointControllerConfig {
  double wheel_radius = 0.18;
  double track_width = 0.70;
  double target_speed = 1.0;
  double max_wheel_angular_speed = 12.0;
  double heading_gain = 2.5;
  double max_yaw_rate = 1.8;
  double waypoint_tolerance = 0.25;
  double corner_slowdown_distance = 0.75;
  bool stop_after_lap = true;
  PlanarPose initial_pose{};

  // Local-frame perimeter matching the usable loop in scene.xml.
  std::vector<Waypoint> waypoints{
      {0.0, 0.0}, {9.35, 0.0}, {9.35, 5.5}, {0.0, 5.5}};
};

// Wheel positions are ordered front-left, front-right, rear-left, rear-right.
// The quaternion uses w, x, y, z order and the gyro uses x, y, z order.
struct VehicleSample {
  double simulation_time = 0.0;
  std::array<double, 4> wheel_positions{};
  std::array<double, 4> imu_quaternion{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> imu_gyro{};
};

struct WaypointControllerOutput {
  std::array<double, 4> wheel_angular_velocities{};
  PlanarPose pose{};
  std::size_t target_waypoint = 0;
  std::size_t completed_laps = 0;
  bool lap_complete = false;
};

class WaypointControllerError : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

class WaypointController {
 public:
  explicit WaypointController(WaypointControllerConfig config = {});

  // Invalid or non-monotonic samples throw without changing controller state.
  WaypointControllerOutput Update(const VehicleSample& sample);
  void Reset();

  [[nodiscard]] const WaypointControllerConfig& config() const noexcept;
  [[nodiscard]] const PlanarPose& pose() const noexcept;
  [[nodiscard]] std::size_t target_waypoint() const noexcept;
  [[nodiscard]] std::size_t completed_laps() const noexcept;
  [[nodiscard]] bool initialized() const noexcept;

 private:
  WaypointControllerConfig config_;
  PlanarPose pose_{};
  std::array<double, 4> previous_wheel_positions_{};
  double previous_time_ = 0.0;
  double previous_imu_yaw_ = 0.0;
  double imu_to_pose_yaw_offset_ = 0.0;
  std::size_t target_waypoint_ = 0;
  std::size_t completed_laps_ = 0;
  bool initialized_ = false;
  bool start_waypoint_seen_ = false;
};

}  // namespace mujoco_sim::sensor_demo
