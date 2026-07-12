#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace mujoco_sim::sensor_demo {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kMinimumQuaternionNorm = 1e-12;

void RequireFinite(double value, const std::string& field) {
  if (!std::isfinite(value)) {
    throw WaypointControllerError("non-finite " + field);
  }
}

void ValidateConfig(const WaypointControllerConfig& config) {
  RequireFinite(config.wheel_radius, "wheel_radius");
  RequireFinite(config.track_width, "track_width");
  RequireFinite(config.target_speed, "target_speed");
  RequireFinite(config.max_wheel_angular_speed,
                "max_wheel_angular_speed");
  RequireFinite(config.heading_gain, "heading_gain");
  RequireFinite(config.max_yaw_rate, "max_yaw_rate");
  RequireFinite(config.waypoint_tolerance, "waypoint_tolerance");
  RequireFinite(config.corner_slowdown_distance,
                "corner_slowdown_distance");
  RequireFinite(config.initial_pose.x, "initial_pose.x");
  RequireFinite(config.initial_pose.y, "initial_pose.y");
  RequireFinite(config.initial_pose.yaw, "initial_pose.yaw");

  if (config.wheel_radius <= 0.0 || config.track_width <= 0.0 ||
      config.target_speed < 0.0 ||
      config.max_wheel_angular_speed <= 0.0 || config.heading_gain <= 0.0 ||
      config.max_yaw_rate <= 0.0 || config.waypoint_tolerance <= 0.0 ||
      config.corner_slowdown_distance <= 0.0) {
    throw WaypointControllerError("controller dimensions and gains are invalid");
  }
  if (config.waypoints.size() < 3) {
    throw WaypointControllerError("a perimeter requires at least 3 waypoints");
  }
  if (!std::isfinite(config.max_wheel_angular_speed * config.wheel_radius) ||
      !std::isfinite(config.max_yaw_rate * config.track_width) ||
      !std::isfinite(config.heading_gain * std::numbers::pi)) {
    throw WaypointControllerError("controller configuration overflows");
  }

  for (std::size_t index = 0; index < config.waypoints.size(); ++index) {
    const Waypoint& waypoint = config.waypoints[index];
    const Waypoint& next =
        config.waypoints[(index + 1) % config.waypoints.size()];
    RequireFinite(waypoint.x,
                  "waypoints[" + std::to_string(index) + "].x");
    RequireFinite(waypoint.y,
                  "waypoints[" + std::to_string(index) + "].y");
    const double segment_length = std::hypot(next.x - waypoint.x,
                                             next.y - waypoint.y);
    if (!std::isfinite(segment_length) ||
        segment_length <= 2.0 * config.waypoint_tolerance) {
      throw WaypointControllerError(
          "adjacent waypoints must be separated by more than twice the "
          "waypoint tolerance");
    }
  }
}

void ValidateSample(const VehicleSample& sample, bool initialized,
                    double previous_time) {
  RequireFinite(sample.simulation_time, "simulation_time");
  for (std::size_t index = 0; index < sample.wheel_positions.size(); ++index) {
    RequireFinite(sample.wheel_positions[index],
                  "wheel_positions[" + std::to_string(index) + "]");
  }
  for (std::size_t index = 0; index < sample.imu_quaternion.size(); ++index) {
    RequireFinite(sample.imu_quaternion[index],
                  "imu_quaternion[" + std::to_string(index) + "]");
  }
  for (std::size_t index = 0; index < sample.imu_gyro.size(); ++index) {
    RequireFinite(sample.imu_gyro[index],
                  "imu_gyro[" + std::to_string(index) + "]");
  }
  if (initialized && sample.simulation_time <= previous_time) {
    throw WaypointControllerError(
        "simulation_time must increase strictly between samples");
  }
}

double QuaternionYaw(const std::array<double, 4>& quaternion) {
  const double first_pair = std::hypot(quaternion[0], quaternion[1]);
  const double second_pair = std::hypot(quaternion[2], quaternion[3]);
  const double norm = std::hypot(first_pair, second_pair);
  if (!std::isfinite(norm) || norm < kMinimumQuaternionNorm) {
    throw WaypointControllerError("IMU quaternion has near-zero norm");
  }

  const double w = quaternion[0] / norm;
  const double x = quaternion[1] / norm;
  const double y = quaternion[2] / norm;
  const double z = quaternion[3] / norm;
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

double UnwrapNear(double wrapped_angle, double reference_angle) {
  const double difference = reference_angle - wrapped_angle;
  RequireFinite(difference, "IMU yaw difference");
  const double unwrapped =
      wrapped_angle + kTwoPi * std::round(difference / kTwoPi);
  RequireFinite(unwrapped, "unwrapped IMU yaw");
  return unwrapped;
}

double WrapToPi(double angle) {
  return std::remainder(angle, kTwoPi);
}

bool Reached(const PlanarPose& pose, const Waypoint& waypoint,
             double tolerance) {
  return std::hypot(waypoint.x - pose.x, waypoint.y - pose.y) <= tolerance;
}

std::array<double, 4> BuildWheelCommand(
    const WaypointControllerConfig& config, const PlanarPose& pose,
    const Waypoint& target, bool stopped) {
  if (stopped) {
    return {};
  }

  const double delta_x = target.x - pose.x;
  const double delta_y = target.y - pose.y;
  const double distance = std::hypot(delta_x, delta_y);
  const double target_heading = std::atan2(delta_y, delta_x);
  const double heading_error = WrapToPi(target_heading - pose.yaw);
  const double alignment = std::max(0.0, std::cos(heading_error));
  const double slowdown =
      std::clamp(distance / config.corner_slowdown_distance, 0.0, 1.0);
  const double linear_velocity = config.target_speed * alignment * slowdown;
  const double yaw_rate =
      std::clamp(config.heading_gain * heading_error, -config.max_yaw_rate,
                 config.max_yaw_rate);

  const double left =
      (linear_velocity - 0.5 * config.track_width * yaw_rate) /
      config.wheel_radius;
  const double right =
      (linear_velocity + 0.5 * config.track_width * yaw_rate) /
      config.wheel_radius;
  RequireFinite(left, "left wheel command");
  RequireFinite(right, "right wheel command");

  const double largest = std::max(std::abs(left), std::abs(right));
  const double scale =
      largest > config.max_wheel_angular_speed
          ? config.max_wheel_angular_speed / largest
          : 1.0;
  return {left * scale, right * scale, left * scale, right * scale};
}

}  // namespace

WaypointController::WaypointController(WaypointControllerConfig config)
    : config_(std::move(config)) {
  ValidateConfig(config_);
  Reset();
}

WaypointControllerOutput WaypointController::Update(
    const VehicleSample& sample) {
  ValidateSample(sample, initialized_, previous_time_);
  const double wrapped_imu_yaw = QuaternionYaw(sample.imu_quaternion);

  PlanarPose next_pose = pose_;
  double next_imu_yaw = wrapped_imu_yaw;
  double next_yaw_offset = imu_to_pose_yaw_offset_;
  if (!initialized_) {
    next_pose = config_.initial_pose;
    next_yaw_offset = next_pose.yaw - wrapped_imu_yaw;
  } else {
    const double delta_time = sample.simulation_time - previous_time_;
    RequireFinite(delta_time, "sample delta time");
    const double predicted_imu_yaw =
        previous_imu_yaw_ + sample.imu_gyro[2] * delta_time;
    RequireFinite(predicted_imu_yaw, "gyro-predicted IMU yaw");
    next_imu_yaw = UnwrapNear(wrapped_imu_yaw, predicted_imu_yaw);

    const double left_wheel_delta =
        0.5 * ((sample.wheel_positions[0] -
                previous_wheel_positions_[0]) +
               (sample.wheel_positions[2] -
                previous_wheel_positions_[2]));
    const double right_wheel_delta =
        0.5 * ((sample.wheel_positions[1] -
                previous_wheel_positions_[1]) +
               (sample.wheel_positions[3] -
                previous_wheel_positions_[3]));
    const double distance =
        0.5 * config_.wheel_radius * (left_wheel_delta + right_wheel_delta);
    const double next_pose_yaw = next_imu_yaw + next_yaw_offset;
    const double midpoint_yaw = 0.5 * (next_pose.yaw + next_pose_yaw);
    RequireFinite(distance, "wheel odometry distance");
    RequireFinite(next_pose_yaw, "pose yaw");
    RequireFinite(midpoint_yaw, "odometry midpoint yaw");

    next_pose.x += distance * std::cos(midpoint_yaw);
    next_pose.y += distance * std::sin(midpoint_yaw);
    next_pose.yaw = next_pose_yaw;
    RequireFinite(next_pose.x, "pose x");
    RequireFinite(next_pose.y, "pose y");
  }

  std::size_t next_target = target_waypoint_;
  std::size_t next_completed_laps = completed_laps_;
  bool next_start_seen = start_waypoint_seen_;
  for (std::size_t transitions = 0;
       transitions < config_.waypoints.size() &&
       Reached(next_pose, config_.waypoints[next_target],
               config_.waypoint_tolerance);
       ++transitions) {
    if (next_target == 0) {
      if (next_start_seen) {
        ++next_completed_laps;
      } else {
        next_start_seen = true;
      }
    }
    next_target = (next_target + 1) % config_.waypoints.size();
    if (config_.stop_after_lap && next_completed_laps > 0) {
      break;
    }
  }

  const bool lap_complete = next_completed_laps > 0;
  const bool stopped = config_.stop_after_lap && lap_complete;
  const std::array<double, 4> command = BuildWheelCommand(
      config_, next_pose, config_.waypoints[next_target], stopped);

  pose_ = next_pose;
  previous_wheel_positions_ = sample.wheel_positions;
  previous_time_ = sample.simulation_time;
  previous_imu_yaw_ = next_imu_yaw;
  imu_to_pose_yaw_offset_ = next_yaw_offset;
  target_waypoint_ = next_target;
  completed_laps_ = next_completed_laps;
  start_waypoint_seen_ = next_start_seen;
  initialized_ = true;

  return {command, pose_, target_waypoint_, completed_laps_, lap_complete};
}

void WaypointController::Reset() {
  pose_ = config_.initial_pose;
  previous_wheel_positions_ = {};
  previous_time_ = 0.0;
  previous_imu_yaw_ = 0.0;
  imu_to_pose_yaw_offset_ = 0.0;
  target_waypoint_ = 0;
  completed_laps_ = 0;
  initialized_ = false;
  start_waypoint_seen_ = false;
}

const WaypointControllerConfig& WaypointController::config() const noexcept {
  return config_;
}

const PlanarPose& WaypointController::pose() const noexcept { return pose_; }

std::size_t WaypointController::target_waypoint() const noexcept {
  return target_waypoint_;
}

std::size_t WaypointController::completed_laps() const noexcept {
  return completed_laps_;
}

bool WaypointController::initialized() const noexcept { return initialized_; }

}  // namespace mujoco_sim::sensor_demo
