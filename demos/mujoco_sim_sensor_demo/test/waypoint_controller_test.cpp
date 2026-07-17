#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>

#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>

namespace demo = mujoco_sim::sensor_demo;
namespace {

demo::VehicleSample MakeSample(double time, const std::array<double, 4>& wheels, double yaw,
                               double yaw_rate = 0.0) {
  demo::VehicleSample sample;
  sample.simulation_time = time;
  sample.wheel_positions = wheels;
  sample.imu_quaternion = {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
  sample.imu_gyro = {0.0, 0.0, yaw_rate};
  return sample;
}

struct RectangleRun {
  std::size_t steps = 0;
  demo::PlanarPose pose{};
};

RectangleRun RunRectangle() {
  demo::WaypointControllerConfig config;
  config.target_speed = 0.9;
  config.heading_gain = 3.0;
  config.max_yaw_rate = 2.0;
  config.waypoint_tolerance = 0.12;
  config.corner_slowdown_distance = 0.45;
  config.waypoints = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {0.0, 1.5}};
  demo::WaypointController controller(config);

  constexpr double kDt = 0.01;
  std::array<double, 4> wheels{};
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;

  for (std::size_t step = 0; step < 20000; ++step) {
    const auto output = controller.Update(MakeSample(step * kDt, wheels, yaw, yaw_rate));
    for (double command : output.wheel_angular_velocities) {
      EXPECT_TRUE(std::isfinite(command));
      EXPECT_LE(std::abs(command), config.max_wheel_angular_speed);
    }
    EXPECT_DOUBLE_EQ(output.wheel_angular_velocities[0], output.wheel_angular_velocities[2]);
    EXPECT_DOUBLE_EQ(output.wheel_angular_velocities[1], output.wheel_angular_velocities[3]);

    if (output.lap_complete) {
      EXPECT_EQ(output.completed_laps, 1U);
      EXPECT_EQ(output.wheel_angular_velocities, (std::array<double, 4>{}));
      return {step + 1, output.pose};
    }

    const double left =
        0.5 * (output.wheel_angular_velocities[0] + output.wheel_angular_velocities[2]);
    const double right =
        0.5 * (output.wheel_angular_velocities[1] + output.wheel_angular_velocities[3]);
    const double linear_velocity = 0.5 * config.wheel_radius * (left + right);
    yaw_rate = config.wheel_radius * (right - left) / config.track_width;
    const double next_yaw = yaw + yaw_rate * kDt;
    const double distance = linear_velocity * kDt;
    x += distance * std::cos(0.5 * (yaw + next_yaw));
    y += distance * std::sin(0.5 * (yaw + next_yaw));
    yaw = next_yaw;
    for (std::size_t index = 0; index < wheels.size(); ++index) {
      wheels[index] += output.wheel_angular_velocities[index] * kDt;
    }
  }

  ADD_FAILURE() << "controller did not complete the rectangle";
  return {0, {x, y, yaw}};
}

}  // namespace

TEST(WaypointControllerTest, DefaultsMatchSkidSteerDemoContract) {
  const demo::WaypointController controller;
  EXPECT_DOUBLE_EQ(controller.config().wheel_radius, 0.18);
  EXPECT_DOUBLE_EQ(controller.config().track_width, 0.70);
  EXPECT_DOUBLE_EQ(controller.config().target_speed, 1.0);
  EXPECT_DOUBLE_EQ(controller.config().max_wheel_angular_speed, 12.0);
  ASSERT_EQ(controller.config().waypoints.size(), 4U);
  EXPECT_FALSE(controller.initialized());
}

TEST(WaypointControllerTest, GyroPredictionUnwrapsQuaternionYaw) {
  demo::WaypointControllerConfig config;
  config.initial_pose.yaw = 3.10;
  demo::WaypointController controller(config);
  controller.Update(MakeSample(0.0, {}, 3.10));

  const std::array<double, 4> wheels{1.0, 1.0, 1.0, 1.0};
  const auto output = controller.Update(
      MakeSample(0.1, wheels, -3.10, (-3.10 + 2.0 * std::numbers::pi - 3.10) / 0.1));

  EXPECT_GT(output.pose.yaw, std::numbers::pi);
  EXPECT_NEAR(output.pose.yaw, -3.10 + 2.0 * std::numbers::pi, 1e-12);
  EXPECT_NEAR(output.pose.x, -config.wheel_radius, 1e-6);
  EXPECT_NEAR(output.pose.y, 0.0, 1e-6);
}

TEST(WaypointControllerTest, InvalidSampleDoesNotAdvanceAnyState) {
  demo::WaypointController controller;
  controller.Update(MakeSample(1.0, {10.0, 10.0, 10.0, 10.0}, 0.0));
  const std::size_t target_before = controller.target_waypoint();

  auto bad = MakeSample(2.0, {11.0, 11.0, 11.0, 11.0}, 0.0);
  bad.wheel_positions[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(controller.Update(bad), demo::WaypointControllerError);
  EXPECT_EQ(controller.target_waypoint(), target_before);
  EXPECT_DOUBLE_EQ(controller.pose().x, 0.0);

  bad = MakeSample(2.0, {11.0, 11.0, 11.0, 11.0}, 0.0);
  bad.imu_quaternion = {};
  EXPECT_THROW(controller.Update(bad), demo::WaypointControllerError);
  EXPECT_EQ(controller.target_waypoint(), target_before);

  const auto accepted = controller.Update(MakeSample(2.0, {11.0, 11.0, 11.0, 11.0}, 0.0));
  EXPECT_NEAR(accepted.pose.x, 0.18, 1e-12);
  EXPECT_NEAR(accepted.pose.y, 0.0, 1e-12);
}

TEST(WaypointControllerTest, DeterministicallyCompletesRectangleAndStops) {
  const RectangleRun first = RunRectangle();
  const RectangleRun second = RunRectangle();

  ASSERT_GT(first.steps, 0U);
  EXPECT_EQ(first.steps, second.steps);
  EXPECT_DOUBLE_EQ(first.pose.x, second.pose.x);
  EXPECT_DOUBLE_EQ(first.pose.y, second.pose.y);
  EXPECT_DOUBLE_EQ(first.pose.yaw, second.pose.yaw);
  EXPECT_LE(std::hypot(first.pose.x, first.pose.y), 0.12);
}
