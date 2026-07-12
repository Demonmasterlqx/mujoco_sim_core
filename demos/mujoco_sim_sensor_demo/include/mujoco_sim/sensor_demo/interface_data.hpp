#pragma once

#include <array>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>
#include <mujoco_sim/v1/sync.pb.h>

namespace mujoco_sim::sensor_demo {

struct ControllerInterfaces {
  std::array<std::string, 4> wheel_positions;
  std::string imu_quaternion;
  std::string imu_angular_velocity;
  std::array<std::string, 4> wheel_velocity_commands;
};

struct StateObserverInterfaces {
  std::array<std::string, 4> wheel_positions;
  std::array<std::string, 4> wheel_velocities;
  std::string imu_quaternion;
  std::string imu_angular_velocity;
  std::string imu_linear_acceleration;
};

struct StateSample {
  double simulation_time = 0.0;
  std::array<double, 4> wheel_positions{};
  std::array<double, 4> wheel_velocities{};
  std::array<double, 4> imu_quaternion{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> imu_angular_velocity{};
  std::array<double, 3> imu_linear_acceleration{};
};

struct ImuOrientationSample {
  double simulation_time = 0.0;
  std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
};

std::vector<std::string> ControllerObservationNames(
    const ControllerInterfaces& interfaces);
std::vector<std::string> ControllerCommandNames(
    const ControllerInterfaces& interfaces);
std::vector<std::string> StateObservationNames(
    const StateObserverInterfaces& interfaces);

bool DecodeVehicleSample(const mujoco_sim::v1::Tick& tick,
                         const ControllerInterfaces& interfaces,
                         VehicleSample* sample, std::string* error_message);
bool DecodeStateSample(const mujoco_sim::v1::ObservationFrame& frame,
                       const StateObserverInterfaces& interfaces,
                       StateSample* sample, std::string* error_message);
bool DecodeImuOrientation(const mujoco_sim::v1::ObservationFrame& frame,
                          const std::string& observation_name,
                          ImuOrientationSample* sample,
                          std::string* error_message);

std::vector<mujoco_sim::adapter_sdk::CommandValue> BuildWheelCommands(
    const ControllerInterfaces& interfaces,
    const std::array<double, 4>& wheel_angular_velocities);

}  // namespace mujoco_sim::sensor_demo
