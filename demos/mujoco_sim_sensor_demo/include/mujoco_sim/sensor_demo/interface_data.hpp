#pragma once

#include <array>
#include <span>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>

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

std::vector<std::string> ControllerObservationNames(const ControllerInterfaces& interfaces);
std::vector<std::string> ControllerCommandNames(const ControllerInterfaces& interfaces);
std::vector<std::string> StateObservationNames(const StateObserverInterfaces& interfaces);

class VehicleDecoder {
 public:
  VehicleDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                 const ControllerInterfaces& interfaces);
  bool Decode(const mujoco_sim::adapter_sdk::StateLease& tick, VehicleSample* sample,
              std::string* error_message) const;
  bool Decode(std::span<const double> observations, double simulation_time_s, VehicleSample* sample,
              std::string* error_message) const;

 private:
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 4> wheel_positions_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_quaternion_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_gyro_{};
  std::string layout_error_;
};

class StateDecoder {
 public:
  StateDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
               const StateObserverInterfaces& interfaces);
  bool Decode(const mujoco_sim::adapter_sdk::ObservationLease& frame, StateSample* sample,
              std::string* error_message) const;
  bool Decode(std::span<const double> observations, double simulation_time_s, StateSample* sample,
              std::string* error_message) const;

 private:
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 4> wheel_positions_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 4> wheel_velocities_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_quaternion_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_angular_velocity_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_linear_acceleration_{};
  std::string layout_error_;
};

class ImuOrientationDecoder {
 public:
  ImuOrientationDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                        const std::string& observation_name);
  bool Decode(const mujoco_sim::adapter_sdk::ObservationLease& frame, ImuOrientationSample* sample,
              std::string* error_message) const;
  bool Decode(std::span<const double> observations, double simulation_time_s,
              ImuOrientationSample* sample, std::string* error_message) const;

 private:
  mujoco_sim::adapter_sdk::ScalarSpan quaternion_{};
  std::string layout_error_;
};

class WheelCommandMapper {
 public:
  WheelCommandMapper(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                     const ControllerInterfaces& interfaces);
  bool Map(const std::array<double, 4>& wheel_angular_velocities, std::vector<double>* commands,
           std::string* error_message) const;

 private:
  std::array<std::size_t, 4> offsets_{};
  std::size_t scalar_count_ = 0;
  std::string layout_error_;
};

}  // namespace mujoco_sim::sensor_demo
