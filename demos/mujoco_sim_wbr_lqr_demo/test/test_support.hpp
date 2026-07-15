#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

namespace mujoco_sim::wbr_lqr_demo::test {

struct DenseTick {
  std::vector<double> observations;
  double simulation_time_s = 0.0;
};

inline std::size_t ObservationDimension(const InterfaceConfig& config,
                                        std::string_view name) {
  if (name == config.imu.quaternion) return 4;
  if (name == config.imu.linear_acceleration ||
      name == config.imu.angular_velocity) return 3;
  return 1;
}

inline bool IsWheelActuator(std::string_view name) {
  return name == "lwheel_actuator" || name == "rwheel_actuator";
}

inline mujoco_sim::adapter_sdk::ControllerLayout MakeValidLayout(
    bool reverse_observations = false, bool reverse_commands = false,
    const InterfaceConfig& config = MakeUpstreamWbrInterfaceConfig()) {
  auto observation_names = WbrObservationNames(config);
  auto command_names = WbrCommandNames(config);
  if (reverse_observations) std::reverse(observation_names.begin(), observation_names.end());
  if (reverse_commands) std::reverse(command_names.begin(), command_names.end());
  std::vector<mujoco_sim::adapter_sdk::ResourceInfo> observations;
  std::vector<mujoco_sim::adapter_sdk::ResourceInfo> commands;
  std::size_t offset = 0;
  for (const auto& name : observation_names) {
    const auto count = ObservationDimension(config, name);
    observations.push_back({name, {offset, count}});
    offset += count;
  }
  std::size_t command_offset = 0;
  for (const auto& name : command_names) {
    double limit = 20.0;
    if (IsWheelActuator(name)) limit = 5.2;
    commands.push_back({name, {command_offset++, 1}, true, -limit, limit});
  }
  return {std::move(observations), std::move(commands), offset,
          command_offset};
}

inline std::vector<double> DefaultValues(std::string_view name) {
  if (name == "imu_quat") return {1.0, 0.0, 0.0, 0.0};
  if (name == "imu_accel") return {0.0, 0.0, 9.81};
  if (name == "imu_gyro") return {0.0, 0.0, 0.0};
  constexpr std::array<std::string_view, 6> motors{
      "ljoint1", "ljoint4", "lwheel", "rjoint1", "rjoint4", "rwheel"};
  constexpr std::array<double, 6> positions{-0.75, -2.25, 0.0,
                                             0.75, 2.25, 0.0};
  for (std::size_t i = 0; i < motors.size(); ++i) {
    const std::string prefix(motors[i]);
    if (name == prefix + "_pos") return {positions[i]};
    if (name == prefix + "_vel" || name == prefix + "_tor") return {0.0};
  }
  return {};
}

inline void SetObservation(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                           std::string_view name, std::span<const double> values,
                           std::vector<double>* output) {
  const auto span = layout.Observation(name);
  if (!span || span->count != values.size()) return;
  std::copy(values.begin(), values.end(), output->begin() + span->offset);
}

inline DenseTick MakeValidTick(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
    double simulation_time_s = 0.0) {
  DenseTick tick{{}, simulation_time_s};
  tick.observations.resize(layout.observation_scalar_count());
  for (const auto& resource : layout.observations()) {
    const auto values = DefaultValues(resource.name);
    SetObservation(layout, resource.name, values, &tick.observations);
  }
  return tick;
}

inline control::msg_raw_state_t MakeHomeObservation() {
  control::msg_raw_state_t raw{};
  raw.quat_gt[0] = 1.0F;
  raw.accel[2] = 9.81F;
  raw.motors[0].q = -0.75F;
  raw.motors[1].q = -2.25F;
  raw.motors[3].q = 0.75F;
  raw.motors[4].q = 2.25F;
  return raw;
}

}  // namespace mujoco_sim::wbr_lqr_demo::test
