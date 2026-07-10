#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/v1/sync.pb.h>
#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

namespace mujoco_sim::wbr_lqr_demo::test {

inline std::uint32_t ObservationDimension(std::string_view name) {
  if (name == "imu_quat") {
    return 4;
  }
  if (name == "imu_accel" || name == "imu_gyro") {
    return 3;
  }
  return 1;
}

inline bool IsWheelActuator(std::string_view name) {
  return name == "lwheel_actuator" || name == "rwheel_actuator";
}

inline mujoco_sim::v1::ResourceLayout MakeValidLayout(
    bool reverse_observations = false, bool reverse_commands = false) {
  auto observations = WbrObservationNames();
  auto commands = WbrCommandNames();
  if (reverse_observations) {
    std::reverse(observations.begin(), observations.end());
  }
  if (reverse_commands) {
    std::reverse(commands.begin(), commands.end());
  }

  mujoco_sim::v1::ResourceLayout layout;
  for (const auto& name : observations) {
    auto* descriptor = layout.add_observations();
    descriptor->set_name(name);
    descriptor->set_type(mujoco_sim::v1::OBSERVATION_SENSOR);
    descriptor->set_dimension(ObservationDimension(name));
  }
  for (const auto& name : commands) {
    auto* descriptor = layout.add_commands();
    descriptor->set_name(name);
    descriptor->set_type(mujoco_sim::v1::COMMAND_ACTUATOR);
    descriptor->set_dimension(1);
    descriptor->set_has_ctrl_range(true);
    const double limit = IsWheelActuator(name) ? 5.2 : 20.0;
    descriptor->set_ctrl_min(-limit);
    descriptor->set_ctrl_max(limit);
  }
  return layout;
}

inline std::vector<double> DefaultValues(std::string_view name) {
  if (name == "imu_quat") {
    return {1.0, 0.0, 0.0, 0.0};
  }
  if (name == "imu_accel") {
    return {0.0, 0.0, 9.81};
  }
  if (name == "imu_gyro") {
    return {0.0, 0.0, 0.0};
  }

  constexpr std::array<std::string_view, 6> motors{
      "ljoint1", "ljoint4", "lwheel", "rjoint1", "rjoint4", "rwheel"};
  constexpr std::array<double, 6> positions{-0.75, -2.25, 0.0,
                                             0.75, 2.25, 0.0};
  for (std::size_t i = 0; i < motors.size(); ++i) {
    const std::string prefix(motors[i]);
    if (name == prefix + "_pos") {
      return {positions[i]};
    }
    if (name == prefix + "_vel" || name == prefix + "_tor") {
      return {0.0};
    }
  }
  return {};
}

inline mujoco_sim::v1::Tick MakeValidTick(
    const mujoco_sim::v1::ResourceLayout& layout, std::uint64_t tick_id = 0,
    bool reverse_blocks = true) {
  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_tick_id(tick_id);
  *tick.mutable_layout() = layout;

  auto names = WbrObservationNames();
  if (reverse_blocks) {
    std::reverse(names.begin(), names.end());
  }
  for (const auto& name : names) {
    auto* block = tick.add_observations();
    block->set_name(name);
    for (const double value : DefaultValues(name)) {
      block->add_values(value);
    }
  }
  return tick;
}

inline mujoco_sim::v1::ObservationBlock* FindBlock(
    mujoco_sim::v1::Tick* tick, std::string_view name) {
  for (auto& block : *tick->mutable_observations()) {
    if (block.name() == name) {
      return &block;
    }
  }
  return nullptr;
}

inline control::msg_raw_state_t MakeHomeObservation() {
  control::msg_raw_state_t raw{};
  raw.quat_gt[0] = 1.0F;
  raw.accel[2] = 9.81F;
  raw.motors[0].q = -0.75F;
  raw.motors[1].q = -2.25F;
  raw.motors[2].q = 0.0F;
  raw.motors[3].q = 0.75F;
  raw.motors[4].q = 2.25F;
  raw.motors[5].q = 0.0F;
  return raw;
}

}  // namespace mujoco_sim::wbr_lqr_demo::test
