#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <control/config.hpp>
#include <control/imu_fusion.hpp>

namespace mujoco_sim::wbr_lqr_demo {

inline constexpr float kControlDtSeconds = 0.001F;

control::chassis_config MakeUpstreamWbrChassisConfig();

struct ImuStateInterfaces {
  std::string quaternion;
  std::string angular_velocity;
  std::string linear_acceleration;
};

struct MotorInterfaces {
  std::string semantic_name;
  std::string position_state;
  std::string velocity_state;
  std::string effort_state;
  std::string effort_command;
};

struct InterfaceConfig {
  ImuStateInterfaces imu;
  std::array<MotorInterfaces, 6> motors;
};

InterfaceConfig MakeUpstreamWbrInterfaceConfig();

struct PipelineConfig {
  control::chassis_config chassis = MakeUpstreamWbrChassisConfig();
  control::imu_mode imu_mode = control::imu_mode::mahony;
  control::mahony_config mahony{};
};

// Stable controller DTO populated by the generated-config conversion layer.
struct AdapterConfig {
  std::string server_name = "default";
  std::string adapter_id = "wbr_lqr_controller";
  std::uint32_t commit_timeout_ms = 1000;
  std::uint32_t session_timeout_ms = 1000;
  std::uint32_t heartbeat_period_ms = 250;
  std::uint32_t service_timeout_ms = 5000;
  std::uint64_t max_ticks = 0;
  InterfaceConfig interfaces = MakeUpstreamWbrInterfaceConfig();
};

struct DemoConfig {
  AdapterConfig adapter{};
  PipelineConfig pipeline{};
};

}  // namespace mujoco_sim::wbr_lqr_demo
