#include <mujoco_sim/wbr_lqr_demo/config.hpp>

namespace mujoco_sim::wbr_lqr_demo {

control::chassis_config MakeUpstreamWbrChassisConfig() {
  control::chassis_config config = control::k_default_chassis;
  config.control_dt = kControlDtSeconds;

  config.motor_zero_rad[0] = -0.06F;
  config.motor_zero_rad[1] = -0.20F;
  config.motor_zero_rad[2] = 0.0F;
  config.motor_zero_rad[3] = 0.06F;
  config.motor_zero_rad[4] = 0.20F;
  config.motor_zero_rad[5] = 0.0F;

  config.leg_pid.len = {6000.0F, 0.0F, -1500.0F, 100.0F, 0.0F, control::pid_mode::dvel};
  config.leg_pid.phi = {0.7F, 0.0F, 1.4F, 20.0F, 0.005F, control::pid_mode::position};
  config.fsm_pid.recover_kick_torque = 15.0F;
  return config;
}

InterfaceConfig MakeUpstreamWbrInterfaceConfig() {
  return {
      {"imu_quat", "imu_gyro", "imu_accel"},
      {{{"left_joint_1", "ljoint1_pos", "ljoint1_vel", "ljoint1_tor", "ljoint1_actuator"},
        {"left_joint_4", "ljoint4_pos", "ljoint4_vel", "ljoint4_tor", "ljoint4_actuator"},
        {"left_wheel", "lwheel_pos", "lwheel_vel", "lwheel_tor", "lwheel_actuator"},
        {"right_joint_1", "rjoint1_pos", "rjoint1_vel", "rjoint1_tor", "rjoint1_actuator"},
        {"right_joint_4", "rjoint4_pos", "rjoint4_vel", "rjoint4_tor", "rjoint4_actuator"},
        {"right_wheel", "rwheel_pos", "rwheel_vel", "rwheel_tor", "rwheel_actuator"}}},
  };
}

}  // namespace mujoco_sim::wbr_lqr_demo
