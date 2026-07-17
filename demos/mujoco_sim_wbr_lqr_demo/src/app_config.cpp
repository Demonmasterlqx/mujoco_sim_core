#include "app_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

namespace mujoco_sim::wbr_lqr_demo {
namespace {

bool ToFloat(double value, const std::string& field, float* output, std::string* error_message) {
  if (!std::isfinite(value) ||
      std::fabs(value) > static_cast<double>(std::numeric_limits<float>::max())) {
    *error_message = field + " is outside the finite float range";
    return false;
  }
  *output = static_cast<float>(value);
  return true;
}

template <typename Mode>
control::pid_mode ToPidMode(Mode mode) {
  return mode == Mode::kDvel ? control::pid_mode::dvel : control::pid_mode::position;
}

template <typename Source>
bool CopyPid(const Source& source, const std::string& field, control::pid_params* target,
             std::string* error_message) {
  return ToFloat(source.kp, field + ".kp", &target->kp, error_message) &&
         ToFloat(source.ki, field + ".ki", &target->ki, error_message) &&
         ToFloat(source.kd, field + ".kd", &target->kd, error_message) &&
         ToFloat(source.max_out, field + ".max_out", &target->max_out, error_message) &&
         ToFloat(source.max_i, field + ".max_i", &target->max_i_out, error_message);
}

template <typename Source>
bool CopyPhiPid(const Source& source, const std::string& field, control::phi_state_pid* target,
                std::string* error_message) {
  return ToFloat(source.kp, field + ".kp", &target->kp, error_message) &&
         ToFloat(source.kd, field + ".kd", &target->kd, error_message) &&
         ToFloat(source.slope, field + ".slope", &target->slope, error_message);
}

bool BuildInterfaces(const generated::WbrConfig::Interfaces& source, InterfaceConfig* target,
                     std::string* error_message) {
  target->imu = {source.imu.quaternion, source.imu.angular_velocity,
                 source.imu.linear_acceleration};
  constexpr std::array<const char*, 6> kSemanticOrder = {
      "left_joint_1", "left_joint_4", "left_wheel", "right_joint_1", "right_joint_4", "right_wheel",
  };
  std::unordered_map<std::string, const generated::WbrConfig::Interfaces::MotorsItem*> by_name;
  for (const auto& motor : source.motors) {
    if (!by_name.emplace(motor.semantic_name, &motor).second) {
      *error_message = "duplicate motor semantic_name: " + motor.semantic_name;
      return false;
    }
  }
  for (std::size_t i = 0; i < kSemanticOrder.size(); ++i) {
    const auto found = by_name.find(kSemanticOrder[i]);
    if (found == by_name.end()) {
      *error_message = "missing motor semantic_name: " + std::string(kSemanticOrder[i]);
      return false;
    }
    const auto& motor = *found->second;
    target->motors[i] = {motor.semantic_name, motor.position_state, motor.velocity_state,
                         motor.effort_state, motor.effort_command};
  }
  for (const auto& [name, _] : by_name) {
    if (std::find(kSemanticOrder.begin(), kSemanticOrder.end(), name) == kSemanticOrder.end()) {
      *error_message = "unknown motor semantic_name: " + name;
      return false;
    }
  }
  try {
    WbrObservationNames(*target);
    WbrCommandNames(*target);
  } catch (const InterfaceError& error) {
    *error_message = error.what();
    return false;
  }
  return true;
}

bool BuildPipeline(const generated::WbrConfig::Pipeline& source, PipelineConfig* target,
                   std::string* error_message) {
  *target = PipelineConfig{};
  target->imu_mode = source.imu_mode == generated::WbrConfig::Pipeline::ImuMode::kBypass
                         ? control::imu_mode::bypass
                         : control::imu_mode::mahony;
  if (!ToFloat(source.mahony.kp, "pipeline.mahony.kp", &target->mahony.kp, error_message) ||
      !ToFloat(source.mahony.ki, "pipeline.mahony.ki", &target->mahony.ki, error_message) ||
      !ToFloat(source.mahony.accel_norm_min, "pipeline.mahony.accel_norm_min",
               &target->mahony.accel_norm_min, error_message)) {
    return false;
  }

  auto& chassis = target->chassis;
  const auto& control = source.control;
  chassis.force_relax = control.force_relax;
  for (std::size_t i = 0; i < control.motor_zero_rad.size(); ++i) {
    if (!ToFloat(control.motor_zero_rad[i],
                 "pipeline.control.motor_zero_rad[" + std::to_string(i) + "]",
                 &chassis.motor_zero_rad[i], error_message)) {
      return false;
    }
  }
  if (!CopyPid(control.pid.leg.len, "pipeline.control.pid.leg.len", &chassis.leg_pid.len,
               error_message) ||
      !CopyPid(control.pid.leg.phi, "pipeline.control.pid.leg.phi", &chassis.leg_pid.phi,
               error_message) ||
      !CopyPid(control.pid.roll, "pipeline.control.pid.roll", &chassis.fsm_pid.roll,
               error_message) ||
      !CopyPid(control.pid.len_balance, "pipeline.control.pid.len_balance",
               &chassis.fsm_pid.len_balance, error_message) ||
      !CopyPhiPid(control.pid.recover, "pipeline.control.pid.recover", &chassis.fsm_pid.recover,
                  error_message) ||
      !CopyPhiPid(control.pid.flatten.high_phi, "pipeline.control.pid.flatten.high_phi",
                  &chassis.fsm_pid.flatten_high, error_message) ||
      !CopyPhiPid(control.pid.flatten.low_phi, "pipeline.control.pid.flatten.low_phi",
                  &chassis.fsm_pid.flatten_low, error_message) ||
      !CopyPhiPid(control.pid.neutral, "pipeline.control.pid.neutral", &chassis.fsm_pid.neutral,
                  error_message) ||
      !ToFloat(control.pid.recover.kick_torque, "pipeline.control.pid.recover.kick_torque",
               &chassis.fsm_pid.recover_kick_torque, error_message) ||
      !ToFloat(control.fsm.enter_normal_alpha, "pipeline.control.fsm.enter_normal_alpha",
               &chassis.fsm.enter_normal_alpha, error_message) ||
      !ToFloat(control.fsm.exit_relax_alpha, "pipeline.control.fsm.exit_relax_alpha",
               &chassis.fsm.exit_relax_alpha, error_message)) {
    return false;
  }
  chassis.leg_pid.len.mode = ToPidMode(control.pid.leg.len.mode);
  chassis.leg_pid.phi.mode = ToPidMode(control.pid.leg.phi.mode);
  chassis.fsm.enter_normal_ticks = control.fsm.enter_normal_ticks;
  chassis.fsm.exit_relax_ticks = control.fsm.exit_relax_ticks;
  chassis.fsm.offground_min_air_ticks = control.fsm.offground_min_air_ticks;
  return true;
}

bool BuildScript(const std::vector<generated::WbrConfig::InputScriptItem>& source,
                 InputScript* target, std::string* error_message) {
  std::vector<InputInterval> intervals;
  intervals.reserve(source.size());
  for (const auto& item : source) {
    control::input_snapshot_t input{};
    input.w = item.w;
    input.s = item.s;
    input.a = item.a;
    input.d = item.d;
    input.q = item.q;
    input.e = item.e;
    input.f = item.f;
    input.space = item.space;
    if (item.space && (item.begin_tick == 0 || item.end_tick != item.begin_tick + 1)) {
      *error_message = "Space input must be a one-tick pulse after tick zero";
      return false;
    }
    intervals.push_back({item.begin_tick, item.end_tick, input});
  }
  std::sort(intervals.begin(), intervals.end(),
            [](const InputInterval& lhs, const InputInterval& rhs) {
              return lhs.begin_tick < rhs.begin_tick;
            });
  for (std::size_t i = 1; i < intervals.size(); ++i) {
    if (intervals[i - 1].input.space && intervals[i].input.space &&
        intervals[i - 1].end_tick == intervals[i].begin_tick) {
      *error_message = "Space pulses require at least one released tick";
      return false;
    }
  }
  try {
    *target = InputScript(std::move(intervals));
  } catch (const std::invalid_argument& error) {
    *error_message = error.what();
    return false;
  }
  return true;
}

}  // namespace

bool BuildAppConfig(const generated::WbrConfig& generated_config, AppConfig* config,
                    std::string* error_message) {
  if (config == nullptr || error_message == nullptr) return false;
  error_message->clear();
  AppConfig output;
  output.adapter.server_name = generated_config.adapter.server_name;
  output.adapter.adapter_id = generated_config.adapter.adapter_id;
  output.adapter.service_timeout_ms = generated_config.adapter.service_timeout_ms;
  output.adapter.max_ticks = generated_config.adapter.max_ticks;
  output.log_every_ticks = generated_config.log_every_ticks;
  if (!BuildInterfaces(generated_config.interfaces, &output.adapter.interfaces, error_message) ||
      !BuildPipeline(generated_config.pipeline, &output.pipeline, error_message) ||
      !BuildScript(generated_config.input_script, &output.input_script, error_message)) {
    return false;
  }
  *config = std::move(output);
  return true;
}

}  // namespace mujoco_sim::wbr_lqr_demo
