#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace mujoco_sim::wbr_lqr_demo {
namespace {

struct ResourceSpec {
  std::string name;
  std::size_t dimension = 1;
};

void MarkDenseSpan(mujoco_sim::adapter_sdk::ScalarSpan span, std::string_view resource,
                   std::vector<bool>* occupied) {
  if (span.offset > occupied->size() || span.count > occupied->size() - span.offset) {
    throw InterfaceError("registered scalar span is out of bounds for " + std::string(resource));
  }
  for (std::size_t index = span.offset; index < span.offset + span.count; ++index) {
    if ((*occupied)[index]) {
      throw InterfaceError("registered scalar span overlaps at " + std::string(resource));
    }
    (*occupied)[index] = true;
  }
}

std::vector<ResourceSpec> ObservationSpecs(const InterfaceConfig& config) {
  std::vector<ResourceSpec> specs = {
      {config.imu.quaternion, 4},
      {config.imu.linear_acceleration, 3},
      {config.imu.angular_velocity, 3},
  };
  specs.reserve(kObservationResourceCount);
  for (const auto& motor : config.motors)
    specs.push_back({motor.position_state, 1});
  for (const auto& motor : config.motors)
    specs.push_back({motor.velocity_state, 1});
  for (const auto& motor : config.motors)
    specs.push_back({motor.effort_state, 1});
  return specs;
}

std::array<std::string, 6> CommandNames(const InterfaceConfig& config) {
  std::array<std::string, 6> names;
  for (std::size_t i = 0; i < names.size(); ++i) {
    names[i] = config.motors[i].effort_command;
  }
  return names;
}

void ValidateInterfaceConfig(const InterfaceConfig& config) {
  std::unordered_set<std::string> names;
  for (const auto& spec : ObservationSpecs(config)) {
    if (spec.name.empty() || !names.insert(spec.name).second) {
      throw InterfaceError("invalid or duplicate observation resource: " + spec.name);
    }
  }
  names.clear();
  for (const auto& name : CommandNames(config)) {
    if (name.empty() || !names.insert(name).second) {
      throw InterfaceError("invalid or duplicate command resource: " + name);
    }
  }
}

const mujoco_sim::adapter_sdk::ResourceInfo& FindResource(
    const std::vector<mujoco_sim::adapter_sdk::ResourceInfo>& resources, std::string_view name) {
  const auto found = std::find_if(resources.begin(), resources.end(),
                                  [name](const auto& resource) { return resource.name == name; });
  if (found == resources.end()) {
    throw InterfaceError("missing registered resource: " + std::string(name));
  }
  return *found;
}

float FiniteFloat(double value, std::string_view resource, std::size_t component) {
  if (!std::isfinite(value) ||
      std::fabs(value) > static_cast<double>(std::numeric_limits<float>::max())) {
    throw InterfaceError("invalid observation value in " + std::string(resource) + "[" +
                         std::to_string(component) + "]");
  }
  return static_cast<float>(value);
}

double Scalar(std::span<const double> values, mujoco_sim::adapter_sdk::ScalarSpan span,
              std::string_view resource) {
  if (span.count != 1 || span.offset >= values.size()) {
    throw InterfaceError("invalid scalar span for " + std::string(resource));
  }
  return values[span.offset];
}

}  // namespace

std::vector<std::string> WbrObservationNames() {
  return WbrObservationNames(MakeUpstreamWbrInterfaceConfig());
}

std::vector<std::string> WbrObservationNames(const InterfaceConfig& config) {
  ValidateInterfaceConfig(config);
  std::vector<std::string> names;
  for (const auto& spec : ObservationSpecs(config))
    names.push_back(spec.name);
  return names;
}

std::vector<std::string> WbrCommandNames() {
  return WbrCommandNames(MakeUpstreamWbrInterfaceConfig());
}

std::vector<std::string> WbrCommandNames(const InterfaceConfig& config) {
  ValidateInterfaceConfig(config);
  const auto names = CommandNames(config);
  return {names.begin(), names.end()};
}

void ValidateWbrLayout(const mujoco_sim::adapter_sdk::ControllerLayout& layout) {
  ValidateWbrLayout(MakeUpstreamWbrInterfaceConfig(), layout);
}

void ValidateWbrLayout(const InterfaceConfig& config,
                       const mujoco_sim::adapter_sdk::ControllerLayout& layout) {
  ValidateInterfaceConfig(config);
  const auto specs = ObservationSpecs(config);
  if (layout.observations().size() != specs.size()) {
    throw InterfaceError("observation layout must contain exactly 21 resources");
  }
  const std::size_t observation_scalar_count = std::accumulate(
      specs.begin(), specs.end(), std::size_t{0},
      [](std::size_t total, const ResourceSpec& spec) { return total + spec.dimension; });
  if (layout.observation_scalar_count() != observation_scalar_count) {
    throw InterfaceError("observation layout has an invalid dense scalar count");
  }
  std::vector<bool> occupied_observations(observation_scalar_count);
  for (const auto& spec : specs) {
    const auto& resource = FindResource(layout.observations(), spec.name);
    if (resource.scalars.count != spec.dimension) {
      throw InterfaceError("observation layout mismatch for " + spec.name);
    }
    MarkDenseSpan(resource.scalars, spec.name, &occupied_observations);
  }
  const auto commands = CommandNames(config);
  if (layout.commands().size() != commands.size()) {
    throw InterfaceError("command layout must contain exactly 6 resources");
  }
  if (layout.command_scalar_count() != commands.size()) {
    throw InterfaceError("command layout has an invalid dense scalar count");
  }
  std::vector<bool> occupied_commands(commands.size());
  for (const auto& name : commands) {
    const auto& resource = FindResource(layout.commands(), name);
    if (resource.scalars.count != 1 || !resource.has_command_range ||
        !std::isfinite(resource.command_min) || !std::isfinite(resource.command_max) ||
        resource.command_min > resource.command_max) {
      throw InterfaceError("invalid command layout for " + name);
    }
    MarkDenseSpan(resource.scalars, name, &occupied_commands);
  }
}

StateDecoder::StateDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout)
    : StateDecoder(MakeUpstreamWbrInterfaceConfig(), layout) {}

StateDecoder::StateDecoder(InterfaceConfig config,
                           const mujoco_sim::adapter_sdk::ControllerLayout& layout)
    : config_(std::move(config)), observation_scalar_count_(layout.observation_scalar_count()) {
  ValidateWbrLayout(config_, layout);
  quaternion_ = *layout.Observation(config_.imu.quaternion);
  acceleration_ = *layout.Observation(config_.imu.linear_acceleration);
  angular_velocity_ = *layout.Observation(config_.imu.angular_velocity);
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    positions_[i] = *layout.Observation(config_.motors[i].position_state);
    velocities_[i] = *layout.Observation(config_.motors[i].velocity_state);
    efforts_[i] = *layout.Observation(config_.motors[i].effort_state);
  }
}

control::msg_raw_state_t StateDecoder::Decode(
    const mujoco_sim::adapter_sdk::StateLease& state) const {
  return Decode(state.observations(), state.simulation_time_s());
}

control::msg_raw_state_t StateDecoder::Decode(std::span<const double> values,
                                              double simulation_time_s) const {
  if (values.size() != observation_scalar_count_) {
    throw InterfaceError("observation vector size mismatch: expected " +
                         std::to_string(observation_scalar_count_) + ", got " +
                         std::to_string(values.size()));
  }
  if (!std::isfinite(simulation_time_s)) {
    throw InterfaceError("invalid simulation timestamp");
  }
  control::msg_raw_state_t raw{};
  raw.time = simulation_time_s;
  for (std::size_t i = 0; i < quaternion_.count; ++i) {
    raw.quat_gt[i] = FiniteFloat(values[quaternion_.offset + i], config_.imu.quaternion, i);
  }
  for (std::size_t i = 0; i < acceleration_.count; ++i) {
    raw.accel[i] =
        FiniteFloat(values[acceleration_.offset + i], config_.imu.linear_acceleration, i);
    raw.gyro[i] =
        FiniteFloat(values[angular_velocity_.offset + i], config_.imu.angular_velocity, i);
  }
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    raw.motors[i].q = FiniteFloat(Scalar(values, positions_[i], config_.motors[i].position_state),
                                  config_.motors[i].position_state, 0);
    raw.motors[i].dq = FiniteFloat(Scalar(values, velocities_[i], config_.motors[i].velocity_state),
                                   config_.motors[i].velocity_state, 0);
    raw.motors[i].tau = FiniteFloat(Scalar(values, efforts_[i], config_.motors[i].effort_state),
                                    config_.motors[i].effort_state, 0);
  }
  return raw;
}

TorqueCommandMapper::TorqueCommandMapper(const mujoco_sim::adapter_sdk::ControllerLayout& layout)
    : TorqueCommandMapper(MakeUpstreamWbrInterfaceConfig(), layout) {}

TorqueCommandMapper::TorqueCommandMapper(InterfaceConfig config,
                                         const mujoco_sim::adapter_sdk::ControllerLayout& layout) {
  ValidateWbrLayout(config, layout);
  const auto names = CommandNames(config);
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto& resource = FindResource(layout.commands(), names[i]);
    ranges_[i] = {resource.command_min, resource.command_max};
    scalar_offsets_[i] = resource.scalars.offset;
  }
}

std::vector<double> TorqueCommandMapper::Map(const std::array<float, 6>& raw_torque) const {
  std::vector<double> commands(kCommandResourceCount);
  last_clamp_count_ = 0;
  for (std::size_t i = 0; i < raw_torque.size(); ++i) {
    if (!std::isfinite(raw_torque[i])) {
      throw InterfaceError("non-finite raw torque at motor index " + std::to_string(i));
    }
    const double raw = raw_torque[i];
    const double clamped = std::clamp(raw, ranges_[i].minimum, ranges_[i].maximum);
    if (clamped != raw) ++last_clamp_count_;
    commands[scalar_offsets_[i]] = clamped;
  }
  return commands;
}

}  // namespace mujoco_sim::wbr_lqr_demo
