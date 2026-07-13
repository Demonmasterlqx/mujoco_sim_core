#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mujoco_sim::wbr_lqr_demo {
namespace {

struct ResourceSpec {
  std::string name;
  std::uint32_t dimension = 1;
};

std::vector<ResourceSpec> ObservationSpecs(const InterfaceConfig& config) {
  std::vector<ResourceSpec> specs = {
      {config.imu.quaternion, 4},
      {config.imu.linear_acceleration, 3},
      {config.imu.angular_velocity, 3},
  };
  specs.reserve(kObservationResourceCount);
  for (const auto& motor : config.motors) specs.push_back({motor.position_state, 1});
  for (const auto& motor : config.motors) specs.push_back({motor.velocity_state, 1});
  for (const auto& motor : config.motors) specs.push_back({motor.effort_state, 1});
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
  const auto observations = ObservationSpecs(config);
  const auto commands = CommandNames(config);
  std::unordered_set<std::string> observation_names;
  for (const auto& spec : observations) {
    if (spec.name.empty()) throw InterfaceError("observation resource name is empty");
    if (!observation_names.insert(spec.name).second) {
      throw InterfaceError("duplicate configured observation resource: " + spec.name);
    }
  }
  std::unordered_set<std::string> command_names;
  std::unordered_set<std::string> semantic_names;
  for (std::size_t i = 0; i < config.motors.size(); ++i) {
    if (config.motors[i].semantic_name.empty()) {
      throw InterfaceError("motor semantic name is empty at index " + std::to_string(i));
    }
    if (!semantic_names.insert(config.motors[i].semantic_name).second) {
      throw InterfaceError("duplicate motor semantic name: " +
                           config.motors[i].semantic_name);
    }
    if (commands[i].empty()) throw InterfaceError("command resource name is empty");
    if (!command_names.insert(commands[i]).second) {
      throw InterfaceError("duplicate configured command resource: " + commands[i]);
    }
  }
}

const ResourceSpec* FindObservationSpec(const std::vector<ResourceSpec>& specs,
                                        std::string_view name) {
  const auto found = std::find_if(specs.begin(), specs.end(),
                                  [name](const ResourceSpec& spec) {
                                    return spec.name == name;
                                  });
  return found == specs.end() ? nullptr : &*found;
}

std::size_t CommandIndex(const std::array<std::string, 6>& names,
                         std::string_view name) {
  const auto found = std::find(names.begin(), names.end(), name);
  if (found == names.end()) {
    throw InterfaceError("unexpected command resource: " + std::string(name));
  }
  return static_cast<std::size_t>(found - names.begin());
}

const mujoco_sim::v2::ResourceDescriptor& FindCommandDescriptor(
    const mujoco_sim::v2::ResourceLayout& layout, std::string_view name) {
  const auto found = std::find_if(
      layout.commands().begin(), layout.commands().end(),
      [name](const mujoco_sim::v2::ResourceDescriptor& descriptor) {
        return descriptor.name() == name;
      });
  if (found == layout.commands().end()) {
    throw InterfaceError("missing command resource: " + std::string(name));
  }
  return *found;
}

void ValidateTickLayoutHash(std::string_view tick_hash,
                            std::string_view registered_hash) {
  if (tick_hash.size() != 16 || registered_hash.size() != 16 ||
      tick_hash != registered_hash) {
    throw InterfaceError("tick layout hash differs from registration");
  }
}

float ToFiniteFloat(double value, const std::string& resource, int component) {
  if (!std::isfinite(value) ||
      value > static_cast<double>(std::numeric_limits<float>::max()) ||
      value < -static_cast<double>(std::numeric_limits<float>::max())) {
    throw InterfaceError("non-finite or out-of-range observation value in " +
                         resource + "[" + std::to_string(component) + "]");
  }
  return static_cast<float>(value);
}

using BlockMap =
    std::unordered_map<std::string, const mujoco_sim::v2::ObservationBlock*>;

const mujoco_sim::v2::ObservationBlock& GetBlock(const BlockMap& blocks,
                                                  const std::string& name) {
  const auto found = blocks.find(name);
  if (found == blocks.end()) {
    throw InterfaceError("missing observation block: " + name);
  }
  return *found->second;
}

float GetScalar(const BlockMap& blocks, const std::string& name) {
  return ToFiniteFloat(GetBlock(blocks, name).values(0), name, 0);
}

}  // namespace

std::vector<std::string> WbrObservationNames() {
  return WbrObservationNames(MakeUpstreamWbrInterfaceConfig());
}

std::vector<std::string> WbrObservationNames(const InterfaceConfig& config) {
  ValidateInterfaceConfig(config);
  std::vector<std::string> names;
  for (const auto& spec : ObservationSpecs(config)) names.push_back(spec.name);
  return names;
}

std::vector<std::string> WbrCommandNames() {
  return WbrCommandNames(MakeUpstreamWbrInterfaceConfig());
}

std::vector<std::string> WbrCommandNames(const InterfaceConfig& config) {
  ValidateInterfaceConfig(config);
  const auto configured = CommandNames(config);
  return {configured.begin(), configured.end()};
}

void ValidateWbrLayout(const mujoco_sim::v2::ResourceLayout& layout) {
  ValidateWbrLayout(MakeUpstreamWbrInterfaceConfig(), layout);
}

void ValidateWbrLayout(const InterfaceConfig& config,
                       const mujoco_sim::v2::ResourceLayout& layout) {
  ValidateInterfaceConfig(config);
  const auto specs = ObservationSpecs(config);
  const auto commands = CommandNames(config);
  if (layout.observations_size() != static_cast<int>(specs.size())) {
    throw InterfaceError("observation layout must contain exactly 21 resources");
  }
  std::unordered_set<std::string> names;
  for (const auto& descriptor : layout.observations()) {
    const ResourceSpec* spec = FindObservationSpec(specs, descriptor.name());
    if (spec == nullptr) {
      throw InterfaceError("unexpected observation resource: " + descriptor.name());
    }
    if (!names.insert(descriptor.name()).second) {
      throw InterfaceError("duplicate observation resource: " + descriptor.name());
    }
    if (descriptor.type() != mujoco_sim::v2::OBSERVATION_SENSOR ||
        descriptor.dimension() != spec->dimension) {
      throw InterfaceError("observation layout mismatch for " + descriptor.name());
    }
  }
  if (layout.commands_size() != static_cast<int>(commands.size())) {
    throw InterfaceError("command layout must contain exactly 6 resources");
  }
  names.clear();
  for (const auto& descriptor : layout.commands()) {
    CommandIndex(commands, descriptor.name());
    if (!names.insert(descriptor.name()).second) {
      throw InterfaceError("duplicate command resource: " + descriptor.name());
    }
    if (descriptor.type() != mujoco_sim::v2::COMMAND_ACTUATOR ||
        descriptor.dimension() != 1 || !descriptor.has_ctrl_range() ||
        !std::isfinite(descriptor.ctrl_min()) ||
        !std::isfinite(descriptor.ctrl_max()) ||
        descriptor.ctrl_min() > descriptor.ctrl_max()) {
      throw InterfaceError("invalid command layout for " + descriptor.name());
    }
  }
}

TickDecoder::TickDecoder(const mujoco_sim::v2::ResourceLayout& registered_layout)
    : TickDecoder(MakeUpstreamWbrInterfaceConfig(), registered_layout) {}

TickDecoder::TickDecoder(
    InterfaceConfig config,
    const mujoco_sim::v2::ResourceLayout& registered_layout)
    : config_(std::move(config)),
      registered_layout_hash_(registered_layout.layout_hash()) {
  ValidateWbrLayout(config_, registered_layout);
  if (registered_layout_hash_.size() != 16) {
    throw InterfaceError("registration layout hash must contain 16 bytes");
  }
}

control::msg_raw_state_t TickDecoder::Decode(
    const mujoco_sim::v2::Tick& tick) const {
  ValidateTickLayoutHash(tick.layout_hash(), registered_layout_hash_);
  const auto specs = ObservationSpecs(config_);
  if (tick.observations_size() != static_cast<int>(specs.size())) {
    throw InterfaceError("tick must contain exactly 21 observation blocks");
  }
  BlockMap blocks;
  for (const auto& block : tick.observations()) {
    const ResourceSpec* spec = FindObservationSpec(specs, block.name());
    if (spec == nullptr) throw InterfaceError("unexpected observation block: " + block.name());
    if (!blocks.emplace(block.name(), &block).second) {
      throw InterfaceError("duplicate observation block: " + block.name());
    }
    if (block.values_size() != static_cast<int>(spec->dimension)) {
      throw InterfaceError("observation value dimension mismatch for " + block.name());
    }
  }

  control::msg_raw_state_t raw{};
  raw.time = static_cast<double>(tick.header().tick_id()) *
             static_cast<double>(kControlDtSeconds);
  const auto& quaternion = GetBlock(blocks, config_.imu.quaternion);
  const auto& acceleration = GetBlock(blocks, config_.imu.linear_acceleration);
  const auto& angular_velocity = GetBlock(blocks, config_.imu.angular_velocity);
  for (int i = 0; i < 4; ++i) {
    raw.quat_gt[i] = ToFiniteFloat(quaternion.values(i), config_.imu.quaternion, i);
  }
  for (int i = 0; i < 3; ++i) {
    raw.accel[i] = ToFiniteFloat(acceleration.values(i),
                                 config_.imu.linear_acceleration, i);
    raw.gyro[i] = ToFiniteFloat(angular_velocity.values(i),
                                config_.imu.angular_velocity, i);
  }
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    raw.motors[i].q = GetScalar(blocks, config_.motors[i].position_state);
    raw.motors[i].dq = GetScalar(blocks, config_.motors[i].velocity_state);
    raw.motors[i].tau = GetScalar(blocks, config_.motors[i].effort_state);
  }
  return raw;
}

TorqueCommandMapper::TorqueCommandMapper(
    const mujoco_sim::v2::ResourceLayout& registered_layout)
    : TorqueCommandMapper(MakeUpstreamWbrInterfaceConfig(), registered_layout) {}

TorqueCommandMapper::TorqueCommandMapper(
    InterfaceConfig config,
    const mujoco_sim::v2::ResourceLayout& registered_layout) {
  ValidateWbrLayout(config, registered_layout);
  command_names_ = CommandNames(config);
  std::size_t order = 0;
  for (const auto& descriptor : registered_layout.commands()) {
    const std::size_t motor_index = CommandIndex(command_names_, descriptor.name());
    ranges_[motor_index] = {descriptor.ctrl_min(), descriptor.ctrl_max()};
    registered_order_[order++] = motor_index;
  }
}

std::vector<mujoco_sim::adapter_sdk::CommandValue> TorqueCommandMapper::Map(
    const std::array<float, 6>& raw_torque) const {
  for (std::size_t i = 0; i < raw_torque.size(); ++i) {
    if (!std::isfinite(raw_torque[i])) {
      throw InterfaceError("non-finite raw torque at motor index " +
                           std::to_string(i));
    }
  }
  std::vector<mujoco_sim::adapter_sdk::CommandValue> commands;
  last_clamp_count_ = 0;
  for (const std::size_t motor_index : registered_order_) {
    const double raw_value = static_cast<double>(raw_torque[motor_index]);
    const double clamped =
        std::clamp(raw_value, ranges_[motor_index].minimum,
                   ranges_[motor_index].maximum);
    if (clamped != raw_value) ++last_clamp_count_;
    commands.push_back(
        {command_names_[motor_index], clamped});
  }
  return commands;
}

}  // namespace mujoco_sim::wbr_lqr_demo
