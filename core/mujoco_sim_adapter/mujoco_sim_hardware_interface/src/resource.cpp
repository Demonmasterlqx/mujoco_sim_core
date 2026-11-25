#include "resource.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string_view>
#include <unordered_set>
#include <utility>

#include <mujoco_sim/protocol/identifier.hpp>

#include "resource_types.hpp"

namespace mujoco_sim::hardware::internal {
namespace {

constexpr std::array<std::string_view, 5> kCommonKeys = {
    "server_id", "adapter_id", "server_time_out_ms", "publish_sim_time", "debug"};

bool Fail(std::string message, std::string& error) {
  error = std::move(message);
  return false;
}

hardware_interface::InterfaceDescription Description(const StateBinding& binding) {
  hardware_interface::InterfaceInfo info{};
  info.name = binding.interface_name;
  info.data_type = "double";
  info.size = 1;
  info.initial_value = "0.0";
  return {binding.component, info};
}

hardware_interface::InterfaceDescription Description(const CommandBinding& binding) {
  hardware_interface::InterfaceInfo info{};
  info.name = binding.interface_name;
  info.data_type = "double";
  info.size = 1;
  info.initial_value = "0.0";
  return {binding.component, info};
}

bool ParseBool(std::string_view text, std::string_view field, bool& output, std::string& error) {
  if (text == "true") {
    output = true;
    return true;
  }
  if (text == "false") {
    output = false;
    return true;
  }
  return Fail(std::string(field) + " must be exactly 'true' or 'false'", error);
}

bool ParsePositiveSize(std::string_view text, std::string_view field, std::size_t& output,
                       std::string& error) {
  std::uint64_t parsed = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
      parsed == 0 || parsed > std::numeric_limits<std::size_t>::max()) {
    return Fail(std::string(field) + " must be a positive integer", error);
  }
  output = static_cast<std::size_t>(parsed);
  return true;
}

bool ParsePositiveUint32(std::string_view text, std::string_view field, std::uint32_t& output,
                         std::string& error) {
  std::uint64_t parsed = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
      parsed == 0 || parsed > std::numeric_limits<std::uint32_t>::max()) {
    return Fail(std::string(field) + " must be in [1, UINT32_MAX]", error);
  }
  output = static_cast<std::uint32_t>(parsed);
  return true;
}

bool IsFrameId(std::string_view value) {
  return !value.empty() && value.front() != '/' &&
         std::none_of(value.begin(), value.end(),
                      [](unsigned char character) { return std::isspace(character); });
}

std::string Get(const ParameterMap& parameters, std::string_view key) {
  const auto found = parameters.find(std::string(key));
  return found == parameters.end() ? std::string{} : found->second;
}

bool DebugEnabled(const ParameterMap& parameters, bool& debug, std::string& error) {
  const auto found = parameters.find("debug");
  return found == parameters.end() || ParseBool(found->second, "debug", debug, error);
}

bool IsAllowed(std::string_view key, std::span<const std::string_view> type_keys) {
  return std::find(kCommonKeys.begin(), kCommonKeys.end(), key) != kCommonKeys.end() ||
         std::find(type_keys.begin(), type_keys.end(), key) != type_keys.end();
}

bool CheckKeys(const ParameterMap& parameters, std::span<const std::string_view> type_keys,
               std::string_view context, std::string& error) {
  for (const auto& [key, value] : parameters) {
    (void)value;
    if (!IsAllowed(key, type_keys)) {
      return Fail(std::string(context) + " has unknown parameter '" + key + "'", error);
    }
  }
  return true;
}

std::vector<std::string> SplitNames(const std::string& text) {
  std::istringstream stream(text);
  std::vector<std::string> names;
  for (std::string name; stream >> name;)
    names.push_back(std::move(name));
  return names;
}

std::unique_ptr<ResourceInterface> MakeJoint(const hardware_interface::ComponentInfo& component,
                                             const ParameterMap& parameters, std::string& error) {
  static constexpr std::array<std::string_view, 6> keys = {"actuator_position", "actuator_velocity",
                                                           "actuator_effort",   "sensor_position",
                                                           "sensor_velocity",   "sensor_effort"};
  if (!CheckKeys(parameters, keys, "joint '" + component.name + "'", error)) return nullptr;
  bool debug = false;
  if (!DebugEnabled(parameters, debug, error)) {
    error = "joint '" + component.name + "': " + error;
    return nullptr;
  }

  std::vector<StateBinding> states;
  std::vector<CommandBinding> commands;
  for (const auto& [key, interface_name] :
       std::array<std::pair<std::string_view, std::string_view>, 3>{{
           {"sensor_position", "position"},
           {"sensor_velocity", "velocity"},
           {"sensor_effort", "effort"},
       }}) {
    const auto found = parameters.find(std::string(key));
    if (found == parameters.end()) continue;
    if (found->second.empty()) {
      Fail("joint '" + component.name + "' parameter '" + std::string(key) + "' must not be empty",
           error);
      return nullptr;
    }
    states.push_back({component.name, std::string(interface_name), found->second, 0, 1});
  }
  for (const auto& [key, interface_name] :
       std::array<std::pair<std::string_view, std::string_view>, 3>{{
           {"actuator_position", "position"},
           {"actuator_velocity", "velocity"},
           {"actuator_effort", "effort"},
       }}) {
    const auto found = parameters.find(std::string(key));
    if (found == parameters.end()) continue;
    if (found->second.empty()) {
      Fail("joint '" + component.name + "' parameter '" + std::string(key) + "' must not be empty",
           error);
      return nullptr;
    }
    commands.push_back({component.name, std::string(interface_name), found->second});
  }
  if (states.empty() && commands.empty()) {
    Fail("joint '" + component.name + "' must map at least one resource", error);
    return nullptr;
  }
  return std::make_unique<JointResource>(component.name, debug, std::move(states),
                                         std::move(commands));
}

std::unique_ptr<ResourceInterface> MakeGenericSensor(
    const hardware_interface::ComponentInfo& component, const ParameterMap& parameters,
    std::string& error) {
  static constexpr std::array<std::string_view, 4> keys = {"sensor_type", "sensor_name",
                                                           "sensor_dim", "sensor_dim_names"};
  if (!CheckKeys(parameters, keys, "sensor '" + component.name + "'", error)) return nullptr;
  bool debug = false;
  if (!DebugEnabled(parameters, debug, error)) {
    error = "sensor '" + component.name + "': " + error;
    return nullptr;
  }
  const auto resource = Get(parameters, "sensor_name");
  std::size_t dimension = 0;
  if (resource.empty()) {
    Fail("sensor '" + component.name + "' requires sensor_name", error);
    return nullptr;
  }
  if (!ParsePositiveSize(Get(parameters, "sensor_dim"), "sensor_dim", dimension, error)) {
    error = "sensor '" + component.name + "': " + error;
    return nullptr;
  }
  const auto names = SplitNames(Get(parameters, "sensor_dim_names"));
  std::unordered_set<std::string> unique;
  if (names.size() != dimension || !std::all_of(names.begin(), names.end(), [&](const auto& name) {
        return !name.empty() && unique.insert(name).second;
      })) {
    Fail("sensor '" + component.name +
             "' sensor_dim_names must contain exactly sensor_dim unique names",
         error);
    return nullptr;
  }
  std::vector<StateBinding> states;
  states.reserve(dimension);
  for (std::size_t index = 0; index < dimension; ++index) {
    states.push_back({component.name, names[index], resource, index, dimension});
  }
  return std::make_unique<GenericSensorResource>(component.name, debug, std::move(states));
}

std::unique_ptr<ResourceInterface> MakeImu(const hardware_interface::ComponentInfo& component,
                                           const ParameterMap& parameters, std::string& error) {
  static constexpr std::array<std::string_view, 7> keys = {
      "sensor_type", "target_link", "father_link", "accelerometer",
      "gyro",        "orientation", "position"};
  if (!CheckKeys(parameters, keys, "IMU '" + component.name + "'", error)) return nullptr;
  bool debug = false;
  if (!DebugEnabled(parameters, debug, error)) {
    error = "IMU '" + component.name + "': " + error;
    return nullptr;
  }
  const auto acceleration = Get(parameters, "accelerometer");
  const auto gyro = Get(parameters, "gyro");
  const auto child = Get(parameters, "target_link");
  const auto parent = Get(parameters, "father_link");
  if (acceleration.empty() || gyro.empty() || !IsFrameId(child) || !IsFrameId(parent) ||
      child == parent) {
    Fail("IMU '" + component.name +
             "' requires accelerometer, gyro, and distinct valid target_link/father_link",
         error);
    return nullptr;
  }
  const auto orientation = Get(parameters, "orientation");
  const auto position = Get(parameters, "position");
  const bool has_orientation = parameters.contains("orientation");
  const bool has_position = parameters.contains("position");
  if (has_orientation != has_position) {
    Fail("IMU '" + component.name + "' orientation and position must be configured together",
         error);
    return nullptr;
  }
  if (has_orientation && (orientation.empty() || position.empty())) {
    Fail("IMU '" + component.name + "' orientation and position must not be empty", error);
    return nullptr;
  }

  std::vector<StateBinding> states;
  for (std::size_t index = 0; index < 3; ++index) {
    states.push_back({component.name, "linear_acceleration." + std::string(1, "xyz"[index]),
                      acceleration, index, 3});
  }
  for (std::size_t index = 0; index < 3; ++index) {
    states.push_back(
        {component.name, "angular_velocity." + std::string(1, "xyz"[index]), gyro, index, 3});
  }
  if (has_orientation) {
    constexpr std::array<std::size_t, 4> kRosFromMujoco = {1, 2, 3, 0};
    for (std::size_t index = 0; index < 4; ++index) {
      states.push_back({component.name, "orientation." + std::string(1, "xyzw"[index]), orientation,
                        kRosFromMujoco[index], 4});
    }
    for (std::size_t index = 0; index < 3; ++index) {
      states.push_back(
          {component.name, "position." + std::string(1, "xyz"[index]), position, index, 3});
    }
  }
  return std::make_unique<ImuResource>(component.name, debug, parent, child, std::move(states),
                                       has_orientation);
}

}  // namespace

std::string StateBinding::FullName() const {
  return component + "/" + interface_name;
}
std::string CommandBinding::FullName() const {
  return component + "/" + interface_name;
}

bool InterfaceRegistry::RegisterResource(std::span<StateBinding> states,
                                         std::span<CommandBinding> commands,
                                         std::span<const std::string> exclusive_topics,
                                         std::string& error) {
  error.clear();
  struct LocalObservation {
    std::size_t dimension = 0;
    std::vector<bool> components;
  };
  std::unordered_map<std::string, LocalObservation> local_observations;
  std::vector<std::string> local_observation_order;
  std::unordered_set<std::string> local_commands;
  std::unordered_set<std::string> local_state_interfaces;
  std::unordered_set<std::string> local_command_interfaces;
  std::unordered_set<std::string> local_topics;

  for (const auto& binding : states) {
    if (binding.resource_name.empty() || binding.resource_dimension == 0 ||
        binding.resource_component >= binding.resource_dimension) {
      return Fail(
          "observation resource '" + binding.resource_name + "' has invalid configured dimensions",
          error);
    }
    const auto full_name = binding.FullName();
    if (!local_state_interfaces.insert(full_name).second || state_interfaces_.contains(full_name)) {
      return Fail("ROS state interface '" + full_name + "' is duplicated", error);
    }
    const auto [found, inserted] = local_observations.try_emplace(
        binding.resource_name,
        LocalObservation{binding.resource_dimension,
                         std::vector<bool>(binding.resource_dimension, false)});
    if (inserted) {
      local_observation_order.push_back(binding.resource_name);
    } else if (found->second.dimension != binding.resource_dimension) {
      return Fail("observation resource '" + binding.resource_name +
                      "' is mapped with conflicting dimensions",
                  error);
    }
    if (found->second.components[binding.resource_component]) {
      return Fail("observation resource '" + binding.resource_name + "' is mapped more than once",
                  error);
    }
    found->second.components[binding.resource_component] = true;
  }
  for (const auto& name : local_observation_order) {
    const auto& components = local_observations.at(name).components;
    if (!std::all_of(components.begin(), components.end(), [](bool value) { return value; })) {
      return Fail("observation resource '" + name + "' is not mapped completely", error);
    }
    if (observations_.contains(name)) {
      return Fail("observation resource '" + name + "' is mapped more than once", error);
    }
  }

  for (const auto& binding : commands) {
    const auto full_name = binding.FullName();
    if (binding.resource_name.empty() || !local_commands.insert(binding.resource_name).second ||
        commands_.contains(binding.resource_name)) {
      return Fail("command resource '" + binding.resource_name + "' is mapped more than once",
                  error);
    }
    if (!local_command_interfaces.insert(full_name).second ||
        command_interfaces_.contains(full_name)) {
      return Fail("ROS command interface '" + full_name + "' is duplicated", error);
    }
  }
  for (const auto& topic : exclusive_topics) {
    if (topic.empty() || !local_topics.insert(topic).second || exclusive_topics_.contains(topic)) {
      return Fail("exclusive ROS publisher topic '" + topic + "' is duplicated", error);
    }
  }

  for (const auto& name : local_observation_order) {
    observations_.insert(name);
    manifest_.observation_names.push_back(name);
  }
  for (auto& binding : states) {
    const auto full_name = binding.FullName();
    state_interfaces_.insert(full_name);
    binding.state_index = manifest_.state_names.size();
    manifest_.state_names.push_back(full_name);
    manifest_.state_descriptions.push_back(Description(binding));
  }
  for (auto& binding : commands) {
    const auto full_name = binding.FullName();
    commands_.insert(binding.resource_name);
    command_interfaces_.insert(full_name);
    binding.command_index = manifest_.command_interface_names.size();
    manifest_.command_names.push_back(binding.resource_name);
    manifest_.command_interface_names.push_back(full_name);
    manifest_.command_descriptions.push_back(Description(binding));
  }
  exclusive_topics_.insert(local_topics.begin(), local_topics.end());
  return true;
}

InterfaceManifest InterfaceRegistry::ReleaseManifest() noexcept {
  return std::move(manifest_);
}

std::unique_ptr<ResourceInterface> MakeResource(const hardware_interface::ComponentInfo& component,
                                                const ParameterMap& parameters,
                                                std::string& error) {
  error.clear();
  if (component.name.empty()) {
    Fail("component name must not be empty", error);
    return nullptr;
  }
  if (component.type == "joint") return MakeJoint(component, parameters, error);
  if (component.type != "sensor") {
    Fail("unsupported component type '" + component.type + "' for '" + component.name + "'", error);
    return nullptr;
  }
  const auto sensor_type = Get(parameters, "sensor_type");
  if (sensor_type.empty()) {
    Fail("sensor '" + component.name + "' requires sensor_type ('imu' or 'other')", error);
    return nullptr;
  }
  if (sensor_type == "imu") return MakeImu(component, parameters, error);
  if (sensor_type == "other") return MakeGenericSensor(component, parameters, error);
  Fail("sensor '" + component.name + "' sensor_type must be exactly 'imu' or 'other'", error);
  return nullptr;
}

bool ParseHardwareOptions(const ParameterMap& parameters, HardwareOptions& options,
                          std::string& error) {
  error.clear();
  for (const auto& [key, value] : parameters) {
    (void)value;
    if (std::find(kCommonKeys.begin(), kCommonKeys.end(), key) == kCommonKeys.end()) {
      return Fail("hardware has unknown parameter '" + key + "'", error);
    }
  }
  HardwareOptions parsed;
  if (parameters.contains("server_id")) parsed.server_id = parameters.at("server_id");
  if (parameters.contains("adapter_id")) parsed.adapter_id = parameters.at("adapter_id");
  std::string identifier_error;
  if (!protocol::ValidateIdentifier(parsed.server_id, protocol::IdentifierKind::kServer,
                                    &identifier_error)) {
    return Fail("server_id is invalid: " + identifier_error, error);
  }
  if (!protocol::ValidateIdentifier(parsed.adapter_id, protocol::IdentifierKind::kAdapter,
                                    &identifier_error)) {
    return Fail("adapter_id is invalid: " + identifier_error, error);
  }
  if (parameters.contains("server_time_out_ms") &&
      !ParsePositiveUint32(parameters.at("server_time_out_ms"), "server_time_out_ms",
                           parsed.server_timeout_ms, error)) {
    return false;
  }
  if (parameters.contains("publish_sim_time") &&
      !ParseBool(parameters.at("publish_sim_time"), "publish_sim_time", parsed.publish_sim_time,
                 error)) {
    return false;
  }
  if (parameters.contains("debug")) {
    bool debug = false;
    if (!ParseBool(parameters.at("debug"), "debug", debug, error)) return false;
  }
  options = std::move(parsed);
  return true;
}

bool ValidateLayout(const adapter_sdk::ControllerLayout& layout,
                    const std::vector<std::unique_ptr<ResourceInterface>>& resources,
                    std::string& error) {
  error.clear();
  std::unordered_map<std::string, std::size_t> observations;
  std::unordered_map<std::string, std::size_t> commands;
  for (const auto& resource : resources) {
    for (const auto& binding : resource->state_bindings()) {
      observations.insert_or_assign(binding.resource_name, binding.resource_dimension);
    }
    for (const auto& binding : resource->command_bindings()) {
      commands.insert_or_assign(binding.resource_name, 1);
    }
  }
  if (layout.observations().size() != observations.size() ||
      layout.commands().size() != commands.size()) {
    return Fail("registered layout resource count does not match requested resources", error);
  }
  const auto validate = [&](const auto& metadata, const auto& expected, std::size_t scalar_count,
                            std::string_view kind, bool validate_command_range) {
    std::vector<bool> occupied(scalar_count);
    std::unordered_set<std::string> names;
    for (const auto& info : metadata) {
      const auto found = expected.find(info.name);
      if (!names.insert(info.name).second) {
        return Fail(std::string(kind) + " layout duplicates resource '" + info.name + "'", error);
      }
      if (found == expected.end() || info.scalars.count != found->second ||
          info.scalars.offset > scalar_count ||
          info.scalars.count > scalar_count - info.scalars.offset) {
        return Fail(std::string(kind) + " layout mismatch for resource '" + info.name + "'", error);
      }
      if (validate_command_range && info.has_command_range &&
          (!std::isfinite(info.command_min) || !std::isfinite(info.command_max) ||
           info.command_min > info.command_max)) {
        return Fail("command resource '" + info.name + "' has an invalid range", error);
      }
      for (std::size_t index = info.scalars.offset;
           index < info.scalars.offset + info.scalars.count; ++index) {
        if (occupied[index]) {
          return Fail(std::string(kind) + " layout contains overlapping spans", error);
        }
        occupied[index] = true;
      }
    }
    if (!std::all_of(occupied.begin(), occupied.end(), [](bool value) { return value; })) {
      return Fail(std::string(kind) + " layout does not cover its dense scalar array", error);
    }
    return true;
  };
  return validate(layout.observations(), observations, layout.observation_scalar_count(),
                  "observation", false) &&
         validate(layout.commands(), commands, layout.command_scalar_count(), "command", true);
}

}  // namespace mujoco_sim::hardware::internal
