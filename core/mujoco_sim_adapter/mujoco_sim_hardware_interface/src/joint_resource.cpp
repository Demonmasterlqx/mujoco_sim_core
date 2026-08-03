#include "resource_types.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace mujoco_sim::hardware::internal {
namespace {

bool Fail(std::string message, std::string& error) {
  error = std::move(message);
  return false;
}

template <class Metadata>
std::optional<ResourceInfoReference> Find(const Metadata& resources, const std::string& name) {
  const auto found = std::find_if(resources.begin(), resources.end(),
                                  [&](const auto& resource) { return resource.name == name; });
  if (found == resources.end()) return std::nullopt;
  return std::cref(*found);
}

std::unique_ptr<ScalarPublisher> MakeScalarPublisher(const rclcpp::Node::SharedPtr& node,
                                                     const std::string& topic) {
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  return std::make_unique<ScalarPublisher>(
      node->create_publisher<std_msgs::msg::Float64>(topic, qos));
}

void PublishScalar(const std::unique_ptr<ScalarPublisher>& publisher, double value) {
  if (publisher && publisher->trylock()) {
    publisher->msg_.data = value;
    publisher->unlockAndPublish();
  }
}

}  // namespace

std::optional<ResourceInfoReference> FindObservation(const adapter_sdk::ControllerLayout& layout,
                                                     const std::string& name) {
  return Find(layout.observations(), name);
}

std::optional<ResourceInfoReference> FindCommand(const adapter_sdk::ControllerLayout& layout,
                                                 const std::string& name) {
  return Find(layout.commands(), name);
}

bool BindStates(const adapter_sdk::ControllerLayout& layout, std::vector<StateBinding>& states,
                std::string& error) {
  for (auto& binding : states) {
    const auto metadata = FindObservation(layout, binding.resource_name);
    if (!metadata || metadata->get().scalars.count != binding.resource_dimension ||
        binding.resource_component >= metadata->get().scalars.count) {
      return Fail("observation resource '" + binding.resource_name +
                      "' is missing or has the wrong dimension",
                  error);
    }
    binding.dense_offset = metadata->get().scalars.offset + binding.resource_component;
  }
  return true;
}

bool BindCommands(const adapter_sdk::ControllerLayout& layout,
                  std::vector<CommandBinding>& commands, std::string& error) {
  for (auto& binding : commands) {
    const auto metadata = FindCommand(layout, binding.resource_name);
    if (!metadata || metadata->get().scalars.count != 1) {
      return Fail("command resource '" + binding.resource_name + "' is missing or is not scalar",
                  error);
    }
    const auto& resource_info = metadata->get();
    if (resource_info.has_command_range &&
        (!std::isfinite(resource_info.command_min) || !std::isfinite(resource_info.command_max) ||
         resource_info.command_min > resource_info.command_max)) {
      return Fail("command resource '" + binding.resource_name + "' has an invalid range", error);
    }
    binding.dense_offset = resource_info.scalars.offset;
    binding.has_range = resource_info.has_command_range;
    binding.minimum = resource_info.command_min;
    binding.maximum = resource_info.command_max;
  }
  return true;
}

bool ValidateReadBindings(const std::vector<StateBinding>& states,
                          std::span<const double> observations,
                          std::span<const double> staged_state, std::string& error) {
  for (const auto& binding : states) {
    if (binding.dense_offset >= observations.size() || binding.state_index >= staged_state.size()) {
      return Fail("dense state offset is out of bounds for '" + binding.FullName() + "'", error);
    }
    const double value = observations[binding.dense_offset];
    if (!std::isfinite(value)) {
      return Fail("non-finite state value for '" + binding.FullName() + "'", error);
    }
  }
  return true;
}

void CopyReadBindings(const std::vector<StateBinding>& states, std::span<const double> observations,
                      std::span<double> staged_state) noexcept {
  for (const auto& binding : states) {
    staged_state[binding.state_index] = observations[binding.dense_offset];
  }
}

bool ReadBindings(const std::vector<StateBinding>& states, std::span<const double> observations,
                  std::span<double> staged_state, std::string& error) {
  if (!ValidateReadBindings(states, observations, staged_state, error)) return false;
  CopyReadBindings(states, observations, staged_state);
  return true;
}

JointResource::JointResource(std::string name, bool debug, std::vector<StateBinding> states,
                             std::vector<CommandBinding> commands)
    : name_(std::move(name)),
      debug_(debug),
      states_(std::move(states)),
      commands_(std::move(commands)) {}

const std::string& JointResource::name() const noexcept {
  return name_;
}
std::vector<std::string> JointResource::ExclusivePublisherTopics() const {
  if (!debug_) return {};
  std::vector<std::string> topics;
  topics.reserve(states_.size() + commands_.size());
  for (const auto& binding : states_) {
    topics.push_back("~/debug/joints/" + name_ + "/state/" + binding.interface_name);
  }
  for (const auto& binding : commands_) {
    topics.push_back("~/debug/joints/" + name_ + "/command/" + binding.interface_name);
  }
  return topics;
}
bool JointResource::RegisterInterfaces(InterfaceRegistry& registry, std::string& error) {
  const auto topics = ExclusivePublisherTopics();
  return registry.RegisterResource(states_, commands_, topics, error);
}
std::span<const StateBinding> JointResource::state_bindings() const noexcept {
  return states_;
}
std::span<const CommandBinding> JointResource::command_bindings() const noexcept {
  return commands_;
}

bool JointResource::Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) {
  return BindStates(layout, states_, error) && BindCommands(layout, commands_, error);
}

bool JointResource::StageState(std::span<const double> observations, std::span<double> staged_state,
                               std::string& error) const {
  return ReadBindings(states_, observations, staged_state, error);
}

bool JointResource::StageCommand(std::span<double> ros_commands, std::span<double> dense_command,
                                 std::string& error) const {
  for (const auto& binding : commands_) {
    if (binding.command_index >= ros_commands.size() ||
        binding.dense_offset >= dense_command.size()) {
      return Fail("dense command offset is out of bounds for '" + binding.FullName() + "'", error);
    }
  }
  for (const auto& binding : commands_) {
    double value = ros_commands[binding.command_index];
    if (!std::isfinite(value)) value = 0.0;
    if (binding.has_range) value = std::clamp(value, binding.minimum, binding.maximum);
    ros_commands[binding.command_index] = value;
    dense_command[binding.dense_offset] = value;
  }
  return true;
}

bool JointResource::ConfigurePublishers(const rclcpp::Node::SharedPtr& node, std::string& error) {
  ResetPublishers();
  if (!debug_) return true;
  if (node == nullptr) return Fail("debug publishing requires a hardware component node", error);
  try {
    for (const auto& binding : states_) {
      state_publishers_.push_back(MakeScalarPublisher(
          node, "~/debug/joints/" + name_ + "/state/" + binding.interface_name));
    }
    for (const auto& binding : commands_) {
      command_publishers_.push_back(MakeScalarPublisher(
          node, "~/debug/joints/" + name_ + "/command/" + binding.interface_name));
    }
  } catch (const std::exception& exception) {
    ResetPublishers();
    return Fail("failed to create joint debug publisher for '" + name_ + "': " + exception.what(),
                error);
  }
  return true;
}

void JointResource::ResetPublishers() noexcept {
  state_publishers_.clear();
  command_publishers_.clear();
}

void JointResource::OnStateCommitted(std::span<const double> state_values,
                                     const builtin_interfaces::msg::Time& stamp) noexcept {
  (void)stamp;
  try {
    for (std::size_t index = 0; index < state_publishers_.size(); ++index) {
      PublishScalar(state_publishers_[index], state_values[states_[index].state_index]);
    }
  } catch (...) {
  }
}

void JointResource::OnCommandAccepted(std::span<const double> command_values) noexcept {
  try {
    for (std::size_t index = 0; index < command_publishers_.size(); ++index) {
      PublishScalar(command_publishers_[index], command_values[commands_[index].command_index]);
    }
  } catch (...) {
  }
}

}  // namespace mujoco_sim::hardware::internal
