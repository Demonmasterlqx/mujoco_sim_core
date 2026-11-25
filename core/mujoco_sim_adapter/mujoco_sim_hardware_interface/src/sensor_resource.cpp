#include "resource_types.hpp"

#include <cmath>
#include <exception>
#include <utility>

#include <tf2_ros/qos.hpp>

namespace mujoco_sim::hardware::internal {
namespace {

bool Fail(std::string message, std::string& error) {
  error = std::move(message);
  return false;
}

std::unique_ptr<ArrayPublisher> MakeArrayPublisher(const rclcpp::Node::SharedPtr& node,
                                                   const std::string& topic, std::size_t size,
                                                   const std::string& label) {
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  auto publisher = std::make_unique<ArrayPublisher>(
      node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, qos));
  publisher->msg_.layout.dim.resize(1);
  publisher->msg_.layout.dim[0].label = label;
  publisher->msg_.layout.dim[0].size = static_cast<std::uint32_t>(size);
  publisher->msg_.layout.dim[0].stride = static_cast<std::uint32_t>(size);
  publisher->msg_.data.resize(size);
  return publisher;
}

void PublishArray(const std::unique_ptr<ArrayPublisher>& publisher,
                  const std::vector<StateBinding>& states, std::span<const double> staged_state) {
  if (!publisher || !publisher->trylock()) return;
  for (std::size_t index = 0; index < states.size(); ++index) {
    publisher->msg_.data[index] = staged_state[states[index].state_index];
  }
  publisher->unlockAndPublish();
}

}  // namespace

GenericSensorResource::GenericSensorResource(std::string name, bool debug,
                                             std::vector<StateBinding> states)
    : name_(std::move(name)), debug_(debug), states_(std::move(states)) {}

const std::string& GenericSensorResource::name() const noexcept {
  return name_;
}
std::vector<std::string> GenericSensorResource::ExclusivePublisherTopics() const {
  return debug_ ? std::vector<std::string>{"~/debug/sensors/" + name_} : std::vector<std::string>{};
}
bool GenericSensorResource::RegisterInterfaces(InterfaceRegistry& registry, std::string& error) {
  const auto topics = ExclusivePublisherTopics();
  return registry.RegisterResource(states_, commands_, topics, error);
}
std::span<const StateBinding> GenericSensorResource::state_bindings() const noexcept {
  return states_;
}
std::span<const CommandBinding> GenericSensorResource::command_bindings() const noexcept {
  return commands_;
}

bool GenericSensorResource::Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) {
  return BindStates(layout, states_, error);
}

bool GenericSensorResource::StageState(std::span<const double> observations,
                                       std::span<double> staged_state, std::string& error) const {
  return ReadBindings(states_, observations, staged_state, error);
}

bool GenericSensorResource::StageCommand(std::span<double> ros_commands,
                                         std::span<double> dense_command,
                                         std::string& error) const {
  (void)ros_commands;
  (void)dense_command;
  error.clear();
  return true;
}

bool GenericSensorResource::ConfigurePublishers(const rclcpp::Node::SharedPtr& node,
                                                std::string& error) {
  ResetPublishers();
  if (!debug_) return true;
  if (node == nullptr) return Fail("debug publishing requires a hardware component node", error);
  try {
    debug_publisher_ = MakeArrayPublisher(node, "~/debug/sensors/" + name_, states_.size(), name_);
  } catch (const std::exception& exception) {
    ResetPublishers();
    return Fail("failed to create sensor debug publisher for '" + name_ + "': " + exception.what(),
                error);
  }
  return true;
}

void GenericSensorResource::ResetPublishers() noexcept {
  debug_publisher_.reset();
}

void GenericSensorResource::OnStateCommitted(std::span<const double> state_values,
                                             const builtin_interfaces::msg::Time& stamp) noexcept {
  (void)stamp;
  try {
    PublishArray(debug_publisher_, states_, state_values);
  } catch (...) {
  }
}

void GenericSensorResource::OnCommandAccepted(std::span<const double> command_values) noexcept {
  (void)command_values;
}

ImuResource::ImuResource(std::string name, bool debug, std::string parent_frame,
                         std::string child_frame, std::vector<StateBinding> states, bool has_pose)
    : name_(std::move(name)),
      debug_(debug),
      parent_frame_(std::move(parent_frame)),
      child_frame_(std::move(child_frame)),
      has_pose_(has_pose),
      states_(std::move(states)) {}

const std::string& ImuResource::name() const noexcept {
  return name_;
}
std::vector<std::string> ImuResource::ExclusivePublisherTopics() const {
  return debug_ ? std::vector<std::string>{"~/debug/imus/" + name_} : std::vector<std::string>{};
}
bool ImuResource::RegisterInterfaces(InterfaceRegistry& registry, std::string& error) {
  const auto topics = ExclusivePublisherTopics();
  return registry.RegisterResource(states_, commands_, topics, error);
}
std::span<const StateBinding> ImuResource::state_bindings() const noexcept {
  return states_;
}
std::span<const CommandBinding> ImuResource::command_bindings() const noexcept {
  return commands_;
}

bool ImuResource::Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) {
  return BindStates(layout, states_, error);
}

bool ImuResource::StageState(std::span<const double> observations, std::span<double> staged_state,
                             std::string& error) const {
  if (!ValidateReadBindings(states_, observations, staged_state, error)) return false;
  double quaternion_norm = 1.0;
  if (has_pose_) {
    const double x = observations[states_[6].dense_offset];
    const double y = observations[states_[7].dense_offset];
    const double z = observations[states_[8].dense_offset];
    const double w = observations[states_[9].dense_offset];
    quaternion_norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-12) {
      return Fail("IMU '" + name_ + "' orientation is not a finite non-zero quaternion", error);
    }
  }
  CopyReadBindings(states_, observations, staged_state);
  if (!has_pose_) return true;
  for (std::size_t index = 6; index < 10; ++index) {
    staged_state[states_[index].state_index] /= quaternion_norm;
  }
  return true;
}

bool ImuResource::StageCommand(std::span<double> ros_commands, std::span<double> dense_command,
                               std::string& error) const {
  (void)ros_commands;
  (void)dense_command;
  error.clear();
  return true;
}

bool ImuResource::ConfigurePublishers(const rclcpp::Node::SharedPtr& node, std::string& error) {
  ResetPublishers();
  if (!debug_ && !has_pose_) return true;
  if (node == nullptr) return Fail("IMU publishing requires a hardware component node", error);
  try {
    if (debug_) {
      debug_publisher_ = MakeArrayPublisher(node, "~/debug/imus/" + name_, states_.size(), name_);
    }
    if (has_pose_) {
      transform_publisher_ =
          std::make_unique<TfPublisher>(node->create_publisher<tf2_msgs::msg::TFMessage>(
              "/tf", tf2_ros::DynamicBroadcasterQoS()));
      transform_publisher_->msg_.transforms.resize(1);
      auto& transform = transform_publisher_->msg_.transforms.front();
      transform.header.frame_id = parent_frame_;
      transform.child_frame_id = child_frame_;
    }
  } catch (const std::exception& exception) {
    ResetPublishers();
    return Fail("failed to create IMU publishers for '" + name_ + "': " + exception.what(), error);
  }
  return true;
}

void ImuResource::ResetPublishers() noexcept {
  debug_publisher_.reset();
  transform_publisher_.reset();
}

void ImuResource::OnStateCommitted(std::span<const double> state_values,
                                   const builtin_interfaces::msg::Time& stamp) noexcept {
  try {
    PublishArray(debug_publisher_, states_, state_values);
    if (!transform_publisher_ || !transform_publisher_->trylock()) return;
    auto& transform = transform_publisher_->msg_.transforms.front();
    transform.header.stamp = stamp;
    transform.transform.rotation.x = state_values[states_[6].state_index];
    transform.transform.rotation.y = state_values[states_[7].state_index];
    transform.transform.rotation.z = state_values[states_[8].state_index];
    transform.transform.rotation.w = state_values[states_[9].state_index];
    transform.transform.translation.x = state_values[states_[10].state_index];
    transform.transform.translation.y = state_values[states_[11].state_index];
    transform.transform.translation.z = state_values[states_[12].state_index];
    transform_publisher_->unlockAndPublish();
  } catch (...) {
  }
}

void ImuResource::OnCommandAccepted(std::span<const double> command_values) noexcept {
  (void)command_values;
}

}  // namespace mujoco_sim::hardware::internal
