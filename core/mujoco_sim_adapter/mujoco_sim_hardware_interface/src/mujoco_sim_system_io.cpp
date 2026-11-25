#include <mujoco_sim_hardware_interface/mujoco_sim_system.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

#include "resource.hpp"

namespace mujoco_sim::hardware {

hardware_interface::return_type MujocoSimSystem::ReadActive() {
  if (!active_) return hardware_interface::return_type::OK;
  adapter_sdk::StateLease state;
  std::string error;
  const auto result = client_->ReadLatestState(&state, &error);
  if (result == adapter_sdk::StateReadResult::kTimeout) return hardware_interface::return_type::OK;
  if (result != adapter_sdk::StateReadResult::kReceived) {
    if (error.empty()) error = "Controller State reception stopped while hardware was active";
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (state.sequence() <= last_sequence_) return hardware_interface::return_type::OK;
  if (!StageState(state, error)) {
    ReportFault(adapter_sdk::FaultCode::kInternalControllerError, error);
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!StoreStateValues(state_staging_, error)) {
    ReportFault(adapter_sdk::FaultCode::kInternalControllerError, error);
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  last_sequence_ = state.sequence();
  const auto nanoseconds =
      static_cast<std::int64_t>(static_cast<long double>(state.simulation_time_s()) * 1.0e9L);
  const builtin_interfaces::msg::Time stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
  for (auto& resource : resources_)
    resource->OnStateCommitted(state_staging_, stamp);
  PublishClock(stamp);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSimSystem::WriteActive() {
  if (!active_) return hardware_interface::return_type::OK;
  std::string error;
  if (!LoadCommandValues(ros_commands_, error)) {
    ReportFault(adapter_sdk::FaultCode::kInvalidOutput, error);
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  for (const auto& resource : resources_) {
    if (!resource->StageCommand(ros_commands_, dense_command_, error)) {
      ReportFault(adapter_sdk::FaultCode::kInvalidOutput, error);
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!client_->SetCommand(dense_command_, &error)) {
    RCLCPP_ERROR(get_logger(), "failed to submit dense command: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!StoreCommandValues(ros_commands_, error)) {
    RCLCPP_ERROR(get_logger(), "failed to store clamped command: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  for (auto& resource : resources_)
    resource->OnCommandAccepted(ros_commands_);
  return hardware_interface::return_type::OK;
}

bool MujocoSimSystem::CacheStateHandles(
    const std::vector<hardware_interface::StateInterface::ConstSharedPtr>& handles,
    std::string& error) {
  if (handles.size() != state_names_.size()) {
    error = "generated ROS state handle count does not match its descriptions";
    return false;
  }
  std::vector<hardware_interface::StateInterface::SharedPtr> cached;
  cached.reserve(handles.size());
  for (std::size_t index = 0; index < handles.size(); ++index) {
    if (handles[index] == nullptr || handles[index]->get_name() != state_names_[index]) {
      error = "generated ROS state handles do not preserve description order";
      return false;
    }
    cached.push_back(std::const_pointer_cast<hardware_interface::StateInterface>(handles[index]));
  }
  state_handles_ = std::move(cached);
  return true;
}

bool MujocoSimSystem::CacheCommandHandles(
    const std::vector<hardware_interface::CommandInterface::SharedPtr>& handles,
    std::string& error) {
  if (handles.size() != command_names_ros_.size()) {
    error = "generated ROS command handle count does not match its descriptions";
    return false;
  }
  for (std::size_t index = 0; index < handles.size(); ++index) {
    if (handles[index] == nullptr || handles[index]->get_name() != command_names_ros_[index]) {
      error = "generated ROS command handles do not preserve description order";
      return false;
    }
  }
  command_handles_ = handles;
  return true;
}

bool MujocoSimSystem::StoreStateValues(std::span<const double> values, std::string& error) {
  if (values.size() != state_handles_.size()) {
    error = "generated ROS state handles are unavailable";
    return false;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    auto& handle = state_handles_[index];
    std::unique_lock<std::shared_mutex> lock(handle->get_mutex());
    if (!handle->set_value(lock, values[index])) {
      error = "failed to store ROS state interface '" + state_names_[index] + "'";
      return false;
    }
  }
  return true;
}

bool MujocoSimSystem::LoadCommandValues(std::span<double> values, std::string& error) const {
  if (values.size() != command_handles_.size()) {
    error = "generated ROS command handles are unavailable";
    return false;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    const auto& handle = command_handles_[index];
    std::shared_lock<std::shared_mutex> lock(handle->get_mutex());
    const auto value = handle->get_optional<double>(lock);
    if (!value.has_value()) {
      error = "failed to load ROS command interface '" + command_names_ros_[index] + "'";
      return false;
    }
    values[index] = *value;
  }
  return true;
}

bool MujocoSimSystem::StoreCommandValues(std::span<const double> values, std::string& error) {
  if (values.size() != command_handles_.size()) {
    error = "generated ROS command handles are unavailable";
    return false;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    auto& handle = command_handles_[index];
    std::unique_lock<std::shared_mutex> lock(handle->get_mutex());
    if (!handle->set_value(lock, values[index])) {
      error = "failed to store ROS command interface '" + command_names_ros_[index] + "'";
      return false;
    }
  }
  return true;
}

bool MujocoSimSystem::InitializeHandles(std::string& error) {
  std::fill(state_staging_.begin(), state_staging_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(ros_commands_.begin(), ros_commands_.end(), 0.0);
  return StoreStateValues(state_staging_, error) && StoreCommandValues(ros_commands_, error);
}

bool MujocoSimSystem::InitializeBoundCommands(std::string& error) {
  std::fill(ros_commands_.begin(), ros_commands_.end(), 0.0);
  for (const auto& resource : resources_) {
    if (!resource->StageCommand(ros_commands_, dense_command_, error)) return false;
  }
  std::fill(state_staging_.begin(), state_staging_.end(), std::numeric_limits<double>::quiet_NaN());
  return StoreCommandValues(ros_commands_, error) && StoreStateValues(state_staging_, error);
}

bool MujocoSimSystem::StageState(const adapter_sdk::StateLease& state, std::string& error) {
  const long double simulation_time_ns =
      static_cast<long double>(state.simulation_time_s()) * 1.0e9L;
  if (!std::isfinite(state.simulation_time_s()) || state.simulation_time_s() < 0.0 ||
      simulation_time_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max()) ||
      state.observations().size() != expected_observation_count()) {
    error = "Controller State has an invalid timestamp or dense scalar count";
    return false;
  }
  for (const auto& resource : resources_) {
    if (!resource->StageState(state.observations(), state_staging_, error)) return false;
  }
  return true;
}

std::size_t MujocoSimSystem::expected_observation_count() const {
  if (client_ == nullptr) return 0;
  return client_->layout().observation_scalar_count();
}

void MujocoSimSystem::ReportFault(adapter_sdk::FaultCode code, const std::string& message) {
  if (fault_reported_ || client_ == nullptr) return;
  fault_reported_ = true;
  std::string fault_error;
  if (!client_->ReportFault(code, message, &fault_error)) {
    RCLCPP_ERROR(get_logger(), "failed to report Controller fault: %s", fault_error.c_str());
  }
}

void MujocoSimSystem::CleanupCandidate(adapter_sdk::ControllerClient& candidate,
                                       std::string& primary_error) {
  const std::string original = primary_error;
  std::string cleanup_error;
  candidate.Stop();
  if (!candidate.Unregister(&cleanup_error) && !cleanup_error.empty()) {
    primary_error = original + "; cleanup failed: " + cleanup_error;
  }
}

void MujocoSimSystem::PublishClock(const builtin_interfaces::msg::Time& stamp) {
  if (clock_publisher_ == nullptr || !clock_publisher_->trylock()) return;
  clock_publisher_->msg_.clock = stamp;
  clock_publisher_->unlockAndPublish();
}

void MujocoSimSystem::ResetRosOutputs() noexcept {
  clock_publisher_.reset();
  for (auto& resource : resources_)
    resource->ResetPublishers();
}

}  // namespace mujoco_sim::hardware
