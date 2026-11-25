#include <mujoco_sim_hardware_interface/mujoco_sim_system.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <rclcpp/qos.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "resource.hpp"

namespace mujoco_sim::hardware {
namespace {

using ClockPublisher = realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>;

bool HasExplicitInterfaces(const hardware_interface::HardwareInfo& info, std::string& error) {
  for (const auto& joint : info.joints) {
    if (!joint.state_interfaces.empty() || !joint.command_interfaces.empty()) {
      error = "joint '" + joint.name +
              "' must not declare state_interface or command_interface; mappings auto-export them";
      return true;
    }
  }
  for (const auto& sensor : info.sensors) {
    if (!sensor.state_interfaces.empty() || !sensor.command_interfaces.empty()) {
      error = "sensor '" + sensor.name +
              "' must not declare state_interface or command_interface; mappings auto-export them";
      return true;
    }
  }
  return false;
}

bool HasSystemParameterOverride(const hardware_interface::ComponentInfo& component,
                                std::string& error) {
  static constexpr std::array<std::string_view, 4> kSystemOnlyParameters = {
      "server_id", "adapter_id", "server_time_out_ms", "publish_sim_time"};
  for (const auto parameter : kSystemOnlyParameters) {
    if (component.parameters.contains(std::string(parameter))) {
      error = component.type + " '" + component.name + "' parameter '" + std::string(parameter) +
              "' is hardware-only; only 'debug' may override a hardware parameter per component";
      return true;
    }
  }
  return false;
}

}  // namespace

bool MujocoSimSystem::Initialize(const hardware_interface::HardwareInfo& info, std::string& error) {
  if (info.type != "system") {
    error = "mujoco_sim_hardware_interface requires ros2_control type='system'";
    return false;
  }
  if (HasExplicitInterfaces(info, error)) return false;
  if (!info.gpios.empty()) {
    error = "GPIO components are not supported";
    return false;
  }
  internal::HardwareOptions options;
  if (!internal::ParseHardwareOptions(info.hardware_parameters, options, error)) return false;
  server_id_ = std::move(options.server_id);
  adapter_id_ = std::move(options.adapter_id);
  server_timeout_ms_ = options.server_timeout_ms;
  publish_sim_time_ = options.publish_sim_time;
  internal::InterfaceRegistry registry;
  auto add_component = [&](const hardware_interface::ComponentInfo& component) {
    if (HasSystemParameterOverride(component, error)) return false;
    internal::ParameterMap merged = info.hardware_parameters;
    for (const auto& [key, value] : component.parameters)
      merged.insert_or_assign(key, value);
    auto resource = internal::MakeResource(component, merged, error);
    if (resource == nullptr) return false;
    if (!resource->RegisterInterfaces(registry, error)) return false;
    resources_.push_back(std::move(resource));
    return true;
  };
  for (const auto& joint : info.joints) {
    if (!add_component(joint)) return false;
  }
  for (const auto& sensor : info.sensors) {
    if (!add_component(sensor)) return false;
  }
  auto manifest = registry.ReleaseManifest();
  if (manifest.command_names.empty()) {
    error = "at least one actuator command mapping is required for ControllerClient";
    return false;
  }
  state_descriptions_ = std::move(manifest.state_descriptions);
  command_descriptions_ = std::move(manifest.command_descriptions);
  observation_names_ = std::move(manifest.observation_names);
  command_names_ = std::move(manifest.command_names);
  state_names_ = std::move(manifest.state_names);
  command_names_ros_ = std::move(manifest.command_interface_names);
  state_staging_.resize(state_names_.size());
  ros_commands_.resize(command_names_ros_.size());
  return true;
}

bool MujocoSimSystem::ConfigureRosOutputs(std::string& error) {
  ResetRosOutputs();
  const auto node = get_node();
  if (publish_sim_time_ && node == nullptr) {
    error =
        "/clock publishing requires the controller manager executor to provide a component node";
    return false;
  }
  try {
    for (auto& resource : resources_) {
      if (!resource->ConfigurePublishers(node, error)) {
        ResetRosOutputs();
        return false;
      }
    }
    if (publish_sim_time_) {
      clock_publisher_ = std::make_unique<ClockPublisher>(
          node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS()));
    }
  } catch (const std::exception& exception) {
    ResetRosOutputs();
    error = std::string("failed to create ROS publisher: ") + exception.what();
    return false;
  } catch (...) {
    ResetRosOutputs();
    error = "failed to create ROS publisher: unknown exception";
    return false;
  }
  configured_ = true;
  if (!InitializeHandles(error)) {
    configured_ = false;
    ResetRosOutputs();
    return false;
  }
  return true;
}

bool MujocoSimSystem::Activate(std::string& error) {
  if (!configured_ || client_ != nullptr) {
    error = "hardware must be configured and inactive before activation";
    return false;
  }
  adapter_sdk::ControllerOptions controller_options;
  controller_options.server_name = server_id_;
  controller_options.adapter_id = adapter_id_;
  controller_options.observation_names = observation_names_;
  controller_options.command_names = command_names_;
  controller_options.service_timeout_ms = server_timeout_ms_;
  auto candidate = std::make_unique<adapter_sdk::ControllerClient>(std::move(controller_options));
  if (!candidate->Start(&error)) return false;
  if (!internal::ValidateLayout(candidate->layout(), resources_, error)) {
    CleanupCandidate(*candidate, error);
    return false;
  }
  for (auto& resource : resources_) {
    if (!resource->Bind(candidate->layout(), error)) {
      CleanupCandidate(*candidate, error);
      return false;
    }
  }
  dense_command_.assign(candidate->layout().command_scalar_count(), 0.0);
  if (!InitializeBoundCommands(error)) {
    CleanupCandidate(*candidate, error);
    return false;
  }
  if (!candidate->Activate(&error)) {
    CleanupCandidate(*candidate, error);
    return false;
  }
  last_sequence_ = 0;
  fault_reported_ = false;
  client_ = std::move(candidate);
  active_ = true;
  return true;
}

bool MujocoSimSystem::Deactivate(std::string& error) {
  active_ = false;
  if (client_ == nullptr) return true;
  client_->Stop();
  if (!client_->Unregister(&error)) return false;
  client_.reset();
  last_sequence_ = 0;
  return true;
}

bool MujocoSimSystem::Cleanup(std::string& error) {
  const bool cleaned = Deactivate(error);
  if (!cleaned) return false;
  ResetRosOutputs();
  configured_ = false;
  return true;
}

void MujocoSimSystem::BestEffortCleanup() noexcept {
  try {
    active_ = false;
    if (client_ != nullptr) {
      std::string ignored;
      client_->Stop();
      (void)client_->Unregister(&ignored);
      client_.reset();
    }
    ResetRosOutputs();
  } catch (...) {
    try {
      if (client_ != nullptr) client_->Stop();
    } catch (...) {
    }
    client_.reset();
    ResetRosOutputs();
  }
}

MujocoSimSystem::MujocoSimSystem() = default;
MujocoSimSystem::~MujocoSimSystem() {
  BestEffortCleanup();
}

hardware_interface::CallbackReturn MujocoSimSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  try {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::string error;
    if (!Initialize(params.hardware_info, error)) {
      RCLCPP_ERROR(get_logger(), "hardware initialization failed: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware initialization threw: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware initialization threw an unknown exception");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::InterfaceDescription>
MujocoSimSystem::export_unlisted_state_interface_descriptions() {
  try {
    return state_descriptions_;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "exporting state descriptions threw: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "exporting state descriptions threw an unknown exception");
  }
  return {};
}

std::vector<hardware_interface::InterfaceDescription>
MujocoSimSystem::export_unlisted_command_interface_descriptions() {
  try {
    return command_descriptions_;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "exporting command descriptions threw: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "exporting command descriptions threw an unknown exception");
  }
  return {};
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
MujocoSimSystem::on_export_state_interfaces() {
  try {
    auto handles = hardware_interface::SystemInterface::on_export_state_interfaces();
    std::string error;
    if (!CacheStateHandles(handles, error)) {
      RCLCPP_ERROR(get_logger(), "failed to cache generated state interfaces: %s", error.c_str());
      return {};
    }
    return handles;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "exporting generated state interfaces threw: %s", exception.what());
    return {};
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "exporting generated state interfaces threw an unknown exception");
    return {};
  }
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
MujocoSimSystem::on_export_command_interfaces() {
  try {
    auto handles = hardware_interface::SystemInterface::on_export_command_interfaces();
    std::string error;
    if (!CacheCommandHandles(handles, error)) {
      RCLCPP_ERROR(get_logger(), "failed to cache generated command interfaces: %s", error.c_str());
      return {};
    }
    return handles;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "exporting generated command interfaces threw: %s",
                 exception.what());
    return {};
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "exporting generated command interfaces threw an unknown exception");
    return {};
  }
}

hardware_interface::CallbackReturn MujocoSimSystem::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  try {
    std::string error;
    if (!ConfigureRosOutputs(error)) {
      RCLCPP_ERROR(get_logger(), "hardware configuration failed: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware configuration threw: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware configuration threw an unknown exception");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MujocoSimSystem::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  try {
    std::string error;
    if (!Activate(error)) {
      RCLCPP_ERROR(get_logger(), "hardware activation failed: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware activation threw: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware activation threw an unknown exception");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MujocoSimSystem::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  try {
    std::string error;
    if (!Deactivate(error)) {
      RCLCPP_ERROR(get_logger(), "hardware deactivation failed: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware deactivation threw: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware deactivation threw an unknown exception");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MujocoSimSystem::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  try {
    std::string error;
    if (!Cleanup(error)) {
      RCLCPP_ERROR(get_logger(), "hardware cleanup failed: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware cleanup threw: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware cleanup threw an unknown exception");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MujocoSimSystem::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  return on_cleanup(previous_state);
}

hardware_interface::CallbackReturn MujocoSimSystem::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  return on_cleanup(previous_state);
}

hardware_interface::return_type MujocoSimSystem::read(const rclcpp::Time& time,
                                                      const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  try {
    return ReadActive();
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware read threw: %s", exception.what());
    return hardware_interface::return_type::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware read threw an unknown exception");
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type MujocoSimSystem::write(const rclcpp::Time& time,
                                                       const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  try {
    return WriteActive();
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "hardware write threw: %s", exception.what());
    return hardware_interface::return_type::ERROR;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "hardware write threw an unknown exception");
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace mujoco_sim::hardware
