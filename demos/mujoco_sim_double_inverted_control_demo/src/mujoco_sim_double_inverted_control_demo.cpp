#include "mujoco_sim_double_inverted_control_demo/mujoco_sim_double_inverted_control_demo.hpp"

#include <iostream>
#include <stdexcept>
#include <string_view>

namespace mujoco_sim_double_inverted_control_demo {

bool DoubleInvertedController::RegisterController() {
  std::string error;

  controller_client_ =
      std::make_unique<mujoco_sim::adapter_sdk::ControllerClient>(controller_options_);

  if (!controller_client_->Start(&error) || !controller_client_->Activate(&error)) {
    std::cerr << "Failed to start controller client: " << error << std::endl;
    return false;
  }

  const auto& layout = controller_client_->layout();
  // log command layout
  std::cout << "Command layout: " << std::endl;
  for (std::size_t i = 0; i < 2; ++i) {
    {
      const auto command_span = layout.Command(server_config_->actuators_name[i]);
      if (!command_span || command_span->count != 1) {
        std::cerr << "Command " << server_config_->actuators_name[i]
                  << " is missing or is not scalar" << std::endl;
        return false;
      }
      std::cout << "Command " << i << ": offset=" << command_span->offset
                << ", count=" << command_span->count << std::endl;
      torque_cmd_span[i] = *command_span;
    }  // control layout span

    const auto state_layout_fit = [&layout](std::string_view name,
                                            mujoco_sim::adapter_sdk::ScalarSpan& output) {
      const auto observation_span = layout.Observation(name);
      if (!observation_span || observation_span->count != 1) {
        std::cerr << "Observation " << name << " is missing or is not scalar" << std::endl;
        return false;
      }
      std::cout << "Observation " << name << ": offset=" << observation_span->offset
                << ", count=" << observation_span->count << std::endl;
      output = *observation_span;
      return true;
    };

    if (!state_layout_fit(server_config_->joints_pos_name[i], pos_span[i]) ||
        !state_layout_fit(server_config_->joints_vel_name[i], vel_span[i]) ||
        !state_layout_fit(server_config_->joints_torque_name[i], torque_span[i])) {
      return false;
    }
  }

  return true;
}

DoubleInvertedController::DoubleInvertedController(std::unique_ptr<ServerConfig> server_config)
    : controller_options_({}), server_config_(std::move(server_config)) {
  if (!server_config_ || !check_config_validity()) {
    std::cerr << "Invalid server config" << std::endl;
    throw std::runtime_error("Invalid server config");
  } else {
    std::cout << "Valid server config" << std::endl;
  }

  controller_options_.adapter_id = server_config_->adapter_id;
  controller_options_.server_name = server_config_->server_name;

  // 校验config

  controller_options_.command_names = server_config_->actuators_name;
  controller_options_.observation_names = [this]() {
    std::vector<std::string> names;
    names.push_back(server_config_->joints_pos_name[0]);
    names.push_back(server_config_->joints_pos_name[1]);
    names.push_back(server_config_->joints_vel_name[0]);
    names.push_back(server_config_->joints_vel_name[1]);
    names.push_back(server_config_->joints_torque_name[0]);
    names.push_back(server_config_->joints_torque_name[1]);
    return names;
  }();
  controller_options_.service_timeout_ms = 5000;
}

DoubleInvertedController::~DoubleInvertedController() {
  if (controller_client_) {
    std::string error;
    if (!controller_client_->Unregister(&error)) {
      std::cerr << "Failed to unregister controller client: " << error << std::endl;
    }
  }
}

bool DoubleInvertedController::check_config_validity() {
  // check empty
  if (server_config_->adapter_id.empty() || server_config_->server_name.empty()) {
    std::cerr << "Adapter ID or Server Name is empty" << std::endl;
    return false;
  }

  // check double control interface names
  const auto& check_names = [](const std::vector<std::string>& names, const std::string& type) {
    if (names.size() != 2) {
      std::cerr << "Expected 2 " << type << " names, got " << names.size() << std::endl;
      return false;
    }
    for (const auto& name : names) {
      if (name.empty()) {
        std::cerr << type << " name is empty" << std::endl;
        return false;
      }
    }
    return true;
  };

  if (!check_names(server_config_->actuators_name, "actuator") ||
      !check_names(server_config_->joints_pos_name, "joint position") ||
      !check_names(server_config_->joints_vel_name, "joint velocity") ||
      !check_names(server_config_->joints_torque_name, "joint torque")) {
    return false;
  }

  return true;
}

}  // namespace mujoco_sim_double_inverted_control_demo
