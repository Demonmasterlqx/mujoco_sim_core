#include "mujoco_sim_double_inverted_control_demo/mujoco_sim_double_inverted_control_demo.hpp"

#include <iostream>
#include <chrono>
#include <thread>

namespace mujoco_sim_double_inverted_control_demo {

bool DoubleInvertedController::RunControlLoop() {
    if (!controller_client_) {
        std::cerr << "Controller client is not initialized" << std::endl;
        return false;
    }

    while (true) {
        if (!read()) {
            std::cerr << "Error reading from controller" << std::endl;
            return false;
        }
        if (!update()) {
            std::cerr << "Error updating controller" << std::endl;
            return false;
        }
        if (!write()) {
            std::cerr << "Error writing to controller" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::duration<double>(1.0 / static_cast<double>(server_config_->control_hz)));
    }
}

bool DoubleInvertedController::read() {
    mujoco_sim::adapter_sdk::StateLease state;
    std::string error;
    auto result = controller_client_->ReadLatestState(&state, &error);
    if (result == mujoco_sim::adapter_sdk::StateReadResult::kReceived) {
        // Process the received state
        joint_pos_[0] = state.observations()[pos_span[0].offset];
        joint_pos_[1] = state.observations()[pos_span[1].offset];
        joint_vel_[0] = state.observations()[vel_span[0].offset];
        joint_vel_[1] = state.observations()[vel_span[1].offset];
        joint_torque_[0] = state.observations()[torque_span[0].offset];
        joint_torque_[1] = state.observations()[torque_span[1].offset];
        return true;
    } else if (result == mujoco_sim::adapter_sdk::StateReadResult::kTimeout) {
        // No new state available, just return true to continue the loop
        return true;
    } else if (result == mujoco_sim::adapter_sdk::StateReadResult::kStopped) {
        return false;
    }
    std::cerr << "Error reading state: " << error << std::endl;
    return false;
}

bool DoubleInvertedController::update() {
    // Simple PD control for demonstration
    constexpr double kp = 20.0;
    constexpr double kd = 3.0;

    static int cnt = 0;

    for (std::size_t i = 0; i < 2; ++i) {
        joint_torque_cmd_[i] =
            -kp * (joint_pos_[i] - joint_pos_ref_[i]) - kd * (joint_vel_[i] - joint_vel_ref_[i]);
    }

    cnt++;
    if (cnt % 100 == 0) {
        std::cout << "Joint Pos: [" << joint_pos_[0] << ", " << joint_pos_[1] << "] " << "Joint Vel: ["
                << joint_vel_[0] << ", " << joint_vel_[1] << "] " << "Joint Torque: ["
                << joint_torque_[0] << ", " << joint_torque_[1] << "] " << "Torque Cmd: ["
                << joint_torque_cmd_[0] << ", " << joint_torque_cmd_[1] << "]" << std::endl;
    }

    return true;
}

bool DoubleInvertedController::write() {
    std::string error;
    std::array<double, 2> command_values;

    command_values[torque_cmd_span[0].offset] =
        std::clamp(joint_torque_cmd_[0], -100.0, 100.0);
    command_values[torque_cmd_span[1].offset] =
        std::clamp(joint_torque_cmd_[1], -100.0, 100.0);

    if (!controller_client_->SetCommand(command_values, &error)) {
        std::cerr << "Error writing command: " << error << std::endl;
        return false;
    }
    return true;
}

}  // namespace mujoco_sim_double_inverted_control_demo
