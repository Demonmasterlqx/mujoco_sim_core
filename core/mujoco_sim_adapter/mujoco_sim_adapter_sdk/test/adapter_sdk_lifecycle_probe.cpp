#include <chrono>
#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>

namespace {

using namespace std::chrono_literals;

bool Unregister(mujoco_sim::adapter_sdk::ControllerClient* client) {
  std::string error;
  if (client->Unregister(&error)) {
    return true;
  }
  std::cerr << "unregister failed: " << error << '\n';
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: adapter_sdk_lifecycle_probe SERVER_NAME\n";
    return 1;
  }

  mujoco_sim::adapter_sdk::ControllerOptions options;
  options.server_name = argv[1];
  options.adapter_id = "sdk_lifecycle_probe";
  options.observation_names = {"hinge_pos", "hinge_vel"};
  options.command_names = {"hinge_motor"};
  options.service_timeout_ms = 5000;
  mujoco_sim::adapter_sdk::ControllerClient client(std::move(options));

  std::string error;
  if (!client.Start(&error)) {
    std::cerr << "start failed: " << error << '\n';
    return 1;
  }
  if (client.ReportFault(mujoco_sim::adapter_sdk::FaultCode::kControllerError,
                         "fault before activation", &error)) {
    std::cerr << "ReportFault succeeded before activation\n";
    Unregister(&client);
    return 1;
  }
  if (!client.Activate(&error)) {
    std::cerr << "activation failed: " << error << '\n';
    Unregister(&client);
    return 1;
  }
  error.clear();
  if (client.ReportFault(static_cast<mujoco_sim::adapter_sdk::FaultCode>(-1), "invalid fault code",
                         &error)) {
    std::cerr << "ReportFault accepted an invalid code\n";
    Unregister(&client);
    return 1;
  }

  mujoco_sim::adapter_sdk::StateLease first;
  const auto first_result = client.WaitForNewState(5s, &first, &error);
  if (first_result != mujoco_sim::adapter_sdk::StateReadResult::kReceived) {
    std::cerr << "did not receive initial State: result=" << static_cast<int>(first_result)
              << " error=" << error << '\n';
    Unregister(&client);
    return 1;
  }

  const std::uint64_t initial_tick_id = first.tick_id();
  std::this_thread::sleep_for(800ms);
  const std::array commands{0.1};
  if (!client.SetCommand(commands, &error)) {
    std::cerr << "delayed command failed: " << error << '\n';
    Unregister(&client);
    return 1;
  }
  std::this_thread::sleep_for(300ms);

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool recovered = false;
  while (std::chrono::steady_clock::now() < deadline) {
    mujoco_sim::adapter_sdk::StateLease state;
    const auto result = client.WaitForNewState(500ms, &state, &error);
    if (result == mujoco_sim::adapter_sdk::StateReadResult::kTimeout) {
      continue;
    }
    if (result != mujoco_sim::adapter_sdk::StateReadResult::kReceived) {
      std::cerr << "recovery wait failed: " << error << '\n';
      break;
    }
    if (state.tick_id() > initial_tick_id) {
      recovered = true;
      break;
    }
  }

  const bool unregistered = Unregister(&client);
  error.clear();
  const bool command_rejected = !client.SetCommand(commands, &error);
  if (!command_rejected) {
    std::cerr << "SetCommand succeeded after unregister\n";
  }
  error.clear();
  const bool fault_rejected = !client.ReportFault(
      mujoco_sim::adapter_sdk::FaultCode::kControllerError, "fault after unregister", &error);
  if (!fault_rejected) {
    std::cerr << "ReportFault succeeded after unregister\n";
  }
  if (!recovered) {
    std::cerr << "controller did not receive a newer State after recovery\n";
  }
  return recovered && unregistered && command_rejected && fault_rejected ? 0 : 1;
}
