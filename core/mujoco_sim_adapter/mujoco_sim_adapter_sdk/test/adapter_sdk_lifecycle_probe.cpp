#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

namespace {

using namespace std::chrono_literals;

bool Unregister(mujoco_sim::adapter_sdk::SyncAdapterClient* client) {
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

  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = argv[1];
  options.adapter_id = "sdk_lifecycle_probe";
  options.observation_names = {"hinge_pos", "hinge_vel"};
  options.command_names = {"hinge_motor"};
  options.timeout_ms = 2000;
  options.service_timeout_ms = 5000;
  mujoco_sim::adapter_sdk::SyncAdapterClient client(std::move(options));

  std::string error;
  if (!client.Start(&error)) {
    std::cerr << "start failed: " << error << '\n';
    return 1;
  }
  if (!client.Activate(&error)) {
    std::cerr << "activation failed: " << error << '\n';
    Unregister(&client);
    return 1;
  }

  mujoco_sim::v2::Tick first;
  const auto first_result = client.WaitForTick(5s, &first, &error);
  if (first_result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived ||
      first.controller_state() != mujoco_sim::v2::ACTIVE) {
    std::cerr << "did not receive initial ACTIVE Tick: result="
              << static_cast<int>(first_result)
              << " state=" << static_cast<int>(first.controller_state())
              << " error=" << error << '\n';
    Unregister(&client);
    return 1;
  }

  const std::uint64_t initial_tick_id = first.header().tick_id();
  std::this_thread::sleep_for(2500ms);
  const std::vector<mujoco_sim::adapter_sdk::CommandValue> commands{
      {"hinge_motor", 0.1}};
  if (!client.Commit(first, commands, &error)) {
    std::cerr << "delayed Commit failed: " << error << '\n';
    Unregister(&client);
    return 1;
  }

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool recovered = false;
  while (std::chrono::steady_clock::now() < deadline) {
    mujoco_sim::v2::Tick tick;
    const auto result = client.WaitForTick(500ms, &tick, &error);
    if (result == mujoco_sim::adapter_sdk::TickWaitResult::kTimeout) {
      continue;
    }
    if (result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
      std::cerr << "recovery wait failed: " << error << '\n';
      break;
    }
    if (!client.Commit(tick, commands, &error)) {
      std::cerr << "recovery Commit failed: " << error << '\n';
      break;
    }
    if (tick.controller_state() == mujoco_sim::v2::ACTIVE &&
        tick.header().tick_id() > initial_tick_id) {
      recovered = true;
      break;
    }
  }

  const bool unregistered = Unregister(&client);
  if (!recovered) {
    std::cerr << "adapter did not recover to a newer ACTIVE Tick\n";
  }
  return recovered && unregistered ? 0 : 1;
}
