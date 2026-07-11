#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/sample_adapter/sample_adapter.hpp>

int main(int argc, char** argv) {
  mujoco_sim::sample_adapter::SampleOptions options;
  std::string error;
  if (!mujoco_sim::sample_adapter::ParseCommandLine(argc, argv, &options,
                                                    &error)) {
    std::cerr << error << '\n';
    if (error == mujoco_sim::sample_adapter::Usage()) {
      return 0;
    }
    return 1;
  }

  mujoco_sim::adapter_sdk::SyncAdapterClient client(options.adapter);
  if (!client.Start(&error)) {
    std::cerr << error << '\n';
    return 1;
  }
  if (!client.Activate(&error)) {
    std::cerr << error << '\n';
    std::string ignored;
    client.Unregister(&ignored);
    return 1;
  }

  int result = 0;
  std::uint64_t handled_ticks = 0;
  while (options.max_ticks == 0 || handled_ticks < options.max_ticks) {
    mujoco_sim::v1::Tick tick;
    const auto wait_result = client.WaitForTick(
        std::chrono::milliseconds(100), &tick, &error);
    if (wait_result == mujoco_sim::adapter_sdk::TickWaitResult::kTimeout) {
      continue;
    }
    if (wait_result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
      if (!error.empty()) {
        std::cerr << error << '\n';
      }
      result = 1;
      break;
    }

    const auto commands =
        mujoco_sim::sample_adapter::ConstantCommandsForTick(
            tick, options.command_value);
    if (!client.Commit(tick, commands, &error)) {
      std::cerr << error << '\n';
      result = 1;
      break;
    }
    if (tick.controller_state() == mujoco_sim::v1::ACTIVE) {
      ++handled_ticks;
    }
  }

  std::string unregister_error;
  if (!client.Unregister(&unregister_error)) {
    std::cerr << "adapter unregister failed: " << unregister_error << '\n';
  }
  if (!client.lifecycle_warning().empty()) {
    std::cerr << "adapter lifecycle warning: " << client.lifecycle_warning()
              << '\n';
  }
  return result;
}
