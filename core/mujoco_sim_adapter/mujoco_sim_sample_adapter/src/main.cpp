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

  const int result = client.Run(
      [value = options.command_value](const mujoco_sim::v1::Tick& tick) {
        return mujoco_sim::sample_adapter::ConstantCommandsForTick(tick, value);
      },
      options.max_ticks);
  if (!client.lifecycle_warning().empty()) {
    std::cerr << "adapter lifecycle warning: " << client.lifecycle_warning()
              << '\n';
  }
  return result;
}
