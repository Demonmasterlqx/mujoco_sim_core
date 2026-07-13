#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::sample_adapter {

struct SampleOptions {
  mujoco_sim::adapter_sdk::AdapterOptions adapter;
  std::uint64_t max_ticks = 0;
  double command_value = 0.0;
};

std::vector<std::string> ParseNameList(const std::string& value);
std::vector<mujoco_sim::adapter_sdk::CommandValue> ConstantCommandsForLayout(
    const mujoco_sim::v2::ResourceLayout& layout, double value);
bool ParseCommandLine(int argc, char** argv, SampleOptions* options,
                      std::string* error_message);
std::string Usage();

}  // namespace mujoco_sim::sample_adapter
