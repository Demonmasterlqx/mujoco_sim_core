#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>

namespace mujoco_sim::sample_adapter {

struct SampleOptions {
  mujoco_sim::adapter_sdk::ControllerOptions adapter;
  std::uint64_t max_ticks = 0;
  double command_value = 0.0;
};

std::vector<std::string> ParseNameList(const std::string& value);
std::vector<double> ConstantCommandsForLayout(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout, double value);
bool ParseCommandLine(int argc, char** argv, SampleOptions* options, std::string* error_message);
std::string Usage();

}  // namespace mujoco_sim::sample_adapter
