#include <mujoco_sim/sample_adapter/sample_adapter.hpp>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sample_config_generated.hpp"

namespace mujoco_sim::sample_adapter {
namespace {

std::string Trim(std::string value) {
  auto not_space = [](unsigned char c) {
    return !std::isspace(c);
  };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

generated::SampleConfig ToGenerated(const SampleOptions& options) {
  generated::SampleConfig output;
  output.max_ticks = options.max_ticks;
  output.command_value = options.command_value;
  output.service_timeout_ms = options.adapter.service_timeout_ms;
  return output;
}

SampleOptions ToPublic(generated::SampleConfig config) {
  SampleOptions output;
  output.adapter.server_name = std::move(config.server_name);
  output.adapter.adapter_id = std::move(config.adapter_id);
  output.adapter.observation_names = std::move(config.observation_names);
  output.adapter.command_names = std::move(config.command_names);
  output.adapter.service_timeout_ms = config.service_timeout_ms;
  output.max_ticks = config.max_ticks;
  output.command_value = config.command_value;
  return output;
}

}  // namespace

std::vector<std::string> ParseNameList(const std::string& value) {
  std::vector<std::string> names;
  std::stringstream input(value);
  std::string item;
  while (std::getline(input, item, ',')) {
    item = Trim(item);
    if (!item.empty()) {
      names.push_back(item);
    }
  }
  return names;
}

std::vector<double> ConstantCommandsForLayout(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout, double value) {
  return std::vector<double>(layout.command_scalar_count(), value);
}

bool ParseCommandLine(int argc, char** argv, SampleOptions* options, std::string* error_message) {
  if (options == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "options and error_message must not be null";
    }
    return false;
  }
  auto parsed = ToGenerated(*options);
  std::vector<const char*> arguments;
  arguments.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    arguments.push_back(argv[i]);
  }
  if (!generated::Parse(argc, arguments.data(), &parsed, error_message)) {
    return false;
  }
  SampleOptions output = ToPublic(std::move(parsed));
  if (output.adapter.adapter_id.empty()) {
    *error_message = "controller id is required";
    return false;
  }
  if (output.adapter.command_names.empty()) {
    *error_message = "at least one command actuator is required";
    return false;
  }
  *options = std::move(output);
  return true;
}

std::string Usage() {
  return generated::Usage();
}

}  // namespace mujoco_sim::sample_adapter
