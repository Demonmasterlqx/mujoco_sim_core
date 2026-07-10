#include <mujoco_sim/sample_adapter/sample_adapter.hpp>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>

namespace mujoco_sim::sample_adapter {
namespace {

std::string Trim(std::string value) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
              value.end());
  return value;
}

bool ParseUnsigned(const std::string& value, std::uint64_t* output) {
  try {
    std::size_t parsed = 0;
    const auto result = std::stoull(value, &parsed);
    if (parsed != value.size()) {
      return false;
    }
    *output = result;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseDouble(const std::string& value, double* output) {
  try {
    std::size_t parsed = 0;
    const auto result = std::stod(value, &parsed);
    if (parsed != value.size()) {
      return false;
    }
    *output = result;
    return true;
  } catch (...) {
    return false;
  }
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

std::vector<mujoco_sim::adapter_sdk::CommandValue> ConstantCommandsForTick(
    const mujoco_sim::v1::Tick& tick, double value) {
  std::vector<mujoco_sim::adapter_sdk::CommandValue> commands;
  commands.reserve(static_cast<std::size_t>(tick.layout().commands_size()));
  for (const auto& command : tick.layout().commands()) {
    commands.push_back({command.name(), value});
  }
  return commands;
}

bool ParseCommandLine(int argc, char** argv, SampleOptions* options,
                      std::string* error_message) {
  options->adapter.server_name = "default";
  options->adapter.controller_id = "sample_controller";
  options->adapter.observation_names = {"hinge_pos", "hinge_vel"};
  options->adapter.command_names = {"hinge_motor"};
  options->adapter.timeout_ms = 1000;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string& name) -> const char* {
      if (i + 1 >= argc) {
        *error_message = "missing value for " + name;
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      *error_message = Usage();
      return false;
    } else if (arg == "--server-name") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options->adapter.server_name = value;
    } else if (arg == "--controller-id") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options->adapter.controller_id = value;
    } else if (arg == "--observations") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options->adapter.observation_names = ParseNameList(value);
    } else if (arg == "--commands") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options->adapter.command_names = ParseNameList(value);
    } else if (arg == "--ticks") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      if (!ParseUnsigned(value, &options->max_ticks)) {
        *error_message = "invalid --ticks: " + std::string(value);
        return false;
      }
    } else if (arg == "--command-value") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      if (!ParseDouble(value, &options->command_value)) {
        *error_message = "invalid --command-value: " + std::string(value);
        return false;
      }
    } else if (arg == "--timeout-ms") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      std::uint64_t parsed = 0;
      if (!ParseUnsigned(value, &parsed)) {
        *error_message = "invalid --timeout-ms: " + std::string(value);
        return false;
      }
      options->adapter.timeout_ms = static_cast<std::uint32_t>(parsed);
    } else if (arg == "--service-timeout-ms") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      std::uint64_t parsed = 0;
      if (!ParseUnsigned(value, &parsed)) {
        *error_message = "invalid --service-timeout-ms: " + std::string(value);
        return false;
      }
      options->adapter.service_timeout_ms = static_cast<std::uint32_t>(parsed);
    } else {
      *error_message = "unknown argument: " + arg;
      return false;
    }
  }

  if (options->adapter.controller_id.empty()) {
    *error_message = "controller id is required";
    return false;
  }
  if (options->adapter.command_names.empty()) {
    *error_message = "at least one command actuator is required";
    return false;
  }
  return true;
}

std::string Usage() {
  return "Usage: mujoco-sim-sample-adapter [--server-name NAME] "
         "[--controller-id ID] [--observations a,b] [--commands a,b] "
         "[--ticks N] [--command-value VALUE]";
}

}  // namespace mujoco_sim::sample_adapter
