#include <mujoco_sim/server/server_config.hpp>

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "server_config_generated.hpp"

namespace mujoco_sim::server {
namespace {

generated::ServerConfig ToGenerated(const ServerConfig& config) {
  generated::ServerConfig output;
  output.server_name = config.server_name;
  output.model_path = config.model_path;
  output.initial_keyframe = config.initial_keyframe;
  output.max_ticks = config.max_ticks;
  output.tick_timeout_ms = config.tick_timeout_ms;
  output.startup_timeout_ms = config.startup_timeout_ms;
  output.status_period_ms = config.status_period_ms;
  output.headless = config.headless;
  return output;
}

ServerConfig ToPublic(generated::ServerConfig config) {
  ServerConfig output;
  output.server_name = std::move(config.server_name);
  output.model_path = std::move(config.model_path).string();
  output.initial_keyframe = std::move(config.initial_keyframe);
  output.max_ticks = config.max_ticks;
  output.tick_timeout_ms = config.tick_timeout_ms;
  output.startup_timeout_ms = config.startup_timeout_ms;
  output.status_period_ms = config.status_period_ms;
  output.headless = config.headless;
  return output;
}

std::vector<const char*> ConstArgv(int argc, char** argv) {
  std::vector<const char*> result;
  result.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    result.push_back(argv[i]);
  }
  return result;
}

bool ValidateFinal(const ServerConfig& config, std::string* error_message) {
  if (config.model_path.empty()) {
    *error_message = "model path is required";
    return false;
  }
  if (config.server_name.empty()) {
    *error_message = "server name is required";
    return false;
  }
  return true;
}

}  // namespace

bool LoadConfigFile(const std::string& path, ServerConfig* config,
                    std::string* error_message) {
  if (config == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "config and error_message must not be null";
    }
    return false;
  }
  auto parsed = ToGenerated(*config);
  if (!generated::Load(std::filesystem::path(path), &parsed, error_message)) {
    return false;
  }
  *config = ToPublic(std::move(parsed));
  return true;
}

bool ParseCommandLine(int argc, char** argv, ServerConfig* config,
                      std::string* error_message) {
  if (config == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "config and error_message must not be null";
    }
    return false;
  }
  auto parsed = ToGenerated(*config);
  const auto arguments = ConstArgv(argc, argv);
  if (!generated::Parse(argc, arguments.data(), &parsed, error_message)) {
    return false;
  }
  ServerConfig output = ToPublic(std::move(parsed));
  if (!ValidateFinal(output, error_message)) {
    return false;
  }
  *config = std::move(output);
  return true;
}

std::string Usage() { return generated::Usage(); }

}  // namespace mujoco_sim::server
