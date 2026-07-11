#include <mujoco_sim/server/server_config.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "server_config_generated.hpp"

namespace mujoco_sim::server {
namespace {

generated::ServerConfig::LogLevel ToGeneratedLogLevel(
    const std::string& level) {
  if (level == "trace") {
    return generated::ServerConfig::LogLevel::kTrace;
  }
  if (level == "debug") {
    return generated::ServerConfig::LogLevel::kDebug;
  }
  if (level == "warn") {
    return generated::ServerConfig::LogLevel::kWarn;
  }
  if (level == "error") {
    return generated::ServerConfig::LogLevel::kError;
  }
  if (level == "critical") {
    return generated::ServerConfig::LogLevel::kCritical;
  }
  if (level == "off") {
    return generated::ServerConfig::LogLevel::kOff;
  }
  return generated::ServerConfig::LogLevel::kInfo;
}

std::string ToPublicLogLevel(generated::ServerConfig::LogLevel level) {
  switch (level) {
    case generated::ServerConfig::LogLevel::kTrace:
      return "trace";
    case generated::ServerConfig::LogLevel::kDebug:
      return "debug";
    case generated::ServerConfig::LogLevel::kInfo:
      return "info";
    case generated::ServerConfig::LogLevel::kWarn:
      return "warn";
    case generated::ServerConfig::LogLevel::kError:
      return "error";
    case generated::ServerConfig::LogLevel::kCritical:
      return "critical";
    case generated::ServerConfig::LogLevel::kOff:
      return "off";
  }
  return "info";
}

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
  output.log_level = ToGeneratedLogLevel(config.log_level);
  output.log_file = config.log_file;
  output.log_file_max_size_mb = config.log_file_max_size_mb;
  output.log_file_max_files = config.log_file_max_files;
  output.log_summary_period_ms = config.log_summary_period_ms;
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
  output.log_level = ToPublicLogLevel(config.log_level);
  output.log_file = std::move(config.log_file).string();
  output.log_file_max_size_mb = config.log_file_max_size_mb;
  output.log_file_max_files = config.log_file_max_files;
  output.log_summary_period_ms = config.log_summary_period_ms;
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

bool ValidateLogging(const ServerConfig& config, std::string* error_message) {
  static const std::vector<std::string> levels = {
      "trace", "debug", "info", "warn", "error", "critical", "off"};
  if (std::find(levels.begin(), levels.end(), config.log_level) ==
      levels.end()) {
    *error_message =
        "log_level must be one of: trace, debug, info, warn, error, "
        "critical, off";
    return false;
  }
  if (config.log_file_max_size_mb == 0) {
    *error_message = "log_file_max_size_mb must be >= 1";
    return false;
  }
  if (config.log_file_max_files == 0) {
    *error_message = "log_file_max_files must be in range [1, 200000]";
    return false;
  }
  if (config.log_file_max_files > 200000) {
    *error_message = "log_file_max_files must be in range [1, 200000]";
    return false;
  }
  return true;
}

bool ValidateFinal(const ServerConfig& config, std::string* error_message) {
  if (!ValidateLogging(config, error_message)) {
    return false;
  }
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
  if (!ValidateLogging(*config, error_message)) {
    return false;
  }
  auto parsed = ToGenerated(*config);
  if (!generated::Load(std::filesystem::path(path), &parsed, error_message)) {
    return false;
  }
  ServerConfig output = ToPublic(std::move(parsed));
  if (!ValidateLogging(output, error_message)) {
    return false;
  }
  *config = std::move(output);
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
  if (!ValidateLogging(*config, error_message)) {
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
