#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_sim::server {

struct ServerConfig {
  std::string server_name = "default";
  std::string model_path;
  std::uint64_t max_ticks = 0;
  std::uint32_t tick_timeout_ms = 1000;
  std::uint32_t startup_timeout_ms = 5000;
  std::uint32_t status_period_ms = 100;
  bool headless = false;
  std::string initial_keyframe;
};

bool LoadConfigFile(const std::string& path, ServerConfig* config,
                    std::string* error_message);
bool ParseCommandLine(int argc, char** argv, ServerConfig* config,
                      std::string* error_message);
std::string Usage();

}  // namespace mujoco_sim::server
