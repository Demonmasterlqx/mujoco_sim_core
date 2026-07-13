#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_sim::server {

#ifndef MUJOCO_SIM_HAS_GUI
#define MUJOCO_SIM_HAS_GUI 1
#endif

inline constexpr double kMinimumSensorStreamRateHz = 0.000001;
inline constexpr double kMaximumSensorStreamRateHz = 1'000'000.0;
inline constexpr std::uint32_t kMaximumSensorStreamQueueDepth = 64;

struct ServerConfig {
  struct SensorStream {
    std::string name;
    std::string kind = "rgb";
    std::string source;
    std::string frame_id;
    double rate_hz = 10.0;
    std::uint32_t width = 320;
    std::uint32_t height = 240;
    std::uint32_t horizontal_samples = 720;
    std::uint32_t vertical_samples = 16;
    double vertical_fov_min_deg = -15.0;
    double vertical_fov_max_deg = 15.0;
    double min_range_m = 0.15;
    double max_range_m = 15.0;
    std::string queue_mode = "latest";
    std::uint32_t queue_depth = 1;
    std::string fifo_overflow = "drop_oldest";
  };

  std::string server_name = "default";
  std::string model_path;
  std::uint64_t max_ticks = 0;
  std::uint32_t tick_timeout_ms = 1000;
  std::uint32_t status_period_ms = 100;
  bool headless = !MUJOCO_SIM_HAS_GUI;
  std::string initial_keyframe;
  std::string log_level = "info";
  std::string log_file;
  std::uint32_t log_file_max_size_mb = 10;
  std::uint32_t log_file_max_files = 5;
  std::uint32_t log_summary_period_ms = 5000;
  std::uint32_t activation_timeout_ms = 5000;
  bool allow_timed_out_adapter_replacement = true;
  std::string render_backend = "auto";
  std::vector<SensorStream> sensor_streams;
};

bool LoadConfigFile(const std::string& path, ServerConfig* config,
                    std::string* error_message);
bool ParseCommandLine(int argc, char** argv, ServerConfig* config,
                      std::string* error_message);
bool ValidateServerConfig(const ServerConfig& config,
                          std::string* error_message);
std::string Usage();

}  // namespace mujoco_sim::server
