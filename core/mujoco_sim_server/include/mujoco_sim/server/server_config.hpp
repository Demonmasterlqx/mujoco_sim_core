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
  struct ControllerOverride {
    std::string adapter_id;
    std::uint32_t command_timeout_ms = 500;
    std::uint32_t command_keepalive_ms = 100;
    std::uint32_t offline_session_expire_ms = 0;
    std::uint32_t state_every_n_steps = 1;
    double state_period_s = 0.0;
    std::string startup_ctrl_policy = "zero";
    std::string offline_ctrl_policy = "hold_last";
    std::string error_ctrl_policy = "zero";
    std::string release_ctrl_policy = "zero";
  };
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
  std::string pacing_mode = "realtime";
  double realtime_factor = 1.0;
  std::uint32_t command_timeout_ms = 500;
  std::uint32_t command_keepalive_ms = 100;
  std::uint32_t offline_session_expire_ms = 0;
  std::uint32_t controller_state_every_n_steps = 1;
  double controller_state_period_s = 0.0;
  double controller_state_max_rate_hz = 1000.0;
  std::string startup_ctrl_policy = "zero";
  std::string offline_ctrl_policy = "hold_last";
  std::string error_ctrl_policy = "zero";
  std::string release_ctrl_policy = "zero";
  std::vector<ControllerOverride> controller_overrides;
  std::uint32_t session_timeout_ms = 1000;
  std::uint32_t observer_heartbeat_period_ms = 250;
  std::uint32_t status_period_ms = 100;
  std::uint32_t status_min_interval_ms = 20;
  double observer_max_rate_hz = 500.0;
  bool allow_every_step_observer = false;
  std::uint32_t transport_subscriber_sample_period_ms = 1000;
  std::uint32_t sensor_subscriber_sample_period_ms = 200;
  bool headless = !MUJOCO_SIM_HAS_GUI;
  std::string initial_keyframe;
  std::string log_level = "info";
  std::string log_file;
  std::uint32_t log_file_max_size_mb = 10;
  std::uint32_t log_file_max_files = 5;
  std::uint32_t log_summary_period_ms = 1000;
  std::uint32_t activation_timeout_ms = 5000;
  std::string render_backend = "auto";
  std::vector<SensorStream> sensor_streams;
};

bool LoadConfigFile(const std::string& path, ServerConfig* config, std::string* error_message);
bool ParseCommandLine(int argc, char** argv, ServerConfig* config, std::string* error_message);
bool ValidateServerConfig(const ServerConfig& config, std::string* error_message);
std::string Usage();

}  // namespace mujoco_sim::server
