#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/sensor_demo/interface_data.hpp>
#include <mujoco_sim/sensor_demo/map_output.hpp>

#include "sensor_demo_generated_config.hpp"

namespace mujoco_sim::sensor_demo {

struct ControllerProcessConfig {
  mujoco_sim::adapter_sdk::AdapterOptions adapter;
  ControllerInterfaces interfaces;
  WaypointControllerConfig control;
  std::uint64_t log_every_ticks = 0;
  double max_simulation_time_s = 0.0;
  std::uint32_t wait_timeout_ms = 1000;
};

struct StateObserverProcessConfig {
  mujoco_sim::adapter_sdk::ObserverOptions observer;
  StateObserverInterfaces interfaces;
  std::uint64_t log_every_frames = 0;
  std::uint64_t max_frames = 0;
  std::uint32_t wait_timeout_ms = 1000;
};

struct CameraConsumerProcessConfig {
  mujoco_sim::adapter_sdk::SensorStreamOptions streams;
  std::string rgb_stream;
  std::string depth_stream;
  std::string segmentation_stream;
  std::string pointcloud_stream;
  std::filesystem::path screenshot_path;
  std::uint64_t max_samples_per_stream = 0;
  std::uint32_t wait_timeout_ms = 1000;
};

struct MapperProcessConfig {
  mujoco_sim::adapter_sdk::ObserverOptions observer;
  mujoco_sim::adapter_sdk::SensorStreamOptions streams;
  std::string imu_quaternion;
  std::string lidar_stream;
  std::filesystem::path output_directory;
  SimpleIcpMapperConfig algorithm;
  MapOutputConfig output;
  double imu_max_age_s = 0.05;
  std::uint64_t max_scans = 0;
  std::uint32_t wait_timeout_ms = 1000;
};

struct AppConfig {
  std::string server_name;
  std::filesystem::path output_directory;
  ControllerProcessConfig controller;
  StateObserverProcessConfig state_observer;
  CameraConsumerProcessConfig camera_consumer;
  MapperProcessConfig mapper;
};

enum class ParseAppConfigResult {
  kSuccess,
  kHelp,
  kError,
};

bool BuildAppConfig(const generated::SensorDemoConfig& generated_config,
                    AppConfig* config, std::string* error_message);
ParseAppConfigResult ParseAppConfig(int argc, char** argv, AppConfig* config,
                                    std::string* message);
std::string AppUsage();

}  // namespace mujoco_sim::sensor_demo
