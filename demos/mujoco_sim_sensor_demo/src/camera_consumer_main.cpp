#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/sensor_demo/stream_processing.hpp>

#include "app_config.hpp"
#include "process_signal.hpp"

namespace demo = mujoco_sim::sensor_demo;

namespace {

struct StreamRunState {
  std::string name;
  demo::CameraStreamKind kind;
  demo::StreamValidationSummary summary;
};

}  // namespace

int main(int argc, char** argv) {
  demo::AppConfig app_config;
  std::string message;
  const auto parse_result = demo::ParseAppConfig(argc, argv, &app_config,
                                                  &message);
  if (parse_result == demo::ParseAppConfigResult::kHelp) {
    std::cout << message;
    return 0;
  }
  if (parse_result != demo::ParseAppConfigResult::kSuccess) {
    std::cerr << message << '\n';
    return 1;
  }
  demo::InstallProcessSignalHandlers();
  const auto& config = app_config.camera_consumer;
  mujoco_sim::adapter_sdk::SensorStreamClient client(config.streams);
  if (!client.Start(&message)) {
    std::cerr << "camera consumer start failed: " << message << '\n';
    return 1;
  }

  std::array<StreamRunState, 4> streams{{
      {config.rgb_stream, demo::CameraStreamKind::kRgb, {}},
      {config.depth_stream, demo::CameraStreamKind::kDepth, {}},
      {config.segmentation_stream, demo::CameraStreamKind::kSegmentation, {}},
      {config.pointcloud_stream, demo::CameraStreamKind::kPointCloud, {}},
  }};
  for (const auto& stream : streams) {
    if (!client.descriptor(stream.name).has_value()) {
      std::cerr << "camera descriptor is missing: " << stream.name << '\n';
      return 1;
    }
  }
  std::cout << "sensor-demo camera consumer ready streams=4" << std::endl;

  bool screenshot_saved = false;
  int result = 0;
  while (!demo::ProcessStopRequested()) {
    bool complete = config.max_samples_per_stream > 0;
    for (auto& stream : streams) {
      if (config.max_samples_per_stream > 0 &&
          stream.summary.sample_count >= config.max_samples_per_stream) {
        continue;
      }
      complete = false;
      mujoco_sim::adapter_sdk::OwnedStreamSample sample;
      const auto wait_result = client.WaitForSample(
          stream.name, std::chrono::milliseconds(config.wait_timeout_ms),
          &sample, &message);
      if (wait_result == mujoco_sim::adapter_sdk::SampleWaitResult::kTimeout) {
        continue;
      }
      if (wait_result !=
          mujoco_sim::adapter_sdk::SampleWaitResult::kReceived) {
        std::cerr << "camera stream stopped: " << stream.name << ": "
                  << message << '\n';
        result = 1;
        break;
      }
      const auto descriptor = client.descriptor(stream.name);
      if (!descriptor.has_value() ||
          !demo::ValidateCameraSample(sample, *descriptor, stream.kind,
                                      &stream.summary, &message)) {
        std::cerr << "camera stream validation failed: " << stream.name
                  << ": " << message << '\n';
        result = 1;
        break;
      }
      if (stream.kind == demo::CameraStreamKind::kRgb &&
          !screenshot_saved) {
        if (!demo::SaveRgbScreenshot(config.screenshot_path, sample,
                                     &message)) {
          std::cerr << "camera screenshot failed: " << message << '\n';
          result = 1;
          break;
        }
        screenshot_saved = true;
      }
    }
    if (result != 0 || complete) break;
    if (config.max_samples_per_stream > 0) {
      complete = true;
      for (const auto& stream : streams) {
        complete = complete &&
                   stream.summary.sample_count >=
                       config.max_samples_per_stream;
      }
      if (complete) break;
    }
  }
  client.Stop();

  std::cout << "sensor-demo camera summary screenshot="
            << (screenshot_saved ? 1 : 0);
  for (const auto& stream : streams) {
    std::cout << ' ' << stream.name << '=' << stream.summary.sample_count
              << " dropped=" << client.dropped_samples(stream.name);
  }
  std::cout << '\n';
  if (result == 0 && !screenshot_saved) {
    std::cerr << "camera consumer stopped before receiving a valid RGB frame\n";
    return 1;
  }
  return result;
}
