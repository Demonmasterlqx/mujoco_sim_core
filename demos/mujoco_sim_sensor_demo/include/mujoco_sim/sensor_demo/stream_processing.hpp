#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/sensor_demo/icp_mapper.hpp>
#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::sensor_demo {

enum class CameraStreamKind {
  kRgb,
  kDepth,
  kSegmentation,
  kPointCloud,
};

struct StreamValidationSummary {
  std::uint64_t sample_count = 0;
  std::uint64_t populated_values = 0;
  double minimum_value = 0.0;
  double maximum_value = 0.0;
};

bool ValidateCameraSample(const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
                          const mujoco_sim::v4::StreamDescriptor& descriptor,
                          CameraStreamKind expected_kind, StreamValidationSummary* summary,
                          std::string* error_message);

// Invalid/missing LiDAR returns are NaNs on the wire and are omitted from the
// dense cloud passed to PCL.
bool DecodeXyzPointCloud(const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
                         const mujoco_sim::v4::StreamDescriptor& descriptor, PointCloud* cloud,
                         std::string* error_message);

bool SaveRgbScreenshot(const std::filesystem::path& output_path,
                       const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
                       std::string* error_message);

}  // namespace mujoco_sim::sensor_demo
