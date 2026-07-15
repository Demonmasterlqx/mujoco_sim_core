#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/v3/sync.pb.h>

namespace mujoco_sim::sensor {

struct CameraOutput {
  mujoco_sim::v3::StreamFormat format =
      mujoco_sim::v3::STREAM_FORMAT_UNSPECIFIED;
  std::string stream_name;
};

struct CameraProfile {
  std::string camera_name;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double rate_hz = 0.0;
  std::string frame_id;
  std::vector<CameraOutput> outputs;
};

struct VerticalFieldOfView {
  std::uint32_t samples = 0;
  double min_angle_rad = 0.0;
  double max_angle_rad = 0.0;
};

struct RayLidarProfile {
  std::string site_name;
  std::uint32_t horizontal_samples = 0;
  std::vector<double> vertical_angles_rad;
  std::optional<VerticalFieldOfView> vertical_fov;
  double min_range_m = 0.0;
  double max_range_m = 0.0;
  double rate_hz = 0.0;
  std::string frame_id;
  std::string stream_name;
};

struct CaptureMetadata {
  std::uint64_t epoch = 0;
  std::uint64_t tick_id = 0;
  double simulation_time_s = 0.0;
  std::uint64_t stream_sequence = 0;
  std::uint64_t capture_id = 0;
  std::uint64_t dropped_since_previous = 0;
};

struct RawFrame {
  mujoco_sim::v3::StreamFrameHeader header;
  std::vector<std::byte> payload;
};

struct StreamShape {
  std::uint32_t row_stride_bytes = 0;
  std::uint64_t payload_bytes = 0;
};

bool StreamShapeFor(mujoco_sim::v3::StreamFormat format,
                    std::uint32_t width, std::uint32_t height,
                    StreamShape* shape, std::string* error_message);

bool DescribeCameraStreams(
    const mujoco_sim::core::SimulationSnapshot& snapshot,
    const CameraProfile& profile,
    std::vector<mujoco_sim::v3::StreamDescriptor>* descriptors,
    std::string* error_message);

bool DescribeRayLidarStream(
    const mujoco_sim::core::SimulationSnapshot& snapshot,
    const RayLidarProfile& profile,
    mujoco_sim::v3::StreamDescriptor* descriptor,
    std::string* error_message);

}  // namespace mujoco_sim::sensor
