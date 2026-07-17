#include <mujoco_sim/sensor/source_profile.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "internal.hpp"

namespace mujoco_sim::sensor {
namespace detail {
namespace {

constexpr double kPi = 3.14159265358979323846264338327950288;

bool IsCameraFormat(mujoco_sim::v4::StreamFormat format) {
  return format == mujoco_sim::v4::RGB8 || format == mujoco_sim::v4::DEPTH32F ||
         format == mujoco_sim::v4::SEGMENTATION32U || format == mujoco_sim::v4::XYZ32F;
}

bool ValidateSnapshot(const core::SimulationSnapshot& snapshot, std::string* error_message) {
  if (snapshot.model() == nullptr || snapshot.data() == nullptr) {
    return Fail("simulation snapshot has no model or data", error_message);
  }
  return true;
}

bool ComputePinholeIntrinsics(const mjModel* model, int camera_id, std::uint32_t width,
                              std::uint32_t height, mujoco_sim::v4::CameraIntrinsics* intrinsics,
                              std::string* error_message) {
  if (model->cam_projection[camera_id] != mjPROJ_PERSPECTIVE) {
    return Fail("orthographic MuJoCo cameras are not supported", error_message);
  }

  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.5 * (static_cast<double>(width) - 1.0);
  double cy = 0.5 * (static_cast<double>(height) - 1.0);
  const double sensor_width = model->cam_sensorsize[2 * camera_id];
  const double sensor_height = model->cam_sensorsize[2 * camera_id + 1];
  if (sensor_width > 0.0 || sensor_height > 0.0) {
    const float* intrinsic = model->cam_intrinsic + 4 * camera_id;
    if (!std::isfinite(sensor_width) || !std::isfinite(sensor_height) || sensor_width <= 0.0 ||
        sensor_height <= 0.0 || !std::isfinite(intrinsic[0]) || !std::isfinite(intrinsic[1]) ||
        !std::isfinite(intrinsic[2]) || !std::isfinite(intrinsic[3]) || intrinsic[0] <= 0.0F ||
        intrinsic[1] <= 0.0F) {
      return Fail("camera has invalid compiled intrinsic parameters", error_message);
    }
    fx = intrinsic[0] / sensor_width * static_cast<double>(width);
    fy = intrinsic[1] / sensor_height * static_cast<double>(height);
    cx = (0.5 - intrinsic[2] / sensor_width) * static_cast<double>(width) - 0.5;
    cy = (0.5 - intrinsic[3] / sensor_height) * static_cast<double>(height) - 0.5;
  } else {
    const double fovy_degrees = model->cam_fovy[camera_id];
    if (!std::isfinite(fovy_degrees) || fovy_degrees <= 0.0 || fovy_degrees >= 180.0) {
      return Fail("camera has an invalid perspective field of view", error_message);
    }
    fy = 0.5 * static_cast<double>(height) / std::tan(0.5 * fovy_degrees * kPi / 180.0);
    fx = fy;
  }
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
      fx <= 0.0 || fy <= 0.0) {
    return Fail("camera intrinsics are not finite", error_message);
  }

  intrinsics->set_fx(fx);
  intrinsics->set_fy(fy);
  intrinsics->set_cx(cx);
  intrinsics->set_cy(cy);
  return true;
}

void PopulateDescriptorCommon(mujoco_sim::v4::StreamDescriptor* descriptor, std::string_view name,
                              mujoco_sim::v4::StreamFormat format, std::uint32_t width,
                              std::uint32_t height, const StreamShape& shape,
                              std::string_view frame_id, mujoco_sim::v4::FrameConvention convention,
                              std::string_view source_name, double rate_hz) {
  descriptor->set_name(name.data(), name.size());
  descriptor->set_format(format);
  descriptor->set_width(width);
  descriptor->set_height(height);
  descriptor->set_row_stride_bytes(shape.row_stride_bytes);
  descriptor->set_frame_id(frame_id.data(), frame_id.size());
  descriptor->set_frame_convention(convention);
  descriptor->set_source_name(source_name.data(), source_name.size());
  descriptor->set_rate_hz(rate_hz);
  descriptor->set_maximum_payload_bytes(shape.payload_bytes);
}

}  // namespace

bool StreamShapeFor(mujoco_sim::v4::StreamFormat format, std::uint32_t width, std::uint32_t height,
                    StreamShape* shape, std::string* error_message) {
  if (shape == nullptr) {
    return Fail("stream shape output is null", error_message);
  }
  if (width == 0 || height == 0) {
    return Fail("stream width and height must be greater than zero", error_message);
  }
  if (width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return Fail("stream dimensions exceed renderer integer limits", error_message);
  }

  std::uint64_t bytes_per_pixel = 0;
  switch (format) {
    case mujoco_sim::v4::RGB8:
      bytes_per_pixel = 3;
      break;
    case mujoco_sim::v4::DEPTH32F:
    case mujoco_sim::v4::SEGMENTATION32U:
      bytes_per_pixel = 4;
      break;
    case mujoco_sim::v4::XYZ32F:
      bytes_per_pixel = 12;
      break;
    default:
      return Fail("unsupported stream format", error_message);
  }

  std::uint64_t stride = 0;
  std::uint64_t payload = 0;
  if (!CheckedMultiply(width, bytes_per_pixel, &stride) ||
      stride > std::numeric_limits<std::uint32_t>::max() ||
      !CheckedMultiply(stride, height, &payload) ||
      payload > protocol::kMaximumStreamPayloadBytes ||
      payload > std::numeric_limits<std::size_t>::max()) {
    return Fail("stream shape exceeds supported payload limits", error_message);
  }

  shape->row_stride_bytes = static_cast<std::uint32_t>(stride);
  shape->payload_bytes = payload;
  return true;
}

bool ValidateCaptureMetadata(const CaptureMetadata& metadata, std::string* error_message) {
  if (!std::isfinite(metadata.simulation_time_s)) {
    return Fail("capture simulation time must be finite", error_message);
  }
  return true;
}

bool BuildCameraDescriptors(const core::SimulationSnapshot& snapshot, const CameraProfile& profile,
                            std::vector<mujoco_sim::v4::StreamDescriptor>* descriptors,
                            int* camera_id, std::string* error_message) {
  ClearError(error_message);
  if (descriptors == nullptr || camera_id == nullptr) {
    return Fail("camera descriptor output is null", error_message);
  }
  descriptors->clear();
  *camera_id = -1;
  if (!ValidateSnapshot(snapshot, error_message)) {
    return false;
  }
  if (profile.camera_name.empty()) {
    return Fail("camera_name must not be empty", error_message);
  }
  if (profile.frame_id.empty()) {
    return Fail("camera frame_id must not be empty", error_message);
  }
  if (!std::isfinite(profile.rate_hz) || profile.rate_hz <= 0.0) {
    return Fail("camera rate_hz must be finite and greater than zero", error_message);
  }
  if (profile.outputs.empty()) {
    return Fail("camera must select at least one output", error_message);
  }

  const auto* model = snapshot.model();
  const int id = mj_name2id(model, mjOBJ_CAMERA, profile.camera_name.c_str());
  if (id < 0) {
    return Fail("MuJoCo camera '" + profile.camera_name + "' was not found", error_message);
  }

  mujoco_sim::v4::CameraIntrinsics intrinsics;
  if (!ComputePinholeIntrinsics(model, id, profile.width, profile.height, &intrinsics,
                                error_message)) {
    return false;
  }

  std::set<int> formats;
  std::set<std::string> stream_names;
  std::vector<mujoco_sim::v4::StreamDescriptor> built_descriptors;
  built_descriptors.reserve(profile.outputs.size());
  for (const CameraOutput& output : profile.outputs) {
    if (!IsCameraFormat(output.format)) {
      return Fail("camera output has an unsupported format", error_message);
    }
    if (output.stream_name.empty()) {
      return Fail("camera stream_name must not be empty", error_message);
    }
    if (!formats.insert(static_cast<int>(output.format)).second) {
      return Fail("camera output formats must be unique", error_message);
    }
    if (!stream_names.insert(output.stream_name).second) {
      return Fail("camera stream names must be unique", error_message);
    }

    StreamShape shape;
    if (!mujoco_sim::sensor::detail::StreamShapeFor(output.format, profile.width, profile.height,
                                                    &shape, error_message)) {
      return false;
    }
    auto& descriptor = built_descriptors.emplace_back();
    PopulateDescriptorCommon(&descriptor, output.stream_name, output.format, profile.width,
                             profile.height, shape, profile.frame_id, mujoco_sim::v4::ROS_OPTICAL,
                             profile.camera_name, profile.rate_hz);
    *descriptor.mutable_camera_intrinsics() = intrinsics;
    if (output.format == mujoco_sim::v4::SEGMENTATION32U) {
      for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
        auto* label = descriptor.add_segmentation_labels();
        label->set_value(static_cast<std::uint32_t>(geom_id) + 1U);
        label->set_mujoco_object_type(mjOBJ_GEOM);
        label->set_mujoco_object_id(geom_id);
        const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
        if (name != nullptr) {
          label->set_name(name);
        }
      }
    }
  }

  *descriptors = std::move(built_descriptors);
  *camera_id = id;
  return true;
}

bool BuildRayLidarDescriptor(const core::SimulationSnapshot& snapshot,
                             const RayLidarProfile& profile,
                             mujoco_sim::v4::StreamDescriptor* descriptor,
                             std::vector<double>* vertical_angles, int* site_id,
                             std::string* error_message) {
  ClearError(error_message);
  if (descriptor == nullptr || vertical_angles == nullptr || site_id == nullptr) {
    return Fail("ray lidar descriptor output is null", error_message);
  }
  descriptor->Clear();
  vertical_angles->clear();
  *site_id = -1;
  if (!ValidateSnapshot(snapshot, error_message)) {
    return false;
  }
  if (profile.site_name.empty()) {
    return Fail("lidar site_name must not be empty", error_message);
  }
  if (profile.stream_name.empty()) {
    return Fail("lidar stream_name must not be empty", error_message);
  }
  if (profile.frame_id.empty()) {
    return Fail("lidar frame_id must not be empty", error_message);
  }
  if (profile.horizontal_samples == 0) {
    return Fail("lidar horizontal_samples must be greater than zero", error_message);
  }
  if (!std::isfinite(profile.min_range_m) || !std::isfinite(profile.max_range_m) ||
      profile.min_range_m < 0.0 || profile.max_range_m <= profile.min_range_m ||
      profile.max_range_m > std::numeric_limits<float>::max()) {
    return Fail("lidar range limits must be finite with 0 <= min < max", error_message);
  }
  if (!std::isfinite(profile.rate_hz) || profile.rate_hz <= 0.0) {
    return Fail("lidar rate_hz must be finite and greater than zero", error_message);
  }

  const bool has_angles = !profile.vertical_angles_rad.empty();
  const bool has_fov = profile.vertical_fov.has_value();
  if (has_angles == has_fov) {
    return Fail("lidar must set exactly one vertical angle mode", error_message);
  }

  std::size_t ring_count = profile.vertical_angles_rad.size();
  if (has_fov) {
    const VerticalFieldOfView& fov = *profile.vertical_fov;
    if (fov.samples == 0 || !std::isfinite(fov.min_angle_rad) ||
        !std::isfinite(fov.max_angle_rad) || fov.min_angle_rad > fov.max_angle_rad ||
        fov.min_angle_rad < -0.5 * kPi || fov.max_angle_rad > 0.5 * kPi) {
      return Fail("lidar vertical FOV is invalid", error_message);
    }
    ring_count = fov.samples;
  }
  if (ring_count > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return Fail("lidar ring count exceeds descriptor limits", error_message);
  }

  const auto rings = static_cast<std::uint32_t>(ring_count);
  StreamShape shape;
  if (!mujoco_sim::sensor::detail::StreamShapeFor(
          mujoco_sim::v4::XYZ32F, profile.horizontal_samples, rings, &shape, error_message)) {
    return false;
  }
  std::uint64_t ray_count = 0;
  if (!CheckedMultiply(profile.horizontal_samples, rings, &ray_count) ||
      ray_count > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
    return Fail("lidar ray count exceeds MuJoCo limits", error_message);
  }

  if (has_angles) {
    for (double angle : profile.vertical_angles_rad) {
      if (!std::isfinite(angle) || angle < -0.5 * kPi || angle > 0.5 * kPi) {
        return Fail("lidar vertical angles must be finite and within +/-pi/2", error_message);
      }
    }
  }

  const auto* model = snapshot.model();
  const int id = mj_name2id(model, mjOBJ_SITE, profile.site_name.c_str());
  if (id < 0) {
    return Fail("MuJoCo site '" + profile.site_name + "' was not found", error_message);
  }

  std::vector<double> built_angles;
  if (has_angles) {
    built_angles = profile.vertical_angles_rad;
  } else {
    const VerticalFieldOfView& fov = *profile.vertical_fov;
    built_angles.resize(fov.samples);
    if (fov.samples == 1) {
      built_angles[0] = 0.5 * (fov.min_angle_rad + fov.max_angle_rad);
    } else {
      const double step =
          (fov.max_angle_rad - fov.min_angle_rad) / static_cast<double>(fov.samples - 1U);
      for (std::uint32_t ring = 0; ring < fov.samples; ++ring) {
        built_angles[ring] = fov.min_angle_rad + step * static_cast<double>(ring);
      }
    }
  }

  PopulateDescriptorCommon(descriptor, profile.stream_name, mujoco_sim::v4::XYZ32F,
                           profile.horizontal_samples, rings, shape, profile.frame_id,
                           mujoco_sim::v4::ROS_BODY, profile.site_name, profile.rate_hz);
  *vertical_angles = std::move(built_angles);
  *site_id = id;
  return true;
}

RawFrame MakeRawFrame(const mujoco_sim::v4::StreamDescriptor& descriptor,
                      const CaptureMetadata& metadata, std::vector<std::byte> payload) {
  RawFrame frame;
  frame.header.set_epoch(metadata.epoch);
  frame.header.set_tick_id(metadata.tick_id);
  frame.header.set_simulation_time_s(metadata.simulation_time_s);
  frame.header.set_stream_name(descriptor.name());
  frame.header.set_stream_sequence(metadata.stream_sequence);
  frame.header.set_capture_id(metadata.capture_id);
  frame.header.set_format(descriptor.format());
  frame.header.set_width(descriptor.width());
  frame.header.set_height(descriptor.height());
  frame.header.set_row_stride_bytes(descriptor.row_stride_bytes());
  frame.header.set_frame_id(descriptor.frame_id());
  frame.header.set_frame_convention(descriptor.frame_convention());
  frame.header.set_payload_size(payload.size());
  frame.header.set_dropped_since_previous(metadata.dropped_since_previous);
  frame.payload = std::move(payload);
  return frame;
}

}  // namespace detail

bool StreamShapeFor(mujoco_sim::v4::StreamFormat format, std::uint32_t width, std::uint32_t height,
                    StreamShape* shape, std::string* error_message) {
  return detail::StreamShapeFor(format, width, height, shape, error_message);
}

bool DescribeCameraStreams(const core::SimulationSnapshot& snapshot, const CameraProfile& profile,
                           std::vector<mujoco_sim::v4::StreamDescriptor>* descriptors,
                           std::string* error_message) {
  int camera_id = -1;
  return detail::BuildCameraDescriptors(snapshot, profile, descriptors, &camera_id, error_message);
}

bool DescribeRayLidarStream(const core::SimulationSnapshot& snapshot,
                            const RayLidarProfile& profile,
                            mujoco_sim::v4::StreamDescriptor* descriptor,
                            std::string* error_message) {
  std::vector<double> vertical_angles;
  int site_id = -1;
  return detail::BuildRayLidarDescriptor(snapshot, profile, descriptor, &vertical_angles, &site_id,
                                         error_message);
}

}  // namespace mujoco_sim::sensor
