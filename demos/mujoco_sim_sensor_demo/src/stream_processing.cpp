#include <mujoco_sim/sensor_demo/stream_processing.hpp>

#include <png.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <span>
#include <string>
#include <system_error>

namespace mujoco_sim::sensor_demo {
namespace {

bool Fail(const std::string& message, std::string* error_message) {
  if (error_message != nullptr) *error_message = message;
  return false;
}

bool CheckedMultiply(std::uint64_t lhs, std::uint64_t rhs,
                     std::uint64_t* product) noexcept {
  if (rhs != 0 && lhs > std::numeric_limits<std::uint64_t>::max() / rhs) {
    return false;
  }
  *product = lhs * rhs;
  return true;
}

std::size_t BytesPerPixel(mujoco_sim::v3::StreamFormat format) {
  switch (format) {
    case mujoco_sim::v3::RGB8:
      return 3;
    case mujoco_sim::v3::DEPTH32F:
    case mujoco_sim::v3::SEGMENTATION32U:
      return 4;
    case mujoco_sim::v3::XYZ32F:
      return 12;
    default:
      return 0;
  }
}

mujoco_sim::v3::StreamFormat ExpectedFormat(CameraStreamKind kind) {
  switch (kind) {
    case CameraStreamKind::kRgb:
      return mujoco_sim::v3::RGB8;
    case CameraStreamKind::kDepth:
      return mujoco_sim::v3::DEPTH32F;
    case CameraStreamKind::kSegmentation:
      return mujoco_sim::v3::SEGMENTATION32U;
    case CameraStreamKind::kPointCloud:
      return mujoco_sim::v3::XYZ32F;
  }
  return mujoco_sim::v3::STREAM_FORMAT_UNSPECIFIED;
}

bool ValidateEnvelope(
    const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
    const mujoco_sim::v3::StreamDescriptor& descriptor,
    mujoco_sim::v3::StreamFormat expected_format,
    std::string* error_message) {
  const auto& header = sample.header;
  const std::size_t bytes_per_pixel = BytesPerPixel(expected_format);
  if (bytes_per_pixel == 0 || descriptor.format() != expected_format ||
      header.format() != expected_format) {
    return Fail("sensor stream format does not match the expected format",
                error_message);
  }
  if (header.stream_name() != descriptor.name() || header.width() == 0 ||
      header.height() == 0 || header.width() != descriptor.width() ||
      header.height() != descriptor.height() ||
      header.row_stride_bytes() != descriptor.row_stride_bytes() ||
      header.frame_id() != descriptor.frame_id() ||
      header.frame_convention() != descriptor.frame_convention()) {
    return Fail("sensor stream header does not match its descriptor",
                error_message);
  }
  const std::uint64_t minimum_stride =
      static_cast<std::uint64_t>(header.width()) * bytes_per_pixel;
  const std::uint64_t expected_payload =
      static_cast<std::uint64_t>(header.row_stride_bytes()) * header.height();
  if (header.row_stride_bytes() < minimum_stride ||
      expected_payload != sample.payload.size() ||
      header.payload_size() != sample.payload.size()) {
    return Fail("sensor stream payload shape is invalid", error_message);
  }
  if (!std::isfinite(header.simulation_time_s()) ||
      header.simulation_time_s() < 0.0 ||
      header.stream_sequence() == 0) {
    return Fail("sensor stream metadata is incomplete", error_message);
  }
  return true;
}

std::uint32_t ReadUint32LittleEndian(const std::byte* source) {
  return std::to_integer<std::uint32_t>(source[0]) |
         (std::to_integer<std::uint32_t>(source[1]) << 8U) |
         (std::to_integer<std::uint32_t>(source[2]) << 16U) |
         (std::to_integer<std::uint32_t>(source[3]) << 24U);
}

float ReadFloat32LittleEndian(const std::byte* source) {
  return std::bit_cast<float>(ReadUint32LittleEndian(source));
}

void Accumulate(double value, bool* initialized, double* minimum,
                double* maximum) {
  if (!*initialized) {
    *minimum = value;
    *maximum = value;
    *initialized = true;
    return;
  }
  *minimum = std::min(*minimum, value);
  *maximum = std::max(*maximum, value);
}

std::string TemporarySuffix() {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  return ".tmp." + std::to_string(ticks) + "." +
         std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
}

}  // namespace

bool ValidateCameraSample(
    const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
    const mujoco_sim::v3::StreamDescriptor& descriptor,
    CameraStreamKind expected_kind, StreamValidationSummary* summary,
    std::string* error_message) {
  if (summary == nullptr || error_message == nullptr) {
    return Fail("summary and error_message must not be null", error_message);
  }
  error_message->clear();
  const auto expected_format = ExpectedFormat(expected_kind);
  if (!ValidateEnvelope(sample, descriptor, expected_format, error_message)) {
    return false;
  }
  if (descriptor.frame_convention() != mujoco_sim::v3::ROS_OPTICAL) {
    return Fail("camera stream must use the ROS optical frame convention",
                error_message);
  }

  std::uint64_t populated = 0;
  bool range_initialized = false;
  double minimum = 0.0;
  double maximum = 0.0;
  const std::size_t width = sample.header.width();
  const std::size_t height = sample.header.height();
  const std::size_t stride = sample.header.row_stride_bytes();
  const std::byte* bytes = sample.payload.data();
  for (std::size_t row = 0; row < height; ++row) {
    const std::byte* current = bytes + row * stride;
    for (std::size_t column = 0; column < width; ++column) {
      if (expected_kind == CameraStreamKind::kRgb) {
        for (std::size_t channel = 0; channel < 3; ++channel) {
          const auto value = std::to_integer<std::uint8_t>(
              current[3 * column + channel]);
          if (value != 0) ++populated;
          Accumulate(value, &range_initialized, &minimum, &maximum);
        }
      } else if (expected_kind == CameraStreamKind::kDepth) {
        const float value = ReadFloat32LittleEndian(current + 4 * column);
        if (!std::isfinite(value) || value <= 0.0F) {
          return Fail("depth image contains a non-positive or non-finite value",
                      error_message);
        }
        ++populated;
        Accumulate(value, &range_initialized, &minimum, &maximum);
      } else if (expected_kind == CameraStreamKind::kSegmentation) {
        const std::uint32_t value =
            ReadUint32LittleEndian(current + 4 * column);
        if (value != 0) ++populated;
        Accumulate(value, &range_initialized, &minimum, &maximum);
      } else {
        const std::byte* point = current + 12 * column;
        const float x = ReadFloat32LittleEndian(point);
        const float y = ReadFloat32LittleEndian(point + 4);
        const float z = ReadFloat32LittleEndian(point + 8);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            z <= 0.0F) {
          return Fail("camera point cloud contains an invalid XYZ value",
                      error_message);
        }
        if (std::abs(x) + std::abs(y) + std::abs(z) > 0.0F) ++populated;
        Accumulate(z, &range_initialized, &minimum, &maximum);
      }
    }
  }
  if (!range_initialized || populated == 0 ||
      (expected_kind == CameraStreamKind::kRgb && minimum == maximum)) {
    return Fail("camera stream payload is blank", error_message);
  }

  StreamValidationSummary next = *summary;
  if (next.sample_count == 0) {
    next.minimum_value = minimum;
    next.maximum_value = maximum;
  } else {
    next.minimum_value = std::min(next.minimum_value, minimum);
    next.maximum_value = std::max(next.maximum_value, maximum);
  }
  ++next.sample_count;
  next.populated_values += populated;
  *summary = next;
  return true;
}

bool DecodeXyzPointCloud(
    const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
    const mujoco_sim::v3::StreamDescriptor& descriptor, PointCloud* cloud,
    std::string* error_message) {
  if (cloud == nullptr || error_message == nullptr) {
    return Fail("cloud and error_message must not be null", error_message);
  }
  error_message->clear();
  if (!ValidateEnvelope(sample, descriptor, mujoco_sim::v3::XYZ32F,
                        error_message)) {
    return false;
  }
  if (descriptor.frame_convention() != mujoco_sim::v3::ROS_BODY) {
    return Fail("LiDAR point cloud must use the ROS body frame convention",
                error_message);
  }

  PointCloud next;
  next.reserve(static_cast<std::size_t>(sample.header.width()) *
               sample.header.height());
  for (std::size_t row = 0; row < sample.header.height(); ++row) {
    const std::byte* current =
        sample.payload.data() + row * sample.header.row_stride_bytes();
    for (std::size_t column = 0; column < sample.header.width(); ++column) {
      const std::byte* point = current + 12 * column;
      const float x = ReadFloat32LittleEndian(point);
      const float y = ReadFloat32LittleEndian(point + 4);
      const float z = ReadFloat32LittleEndian(point + 8);
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        next.push_back({x, y, z});
      }
    }
  }
  if (next.empty()) {
    return Fail("LiDAR point cloud contains no finite returns", error_message);
  }
  next.width = static_cast<std::uint32_t>(next.size());
  next.height = 1;
  next.is_dense = true;
  *cloud = std::move(next);
  return true;
}

bool SaveRgbScreenshot(
    const std::filesystem::path& output_path,
    const mujoco_sim::adapter_sdk::OwnedStreamSample& sample,
    std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  if (output_path.empty() || output_path.filename().empty()) {
    return Fail("RGB screenshot path must name a file", error_message);
  }
  if (sample.header.format() != mujoco_sim::v3::RGB8 ||
      sample.header.width() == 0 || sample.header.height() == 0) {
    return Fail("RGB screenshot sample has an invalid shape", error_message);
  }
  if (sample.header.width() > PNG_UINT_31_MAX ||
      sample.header.height() > PNG_UINT_31_MAX ||
      sample.header.row_stride_bytes() >
          static_cast<std::uint64_t>(
              std::numeric_limits<png_int_32>::max())) {
    return Fail("RGB screenshot dimensions exceed libpng limits",
                error_message);
  }

  std::uint64_t minimum_stride = 0;
  std::uint64_t payload_size = 0;
  const std::uint64_t row_stride = sample.header.row_stride_bytes();
  if (!CheckedMultiply(sample.header.width(), 3U, &minimum_stride) ||
      row_stride < minimum_stride ||
      !CheckedMultiply(row_stride, sample.header.height(), &payload_size) ||
      payload_size > std::numeric_limits<std::size_t>::max() ||
      payload_size > std::numeric_limits<png_alloc_size_t>::max() ||
      payload_size != sample.payload.size()) {
    return Fail("RGB screenshot sample has an invalid shape", error_message);
  }

  std::error_code filesystem_error;
  const auto parent = output_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, filesystem_error);
    if (filesystem_error) {
      return Fail("failed to create screenshot directory: " +
                      filesystem_error.message(),
                  error_message);
    }
  }
  const std::filesystem::path temporary =
      output_path.string() + TemporarySuffix();
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  image.width = sample.header.width();
  image.height = sample.header.height();
  image.format = PNG_FORMAT_RGB;
  if (png_image_write_to_file(
          &image, temporary.string().c_str(), 0, sample.payload.data(),
          static_cast<png_int_32>(row_stride), nullptr) == 0) {
    const std::string message = image.message;
    png_image_free(&image);
    std::filesystem::remove(temporary, filesystem_error);
    return Fail("failed to write RGB screenshot: " + message, error_message);
  }
  png_image_free(&image);
  std::filesystem::rename(temporary, output_path, filesystem_error);
  if (filesystem_error) {
    std::filesystem::remove(temporary, filesystem_error);
    return Fail("failed to install RGB screenshot: " +
                    filesystem_error.message(),
                error_message);
  }
  return true;
}

}  // namespace mujoco_sim::sensor_demo
