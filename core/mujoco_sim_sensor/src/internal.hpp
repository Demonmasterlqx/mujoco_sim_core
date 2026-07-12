#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/sensor/source_profile.hpp>

namespace mujoco_sim::sensor::detail {

using StreamShape = mujoco_sim::sensor::StreamShape;

inline bool Fail(std::string_view message, std::string* error_message) {
  if (error_message != nullptr) {
    *error_message = message;
  }
  return false;
}

inline void ClearError(std::string* error_message) {
  if (error_message != nullptr) {
    error_message->clear();
  }
}

inline bool CheckedMultiply(std::uint64_t lhs, std::uint64_t rhs,
                            std::uint64_t* result) {
  if (result == nullptr ||
      (lhs != 0 && rhs > std::numeric_limits<std::uint64_t>::max() / lhs)) {
    return false;
  }
  *result = lhs * rhs;
  return true;
}

bool StreamShapeFor(mujoco_sim::v1::StreamFormat format,
                    std::uint32_t width, std::uint32_t height,
                    StreamShape* shape, std::string* error_message);

bool ValidateCaptureMetadata(const CaptureMetadata& metadata,
                             std::string* error_message);

bool BuildCameraDescriptors(
    const mujoco_sim::core::SimulationSnapshot& snapshot,
    const CameraProfile& profile,
    std::vector<mujoco_sim::v1::StreamDescriptor>* descriptors,
    int* camera_id, std::string* error_message);

bool BuildRayLidarDescriptor(
    const mujoco_sim::core::SimulationSnapshot& snapshot,
    const RayLidarProfile& profile,
    mujoco_sim::v1::StreamDescriptor* descriptor,
    std::vector<double>* vertical_angles, int* site_id,
    std::string* error_message);

RawFrame MakeRawFrame(const mujoco_sim::v1::StreamDescriptor& descriptor,
                      const CaptureMetadata& metadata,
                      std::vector<std::byte> payload);

template <typename T>
std::vector<std::byte> ToBytes(std::span<const T> values) {
  static_assert(std::is_trivially_copyable_v<T>);
  std::vector<std::byte> bytes(values.size_bytes());
  if constexpr (std::endian::native == std::endian::little) {
    if (!bytes.empty()) {
      std::memcpy(bytes.data(), values.data(), values.size_bytes());
    }
  } else {
    static_assert(std::endian::native == std::endian::big);
    for (std::size_t index = 0; index < values.size(); ++index) {
      const auto native = std::bit_cast<std::array<std::byte, sizeof(T)>>(
          values[index]);
      std::reverse_copy(native.begin(), native.end(),
                        bytes.begin() + index * sizeof(T));
    }
  }
  return bytes;
}

}  // namespace mujoco_sim::sensor::detail
