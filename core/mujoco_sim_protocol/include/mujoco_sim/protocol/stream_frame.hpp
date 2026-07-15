#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include <mujoco_sim/v3/sync.pb.h>

namespace mujoco_sim::protocol {

inline constexpr std::size_t kStreamFramePrefixSize = 24;
inline constexpr std::uint16_t kStreamFramingMajor = 1;
inline constexpr std::uint16_t kStreamFramingMinor = 0;
inline constexpr std::uint32_t kMaximumStreamHeaderBytes = 1024U * 1024U;
inline constexpr std::uint64_t kMaximumStreamPayloadBytes =
    512ULL * 1024ULL * 1024ULL;

struct ParsedStreamFrame {
  mujoco_sim::v3::StreamFrameHeader header;
  std::span<const std::byte> payload;
};

bool SerializeStreamFrame(const mujoco_sim::v3::StreamFrameHeader& header,
                          std::span<const std::byte> payload,
                          std::string* output,
                          std::string* error_message);

bool ParseStreamFrame(std::span<const std::byte> frame,
                      ParsedStreamFrame* output,
                      std::string* error_message);

inline bool ParseStreamFrame(std::string_view frame, ParsedStreamFrame* output,
                             std::string* error_message) {
  return ParseStreamFrame(
      std::as_bytes(std::span(frame.data(), frame.size())), output,
      error_message);
}

}  // namespace mujoco_sim::protocol
