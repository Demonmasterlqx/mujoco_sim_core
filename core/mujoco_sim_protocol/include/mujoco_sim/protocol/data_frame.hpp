#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace mujoco_sim::protocol {

inline constexpr std::size_t kStateDataFrameHeaderSize = 80;
inline constexpr std::size_t kCommandDataFrameHeaderSize = 64;
inline constexpr std::size_t kDataFrameHeaderSize = kStateDataFrameHeaderSize;
inline constexpr std::uint16_t kDataFramingMajor = 2;
inline constexpr std::uint16_t kDataFramingMinor = 0;
inline constexpr std::uint32_t kMaximumDataFrameBytes = 4U * 1024U * 1024U;

enum class DataFrameKind : std::uint8_t {
  kControllerState = 1,
  kControllerCommand = 2,
  kObservation = 3,
};

struct LayoutFingerprint {
  std::uint64_t low = 0;
  std::uint64_t high = 0;

  friend bool operator==(const LayoutFingerprint&, const LayoutFingerprint&) = default;
};

struct DataFrameHeader {
  DataFrameKind kind = DataFrameKind::kControllerState;
  std::uint8_t flags = 0;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint64_t tick_id = 0;
  LayoutFingerprint layout;
  std::uint64_t sequence = 0;
  double simulation_time_s = 0.0;
};

enum class DataFrameErrorCode {
  kNone,
  kInvalidArgument,
  kFrameTooSmall,
  kFrameTooLarge,
  kMagicMismatch,
  kVersionMismatch,
  kInvalidKind,
  kInvalidFlags,
  kHeaderSizeMismatch,
  kFrameSizeMismatch,
  kScalarCountMismatch,
};

struct DataFrameError {
  DataFrameErrorCode code = DataFrameErrorCode::kNone;
  std::string message;
};

struct ParsedDataFrame {
  DataFrameHeader header;
  std::uint32_t scalar_count = 0;
  std::span<const std::byte> scalar_bytes;
};

std::size_t DataFrameHeaderSize(DataFrameKind kind) noexcept;

bool EncodeDataFrame(const DataFrameHeader& header, std::span<const double> scalars,
                     std::string* output, DataFrameError* error);

bool ParseDataFrame(std::span<const std::byte> frame, ParsedDataFrame* output,
                    DataFrameError* error);

inline bool ParseDataFrame(std::string_view frame, ParsedDataFrame* output, DataFrameError* error) {
  return ParseDataFrame(std::as_bytes(std::span(frame.data(), frame.size())), output, error);
}

bool DecodeDataFrameScalars(const ParsedDataFrame& frame, std::span<double> output,
                            DataFrameError* error);

}  // namespace mujoco_sim::protocol
