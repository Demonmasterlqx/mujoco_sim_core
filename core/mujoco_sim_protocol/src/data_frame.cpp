#include <mujoco_sim/protocol/data_frame.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

namespace mujoco_sim::protocol {
namespace {

static_assert(sizeof(double) == sizeof(std::uint64_t));
static_assert(std::numeric_limits<double>::is_iec559);

constexpr std::array<std::byte, 8> kMagic = {
    std::byte{'M'}, std::byte{'J'}, std::byte{'S'}, std::byte{'Y'},
    std::byte{'N'}, std::byte{'C'}, std::byte{'3'}, std::byte{0}};

template <typename T>
void WriteLittleEndian(T value, std::byte* output) {
  static_assert(std::is_unsigned_v<T>);
  for (std::size_t index = 0; index < sizeof(T); ++index) {
    output[index] = static_cast<std::byte>(value & static_cast<T>(0xffU));
    value >>= 8U;
  }
}

template <typename T>
T ReadLittleEndian(const std::byte* input) {
  static_assert(std::is_unsigned_v<T>);
  T value = 0;
  for (std::size_t index = 0; index < sizeof(T); ++index) {
    value |= static_cast<T>(std::to_integer<unsigned int>(input[index]))
             << (8U * index);
  }
  return value;
}

bool Fail(DataFrameErrorCode code, std::string message,
          DataFrameError* error) {
  if (error != nullptr) {
    error->code = code;
    error->message = std::move(message);
  }
  return false;
}

void ClearError(DataFrameError* error) {
  if (error != nullptr) {
    error->code = DataFrameErrorCode::kNone;
    error->message.clear();
  }
}

bool IsKnownKind(DataFrameKind kind) {
  switch (kind) {
    case DataFrameKind::kTick:
    case DataFrameKind::kCommit:
    case DataFrameKind::kObservation:
      return true;
  }
  return false;
}

bool ValidateHeaderSemantics(const DataFrameHeader& header,
                             DataFrameError* error) {
  if (!IsKnownKind(header.kind)) {
    return Fail(DataFrameErrorCode::kInvalidKind,
                "data frame kind is not supported", error);
  }
  const std::uint8_t allowed_flags =
      header.kind == DataFrameKind::kTick ? kDataFrameRecoveryProof : 0U;
  if ((header.flags & static_cast<std::uint8_t>(~allowed_flags)) != 0U) {
    return Fail(DataFrameErrorCode::kInvalidFlags,
                "data frame contains unsupported flags", error);
  }
  if (header.kind != DataFrameKind::kObservation && header.sequence != 0) {
    return Fail(DataFrameErrorCode::kInvalidArgument,
                "Tick and Commit sequence must be zero", error);
  }
  if (header.kind == DataFrameKind::kObservation && header.sequence == 0) {
    return Fail(DataFrameErrorCode::kInvalidArgument,
                "Observation sequence must be nonzero", error);
  }
  return true;
}

bool ValidateKindAndFlags(const DataFrameHeader& header,
                          DataFrameError* error) {
  if (!IsKnownKind(header.kind)) {
    return Fail(DataFrameErrorCode::kInvalidKind,
                "data frame kind is not supported", error);
  }
  const std::uint8_t allowed_flags =
      header.kind == DataFrameKind::kTick ? kDataFrameRecoveryProof : 0U;
  if ((header.flags & static_cast<std::uint8_t>(~allowed_flags)) != 0U) {
    return Fail(DataFrameErrorCode::kInvalidFlags,
                "data frame contains unsupported flags", error);
  }
  return true;
}

}  // namespace

bool EncodeDataFrame(const DataFrameHeader& header,
                     std::span<const double> scalars, std::string* output,
                     DataFrameError* error) {
  if (output == nullptr || error == nullptr) {
    return false;
  }
  ClearError(error);
  if (!ValidateHeaderSemantics(header, error)) return false;
  if (scalars.size() >
      (kMaximumDataFrameBytes - kDataFrameHeaderSize) / sizeof(double)) {
    return Fail(DataFrameErrorCode::kFrameTooLarge,
                "data frame exceeds the protocol payload limit", error);
  }

  const auto scalar_count = static_cast<std::uint32_t>(scalars.size());
  const auto frame_size = static_cast<std::uint32_t>(
      kDataFrameHeaderSize + scalars.size() * sizeof(double));
  output->resize(frame_size);
  auto* bytes = reinterpret_cast<std::byte*>(output->data());
  std::copy(kMagic.begin(), kMagic.end(), bytes);
  WriteLittleEndian(kDataFramingMajor, bytes + 8);
  WriteLittleEndian(kDataFramingMinor, bytes + 10);
  bytes[12] = static_cast<std::byte>(header.kind);
  bytes[13] = static_cast<std::byte>(header.flags);
  WriteLittleEndian(static_cast<std::uint16_t>(kDataFrameHeaderSize),
                    bytes + 14);
  WriteLittleEndian(frame_size, bytes + 16);
  WriteLittleEndian(scalar_count, bytes + 20);
  WriteLittleEndian(header.session_id, bytes + 24);
  WriteLittleEndian(header.epoch, bytes + 32);
  WriteLittleEndian(header.tick_id, bytes + 40);
  WriteLittleEndian(header.layout.low, bytes + 48);
  WriteLittleEndian(header.layout.high, bytes + 56);
  WriteLittleEndian(header.sequence, bytes + 64);
  WriteLittleEndian(std::bit_cast<std::uint64_t>(header.simulation_time_s),
                    bytes + 72);
  for (std::size_t index = 0; index < scalars.size(); ++index) {
    WriteLittleEndian(std::bit_cast<std::uint64_t>(scalars[index]),
                      bytes + kDataFrameHeaderSize + index * sizeof(double));
  }
  return true;
}

bool ParseDataFrame(std::span<const std::byte> frame, ParsedDataFrame* output,
                    DataFrameError* error) {
  if (output == nullptr || error == nullptr) {
    return false;
  }
  ClearError(error);
  *output = {};
  if (frame.size() < kDataFrameHeaderSize) {
    return Fail(DataFrameErrorCode::kFrameTooSmall,
                "data frame is shorter than its fixed header", error);
  }
  if (frame.size() > kMaximumDataFrameBytes) {
    return Fail(DataFrameErrorCode::kFrameTooLarge,
                "data frame exceeds the protocol payload limit", error);
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), frame.begin())) {
    return Fail(DataFrameErrorCode::kMagicMismatch,
                "data frame magic mismatch", error);
  }
  if (ReadLittleEndian<std::uint16_t>(frame.data() + 8) !=
          kDataFramingMajor ||
      ReadLittleEndian<std::uint16_t>(frame.data() + 10) !=
          kDataFramingMinor) {
    return Fail(DataFrameErrorCode::kVersionMismatch,
                "unsupported data framing version", error);
  }

  DataFrameHeader header;
  header.kind = static_cast<DataFrameKind>(
      std::to_integer<std::uint8_t>(frame[12]));
  header.flags = std::to_integer<std::uint8_t>(frame[13]);
  if (!ValidateKindAndFlags(header, error)) return false;
  if (ReadLittleEndian<std::uint16_t>(frame.data() + 14) !=
      kDataFrameHeaderSize) {
    return Fail(DataFrameErrorCode::kHeaderSizeMismatch,
                "data frame header size mismatch", error);
  }
  const auto encoded_frame_size =
      ReadLittleEndian<std::uint32_t>(frame.data() + 16);
  if (encoded_frame_size != frame.size()) {
    return Fail(DataFrameErrorCode::kFrameSizeMismatch,
                "data frame size does not match its header", error);
  }
  const auto scalar_count =
      ReadLittleEndian<std::uint32_t>(frame.data() + 20);
  const std::size_t payload_size = frame.size() - kDataFrameHeaderSize;
  if (payload_size % sizeof(double) != 0 ||
      scalar_count != payload_size / sizeof(double)) {
    return Fail(DataFrameErrorCode::kScalarCountMismatch,
                "data frame scalar count does not match its payload", error);
  }

  header.session_id = ReadLittleEndian<std::uint64_t>(frame.data() + 24);
  header.epoch = ReadLittleEndian<std::uint64_t>(frame.data() + 32);
  header.tick_id = ReadLittleEndian<std::uint64_t>(frame.data() + 40);
  header.layout.low = ReadLittleEndian<std::uint64_t>(frame.data() + 48);
  header.layout.high = ReadLittleEndian<std::uint64_t>(frame.data() + 56);
  header.sequence = ReadLittleEndian<std::uint64_t>(frame.data() + 64);
  header.simulation_time_s = std::bit_cast<double>(
      ReadLittleEndian<std::uint64_t>(frame.data() + 72));
  if (!ValidateHeaderSemantics(header, error)) return false;
  output->header = header;
  output->scalar_count = scalar_count;
  output->scalar_bytes = frame.subspan(kDataFrameHeaderSize);
  return true;
}

bool DecodeDataFrameScalars(const ParsedDataFrame& frame,
                            std::span<double> output, DataFrameError* error) {
  if (error == nullptr) return false;
  ClearError(error);
  if (output.size() != frame.scalar_count ||
      frame.scalar_bytes.size() != output.size() * sizeof(double)) {
    return Fail(DataFrameErrorCode::kScalarCountMismatch,
                "data frame decode output has the wrong scalar count", error);
  }
  for (std::size_t index = 0; index < output.size(); ++index) {
    output[index] = std::bit_cast<double>(ReadLittleEndian<std::uint64_t>(
        frame.scalar_bytes.data() + index * sizeof(double)));
  }
  return true;
}

}  // namespace mujoco_sim::protocol
