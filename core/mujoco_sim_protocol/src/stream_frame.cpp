#include <mujoco_sim/protocol/stream_frame.hpp>

#include <algorithm>
#include <array>
#include <cstring>

namespace mujoco_sim::protocol {
namespace {

constexpr std::array<std::byte, 8> kMagic = {
    static_cast<std::byte>('M'), static_cast<std::byte>('J'),
    static_cast<std::byte>('S'), static_cast<std::byte>('T'),
    static_cast<std::byte>('R'), static_cast<std::byte>('E'),
    static_cast<std::byte>('A'), static_cast<std::byte>('M')};

template <typename T>
void WriteLittleEndian(T value, std::byte* output) {
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    output[i] = static_cast<std::byte>(value & 0xffU);
    value >>= 8U;
  }
}

template <typename T>
T ReadLittleEndian(const std::byte* input) {
  T value = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    value |= static_cast<T>(std::to_integer<unsigned int>(input[i]))
             << (8U * i);
  }
  return value;
}

bool Fail(const std::string& message, std::string* error_message) {
  if (error_message != nullptr) {
    *error_message = message;
  }
  return false;
}

}  // namespace

bool SerializeStreamFrame(const mujoco_sim::v3::StreamFrameHeader& header,
                          std::span<const std::byte> payload,
                          std::string* output,
                          std::string* error_message) {
  if (output == nullptr || error_message == nullptr) {
    return false;
  }
  error_message->clear();
  if (payload.size() > kMaximumStreamPayloadBytes) {
    return Fail("stream payload exceeds maximum size", error_message);
  }
  if (header.payload_size() != payload.size()) {
    return Fail("stream header payload_size does not match payload",
                error_message);
  }
  std::string serialized_header;
  if (!header.SerializeToString(&serialized_header)) {
    return Fail("failed to serialize stream frame header", error_message);
  }
  if (serialized_header.size() > kMaximumStreamHeaderBytes) {
    return Fail("stream frame header exceeds maximum size", error_message);
  }

  std::string framed(kStreamFramePrefixSize + serialized_header.size() +
                         payload.size(),
                     '\0');
  auto* bytes = reinterpret_cast<std::byte*>(framed.data());
  std::memcpy(bytes, kMagic.data(), kMagic.size());
  WriteLittleEndian(kStreamFramingMajor, bytes + 8);
  WriteLittleEndian(kStreamFramingMinor, bytes + 10);
  WriteLittleEndian(static_cast<std::uint32_t>(serialized_header.size()),
                    bytes + 12);
  WriteLittleEndian(static_cast<std::uint64_t>(payload.size()), bytes + 16);
  std::memcpy(bytes + kStreamFramePrefixSize, serialized_header.data(),
              serialized_header.size());
  if (!payload.empty()) {
    std::memcpy(bytes + kStreamFramePrefixSize + serialized_header.size(),
                payload.data(), payload.size());
  }
  output->swap(framed);
  return true;
}

bool ParseStreamFrame(std::span<const std::byte> frame,
                      ParsedStreamFrame* output,
                      std::string* error_message) {
  if (output == nullptr || error_message == nullptr) {
    return false;
  }
  error_message->clear();
  output->header.Clear();
  output->payload = {};
  if (frame.size() < kStreamFramePrefixSize) {
    return Fail("stream frame is shorter than its prefix", error_message);
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), frame.begin())) {
    return Fail("stream frame magic mismatch", error_message);
  }
  const auto major = ReadLittleEndian<std::uint16_t>(frame.data() + 8);
  if (major != kStreamFramingMajor) {
    return Fail("unsupported stream framing major version", error_message);
  }
  const auto minor = ReadLittleEndian<std::uint16_t>(frame.data() + 10);
  if (minor != kStreamFramingMinor) {
    return Fail("unsupported stream framing minor version", error_message);
  }
  const auto header_size =
      ReadLittleEndian<std::uint32_t>(frame.data() + 12);
  const auto payload_size =
      ReadLittleEndian<std::uint64_t>(frame.data() + 16);
  if (payload_size > kMaximumStreamPayloadBytes) {
    return Fail("stream payload exceeds maximum size", error_message);
  }
  if (header_size > kMaximumStreamHeaderBytes) {
    return Fail("stream frame header exceeds maximum size", error_message);
  }
  const std::uint64_t expected = kStreamFramePrefixSize +
                                 static_cast<std::uint64_t>(header_size) +
                                 payload_size;
  if (expected != frame.size()) {
    return Fail("stream frame size does not match prefix", error_message);
  }
  const auto* header_data = frame.data() + kStreamFramePrefixSize;
  if (!output->header.ParseFromArray(header_data,
                                     static_cast<int>(header_size))) {
    output->header.Clear();
    return Fail("failed to parse stream frame header", error_message);
  }
  if (output->header.payload_size() != payload_size) {
    output->header.Clear();
    return Fail("stream frame header payload_size mismatch", error_message);
  }
  output->payload = frame.subspan(kStreamFramePrefixSize + header_size,
                                  static_cast<std::size_t>(payload_size));
  return true;
}

}  // namespace mujoco_sim::protocol
