#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <string>
#include <string_view>

#include <mujoco_sim/core/domain.hpp>

namespace mujoco_sim::core::internal {

inline bool ValidateAdapterId(std::string_view value, std::string* error) {
  const auto fail = [&](const char* reason) {
    if (error != nullptr) *error = std::string("adapter_id ") + reason;
    return false;
  };
  const auto alphanumeric = [](unsigned char value) {
    return (value >= 'A' && value <= 'Z') ||
           (value >= 'a' && value <= 'z') ||
           (value >= '0' && value <= '9');
  };
  if (value.empty()) return fail("must not be empty");
  if (value.size() > 64) return fail("must not exceed 64 characters");
  if (!alphanumeric(static_cast<unsigned char>(value.front()))) {
    return fail("must start with an ASCII letter or digit");
  }
  for (const unsigned char character : value) {
    if (!alphanumeric(character) && character != '_' && character != '-') {
      return fail("may contain only ASCII letters, digits, '_' and '-'");
    }
  }
  if (error != nullptr) error->clear();
  return true;
}

inline void AppendU32(std::uint32_t value, std::string* output) {
  for (unsigned int index = 0; index < 4; ++index) {
    output->push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  }
}

inline void AppendU64(std::uint64_t value, std::string* output) {
  for (unsigned int index = 0; index < 8; ++index) {
    output->push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  }
}

inline void AppendString(const std::string& value, std::string* output) {
  AppendU32(static_cast<std::uint32_t>(value.size()), output);
  output->append(value);
}

inline void AppendDescriptor(const ResourceDescriptor& descriptor,
                             std::string* output) {
  AppendString(descriptor.name(), output);
  AppendU32(static_cast<std::uint32_t>(descriptor.type()), output);
  AppendU32(descriptor.dimension(), output);
  AppendU32(static_cast<std::uint32_t>(descriptor.mujoco_id()), output);
  AppendU32(static_cast<std::uint32_t>(descriptor.mujoco_type()), output);
  AppendU32(static_cast<std::uint32_t>(descriptor.mujoco_datatype()), output);
  output->push_back(descriptor.has_ctrl_range() ? '\x01' : '\x00');
  AppendU64(std::bit_cast<std::uint64_t>(descriptor.ctrl_min()), output);
  AppendU64(std::bit_cast<std::uint64_t>(descriptor.ctrl_max()), output);
  AppendU32(static_cast<std::uint32_t>(descriptor.mujoco_stage()), output);
  AppendU64(std::bit_cast<std::uint64_t>(descriptor.interval_period_s()), output);
  AppendU64(std::bit_cast<std::uint64_t>(descriptor.interval_phase_s()), output);
  AppendU64(std::bit_cast<std::uint64_t>(descriptor.delay_s()), output);
  AppendString(descriptor.frame_id(), output);
  AppendString(descriptor.unit(), output);
  AppendString(descriptor.object_name(), output);
  AppendU32(static_cast<std::uint32_t>(descriptor.object_type()), output);
  AppendString(descriptor.interpolation(), output);
}

inline LayoutFingerprint ComputeLayoutFingerprint(const ResourceLayout& layout) {
  std::string bytes;
  bytes.push_back('\x01');
  bytes.push_back('O');
  AppendU32(static_cast<std::uint32_t>(layout.observations_.size()), &bytes);
  for (const auto& descriptor : layout.observations_) {
    AppendDescriptor(descriptor, &bytes);
  }
  bytes.push_back('C');
  AppendU32(static_cast<std::uint32_t>(layout.commands_.size()), &bytes);
  for (const auto& descriptor : layout.commands_) AppendDescriptor(descriptor, &bytes);

  std::array<std::uint32_t, 4> state = {
      0x6295c58dU, 0x62b82175U, 0x07bb0142U, 0x6c62272eU};
  constexpr std::array<std::uint32_t, 4> prime = {
      0x0000013bU, 0x00000000U, 0x01000000U, 0x00000000U};
  for (const unsigned char byte : bytes) {
    state[0] ^= byte;
    std::array<std::uint32_t, 4> product{};
    for (std::size_t left = 0; left < state.size(); ++left) {
      std::uint64_t carry = 0;
      for (std::size_t right = 0; left + right < state.size(); ++right) {
        const auto index = left + right;
        const std::uint64_t value =
            static_cast<std::uint64_t>(state[left]) * prime[right] +
            product[index] + carry;
        product[index] = static_cast<std::uint32_t>(value);
        carry = value >> 32U;
      }
    }
    state = product;
  }
  return {static_cast<std::uint64_t>(state[0]) |
              (static_cast<std::uint64_t>(state[1]) << 32U),
          static_cast<std::uint64_t>(state[2]) |
              (static_cast<std::uint64_t>(state[3]) << 32U)};
}

inline void FinalizeLayout(ResourceLayout* layout) {
  layout->observation_spans.clear();
  layout->command_spans.clear();
  layout->observation_scalar_count = 0;
  layout->command_scalar_count = 0;
  for (const auto& descriptor : layout->observations_) {
    layout->observation_spans.push_back(
        {layout->observation_scalar_count, descriptor.dimension()});
    layout->observation_scalar_count += descriptor.dimension();
  }
  for (const auto& descriptor : layout->commands_) {
    layout->command_spans.push_back(
        {layout->command_scalar_count, descriptor.dimension()});
    layout->command_scalar_count += descriptor.dimension();
  }
  layout->fingerprint = ComputeLayoutFingerprint(*layout);
}

}  // namespace mujoco_sim::core::internal
