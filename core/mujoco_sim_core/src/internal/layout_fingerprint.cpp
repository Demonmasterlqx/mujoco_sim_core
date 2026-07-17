#include "layout_fingerprint.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <string>

namespace mujoco_sim::core::internal {

LayoutFingerprint ComputeLayoutFingerprint(const ResourceLayout& layout) {
  std::string canonical;
  const auto append_u32 = [&canonical](std::uint32_t value) {
    for (unsigned int index = 0; index < 4; ++index)
      canonical.push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  };
  const auto append_u64 = [&canonical](std::uint64_t value) {
    for (unsigned int index = 0; index < 8; ++index)
      canonical.push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  };
  const auto append_string = [&](const std::string& value) {
    append_u32(static_cast<std::uint32_t>(value.size()));
    canonical.append(value);
  };
  const auto append_descriptor = [&](const ResourceDescriptor& descriptor) {
    append_string(descriptor.name());
    append_u32(static_cast<std::uint32_t>(descriptor.type()));
    append_u32(descriptor.dimension());
    append_u32(static_cast<std::uint32_t>(descriptor.mujoco_id()));
    append_u32(static_cast<std::uint32_t>(descriptor.mujoco_type()));
    append_u32(static_cast<std::uint32_t>(descriptor.mujoco_datatype()));
    canonical.push_back(descriptor.has_ctrl_range() ? '\x01' : '\x00');
    append_u64(std::bit_cast<std::uint64_t>(descriptor.ctrl_min()));
    append_u64(std::bit_cast<std::uint64_t>(descriptor.ctrl_max()));
    append_u32(static_cast<std::uint32_t>(descriptor.mujoco_stage()));
    append_u64(std::bit_cast<std::uint64_t>(descriptor.interval_period_s()));
    append_u64(std::bit_cast<std::uint64_t>(descriptor.interval_phase_s()));
    append_u64(std::bit_cast<std::uint64_t>(descriptor.delay_s()));
    append_string(descriptor.frame_id());
    append_string(descriptor.unit());
    append_string(descriptor.object_name());
    append_u32(static_cast<std::uint32_t>(descriptor.object_type()));
    append_string(descriptor.interpolation());
  };
  canonical.push_back('\x01');
  canonical.push_back('O');
  append_u32(static_cast<std::uint32_t>(layout.observations_.size()));
  for (std::size_t index = 0; index < layout.observations_.size(); ++index) {
    append_u32(static_cast<std::uint32_t>(layout.observation_spans[index].offset));
    append_descriptor(layout.observations_[index]);
  }
  append_u32(static_cast<std::uint32_t>(layout.observation_scalar_count));
  canonical.push_back('C');
  append_u32(static_cast<std::uint32_t>(layout.commands_.size()));
  for (std::size_t index = 0; index < layout.commands_.size(); ++index) {
    append_u32(static_cast<std::uint32_t>(layout.command_spans[index].offset));
    append_descriptor(layout.commands_[index]);
  }
  append_u32(static_cast<std::uint32_t>(layout.command_scalar_count));

  std::array<std::uint32_t, 4> state = {0x6295c58dU, 0x62b82175U, 0x07bb0142U, 0x6c62272eU};
  constexpr std::array<std::uint32_t, 4> prime = {0x0000013bU, 0x00000000U, 0x01000000U,
                                                  0x00000000U};
  for (const unsigned char byte : canonical) {
    state[0] ^= byte;
    std::array<std::uint32_t, 4> product{};
    for (std::size_t left = 0; left < state.size(); ++left) {
      std::uint64_t carry = 0;
      for (std::size_t right = 0; left + right < state.size(); ++right) {
        const std::size_t index = left + right;
        const std::uint64_t value =
            static_cast<std::uint64_t>(state[left]) * prime[right] + product[index] + carry;
        product[index] = static_cast<std::uint32_t>(value);
        carry = value >> 32U;
      }
    }
    state = product;
  }
  return {static_cast<std::uint64_t>(state[0]) | (static_cast<std::uint64_t>(state[1]) << 32U),
          static_cast<std::uint64_t>(state[2]) | (static_cast<std::uint64_t>(state[3]) << 32U)};
}

}  // namespace mujoco_sim::core::internal
