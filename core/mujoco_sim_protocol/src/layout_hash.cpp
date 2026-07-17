#include <mujoco_sim/protocol/layout_hash.hpp>

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <string>

namespace mujoco_sim::protocol {
namespace {

void AppendU32(std::uint32_t value, std::string* output) {
  for (unsigned int index = 0; index < 4; ++index) {
    output->push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  }
}

void AppendU64(std::uint64_t value, std::string* output) {
  for (unsigned int index = 0; index < 8; ++index) {
    output->push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  }
}

void AppendString(const std::string& value, std::string* output) {
  AppendU32(static_cast<std::uint32_t>(value.size()), output);
  output->append(value);
}

void AppendDescriptor(const mujoco_sim::v4::ResourceDescriptor& descriptor, std::string* output) {
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

std::string CanonicalLayoutBytes(const mujoco_sim::v4::ResourceLayout& layout) {
  std::string output;
  output.push_back('\x01');
  output.push_back('O');
  AppendU32(static_cast<std::uint32_t>(layout.observations_size()), &output);
  for (const auto& slot : layout.observations()) {
    AppendU32(slot.scalar_offset(), &output);
    AppendDescriptor(slot.resource(), &output);
  }
  AppendU32(layout.observation_scalar_count(), &output);
  output.push_back('C');
  AppendU32(static_cast<std::uint32_t>(layout.commands_size()), &output);
  for (const auto& slot : layout.commands()) {
    AppendU32(slot.scalar_offset(), &output);
    AppendDescriptor(slot.resource(), &output);
  }
  AppendU32(layout.command_scalar_count(), &output);
  return output;
}

template <class Slots>
bool ValidateScalarLayout(const Slots& slots, std::uint32_t scalar_count, std::string_view name,
                          std::string* error_message) {
  std::uint64_t expected_offset = 0;
  for (const auto& slot : slots) {
    if (slot.resource().dimension() == 0 || slot.scalar_offset() != expected_offset) {
      if (error_message != nullptr) {
        *error_message = std::string(name) + " scalar offsets must be contiguous";
      }
      return false;
    }
    expected_offset += slot.resource().dimension();
    if (expected_offset > std::numeric_limits<std::uint32_t>::max()) {
      if (error_message != nullptr) {
        *error_message = std::string(name) + " scalar count overflows";
      }
      return false;
    }
  }
  if (expected_offset != scalar_count) {
    if (error_message != nullptr) {
      *error_message = std::string(name) + " scalar count is inconsistent";
    }
    return false;
  }
  return true;
}

std::array<std::uint32_t, 4> MultiplyFNVPrime(const std::array<std::uint32_t, 4>& value) {
  constexpr std::array<std::uint32_t, 4> kPrime = {0x0000013bU, 0x00000000U, 0x01000000U,
                                                   0x00000000U};
  std::array<std::uint32_t, 4> result{};
  for (std::size_t left = 0; left < value.size(); ++left) {
    std::uint64_t carry = 0;
    for (std::size_t right = 0; left + right < value.size(); ++right) {
      const std::size_t index = left + right;
      const std::uint64_t product =
          static_cast<std::uint64_t>(value[left]) * kPrime[right] + result[index] + carry;
      result[index] = static_cast<std::uint32_t>(product);
      carry = product >> 32U;
    }
  }
  return result;
}

std::string FNV1a128(std::string_view input) {
  std::array<std::uint32_t, 4> state = {0x6295c58dU, 0x62b82175U, 0x07bb0142U, 0x6c62272eU};
  for (const unsigned char byte : input) {
    state[0] ^= byte;
    state = MultiplyFNVPrime(state);
  }
  std::string output;
  output.reserve(kLayoutHashSize);
  for (std::size_t limb = 0; limb < state.size(); ++limb) {
    for (unsigned int byte = 0; byte < 4; ++byte) {
      output.push_back(static_cast<char>((state[limb] >> (byte * 8U)) & 0xffU));
    }
  }
  return output;
}

}  // namespace

std::string ComputeLayoutHash(const mujoco_sim::v4::ResourceLayout& layout) {
  return FNV1a128(CanonicalLayoutBytes(layout));
}

bool ValidateLayoutHash(std::string_view layout_hash, std::string* error_message) {
  if (layout_hash.size() != kLayoutHashSize) {
    if (error_message != nullptr) {
      *error_message = "layout_hash must contain exactly 16 bytes";
    }
    return false;
  }
  if (error_message != nullptr) error_message->clear();
  return true;
}

bool ValidateLayoutHash(const mujoco_sim::v4::ResourceLayout& layout, std::string* error_message) {
  if (!ValidateScalarLayout(layout.observations(), layout.observation_scalar_count(), "observation",
                            error_message) ||
      !ValidateScalarLayout(layout.commands(), layout.command_scalar_count(), "command",
                            error_message)) {
    return false;
  }
  if (!ValidateLayoutHash(layout.layout_fingerprint(), error_message)) return false;
  if (layout.layout_fingerprint() != ComputeLayoutHash(layout)) {
    if (error_message != nullptr) {
      *error_message = "layout_hash does not match the resource descriptors";
    }
    return false;
  }
  if (error_message != nullptr) error_message->clear();
  return true;
}

}  // namespace mujoco_sim::protocol
