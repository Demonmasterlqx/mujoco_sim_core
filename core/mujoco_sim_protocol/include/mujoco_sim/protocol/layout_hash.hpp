#pragma once

#include <string>
#include <string_view>

#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::protocol {

inline constexpr std::size_t kLayoutHashSize = 16;

std::string ComputeLayoutHash(const mujoco_sim::v4::ResourceLayout& layout);
bool ValidateLayoutHash(const mujoco_sim::v4::ResourceLayout& layout, std::string* error_message);
bool ValidateLayoutHash(std::string_view layout_hash, std::string* error_message);

}  // namespace mujoco_sim::protocol
