#pragma once

#include <mujoco_sim/core/domain.hpp>

namespace mujoco_sim::core::internal {

LayoutFingerprint ComputeLayoutFingerprint(const ResourceLayout& layout);

}  // namespace mujoco_sim::core::internal
