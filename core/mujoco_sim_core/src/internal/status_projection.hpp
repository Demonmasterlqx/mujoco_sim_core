#pragma once

#include <string>

#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::core::internal {

class AdapterRegistry;
class BarrierCoordinator;

class StatusProjection {
 public:
  mujoco_sim::v2::ServerStatus Build(
      const mujoco_sim::v2::Header& header,
      mujoco_sim::v2::ServerState server_state, const std::string& message,
      const AdapterRegistry& registry,
      const BarrierCoordinator& barrier) const;
};

}  // namespace mujoco_sim::core::internal
