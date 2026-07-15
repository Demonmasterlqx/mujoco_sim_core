#pragma once

#include <string>

#include <mujoco_sim/core/domain.hpp>


namespace mujoco_sim::core::internal {

class AdapterRegistry;
class BarrierCoordinator;

class StatusProjection {
 public:
  mujoco_sim::core::ServerStatus Build(
      std::uint64_t epoch, std::uint64_t tick_id, double simulation_time_s,
      mujoco_sim::core::ServerState server_state, const std::string& message,
      const AdapterRegistry& registry,
      const BarrierCoordinator& barrier) const;
};

}  // namespace mujoco_sim::core::internal
