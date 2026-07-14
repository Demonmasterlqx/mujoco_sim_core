#pragma once

#include <cstdint>

namespace mujoco_sim::core::internal {

struct ObserverRecord;

class ObservationScheduler {
 public:
  bool Due(const ObserverRecord& observer, std::uint64_t tick_id,
           double timestep) const;
  void MarkDelivered(ObserverRecord* observer, std::uint64_t tick_id,
                     double timestep) const;
};

}  // namespace mujoco_sim::core::internal
