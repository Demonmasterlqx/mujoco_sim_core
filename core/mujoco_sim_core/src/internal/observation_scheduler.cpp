#include "observation_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "adapter_registry.hpp"

namespace mujoco_sim::core::internal {

bool ObservationScheduler::Due(const ObserverRecord& observer,
                               std::uint64_t tick_id,
                               double timestep) const {
  if (!observer.last_delivery_tick) return true;
  if (observer.delivery.has_every_n_steps()) {
    return tick_id - *observer.last_delivery_tick >=
           observer.delivery.every_n_steps();
  }
  if (!observer.delivery.has_period_s()) return false;
  const long double elapsed_steps =
      static_cast<long double>(tick_id - observer.delivery_phase_tick);
  const long double elapsed =
      elapsed_steps * static_cast<long double>(timestep);
  const long double deadline =
      static_cast<long double>(observer.next_delivery_period) *
      static_cast<long double>(observer.delivery.period_s());
  const long double tolerance =
      static_cast<long double>(std::numeric_limits<double>::epsilon()) * 8.0L *
      std::max({1.0L, std::abs(elapsed), std::abs(deadline)});
  return elapsed + tolerance >= deadline;
}

void ObservationScheduler::MarkDelivered(ObserverRecord* observer,
                                          std::uint64_t tick_id,
                                          double timestep) const {
  const bool first_delivery = !observer->last_delivery_tick.has_value();
  observer->last_delivery_tick = tick_id;
  if (!observer->delivery.has_period_s()) return;
  const double period = observer->delivery.period_s();
  if (first_delivery) {
    observer->delivery_phase_tick = tick_id;
    observer->next_delivery_period = 1;
  } else if (period <= timestep) {
    observer->next_delivery_period =
        tick_id - observer->delivery_phase_tick + 1;
  } else {
    ++observer->next_delivery_period;
  }
}

}  // namespace mujoco_sim::core::internal
