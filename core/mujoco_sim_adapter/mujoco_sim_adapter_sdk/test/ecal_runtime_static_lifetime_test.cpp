#include <memory>
#include <string>

#include "ecal_runtime.hpp"

namespace {

std::shared_ptr<void> global_lease;

struct LateLeaseReset {
  ~LateLeaseReset() { global_lease.reset(); }
};

LateLeaseReset late_lease_reset;

}  // namespace

int main() {
  std::string error;
  global_lease = mujoco_sim::adapter_sdk::internal::AcquireEcalRuntime(
      "mujoco-sim-static-lifetime-test", &error);
  return global_lease ? 0 : 1;
}
