#include "ecal_runtime.hpp"

#include <mutex>

#include <ecal/ecal.h>

namespace mujoco_sim::adapter_sdk::internal {
namespace {

struct RuntimeState {
  std::mutex mutex;
  std::size_t leases = 0;
  bool owns_ecal = false;
};

RuntimeState& State() {
  // Runtime leases may belong to clients with static storage duration. Keep
  // the process-wide bookkeeping alive until the OS reclaims the process.
  static RuntimeState* state = new RuntimeState();
  return *state;
}

void Release(void*) {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (state.leases == 0) {
    return;
  }
  --state.leases;
  if (state.leases == 0 && state.owns_ecal) {
    eCAL::Finalize();
    state.owns_ecal = false;
  }
}

}  // namespace

std::shared_ptr<void> AcquireEcalRuntime(const std::string& unit_name,
                                         std::string* error_message) {
  if (error_message == nullptr) {
    return {};
  }
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (state.leases == 0 && !eCAL::IsInitialized()) {
    if (!eCAL::Initialize(unit_name)) {
      *error_message = "failed to initialize eCAL";
      return {};
    }
    state.owns_ecal = true;
  }
  ++state.leases;
  return std::shared_ptr<void>(reinterpret_cast<void*>(1), Release);
}

}  // namespace mujoco_sim::adapter_sdk::internal
