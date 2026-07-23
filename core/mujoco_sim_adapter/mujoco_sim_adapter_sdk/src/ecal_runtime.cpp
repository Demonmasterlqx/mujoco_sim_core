#include "ecal_runtime.hpp"

#include <condition_variable>
#include <cstdio>
#include <exception>
#include <mutex>
#include <utility>

#include <ecal/ecal.h>

namespace mujoco_sim::adapter_sdk::internal {
namespace {

struct RuntimeState {
  enum class Phase {
    kIdle,
    kInitializing,
    kReady,
    kFinalizing,
  };

  std::mutex mutex;
  std::condition_variable condition;
  std::size_t leases = 0;
  bool owns_ecal = false;
  Phase phase = Phase::kIdle;
};

RuntimeState& State() {
  // Runtime leases may belong to clients with static storage duration. Keep
  // the process-wide bookkeeping alive until the OS reclaims the process.
  static RuntimeState* state = new RuntimeState();
  return *state;
}

struct FinalizeResult {
  bool finalized = false;
  bool remains_initialized = true;
};

FinalizeResult FinalizeOwnedRuntime() noexcept {
  FinalizeResult result;
  try {
    result.finalized = eCAL::Finalize();
    if (!result.finalized) result.remains_initialized = eCAL::IsInitialized();
  } catch (const std::exception& exception) {
    std::fprintf(stderr, "mujoco_sim_adapter_sdk: eCAL finalization threw: %s\n", exception.what());
  } catch (...) {
    std::fputs("mujoco_sim_adapter_sdk: eCAL finalization threw\n", stderr);
  }
  if (!result.finalized) {
    std::fputs("mujoco_sim_adapter_sdk: failed to finalize eCAL runtime\n", stderr);
  }
  return result;
}

void ReleaseLease() {
  auto& state = State();
  bool finalize = false;
  {
    std::unique_lock lock(state.mutex);
    state.condition.wait(lock, [&state] {
      return state.phase != RuntimeState::Phase::kInitializing &&
             state.phase != RuntimeState::Phase::kFinalizing;
    });
    if (state.leases == 0) return;
    --state.leases;
    if (state.leases != 0) return;
    finalize = state.owns_ecal;
    state.owns_ecal = false;
    state.phase = finalize ? RuntimeState::Phase::kFinalizing : RuntimeState::Phase::kIdle;
  }
  if (finalize) {
    const auto result = FinalizeOwnedRuntime();
    {
      std::lock_guard lock(state.mutex);
      state.owns_ecal = !result.finalized && result.remains_initialized;
      state.phase = state.owns_ecal ? RuntimeState::Phase::kReady : RuntimeState::Phase::kIdle;
    }
  }
  state.condition.notify_all();
}

void Release(void*) noexcept {
  try {
    ReleaseLease();
  } catch (const std::exception& exception) {
    std::fprintf(stderr, "mujoco_sim_adapter_sdk: eCAL lease release threw: %s\n",
                 exception.what());
  } catch (...) {
    std::fputs("mujoco_sim_adapter_sdk: eCAL lease release threw\n", stderr);
  }
}

std::shared_ptr<void> MakeLease() {
  return std::shared_ptr<void>(&State(), Release);
}

std::shared_ptr<void> TryMakeLease() noexcept {
  try {
    return MakeLease();
  } catch (...) {
    return {};
  }
}

struct LeaseAdmission {
  std::shared_ptr<void> lease;
  bool initialize = false;
};

LeaseAdmission AdmitLease(RuntimeState* state) {
  std::unique_lock lock(state->mutex);
  state->condition.wait(lock, [state] {
    return state->phase != RuntimeState::Phase::kInitializing &&
           state->phase != RuntimeState::Phase::kFinalizing;
  });
  if (state->phase == RuntimeState::Phase::kReady) {
    auto lease = TryMakeLease();
    if (lease) ++state->leases;
    return {std::move(lease), false};
  }
  state->phase = RuntimeState::Phase::kInitializing;
  return {{}, true};
}

struct RuntimeInitialization {
  bool initialized = false;
  bool owns_ecal = false;
};

RuntimeInitialization InitializeRuntime(const std::string& unit_name, std::string* error_message) {
  RuntimeInitialization result;
  try {
    result.initialized = eCAL::IsInitialized();
    if (!result.initialized) {
      result.owns_ecal = eCAL::Initialize(unit_name);
      result.initialized = result.owns_ecal;
    }
  } catch (const std::exception& exception) {
    *error_message = std::string("eCAL initialization threw: ") + exception.what();
  } catch (...) {
    *error_message = "eCAL initialization threw";
  }
  return result;
}

void CommitInitializedLease(RuntimeState* state, const std::shared_ptr<void>& lease, bool owns_ecal,
                            bool retained_owned_runtime) {
  {
    std::lock_guard lock(state->mutex);
    state->leases = lease ? 1 : 0;
    state->owns_ecal = lease ? owns_ecal : retained_owned_runtime;
    state->phase =
        state->owns_ecal || lease ? RuntimeState::Phase::kReady : RuntimeState::Phase::kIdle;
  }
  state->condition.notify_all();
}

}  // namespace

std::shared_ptr<void> AcquireEcalRuntime(const std::string& unit_name, std::string* error_message) {
  if (error_message == nullptr) {
    return {};
  }
  error_message->clear();
  auto& state = State();
  auto admission = AdmitLease(&state);
  if (!admission.initialize) {
    if (!admission.lease) *error_message = "failed to allocate eCAL runtime lease";
    return admission.lease;
  }

  const auto initialization = InitializeRuntime(unit_name, error_message);
  auto lease = initialization.initialized ? TryMakeLease() : std::shared_ptr<void>{};
  bool retained_owned_runtime = false;
  if (!lease && initialization.owns_ecal) {
    const auto cleanup = FinalizeOwnedRuntime();
    retained_owned_runtime = !cleanup.finalized && cleanup.remains_initialized;
  }
  CommitInitializedLease(&state, lease, initialization.owns_ecal, retained_owned_runtime);
  if (!lease) {
    if (error_message->empty()) {
      *error_message = initialization.initialized ? "failed to allocate eCAL runtime lease"
                                                  : "failed to initialize eCAL";
    }
    return {};
  }
  return lease;
}

}  // namespace mujoco_sim::adapter_sdk::internal
