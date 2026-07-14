#pragma once

#include <exception>
#include <utility>

namespace mujoco_sim::adapter_sdk::internal {

template <typename Callback, typename OnError>
void InvokeCallbackBoundary(Callback&& callback, OnError&& on_error) noexcept {
  try {
    std::forward<Callback>(callback)();
  } catch (const std::exception& error) {
    try {
      std::forward<OnError>(on_error)(error.what());
    } catch (...) {
    }
  } catch (...) {
    try {
      std::forward<OnError>(on_error)("unknown exception");
    } catch (...) {
    }
  }
}

}  // namespace mujoco_sim::adapter_sdk::internal
