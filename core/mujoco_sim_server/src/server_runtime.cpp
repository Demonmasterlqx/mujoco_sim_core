#include "server_runtime.hpp"

#include <limits>

namespace mujoco_sim::server {

bool IsProtocolPayloadSizeAllowed(std::size_t size) {
  return size <= kMaxProtocolPayloadBytes &&
         size <= static_cast<std::size_t>(std::numeric_limits<int>::max());
}

bool RunWithExceptionBoundary(const std::function<void()>& action,
                              std::string* error_message) noexcept {
  return RunWithExceptionBoundary<const std::function<void()>&>(action, error_message);
}

bool CallbackState::Enter() {
  std::lock_guard lock(mutex_);
  if (!accepting_) return false;
  ++active_;
  return true;
}

void CallbackState::Leave() {
  std::lock_guard lock(mutex_);
  if (--active_ == 0) condition_.notify_all();
}

void CallbackState::StopAccepting() {
  std::lock_guard lock(mutex_);
  accepting_ = false;
}

void CallbackState::WaitForIdle() {
  std::unique_lock lock(mutex_);
  condition_.wait(lock, [this]() { return active_ == 0; });
}

}  // namespace mujoco_sim::server
