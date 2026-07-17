#pragma once

#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

namespace mujoco_sim::server {

inline constexpr std::size_t kMaxProtocolPayloadBytes = 4U * 1024U * 1024U;

bool IsProtocolPayloadSizeAllowed(std::size_t size);

bool RunWithExceptionBoundary(const std::function<void()>& action,
                              std::string* error_message) noexcept;

template <typename Action>
bool RunWithExceptionBoundary(Action&& action, std::string* error_message) noexcept {
  try {
    std::forward<Action>(action)();
    if (error_message != nullptr) error_message->clear();
    return true;
  } catch (const std::exception& exception) {
    if (error_message != nullptr) {
      try {
        *error_message = exception.what();
      } catch (...) {
      }
    }
  } catch (...) {
    if (error_message != nullptr) {
      try {
        *error_message = "non-standard exception";
      } catch (...) {
      }
    }
  }
  return false;
}

class CallbackState {
 public:
  bool Enter();
  void Leave();
  void StopAccepting();
  void WaitForIdle();

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t active_ = 0;
  bool accepting_ = true;
};

}  // namespace mujoco_sim::server
