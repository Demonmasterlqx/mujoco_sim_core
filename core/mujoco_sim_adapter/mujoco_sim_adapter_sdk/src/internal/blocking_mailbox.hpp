#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <utility>

namespace mujoco_sim::adapter_sdk::internal {

enum class MailboxWaitResult {
  kReceived,
  kTimeout,
  kStopped,
  kError,
  kConcurrentWait,
};

template <typename PublicResult>
PublicResult ToPublicWaitResult(MailboxWaitResult result) {
  switch (result) {
    case MailboxWaitResult::kReceived:
      return PublicResult::kReceived;
    case MailboxWaitResult::kTimeout:
      return PublicResult::kTimeout;
    case MailboxWaitResult::kStopped:
      return PublicResult::kStopped;
    case MailboxWaitResult::kError:
    case MailboxWaitResult::kConcurrentWait:
      return PublicResult::kError;
  }
  return PublicResult::kError;
}

// The owner supplies the lock and condition variable so lifecycle state and
// mailbox termination can be changed atomically under one mutex.
template <typename T>
class BlockingMailbox {
 public:
  std::size_t Push(T value, std::size_t capacity) {
    std::size_t dropped = 0;
    if (items_.size() == capacity) {
      items_.pop_front();
      dropped = 1;
    }
    items_.push_back(std::move(value));
    return dropped;
  }

  template <typename StopPredicate, typename ErrorProvider>
  MailboxWaitResult Wait(std::unique_lock<std::mutex>& lock,
                         std::condition_variable& condition,
                         std::chrono::milliseconds timeout,
                         StopPredicate stopped, ErrorProvider error,
                         T* output, std::string* error_message) {
    if (wait_in_progress_) return MailboxWaitResult::kConcurrentWait;
    wait_in_progress_ = true;
    const auto finish = [this] { wait_in_progress_ = false; };
    const auto ready = [&] {
      return !items_.empty() || !error().empty() || stopped();
    };
    bool signaled = ready();
    if (!signaled && timeout == std::chrono::milliseconds::max()) {
      while (!(signaled = ready())) {
        condition.wait_for(lock, std::chrono::milliseconds(50));
      }
    } else if (!signaled) {
      signaled = condition.wait_for(lock, timeout, ready);
    }
    if (!signaled) {
      finish();
      return MailboxWaitResult::kTimeout;
    }
    const auto terminal_error = error();
    if (!terminal_error.empty()) {
      *error_message = terminal_error;
      finish();
      return MailboxWaitResult::kError;
    }
    if (stopped()) {
      finish();
      return MailboxWaitResult::kStopped;
    }
    *output = std::move(items_.front());
    items_.pop_front();
    finish();
    return MailboxWaitResult::kReceived;
  }

  void Clear() { items_.clear(); }
  std::size_t size() const { return items_.size(); }
  bool wait_in_progress() const { return wait_in_progress_; }

 private:
  std::deque<T> items_;
  bool wait_in_progress_ = false;
};

}  // namespace mujoco_sim::adapter_sdk::internal
