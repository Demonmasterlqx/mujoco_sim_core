#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace mujoco_sim::server {

inline constexpr std::size_t kMaxProtocolPayloadBytes =
    4U * 1024U * 1024U;

bool IsProtocolPayloadSizeAllowed(std::size_t size);

bool RunWithExceptionBoundary(const std::function<void()>& action,
                              std::string* error_message) noexcept;

template <typename Action>
bool RunWithExceptionBoundary(Action&& action,
                              std::string* error_message) noexcept {
  try {
    std::forward<Action>(action)();
    if (error_message != nullptr) {
      error_message->clear();
    }
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

class BestEffortCleanup {
 public:
  using FailureHandler =
      std::function<void(std::string_view operation, const std::string& error)>;

  explicit BestEffortCleanup(FailureHandler failure_handler)
      : failure_handler_(std::move(failure_handler)) {}

  template <typename Action>
  void Run(std::string_view operation, Action&& action) noexcept {
    std::string error;
    if (RunWithExceptionBoundary(std::forward<Action>(action), &error)) {
      return;
    }
    succeeded_ = false;
    try {
      if (failure_handler_) {
        failure_handler_(operation, error);
      }
    } catch (...) {
    }
  }

  bool succeeded() const { return succeeded_; }

 private:
  FailureHandler failure_handler_;
  bool succeeded_ = true;
};

enum class SendOutcome {
  kSent,
  kNoSubscribers,
  kRejectedWithSubscribers,
};

SendOutcome ClassifySendOutcome(bool sent, std::size_t subscribers_before,
                                std::size_t subscribers_after);

std::chrono::steady_clock::time_point ComputeLoopWakeDeadline(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point next_tick, bool tick_open,
    std::chrono::steady_clock::duration poll_interval);

struct RuntimeRates {
  double step_rate_hz = 0.0;
  double real_time_factor = 0.0;
  double barrier_latency_avg_ms = 0.0;
  double barrier_latency_max_ms = 0.0;
};

RuntimeRates CalculateRuntimeRates(
    double window_seconds, std::uint64_t start_tick, std::uint64_t end_tick,
    double start_simulation_time, double end_simulation_time,
    std::uint64_t completed_barriers, double barrier_latency_total_ms,
    double barrier_latency_max_ms);

struct RuntimeMetricsSnapshot {
  double window_seconds = 0.0;
  std::uint64_t tick_id = 0;
  double simulation_time = 0.0;
  std::uint64_t ticks_opened = 0;
  std::uint64_t tick_publish_attempts = 0;
  std::uint64_t tick_serialization_failures = 0;
  std::uint64_t tick_messages_sent = 0;
  std::uint64_t tick_send_rejected_with_subscribers = 0;
  std::uint64_t tick_no_subscribers = 0;
  std::uint64_t tick_retransmissions = 0;
  std::uint64_t tick_serializations = 0;
  std::uint64_t tick_cache_hits = 0;
  std::uint64_t tick_cache_misses = 0;
  std::uint64_t tick_cached_bytes = 0;
  std::uint64_t layout_bytes_avoided = 0;
  std::uint64_t layout_hash_mismatches = 0;
  std::uint64_t blocking_wait_count = 0;
  double blocking_wait_total_ms = 0.0;
  double blocking_wait_max_ms = 0.0;
  std::uint64_t scheduler_wakeups = 0;
  std::uint64_t scheduler_late_wakeups = 0;
  std::uint64_t commits = 0;
  std::uint64_t duplicate_commits = 0;
  std::uint64_t rejected_commits = 0;
  std::uint64_t timeouts = 0;
  std::uint64_t recoveries = 0;
  std::uint64_t replacements = 0;
  std::uint64_t local_errors = 0;
  std::uint64_t activation_expirations = 0;
  std::uint64_t malformed_messages = 0;
  std::uint64_t suppressed_warnings = 0;
  std::uint64_t completed_barriers = 0;
  RuntimeRates rates;
};

class RuntimeMetrics {
 public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void Initialize(TimePoint now, std::uint64_t tick_id,
                  double simulation_time);
  bool initialized() const { return initialized_; }
  bool SummaryDue(TimePoint now, std::chrono::milliseconds period) const;

  void RecordTickOpened(TimePoint now);
  void RecordTickPublishBatch(std::uint64_t attempts,
                              std::uint64_t serialization_failures,
                              std::uint64_t sent,
                              std::uint64_t rejected_with_subscribers,
                              std::uint64_t no_subscribers,
                              std::uint64_t retransmissions);
  void RecordTickCache(std::uint64_t serializations, std::uint64_t hits,
                       std::uint64_t misses, std::uint64_t cached_bytes,
                       std::uint64_t layout_bytes_avoided);
  void RecordLayoutHashMismatch() { ++layout_hash_mismatches_; }
  void RecordBlockingWait(double elapsed_ms);
  void RecordSchedulerWakeup(bool late) {
    ++scheduler_wakeups_;
    scheduler_late_wakeups_ += late ? 1U : 0U;
  }
  double RecordCommit(bool duplicate, bool rejected, bool stepped,
                      TimePoint now);
  double RecordBarrierCompleted(TimePoint now);
  void RecordTimeout() { ++timeouts_; }
  void RecordRecovery() { ++recoveries_; }
  void RecordReplacement() { ++replacements_; }
  void RecordLocalError() { ++local_errors_; }
  void RecordActivationExpiration() { ++activation_expirations_; }
  void RecordMalformedMessage() { ++malformed_messages_; }
  void RecordSuppressedWarning() { ++suppressed_warnings_; }

  RuntimeMetricsSnapshot TakeSummary(TimePoint now, std::uint64_t tick_id,
                                     double simulation_time);

 private:
  void ResetWindow(TimePoint now, std::uint64_t tick_id,
                   double simulation_time);

  TimePoint window_started_{};
  std::uint64_t window_start_tick_ = 0;
  double window_start_simulation_time_ = 0.0;
  std::uint64_t ticks_opened_ = 0;
  std::uint64_t tick_publish_attempts_ = 0;
  std::uint64_t tick_serialization_failures_ = 0;
  std::uint64_t tick_messages_sent_ = 0;
  std::uint64_t tick_send_rejected_with_subscribers_ = 0;
  std::uint64_t tick_no_subscribers_ = 0;
  std::uint64_t tick_retransmissions_ = 0;
  std::uint64_t tick_serializations_ = 0;
  std::uint64_t tick_cache_hits_ = 0;
  std::uint64_t tick_cache_misses_ = 0;
  std::uint64_t tick_cached_bytes_ = 0;
  std::uint64_t layout_bytes_avoided_ = 0;
  std::uint64_t layout_hash_mismatches_ = 0;
  std::uint64_t blocking_wait_count_ = 0;
  double blocking_wait_total_ms_ = 0.0;
  double blocking_wait_max_ms_ = 0.0;
  std::uint64_t scheduler_wakeups_ = 0;
  std::uint64_t scheduler_late_wakeups_ = 0;
  std::uint64_t commits_ = 0;
  std::uint64_t duplicate_commits_ = 0;
  std::uint64_t rejected_commits_ = 0;
  std::uint64_t timeouts_ = 0;
  std::uint64_t recoveries_ = 0;
  std::uint64_t replacements_ = 0;
  std::uint64_t local_errors_ = 0;
  std::uint64_t activation_expirations_ = 0;
  std::uint64_t malformed_messages_ = 0;
  std::uint64_t suppressed_warnings_ = 0;
  std::uint64_t completed_barriers_ = 0;
  double barrier_latency_total_ms_ = 0.0;
  double barrier_latency_max_ms_ = 0.0;
  std::optional<TimePoint> barrier_started_;
  bool initialized_ = false;
};

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
