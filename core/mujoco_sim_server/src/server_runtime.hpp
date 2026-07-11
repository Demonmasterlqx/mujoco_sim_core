#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace mujoco_sim::server {

inline constexpr std::size_t kMaxProtocolPayloadBytes =
    4U * 1024U * 1024U;

bool IsProtocolPayloadSizeAllowed(std::size_t size);

enum class SendOutcome {
  kSent,
  kNoSubscribers,
  kRejectedWithSubscribers,
};

SendOutcome ClassifySendOutcome(bool sent, std::size_t subscribers_before,
                                std::size_t subscribers_after);

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
  std::uint64_t commits = 0;
  std::uint64_t duplicate_commits = 0;
  std::uint64_t rejected_commits = 0;
  std::uint64_t timeouts = 0;
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
  double RecordCommit(bool duplicate, bool rejected, bool stepped,
                      TimePoint now);
  void RecordTimeout() { ++timeouts_; }
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
  std::uint64_t commits_ = 0;
  std::uint64_t duplicate_commits_ = 0;
  std::uint64_t rejected_commits_ = 0;
  std::uint64_t timeouts_ = 0;
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
