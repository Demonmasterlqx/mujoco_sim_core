#include "server_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace mujoco_sim::server {

bool IsProtocolPayloadSizeAllowed(std::size_t size) {
  return size <= kMaxProtocolPayloadBytes &&
         size <= static_cast<std::size_t>(std::numeric_limits<int>::max());
}

bool RunWithExceptionBoundary(const std::function<void()>& action,
                              std::string* error_message) noexcept {
  return RunWithExceptionBoundary<const std::function<void()>&>(action,
                                                                error_message);
}

SendOutcome ClassifySendOutcome(bool sent, std::size_t subscribers_before,
                                std::size_t subscribers_after) {
  if (sent) {
    return SendOutcome::kSent;
  }
  if (subscribers_before == 0 || subscribers_after == 0) {
    return SendOutcome::kNoSubscribers;
  }
  return SendOutcome::kRejectedWithSubscribers;
}

std::chrono::steady_clock::time_point ComputeLoopWakeDeadline(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point next_tick, bool tick_open,
    std::chrono::steady_clock::duration poll_interval) {
  const auto poll_deadline = now + poll_interval;
  return tick_open ? poll_deadline : std::min(poll_deadline, next_tick);
}

std::optional<TickRetrySchedule> MakeTickRetrySchedule(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point deadline,
    std::chrono::milliseconds initial_delay) {
  if (initial_delay.count() <= 0 || now + initial_delay > deadline) {
    return std::nullopt;
  }
  return TickRetrySchedule{0, now + initial_delay, deadline, initial_delay};
}

bool AdvanceTickRetry(TickRetrySchedule* schedule,
                      std::chrono::steady_clock::time_point now,
                      std::uint32_t max_attempts, double multiplier,
                      std::chrono::milliseconds max_delay) {
  if (schedule == nullptr || schedule->attempts >= max_attempts ||
      now < schedule->next_retry || now >= schedule->deadline) {
    return false;
  }
  ++schedule->attempts;
  const auto scaled = static_cast<std::uint64_t>(
      static_cast<double>(schedule->delay.count()) * multiplier);
  const auto next_delay = std::min<std::uint64_t>(
      std::max<std::uint64_t>(scaled, 1U),
      static_cast<std::uint64_t>(max_delay.count()));
  schedule->delay = std::chrono::milliseconds(next_delay);
  schedule->next_retry =
      std::min(now + schedule->delay, schedule->deadline);
  return true;
}

bool ValidateObserverCadenceRate(
    std::optional<std::uint32_t> every_n_steps,
    std::optional<double> period_s, double timestep_s, double max_rate_hz,
    bool allow_every_step, std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  if (every_n_steps.has_value() == period_s.has_value()) {
    *error_message = "Observer must select exactly one delivery cadence";
    return false;
  }
  double requested_rate_hz = 0.0;
  if (every_n_steps) {
    if (*every_n_steps == 0 || timestep_s <= 0.0) {
      *error_message = "Observer every_n_steps must be positive";
      return false;
    }
    if (*every_n_steps == 1 && !allow_every_step) {
      *error_message = "every-step Observer delivery is disabled";
      return false;
    }
    requested_rate_hz = 1.0 / (timestep_s * *every_n_steps);
  } else {
    if (!std::isfinite(*period_s) || *period_s <= 0.0) {
      *error_message = "Observer period_s must be finite and positive";
      return false;
    }
    requested_rate_hz = 1.0 / *period_s;
  }
  if (requested_rate_hz > max_rate_hz) {
    *error_message = "Observer delivery cadence exceeds observer_max_rate_hz";
    return false;
  }
  return true;
}

RuntimeRates CalculateRuntimeRates(
    double window_seconds, std::uint64_t start_tick, std::uint64_t end_tick,
    double start_simulation_time, double end_simulation_time,
    std::uint64_t completed_barriers, double barrier_latency_total_ms,
    double barrier_latency_max_ms) {
  RuntimeRates rates;
  if (window_seconds > 0.0) {
    if (end_tick >= start_tick) {
      rates.step_rate_hz =
          static_cast<double>(end_tick - start_tick) / window_seconds;
    }
    rates.real_time_factor =
        (end_simulation_time - start_simulation_time) / window_seconds;
  }
  if (completed_barriers > 0) {
    rates.barrier_latency_avg_ms =
        barrier_latency_total_ms / static_cast<double>(completed_barriers);
  }
  rates.barrier_latency_max_ms = barrier_latency_max_ms;
  return rates;
}

void RuntimeMetrics::Initialize(TimePoint now, std::uint64_t tick_id,
                                double simulation_time) {
  barrier_started_.reset();
  ResetWindow(now, tick_id, simulation_time);
  initialized_ = true;
}

bool RuntimeMetrics::SummaryDue(TimePoint now,
                                std::chrono::milliseconds period) const {
  return initialized_ && now - window_started_ >= period;
}

void RuntimeMetrics::RecordTickOpened(TimePoint now) {
  ++ticks_opened_;
  barrier_started_ = now;
}

void RuntimeMetrics::RecordTickPublishBatch(
    std::uint64_t attempts, std::uint64_t serialization_failures,
    std::uint64_t sent,
    std::uint64_t rejected_with_subscribers, std::uint64_t no_subscribers,
    std::uint64_t retransmissions, std::uint64_t bytes) {
  tick_publish_attempts_ += attempts;
  tick_serialization_failures_ += serialization_failures;
  tick_messages_sent_ += sent;
  tick_send_rejected_with_subscribers_ += rejected_with_subscribers;
  tick_no_subscribers_ += no_subscribers;
  tick_retransmissions_ += retransmissions;
  tick_bytes_ += bytes;
}

void RuntimeMetrics::RecordTickCache(
    std::uint64_t serializations, std::uint64_t hits, std::uint64_t misses,
    std::uint64_t cached_bytes, std::uint64_t cache_entries,
    std::uint64_t layout_bytes_avoided, double encode_us) {
  tick_serializations_ += serializations;
  tick_cache_hits_ += hits;
  tick_cache_misses_ += misses;
  tick_cached_bytes_ += cached_bytes;
  tick_cache_entries_ = cache_entries;
  layout_bytes_avoided_ += layout_bytes_avoided;
  tick_encode_total_us_ += encode_us;
  tick_encode_max_us_ = std::max(tick_encode_max_us_, encode_us);
}

void RuntimeMetrics::RecordBlockingWait(double elapsed_ms) {
  ++blocking_wait_count_;
  blocking_wait_total_ms_ += elapsed_ms;
  blocking_wait_max_ms_ = std::max(blocking_wait_max_ms_, elapsed_ms);
}

double RuntimeMetrics::RecordCommit(bool duplicate, bool rejected,
                                    bool stepped, TimePoint now,
                                    double validate_us) {
  ++commits_;
  duplicate_commits_ += duplicate ? 1U : 0U;
  rejected_commits_ += rejected ? 1U : 0U;
  commit_validate_total_us_ += validate_us;
  commit_validate_max_us_ = std::max(commit_validate_max_us_, validate_us);
  if (!stepped) {
    return 0.0;
  }

  return RecordBarrierCompleted(now);
}

double RuntimeMetrics::RecordBarrierCompleted(TimePoint now) {
  double latency_ms = 0.0;
  if (barrier_started_.has_value()) {
    latency_ms = std::chrono::duration<double, std::milli>(
                     now - *barrier_started_)
                     .count();
    ++completed_barriers_;
    barrier_latency_total_ms_ += latency_ms;
    barrier_latency_max_ms_ = std::max(barrier_latency_max_ms_, latency_ms);
  }
  barrier_started_.reset();
  return latency_ms;
}

RuntimeMetricsSnapshot RuntimeMetrics::TakeSummary(
    TimePoint now, std::uint64_t tick_id, double simulation_time) {
  RuntimeMetricsSnapshot snapshot;
  if (!initialized_) {
    return snapshot;
  }
  snapshot.window_seconds = std::max(
      std::chrono::duration<double>(now - window_started_).count(), 1e-9);
  snapshot.tick_id = tick_id;
  snapshot.simulation_time = simulation_time;
  snapshot.ticks_opened = ticks_opened_;
  snapshot.tick_publish_attempts = tick_publish_attempts_;
  snapshot.tick_serialization_failures = tick_serialization_failures_;
  snapshot.tick_messages_sent = tick_messages_sent_;
  snapshot.tick_send_rejected_with_subscribers =
      tick_send_rejected_with_subscribers_;
  snapshot.tick_no_subscribers = tick_no_subscribers_;
  snapshot.tick_retransmissions = tick_retransmissions_;
  snapshot.tick_bytes = tick_bytes_;
  snapshot.tick_encode_total_us = tick_encode_total_us_;
  snapshot.tick_encode_max_us = tick_encode_max_us_;
  snapshot.tick_serializations = tick_serializations_;
  snapshot.tick_cache_hits = tick_cache_hits_;
  snapshot.tick_cache_misses = tick_cache_misses_;
  snapshot.tick_cached_bytes = tick_cached_bytes_;
  snapshot.tick_cache_entries = tick_cache_entries_;
  snapshot.layout_bytes_avoided = layout_bytes_avoided_;
  snapshot.layout_hash_mismatches = layout_hash_mismatches_;
  snapshot.blocking_wait_count = blocking_wait_count_;
  snapshot.blocking_wait_total_ms = blocking_wait_total_ms_;
  snapshot.blocking_wait_max_ms = blocking_wait_max_ms_;
  snapshot.scheduler_wakeups = scheduler_wakeups_;
  snapshot.scheduler_late_wakeups = scheduler_late_wakeups_;
  snapshot.commits = commits_;
  snapshot.commit_validate_total_us = commit_validate_total_us_;
  snapshot.commit_validate_max_us = commit_validate_max_us_;
  snapshot.duplicate_commits = duplicate_commits_;
  snapshot.rejected_commits = rejected_commits_;
  snapshot.timeouts = timeouts_;
  snapshot.recoveries = recoveries_;
  snapshot.replacements = replacements_;
  snapshot.local_errors = local_errors_;
  snapshot.activation_expirations = activation_expirations_;
  snapshot.malformed_messages = malformed_messages_;
  snapshot.observation_frames = observation_frames_;
  snapshot.observation_bytes = observation_bytes_;
  snapshot.controller_heartbeats = controller_heartbeats_;
  snapshot.observer_heartbeats = observer_heartbeats_;
  snapshot.status_periodic = status_periodic_;
  snapshot.status_lifecycle = status_lifecycle_;
  snapshot.status_terminal = status_terminal_;
  snapshot.subscriber_count_queries = subscriber_count_queries_;
  snapshot.suppressed_warnings = suppressed_warnings_;
  snapshot.completed_barriers = completed_barriers_;
  snapshot.rates = CalculateRuntimeRates(
      snapshot.window_seconds, window_start_tick_, tick_id,
      window_start_simulation_time_, simulation_time, completed_barriers_,
      barrier_latency_total_ms_, barrier_latency_max_ms_);
  ResetWindow(now, tick_id, simulation_time);
  return snapshot;
}

void RuntimeMetrics::ResetWindow(TimePoint now, std::uint64_t tick_id,
                                 double simulation_time) {
  window_started_ = now;
  window_start_tick_ = tick_id;
  window_start_simulation_time_ = simulation_time;
  ticks_opened_ = 0;
  tick_publish_attempts_ = 0;
  tick_serialization_failures_ = 0;
  tick_messages_sent_ = 0;
  tick_send_rejected_with_subscribers_ = 0;
  tick_no_subscribers_ = 0;
  tick_retransmissions_ = 0;
  tick_bytes_ = 0;
  tick_encode_total_us_ = 0.0;
  tick_encode_max_us_ = 0.0;
  tick_serializations_ = 0;
  tick_cache_hits_ = 0;
  tick_cache_misses_ = 0;
  tick_cached_bytes_ = 0;
  tick_cache_entries_ = 0;
  layout_bytes_avoided_ = 0;
  layout_hash_mismatches_ = 0;
  blocking_wait_count_ = 0;
  blocking_wait_total_ms_ = 0.0;
  blocking_wait_max_ms_ = 0.0;
  scheduler_wakeups_ = 0;
  scheduler_late_wakeups_ = 0;
  commits_ = 0;
  commit_validate_total_us_ = 0.0;
  commit_validate_max_us_ = 0.0;
  duplicate_commits_ = 0;
  rejected_commits_ = 0;
  timeouts_ = 0;
  recoveries_ = 0;
  replacements_ = 0;
  local_errors_ = 0;
  activation_expirations_ = 0;
  malformed_messages_ = 0;
  observation_frames_ = 0;
  observation_bytes_ = 0;
  controller_heartbeats_ = 0;
  observer_heartbeats_ = 0;
  status_periodic_ = 0;
  status_lifecycle_ = 0;
  status_terminal_ = 0;
  subscriber_count_queries_ = 0;
  suppressed_warnings_ = 0;
  completed_barriers_ = 0;
  barrier_latency_total_ms_ = 0.0;
  barrier_latency_max_ms_ = 0.0;
}

bool CallbackState::Enter() {
  std::lock_guard lock(mutex_);
  if (!accepting_) {
    return false;
  }
  ++active_;
  return true;
}

void CallbackState::Leave() {
  std::lock_guard lock(mutex_);
  if (--active_ == 0) {
    condition_.notify_all();
  }
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
