#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <algorithm>
#include <bit>
#include <limits>
#include <string>

#include "internal/adapter_status_monitor.hpp"
#include "internal/sync_adapter_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

bool SameIdentity(const TickLease::Data& left, const TickLease::Data& right) {
  return left.header.session_id == right.header.session_id &&
         left.header.epoch == right.header.epoch &&
         left.header.tick_id == right.header.tick_id &&
         left.header.layout == right.header.layout;
}

bool SamePayload(const TickLease::Data& left, const TickLease::Data& right) {
  if (!SameIdentity(left, right) || left.header.flags != right.header.flags ||
      left.header.simulation_time_s != right.header.simulation_time_s ||
      left.scalars.size() != right.scalars.size()) {
    return false;
  }
  for (std::size_t index = 0; index < left.scalars.size(); ++index) {
    if (std::bit_cast<std::uint64_t>(left.scalars[index]) !=
        std::bit_cast<std::uint64_t>(right.scalars[index])) {
      return false;
    }
  }
  return true;
}

}  // namespace

void SyncAdapterClient::Impl::HandleStatusBytes(const void* data,
                                                 std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v3::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size))) return;
  internal::SessionIdentity identity;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    identity = session_.identity();
  }
  const auto decision = internal::EvaluateAdapterStatus(
      status, options_.server_name, identity.epoch, options_.adapter_id,
      identity.session_id, mujoco_sim::v3::CONTROLLER, false);
  if (decision.action == internal::StatusAction::kStopped) TransitionToStopped();
  else if (decision.action == internal::StatusAction::kError)
    SetTerminalError(decision.message);
}

void SyncAdapterClient::Impl::HandleTickBytes(const void* data,
                                               std::size_t size) {
  if (data == nullptr || size == 0) return;
  const std::string_view bytes(static_cast<const char*>(data), size);
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError parse_error;
  if (!mujoco_sim::protocol::ParseDataFrame(bytes, &parsed, &parse_error) ||
      parsed.header.kind != mujoco_sim::protocol::DataFrameKind::kTick) {
    return;
  }
  internal::SessionIdentity session;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    session = session_.identity();
  }
  if (parsed.header.session_id != session.session_id ||
      parsed.header.epoch != session.epoch) {
    return;
  }
  if (parsed.header.layout != layout_fingerprint_) {
    {
      std::lock_guard lock(mutex_);
      ++layout_hash_mismatches_;
    }
    SetTerminalError("event=layout_fingerprint_mismatch adapter_id=" +
                     options_.adapter_id);
    return;
  }
  if (parsed.scalar_count !=
      registration_.layout.observation_scalar_count()) {
    SetTerminalError("Tick scalar count does not match registered layout");
    return;
  }
  auto tick = std::make_shared<TickLease::Data>();
  tick->header = parsed.header;
  tick->scalars.resize(parsed.scalar_count);
  if (!mujoco_sim::protocol::DecodeDataFrameScalars(
          parsed, tick->scalars, &parse_error)) {
    return;
  }
  HandleTick(std::move(tick));
}

void SyncAdapterClient::Impl::HandleTick(
    std::shared_ptr<const TickLease::Data> tick) {
  std::shared_ptr<const std::string> retry_commit;
  TickAction action;
  {
    std::lock_guard lock(mutex_);
    action = ClassifyTickLocked(tick, &retry_commit);
  }
  if (action == TickAction::kRetryCommit && retry_commit) {
    (void)transport_->PublishCommit(*retry_commit);
  } else if (action == TickAction::kQueue) {
    condition_.notify_all();
  }
}

SyncAdapterClient::Impl::TickAction SyncAdapterClient::Impl::ClassifyTickLocked(
    const std::shared_ptr<const TickLease::Data>& tick,
    std::shared_ptr<const std::string>* retry_commit) {
  if (!session_.accepting_data() || stop_requested_) return TickAction::kReject;
  if (committed_tick_ && SameIdentity(*tick, *committed_tick_)) {
    if (!AcceptKnownTickLocked(tick, committed_tick_)) return TickAction::kReject;
    *retry_commit = cached_commit_;
    return TickAction::kRetryCommit;
  }
  if (outstanding_tick_ && SameIdentity(*tick, *outstanding_tick_)) {
    (void)AcceptKnownTickLocked(tick, outstanding_tick_);
    return TickAction::kReject;
  }
  if (queued_tick_ && SameIdentity(*tick, *queued_tick_)) {
    (void)AcceptKnownTickLocked(tick, queued_tick_);
    return TickAction::kReject;
  }
  const auto newest = NewestKnownTickLocked(*tick);
  if (newest && tick->header.tick_id < *newest) return TickAction::kReject;
  queued_tick_ = tick;
  return TickAction::kQueue;
}

std::optional<std::uint64_t>
SyncAdapterClient::Impl::NewestKnownTickLocked(
    const TickLease::Data& tick) const {
  std::optional<std::uint64_t> newest;
  for (const auto& known : {committed_tick_, outstanding_tick_, queued_tick_}) {
    if (!known || known->header.session_id != tick.header.session_id ||
        known->header.epoch != tick.header.epoch) {
      continue;
    }
    newest = newest ? std::max(*newest, known->header.tick_id)
                    : known->header.tick_id;
  }
  return newest;
}

bool SyncAdapterClient::Impl::AcceptKnownTickLocked(
    const std::shared_ptr<const TickLease::Data>& tick,
    const std::shared_ptr<const TickLease::Data>& known) {
  if (SamePayload(*tick, *known)) return true;
  RecordTickConflictLocked(tick->header.tick_id);
  session_.Fail();
  terminal_error_ = "conflicting payload for Tick " +
                    std::to_string(tick->header.tick_id);
  stop_requested_ = true;
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  return false;
}

void SyncAdapterClient::Impl::RecordTickConflictLocked(std::uint64_t tick_id) {
  const std::string warning =
      "conflicting payload for Tick " + std::to_string(tick_id);
  if (lifecycle_warning_.empty()) lifecycle_warning_ = warning;
  else if (lifecycle_warning_.find(warning) == std::string::npos)
    lifecycle_warning_ += "; " + warning;
}

void SyncAdapterClient::Impl::StartHeartbeatThread() {
  std::lock_guard lock(heartbeat_thread_mutex_);
  if (!stop_requested_ && !heartbeat_thread_.joinable()) {
    heartbeat_thread_ = std::thread([this] { HeartbeatLoop(); });
  }
}

void SyncAdapterClient::Impl::JoinHeartbeatThread() {
  std::lock_guard lock(heartbeat_thread_mutex_);
  if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
}

void SyncAdapterClient::Impl::HeartbeatLoop() {
  const auto period = std::chrono::milliseconds(options_.heartbeat_period_ms);
  std::unique_lock lock(mutex_);
  while (!stop_requested_) {
    heartbeat_condition_.wait_for(lock, period, [this] {
      return stop_requested_.load() || !session_.has_session();
    });
    if (stop_requested_) break;
    const auto now = std::chrono::steady_clock::now();
    if (!session_.active() || now - last_commit_publish_ < period ||
        now - last_heartbeat_publish_ < period) {
      continue;
    }
    const auto heartbeat = BuildHeartbeat();
    last_heartbeat_publish_ = now;
    lock.unlock();
    transport_->PublishHeartbeat(heartbeat);
    lock.lock();
  }
}

mujoco_sim::v3::SessionHeartbeat SyncAdapterClient::Impl::BuildHeartbeat() const {
  mujoco_sim::v3::SessionHeartbeat heartbeat;
  heartbeat.set_session_id(session_.identity().session_id);
  heartbeat.set_epoch(session_.identity().epoch);
  std::uint64_t last_seen = 0;
  if (outstanding_tick_) last_seen = outstanding_tick_->header.tick_id;
  if (committed_tick_) last_seen = std::max(last_seen, committed_tick_->header.tick_id);
  if (queued_tick_) last_seen = std::max(last_seen, queued_tick_->header.tick_id);
  heartbeat.set_last_seen(last_seen);
  return heartbeat;
}

}  // namespace mujoco_sim::adapter_sdk
