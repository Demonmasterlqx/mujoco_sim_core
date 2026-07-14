#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <string>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "internal/adapter_status_monitor.hpp"
#include "internal/sync_adapter_client_impl.hpp"
#include "internal/tick_identity.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

using internal::SameTickIdentity;
using internal::SameTickPayload;
using internal::SameTickStream;

mujoco_sim::v2::AdapterHeartbeat BuildHeartbeat(
    const mujoco_sim::v2::Tick& tick) {
  mujoco_sim::v2::AdapterHeartbeat heartbeat;
  *heartbeat.mutable_header() = tick.header();
  heartbeat.set_adapter_id(tick.adapter_id());
  heartbeat.set_role(mujoco_sim::v2::CONTROLLER);
  return heartbeat;
}

}  // namespace

bool SyncAdapterClient::Impl::HasExpectedTickIdentity(
    const mujoco_sim::v2::Tick& tick) const {
  std::lock_guard lock(mutex_);
  if (!session_.has_session()) return false;
  const auto& header = tick.header();
  return tick.adapter_id() == options_.adapter_id &&
         (tick.controller_state() == mujoco_sim::v2::ACTIVE ||
          tick.controller_state() == mujoco_sim::v2::TIMED_OUT) &&
         header.server_name() == options_.server_name &&
         header.session_id() == session_.identity().session_id &&
         header.epoch() == session_.identity().epoch &&
         header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
         header.protocol_minor() == mujoco_sim::protocol::kProtocolMinor;
}

bool SyncAdapterClient::Impl::IsExpectedTick(
    const mujoco_sim::v2::Tick& tick) const {
  return HasExpectedTickIdentity(tick) &&
         tick.layout_hash().size() == mujoco_sim::protocol::kLayoutHashSize &&
         tick.layout_hash() == registration_.layout().layout_hash();
}

void SyncAdapterClient::Impl::HandleStatusBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v2::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size))) return;
  internal::SessionIdentity identity;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    identity = session_.identity();
  }
  const auto decision = internal::EvaluateAdapterStatus(
      status, options_.server_name, identity.epoch, options_.adapter_id,
      identity.session_id, mujoco_sim::v2::CONTROLLER, false);
  if (decision.action == internal::StatusAction::kStopped) {
    TransitionToStopped();
  } else if (decision.action == internal::StatusAction::kError) {
    SetTerminalError(decision.message);
  }
}

void SyncAdapterClient::Impl::HandleTickBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v2::Tick tick;
  if (!tick.ParseFromArray(data, static_cast<int>(size)) ||
      !HasExpectedTickIdentity(tick)) {
    return;
  }
  if (tick.layout_hash().size() != mujoco_sim::protocol::kLayoutHashSize ||
      tick.layout_hash() != registration_.layout().layout_hash()) {
    {
      std::lock_guard lock(mutex_);
      ++layout_hash_mismatches_;
    }
    SetTerminalError(
        "event=layout_hash_mismatch adapter_id=" + options_.adapter_id +
        " expected_bytes=" +
        std::to_string(mujoco_sim::protocol::kLayoutHashSize) +
        " actual_bytes=" + std::to_string(tick.layout_hash().size()));
    return;
  }
  HandleTick(tick);
}

void SyncAdapterClient::Impl::HandleTick(const mujoco_sim::v2::Tick& tick) {
  std::optional<mujoco_sim::v2::AdapterCommit> retry_commit;
  TickAction action;
  {
    std::lock_guard lock(mutex_);
    action = ClassifyTickLocked(tick, &retry_commit);
  }
  if (action == TickAction::kReject) return;
  transport_->PublishHeartbeat(BuildHeartbeat(tick));
  if (action == TickAction::kRetryCommit && retry_commit.has_value()) {
    (void)transport_->PublishCommit(*retry_commit);
  }
  if (action == TickAction::kQueue) condition_.notify_all();
}

SyncAdapterClient::Impl::TickAction
SyncAdapterClient::Impl::ClassifyTickLocked(
    const mujoco_sim::v2::Tick& tick,
    std::optional<mujoco_sim::v2::AdapterCommit>* retry_commit) {
  if (!session_.accepting_data() || stop_requested_) {
    return TickAction::kReject;
  }
  if (committed_tick_.has_value() &&
      SameTickIdentity(tick, *committed_tick_)) {
    if (!AcceptKnownTickLocked(tick, committed_tick_))
      return TickAction::kReject;
    *retry_commit = cached_commit_;
    return TickAction::kRetryCommit;
  }
  if (outstanding_tick_.has_value() &&
      SameTickIdentity(tick, *outstanding_tick_)) {
    return AcceptKnownTickLocked(tick, outstanding_tick_)
               ? TickAction::kHeartbeat
               : TickAction::kReject;
  }
  if (queued_tick_.has_value() && SameTickIdentity(tick, *queued_tick_)) {
    return AcceptKnownTickLocked(tick, queued_tick_) ? TickAction::kHeartbeat
                                                     : TickAction::kReject;
  }
  const auto newest = NewestKnownTickIdLocked(tick);
  if (newest.has_value() && tick.header().tick_id() < *newest)
    return TickAction::kReject;
  queued_tick_ = tick;
  return TickAction::kQueue;
}

bool SyncAdapterClient::Impl::AcceptKnownTickLocked(
    const mujoco_sim::v2::Tick& tick,
    const std::optional<mujoco_sim::v2::Tick>& known) {
  if (SameTickPayload(tick, *known)) return true;
  RecordTickConflictLocked(tick.header().tick_id());
  return false;
}

std::optional<std::uint64_t>
SyncAdapterClient::Impl::NewestKnownTickIdLocked(
    const mujoco_sim::v2::Tick& tick) const {
  std::optional<std::uint64_t> newest;
  const auto include = [&](const auto& known) {
    if (!known.has_value() || !SameTickStream(tick, *known)) return;
    newest = newest.has_value()
                 ? std::max(*newest, known->header().tick_id())
                 : known->header().tick_id();
  };
  include(committed_tick_);
  include(outstanding_tick_);
  include(queued_tick_);
  return newest;
}

void SyncAdapterClient::Impl::RecordTickConflictLocked(std::uint64_t tick_id) {
  const std::string warning =
      "conflicting payload for Tick " + std::to_string(tick_id);
  if (lifecycle_warning_.empty()) {
    lifecycle_warning_ = warning;
  } else if (lifecycle_warning_.find(warning) == std::string::npos) {
    lifecycle_warning_ += "; " + warning;
  }
}

}  // namespace mujoco_sim::adapter_sdk
