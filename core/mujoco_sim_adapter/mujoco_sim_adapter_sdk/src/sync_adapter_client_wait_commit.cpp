#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <cmath>
#include <string>

#include <ecal/ecal.h>

#include "internal/sync_adapter_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

bool ValidateWaitArguments(std::chrono::milliseconds timeout, TickLease* tick,
                           std::string* error) {
  if (tick == nullptr || error == nullptr) {
    if (error != nullptr) *error = "tick and error_message must not be null";
    return false;
  }
  error->clear();
  if (timeout.count() >= 0) return true;
  *error = "tick wait timeout must not be negative";
  return false;
}

mujoco_sim::v3::FaultCode WireFaultCode(FaultCode code) {
  switch (code) {
    case FaultCode::kControllerError:
      return mujoco_sim::v3::CONTROLLER_ERROR;
    case FaultCode::kInvalidOutput:
      return mujoco_sim::v3::INVALID_OUTPUT;
    case FaultCode::kInternalControllerError:
      return mujoco_sim::v3::INTERNAL_CONTROLLER_ERROR;
  }
  return mujoco_sim::v3::FAULT_CODE_UNSPECIFIED;
}

std::string FingerprintBytes(
    mujoco_sim::protocol::LayoutFingerprint fingerprint) {
  std::string bytes(16, '\0');
  const std::array words{fingerprint.low, fingerprint.high};
  for (std::size_t word = 0; word < words.size(); ++word) {
    for (std::size_t index = 0; index < 8; ++index) {
      bytes[word * 8 + index] = static_cast<char>(
          (words[word] >> (8U * index)) & 0xffU);
    }
  }
  return bytes;
}

}  // namespace

TickWaitResult SyncAdapterClient::Impl::WaitForTick(
    std::chrono::milliseconds timeout, TickLease* tick, std::string* error) {
  if (!ValidateWaitArguments(timeout, tick, error)) return TickWaitResult::kError;
  std::unique_lock lock(mutex_);
  if (const auto rejection = TickWaitRejectionLocked(error)) return *rejection;
  if (wait_in_progress_) {
    *error = "another thread is already waiting for a Tick";
    return TickWaitResult::kError;
  }
  if (outstanding_tick_) {
    *error = "the previously received Tick has not been committed";
    return TickWaitResult::kError;
  }
  wait_in_progress_ = true;
  const auto finish = [this] { wait_in_progress_ = false; };
  if (!WaitForQueuedTick(lock, timeout)) {
    finish();
    return TickWaitResult::kTimeout;
  }
  if (const auto rejection = TickWaitRejectionLocked(error)) {
    finish();
    return *rejection;
  }
  outstanding_tick_ = std::move(queued_tick_);
  TickLease received;
  received.data_ = outstanding_tick_;
  received.owner_ = this;
  *tick = std::move(received);
  finish();
  return TickWaitResult::kReceived;
}

std::optional<TickWaitResult> SyncAdapterClient::Impl::TickWaitRejectionLocked(
    std::string* error) const {
  if (!terminal_error_.empty()) {
    *error = terminal_error_;
    return TickWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok()))
    return TickWaitResult::kStopped;
  if (!session_.active()) {
    *error = "adapter client must be activated before waiting";
    return TickWaitResult::kError;
  }
  return std::nullopt;
}

bool SyncAdapterClient::Impl::WaitForQueuedTick(
    std::unique_lock<std::mutex>& lock, std::chrono::milliseconds timeout) {
  const auto ready = [this] {
    return queued_tick_ || !terminal_error_.empty() || stop_requested_.load() ||
           (monitor_ecal_ && !eCAL::Ok());
  };
  if (ready()) return true;
  if (timeout != std::chrono::milliseconds::max())
    return condition_.wait_for(lock, timeout, ready);
  while (!ready()) condition_.wait_for(lock, std::chrono::milliseconds(50));
  return true;
}

bool SyncAdapterClient::Impl::ValidateLeaseLocked(
    const TickLease& tick, std::shared_ptr<const TickLease::Data>* canonical,
    std::string* error) const {
  if (!session_.active() || stop_requested_) {
    *error = "adapter client is not active";
    return false;
  }
  if (tick.owner_ != this || !tick.data_) {
    *error = "Tick lease does not belong to this client";
    return false;
  }
  if (outstanding_tick_ == tick.data_) {
    *canonical = outstanding_tick_;
    return true;
  }
  if (committed_tick_ == tick.data_) {
    *canonical = committed_tick_;
    return true;
  }
  *error = "Tick is not the current outstanding or committed Tick";
  return false;
}

bool SyncAdapterClient::Impl::ValidateCommands(std::span<const double> commands,
                                                std::string* error) const {
  if (commands.size() != registration_.layout.command_scalar_count()) {
    *error = "command scalar count does not match registered layout";
    return false;
  }
  for (std::size_t index = 0; index < commands.size(); ++index) {
    if (!std::isfinite(commands[index])) {
      *error = "command value must be finite";
      return false;
    }
  }
  for (const auto& resource : registration_.layout.commands()) {
    if (!resource.has_command_range) continue;
    for (std::size_t offset = 0; offset < resource.scalars.count; ++offset) {
      const double value = commands[resource.scalars.offset + offset];
      if (value < resource.command_min || value > resource.command_max) {
        *error = "actuator command is outside ctrlrange: " + resource.name;
        return false;
      }
    }
  }
  return true;
}

bool SyncAdapterClient::Impl::Commit(const TickLease& tick,
                                      std::span<const double> commands,
                                      std::string* error) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  if (error == nullptr) return false;
  error->clear();
  std::shared_ptr<const TickLease::Data> canonical;
  {
    std::lock_guard lock(mutex_);
    if (!ValidateLeaseLocked(tick, &canonical, error) ||
        !ValidateCommands(commands, error)) {
      return false;
    }
  }
  mujoco_sim::protocol::DataFrameHeader header = canonical->header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kCommit;
  header.flags = 0;
  header.sequence = 0;
  auto mutable_encoded = std::make_shared<std::string>();
  mujoco_sim::protocol::DataFrameError encode_error;
  if (!mujoco_sim::protocol::EncodeDataFrame(
          header, commands, mutable_encoded.get(), &encode_error)) {
    *error = encode_error.message;
    return false;
  }
  std::shared_ptr<const std::string> encoded = std::move(mutable_encoded);
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_ || !session_.active()) {
      *error = "adapter client stopped before Commit";
      return false;
    }
    if (outstanding_tick_ == canonical) {
      committed_tick_ = canonical;
      cached_commit_ = encoded;
      outstanding_tick_.reset();
    } else if (committed_tick_ != canonical || !cached_commit_) {
      *error = "Tick is no longer available for Commit";
      return false;
    } else if (*encoded != *cached_commit_) {
      *error = "a different Commit was already sent for this Tick";
      return false;
    } else {
      encoded = cached_commit_;
    }
  }
  const bool published = transport_->PublishCommit(*encoded);
  if (published) {
    std::lock_guard lock(mutex_);
    last_commit_publish_ = std::chrono::steady_clock::now();
  } else {
    *error = "failed to publish Commit";
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  return published;
}

bool SyncAdapterClient::Impl::FailTick(const TickLease& tick, FaultCode code,
                                       std::string_view message,
                                       std::string* error) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  if (error == nullptr) return false;
  error->clear();
  std::shared_ptr<const TickLease::Data> canonical;
  {
    std::lock_guard lock(mutex_);
    if (!ValidateLeaseLocked(tick, &canonical, error)) return false;
  }
  mujoco_sim::v3::ControllerFault fault;
  fault.set_session_id(canonical->header.session_id);
  fault.set_epoch(canonical->header.epoch);
  fault.set_tick_id(canonical->header.tick_id);
  fault.set_layout_fingerprint(FingerprintBytes(canonical->header.layout));
  fault.set_code(WireFaultCode(code));
  fault.set_message(std::string(message));
  if (!transport_->PublishFault(fault)) {
    *error = "failed to publish ControllerFault";
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    if (outstanding_tick_ == canonical) outstanding_tick_.reset();
  }
  condition_.notify_all();
  return true;
}

}  // namespace mujoco_sim::adapter_sdk
