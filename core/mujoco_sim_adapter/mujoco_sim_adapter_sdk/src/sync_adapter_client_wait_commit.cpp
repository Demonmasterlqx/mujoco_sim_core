#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>

#include <ecal/ecal.h>

#include "internal/sync_adapter_client_impl.hpp"
#include "internal/tick_identity.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

using mujoco_sim::v2::COMMIT_OK;
using internal::SameTickIdentity;
using internal::SameTickPayload;

bool ValidateTickWaitArguments(std::chrono::milliseconds timeout,
                               mujoco_sim::v2::Tick* tick,
                               std::string* error_message) {
  if (tick == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "tick and error_message must not be null";
    }
    return false;
  }
  error_message->clear();
  if (timeout.count() >= 0) return true;
  *error_message = "tick wait timeout must not be negative";
  return false;
}

using CommandMap = std::unordered_map<std::string, double>;

bool BuildCommandMap(const std::vector<CommandValue>& commands,
                     CommandMap* values, std::string* error_message) {
  values->reserve(commands.size());
  for (const auto& command : commands) {
    if (command.name.empty() || !std::isfinite(command.value) ||
        !values->emplace(command.name, command.value).second) {
      if (command.name.empty()) {
        *error_message = "command name must not be empty";
      } else if (!std::isfinite(command.value)) {
        *error_message = "command value must be finite: " + command.name;
      } else {
        *error_message = "duplicate actuator command: " + command.name;
      }
      return false;
    }
  }
  return true;
}

bool AppendRegisteredCommands(
    const mujoco_sim::v2::ResourceLayout& layout, const CommandMap& values,
    mujoco_sim::v2::AdapterCommit* commit, std::string* error_message) {
  for (const auto& descriptor : layout.commands()) {
    const auto value = values.find(descriptor.name());
    if (descriptor.dimension() != 1 || value == values.end()) {
      *error_message = descriptor.dimension() != 1
                           ? "unsupported non-scalar actuator command: " +
                                 descriptor.name()
                           : "missing actuator command: " + descriptor.name();
      return false;
    }
    if (descriptor.has_ctrl_range() &&
        (value->second < descriptor.ctrl_min() ||
         value->second > descriptor.ctrl_max())) {
      *error_message = "actuator command is outside ctrlrange: " +
                       descriptor.name();
      return false;
    }
    auto* block = commit->add_commands();
    block->set_name(descriptor.name());
    block->add_values(value->second);
  }
  return true;
}

bool HasOnlyRegisteredCommands(const mujoco_sim::v2::ResourceLayout& layout,
                               const CommandMap& values,
                               std::string* error_message) {
  if (values.size() == static_cast<std::size_t>(layout.commands_size()))
    return true;
  for (const auto& [name, _] : values) {
    const auto known = std::find_if(
        layout.commands().begin(), layout.commands().end(),
        [&name](const auto& descriptor) { return descriptor.name() == name; });
    if (known == layout.commands().end()) {
      *error_message = "unknown actuator command: " + name;
      return false;
    }
  }
  *error_message = "command set does not match the registered layout";
  return false;
}

}  // namespace

TickWaitResult SyncAdapterClient::Impl::WaitForTick(
    std::chrono::milliseconds timeout, mujoco_sim::v2::Tick* tick,
    std::string* error_message) {
  if (!ValidateTickWaitArguments(timeout, tick, error_message))
    return TickWaitResult::kError;

  std::unique_lock lock(mutex_);
  if (const auto rejection = TickWaitRejectionLocked(error_message))
    return *rejection;
  if (wait_in_progress_) {
    *error_message = "another thread is already waiting for a Tick";
    return TickWaitResult::kError;
  }
  if (outstanding_tick_.has_value()) {
    *error_message = "the previously received Tick has not been committed";
    return TickWaitResult::kError;
  }
  wait_in_progress_ = true;
  const auto finish = [this] { wait_in_progress_ = false; };
  if (!WaitForQueuedTick(lock, timeout)) {
    finish();
    return TickWaitResult::kTimeout;
  }
  if (const auto rejection = TickWaitRejectionLocked(error_message)) {
    finish();
    return *rejection;
  }

  outstanding_tick_ = std::move(queued_tick_);
  queued_tick_.reset();
  *tick = *outstanding_tick_;
  finish();
  return TickWaitResult::kReceived;
}

std::optional<TickWaitResult>
SyncAdapterClient::Impl::TickWaitRejectionLocked(
    std::string* error_message) const {
  if (!terminal_error_.empty()) {
    *error_message = terminal_error_;
    return TickWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    return TickWaitResult::kStopped;
  }
  if (!session_.active()) {
    *error_message = "adapter client must be activated before waiting";
    return TickWaitResult::kError;
  }
  return std::nullopt;
}

bool SyncAdapterClient::Impl::WaitForQueuedTick(
    std::unique_lock<std::mutex>& lock, std::chrono::milliseconds timeout) {
  const auto ready = [this] {
    return queued_tick_.has_value() || !terminal_error_.empty() ||
           stop_requested_.load() || (monitor_ecal_ && !eCAL::Ok());
  };
  if (ready()) return true;
  if (timeout != std::chrono::milliseconds::max()) {
    return condition_.wait_for(lock, timeout, ready);
  }
  while (!ready()) condition_.wait_for(lock, std::chrono::milliseconds(50));
  return true;
}

bool SyncAdapterClient::Impl::BuildValidatedCommit(
    const mujoco_sim::v2::Tick& tick,
    const std::vector<CommandValue>& commands,
    mujoco_sim::v2::AdapterCommit* commit,
    std::string* error_message) const {
  if (commit == nullptr || error_message == nullptr) {
    return false;
  }
  CommandMap values;
  if (!BuildCommandMap(commands, &values, error_message)) return false;

  mujoco_sim::v2::AdapterCommit output;
  *output.mutable_header() = tick.header();
  output.set_adapter_id(tick.adapter_id());
  output.set_status(COMMIT_OK);
  if (!AppendRegisteredCommands(registration_.layout(), values, &output,
                                error_message) ||
      !HasOnlyRegisteredCommands(registration_.layout(), values,
                                 error_message))
    return false;
  *commit = std::move(output);
  return true;
}

bool SyncAdapterClient::Impl::Commit(
    const mujoco_sim::v2::Tick& tick,
    const std::vector<CommandValue>& commands,
    std::string* error_message) {
  std::lock_guard session_lock(lifecycle_mutex_);
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();

  mujoco_sim::v2::Tick canonical_tick;
  {
    std::lock_guard lock(mutex_);
    if (!SelectCanonicalTickLocked(tick, &canonical_tick, error_message))
      return false;
  }

  mujoco_sim::v2::AdapterCommit commit;
  if (!BuildValidatedCommit(canonical_tick, commands, &commit,
                            error_message)) {
    return false;
  }

  {
    std::lock_guard lock(mutex_);
    if (!FinalizeCommitLocked(canonical_tick, &commit, error_message))
      return false;
  }

  const bool published = transport_->PublishCommit(commit);
  condition_.notify_all();
  if (!published) {
    *error_message = "failed to publish Commit";
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::SelectCanonicalTickLocked(
    const mujoco_sim::v2::Tick& tick, mujoco_sim::v2::Tick* canonical_tick,
    std::string* error_message) const {
  if (!session_.active() || stop_requested_) {
    *error_message = "adapter client is not active";
    return false;
  }
  if (outstanding_tick_.has_value() &&
      SameTickIdentity(tick, *outstanding_tick_) &&
      SameTickPayload(tick, *outstanding_tick_)) {
    *canonical_tick = *outstanding_tick_;
    return true;
  }
  if (committed_tick_.has_value() &&
      SameTickIdentity(tick, *committed_tick_) &&
      SameTickPayload(tick, *committed_tick_)) {
    *canonical_tick = *committed_tick_;
    return true;
  }
  *error_message = "Tick is not the current outstanding or committed Tick";
  return false;
}

bool SyncAdapterClient::Impl::FinalizeCommitLocked(
    const mujoco_sim::v2::Tick& canonical_tick,
    mujoco_sim::v2::AdapterCommit* commit, std::string* error_message) {
  if (stop_requested_ || !session_.active()) {
    *error_message = "adapter client stopped before Commit";
    return false;
  }
  if (outstanding_tick_.has_value() &&
      SameTickIdentity(canonical_tick, *outstanding_tick_) &&
      SameTickPayload(canonical_tick, *outstanding_tick_)) {
    committed_tick_ = *outstanding_tick_;
    cached_commit_ = *commit;
    outstanding_tick_.reset();
    return true;
  }
  if (!committed_tick_.has_value() || !cached_commit_.has_value() ||
      !SameTickIdentity(canonical_tick, *committed_tick_) ||
      !SameTickPayload(canonical_tick, *committed_tick_)) {
    *error_message = "Tick is no longer available for Commit";
    return false;
  }
  if (commit->SerializeAsString() != cached_commit_->SerializeAsString()) {
    *error_message = "a different Commit was already sent for this Tick";
    return false;
  }
  *commit = *cached_commit_;
  return true;
}

}  // namespace mujoco_sim::adapter_sdk
