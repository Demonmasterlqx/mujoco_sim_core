#include "controller_command_inbox.hpp"

#include <utility>

namespace mujoco_sim::server {

ControllerCommandInbox::RegistrationReservation::RegistrationReservation(
    ControllerCommandInbox* owner, std::unique_lock<std::mutex> lock, bool controller_slot)
    : owner_(owner), lock_(std::move(lock)), controller_slot_(controller_slot) {}

ControllerCommandInbox::RegistrationReservation::RegistrationReservation(
    RegistrationReservation&& other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)),
      lock_(std::move(other.lock_)),
      controller_slot_(std::exchange(other.controller_slot_, false)) {}

ControllerCommandInbox::RegistrationReservation&
ControllerCommandInbox::RegistrationReservation::operator=(
    RegistrationReservation&& other) noexcept {
  if (this != &other) {
    Reset();
    lock_ = std::move(other.lock_);
    owner_ = std::exchange(other.owner_, nullptr);
    controller_slot_ = std::exchange(other.controller_slot_, false);
  }
  return *this;
}

ControllerCommandInbox::RegistrationReservation::~RegistrationReservation() {
  Reset();
}

void ControllerCommandInbox::RegistrationReservation::Reset() noexcept {
  if (owner_ && controller_slot_) {
    owner_->slots_.erase(kReservedSessionId);
  }
  owner_ = nullptr;
  controller_slot_ = false;
}

bool ControllerCommandInbox::RegistrationReservation::Commit(
    std::uint64_t session_id, std::uint64_t epoch,
    std::span<const std::uint64_t> removed_sessions) {
  if (!owner_ || session_id == kReservedSessionId) {
    return false;
  }
  if (controller_slot_) {
    auto node = owner_->slots_.extract(kReservedSessionId);
    if (node.empty()) {
      return false;
    }
    node.key() = session_id;
    if (!owner_->slots_.insert(std::move(node)).inserted) {
      return false;
    }
    owner_->slots_.at(session_id).epoch = epoch;
  }
  for (const auto removed_session : removed_sessions) {
    owner_->slots_.erase(removed_session);
  }
  owner_ = nullptr;
  lock_.unlock();
  return true;
}

ControllerCommandInbox::RegistrationReservation ControllerCommandInbox::PrepareRegistration(
    bool controller_slot) {
  std::unique_lock lock(mutex_);
  if (closed_) {
    return {};
  }
  if (controller_slot && !slots_.try_emplace(kReservedSessionId).second) {
    return {};
  }
  return {this, std::move(lock), controller_slot};
}

bool ControllerCommandInbox::Register(std::uint64_t session_id, std::uint64_t epoch) {
  std::lock_guard lock(mutex_);
  if (session_id == kReservedSessionId || closed_) return false;
  auto [entry, inserted] = slots_.try_emplace(session_id);
  if (inserted) entry->second.epoch = epoch;
  return inserted;
}

void ControllerCommandInbox::Remove(std::uint64_t session_id) {
  std::lock_guard lock(mutex_);
  slots_.erase(session_id);
}

bool ControllerCommandInbox::Publish(mujoco_sim::core::ControllerCommand command) {
  std::lock_guard lock(mutex_);
  auto found = slots_.find(command.session.session_id);
  if (closed_ || found == slots_.end() || command.session.epoch != found->second.epoch) {
    return false;
  }
  auto& slot = found->second;
  if (slot.pending && command.sequence <= slot.command.sequence) return false;
  const bool wake = !slot.pending;
  slot.command = std::move(command);
  slot.pending = true;
  return wake;
}

bool ControllerCommandInbox::PublishFault(mujoco_sim::core::ControllerFault fault) {
  std::lock_guard lock(mutex_);
  auto found = slots_.find(fault.session.session_id);
  if (closed_ || found == slots_.end() || fault.session.epoch != found->second.epoch ||
      fault.code == mujoco_sim::core::FaultCode::kUnspecified) {
    return false;
  }
  const bool wake = !found->second.fault.has_value();
  found->second.fault = std::move(fault);
  return wake;
}

std::vector<mujoco_sim::core::ControllerCommand> ControllerCommandInbox::Drain() {
  std::lock_guard lock(mutex_);
  std::vector<mujoco_sim::core::ControllerCommand> output;
  output.reserve(slots_.size());
  for (auto& entry : slots_) {
    auto& slot = entry.second;
    if (!slot.pending) continue;
    output.push_back(std::move(slot.command));
    slot.pending = false;
  }
  return output;
}

std::vector<mujoco_sim::core::ControllerFault> ControllerCommandInbox::DrainFaults() {
  std::lock_guard lock(mutex_);
  std::vector<mujoco_sim::core::ControllerFault> output;
  output.reserve(slots_.size());
  for (auto& entry : slots_) {
    auto& slot = entry.second;
    if (!slot.fault) continue;
    output.push_back(std::move(*slot.fault));
    slot.fault.reset();
  }
  return output;
}

void ControllerCommandInbox::Close() {
  std::lock_guard lock(mutex_);
  closed_ = true;
  slots_.clear();
}

}  // namespace mujoco_sim::server
