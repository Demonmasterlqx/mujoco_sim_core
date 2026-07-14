#include "barrier_coordinator.hpp"

#include <algorithm>

#include "resource_catalog.hpp"

namespace mujoco_sim::core::internal {

void BarrierCoordinator::Reset() {
  pending_commits_.clear();
  required_controllers_.clear();
  recipient_controllers_.clear();
  pending_zero_actuators_.clear();
}

bool BarrierCoordinator::Ready() const {
  return pending_commits_.size() == required_controllers_.size() &&
         std::all_of(required_controllers_.begin(), required_controllers_.end(),
                     [this](const std::string& id) {
                       return pending_commits_.contains(id);
                     });
}

std::vector<ValidatedActuatorWrite> BarrierCoordinator::BuildWritePlan(
    const ResourceCatalog& catalog) const {
  std::vector<ValidatedActuatorWrite> writes;
  for (const auto& [_, pending] : pending_commits_) {
    for (const auto& command : pending.commands()) {
      const auto* descriptor = catalog.FindCommand(command.name());
      if (descriptor != nullptr) {
        writes.push_back({descriptor->mujoco_id(), command.values(0)});
      }
    }
  }
  for (const int actuator_id : pending_zero_actuators_) {
    writes.push_back({actuator_id, 0.0});
  }
  return writes;
}

void BarrierCoordinator::AddRequiredController(const std::string& adapter_id) {
  required_controllers_.insert(adapter_id);
  recipient_controllers_.insert(adapter_id);
}

void BarrierCoordinator::AddRecipient(const std::string& adapter_id) {
  recipient_controllers_.insert(adapter_id);
}

void BarrierCoordinator::RemoveController(const std::string& adapter_id) {
  pending_commits_.erase(adapter_id);
  required_controllers_.erase(adapter_id);
  recipient_controllers_.erase(adapter_id);
}

void BarrierCoordinator::RemoveRequiredController(
    const std::string& adapter_id) {
  pending_commits_.erase(adapter_id);
  required_controllers_.erase(adapter_id);
}

bool BarrierCoordinator::IsRequired(const std::string& adapter_id) const {
  return required_controllers_.contains(adapter_id);
}

bool BarrierCoordinator::HasPendingCommit(const std::string& adapter_id) const {
  return pending_commits_.contains(adapter_id);
}

const mujoco_sim::v2::AdapterCommit* BarrierCoordinator::FindPendingCommit(
    const std::string& adapter_id) const {
  const auto found = pending_commits_.find(adapter_id);
  return found == pending_commits_.end() ? nullptr : &found->second;
}

void BarrierCoordinator::AcceptCommit(
    const std::string& adapter_id,
    const mujoco_sim::v2::AdapterCommit& commit) {
  pending_commits_.emplace(adapter_id, commit);
}

void BarrierCoordinator::QueueZeroActuators(
    const std::vector<int>& actuator_ids) {
  pending_zero_actuators_.insert(actuator_ids.begin(), actuator_ids.end());
}

std::vector<std::string> BarrierCoordinator::RecipientIds() const {
  return {recipient_controllers_.begin(), recipient_controllers_.end()};
}

}  // namespace mujoco_sim::core::internal
