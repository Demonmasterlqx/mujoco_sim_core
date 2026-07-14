#pragma once

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/v2/sync.pb.h>

#include "model_runtime.hpp"

namespace mujoco_sim::core::internal {

class ResourceCatalog;

class BarrierCoordinator {
 public:
  void Reset();
  bool Ready() const;
  std::vector<ValidatedActuatorWrite> BuildWritePlan(
      const ResourceCatalog& catalog) const;

  void AddRequiredController(const std::string& adapter_id);
  void AddRecipient(const std::string& adapter_id);
  void RemoveController(const std::string& adapter_id);
  void RemoveRequiredController(const std::string& adapter_id);
  bool IsRequired(const std::string& adapter_id) const;
  bool HasPendingCommit(const std::string& adapter_id) const;
  const mujoco_sim::v2::AdapterCommit* FindPendingCommit(
      const std::string& adapter_id) const;
  void AcceptCommit(const std::string& adapter_id,
                    const mujoco_sim::v2::AdapterCommit& commit);
  void QueueZeroActuators(const std::vector<int>& actuator_ids);
  std::vector<std::string> RecipientIds() const;

 private:
  std::unordered_map<std::string, mujoco_sim::v2::AdapterCommit>
      pending_commits_;
  std::set<std::string> required_controllers_;
  std::set<std::string> recipient_controllers_;
  std::set<int> pending_zero_actuators_;
};

}  // namespace mujoco_sim::core::internal
