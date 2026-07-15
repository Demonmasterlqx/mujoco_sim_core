#pragma once

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/domain.hpp>


#include "model_runtime.hpp"

namespace mujoco_sim::core::internal {

class BarrierCoordinator {
 public:
  void Reset();
  bool Ready() const;
  std::vector<ValidatedActuatorWrite> BuildWritePlan() const;

  void AddRequiredController(const std::string& adapter_id);
  void AddRecipient(const std::string& adapter_id);
  void RemoveController(const std::string& adapter_id);
  void RemoveRequiredController(const std::string& adapter_id);
  bool IsRequired(const std::string& adapter_id) const;
  bool HasPendingCommit(const std::string& adapter_id) const;
  const mujoco_sim::core::AdapterCommit* FindPendingCommit(
      const std::string& adapter_id) const;
  void AcceptCommit(const std::string& adapter_id,
                    const mujoco_sim::core::AdapterCommit& commit,
                    const mujoco_sim::core::ResourceLayout& layout);
  void QueueZeroActuators(const std::vector<int>& actuator_ids);
  std::vector<std::string> RecipientIds() const;

 private:
  struct PendingCommit {
    mujoco_sim::core::AdapterCommit commit;
    std::vector<ValidatedActuatorWrite> writes;
  };
  std::unordered_map<std::string, PendingCommit> pending_commits_;
  std::set<std::string> required_controllers_;
  std::set<std::string> recipient_controllers_;
  std::set<int> pending_zero_actuators_;
};

}  // namespace mujoco_sim::core::internal
