#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/domain.hpp>


struct mjModel_;

namespace mujoco_sim::core::internal {

class ResourceCatalog {
 public:
  static ResourceCatalog Build(const mjModel_& model);

  const std::vector<mujoco_sim::core::ResourceDescriptor>& observations() const {
    return observations_;
  }
  const std::vector<mujoco_sim::core::ResourceDescriptor>& commands() const {
    return commands_;
  }
  const mujoco_sim::core::ResourceDescriptor* FindObservation(
      const std::string& name) const;
  const mujoco_sim::core::ResourceDescriptor* FindCommand(
      const std::string& name) const;

 private:
  std::vector<mujoco_sim::core::ResourceDescriptor> observations_;
  std::vector<mujoco_sim::core::ResourceDescriptor> commands_;
  std::unordered_map<std::string, mujoco_sim::core::ResourceDescriptor>
      observations_by_name_;
  std::unordered_map<std::string, mujoco_sim::core::ResourceDescriptor>
      commands_by_name_;
};

}  // namespace mujoco_sim::core::internal
