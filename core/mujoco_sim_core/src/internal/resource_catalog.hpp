#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/v2/sync.pb.h>

struct mjModel_;

namespace mujoco_sim::core::internal {

class ResourceCatalog {
 public:
  static ResourceCatalog Build(const mjModel_& model);

  const std::vector<mujoco_sim::v2::ResourceDescriptor>& observations() const {
    return observations_;
  }
  const std::vector<mujoco_sim::v2::ResourceDescriptor>& commands() const {
    return commands_;
  }
  const mujoco_sim::v2::ResourceDescriptor* FindObservation(
      const std::string& name) const;
  const mujoco_sim::v2::ResourceDescriptor* FindCommand(
      const std::string& name) const;

 private:
  std::vector<mujoco_sim::v2::ResourceDescriptor> observations_;
  std::vector<mujoco_sim::v2::ResourceDescriptor> commands_;
  std::unordered_map<std::string, mujoco_sim::v2::ResourceDescriptor>
      observations_by_name_;
  std::unordered_map<std::string, mujoco_sim::v2::ResourceDescriptor>
      commands_by_name_;
};

}  // namespace mujoco_sim::core::internal
