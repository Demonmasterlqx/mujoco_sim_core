#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <mujoco_sim/core/simulation_core.hpp>

namespace mujoco_sim::core::internal {
class ModelRuntime;
}

namespace mujoco_sim::core::detail {

class SnapshotManager {
 public:
  void Reset();
  std::unique_ptr<SimulationSnapshot> Create(
      const internal::ModelRuntime& model_runtime,
      std::string* error_message) const;
  std::unique_ptr<SimulationSnapshot> Clone(
      const internal::ModelRuntime& model_runtime,
      const SimulationSnapshot& source, std::string* error_message) const;
  bool Update(const internal::ModelRuntime& model_runtime,
              SimulationSnapshot* snapshot, std::string* error_message) const;

 private:
  std::shared_ptr<void> identity_ = std::make_shared<char>();
  std::uint64_t model_generation_ = 0;
};

}  // namespace mujoco_sim::core::detail
