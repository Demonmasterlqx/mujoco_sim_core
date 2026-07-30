#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <mujoco_sim/core/interactive_input.hpp>

struct mjData_;
struct mjModel_;

namespace mujoco_sim::core {

struct ModelLoadOptions;

namespace internal {

struct ModelDeleter {
  void operator()(mjModel_* model) const;
};

struct DataDeleter {
  void operator()(mjData_* data) const;
};

struct ValidatedActuatorWrite {
  int actuator_id = -1;
  double value = 0.0;
};

class ModelRuntime {
 public:
  struct PreparedModel {
    std::unique_ptr<mjModel_, ModelDeleter> model;
    std::unique_ptr<mjData_, DataDeleter> data;
  };

  static std::optional<PreparedModel> Prepare(const std::string& model_path,
                                              const ModelLoadOptions& options,
                                              std::string* error_message);

  void Commit(PreparedModel prepared);
  bool loaded() const noexcept;
  void ApplyActuatorWrites(const std::vector<ValidatedActuatorWrite>& writes);
  /** Applies finite, valid-ID interactive controls without clamping.
   * 应用 ID 有效且数值有限的交互控制，不执行钳制。 */
  void ApplyInteractiveActuatorWrites(std::span<const InteractiveActuatorWrite> writes);
  /** Replaces the complete dynamic-body interactive wrench state.
   * 替换动态刚体的完整交互扰动力状态。 */
  void ReplaceInteractiveBodyWrenches(std::span<const InteractiveBodyWrench> wrenches);
  void ZeroActuators(const std::vector<int>& actuator_ids);
  void ZeroAllControls();
  void Step();

  const mjModel_* model_ptr() const noexcept { return model_.get(); }
  const mjData_* data_ptr() const noexcept { return data_.get(); }
  double simulation_time() const noexcept { return simulation_time_; }
  double timestep() const noexcept;

 private:
  std::unique_ptr<mjModel_, ModelDeleter> model_;
  std::unique_ptr<mjData_, DataDeleter> data_;
  double simulation_time_ = 0.0;
  std::uint64_t generation_ = 0;
  /** Dynamic bodies occupied by the preceding interactive wrench frame.
   * 上一交互扰动力帧占用的动态刚体。 */
  std::vector<int> interactive_wrench_body_ids_;
  /** Model generation owning the recorded body IDs; stale IDs are never used.
   * 持有已记录刚体 ID 的模型代次；陈旧 ID 永远不会被使用。 */
  std::uint64_t interactive_wrench_generation_ = 0;
};

}  // namespace internal
}  // namespace mujoco_sim::core
