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
    /**
     * @brief Model-owned configured initial keyframe ID.
     *
     * A nonnegative value is valid for `model`; -1 selects `mj_resetData()`.
     * Commit transfers this value together with the prepared model and data.
     *
     * model 所属的已配置初始 keyframe ID。
     *
     * 非负值对 `model` 有效；-1 表示选择 `mj_resetData()`。Commit 会将该值与已准备的
     * model 和 data 一并转移。
     */
    int initial_keyframe_id = -1;
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
  /**
   * @brief Resets authoritative data to the configured initial state.
   *
   * Uses `mj_resetDataKeyframe()` when a keyframe was committed and
   * `mj_resetData()` otherwise, then zeros all controls, calls `mj_forward()`,
   * refreshes cached simulation time, and forgets interactive wrench bodies.
   * Model generation and snapshot-model ownership are unchanged.
   *
   * 将权威 data 重置到配置的初始状态。
   *
   * 已提交 keyframe 时使用 `mj_resetDataKeyframe()`，否则使用 `mj_resetData()`；随后
   * 清零全部 control、调用 `mj_forward()`、刷新缓存的仿真时间并忘记交互扰动力刚体。
   * model generation 与 snapshot model 所有权保持不变。
   *
   * @pre A model is loaded and the SimulationCore owner thread is the caller.
   *      已加载 model，且调用者是 SimulationCore 所有者线程。
   */
  void ResetToInitialState();
  void Step();

  const mjModel_* model_ptr() const noexcept { return model_.get(); }
  const mjData_* data_ptr() const noexcept { return data_.get(); }
  double simulation_time() const noexcept { return simulation_time_; }
  double timestep() const noexcept;

 private:
  /**
   * @brief Adopts prepared MuJoCo data and its matching reset keyframe.
   *
   * The caller retains ownership of the prepared model until it has been
   * transferred separately. This helper performs no physics step and is used
   * only while committing a successfully prepared model.
   *
   * 接管已准备的 MuJoCo data 及其匹配的 Reset keyframe。
   *
   * 调用者在另行转移已准备 model 前仍持有该 model。本 helper 不执行物理步进，仅在提交
   * 已成功准备的 model 时使用。
   *
   * @param prepared Non-null, exclusively owned prepared state.
   *                 非空且由调用者独占的已准备状态。
   */
  void AdoptPreparedData(PreparedModel* prepared);

  std::unique_ptr<mjModel_, ModelDeleter> model_;
  std::unique_ptr<mjData_, DataDeleter> data_;
  double simulation_time_ = 0.0;
  std::uint64_t generation_ = 0;
  int initial_keyframe_id_ = -1;
  /** Dynamic bodies occupied by the preceding interactive wrench frame.
   * 上一交互扰动力帧占用的动态刚体。 */
  std::vector<int> interactive_wrench_body_ids_;
  /** Model generation owning the recorded body IDs; stale IDs are never used.
   * 持有已记录刚体 ID 的模型代次；陈旧 ID 永远不会被使用。 */
  std::uint64_t interactive_wrench_generation_ = 0;
};

}  // namespace internal
}  // namespace mujoco_sim::core
