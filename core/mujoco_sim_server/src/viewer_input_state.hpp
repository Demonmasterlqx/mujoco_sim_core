#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include <mujoco_sim/core/interactive_input.hpp>

#include "sim_viewer.hpp"

namespace mujoco_sim::server {

/**
 * @brief Owns simulation-thread GUI input between viewer consumption and Step.
 *
 * Control writes are merged per actuator until one Step completes. Wrench
 * frames are complete state replacements and remain active until replaced,
 * explicitly cleared, or expired using their viewer generation timestamp.
 *
 * 在 viewer 消费与 Step 之间持有仅属于仿真线程的 GUI 输入。
 *
 * Control 写入按 actuator 合并并保留到一个 Step 完成。扰动力帧是完整状态替换，并持续
 * 有效，直到被替换、显式清除或依据 viewer 生成时间过期。
 *
 * @par Thread safety / 线程安全
 * All methods must be called by the SimulationLoop thread only.
 * 所有方法都只能由 SimulationLoop 线程调用。
 */
class ViewerInputState {
 public:
  /** One Step preparation and optional one-shot expiry diagnostic.
   * 一次 Step 准备结果以及可选的一次性过期诊断。 */
  struct PreparedStep {
    /** Borrowed input valid until the next state mutation. 下次状态修改前有效的借用输入。 */
    mujoco_sim::core::InteractiveStepInput input;
    /** Whether this preparation expired a nonempty wrench state.
     * 本次准备是否使一个非空扰动力状态过期。 */
    bool wrench_expired = false;
    /** Number of bodies cleared by expiry. 因过期清除的刚体数。 */
    std::size_t expired_body_count = 0;
    /** Age at expiry in milliseconds. 过期时的状态年龄（毫秒）。 */
    std::chrono::milliseconds wrench_age{};
  };

  /** Creates empty state with a positive steady-clock timeout.
   * 使用正的 steady clock 超时创建空状态。 */
  explicit ViewerInputState(std::chrono::milliseconds wrench_timeout);

  /** Merges one consumed Viewer batch using per-input latest rules.
   * 按各输入的 latest 规则合并一个已消费 Viewer 批次。 */
  void Merge(const SimViewer::ViewerInputBatch& input);

  /** Expires stale force state and borrows values for the next Step.
   * 使陈旧力状态过期，并为下一 Step 借用输入值。 */
  PreparedStep PrepareStep(std::chrono::steady_clock::time_point now);

  /** Clears one-shot values after a successful completed Step.
   * 在成功完成一个 Step 后清除一次性值。 */
  void CommitCompletedStep() noexcept;

 private:
  std::chrono::milliseconds wrench_timeout_;
  std::vector<mujoco_sim::core::InteractiveActuatorWrite> pending_control_writes_;
  std::vector<mujoco_sim::core::InteractiveBodyWrench> active_body_wrenches_;
  std::chrono::steady_clock::time_point last_wrench_update_{};
  bool wrench_state_received_ = false;
  bool wrench_dirty_ = false;
  bool wrench_timed_out_ = false;
};

}  // namespace mujoco_sim::server
