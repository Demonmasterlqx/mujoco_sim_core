#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
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
 * Pause/Run retains only the latest desired state and Reset coalesces as an
 * edge until consumed independently from Step input.
 *
 * 在 viewer 消费与 Step 之间持有仅属于仿真线程的 GUI 输入。
 *
 * Control 写入按 actuator 合并并保留到一个 Step 完成。扰动力帧是完整状态替换，并持续
 * 有效，直到被替换、显式清除或依据 viewer 生成时间过期。Pause/Run 仅保留最新期望
 * 状态，Reset 则将边沿合并并保持到独立于 Step 输入的消费时刻。
 *
 * @par Thread safety / 线程安全
 * All methods must be called by the SimulationLoop thread only.
 * 所有方法都只能由 SimulationLoop 线程调用。
 */
class ViewerInputState {
 public:
  /**
   * @brief Owns one-shot local simulation actions ready for SimulationLoop.
   *
   * Values are detached from data input so Reset can run before Pause/Run and
   * both can be consumed before deciding whether a Step is allowed.
   *
   * 持有供 SimulationLoop 使用的一次性本地仿真动作。
   *
   * 这些值与数据输入分离，因此 Reset 可先于 Pause/Run 执行，并且二者可在决定是否允许
   * Step 前完成消费。
   */
  struct PendingSimulationActions {
    /** Latest desired running state. 最新期望运行状态。 */
    std::optional<bool> running;
    /** Whether one or more Reset edges were received. 是否收到一个或多个 Reset 边沿。 */
    bool reset_requested = false;
  };

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

  /**
   * @brief Takes and clears pending lifecycle actions without touching data input.
   * @return Owned latest Pause/Run selection and coalesced Reset edge.
   *         自有的最新 Pause/Run 选择与已合并 Reset 边沿。
   *
   * 取出并清除待处理生命周期动作，不修改数据输入。
   */
  PendingSimulationActions TakeSimulationActions() noexcept;

  /**
   * @brief Clears all persistent GUI data input after authoritative Reset.
   *
   * Removes Control, active Wrench, generation time, dirty, and timeout state.
   * Lifecycle actions already returned by `TakeSimulationActions()` are not
   * modified.
   *
   * 在权威 Reset 后清除全部持久 GUI 数据输入。
   *
   * 清除 Control、活动 Wrench、生成时间、dirty 与 timeout 状态。不修改已经由
   * `TakeSimulationActions()` 返回的生命周期动作。
   */
  void ResetAfterSimulationReset() noexcept;

 private:
  std::chrono::milliseconds wrench_timeout_;
  std::vector<mujoco_sim::core::InteractiveActuatorWrite> pending_control_writes_;
  std::vector<mujoco_sim::core::InteractiveBodyWrench> active_body_wrenches_;
  std::chrono::steady_clock::time_point last_wrench_update_{};
  bool wrench_state_received_ = false;
  bool wrench_dirty_ = false;
  bool wrench_timed_out_ = false;
  std::optional<bool> pending_running_;
  bool reset_pending_ = false;
};

}  // namespace mujoco_sim::server
