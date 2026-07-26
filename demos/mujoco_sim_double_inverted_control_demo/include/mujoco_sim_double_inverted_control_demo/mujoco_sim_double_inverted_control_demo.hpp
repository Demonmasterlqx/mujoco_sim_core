#pragma once

#include <array>
#include <memory>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>

#include "mujoco_sim_double_inverted_control_config.hpp"

namespace mujoco_sim_double_inverted_control_demo {

/**
 * @brief Runs the two-joint controller through one Adapter SDK Controller session.
 *        通过一个 Adapter SDK 控制器会话运行双关节控制器。
 *
 * The object owns its configuration and Controller client. Public lifecycle
 * methods are intended to be called sequentially from one control thread.
 * 本对象持有配置和控制器客户端。公开生命周期方法应由同一控制线程顺序调用。
 */
class DoubleInvertedController {
 public:
  /**
   * @brief Creates a controller from an owned configuration.
   *        使用自有配置创建控制器。
   * @param server_config Non-null configuration transferred to this object.
   *                      转移给本对象的非空配置。
   * @throws std::runtime_error If the configuration is invalid.
   *                            配置无效时抛出 std::runtime_error。
   * @par Thread safety / 线程安全
   * Construction is not thread-safe with concurrent access to the supplied configuration.
   * 构造期间不得并发访问传入的配置。
   */
  explicit DoubleInvertedController(
      std::unique_ptr<ServerConfig> server_config = std::make_unique<ServerConfig>());

  /**
   * @brief Registers and activates the configured Controller session.
   *        注册并激活已配置的控制器会话。
   * @return true when registration, activation, and scalar layout validation succeed.
   *         注册、激活和标量布局校验均成功时返回 true。
   * @par Blocking / 阻塞行为
   * May block for bounded service discovery and RPC operations.
   * 可能因有界服务发现和 RPC 操作而阻塞。
   */
  bool RegisterController();

  /**
   * @brief Repeatedly reads state, updates control, and submits commands.
   *        循环读取状态、更新控制并提交指令。
   * @return false when state reception or command submission stops or fails.
   *         状态接收或指令提交停止或失败时返回 false。
   * @pre RegisterController() returned true. RegisterController() 已返回 true。
   * @par Blocking / 阻塞行为
   * Runs until the SDK stops or an operation fails.
   * 持续运行，直到 SDK 停止或某个操作失败。
   */
  bool RunControlLoop();

  /**
   * @brief Performs best-effort session unregistration during destruction.
   *        析构时尽力注销控制器会话。
   * @par Blocking / 阻塞行为
   * May perform a bounded cleanup RPC and wait for SDK worker shutdown.
   * 可能执行有界清理 RPC，并等待 SDK 工作线程停止。
   */
  ~DoubleInvertedController();

 private:
  std::unique_ptr<mujoco_sim::adapter_sdk::ControllerClient> controller_client_;
  mujoco_sim::adapter_sdk::ControllerOptions controller_options_;
  std::unique_ptr<ServerConfig> server_config_;

  std::array<double, 2> joint_pos_{};
  std::array<double, 2> joint_vel_{};
  std::array<double, 2> joint_torque_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 2> pos_span{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 2> vel_span{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 2> torque_span{};

  std::array<double, 2> joint_pos_ref_{};
  std::array<double, 2> joint_vel_ref_{};

  std::array<double, 2> joint_torque_cmd_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 2> torque_cmd_span{};

  /** @brief Reads the latest state into local scalar values. 将最新状态读取到本地标量值。 */
  bool read();
  /** @brief Updates the local torque command. 更新本地力矩指令。 */
  bool update();
  /** @brief Submits the latest dense torque command. 提交最新的稠密力矩指令。 */
  bool write();

  /** @brief Validates required identifiers and two-joint resource lists.
   *         校验必需标识符和双关节资源列表。
   */
  bool check_config_validity();
};

}  // namespace mujoco_sim_double_inverted_control_demo
