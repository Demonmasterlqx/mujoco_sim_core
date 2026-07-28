#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>

#include "adapter_transport.hpp"

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Provides deterministic, non-installed Controller test hooks.
 *        提供确定性的、不安装的控制器测试钩子。
 *
 * Unless a member states otherwise, the borrowed client reference must remain
 * valid and quiescent while the test mutates internal state.
 * 除非成员另有说明，借用的客户端引用必须保持有效，且测试修改内部状态时客户端必须处于静止状态。
 */
class ControllerClientTestPeer {
 public:
  /** @brief Installs a fake transport and a deterministic registered layout.
   *         安装假传输与确定性注册布局。
   * @param client Borrowed quiescent test client. 借用的静止测试客户端。
   * @param transport Shared fake transport retained by the client. 客户端保留的共享假传输。
   * @param keepalive Positive publisher keepalive period. 正数的发布 keepalive 周期。
   * @post The client can receive injected State and be prepared for activation.
   *       客户端可接收注入的 State，并可进一步准备激活。
   */
  static void Configure(ControllerClient& client,
                        std::shared_ptr<internal::AdapterTransport> transport,
                        std::chrono::milliseconds keepalive = std::chrono::milliseconds(100));
  /**
   * @brief Installs a fake transport and caller-supplied deterministic layout.
   *        安装假传输与调用方提供的确定性布局。
   * @param client Borrowed quiescent test client. 借用的静止测试客户端。
   * @param transport Shared fake transport retained by the client. 客户端保留的共享假传输。
   * @param layout Complete immutable layout moved into the test registration.
   *               移入测试注册信息的完整不可变布局。
   * @param keepalive Positive publisher keepalive period. 正数的发布 keepalive 周期。
   */
  static void ConfigureWithLayout(
      ControllerClient& client, std::shared_ptr<internal::AdapterTransport> transport,
      ControllerLayout layout,
      std::chrono::milliseconds keepalive = std::chrono::milliseconds(100));
  /** @brief Injects one valid State frame through the production receive path.
   *         通过生产接收路径注入一帧有效 State。
   * @param client Borrowed configured test client. 借用的已配置测试客户端。
   * @param sequence Positive session-local message sequence. 正数的会话内消息序列。
   * @param tick_id Completed-step tick identifier. 已完成仿真步 tick 标识符。
   * @param observations Dense observations copied during this call. 本次调用复制的稠密观测值。
   */
  static void HandleState(ControllerClient& client, std::uint64_t sequence, std::uint64_t tick_id,
                          std::span<const double> observations);
  /** @brief Starts the production publisher loop for a configured active client.
   *         为已配置的活动客户端启动生产发布循环。
   * @param client Borrowed client not already owning a publisher thread.
   * 尚未持有发布线程的借用客户端。
   */
  static void StartPublisher(ControllerClient& client);
  /** @brief Marks the configured client as owning a remote session.
   *         将已配置客户端标记为持有远端会话。
   * @param client Borrowed quiescent test client. 借用的静止测试客户端。
   */
  static void EnableRemoteRegistration(ControllerClient& client);

  /**
   * @brief Injects a failure immediately before the Controller publisher thread is created.
   *        在创建控制器发布线程前立即注入失败。
   * @param client Borrowed test client; must not be used concurrently.
   *               借用的测试客户端；不得被并发使用。
   */
  static void FailPublisherStart(ControllerClient& client);
  /** @brief Reports whether a State read currently owns single-reader admission.
   *         返回 State 读取是否正在持有单读取方准入权。
   * @param client Borrowed test client; may be read concurrently with the waiter.
   *               借用测试客户端；可与等待方并发读取。
   * @return true while one read is waiting. 一个读取正在等待时返回 true。
   */
  static bool IsWaiting(ControllerClient& client);

  /**
   * @brief Makes a configured test client ready for Activate().
   *        使已配置的测试客户端进入可激活状态。
   * @param client Borrowed test client. 借用的测试客户端。
   */
  static void PrepareActivation(ControllerClient& client);

  /**
   * @brief Reports whether local cancellation has been published.
   *        返回本地取消状态是否已经发布。
   * @param client Borrowed test client. 借用的测试客户端。
   * @return true after local cancellation is visible; otherwise false.
   *         本地取消状态可见后返回 true，否则返回 false。
   */
  static bool StopRequested(ControllerClient& client);
};

}  // namespace mujoco_sim::adapter_sdk
