#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>

#include "adapter_transport.hpp"

namespace mujoco_sim::adapter_sdk {

class ControllerClientTestPeer {
 public:
  static void Configure(ControllerClient* client,
                        std::shared_ptr<internal::AdapterTransport> transport,
                        std::chrono::milliseconds keepalive = std::chrono::milliseconds(100));
  static void HandleState(ControllerClient* client, std::uint64_t sequence, std::uint64_t tick_id,
                          std::span<const double> observations);
  static void StartPublisher(ControllerClient* client);
  static void EnableRemoteRegistration(ControllerClient* client);

  /**
   * @brief Injects a failure immediately before the Controller publisher thread is created.
   *        在创建控制器发布线程前立即注入失败。
   * @param client Borrowed test client; must not be null and must not be used concurrently.
   *               借用的测试客户端；不得为空，也不得被并发使用。
   */
  static void FailPublisherStart(ControllerClient* client);
  static bool IsWaiting(ControllerClient* client);

  /**
   * @brief Makes a configured test client ready for Activate().
   *        使已配置的测试客户端进入可激活状态。
   * @param client Borrowed test client; must not be null.
   *               借用的测试客户端；不得为空。
   */
  static void PrepareActivation(ControllerClient* client);

  /**
   * @brief Reports whether local cancellation has been published.
   *        返回本地取消状态是否已经发布。
   * @param client Borrowed test client; must not be null.
   *               借用的测试客户端；不得为空。
   * @return true after local cancellation is visible; otherwise false.
   *         本地取消状态可见后返回 true，否则返回 false。
   */
  static bool StopRequested(ControllerClient* client);
};

}  // namespace mujoco_sim::adapter_sdk
