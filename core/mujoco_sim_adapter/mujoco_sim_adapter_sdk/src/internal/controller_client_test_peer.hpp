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
  static bool IsWaiting(ControllerClient* client);
};

}  // namespace mujoco_sim::adapter_sdk
