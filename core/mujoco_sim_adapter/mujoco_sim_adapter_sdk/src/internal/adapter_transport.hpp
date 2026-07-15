#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <mujoco_sim/v3/sync.pb.h>

namespace mujoco_sim::adapter_sdk::internal {

class AdapterTransport {
 public:
  virtual ~AdapterTransport() = default;
  virtual bool Register(const mujoco_sim::v3::RegisterAdapterRequest& request,
                        mujoco_sim::v3::RegisterAdapterResponse* response,
                        std::string* error_message) = 0;
  virtual bool Activate(const mujoco_sim::v3::ActivateAdapterRequest& request,
                        mujoco_sim::v3::ActivateAdapterResponse* response,
                        std::string* error_message) = 0;
  virtual bool Unregister(
      const mujoco_sim::v3::UnregisterAdapterRequest& request,
      mujoco_sim::v3::UnregisterAdapterResponse* response,
      std::string* error_message) = 0;
  virtual bool PublishCommit(std::string_view encoded_commit) = 0;
  virtual bool PublishFault(const mujoco_sim::v3::ControllerFault& fault) = 0;
  virtual void PublishHeartbeat(
      const mujoco_sim::v3::SessionHeartbeat& heartbeat) = 0;
};

class SensorStreamTransport {
 public:
  virtual ~SensorStreamTransport() = default;
  virtual bool Discover(
      const mujoco_sim::v3::GetSensorStreamsRequest& request,
      mujoco_sim::v3::GetSensorStreamsResponse* response,
      std::string* error_message) = 0;
};

std::shared_ptr<AdapterTransport> MakeEcalAdapterTransport(
    const std::string& server_name, std::uint32_t service_timeout_ms,
    const std::atomic<bool>* stop_requested, bool publish_commits);
std::shared_ptr<SensorStreamTransport> MakeEcalSensorStreamTransport(
    const std::string& server_name, std::uint32_t service_timeout_ms,
    const std::atomic<bool>* stop_requested);

}  // namespace mujoco_sim::adapter_sdk::internal
