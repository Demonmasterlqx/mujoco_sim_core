#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/v2/sync.pb.h>

namespace eCAL {
class CPublisher;
class CServiceServer;
class CSubscriber;
}  // namespace eCAL

namespace spdlog {
class logger;
}  // namespace spdlog

namespace mujoco_sim::server {

class CallbackState;

class EcalServerTransport {
 public:
  struct Handlers {
    std::function<mujoco_sim::v2::GetAvailableResourcesResponse(
        mujoco_sim::v2::GetAvailableResourcesRequest)>
        get_available_resources;
    std::function<mujoco_sim::v2::RegisterAdapterResponse(
        mujoco_sim::v2::RegisterAdapterRequest)>
        register_adapter;
    std::function<mujoco_sim::v2::ActivateAdapterResponse(
        mujoco_sim::v2::ActivateAdapterRequest)>
        activate_adapter;
    std::function<mujoco_sim::v2::UnregisterAdapterResponse(
        mujoco_sim::v2::UnregisterAdapterRequest)>
        unregister_adapter;
    std::function<mujoco_sim::v2::GetSensorStreamsResponse(
        mujoco_sim::v2::GetSensorStreamsRequest)>
        get_sensor_streams;
    std::function<void(mujoco_sim::v2::AdapterCommit)> commit;
    std::function<void(mujoco_sim::v2::AdapterHeartbeat)> heartbeat;
    std::function<void(std::string_view, std::size_t)> malformed;
  };

  struct PublishResult {
    std::string topic;
    std::size_t subscribers_before = 0;
    std::size_t subscribers_after = 0;
    bool attempted = false;
    bool sent = false;
    bool publisher_created = false;
    bool serialization_failed = false;
    std::size_t bytes = 0;
    std::string transport_error;
  };

  EcalServerTransport(std::string server_name,
                      std::shared_ptr<spdlog::logger> logger);
  ~EcalServerTransport();

  EcalServerTransport(const EcalServerTransport&) = delete;
  EcalServerTransport& operator=(const EcalServerTransport&) = delete;

  bool Start(Handlers handlers, std::string* error_message);
  void StopInputs();
  void Reset();

  PublishResult PublishTick(
      const mujoco_sim::core::IssuedTickHandle& issued);
  PublishResult PublishObservation(
      const mujoco_sim::v2::ObservationFrame& frame);
  PublishResult PublishStatus(const mujoco_sim::v2::ServerStatus& status);
  void EraseAdapterPublisher(std::string_view adapter_id,
                             mujoco_sim::v2::AdapterRole role);

 private:
  std::string server_name_;
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<CallbackState> callback_state_;
  std::unique_ptr<eCAL::CServiceServer> registry_service_;
  std::unique_ptr<eCAL::CSubscriber> commit_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> heartbeat_subscriber_;
  std::unique_ptr<eCAL::CPublisher> status_publisher_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>>
      tick_publishers_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>>
      observation_publishers_;
  std::vector<std::string> methods_;
};

}  // namespace mujoco_sim::server
