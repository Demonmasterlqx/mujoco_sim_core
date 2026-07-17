#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/v4/sync.pb.h>

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
    std::function<mujoco_sim::v4::GetAvailableResourcesResponse(
        mujoco_sim::v4::GetAvailableResourcesRequest)>
        get_available_resources;
    std::function<mujoco_sim::v4::RegisterAdapterResponse(mujoco_sim::v4::RegisterAdapterRequest)>
        register_adapter;
    std::function<mujoco_sim::v4::ActivateAdapterResponse(mujoco_sim::v4::ActivateAdapterRequest)>
        activate_adapter;
    std::function<mujoco_sim::v4::UnregisterAdapterResponse(
        mujoco_sim::v4::UnregisterAdapterRequest)>
        unregister_adapter;
    std::function<mujoco_sim::v4::GetSensorStreamsResponse(mujoco_sim::v4::GetSensorStreamsRequest)>
        get_sensor_streams;
    std::function<void(mujoco_sim::core::ControllerCommand)> command;
    std::function<void(mujoco_sim::core::AdapterHeartbeat)> heartbeat;
    std::function<void(mujoco_sim::core::ControllerFault)> fault;
    std::function<void(std::string_view, std::size_t)> malformed;
  };

  struct PublishResult {
    std::string topic;
    bool attempted = false;
    bool sent = false;
    bool publisher_created = false;
    bool serialization_failed = false;
    std::size_t bytes = 0;
    std::string transport_error;
  };

  EcalServerTransport(std::string server_name, std::shared_ptr<spdlog::logger> logger);
  ~EcalServerTransport();

  EcalServerTransport(const EcalServerTransport&) = delete;
  EcalServerTransport& operator=(const EcalServerTransport&) = delete;

  bool Start(Handlers handlers, std::string* error_message);
  void StopInputs();
  void Reset();

  PublishResult PublishControllerState(std::string_view adapter_id, std::string_view encoded_frame);
  PublishResult PublishObservation(std::string_view adapter_id, std::string_view encoded_frame);
  PublishResult PublishStatus(const mujoco_sim::v4::ServerStatus& status);
  bool PrepareAdapterPublisher(std::string_view adapter_id, mujoco_sim::v4::AdapterRole role,
                               std::string* error_message, bool* created);
  void RemoveAdapterPublisher(std::string_view adapter_id, mujoco_sim::v4::AdapterRole role);
  std::size_t SampleSubscriberCounts();
  bool HasControllerStateSubscriber(std::string_view adapter_id) const;

 private:
  std::string server_name_;
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<CallbackState> callback_state_;
  std::unique_ptr<eCAL::CServiceServer> registry_service_;
  std::unique_ptr<eCAL::CSubscriber> command_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> heartbeat_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> fault_subscriber_;
  std::unique_ptr<eCAL::CPublisher> status_publisher_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>> state_publishers_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>> observation_publishers_;
  std::unordered_map<std::string, std::size_t> state_subscriber_counts_;
  std::vector<std::string> methods_;
};

}  // namespace mujoco_sim::server
