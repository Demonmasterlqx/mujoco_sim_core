#include "adapter_transport.hpp"

#include <chrono>
#include <mutex>
#include <thread>
#include <utility>

#include <ecal/pubsub/publisher.h>
#include <ecal/service/client.h>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::adapter_sdk::internal {
namespace {

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

class EcalAdapterTransport final : public AdapterTransport {
 public:
  EcalAdapterTransport(std::string server_name, std::uint32_t service_timeout_ms,
                       const std::atomic<bool>* stop_requested, bool publish_commands)
      : service_timeout_ms_(service_timeout_ms), stop_requested_(stop_requested) {
    eCAL::ServiceMethodInformationSetT methods;
    methods.insert(MethodInfo(mujoco_sim::protocol::kRegisterAdapterMethod));
    methods.insert(MethodInfo(mujoco_sim::protocol::kActivateAdapterMethod));
    methods.insert(MethodInfo(mujoco_sim::protocol::kUnregisterAdapterMethod));
    registry_client_ = std::make_unique<eCAL::CServiceClient>(
        mujoco_sim::protocol::RegistryServiceName(server_name), methods);
    heartbeat_publisher_ = std::make_unique<eCAL::CPublisher>(
        mujoco_sim::protocol::SessionHeartbeatTopicName(server_name));
    if (publish_commands) {
      command_publisher_ = std::make_unique<eCAL::CPublisher>(
          mujoco_sim::protocol::ControllerCommandTopicName(server_name));
      fault_publisher_ = std::make_unique<eCAL::CPublisher>(
          mujoco_sim::protocol::ControllerFaultTopicName(server_name));
    }
  }

  bool Register(const mujoco_sim::v4::RegisterAdapterRequest& request,
                mujoco_sim::v4::RegisterAdapterResponse* response,
                std::string* error_message) override {
    return Call(mujoco_sim::protocol::kRegisterAdapterMethod, request, response, error_message,
                false);
  }

  bool Activate(const mujoco_sim::v4::ActivateAdapterRequest& request,
                mujoco_sim::v4::ActivateAdapterResponse* response,
                std::string* error_message) override {
    return Call(mujoco_sim::protocol::kActivateAdapterMethod, request, response, error_message,
                false);
  }

  bool Unregister(const mujoco_sim::v4::UnregisterAdapterRequest& request,
                  mujoco_sim::v4::UnregisterAdapterResponse* response,
                  std::string* error_message) override {
    return Call(mujoco_sim::protocol::kUnregisterAdapterMethod, request, response, error_message,
                true);
  }

  bool PublishCommand(std::string_view encoded_command) override {
    std::lock_guard lock(publisher_mutex_);
    return command_publisher_ != nullptr &&
           command_publisher_->Send(encoded_command.data(), encoded_command.size());
  }

  bool PublishFault(const mujoco_sim::v4::ControllerFault& fault) override {
    return Publish(fault, fault_publisher_.get());
  }

  void PublishHeartbeat(const mujoco_sim::v4::SessionHeartbeat& heartbeat) override {
    (void)Publish(heartbeat, heartbeat_publisher_.get());
  }

 private:
  template <typename Request, typename Response>
  bool Call(const std::string& method, const Request& request, Response* response,
            std::string* error_message, bool allow_after_stop) {
    if (response == nullptr || error_message == nullptr) return false;
    std::string bytes;
    if (!request.SerializeToString(&bytes)) {
      *error_message = "failed to serialize service request";
      return false;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(service_timeout_ms_);
    while ((allow_after_stop || !stop_requested_->load()) &&
           std::chrono::steady_clock::now() < deadline) {
      if (!registry_client_->IsConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      eCAL::ServiceResponseVecT responses;
      registry_client_->CallWithResponse(method, bytes, responses, 100);
      for (const auto& service_response : responses) {
        if (service_response.call_state != eCAL::eCallState::executed) continue;
        if (!response->ParseFromString(service_response.response)) {
          *error_message = "failed to parse service response";
          return false;
        }
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    *error_message = stop_requested_->load() && !allow_after_stop
                         ? "service call cancelled: " + method
                         : "service call timed out: " + method;
    return false;
  }

  template <typename Message>
  bool Publish(const Message& message, eCAL::CPublisher* publisher) {
    std::string bytes;
    if (publisher == nullptr || !message.SerializeToString(&bytes)) return false;
    std::lock_guard lock(publisher_mutex_);
    return publisher->Send(bytes);
  }

  std::uint32_t service_timeout_ms_;
  const std::atomic<bool>* stop_requested_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
  std::unique_ptr<eCAL::CPublisher> command_publisher_;
  std::unique_ptr<eCAL::CPublisher> fault_publisher_;
  std::unique_ptr<eCAL::CPublisher> heartbeat_publisher_;
  std::mutex publisher_mutex_;
};

class EcalSensorStreamTransport final : public SensorStreamTransport {
 public:
  EcalSensorStreamTransport(std::string server_name, std::uint32_t service_timeout_ms,
                            const std::atomic<bool>* stop_requested)
      : service_timeout_ms_(service_timeout_ms), stop_requested_(stop_requested) {
    eCAL::ServiceMethodInformationSetT methods;
    methods.insert(MethodInfo(mujoco_sim::protocol::kGetSensorStreamsMethod));
    registry_client_ = std::make_unique<eCAL::CServiceClient>(
        mujoco_sim::protocol::RegistryServiceName(server_name), methods);
  }

  bool Discover(const mujoco_sim::v4::GetSensorStreamsRequest& request,
                mujoco_sim::v4::GetSensorStreamsResponse* response,
                std::string* error_message) override {
    if (response == nullptr || error_message == nullptr) return false;
    std::string bytes;
    if (!request.SerializeToString(&bytes)) {
      *error_message = "failed to serialize service request";
      return false;
    }
    const auto method = mujoco_sim::protocol::kGetSensorStreamsMethod;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(service_timeout_ms_);
    while (!stop_requested_->load() && std::chrono::steady_clock::now() < deadline) {
      if (!registry_client_->IsConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      eCAL::ServiceResponseVecT responses;
      registry_client_->CallWithResponse(method, bytes, responses, 100);
      for (const auto& service_response : responses) {
        if (service_response.call_state != eCAL::eCallState::executed) continue;
        if (!response->ParseFromString(service_response.response)) {
          *error_message = "failed to parse service response";
          return false;
        }
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    *error_message = std::string(stop_requested_->load() ? "service call cancelled: "
                                                         : "service call timed out: ") +
                     method;
    return false;
  }

 private:
  std::uint32_t service_timeout_ms_;
  const std::atomic<bool>* stop_requested_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
};

}  // namespace

std::shared_ptr<AdapterTransport> MakeEcalAdapterTransport(const std::string& server_name,
                                                           std::uint32_t service_timeout_ms,
                                                           const std::atomic<bool>* stop_requested,
                                                           bool publish_commands) {
  return std::make_shared<EcalAdapterTransport>(server_name, service_timeout_ms, stop_requested,
                                                publish_commands);
}

std::shared_ptr<SensorStreamTransport> MakeEcalSensorStreamTransport(
    const std::string& server_name, std::uint32_t service_timeout_ms,
    const std::atomic<bool>* stop_requested) {
  return std::make_shared<EcalSensorStreamTransport>(server_name, service_timeout_ms,
                                                     stop_requested);
}

}  // namespace mujoco_sim::adapter_sdk::internal
