#include "adapter_transport.hpp"

#include <chrono>
#include <exception>
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

bool MayRetryServiceCall(bool allow_after_stop, const std::atomic<bool>* stop_requested,
                         std::chrono::steady_clock::time_point deadline) {
  return (allow_after_stop || !stop_requested->load()) &&
         std::chrono::steady_clock::now() < deadline;
}

std::string ServiceExecutionError(const std::string& method,
                                  const eCAL::SServiceResponse& response) {
  std::string error = "service execution failed for '" + method +
                      "': ret_state=" + std::to_string(response.ret_state);
  if (!response.error_msg.empty()) error += ", eCAL error: " + response.error_msg;
  return error;
}

enum class ServiceResponseResult {
  kNoExecutedResponse,
  kSuccess,
  kFailure,
};

template <typename Response>
ServiceResponseResult ConsumeServiceResponses(const std::string& method,
                                              const eCAL::ServiceResponseVecT& responses,
                                              Response* response, std::string* last_ecal_error,
                                              std::string* error_message) {
  for (const auto& service_response : responses) {
    if (service_response.call_state != eCAL::eCallState::executed) {
      if (!service_response.error_msg.empty()) *last_ecal_error = service_response.error_msg;
      continue;
    }
    if (service_response.ret_state != 0) {
      *error_message = ServiceExecutionError(method, service_response);
      return ServiceResponseResult::kFailure;
    }
    if (!response->ParseFromString(service_response.response)) {
      *error_message = "failed to parse service response: " + method;
      return ServiceResponseResult::kFailure;
    }
    return ServiceResponseResult::kSuccess;
  }
  return ServiceResponseResult::kNoExecutedResponse;
}

template <typename Request, typename Response>
bool CallService(eCAL::CServiceClient* client, std::uint32_t service_timeout_ms,
                 const std::atomic<bool>* stop_requested, bool allow_after_stop,
                 const std::string& method, const Request& request, Response* response,
                 std::string* error_message) {
  error_message->clear();
  try {
    std::string bytes;
    if (!request.SerializeToString(&bytes)) {
      *error_message = "failed to serialize service request: " + method;
      return false;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(service_timeout_ms);
    std::string last_ecal_error;
    while (MayRetryServiceCall(allow_after_stop, stop_requested, deadline)) {
      eCAL::ServiceResponseVecT responses;
      if (!client->IsConnected() || !client->CallWithResponse(method, bytes, responses, 100)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      const auto result =
          ConsumeServiceResponses(method, responses, response, &last_ecal_error, error_message);
      if (result == ServiceResponseResult::kSuccess) return true;
      if (result == ServiceResponseResult::kFailure) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    *error_message = stop_requested->load() && !allow_after_stop
                         ? "service call cancelled: " + method
                         : "service call timed out: " + method;
    if (!last_ecal_error.empty()) *error_message += "; last eCAL error: " + last_ecal_error;
  } catch (const std::exception& error) {
    *error_message = "service call threw for '" + method + "': " + error.what();
  } catch (...) {
    *error_message = "service call threw for '" + method + "': unknown exception";
  }
  return false;
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

  PublishResult PublishCommand(std::string_view encoded_command) override {
    PublishResult result;
    if (command_publisher_ == nullptr) {
      result.error = "ControllerCommand publisher is unavailable";
      return result;
    }
    try {
      result.sent = command_publisher_->Send(encoded_command.data(), encoded_command.size());
    } catch (const std::exception& error) {
      result.error = std::string("eCAL ControllerCommand publish threw: ") + error.what();
    } catch (...) {
      result.error = "eCAL ControllerCommand publish threw: unknown exception";
    }
    return result;
  }

  PublishResult PublishFault(const mujoco_sim::v4::ControllerFault& fault) override {
    return Publish(fault, fault_publisher_.get(), "ControllerFault");
  }

  PublishResult PublishHeartbeat(const mujoco_sim::v4::SessionHeartbeat& heartbeat) override {
    return Publish(heartbeat, heartbeat_publisher_.get(), "SessionHeartbeat");
  }

 private:
  template <typename Request, typename Response>
  bool Call(const std::string& method, const Request& request, Response* response,
            std::string* error_message, bool allow_after_stop) {
    if (response == nullptr || error_message == nullptr) return false;
    return CallService(registry_client_.get(), service_timeout_ms_, stop_requested_,
                       allow_after_stop, method, request, response, error_message);
  }

  template <typename Message>
  PublishResult Publish(const Message& message, eCAL::CPublisher* publisher,
                        std::string_view message_type) {
    PublishResult result;
    if (publisher == nullptr) {
      result.error = std::string(message_type) + " publisher is unavailable";
      return result;
    }
    try {
      std::string bytes;
      if (!message.SerializeToString(&bytes)) {
        result.error = "failed to serialize " + std::string(message_type);
        return result;
      }
      result.sent = publisher->Send(bytes);
    } catch (const std::exception& error) {
      result.error = "eCAL " + std::string(message_type) + " publish threw: " + error.what();
    } catch (...) {
      result.error = "eCAL " + std::string(message_type) + " publish threw: unknown exception";
    }
    return result;
  }

  std::uint32_t service_timeout_ms_;
  const std::atomic<bool>* stop_requested_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
  std::unique_ptr<eCAL::CPublisher> command_publisher_;
  std::unique_ptr<eCAL::CPublisher> fault_publisher_;
  std::unique_ptr<eCAL::CPublisher> heartbeat_publisher_;
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
    const std::string method = mujoco_sim::protocol::kGetSensorStreamsMethod;
    return CallService(registry_client_.get(), service_timeout_ms_, stop_requested_, false, method,
                       request, response, error_message);
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
