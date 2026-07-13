#include "ecal_server_transport.hpp"

#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/server.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <spdlog/logger.h>

#include "server_logging.hpp"
#include "server_runtime.hpp"

#include <exception>
#include <type_traits>
#include <utility>

namespace mujoco_sim::server {
namespace {

class CallbackGuard {
 public:
  explicit CallbackGuard(std::shared_ptr<CallbackState> state)
      : state_(std::move(state)), entered_(state_ && state_->Enter()) {}
  ~CallbackGuard() {
    if (entered_) state_->Leave();
  }
  explicit operator bool() const { return entered_; }

 private:
  std::shared_ptr<CallbackState> state_;
  bool entered_ = false;
};

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation information;
  information.method_name = method;
  return information;
}

void LogCallbackFailure(const std::shared_ptr<spdlog::logger>& logger,
                        std::string_view operation,
                        std::string_view error) noexcept {
  try {
    if (logger) {
      logger->error("event=ecal_exception boundary=callback operation={} error={}",
                    LogValue(operation), LogValue(error));
    }
  } catch (...) {
  }
}

template <typename Response>
Response ErrorResponse(mujoco_sim::v2::ErrorCode code, std::string message) {
  Response response;
  response.set_code(code);
  response.set_message(std::move(message));
  return response;
}

template <typename Request, typename Response, typename Handler>
bool BindRpc(eCAL::CServiceServer* service, const std::string& method,
             const std::shared_ptr<CallbackState>& callback_state,
             const std::shared_ptr<spdlog::logger>& logger,
             const EcalServerTransport::Handlers& handlers,
             Handler handler) {
  if (service == nullptr || !handler) return false;
  return service->SetMethodCallback(
      MethodInfo(method),
      [callback_state, logger, handlers, method,
       handler = std::move(handler)](const eCAL::SServiceMethodInformation&,
                                     const std::string& bytes,
                                     std::string& response_bytes) noexcept {
        CallbackGuard guard(callback_state);
        if (!guard) return 1;
        Response response;
        std::string error;
        const bool completed = RunWithExceptionBoundary(
            [&] {
              Request request;
              if (!IsProtocolPayloadSizeAllowed(bytes.size()) ||
                  !request.ParseFromString(bytes)) {
                if (handlers.malformed) handlers.malformed(method, bytes.size());
                response = ErrorResponse<Response>(
                    mujoco_sim::v2::INVALID_ARGUMENT, "invalid request");
              } else {
                response = handler(std::move(request));
              }
              if (!response.SerializeToString(&response_bytes)) {
                throw std::runtime_error("failed to serialize service response");
              }
            },
            &error);
        if (!completed) {
          LogCallbackFailure(logger, method, error);
          return 1;
        }
        return 0;
      });
}

template <typename Message, typename Handler>
void BindSubscriber(std::unique_ptr<eCAL::CSubscriber>* subscriber,
                    const std::string& topic, std::string_view operation,
                    const std::shared_ptr<CallbackState>& callback_state,
                    const std::shared_ptr<spdlog::logger>& logger,
                    const EcalServerTransport::Handlers& handlers,
                    Handler handler) {
  *subscriber = std::make_unique<eCAL::CSubscriber>(topic);
  (*subscriber)->SetReceiveCallback(
      [callback_state, logger, handlers, operation = std::string(operation),
       handler = std::move(handler)](
          const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
          const eCAL::SReceiveCallbackData& data) noexcept {
        CallbackGuard guard(callback_state);
        if (!guard) return;
        std::string error;
        if (!RunWithExceptionBoundary(
                [&] {
                  Message message;
                  if (data.buffer == nullptr || data.buffer_size == 0 ||
                      !IsProtocolPayloadSizeAllowed(data.buffer_size) ||
                      !message.ParseFromArray(
                          data.buffer, static_cast<int>(data.buffer_size))) {
                    if (handlers.malformed) {
                      handlers.malformed(operation, data.buffer_size);
                    }
                    return;
                  }
                  handler(std::move(message));
                },
                &error)) {
          LogCallbackFailure(logger, operation, error);
        }
      });
}

template <typename Message>
EcalServerTransport::PublishResult PublishProto(
    const Message& message, eCAL::CPublisher* publisher) {
  EcalServerTransport::PublishResult result;
  if (publisher == nullptr) return result;
  std::string bytes;
  if (!message.SerializeToString(&bytes)) {
    result.serialization_failed = true;
    return result;
  }
  result.attempted = true;
  result.bytes = bytes.size();
  RunWithExceptionBoundary(
      [&] {
        result.subscribers_before = publisher->GetSubscriberCount();
        result.sent = publisher->Send(bytes);
        result.subscribers_after = publisher->GetSubscriberCount();
      },
      &result.transport_error);
  return result;
}

}  // namespace

EcalServerTransport::EcalServerTransport(
    std::string server_name, std::shared_ptr<spdlog::logger> logger)
    : server_name_(std::move(server_name)),
      logger_(std::move(logger)),
      callback_state_(std::make_shared<CallbackState>()) {}

EcalServerTransport::~EcalServerTransport() {
  StopInputs();
  Reset();
}

bool EcalServerTransport::Start(Handlers handlers,
                                std::string* error_message) {
  if (error_message == nullptr) return false;
  registry_service_ = std::make_unique<eCAL::CServiceServer>(
      mujoco_sim::protocol::RegistryServiceName(server_name_));

  const auto bind = [this, &handlers, error_message]<typename Request,
                                                       typename Response>(
                        const char* method,
                        std::function<Response(Request)> handler) {
    if (BindRpc<Request, Response>(registry_service_.get(), method,
                                   callback_state_, logger_, handlers,
                                   std::move(handler))) {
      methods_.push_back(method);
      return true;
    }
    *error_message = "failed to register method: " + std::string(method);
    return false;
  };

  if (!bind(mujoco_sim::protocol::kGetAvailableResourcesMethod,
            handlers.get_available_resources) ||
      !bind(mujoco_sim::protocol::kRegisterAdapterMethod,
            handlers.register_adapter) ||
      !bind(mujoco_sim::protocol::kActivateAdapterMethod,
            handlers.activate_adapter) ||
      !bind(mujoco_sim::protocol::kUnregisterAdapterMethod,
            handlers.unregister_adapter) ||
      !bind(mujoco_sim::protocol::kGetSensorStreamsMethod,
            handlers.get_sensor_streams)) {
    StopInputs();
    return false;
  }

  BindSubscriber<mujoco_sim::v2::AdapterCommit>(
      &commit_subscriber_,
      mujoco_sim::protocol::CommitTopicName(server_name_), "AdapterCommit",
      callback_state_, logger_, handlers, handlers.commit);
  BindSubscriber<mujoco_sim::v2::AdapterHeartbeat>(
      &heartbeat_subscriber_,
      mujoco_sim::protocol::AdapterHeartbeatTopicName(server_name_),
      "AdapterHeartbeat", callback_state_, logger_, handlers,
      handlers.heartbeat);
  status_publisher_ = std::make_unique<eCAL::CPublisher>(
      mujoco_sim::protocol::StatusTopicName(server_name_));
  return true;
}

void EcalServerTransport::StopInputs() {
  if (!callback_state_) return;
  callback_state_->StopAccepting();
  if (commit_subscriber_) {
    commit_subscriber_->RemoveReceiveCallback();
    commit_subscriber_.reset();
  }
  if (heartbeat_subscriber_) {
    heartbeat_subscriber_->RemoveReceiveCallback();
    heartbeat_subscriber_.reset();
  }
  if (registry_service_) {
    for (const auto& method : methods_) {
      registry_service_->RemoveMethodCallback(method);
    }
  }
  methods_.clear();
  callback_state_->WaitForIdle();
}

void EcalServerTransport::Reset() {
  tick_publishers_.clear();
  observation_publishers_.clear();
  status_publisher_.reset();
  registry_service_.reset();
}

EcalServerTransport::PublishResult EcalServerTransport::PublishTick(
    const mujoco_sim::core::IssuedTickHandle& issued) {
  PublishResult result;
  if (!issued || !issued->current.load(std::memory_order_acquire)) return result;
  result.attempted = true;
  result.bytes = issued->serialized_bytes.size();
  RunWithExceptionBoundary(
      [&] {
        auto& publisher = tick_publishers_[issued->message.adapter_id()];
        if (!publisher) {
          result.topic = mujoco_sim::protocol::TickTopicName(
              server_name_, issued->message.adapter_id());
          publisher = std::make_unique<eCAL::CPublisher>(result.topic);
          result.publisher_created = true;
        }
        result.subscribers_before = publisher->GetSubscriberCount();
        result.sent = publisher->Send(issued->serialized_bytes);
        result.subscribers_after = publisher->GetSubscriberCount();
      },
      &result.transport_error);
  return result;
}

EcalServerTransport::PublishResult EcalServerTransport::PublishObservation(
    const mujoco_sim::v2::ObservationFrame& frame) {
  auto& publisher = observation_publishers_[frame.adapter_id()];
  PublishResult result;
  if (!publisher) {
    result.topic = mujoco_sim::protocol::ObservationTopicName(
        server_name_, frame.adapter_id());
    publisher = std::make_unique<eCAL::CPublisher>(result.topic);
    result.publisher_created = true;
  }
  auto published = PublishProto(frame, publisher.get());
  published.topic = std::move(result.topic);
  published.publisher_created = result.publisher_created;
  return published;
}

EcalServerTransport::PublishResult EcalServerTransport::PublishStatus(
    const mujoco_sim::v2::ServerStatus& status) {
  return PublishProto(status, status_publisher_.get());
}

void EcalServerTransport::EraseAdapterPublisher(
    std::string_view adapter_id, mujoco_sim::v2::AdapterRole role) {
  if (role == mujoco_sim::v2::OBSERVER) {
    observation_publishers_.erase(std::string(adapter_id));
  } else {
    tick_publishers_.erase(std::string(adapter_id));
  }
}

}  // namespace mujoco_sim::server
