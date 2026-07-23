#include "ecal_server_transport.hpp"

#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/server.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <spdlog/logger.h>

#include "server_logging.hpp"
#include "server_runtime.hpp"

#include <exception>
#include <type_traits>
#include <utility>
#include <vector>

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

void LogCallbackFailure(const std::shared_ptr<spdlog::logger>& logger, std::string_view operation,
                        std::string_view error) noexcept {
  try {
    if (logger) {
      logger->error("event=ecal_exception boundary=callback operation={} error={}",
                    LogValue(operation), LogValue(error));
    }
  } catch (...) {
  }
}

void LogTransportFailure(const std::shared_ptr<spdlog::logger>& logger, std::string_view operation,
                         std::string_view endpoint, std::string_view error) noexcept {
  try {
    if (logger) {
      logger->warn("event=ecal_transport_failure operation={} endpoint={} error={}",
                   LogValue(operation), LogValue(endpoint), LogValue(error));
    }
  } catch (...) {
  }
}

template <typename Response>
Response ErrorResponse(mujoco_sim::v4::ErrorCode code, std::string message) {
  Response response;
  response.set_code(code);
  response.set_message(std::move(message));
  return response;
}

template <typename Request, typename Response, typename Handler>
bool BindRpc(eCAL::CServiceServer* service, const std::string& method,
             const std::shared_ptr<CallbackState>& callback_state,
             const std::shared_ptr<spdlog::logger>& logger,
             const EcalServerTransport::Handlers& handlers, Handler handler) {
  if (service == nullptr || !handler) return false;
  return service->SetMethodCallback(
      MethodInfo(method), [callback_state, logger, handlers, method, handler = std::move(handler)](
                              const eCAL::SServiceMethodInformation&, const std::string& bytes,
                              std::string& response_bytes) noexcept {
        CallbackGuard guard(callback_state);
        if (!guard) return 1;
        Response response;
        std::string error;
        const bool completed = RunWithExceptionBoundary(
            [&] {
              Request request;
              if (!IsProtocolPayloadSizeAllowed(bytes.size()) || !request.ParseFromString(bytes)) {
                if (handlers.malformed) handlers.malformed(method, bytes.size());
                response =
                    ErrorResponse<Response>(mujoco_sim::v4::INVALID_ARGUMENT, "invalid request");
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
void BindSubscriber(std::unique_ptr<eCAL::CSubscriber>* subscriber, const std::string& topic,
                    std::string_view operation,
                    const std::shared_ptr<CallbackState>& callback_state,
                    const std::shared_ptr<spdlog::logger>& logger,
                    const EcalServerTransport::Handlers& handlers, Handler handler) {
  *subscriber = std::make_unique<eCAL::CSubscriber>(topic);
  (*subscriber)
      ->SetReceiveCallback([callback_state, logger, handlers, operation = std::string(operation),
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
                      !message.ParseFromArray(data.buffer, static_cast<int>(data.buffer_size))) {
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
EcalServerTransport::PublishResult PublishProto(const Message& message, eCAL::CPublisher* publisher,
                                                std::string topic) {
  EcalServerTransport::PublishResult result;
  result.topic = std::move(topic);
  if (publisher == nullptr) {
    result.transport_error = "publisher is unavailable";
    return result;
  }
  std::string bytes;
  bool serialized = false;
  if (!RunWithExceptionBoundary([&] { serialized = message.SerializeToString(&bytes); },
                                &result.transport_error) ||
      !serialized) {
    result.serialization_failed = true;
    return result;
  }
  result.attempted = true;
  result.bytes = bytes.size();
  RunWithExceptionBoundary([&] { result.sent = publisher->Send(bytes); }, &result.transport_error);
  return result;
}

void BindCommandSubscriber(std::unique_ptr<eCAL::CSubscriber>* subscriber, const std::string& topic,
                           const std::shared_ptr<CallbackState>& callback_state,
                           const std::shared_ptr<spdlog::logger>& logger,
                           const EcalServerTransport::Handlers& handlers) {
  *subscriber = std::make_unique<eCAL::CSubscriber>(topic);
  (*subscriber)
      ->SetReceiveCallback([callback_state, logger, handlers](
                               const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
                               const eCAL::SReceiveCallbackData& data) noexcept {
        CallbackGuard guard(callback_state);
        if (!guard) return;
        std::string exception_error;
        if (!RunWithExceptionBoundary(
                [&] {
                  if (data.buffer == nullptr || data.buffer_size == 0 ||
                      !IsProtocolPayloadSizeAllowed(data.buffer_size)) {
                    if (handlers.malformed) {
                      handlers.malformed("Command", data.buffer_size);
                    }
                    return;
                  }
                  const auto bytes =
                      std::span(static_cast<const std::byte*>(data.buffer), data.buffer_size);
                  mujoco_sim::protocol::ParsedDataFrame parsed;
                  mujoco_sim::protocol::DataFrameError error;
                  if (!mujoco_sim::protocol::ParseDataFrame(bytes, &parsed, &error) ||
                      parsed.header.kind !=
                          mujoco_sim::protocol::DataFrameKind::kControllerCommand) {
                    if (handlers.malformed) {
                      handlers.malformed("Command", data.buffer_size);
                    }
                    return;
                  }
                  mujoco_sim::core::ControllerCommand command;
                  command.session = {parsed.header.session_id, parsed.header.epoch};
                  command.layout = {parsed.header.layout.low, parsed.header.layout.high};
                  command.sequence = parsed.header.sequence;
                  command.commands.resize(parsed.scalar_count);
                  if (!mujoco_sim::protocol::DecodeDataFrameScalars(parsed, command.commands,
                                                                    &error)) {
                    if (handlers.malformed) {
                      handlers.malformed("Command", data.buffer_size);
                    }
                    return;
                  }
                  handlers.command(std::move(command));
                },
                &exception_error)) {
          LogCallbackFailure(logger, "Command", exception_error);
        }
      });
}

}  // namespace

EcalServerTransport::EcalServerTransport(std::string server_name,
                                         std::shared_ptr<spdlog::logger> logger)
    : server_name_(std::move(server_name)),
      logger_(std::move(logger)),
      callback_state_(std::make_shared<CallbackState>()) {}

EcalServerTransport::~EcalServerTransport() noexcept {
  std::string error;
  if (!RunWithExceptionBoundary([this] { StopInputs(); }, &error)) {
    LogTransportFailure(logger_, "transport_destructor_stop_inputs", server_name_, error);
  }
  if (!RunWithExceptionBoundary([this] { Reset(); }, &error)) {
    LogTransportFailure(logger_, "transport_destructor_reset", server_name_, error);
  }
}

bool EcalServerTransport::Start(Handlers handlers, std::string* error_message) {
  if (error_message == nullptr) return false;
  try {
    registry_service_ = std::make_unique<eCAL::CServiceServer>(
        mujoco_sim::protocol::RegistryServiceName(server_name_));

    const auto bind = [this, &handlers, error_message]<typename Request, typename Response>(
                          const char* method, std::function<Response(Request)> handler) {
      if (BindRpc<Request, Response>(registry_service_.get(), method, callback_state_, logger_,
                                     handlers, std::move(handler))) {
        methods_.push_back(method);
        return true;
      }
      *error_message = "failed to register method: " + std::string(method);
      return false;
    };

    if (!bind(mujoco_sim::protocol::kGetAvailableResourcesMethod,
              handlers.get_available_resources) ||
        !bind(mujoco_sim::protocol::kRegisterAdapterMethod, handlers.register_adapter) ||
        !bind(mujoco_sim::protocol::kActivateAdapterMethod, handlers.activate_adapter) ||
        !bind(mujoco_sim::protocol::kUnregisterAdapterMethod, handlers.unregister_adapter) ||
        !bind(mujoco_sim::protocol::kGetSensorStreamsMethod, handlers.get_sensor_streams)) {
      StopInputs();
      return false;
    }

    BindCommandSubscriber(&command_subscriber_,
                          mujoco_sim::protocol::ControllerCommandTopicName(server_name_),
                          callback_state_, logger_, handlers);
    BindSubscriber<mujoco_sim::v4::SessionHeartbeat>(
        &heartbeat_subscriber_, mujoco_sim::protocol::SessionHeartbeatTopicName(server_name_),
        "SessionHeartbeat", callback_state_, logger_, handlers,
        [handler = handlers.heartbeat](mujoco_sim::v4::SessionHeartbeat wire) {
          handler({{wire.session_id(), wire.epoch()}, wire.last_seen()});
        });
    BindSubscriber<mujoco_sim::v4::ControllerFault>(
        &fault_subscriber_, mujoco_sim::protocol::ControllerFaultTopicName(server_name_),
        "ControllerFault", callback_state_, logger_, handlers,
        [handler = handlers.fault](mujoco_sim::v4::ControllerFault wire) {
          mujoco_sim::core::ControllerFault fault;
          fault.session = {wire.session_id(), wire.epoch()};
          switch (wire.code()) {
            case mujoco_sim::v4::CONTROLLER_ERROR:
              fault.code = mujoco_sim::core::FaultCode::kControllerError;
              break;
            case mujoco_sim::v4::INVALID_OUTPUT:
              fault.code = mujoco_sim::core::FaultCode::kInvalidOutput;
              break;
            case mujoco_sim::v4::INTERNAL_CONTROLLER_ERROR:
              fault.code = mujoco_sim::core::FaultCode::kInternalControllerError;
              break;
            default:
              fault.code = mujoco_sim::core::FaultCode::kUnspecified;
              break;
          }
          fault.message = wire.message();
          handler(std::move(fault));
        });
    status_publisher_ =
        std::make_unique<eCAL::CPublisher>(mujoco_sim::protocol::StatusTopicName(server_name_));
    return true;
  } catch (const std::exception& error) {
    *error_message = "failed to start eCAL server transport: " + std::string(error.what());
  } catch (...) {
    *error_message = "failed to start eCAL server transport: unknown exception";
  }
  StopInputs();
  Reset();
  return false;
}

void EcalServerTransport::StopInputs() {
  if (!callback_state_) return;
  callback_state_->StopAccepting();
  const auto remove_receive_callback = [this](auto* subscriber, std::string_view operation) {
    if (!*subscriber) return;
    std::string error;
    if (!RunWithExceptionBoundary([&] { (*subscriber)->RemoveReceiveCallback(); }, &error)) {
      LogTransportFailure(logger_, operation, server_name_, error);
    }
  };
  if (command_subscriber_) {
    remove_receive_callback(&command_subscriber_, "remove_command_callback");
  }
  if (heartbeat_subscriber_) {
    remove_receive_callback(&heartbeat_subscriber_, "remove_heartbeat_callback");
  }
  if (fault_subscriber_) {
    remove_receive_callback(&fault_subscriber_, "remove_fault_callback");
  }
  if (registry_service_) {
    for (const auto& method : methods_) {
      bool removed = false;
      std::string error;
      const bool completed = RunWithExceptionBoundary(
          [&] { removed = registry_service_->RemoveMethodCallback(method); }, &error);
      if (!completed || !removed) {
        LogTransportFailure(logger_, "remove_rpc_callback", method,
                            completed ? "eCAL returned failure" : error);
      }
    }
  }
  methods_.clear();
  callback_state_->WaitForIdle();
  command_subscriber_.reset();
  heartbeat_subscriber_.reset();
  fault_subscriber_.reset();
}

void EcalServerTransport::Reset() {
  state_publishers_.clear();
  observation_publishers_.clear();
  state_subscriber_counts_.clear();
  status_publisher_.reset();
  registry_service_.reset();
  reported_transport_failures_.clear();
}

EcalServerTransport::PublishResult EcalServerTransport::PublishControllerState(
    std::string_view adapter_id, std::string_view encoded_frame) {
  PublishResult result;
  result.topic =
      mujoco_sim::protocol::ControllerStateTopicName(server_name_, std::string(adapter_id));
  result.attempted = true;
  result.bytes = encoded_frame.size();
  RunWithExceptionBoundary(
      [&] {
        auto& publisher = state_publishers_[std::string(adapter_id)];
        if (!publisher) {
          publisher = std::make_unique<eCAL::CPublisher>(result.topic);
          result.publisher_created = true;
        }
        result.sent = publisher->Send(encoded_frame.data(), encoded_frame.size());
      },
      &result.transport_error);
  return result;
}

EcalServerTransport::PublishResult EcalServerTransport::PublishObservation(
    std::string_view adapter_id, std::string_view encoded_frame) {
  PublishResult result;
  result.topic = mujoco_sim::protocol::ObservationTopicName(server_name_, std::string(adapter_id));
  result.attempted = true;
  result.bytes = encoded_frame.size();
  RunWithExceptionBoundary(
      [&] {
        auto& publisher = observation_publishers_[std::string(adapter_id)];
        if (!publisher) {
          publisher = std::make_unique<eCAL::CPublisher>(result.topic);
          result.publisher_created = true;
        }
        result.sent = publisher->Send(encoded_frame.data(), encoded_frame.size());
      },
      &result.transport_error);
  return result;
}

EcalServerTransport::PublishResult EcalServerTransport::PublishStatus(
    const mujoco_sim::v4::ServerStatus& status) {
  return PublishProto(status, status_publisher_.get(),
                      mujoco_sim::protocol::StatusTopicName(server_name_));
}

bool EcalServerTransport::PrepareAdapterPublisher(std::string_view adapter_id,
                                                  mujoco_sim::v4::AdapterRole role,
                                                  std::string* error_message, bool* created) {
  if (error_message == nullptr || created == nullptr) return false;
  error_message->clear();
  *created = false;
  if (role != mujoco_sim::v4::CONTROLLER && role != mujoco_sim::v4::OBSERVER) {
    *error_message = "unsupported adapter role";
    return false;
  }
  const bool ready = RunWithExceptionBoundary(
      [&] {
        const std::string id(adapter_id);
        if (role == mujoco_sim::v4::OBSERVER) {
          auto& publisher = observation_publishers_[id];
          if (!publisher) {
            publisher = std::make_unique<eCAL::CPublisher>(
                mujoco_sim::protocol::ObservationTopicName(server_name_, id));
            *created = true;
          }
        } else if (role == mujoco_sim::v4::CONTROLLER) {
          auto& publisher = state_publishers_[id];
          if (!publisher) {
            const std::string topic =
                mujoco_sim::protocol::ControllerStateTopicName(server_name_, id);
            publisher = std::make_unique<eCAL::CPublisher>(topic);
            *created = true;
            // eCAL 6 does not advertise a raw publisher's transport until its
            // first Send(). Prime it before the registration response, while
            // the adapter cannot yet have created its State subscriber.
            constexpr char kTransportPrime = 0;
            std::string prime_error;
            const bool completed = RunWithExceptionBoundary(
                [&] { (void)publisher->Send(&kTransportPrime, 1); }, &prime_error);
            const std::string failure_key = "prime:" + topic;
            if (!completed) {
              if (reported_transport_failures_.insert(failure_key).second) {
                LogTransportFailure(logger_, "prime_publisher", topic, prime_error);
              }
            } else {
              reported_transport_failures_.erase(failure_key);
            }
          }
        }
      },
      error_message);
  if (!ready && *created) {
    RemoveAdapterPublisher(adapter_id, role);
    *created = false;
  }
  return ready;
}

void EcalServerTransport::RemoveAdapterPublisher(std::string_view adapter_id,
                                                 mujoco_sim::v4::AdapterRole role) {
  const std::string id(adapter_id);
  if (role == mujoco_sim::v4::CONTROLLER) {
    state_publishers_.erase(id);
    state_subscriber_counts_.erase(id);
    const std::string topic = mujoco_sim::protocol::ControllerStateTopicName(server_name_, id);
    reported_transport_failures_.erase("sample:" + topic);
    reported_transport_failures_.erase("prime:" + topic);
  } else if (role == mujoco_sim::v4::OBSERVER) {
    observation_publishers_.erase(id);
    const std::string topic = mujoco_sim::protocol::ObservationTopicName(server_name_, id);
    reported_transport_failures_.erase("sample:" + topic);
  }
}

std::size_t EcalServerTransport::SampleSubscriberCounts() {
  std::size_t queries = 0;
  const auto sample_count = [this](eCAL::CPublisher* publisher, const std::string& topic) {
    std::size_t subscribers = 0;
    std::string error;
    const bool completed =
        RunWithExceptionBoundary([&] { subscribers = publisher->GetSubscriberCount(); }, &error);
    const std::string failure_key = "sample:" + topic;
    if (!completed) {
      if (reported_transport_failures_.insert(failure_key).second) {
        LogTransportFailure(logger_, "sample_subscribers", topic, error);
      }
    } else {
      reported_transport_failures_.erase(failure_key);
    }
    return subscribers;
  };
  const auto prime = [this](eCAL::CPublisher* publisher, const std::string& topic) {
    constexpr char kTransportPrime = 0;
    std::string error;
    const bool completed =
        RunWithExceptionBoundary([&] { (void)publisher->Send(&kTransportPrime, 1); }, &error);
    const std::string failure_key = "prime:" + topic;
    if (!completed) {
      if (reported_transport_failures_.insert(failure_key).second) {
        LogTransportFailure(logger_, "prime_publisher", topic, error);
      }
    } else {
      reported_transport_failures_.erase(failure_key);
    }
  };
  for (const auto& [adapter_id, publisher] : state_publishers_) {
    const std::string topic =
        mujoco_sim::protocol::ControllerStateTopicName(server_name_, adapter_id);
    const auto subscribers = sample_count(publisher.get(), topic);
    state_subscriber_counts_[adapter_id] = subscribers;
    if (subscribers == 0) {
      // Raw eCAL publishers may remain undiscoverable until Send() occurs
      // while the subscriber exists. Keep bootstrap traffic on this sampled
      // lifecycle path, never on the State publish path.
      prime(publisher.get(), topic);
    }
    ++queries;
  }
  for (const auto& [adapter_id, publisher] : observation_publishers_) {
    sample_count(publisher.get(),
                 mujoco_sim::protocol::ObservationTopicName(server_name_, adapter_id));
    ++queries;
  }
  if (status_publisher_) {
    sample_count(status_publisher_.get(), mujoco_sim::protocol::StatusTopicName(server_name_));
    ++queries;
  }
  return queries;
}

bool EcalServerTransport::HasControllerStateSubscriber(std::string_view adapter_id) const {
  const auto found = state_subscriber_counts_.find(std::string(adapter_id));
  return found != state_subscriber_counts_.end() && found->second > 0;
}

}  // namespace mujoco_sim::server
