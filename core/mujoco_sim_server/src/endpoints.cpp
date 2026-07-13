#include "endpoints.hpp"

#include <mujoco_sim/protocol/topics.hpp>

#include "server_runtime.hpp"

#include <utility>

namespace mujoco_sim::server {

namespace {
eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation information;
  information.method_name = method;
  return information;
}

template <typename Message>
bool ParseServicePayload(const std::string& bytes, Message* message) {
  return message != nullptr && IsProtocolPayloadSizeAllowed(bytes.size()) &&
         message->ParseFromString(bytes);
}

template <typename Message>
bool ParseTopicPayload(const void* data, std::size_t size, Message* message) {
  return data != nullptr && size > 0 && message != nullptr &&
         IsProtocolPayloadSizeAllowed(size) &&
         message->ParseFromArray(data, static_cast<int>(size));
}
}  // namespace

RegistryEndpoint::RegistryEndpoint(const std::string& service_name)
    : service_(std::make_unique<eCAL::CServiceServer>(service_name)) {}

bool RegistryEndpoint::RegisterMethod(const std::string& method,
                                      ServiceHandler handler) {
  if (!service_ || !handler) return false;
  const bool registered = service_->SetMethodCallback(
      MethodInfo(method),
      [handler = std::move(handler)](
          const eCAL::SServiceMethodInformation&, const std::string& request,
          std::string& response) noexcept {
        return handler(request, &response);
      });
  if (registered) methods_.push_back(method);
  return registered;
}

bool RegistryEndpoint::Parse(
    const std::string& bytes,
    mujoco_sim::v2::GetAvailableResourcesRequest* message) const {
  return ParseServicePayload(bytes, message);
}
bool RegistryEndpoint::Parse(
    const std::string& bytes,
    mujoco_sim::v2::RegisterAdapterRequest* message) const {
  return ParseServicePayload(bytes, message);
}
bool RegistryEndpoint::Parse(
    const std::string& bytes,
    mujoco_sim::v2::ActivateAdapterRequest* message) const {
  return ParseServicePayload(bytes, message);
}
bool RegistryEndpoint::Parse(
    const std::string& bytes,
    mujoco_sim::v2::UnregisterAdapterRequest* message) const {
  return ParseServicePayload(bytes, message);
}
bool RegistryEndpoint::Parse(
    const std::string& bytes,
    mujoco_sim::v2::GetSensorStreamsRequest* message) const {
  return ParseServicePayload(bytes, message);
}
bool RegistryEndpoint::ParseHeartbeat(
    const void* data, std::size_t size,
    mujoco_sim::v2::AdapterHeartbeat* message) const {
  return ParseTopicPayload(data, size, message);
}

void RegistryEndpoint::SubscribeHeartbeat(const std::string& topic,
                                          DataHandler handler) {
  heartbeat_subscriber_ = std::make_unique<eCAL::CSubscriber>(topic);
  heartbeat_subscriber_->SetReceiveCallback(
      [handler = std::move(handler)](
          const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
          const eCAL::SReceiveCallbackData& data) noexcept {
        handler(data.buffer, data.buffer_size);
      });
}

void RegistryEndpoint::StopCallbacks() {
  if (heartbeat_subscriber_) {
    heartbeat_subscriber_->RemoveReceiveCallback();
    heartbeat_subscriber_.reset();
  }
  if (service_) {
    for (const auto& method : methods_) {
      service_->RemoveMethodCallback(method);
    }
  }
  methods_.clear();
}
void RegistryEndpoint::ResetService() { service_.reset(); }

void ControlEndpoint::SubscribeCommits(const std::string& topic,
                                       DataHandler handler) {
  commit_subscriber_ = std::make_unique<eCAL::CSubscriber>(topic);
  commit_subscriber_->SetReceiveCallback(
      [handler = std::move(handler)](
          const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
          const eCAL::SReceiveCallbackData& data) noexcept {
        handler(data.buffer, data.buffer_size);
      });
}

bool ControlEndpoint::ParseCommit(
    const void* data, std::size_t size,
    mujoco_sim::v2::AdapterCommit* message) const {
  return ParseTopicPayload(data, size, message);
}

void ControlEndpoint::StopSubscriber() {
  if (commit_subscriber_) {
    commit_subscriber_->RemoveReceiveCallback();
    commit_subscriber_.reset();
  }
}
void ControlEndpoint::ClearPublishers() {
  std::lock_guard lock(publisher_mutex_);
  publishers_.clear();
}
void ControlEndpoint::ErasePublisher(const std::string& adapter_id) {
  std::lock_guard lock(publisher_mutex_);
  publishers_.erase(adapter_id);
}

ControlEndpoint::PublishResult ControlEndpoint::Publish(
    const mujoco_sim::core::IssuedTickHandle& issued,
    const std::string& server_name) {
  PublishResult result;
  if (!issued->current.load(std::memory_order_acquire)) return result;
  result.attempted = true;
  result.bytes = issued->serialized_bytes.size();
  std::lock_guard lock(publisher_mutex_);
  RunWithExceptionBoundary(
      [&] {
        auto& publisher = publishers_[issued->message.adapter_id()];
        if (!publisher) {
          result.topic = mujoco_sim::protocol::TickTopicName(
              server_name, issued->message.adapter_id());
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

void ObservationEndpoint::ClearPublishers() {
  std::lock_guard lock(publisher_mutex_);
  publishers_.clear();
}
void ObservationEndpoint::ErasePublisher(const std::string& adapter_id) {
  std::lock_guard lock(publisher_mutex_);
  publishers_.erase(adapter_id);
}

ObservationEndpoint::PublishResult ObservationEndpoint::Publish(
    const mujoco_sim::v2::ObservationFrame& frame,
    const std::string& server_name) {
  PublishResult result;
  std::string bytes;
  if (!frame.SerializeToString(&bytes)) {
    result.serialization_failed = true;
    return result;
  }
  result.attempted = true;
  result.bytes = bytes.size();
  std::lock_guard lock(publisher_mutex_);
  RunWithExceptionBoundary(
      [&] {
        auto& publisher = publishers_[frame.adapter_id()];
        if (!publisher) {
          result.topic = mujoco_sim::protocol::ObservationTopicName(
              server_name, frame.adapter_id());
          publisher = std::make_unique<eCAL::CPublisher>(result.topic);
          result.publisher_created = true;
        }
        result.subscribers_before = publisher->GetSubscriberCount();
        result.sent = publisher->Send(bytes);
        result.subscribers_after = publisher->GetSubscriberCount();
      },
      &result.transport_error);
  return result;
}

StatusEndpoint::StatusEndpoint(const std::string& topic)
    : publisher_(std::make_unique<eCAL::CPublisher>(topic)) {}
void StatusEndpoint::Reset() { publisher_.reset(); }
StatusEndpoint::PublishResult StatusEndpoint::Publish(
    const mujoco_sim::v2::ServerStatus& status) {
  PublishResult result;
  if (!publisher_) return result;
  std::string bytes;
  if (!status.SerializeToString(&bytes)) {
    result.serialization_failed = true;
    return result;
  }
  result.attempted = true;
  result.bytes = bytes.size();
  std::lock_guard lock(publisher_mutex_);
  RunWithExceptionBoundary(
      [&] {
        result.subscribers_before = publisher_->GetSubscriberCount();
        result.sent = publisher_->Send(bytes);
        result.subscribers_after = publisher_->GetSubscriberCount();
      },
      &result.transport_error);
  return result;
}

}  // namespace mujoco_sim::server
