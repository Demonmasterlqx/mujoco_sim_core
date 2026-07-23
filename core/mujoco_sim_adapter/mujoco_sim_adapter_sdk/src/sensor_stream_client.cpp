#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>

#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "ecal_runtime.hpp"
#include "internal/callback_boundary.hpp"
#include "internal/sensor_stream_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

std::optional<std::uint64_t> BytesPerPixel(mujoco_sim::v4::StreamFormat format) {
  switch (format) {
    case mujoco_sim::v4::RGB8:
      return 3;
    case mujoco_sim::v4::DEPTH32F:
    case mujoco_sim::v4::SEGMENTATION32U:
      return 4;
    case mujoco_sim::v4::XYZ32F:
      return 12;
    default:
      return std::nullopt;
  }
}

}  // namespace

OwnedStreamSample StreamSampleView::Copy() const {
  return OwnedStreamSample{header_, std::vector<std::byte>(payload_.begin(), payload_.end())};
}

SensorStreamClient::Impl::Impl(SensorStreamOptions options)
    : options_(std::move(options)), callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

SensorStreamClient::Impl::~Impl() {
  Stop();
  StopSampleReception();
  transport_.reset();
  ecal_runtime_.reset();
}

bool SensorStreamClient::Impl::ValidateOptions(std::string* error_message) const {
  if (!mujoco_sim::protocol::ValidateIdentifier(
          options_.server_name, mujoco_sim::protocol::IdentifierKind::kServer, error_message)) {
    return false;
  }
  if (options_.stream_names.empty()) {
    *error_message = "at least one sensor stream is required";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto& name : options_.stream_names) {
    if (!mujoco_sim::protocol::ValidateIdentifier(
            name, mujoco_sim::protocol::IdentifierKind::kStream, error_message)) {
      return false;
    }
    if (!names.insert(name).second) {
      *error_message = "duplicate stream name: " + name;
      return false;
    }
  }
  if (options_.service_timeout_ms == 0) {
    *error_message = "service_timeout_ms must be greater than zero";
    return false;
  }
  if (options_.queue_capacity == 0) {
    *error_message = "queue_capacity must be greater than zero";
    return false;
  }
  return true;
}

bool SensorStreamClient::Impl::Start(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  auto lifecycle_operation = lifecycle_.Acquire();
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (phase_ == Phase::kActive) {
      *error_message = "sensor stream client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error_message = "sensor stream client has been stopped";
    return false;
  }
  if (!ValidateOptions(error_message)) {
    return false;
  }

  ecal_runtime_ = internal::AcquireEcalRuntime("mujoco-sim-sensor-stream-client", error_message);
  if (!ecal_runtime_) {
    return false;
  }
  try {
    if (!transport_) {
      transport_ = internal::MakeEcalSensorStreamTransport(
          options_.server_name, options_.service_timeout_ms, &stop_requested_);
    }
  } catch (const std::exception& error) {
    *error_message = std::string("failed to create Sensor Stream transport: ") + error.what();
    return false;
  } catch (...) {
    *error_message = "failed to create Sensor Stream transport: unknown exception";
    return false;
  }
  if (!DiscoverStreams(error_message) || !ConfigureDiscoveredStreams(error_message)) {
    transport_.reset();
    ecal_runtime_.reset();
    return false;
  }

  if (!StartSampleReception(error_message) || !FinishStart(error_message)) {
    transport_.reset();
    ecal_runtime_.reset();
    return false;
  }
  return true;
}

bool SensorStreamClient::Impl::StartSampleReception(std::string* error_message) {
  try {
    {
      std::lock_guard callback_lock(callback_state_->mutex);
      callback_state_->client = this;
      callback_state_->enabled = true;
    }
    {
      std::lock_guard lock(mutex_);
      phase_ = Phase::kStarting;
    }
    const auto callback_state = callback_state_;
    const auto weak_self = weak_from_this();
    for (const auto& stream_name : options_.stream_names) {
      auto subscriber = std::make_unique<eCAL::CSubscriber>(
          mujoco_sim::protocol::SensorStreamTopicName(options_.server_name, stream_name));
      subscriber->SetReceiveCallback([callback_state, weak_self, stream_name](
                                         const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
                                         const eCAL::SReceiveCallbackData& data) noexcept {
        internal::InvokeCallbackBoundary(
            [&] {
              const auto self = weak_self.lock();
              if (!self) return;
              CallbackLease callback(callback_state);
              if (callback.client() != nullptr) {
                self->HandleStreamBytes(stream_name, data.buffer, data.buffer_size);
              }
            },
            [&](const char* error) {
              if (const auto self = weak_self.lock()) {
                self->RecordDiagnostic(stream_name,
                                       std::string("stream callback failed: ") + error);
              }
            });
      });
      subscribers_.emplace(stream_name, std::move(subscriber));
    }
    status_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::StatusTopicName(options_.server_name));
    status_subscriber_->SetReceiveCallback(
        [callback_state, weak_self](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
                                    const eCAL::SReceiveCallbackData& data) noexcept {
          internal::InvokeCallbackBoundary(
              [&] {
                const auto self = weak_self.lock();
                if (!self) return;
                CallbackLease callback(callback_state);
                if (callback.client() != nullptr) {
                  self->HandleStatusBytes(data.buffer, data.buffer_size);
                }
              },
              [&](const char*) {
                if (const auto self = weak_self.lock()) self->Stop();
              });
        });
  } catch (const std::exception& error) {
    StopSampleReception();
    *error_message = std::string("failed to create stream endpoints: ") + error.what();
    return false;
  } catch (...) {
    StopSampleReception();
    *error_message = "failed to create stream endpoints: unknown exception";
    return false;
  }
  return true;
}

bool SensorStreamClient::Impl::FinishStart(std::string* error_message) {
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      phase_ = Phase::kStopped;
      *error_message = "sensor stream start was cancelled";
    } else {
      monitor_ecal_ = true;
      phase_ = Phase::kActive;
    }
  }
  if (!error_message->empty()) {
    StopSampleReception();
    return false;
  }
  return true;
}

bool SensorStreamClient::Impl::DiscoverStreams(std::string* error_message) {
  mujoco_sim::v4::GetSensorStreamsRequest request;
  if (!transport_->Discover(request, &discovery_, error_message)) {
    return false;
  }
  if (discovery_.code() != mujoco_sim::v4::OK) {
    *error_message = discovery_.message();
    return false;
  }
  if (discovery_.epoch() == 0) {
    *error_message = "invalid sensor stream discovery response";
    return false;
  }
  return true;
}

bool SensorStreamClient::Impl::ConfigureDiscoveredStreams(std::string* error_message) {
  std::unordered_map<std::string, const mujoco_sim::v4::StreamDescriptor*> available;
  for (const auto& descriptor : discovery_.streams()) {
    if (!available.emplace(descriptor.name(), &descriptor).second) {
      *error_message = "duplicate sensor stream descriptor: " + descriptor.name();
      return false;
    }
  }

  std::unordered_map<std::string, std::shared_ptr<StreamState>> configured;
  for (const auto& name : options_.stream_names) {
    const auto found = available.find(name);
    if (found == available.end()) {
      *error_message = "requested sensor stream was not found: " + name;
      return false;
    }
    if (!ValidateDescriptor(*found->second, error_message)) {
      return false;
    }
    auto state = std::make_shared<StreamState>();
    state->descriptor = *found->second;
    configured.emplace(name, std::move(state));
  }

  std::lock_guard lock(mutex_);
  streams_ = std::move(configured);
  last_status_tick_id_.reset();
  return true;
}

bool SensorStreamClient::Impl::ValidateDescriptor(
    const mujoco_sim::v4::StreamDescriptor& descriptor, std::string* error_message) const {
  const auto bytes_per_pixel = BytesPerPixel(descriptor.format());
  if (descriptor.name().empty() || !bytes_per_pixel.has_value()) {
    *error_message = "sensor stream has an invalid name or format";
    return false;
  }
  if (descriptor.topic() !=
      mujoco_sim::protocol::SensorStreamTopicName(options_.server_name, descriptor.name())) {
    *error_message = "sensor stream topic does not match its descriptor: " + descriptor.name();
    return false;
  }
  if (descriptor.width() == 0 || descriptor.height() == 0) {
    *error_message = "sensor stream dimensions must be greater than zero: " + descriptor.name();
    return false;
  }
  const std::uint64_t minimum_stride =
      static_cast<std::uint64_t>(descriptor.width()) * *bytes_per_pixel;
  if (descriptor.row_stride_bytes() < minimum_stride) {
    *error_message = "sensor stream row stride is too small: " + descriptor.name();
    return false;
  }
  const std::uint64_t payload_size =
      static_cast<std::uint64_t>(descriptor.row_stride_bytes()) * descriptor.height();
  if (payload_size > mujoco_sim::protocol::kMaximumStreamPayloadBytes ||
      descriptor.maximum_payload_bytes() < payload_size ||
      descriptor.maximum_payload_bytes() > mujoco_sim::protocol::kMaximumStreamPayloadBytes) {
    *error_message = "sensor stream payload bounds are invalid: " + descriptor.name();
    return false;
  }
  if (descriptor.frame_convention() == mujoco_sim::v4::FRAME_CONVENTION_UNSPECIFIED) {
    *error_message = "sensor stream frame convention is unspecified: " + descriptor.name();
    return false;
  }
  if (!std::isfinite(descriptor.rate_hz()) || descriptor.rate_hz() <= 0.0) {
    *error_message = "sensor stream rate must be finite and positive: " + descriptor.name();
    return false;
  }
  return true;
}

bool SensorStreamClient::Impl::SetCallback(const std::string& stream_name, StreamCallback callback,
                                           std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  if (!callback) {
    *error_message = "stream callback must not be empty";
    return false;
  }

  std::lock_guard lock(mutex_);
  if (stop_requested_) {
    *error_message = "sensor stream client has been stopped";
    return false;
  }
  if (phase_ != Phase::kActive) {
    *error_message = "sensor stream client must be started first";
    return false;
  }
  const auto found = streams_.find(stream_name);
  if (found == streams_.end()) {
    *error_message = "unknown sensor stream: " + stream_name;
    return false;
  }
  auto& stream = *found->second;
  if (stream.mode == DeliveryMode::kWait) {
    *error_message = "stream is already in blocking wait mode: " + stream_name;
    return false;
  }
  stream.dropped_samples += stream.samples.size();
  stream.samples.Clear();
  stream.mode = DeliveryMode::kCallback;
  stream.callback = std::move(callback);
  return true;
}

SampleWaitResult SensorStreamClient::Impl::WaitForSample(const std::string& stream_name,
                                                         std::chrono::milliseconds timeout,
                                                         OwnedStreamSample* sample,
                                                         std::string* error_message) {
  if (sample == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "sample and error_message must not be null";
    }
    return SampleWaitResult::kError;
  }
  error_message->clear();
  if (timeout.count() < 0) {
    *error_message = "sample wait timeout must not be negative";
    return SampleWaitResult::kError;
  }

  std::unique_lock lock(mutex_);
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    return SampleWaitResult::kStopped;
  }
  if (phase_ != Phase::kActive) {
    *error_message = "sensor stream client must be started before waiting";
    return SampleWaitResult::kError;
  }
  const auto found = streams_.find(stream_name);
  if (found == streams_.end()) {
    *error_message = "unknown sensor stream: " + stream_name;
    return SampleWaitResult::kError;
  }
  const auto stream = found->second;
  if (stream->mode == DeliveryMode::kCallback) {
    *error_message = "stream is already in callback mode: " + stream_name;
    return SampleWaitResult::kError;
  }
  if (stream->samples.wait_in_progress()) {
    *error_message = "another thread is already waiting for stream: " + stream_name;
    return SampleWaitResult::kError;
  }
  stream->mode = DeliveryMode::kWait;
  const auto result = stream->samples.Wait(
      lock, condition_, timeout,
      [this] { return stop_requested_.load() || (monitor_ecal_ && !eCAL::Ok()); },
      [&stream] { return stream->terminal_error; }, sample, error_message);
  return internal::ToPublicWaitResult<SampleWaitResult>(result);
}

void SensorStreamClient::Impl::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    phase_ = Phase::kStopped;
  }
  condition_.notify_all();
}

std::optional<mujoco_sim::v4::StreamDescriptor> SensorStreamClient::Impl::descriptor(
    const std::string& stream_name) const {
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  if (found == streams_.end()) {
    return std::nullopt;
  }
  return found->second->descriptor;
}

std::uint64_t SensorStreamClient::Impl::dropped_samples(const std::string& stream_name) const {
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  return found == streams_.end() ? 0 : found->second->dropped_samples;
}

std::string SensorStreamClient::Impl::stream_diagnostic(const std::string& stream_name) const {
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  return found == streams_.end() ? std::string{} : found->second->diagnostic;
}

}  // namespace mujoco_sim::adapter_sdk
