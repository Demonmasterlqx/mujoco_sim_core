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

namespace mujoco_sim::adapter_sdk {
namespace {

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

std::optional<std::uint64_t> BytesPerPixel(
    mujoco_sim::v2::StreamFormat format) {
  switch (format) {
    case mujoco_sim::v2::RGB8:
      return 3;
    case mujoco_sim::v2::DEPTH32F:
    case mujoco_sim::v2::SEGMENTATION32U:
      return 4;
    case mujoco_sim::v2::XYZ32F:
      return 12;
    default:
      return std::nullopt;
  }
}

}  // namespace

OwnedStreamSample StreamSampleView::Copy() const {
  return OwnedStreamSample{
      header_, std::vector<std::byte>(payload_.begin(), payload_.end())};
}

SensorStreamClient::SensorStreamClient(SensorStreamOptions options)
    : options_(std::move(options)),
      callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

SensorStreamClient::~SensorStreamClient() {
  Stop();
  StopSampleReception();
  registry_client_.reset();
  ecal_runtime_.reset();
}

bool SensorStreamClient::ValidateOptions(std::string* error_message) const {
  if (!mujoco_sim::protocol::ValidateIdentifier(
          options_.server_name,
          mujoco_sim::protocol::IdentifierKind::kServer, error_message)) {
    return false;
  }
  if (options_.stream_names.empty()) {
    *error_message = "at least one sensor stream is required";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto& name : options_.stream_names) {
    if (!mujoco_sim::protocol::ValidateIdentifier(
            name, mujoco_sim::protocol::IdentifierKind::kStream,
            error_message)) {
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

bool SensorStreamClient::Start(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (started_) {
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

  ecal_runtime_ = internal::AcquireEcalRuntime(
      "mujoco-sim-sensor-stream-client", error_message);
  if (!ecal_runtime_) {
    return false;
  }
  eCAL::ServiceMethodInformationSetT methods;
  methods.insert(MethodInfo(mujoco_sim::protocol::kGetSensorStreamsMethod));
  registry_client_ = std::make_unique<eCAL::CServiceClient>(
      mujoco_sim::protocol::RegistryServiceName(options_.server_name),
      methods);
  if (!DiscoverStreams(error_message) ||
      !ConfigureDiscoveredStreams(error_message)) {
    registry_client_.reset();
    ecal_runtime_.reset();
    return false;
  }

  try {
    {
      std::lock_guard callback_lock(callback_state_->mutex);
      callback_state_->client = this;
      callback_state_->enabled = true;
    }
    {
      std::lock_guard lock(mutex_);
      accepting_samples_ = true;
    }
    const auto callback_state = callback_state_;
    for (const auto& stream_name : options_.stream_names) {
      auto subscriber = std::make_unique<eCAL::CSubscriber>(
          mujoco_sim::protocol::SensorStreamTopicName(options_.server_name,
                                                      stream_name));
      subscriber->SetReceiveCallback(
          [callback_state, stream_name](
              const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
              const eCAL::SReceiveCallbackData& data) {
            CallbackLease callback(callback_state);
            if (callback.client() != nullptr) {
              callback.client()->HandleStreamBytes(
                  stream_name, data.buffer, data.buffer_size);
            }
          });
      subscribers_.emplace(stream_name, std::move(subscriber));
    }
    status_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::StatusTopicName(options_.server_name));
    status_subscriber_->SetReceiveCallback(
        [callback_state](const eCAL::STopicId&,
                         const eCAL::SDataTypeInformation&,
                         const eCAL::SReceiveCallbackData& data) {
          CallbackLease callback(callback_state);
          if (callback.client() != nullptr) {
            callback.client()->HandleStatusBytes(data.buffer,
                                                 data.buffer_size);
          }
        });
  } catch (const std::exception& error) {
    StopSampleReception();
    registry_client_.reset();
    ecal_runtime_.reset();
    *error_message = std::string("failed to create stream endpoints: ") +
                     error.what();
    return false;
  }

  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      accepting_samples_ = false;
      *error_message = "sensor stream start was cancelled";
    } else {
      monitor_ecal_ = true;
      started_ = true;
    }
  }
  if (!error_message->empty()) {
    StopSampleReception();
    registry_client_.reset();
    ecal_runtime_.reset();
    return false;
  }
  return true;
}

bool SensorStreamClient::DiscoverStreams(std::string* error_message) {
  mujoco_sim::v2::GetSensorStreamsRequest request;
  request.set_server_name(options_.server_name);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  if (!CallService(mujoco_sim::protocol::kGetSensorStreamsMethod, request,
                   &discovery_, error_message)) {
    return false;
  }
  if (discovery_.code() != mujoco_sim::v2::OK) {
    *error_message = discovery_.message();
    return false;
  }
  if (discovery_.epoch() == 0) {
    *error_message = "invalid sensor stream discovery response";
    return false;
  }
  return true;
}

bool SensorStreamClient::ConfigureDiscoveredStreams(
    std::string* error_message) {
  std::unordered_map<std::string, const mujoco_sim::v2::StreamDescriptor*>
      available;
  for (const auto& descriptor : discovery_.streams()) {
    if (!available.emplace(descriptor.name(), &descriptor).second) {
      *error_message = "duplicate sensor stream descriptor: " +
                       descriptor.name();
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

bool SensorStreamClient::ValidateDescriptor(
    const mujoco_sim::v2::StreamDescriptor& descriptor,
    std::string* error_message) const {
  const auto bytes_per_pixel = BytesPerPixel(descriptor.format());
  if (descriptor.name().empty() || !bytes_per_pixel.has_value()) {
    *error_message = "sensor stream has an invalid name or format";
    return false;
  }
  if (descriptor.topic() != mujoco_sim::protocol::SensorStreamTopicName(
                                options_.server_name, descriptor.name())) {
    *error_message = "sensor stream topic does not match its descriptor: " +
                     descriptor.name();
    return false;
  }
  if (descriptor.width() == 0 || descriptor.height() == 0) {
    *error_message = "sensor stream dimensions must be greater than zero: " +
                     descriptor.name();
    return false;
  }
  const std::uint64_t minimum_stride =
      static_cast<std::uint64_t>(descriptor.width()) * *bytes_per_pixel;
  if (descriptor.row_stride_bytes() < minimum_stride) {
    *error_message = "sensor stream row stride is too small: " +
                     descriptor.name();
    return false;
  }
  const std::uint64_t payload_size =
      static_cast<std::uint64_t>(descriptor.row_stride_bytes()) *
      descriptor.height();
  if (payload_size > mujoco_sim::protocol::kMaximumStreamPayloadBytes ||
      descriptor.maximum_payload_bytes() < payload_size ||
      descriptor.maximum_payload_bytes() >
          mujoco_sim::protocol::kMaximumStreamPayloadBytes) {
    *error_message = "sensor stream payload bounds are invalid: " +
                     descriptor.name();
    return false;
  }
  if (descriptor.frame_convention() ==
      mujoco_sim::v2::FRAME_CONVENTION_UNSPECIFIED) {
    *error_message = "sensor stream frame convention is unspecified: " +
                     descriptor.name();
    return false;
  }
  if (!std::isfinite(descriptor.rate_hz()) || descriptor.rate_hz() <= 0.0) {
    *error_message = "sensor stream rate must be finite and positive: " +
                     descriptor.name();
    return false;
  }
  return true;
}

template <typename Request, typename Response>
bool SensorStreamClient::CallService(const std::string& method,
                                     const Request& request,
                                     Response* response,
                                     std::string* error_message) {
  if (response == nullptr || error_message == nullptr) {
    return false;
  }
  if (!registry_client_) {
    *error_message = "registry client is not initialized";
    return false;
  }
  std::string bytes;
  if (!request.SerializeToString(&bytes)) {
    *error_message = "failed to serialize service request";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(options_.service_timeout_ms);
  while (!stop_requested_ && std::chrono::steady_clock::now() < deadline) {
    if (!registry_client_->IsConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    eCAL::ServiceResponseVecT responses;
    registry_client_->CallWithResponse(method, bytes, responses, 100);
    for (const auto& service_response : responses) {
      if (service_response.call_state != eCAL::eCallState::executed) {
        continue;
      }
      if (!response->ParseFromString(service_response.response)) {
        *error_message = "failed to parse service response";
        return false;
      }
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  *error_message = stop_requested_
                       ? "service call cancelled: " + method
                       : "service call timed out: " + method;
  return false;
}

bool SensorStreamClient::SetCallback(const std::string& stream_name,
                                     StreamCallback callback,
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
  if (!started_) {
    *error_message = "sensor stream client must be started first";
    return false;
  }
  if (stop_requested_) {
    *error_message = "sensor stream client has been stopped";
    return false;
  }
  const auto found = streams_.find(stream_name);
  if (found == streams_.end()) {
    *error_message = "unknown sensor stream: " + stream_name;
    return false;
  }
  auto& stream = *found->second;
  if (stream.mode == DeliveryMode::kWait) {
    *error_message = "stream is already in blocking wait mode: " +
                     stream_name;
    return false;
  }
  stream.dropped_samples += stream.samples.size();
  stream.samples.clear();
  stream.mode = DeliveryMode::kCallback;
  stream.callback = std::move(callback);
  return true;
}

SampleWaitResult SensorStreamClient::WaitForSample(
    const std::string& stream_name, std::chrono::milliseconds timeout,
    OwnedStreamSample* sample, std::string* error_message) {
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
  if (!started_) {
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
  if (stream->wait_in_progress) {
    *error_message = "another thread is already waiting for stream: " +
                     stream_name;
    return SampleWaitResult::kError;
  }
  stream->mode = DeliveryMode::kWait;
  stream->wait_in_progress = true;
  const auto finish = [&stream] { stream->wait_in_progress = false; };
  const auto ready = [this, &stream] {
    return !stream->samples.empty() || !stream->terminal_error.empty() ||
           stop_requested_.load() || (monitor_ecal_ && !eCAL::Ok());
  };

  bool signaled = ready();
  if (!signaled && timeout == std::chrono::milliseconds::max()) {
    while (!(signaled = ready())) {
      condition_.wait_for(lock, std::chrono::milliseconds(50));
    }
  } else if (!signaled) {
    signaled = condition_.wait_for(lock, timeout, ready);
  }
  if (!signaled) {
    finish();
    return SampleWaitResult::kTimeout;
  }
  if (!stream->terminal_error.empty()) {
    *error_message = stream->terminal_error;
    finish();
    return SampleWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    finish();
    return SampleWaitResult::kStopped;
  }

  *sample = std::move(stream->samples.front());
  stream->samples.pop_front();
  finish();
  return SampleWaitResult::kReceived;
}

void SensorStreamClient::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    accepting_samples_ = false;
  }
  condition_.notify_all();
}

std::optional<mujoco_sim::v2::StreamDescriptor>
SensorStreamClient::descriptor(const std::string& stream_name) const {
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  if (found == streams_.end()) {
    return std::nullopt;
  }
  return found->second->descriptor;
}

std::uint64_t SensorStreamClient::dropped_samples(
    const std::string& stream_name) const {
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  return found == streams_.end() ? 0 : found->second->dropped_samples;
}

std::string SensorStreamClient::stream_diagnostic(
    const std::string& stream_name) const {
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  return found == streams_.end() ? std::string{} : found->second->diagnostic;
}

bool SensorStreamClient::IsExpectedFrame(
    const std::string& stream_name,
    const mujoco_sim::protocol::ParsedStreamFrame& frame,
    std::string* error_message) const {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  if (found == streams_.end()) {
    *error_message = "frame arrived for an unknown sensor stream";
    return false;
  }
  const auto& descriptor = found->second->descriptor;
  const auto& header = frame.header;
  if (header.header().server_name() != options_.server_name ||
      header.header().epoch() != discovery_.epoch() ||
      header.header().protocol_major() !=
          mujoco_sim::protocol::kProtocolMajor ||
      header.header().protocol_minor() !=
          mujoco_sim::protocol::kProtocolMinor) {
    *error_message = "sensor stream frame identity mismatch";
    return false;
  }
  if (header.stream_name() != stream_name ||
      header.stream_name() != descriptor.name()) {
    *error_message = "sensor stream frame name mismatch";
    return false;
  }
  if (header.stream_sequence() == 0) {
    *error_message = "sensor stream sequence must be greater than zero";
    return false;
  }
  if (header.format() != descriptor.format()) {
    *error_message = "sensor stream frame format mismatch";
    return false;
  }
  if (header.width() != descriptor.width() ||
      header.height() != descriptor.height() ||
      header.row_stride_bytes() != descriptor.row_stride_bytes() ||
      header.frame_id() != descriptor.frame_id() ||
      header.frame_convention() != descriptor.frame_convention()) {
    *error_message = "sensor stream frame shape mismatch";
    return false;
  }
  const std::uint64_t expected_payload =
      static_cast<std::uint64_t>(descriptor.row_stride_bytes()) *
      descriptor.height();
  if (header.payload_size() != expected_payload ||
      frame.payload.size() != expected_payload ||
      header.payload_size() > descriptor.maximum_payload_bytes()) {
    *error_message = "sensor stream frame payload mismatch";
    return false;
  }
  return true;
}

void SensorStreamClient::HandleStreamBytes(const std::string& stream_name,
                                           const void* data,
                                           std::size_t size) {
  if (data == nullptr || size == 0) {
    RecordDiagnostic(stream_name, "stream frame is empty");
    return;
  }
  const auto bytes = std::span(
      static_cast<const std::byte*>(data), size);
  mujoco_sim::protocol::ParsedStreamFrame frame;
  std::string error;
  if (!mujoco_sim::protocol::ParseStreamFrame(bytes, &frame, &error)) {
    RecordDiagnostic(stream_name, error);
    return;
  }
  if (!IsExpectedFrame(stream_name, frame, &error)) {
    RecordDiagnostic(stream_name, error);
    return;
  }
  HandleStreamFrame(stream_name, frame);
}

void SensorStreamClient::HandleStatusBytes(const void* data,
                                           std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v2::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size))) {
    return;
  }

  bool terminal_error_recorded = false;
  {
    std::lock_guard lock(mutex_);
    const auto& header = status.header();
    if (!accepting_samples_ || stop_requested_ ||
        header.server_name() != options_.server_name ||
        header.epoch() != discovery_.epoch() ||
        header.protocol_major() != mujoco_sim::protocol::kProtocolMajor ||
        header.protocol_minor() != mujoco_sim::protocol::kProtocolMinor) {
      return;
    }
    if (last_status_tick_id_.has_value() &&
        header.tick_id() < *last_status_tick_id_) {
      return;
    }
    last_status_tick_id_ = header.tick_id();

    if (status.state() == mujoco_sim::v2::STOPPED) {
      accepting_samples_ = false;
      stop_requested_ = true;
      terminal_error_recorded = true;
    } else if (status.state() == mujoco_sim::v2::ERROR_PAUSED) {
      const std::string error = status.message().empty()
                                    ? "server entered ERROR_PAUSED"
                                    : status.message();
      accepting_samples_ = false;
      stop_requested_ = true;
      for (auto& [_, stream] : streams_) {
        if (stream->terminal_error.empty()) {
          stream->terminal_error = error;
          stream->diagnostic = error;
        }
      }
      terminal_error_recorded = true;
    }

    if (status.state() != mujoco_sim::v2::STOPPED) {
      for (const auto& stream_status : status.streams()) {
        const auto found = streams_.find(stream_status.name());
        if (found == streams_.end() ||
            stream_status.state() != mujoco_sim::v2::STREAM_ERROR) {
          continue;
        }
        auto& stream = *found->second;
        if (!stream.terminal_error.empty()) {
          continue;
        }
        stream.terminal_error =
            stream_status.message().empty()
                ? "server marked sensor stream as STREAM_ERROR: " +
                      stream_status.name()
                : stream_status.message();
        stream.diagnostic = stream.terminal_error;
        terminal_error_recorded = true;
      }
    }
  }
  if (terminal_error_recorded) {
    condition_.notify_all();
  }
}

void SensorStreamClient::HandleStreamFrame(
    const std::string& stream_name,
    const mujoco_sim::protocol::ParsedStreamFrame& frame) {
  std::shared_ptr<StreamState> selected_stream;
  {
    std::lock_guard lock(mutex_);
    const auto found = streams_.find(stream_name);
    if (found == streams_.end()) {
      return;
    }
    selected_stream = found->second;
  }
  std::unique_lock delivery_lock(selected_stream->delivery_mutex);
  StreamCallback callback;
  bool queued = false;
  {
    std::lock_guard lock(mutex_);
    const auto found = streams_.find(stream_name);
    if (found == streams_.end() || found->second != selected_stream ||
        !accepting_samples_ || stop_requested_ ||
        !selected_stream->terminal_error.empty()) {
      return;
    }
    auto& stream = *selected_stream;
    if (frame.header.stream_sequence() <= stream.last_sequence) {
      return;
    }
    if (stream.last_sequence > 0 &&
        frame.header.stream_sequence() > stream.last_sequence + 1) {
      stream.dropped_samples +=
          frame.header.stream_sequence() - stream.last_sequence - 1;
    }
    stream.dropped_samples += frame.header.dropped_since_previous();
    stream.last_sequence = frame.header.stream_sequence();
    if (stream.mode == DeliveryMode::kCallback) {
      callback = stream.callback;
    } else {
      if (stream.samples.size() == options_.queue_capacity) {
        stream.samples.pop_front();
        ++stream.dropped_samples;
      }
      stream.samples.push_back(OwnedStreamSample{
          frame.header,
          std::vector<std::byte>(frame.payload.begin(), frame.payload.end())});
      queued = true;
    }
  }

  if (callback) {
    try {
      callback(StreamSampleView(frame.header, frame.payload));
    } catch (const std::exception& error) {
      RecordDiagnostic(stream_name,
                       std::string("stream callback threw: ") + error.what());
    } catch (...) {
      RecordDiagnostic(stream_name, "stream callback threw");
    }
  }
  if (queued) {
    condition_.notify_all();
  }
}

void SensorStreamClient::RecordDiagnostic(
    const std::string& stream_name, const std::string& diagnostic) {
  if (diagnostic.empty()) {
    return;
  }
  std::lock_guard lock(mutex_);
  const auto found = streams_.find(stream_name);
  if (found != streams_.end() && found->second->terminal_error.empty()) {
    found->second->diagnostic = diagnostic;
  }
}

SensorStreamClient::CallbackLease::CallbackLease(
    std::shared_ptr<CallbackState> state)
    : state_(std::move(state)) {
  std::lock_guard lock(state_->mutex);
  if (state_->enabled && state_->client != nullptr) {
    ++state_->active_callbacks;
    ++state_->callbacks_by_thread[std::this_thread::get_id()];
    client_ = state_->client;
  }
}

SensorStreamClient::CallbackLease::~CallbackLease() {
  if (client_ == nullptr) {
    return;
  }
  {
    std::lock_guard lock(state_->mutex);
    --state_->active_callbacks;
    const auto thread_id = std::this_thread::get_id();
    auto found = state_->callbacks_by_thread.find(thread_id);
    if (found != state_->callbacks_by_thread.end() && --found->second == 0) {
      state_->callbacks_by_thread.erase(found);
    }
  }
  state_->condition.notify_all();
}

void SensorStreamClient::StopSampleReception() {
  const auto callback_state = callback_state_;
  {
    std::lock_guard callback_lock(callback_state->mutex);
    callback_state->enabled = false;
    callback_state->client = nullptr;
  }
  callback_state->condition.notify_all();
  {
    std::lock_guard lock(mutex_);
    accepting_samples_ = false;
  }
  for (auto& [_, subscriber] : subscribers_) {
    if (subscriber) {
      subscriber->RemoveReceiveCallback();
    }
  }
  if (status_subscriber_) {
    status_subscriber_->RemoveReceiveCallback();
  }
  std::unique_lock callback_lock(callback_state->mutex);
  const auto current = callback_state->callbacks_by_thread.find(
      std::this_thread::get_id());
  const std::size_t callbacks_on_this_thread =
      current == callback_state->callbacks_by_thread.end() ? 0
                                                           : current->second;
  callback_state->condition.wait(callback_lock, [&] {
    return callback_state->active_callbacks <= callbacks_on_this_thread;
  });
  callback_lock.unlock();
  subscribers_.clear();
  status_subscriber_.reset();
}

}  // namespace mujoco_sim::adapter_sdk
