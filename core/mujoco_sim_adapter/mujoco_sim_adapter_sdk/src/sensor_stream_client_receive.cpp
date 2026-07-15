#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>

#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>

#include "internal/sensor_stream_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

bool HasExpectedStreamIdentity(
    const mujoco_sim::v3::StreamFrameHeader& header,
    const SensorStreamOptions& options, std::uint64_t epoch,
    std::string* error_message) {
  (void)options;
  if (header.epoch() == epoch) {
    return true;
  }
  *error_message = "sensor stream frame identity mismatch";
  return false;
}

bool HasExpectedStreamShape(
    const mujoco_sim::v3::StreamFrameHeader& header,
    const mujoco_sim::v3::StreamDescriptor& descriptor,
    const std::string& stream_name, std::string* error_message) {
  if (header.stream_name() != stream_name ||
      header.stream_name() != descriptor.name()) {
    *error_message = "sensor stream frame name mismatch";
    return false;
  }
  if (header.stream_sequence() == 0 || header.format() != descriptor.format()) {
    *error_message = header.stream_sequence() == 0
                         ? "sensor stream sequence must be greater than zero"
                         : "sensor stream frame format mismatch";
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
  return true;
}

bool HasExpectedStreamPayload(
    const mujoco_sim::v3::StreamFrameHeader& header,
    const mujoco_sim::v3::StreamDescriptor& descriptor,
    std::size_t payload_size, std::string* error_message) {
  const std::uint64_t expected =
      static_cast<std::uint64_t>(descriptor.row_stride_bytes()) *
      descriptor.height();
  if (header.payload_size() == expected && payload_size == expected &&
      header.payload_size() <= descriptor.maximum_payload_bytes()) {
    return true;
  }
  *error_message = "sensor stream frame payload mismatch";
  return false;
}

}  // namespace

bool SensorStreamClient::Impl::IsExpectedFrame(
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
  return HasExpectedStreamIdentity(frame.header, options_, discovery_.epoch(),
                                   error_message) &&
         HasExpectedStreamShape(frame.header, descriptor, stream_name,
                                error_message) &&
         HasExpectedStreamPayload(frame.header, descriptor, frame.payload.size(),
                                  error_message);
}

void SensorStreamClient::Impl::HandleStreamBytes(const std::string& stream_name,
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

void SensorStreamClient::Impl::HandleStatusBytes(const void* data,
                                           std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v3::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size))) {
    return;
  }

  bool terminal_error_recorded = false;
  {
    std::lock_guard lock(mutex_);
    if (!CanAcceptStatusLocked(status)) return;
    last_status_tick_id_ = status.tick_id();
    terminal_error_recorded = ApplyGlobalStatusLocked(status);
    if (status.state() != mujoco_sim::v3::STOPPED) {
      terminal_error_recorded =
          ApplyStreamErrorsLocked(status) || terminal_error_recorded;
    }
  }
  if (terminal_error_recorded) {
    condition_.notify_all();
  }
}

bool SensorStreamClient::Impl::CanAcceptStatusLocked(
    const mujoco_sim::v3::ServerStatus& status) const {
  const bool expected =
      phase_ == Phase::kActive && !stop_requested_ &&
      status.epoch() == discovery_.epoch();
  return expected &&
         (!last_status_tick_id_.has_value() ||
          status.tick_id() >= *last_status_tick_id_);
}

bool SensorStreamClient::Impl::ApplyGlobalStatusLocked(
    const mujoco_sim::v3::ServerStatus& status) {
  if (status.state() == mujoco_sim::v3::STOPPED) {
    phase_ = Phase::kStopped;
    stop_requested_ = true;
    return true;
  }
  if (status.state() != mujoco_sim::v3::ERROR_PAUSED) return false;

  const std::string error = status.message().empty()
                                ? "server entered ERROR_PAUSED"
                                : status.message();
  phase_ = Phase::kStopped;
  stop_requested_ = true;
  for (auto& [_, stream] : streams_) {
    if (stream->terminal_error.empty()) {
      stream->terminal_error = error;
      stream->diagnostic = error;
    }
  }
  return true;
}

bool SensorStreamClient::Impl::ApplyStreamErrorsLocked(
    const mujoco_sim::v3::ServerStatus& status) {
  bool changed = false;
  for (const auto& stream_status : status.streams()) {
    const auto found = streams_.find(stream_status.name());
    if (found == streams_.end() ||
        stream_status.state() != mujoco_sim::v3::STREAM_ERROR ||
        !found->second->terminal_error.empty()) {
      continue;
    }
    auto& stream = *found->second;
    stream.terminal_error =
        stream_status.message().empty()
            ? "server marked sensor stream as STREAM_ERROR: " +
                  stream_status.name()
            : stream_status.message();
    stream.diagnostic = stream.terminal_error;
    changed = true;
  }
  return changed;
}

void SensorStreamClient::Impl::HandleStreamFrame(
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
        phase_ != Phase::kActive || stop_requested_ ||
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
      stream.dropped_samples += stream.samples.Push(
          OwnedStreamSample{
              frame.header,
              std::vector<std::byte>(frame.payload.begin(), frame.payload.end())},
          options_.queue_capacity);
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

void SensorStreamClient::Impl::RecordDiagnostic(
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

void SensorStreamClient::Impl::StopSampleReception() {
  const auto callback_state = callback_state_;
  CallbackGate::Disable(callback_state);
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) phase_ = Phase::kStopped;
  }
  for (auto& [_, subscriber] : subscribers_) {
    if (subscriber) {
      subscriber->RemoveReceiveCallback();
    }
  }
  if (status_subscriber_) {
    status_subscriber_->RemoveReceiveCallback();
  }
  CallbackGate::WaitForOtherCallbacks(callback_state);
  subscribers_.clear();
  status_subscriber_.reset();
}

SensorStreamClient::SensorStreamClient(SensorStreamOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}

SensorStreamClient::~SensorStreamClient() {
  impl_->Stop();
  impl_->StopSampleReception();
}

bool SensorStreamClient::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

bool SensorStreamClient::SetCallback(const std::string& stream_name,
                                     StreamCallback callback,
                                     std::string* error_message) {
  return impl_->SetCallback(stream_name, std::move(callback), error_message);
}

SampleWaitResult SensorStreamClient::WaitForSample(
    const std::string& stream_name, std::chrono::milliseconds timeout,
    OwnedStreamSample* sample, std::string* error_message) {
  return impl_->WaitForSample(stream_name, timeout, sample, error_message);
}

void SensorStreamClient::Stop() { impl_->Stop(); }

std::optional<mujoco_sim::v3::StreamDescriptor>
SensorStreamClient::descriptor(const std::string& stream_name) const {
  return impl_->descriptor(stream_name);
}

std::uint64_t SensorStreamClient::dropped_samples(
    const std::string& stream_name) const {
  return impl_->dropped_samples(stream_name);
}

std::string SensorStreamClient::stream_diagnostic(
    const std::string& stream_name) const {
  return impl_->stream_diagnostic(stream_name);
}

const mujoco_sim::v3::GetSensorStreamsResponse&
SensorStreamClient::discovery() const {
  return impl_->discovery_;
}

}  // namespace mujoco_sim::adapter_sdk
