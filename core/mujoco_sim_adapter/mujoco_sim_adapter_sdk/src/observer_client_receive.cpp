#include <mujoco_sim/adapter_sdk/observer_client.hpp>

#include <exception>
#include <limits>
#include <string>

#include <mujoco_sim/protocol/topics.hpp>

#include "internal/adapter_status_monitor.hpp"
#include "internal/callback_boundary.hpp"
#include "internal/observer_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {

bool ObserverClient::Impl::StartObservationReception(std::string* error) {
  try {
    {
      std::lock_guard callback_lock(callback_state_->mutex);
      callback_state_->client = this;
      callback_state_->enabled = true;
    }
    const auto state = callback_state_;
    const auto weak_self = weak_from_this();
    observation_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::ObservationTopicName(options_.server_name, options_.observer_id));
    observation_subscriber_->SetReceiveCallback(
        [state, weak_self](const auto&, const auto&, const auto& data) {
          const auto self = weak_self.lock();
          if (!self) return;
          CallbackLease callback(state);
          if (callback.client() != nullptr) {
            internal::InvokeCallbackBoundary(
                [&] { self->HandleObservationBytes(data.buffer, data.buffer_size); },
                [&](const char* message) {
                  self->SetTerminalError(std::string("observation callback failed: ") + message);
                });
          }
        });
    status_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::StatusTopicName(options_.server_name));
    status_subscriber_->SetReceiveCallback(
        [state, weak_self](const auto&, const auto&, const auto& data) {
          const auto self = weak_self.lock();
          if (!self) return;
          CallbackLease callback(state);
          if (callback.client() != nullptr) {
            internal::InvokeCallbackBoundary(
                [&] { self->HandleStatusBytes(data.buffer, data.buffer_size); },
                [&](const char* message) {
                  self->SetTerminalError(std::string("status callback failed: ") + message);
                });
          }
        });
  } catch (const std::exception& exception) {
    *error = std::string("failed to create observer endpoints: ") + exception.what();
    return false;
  }
  return true;
}

void ObserverClient::Impl::HandleObservationBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) return;
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError parse_error;
  const std::string_view bytes(static_cast<const char*>(data), size);
  if (!mujoco_sim::protocol::ParseDataFrame(bytes, &parsed, &parse_error) ||
      parsed.header.kind != mujoco_sim::protocol::DataFrameKind::kObservation ||
      parsed.header.sequence == 0)
    return;
  internal::SessionIdentity session;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    session = session_.identity();
  }
  if (parsed.header.session_id != session.session_id || parsed.header.epoch != session.epoch ||
      parsed.header.layout != layout_fingerprint_ ||
      parsed.scalar_count != registration_.layout.observation_scalar_count()) {
    return;
  }
  auto frame = std::make_shared<ObservationLease::Data>();
  frame->header = parsed.header;
  frame->scalars.resize(parsed.scalar_count);
  if (!mujoco_sim::protocol::DecodeDataFrameScalars(parsed, frame->scalars, &parse_error)) return;
  HandleObservation(std::move(frame));
}

void ObserverClient::Impl::HandleObservation(std::shared_ptr<const ObservationLease::Data> frame) {
  {
    std::lock_guard lock(mutex_);
    if (!session_.accepting_data() || stop_requested_ || frame->header.sequence <= last_sequence_)
      return;
    if (last_sequence_ > 0 && frame->header.sequence > last_sequence_ + 1)
      dropped_frames_ += frame->header.sequence - last_sequence_ - 1;
    last_sequence_ = frame->header.sequence;
    last_tick_id_ = frame->header.tick_id;
    dropped_frames_ += observations_.Push(std::move(frame), options_.queue_capacity);
  }
  condition_.notify_all();
}

void ObserverClient::Impl::HandleStatusBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    return;
  mujoco_sim::v4::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size))) return;
  internal::SessionIdentity identity;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    identity = session_.identity();
  }
  auto decision = internal::EvaluateAdapterStatus(status, options_.server_name, identity.epoch,
                                                  options_.observer_id, identity.session_id,
                                                  mujoco_sim::v4::OBSERVER, true);
  if (decision.message == "server replaced adapter session")
    decision.message = "server replaced observer session";
  if (decision.action == internal::StatusAction::kStopped)
    TransitionToStopped();
  else if (decision.action == internal::StatusAction::kError)
    SetTerminalError(decision.message);
}

void ObserverClient::Impl::SetTerminalError(const std::string& error) {
  {
    std::lock_guard lock(mutex_);
    session_.Fail();
    if (terminal_error_.empty()) terminal_error_ = error;
    if (lifecycle_warning_.empty())
      lifecycle_warning_ = error;
    else if (lifecycle_warning_.find(error) == std::string::npos)
      lifecycle_warning_ += "; " + error;
  }
  stop_requested_ = true;
  condition_.notify_all();
  heartbeat_condition_.notify_all();
}

std::uint64_t ObserverClient::Impl::dropped_frames() const {
  std::lock_guard lock(mutex_);
  return dropped_frames_;
}
std::string ObserverClient::Impl::lifecycle_warning() const {
  std::lock_guard lock(mutex_);
  return lifecycle_warning_;
}

void ObserverClient::Impl::StopObservationReception() {
  CallbackGate::Disable(callback_state_);
  if (observation_subscriber_) observation_subscriber_->RemoveReceiveCallback();
  if (status_subscriber_) status_subscriber_->RemoveReceiveCallback();
  CallbackGate::WaitForOtherCallbacks(callback_state_);
  observation_subscriber_.reset();
  status_subscriber_.reset();
}

void ObserverClient::Impl::StartHeartbeatThread() {
  std::lock_guard lock(heartbeat_thread_mutex_);
  if (!stop_requested_ && !heartbeat_thread_.joinable())
    heartbeat_thread_ = std::thread([this] { HeartbeatLoop(); });
}

void ObserverClient::Impl::JoinHeartbeatThread() {
  std::lock_guard lock(heartbeat_thread_mutex_);
  if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
}

void ObserverClient::Impl::HeartbeatLoop() {
  const auto period = std::chrono::milliseconds(options_.heartbeat_period_ms);
  std::unique_lock lock(mutex_);
  while (!stop_requested_) {
    heartbeat_condition_.wait_for(
        lock, period, [this] { return stop_requested_.load() || !session_.has_session(); });
    if (stop_requested_) break;
    if (!session_.active()) continue;
    const auto heartbeat = BuildHeartbeat();
    lock.unlock();
    transport_->PublishHeartbeat(heartbeat);
    lock.lock();
  }
}

mujoco_sim::v4::SessionHeartbeat ObserverClient::Impl::BuildHeartbeat() const {
  mujoco_sim::v4::SessionHeartbeat heartbeat;
  heartbeat.set_session_id(session_.identity().session_id);
  heartbeat.set_epoch(session_.identity().epoch);
  heartbeat.set_last_seen(last_sequence_);
  return heartbeat;
}

}  // namespace mujoco_sim::adapter_sdk
