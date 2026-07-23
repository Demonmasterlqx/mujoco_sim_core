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
        [state, weak_self](const auto&, const auto&, const auto& data) noexcept {
          internal::InvokeCallbackBoundary(
              [&] {
                const auto self = weak_self.lock();
                if (!self) return;
                CallbackLease callback(state);
                if (callback.client() != nullptr) {
                  self->HandleObservationBytes(data.buffer, data.buffer_size);
                }
              },
              [&](const char* message) {
                if (const auto self = weak_self.lock()) {
                  self->SetTerminalError(std::string("observation callback failed: ") + message);
                }
              });
        });
    status_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::StatusTopicName(options_.server_name));
    status_subscriber_->SetReceiveCallback(
        [state, weak_self](const auto&, const auto&, const auto& data) noexcept {
          internal::InvokeCallbackBoundary(
              [&] {
                const auto self = weak_self.lock();
                if (!self) return;
                CallbackLease callback(state);
                if (callback.client() != nullptr) {
                  self->HandleStatusBytes(data.buffer, data.buffer_size);
                }
              },
              [&](const char* message) {
                if (const auto self = weak_self.lock()) {
                  self->SetTerminalError(std::string("status callback failed: ") + message);
                }
              });
        });
  } catch (const std::exception& exception) {
    *error = std::string("failed to create observer endpoints: ") + exception.what();
    return false;
  } catch (...) {
    *error = "failed to create observer endpoints: unknown exception";
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
  bool session_was_seen = false;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    identity = session_.identity();
    session_was_seen = seen_status_;
  }
  auto decision = internal::EvaluateAdapterStatus(status, identity.epoch, options_.observer_id,
                                                  identity.session_id, mujoco_sim::v4::OBSERVER,
                                                  true, session_was_seen);
  if (decision.matched_session) {
    std::lock_guard lock(mutex_);
    seen_status_ = true;
  }
  if (decision.message == "server replaced adapter session") {
    decision.message = "server replaced observer session";
  } else if (decision.message == "adapter session disappeared from ServerStatus") {
    decision.message = "observer session disappeared from ServerStatus";
  } else if (decision.message.starts_with("server marked adapter as ")) {
    decision.message.replace(14, 7, "observer");
  }
  if (decision.action == internal::StatusAction::kStopped) {
    TransitionToStopped();
  } else if (decision.action == internal::StatusAction::kError) {
    SetTerminalError(decision.message);
  }
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

void ObserverClient::Impl::RecordLifecycleWarning(const std::string& warning) {
  std::lock_guard lock(mutex_);
  if (lifecycle_warning_.empty())
    lifecycle_warning_ = warning;
  else if (lifecycle_warning_.find(warning) == std::string::npos)
    lifecycle_warning_ += "; " + warning;
}

std::uint64_t ObserverClient::Impl::dropped_frames() const {
  std::lock_guard lock(mutex_);
  return dropped_frames_;
}
std::string ObserverClient::Impl::lifecycle_warning() const {
  std::lock_guard lock(mutex_);
  return lifecycle_warning_;
}

void ObserverClient::Impl::StopObservationReception() noexcept {
  CallbackGate::Disable(callback_state_);
  const auto remove_callback = [this](auto& subscriber, const char* endpoint) {
    if (!subscriber) return;
    internal::InvokeCallbackBoundary([&] { subscriber->RemoveReceiveCallback(); },
                                     [this, endpoint](const char* message) {
                                       RecordLifecycleWarning(std::string("failed to remove ") +
                                                              endpoint +
                                                              " receive callback: " + message);
                                     });
  };
  remove_callback(observation_subscriber_, "observation");
  remove_callback(status_subscriber_, "status");
  CallbackGate::WaitForOtherCallbacks(callback_state_);
  observation_subscriber_.reset();
  status_subscriber_.reset();
}

bool ObserverClient::Impl::StartHeartbeatThread() {
  std::lock_guard lock(heartbeat_thread_mutex_);
  if (stop_requested_) return false;
  if (!heartbeat_thread_.joinable())
    heartbeat_thread_ = std::thread([this] { RunHeartbeatLoop(); });
  return true;
}

void ObserverClient::Impl::JoinHeartbeatThread() {
  std::thread heartbeat;
  {
    std::lock_guard lock(heartbeat_thread_mutex_);
    if (heartbeat_thread_.joinable()) heartbeat = std::move(heartbeat_thread_);
  }
  if (heartbeat.joinable()) heartbeat.join();
}

void ObserverClient::Impl::RunHeartbeatLoop() noexcept {
  internal::InvokeCallbackBoundary(
      [this] { HeartbeatLoop(); },
      [this](const char* message) {
        SetTerminalError(std::string("observer heartbeat thread failed: ") + message);
      });
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
    const auto result = transport_->PublishHeartbeat(heartbeat);
    if (!result.error.empty()) {
      SetTerminalError("observer heartbeat publish failed: " + result.error);
      return;
    }
    if (!result.sent) {
      RecordLifecycleWarning("observer heartbeat publish failed; retrying");
    }
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
