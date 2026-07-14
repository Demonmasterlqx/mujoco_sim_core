#include <mujoco_sim/adapter_sdk/observer_client.hpp>

#include <chrono>
#include <limits>
#include <string>
#include <thread>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "internal/adapter_status_monitor.hpp"
#include "internal/callback_boundary.hpp"
#include "internal/observer_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

std::uint64_t NowUnixNanos() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

}  // namespace

bool ObserverClient::Impl::IsExpectedObservation(
    const mujoco_sim::v2::ObservationFrame& frame) const {
  std::lock_guard lock(mutex_);
  if (!session_.has_session()) return false;
  const auto& header = frame.header();
  if (frame.adapter_id() != options_.observer_id || frame.sequence() == 0 ||
      header.server_name() != options_.server_name ||
      header.session_id() != session_.identity().session_id ||
      header.epoch() != session_.identity().epoch ||
      header.protocol_major() != mujoco_sim::protocol::kProtocolMajor ||
      header.protocol_minor() != mujoco_sim::protocol::kProtocolMinor ||
      frame.observations_size() !=
          registration_.layout().observations_size()) {
    return false;
  }
  for (int i = 0; i < frame.observations_size(); ++i) {
    const auto& block = frame.observations(i);
    const auto& descriptor = registration_.layout().observations(i);
    if (block.name() != descriptor.name() ||
        block.values_size() != static_cast<int>(descriptor.dimension())) {
      return false;
    }
  }
  return true;
}

void ObserverClient::Impl::HandleObservationBytes(const void* data,
                                            std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v2::ObservationFrame frame;
  if (!frame.ParseFromArray(data, static_cast<int>(size)) ||
      !IsExpectedObservation(frame)) {
    return;
  }
  HandleObservation(frame);
}

void ObserverClient::Impl::HandleObservation(
    const mujoco_sim::v2::ObservationFrame& frame) {
  bool queued = false;
  {
    std::lock_guard lock(mutex_);
    if (!session_.accepting_data() || stop_requested_ ||
        frame.sequence() <= last_sequence_) {
      return;
    }
    if (last_sequence_ > 0 && frame.sequence() > last_sequence_ + 1) {
      dropped_frames_ += frame.sequence() - last_sequence_ - 1;
    }
    last_sequence_ = frame.sequence();
    last_tick_id_ = frame.header().tick_id();
    dropped_frames_ += observations_.Push(frame, options_.queue_capacity);
    queued = true;
  }
  if (queued) {
    condition_.notify_all();
  }
}

void ObserverClient::Impl::HandleStatusBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v2::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size))) return;
  internal::SessionIdentity identity;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return;
    identity = session_.identity();
  }
  auto decision = internal::EvaluateAdapterStatus(
      status, options_.server_name, identity.epoch, options_.observer_id,
      identity.session_id, mujoco_sim::v2::OBSERVER, true);
  if (decision.message == "server replaced adapter session") {
    decision.message = "server replaced observer session";
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
    if (terminal_error_.empty()) {
      terminal_error_ = error;
    }
    if (lifecycle_warning_.empty()) {
      lifecycle_warning_ = error;
    } else if (lifecycle_warning_.find(error) == std::string::npos) {
      lifecycle_warning_ += "; " + error;
    }
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
  if (observation_subscriber_) {
    observation_subscriber_->RemoveReceiveCallback();
  }
  if (status_subscriber_) {
    status_subscriber_->RemoveReceiveCallback();
  }
  CallbackGate::WaitForOtherCallbacks(callback_state_);
  observation_subscriber_.reset();
  status_subscriber_.reset();
}

void ObserverClient::Impl::StartHeartbeatThread() {
  std::lock_guard thread_lock(heartbeat_thread_mutex_);
  if (!stop_requested_ && !heartbeat_thread_.joinable()) {
    heartbeat_thread_ = std::thread([this] { HeartbeatLoop(); });
  }
}

void ObserverClient::Impl::JoinHeartbeatThread() {
  std::lock_guard thread_lock(heartbeat_thread_mutex_);
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
}

void ObserverClient::Impl::HeartbeatLoop() {
  std::unique_lock lock(mutex_);
  while (!stop_requested_) {
    heartbeat_condition_.wait(lock, [this] {
      return stop_requested_.load() ||
             (session_.active() && session_.has_session());
    });
    if (stop_requested_) {
      break;
    }
    lock.unlock();
    transport_->PublishHeartbeat(BuildHeartbeat());
    lock.lock();
    heartbeat_condition_.wait_for(
        lock, std::chrono::milliseconds(options_.keepalive_period_ms),
        [this] {
          return stop_requested_.load() || !session_.active() ||
                 !session_.has_session();
        });
  }
}

mujoco_sim::v2::AdapterHeartbeat ObserverClient::Impl::BuildHeartbeat() const {
  mujoco_sim::v2::AdapterHeartbeat heartbeat;
  std::lock_guard lock(mutex_);
  auto* header = heartbeat.mutable_header();
  header->set_server_name(options_.server_name);
  header->set_session_id(session_.identity().session_id);
  header->set_epoch(session_.identity().epoch);
  header->set_tick_id(last_tick_id_);
  header->set_timestamp_unix_nanos(NowUnixNanos());
  header->set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  header->set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  heartbeat.set_adapter_id(options_.observer_id);
  heartbeat.set_role(mujoco_sim::v2::OBSERVER);
  heartbeat.set_last_observation_sequence(last_sequence_);
  return heartbeat;
}

ObserverClient::ObserverClient(ObserverOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}

ObserverClient::~ObserverClient() {
  impl_->Stop();
  impl_->StopObservationReception();
  std::string ignored;
  impl_->Unregister(&ignored);
}

bool ObserverClient::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

bool ObserverClient::Activate(std::string* error_message) {
  return impl_->Activate(error_message);
}

ObservationWaitResult ObserverClient::WaitForObservation(
    std::chrono::milliseconds timeout,
    mujoco_sim::v2::ObservationFrame* frame, std::string* error_message) {
  return impl_->WaitForObservation(timeout, frame, error_message);
}

void ObserverClient::Stop() { impl_->Stop(); }

bool ObserverClient::Unregister(std::string* error_message) {
  return impl_->Unregister(error_message);
}

std::uint64_t ObserverClient::dropped_frames() const {
  return impl_->dropped_frames();
}

std::string ObserverClient::lifecycle_warning() const {
  return impl_->lifecycle_warning();
}

const mujoco_sim::v2::RegisterAdapterResponse&
ObserverClient::registration() const {
  return impl_->registration_;
}

bool ObserverClient::Impl::StartHeartbeat(std::string* error_message) {
  try {
    StartHeartbeatThread();
  } catch (const std::exception& error) {
    *error_message = std::string("failed to start observer keepalive: ") +
                     error.what();
    AbortStart();
    return false;
  }
  return true;
}

void ObserverClient::Impl::AbortStart() {
  Stop();
  StopObservationReception();
  std::string ignored;
  UnregisterSession(&ignored);
  transport_.reset();
  ecal_runtime_.reset();
}

bool ObserverClient::Impl::StartObservationReception(
    std::string* error_message) {
  try {
    {
      std::lock_guard callback_lock(callback_state_->mutex);
      callback_state_->client = this;
      callback_state_->enabled = true;
    }
    const auto callback_state = callback_state_;
    const auto weak_self = weak_from_this();
    observation_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::ObservationTopicName(options_.server_name,
                                                    options_.observer_id));
    observation_subscriber_->SetReceiveCallback(
        [callback_state, weak_self](const eCAL::STopicId&,
                         const eCAL::SDataTypeInformation&,
                         const eCAL::SReceiveCallbackData& data) {
          const auto self = weak_self.lock();
          if (!self) return;
          CallbackLease callback(callback_state);
          if (callback.client() != nullptr) {
            internal::InvokeCallbackBoundary(
                [&] {
                  self->HandleObservationBytes(data.buffer, data.buffer_size);
                },
                [&](const char* error) {
                  self->SetTerminalError(
                      std::string("observation callback failed: ") + error);
                });
          }
        });
    status_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::StatusTopicName(options_.server_name));
    status_subscriber_->SetReceiveCallback(
        [callback_state, weak_self](const eCAL::STopicId&,
                         const eCAL::SDataTypeInformation&,
                         const eCAL::SReceiveCallbackData& data) {
          const auto self = weak_self.lock();
          if (!self) return;
          CallbackLease callback(callback_state);
          if (callback.client() != nullptr) {
            internal::InvokeCallbackBoundary(
                [&] { self->HandleStatusBytes(data.buffer, data.buffer_size); },
                [&](const char* error) {
                  self->SetTerminalError(
                      std::string("status callback failed: ") + error);
                });
          }
        });
  } catch (const std::exception& error) {
    *error_message = std::string("failed to create observer endpoints: ") +
                     error.what();
    AbortStart();
    return false;
  }
  return true;
}

}  // namespace mujoco_sim::adapter_sdk
