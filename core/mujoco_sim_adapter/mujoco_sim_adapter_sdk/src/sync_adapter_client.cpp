#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "ecal_runtime.hpp"
#include "internal/callback_boundary.hpp"
#include "internal/adapter_status_monitor.hpp"
#include "internal/sync_adapter_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

using mujoco_sim::v2::OK;

}  // namespace

SyncAdapterClient::Impl::Impl(AdapterOptions options)
    : options_(std::move(options)),
      callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

SyncAdapterClient::Impl::~Impl() {
  Stop();
  StopTickReception();
  std::string ignored;
  Unregister(&ignored);
  transport_.reset();
  ecal_runtime_.reset();
}

bool SyncAdapterClient::Impl::ValidateOptions(
    std::string* error_message) const {
  if (options_.adapter_id.empty()) {
    *error_message = "adapter_id is required";
    return false;
  }
  if (!mujoco_sim::protocol::ValidateIdentifier(
          options_.server_name,
          mujoco_sim::protocol::IdentifierKind::kServer, error_message) ||
      !mujoco_sim::protocol::ValidateIdentifier(
          options_.adapter_id,
          mujoco_sim::protocol::IdentifierKind::kAdapter, error_message)) {
    return false;
  }
  if (options_.command_names.empty()) {
    *error_message = "at least one command actuator is required";
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::Start(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  std::lock_guard session_lock(lifecycle_mutex_);
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (session_.started()) {
      *error_message = "adapter client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error_message = "adapter client has been stopped";
    return false;
  }
  if (!ValidateOptions(error_message)) return false;
  ecal_runtime_ = internal::AcquireEcalRuntime(
      "mujoco-sim-adapter-" + options_.adapter_id, error_message);
  if (!ecal_runtime_) {
    return false;
  }

  if (!transport_) {
    transport_ = internal::MakeEcalAdapterTransport(
        options_.server_name, options_.service_timeout_ms, &stop_requested_,
        true);
  }
  if (!Register(error_message)) {
    std::string ignored;
    UnregisterSession(&ignored);
    return false;
  }

  if (!StartTickReception(error_message)) {
    StopTickReception();
    std::string ignored;
    UnregisterSession(&ignored);
    return false;
  }
  return FinishStart(error_message);
}

bool SyncAdapterClient::Impl::StartTickReception(
    std::string* error_message) {
  try {
    {
    std::lock_guard callback_lock(callback_state_->mutex);
    callback_state_->client = this;
    callback_state_->enabled = true;
    }
    const auto callback_state = callback_state_;
    const auto weak_self = weak_from_this();
    tick_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::TickTopicName(options_.server_name,
                                            options_.adapter_id));
    tick_subscriber_->SetReceiveCallback(
        [callback_state, weak_self](const eCAL::STopicId&,
                       const eCAL::SDataTypeInformation&,
                       const eCAL::SReceiveCallbackData& data) {
          const auto self = weak_self.lock();
          if (!self) return;
          CallbackLease callback(callback_state);
          if (callback.client() != nullptr) {
            internal::InvokeCallbackBoundary(
                [&] { self->HandleTickBytes(data.buffer, data.buffer_size); },
                [&](const char* error) {
                  self->SetTerminalError(
                      std::string("Tick callback failed: ") + error);
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
    *error_message = std::string("failed to create controller endpoints: ") +
                     error.what();
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::FinishStart(std::string* error_message) {
  bool cancelled = false;
  {
    std::lock_guard lock(mutex_);
    cancelled = stop_requested_;
    if (!cancelled) {
      monitor_ecal_ = true;
      session_.StartCompleted();
    }
  }
  if (cancelled) {
    StopTickReception();
    std::string ignored;
    UnregisterSession(&ignored);
    *error_message = "adapter start was cancelled";
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::Register(std::string* error_message) {
  mujoco_sim::v2::RegisterAdapterRequest request;
  request.set_server_name(options_.server_name);
  request.set_adapter_id(options_.adapter_id);
  request.set_role(mujoco_sim::v2::CONTROLLER);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.set_timeout_ms(options_.timeout_ms);
  for (const auto& name : options_.observation_names) {
    request.add_observation_names(name);
  }
  for (const auto& name : options_.command_names) {
    request.add_command_names(name);
  }

  if (!transport_->Register(request, &registration_, error_message)) {
    return false;
  }
  if (registration_.code() != OK) {
    *error_message = registration_.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    session_.Registered(
        {registration_.session_id(), registration_.epoch()});
  }
  if (registration_.adapter_id() != options_.adapter_id ||
      registration_.role() != mujoco_sim::v2::CONTROLLER ||
      registration_.session_id() == 0 || registration_.epoch() == 0 ||
      registration_.state() != mujoco_sim::v2::PROVISIONAL ||
      !mujoco_sim::protocol::ValidateLayoutHash(registration_.layout(),
                                                error_message)) {
    if (error_message->empty()) {
      *error_message = "invalid controller registration response";
    }
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    queued_tick_.reset();
    outstanding_tick_.reset();
    committed_tick_.reset();
    cached_commit_.reset();
    terminal_error_.clear();
    lifecycle_warning_.clear();
  }
  return true;
}

bool SyncAdapterClient::Impl::Activate(std::string* error_message) {
  std::lock_guard session_lock(lifecycle_mutex_);
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (!session_.started() || !session_.has_session()) {
      *error_message = "adapter client must be started before activation";
      return false;
    }
    if (session_.active()) {
      *error_message = "adapter client has already been activated";
      return false;
    }
    if (stop_requested_) {
      *error_message = "adapter client has been stopped";
      return false;
    }
  }
  if (!WaitBeforeActivation(activation_delay_)) {
    *error_message = "adapter activation was cancelled";
    return false;
  }

  mujoco_sim::v2::ActivateAdapterRequest request;
  request.set_adapter_id(options_.adapter_id);
  request.set_role(mujoco_sim::v2::CONTROLLER);
  request.set_session_id(session_.identity().session_id);
  request.set_epoch(session_.identity().epoch);

  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      *error_message = "adapter activation was cancelled";
      return false;
    }
    session_.BeginActivation();
  }
  mujoco_sim::v2::ActivateAdapterResponse response;
  const bool called = transport_->Activate(request, &response, error_message);
  if (!called || response.code() != OK) {
    if (called) {
      *error_message = response.message();
    }
    std::lock_guard lock(mutex_);
    session_.ActivationRejected();
    queued_tick_.reset();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      queued_tick_.reset();
      *error_message = "adapter activation was cancelled";
      return false;
    }
    session_.Activated();
  }
  condition_.notify_all();
  return true;
}

bool SyncAdapterClient::Impl::WaitBeforeActivation(std::chrono::milliseconds delay) {
  std::unique_lock lock(mutex_);
  condition_.wait_for(lock, delay,
                      [this] { return stop_requested_.load(); });
  return !stop_requested_;
}

void SyncAdapterClient::Impl::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    session_.Stop();
  }
  condition_.notify_all();
}

bool SyncAdapterClient::Impl::Unregister(std::string* error_message) {
  std::lock_guard session_lock(lifecycle_mutex_);
  return UnregisterSession(error_message);
}

bool SyncAdapterClient::Impl::UnregisterSession(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  mujoco_sim::v2::UnregisterAdapterRequest request;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) {
      return true;
    }
    request.set_adapter_id(options_.adapter_id);
    request.set_role(mujoco_sim::v2::CONTROLLER);
    request.set_session_id(session_.identity().session_id);
    request.set_epoch(session_.identity().epoch);
  }

  mujoco_sim::v2::UnregisterAdapterResponse response;
  if (!transport_ || !transport_->Unregister(request, &response, error_message)) {
    return false;
  }
  return CompleteUnregister(response, error_message);
}

bool SyncAdapterClient::Impl::CompleteUnregister(
    const mujoco_sim::v2::UnregisterAdapterResponse& response,
    std::string* error_message) {
  if (response.code() != OK && response.code() != mujoco_sim::v2::NOT_FOUND &&
      response.code() != mujoco_sim::v2::INVALID_SESSION) {
    *error_message = response.message();
    return false;
  }
  TransitionToStopped();
  return true;
}

void SyncAdapterClient::Impl::MarkStoppedLocked() {
  stop_requested_ = true;
  session_.ReleaseSession();
  queued_tick_.reset();
  outstanding_tick_.reset();
}

void SyncAdapterClient::Impl::TransitionToStopped() {
  {
    std::lock_guard lock(mutex_);
    MarkStoppedLocked();
  }
  condition_.notify_all();
}

void SyncAdapterClient::Impl::AddLifecycleWarning(const std::string& warning) {
  if (warning.empty()) {
    return;
  }
  std::lock_guard lock(mutex_);
  if (lifecycle_warning_.empty()) {
    lifecycle_warning_ = warning;
  } else if (lifecycle_warning_.find(warning) == std::string::npos) {
    lifecycle_warning_ += "; " + warning;
  }
}

void SyncAdapterClient::Impl::SetTerminalError(const std::string& error) {
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
}

std::string SyncAdapterClient::Impl::lifecycle_warning() const {
  std::lock_guard lock(mutex_);
  return lifecycle_warning_;
}

std::uint64_t SyncAdapterClient::Impl::layout_hash_mismatches() const {
  std::lock_guard lock(mutex_);
  return layout_hash_mismatches_;
}

void SyncAdapterClient::Impl::StopTickReception() {
  CallbackGate::Disable(callback_state_);
  if (tick_subscriber_) {
    tick_subscriber_->RemoveReceiveCallback();
  }
  if (status_subscriber_) {
    status_subscriber_->RemoveReceiveCallback();
  }
  CallbackGate::WaitForOtherCallbacks(callback_state_);
  tick_subscriber_.reset();
  status_subscriber_.reset();
}

SyncAdapterClient::SyncAdapterClient(AdapterOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}

SyncAdapterClient::~SyncAdapterClient() {
  impl_->Stop();
  impl_->StopTickReception();
  std::string ignored;
  impl_->Unregister(&ignored);
}

bool SyncAdapterClient::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

bool SyncAdapterClient::Activate(std::string* error_message) {
  return impl_->Activate(error_message);
}

TickWaitResult SyncAdapterClient::WaitForTick(
    std::chrono::milliseconds timeout, mujoco_sim::v2::Tick* tick,
    std::string* error_message) {
  return impl_->WaitForTick(timeout, tick, error_message);
}

bool SyncAdapterClient::Commit(const mujoco_sim::v2::Tick& tick,
                               const std::vector<CommandValue>& commands,
                               std::string* error_message) {
  return impl_->Commit(tick, commands, error_message);
}

void SyncAdapterClient::Stop() { impl_->Stop(); }

bool SyncAdapterClient::Unregister(std::string* error_message) {
  return impl_->Unregister(error_message);
}

std::string SyncAdapterClient::lifecycle_warning() const {
  return impl_->lifecycle_warning();
}

std::uint64_t SyncAdapterClient::layout_hash_mismatches() const {
  return impl_->layout_hash_mismatches();
}

const mujoco_sim::v2::RegisterAdapterResponse&
SyncAdapterClient::registration() const {
  return impl_->registration_;
}

}  // namespace mujoco_sim::adapter_sdk
