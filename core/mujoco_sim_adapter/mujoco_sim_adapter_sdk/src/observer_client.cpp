#include <mujoco_sim/adapter_sdk/observer_client.hpp>

#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "ecal_runtime.hpp"
#include "internal/adapter_status_monitor.hpp"
#include "internal/observer_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {


bool ValidateObserverIdentityAndNames(const ObserverOptions& options,
                                      std::string* error_message) {
  if (!mujoco_sim::protocol::ValidateIdentifier(
          options.server_name,
          mujoco_sim::protocol::IdentifierKind::kServer, error_message) ||
      !mujoco_sim::protocol::ValidateIdentifier(
          options.observer_id,
          mujoco_sim::protocol::IdentifierKind::kAdapter, error_message)) {
    return false;
  }
  if (options.observation_names.empty()) {
    *error_message = "at least one observation sensor is required";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto& name : options.observation_names) {
    if (name.empty() || !names.insert(name).second) {
      *error_message = name.empty() ? "observation name must not be empty"
                                    : "duplicate observation name: " + name;
      return false;
    }
  }
  return true;
}

bool ValidateObserverTiming(const ObserverOptions& options,
                            std::string* error_message) {
  if (options.service_timeout_ms == 0 || options.keepalive_period_ms == 0) {
    *error_message = options.service_timeout_ms == 0
                         ? "service_timeout_ms must be greater than zero"
                         : "keepalive_period_ms must be greater than zero";
    return false;
  }
  if (options.timeout_ms != 0 &&
      options.keepalive_period_ms >= options.timeout_ms) {
    *error_message = "keepalive_period_ms must be less than timeout_ms";
    return false;
  }
  return true;
}

bool ValidateObserverDelivery(const ObserverOptions& options,
                              std::string* error_message) {
  if (options.queue_capacity == 0) {
    *error_message = "queue_capacity must be greater than zero";
    return false;
  }
  if (options.every_n_steps.has_value() == options.period_s.has_value()) {
    *error_message =
        "exactly one of every_n_steps or period_s must be configured";
    return false;
  }
  if (options.every_n_steps.has_value() && *options.every_n_steps == 0) {
    *error_message = "every_n_steps must be greater than zero";
    return false;
  }
  if (options.period_s.has_value() &&
      (!std::isfinite(*options.period_s) || *options.period_s <= 0.0)) {
    *error_message = "period_s must be finite and greater than zero";
    return false;
  }
  return true;
}

}  // namespace

ObserverClient::Impl::Impl(ObserverOptions options)
    : options_(std::move(options)),
      callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

ObserverClient::Impl::~Impl() {
  Stop();
  StopObservationReception();
  std::string ignored;
  Unregister(&ignored);
  transport_.reset();
  ecal_runtime_.reset();
}

bool ObserverClient::Impl::ValidateOptions(std::string* error_message) const {
  if (options_.server_name.empty()) {
    *error_message = "server_name is required";
    return false;
  }
  if (options_.observer_id.empty()) {
    *error_message = "observer_id is required";
    return false;
  }
  return ValidateObserverIdentityAndNames(options_, error_message) &&
         ValidateObserverTiming(options_, error_message) &&
         ValidateObserverDelivery(options_, error_message);
}

bool ObserverClient::Impl::Start(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  std::lock_guard session_lock(lifecycle_mutex_);
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (session_.started()) {
      *error_message = "observer client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error_message = "observer client has been stopped";
    return false;
  }
  if (!ValidateOptions(error_message)) {
    return false;
  }

  ecal_runtime_ = internal::AcquireEcalRuntime(
      "mujoco-sim-observer-" + options_.observer_id, error_message);
  if (!ecal_runtime_) {
    return false;
  }

  if (!transport_) {
    transport_ = internal::MakeEcalAdapterTransport(
        options_.server_name, options_.service_timeout_ms, &stop_requested_,
        false);
  }
  if (!Register(error_message)) {
    const std::string registration_error = *error_message;
    std::string ignored;
    UnregisterSession(&ignored);
    transport_.reset();
    ecal_runtime_.reset();
    *error_message = registration_error;
    return false;
  }

  if (!StartObservationReception(error_message)) return false;
  if (!FinishStart(error_message)) {
    transport_.reset();
    ecal_runtime_.reset();
    return false;
  }
  return StartHeartbeat(error_message);
}

bool ObserverClient::Impl::FinishStart(std::string* error_message) {
  bool cancelled = false;
  {
    std::lock_guard lock(mutex_);
    cancelled = stop_requested_;
    if (!cancelled) {
      monitor_ecal_ = true;
      session_.StartCompleted();
    }
  }
  if (!cancelled) {
    return true;
  }
  StopObservationReception();
  std::string ignored;
  UnregisterSession(&ignored);
  *error_message = "observer start was cancelled";
  return false;
}

bool ObserverClient::Impl::Register(std::string* error_message) {
  mujoco_sim::v2::RegisterAdapterRequest request;
  request.set_server_name(options_.server_name);
  request.set_adapter_id(options_.observer_id);
  request.set_role(mujoco_sim::v2::OBSERVER);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.set_timeout_ms(options_.timeout_ms);
  for (const auto& name : options_.observation_names) {
    request.add_observation_names(name);
  }
  if (options_.every_n_steps.has_value()) {
    request.mutable_delivery()->set_every_n_steps(*options_.every_n_steps);
  } else {
    request.mutable_delivery()->set_period_s(*options_.period_s);
  }

  if (!transport_->Register(request, &registration_, error_message)) {
    return false;
  }
  if (registration_.code() != mujoco_sim::v2::OK) {
    *error_message = registration_.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    session_.Registered(
        {registration_.session_id(), registration_.epoch()});
  }
  if (!ValidateRegistration(error_message)) return false;
  ResetSessionData();
  return true;
}

bool ObserverClient::Impl::ValidateRegistration(
    std::string* error_message) const {
  if (registration_.adapter_id() != options_.observer_id ||
      registration_.role() != mujoco_sim::v2::OBSERVER ||
      registration_.session_id() == 0 ||
      registration_.epoch() == 0 ||
      registration_.layout().commands_size() != 0 ||
      registration_.state() != mujoco_sim::v2::PROVISIONAL ||
      !mujoco_sim::protocol::ValidateLayoutHash(registration_.layout(),
                                                error_message)) {
    if (!error_message->empty()) return false;
    *error_message = "invalid observer registration response";
    return false;
  }
  if (registration_.layout().observations_size() !=
      static_cast<int>(options_.observation_names.size())) {
    *error_message = "observer registration layout does not match request";
    return false;
  }
  for (int i = 0; i < registration_.layout().observations_size(); ++i) {
    const auto& descriptor = registration_.layout().observations(i);
    if (descriptor.name() != options_.observation_names[i] ||
        descriptor.type() != mujoco_sim::v2::OBSERVATION_SENSOR ||
        descriptor.dimension() == 0) {
      *error_message = "observer registration layout does not match request";
      return false;
    }
  }

  return true;
}

void ObserverClient::Impl::ResetSessionData() {
  std::lock_guard lock(mutex_);
  observations_.Clear();
  last_sequence_ = 0;
  last_tick_id_ = 0;
  dropped_frames_ = 0;
  terminal_error_.clear();
  lifecycle_warning_.clear();
}

bool ObserverClient::Impl::Activate(std::string* error_message) {
  std::lock_guard session_lock(lifecycle_mutex_);
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (!session_.started() || !session_.has_session()) {
      *error_message = "observer client must be started before activation";
      return false;
    }
    if (session_.active()) {
      *error_message = "observer client has already been activated";
      return false;
    }
    if (stop_requested_) {
      *error_message = "observer client has been stopped";
      return false;
    }
  }
  if (!WaitBeforeActivation(activation_delay_)) {
    *error_message = "observer activation was cancelled";
    return false;
  }

  mujoco_sim::v2::ActivateAdapterRequest request;
  request.set_adapter_id(options_.observer_id);
  request.set_role(mujoco_sim::v2::OBSERVER);
  request.set_session_id(session_.identity().session_id);
  request.set_epoch(session_.identity().epoch);

  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      *error_message = "observer activation was cancelled";
      return false;
    }
    session_.BeginActivation();
  }
  mujoco_sim::v2::ActivateAdapterResponse response;
  const bool called = transport_->Activate(request, &response, error_message);
  if (!called || response.code() != mujoco_sim::v2::OK ||
      response.state() != mujoco_sim::v2::ACTIVE) {
    if (called) {
      *error_message = response.message().empty()
                           ? "observer activation was rejected"
                           : response.message();
    }
    std::lock_guard lock(mutex_);
    session_.ActivationRejected();
    observations_.Clear();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      observations_.Clear();
      *error_message = "observer activation was cancelled";
      return false;
    }
    session_.Activated();
  }
  heartbeat_condition_.notify_all();
  condition_.notify_all();
  return true;
}

bool ObserverClient::Impl::WaitBeforeActivation(std::chrono::milliseconds delay) {
  std::unique_lock lock(mutex_);
  heartbeat_condition_.wait_for(
      lock, delay, [this] { return stop_requested_.load(); });
  return !stop_requested_;
}

ObservationWaitResult ObserverClient::Impl::WaitForObservation(
    std::chrono::milliseconds timeout,
    mujoco_sim::v2::ObservationFrame* frame, std::string* error_message) {
  if (frame == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "frame and error_message must not be null";
    }
    return ObservationWaitResult::kError;
  }
  error_message->clear();
  if (timeout.count() < 0) {
    *error_message = "observation wait timeout must not be negative";
    return ObservationWaitResult::kError;
  }

  std::unique_lock lock(mutex_);
  if (!terminal_error_.empty()) {
    *error_message = terminal_error_;
    return ObservationWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    return ObservationWaitResult::kStopped;
  }
  if (!session_.started() || !session_.active()) {
    *error_message = "observer client must be activated before waiting";
    return ObservationWaitResult::kError;
  }
  if (observations_.wait_in_progress()) {
    *error_message = "another thread is already waiting for an observation";
    return ObservationWaitResult::kError;
  }
  const auto result = observations_.Wait(
      lock, condition_, timeout,
      [this] {
        return stop_requested_.load() || (monitor_ecal_ && !eCAL::Ok());
      },
      [this] { return terminal_error_; }, frame, error_message);
  return internal::ToPublicWaitResult<ObservationWaitResult>(result);
}

void ObserverClient::Impl::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    session_.Stop();
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  JoinHeartbeatThread();
}

bool ObserverClient::Impl::Unregister(std::string* error_message) {
  std::lock_guard session_lock(lifecycle_mutex_);
  return UnregisterSession(error_message);
}

bool ObserverClient::Impl::UnregisterSession(std::string* error_message) {
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
    request.set_adapter_id(options_.observer_id);
    request.set_role(mujoco_sim::v2::OBSERVER);
    request.set_session_id(session_.identity().session_id);
    request.set_epoch(session_.identity().epoch);
  }

  mujoco_sim::v2::UnregisterAdapterResponse response;
  if (!transport_ || !transport_->Unregister(request, &response, error_message)) {
    return false;
  }
  return CompleteUnregister(response, error_message);
}

bool ObserverClient::Impl::CompleteUnregister(
    const mujoco_sim::v2::UnregisterAdapterResponse& response,
    std::string* error_message) {
  if (response.code() != mujoco_sim::v2::OK &&
      response.code() != mujoco_sim::v2::NOT_FOUND &&
      response.code() != mujoco_sim::v2::INVALID_SESSION) {
    *error_message = response.message();
    return false;
  }
  TransitionToStopped();
  return true;
}

void ObserverClient::Impl::MarkStoppedLocked() {
  stop_requested_ = true;
  session_.ReleaseSession();
  observations_.Clear();
}

void ObserverClient::Impl::TransitionToStopped() {
  {
    std::lock_guard lock(mutex_);
    MarkStoppedLocked();
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  JoinHeartbeatThread();
}


}  // namespace mujoco_sim::adapter_sdk
