#include <mujoco_sim/adapter_sdk/observer_client.hpp>

#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>

#include "ecal_runtime.hpp"
#include "internal/observer_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

std::uint64_t ReadWord(std::string_view bytes, std::size_t offset) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[offset + index]))
             << (8U * index);
  }
  return value;
}

ResourceInfo MapResource(const mujoco_sim::v4::ResourceSlot& slot) {
  ResourceInfo result;
  result.name = slot.resource().name();
  result.scalars = {slot.scalar_offset(), slot.resource().dimension()};
  return result;
}

}  // namespace

ObservationLease::ObservationLease() = default;
ObservationLease::~ObservationLease() = default;
ObservationLease::ObservationLease(ObservationLease&&) noexcept = default;
ObservationLease& ObservationLease::operator=(ObservationLease&&) noexcept = default;
std::uint64_t ObservationLease::sequence() const noexcept {
  return data_ ? data_->header.sequence : 0;
}
std::uint64_t ObservationLease::tick_id() const noexcept {
  return data_ ? data_->header.tick_id : 0;
}
double ObservationLease::simulation_time_s() const noexcept {
  return data_ ? data_->header.simulation_time_s : 0.0;
}
std::span<const double> ObservationLease::observations() const noexcept {
  return data_ ? std::span<const double>(data_->scalars) : std::span<const double>{};
}
ObservationLease::operator bool() const noexcept {
  return data_ != nullptr;
}

ObserverClient::Impl::Impl(ObserverOptions options)
    : options_(std::move(options)), callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

ObserverClient::Impl::~Impl() {
  Stop();
  JoinHeartbeatThread();
  StopObservationReception();
  std::string ignored;
  Unregister(&ignored);
  transport_.reset();
  ecal_runtime_.reset();
}

bool ObserverClient::Impl::ValidateOptions(std::string* error) const {
  if (options_.observer_id.empty()) {
    *error = "observer_id is required";
    return false;
  }
  if (!mujoco_sim::protocol::ValidateIdentifier(
          options_.server_name, mujoco_sim::protocol::IdentifierKind::kServer, error) ||
      !mujoco_sim::protocol::ValidateIdentifier(
          options_.observer_id, mujoco_sim::protocol::IdentifierKind::kAdapter, error))
    return false;
  if (options_.observation_names.empty() || options_.session_timeout_ms == 0 ||
      options_.heartbeat_period_ms < 100 ||
      options_.heartbeat_period_ms > options_.session_timeout_ms / 3 ||
      options_.queue_capacity == 0) {
    *error = "invalid observer resources, timeout, heartbeat, or queue capacity";
    return false;
  }
  if (options_.every_n_steps.has_value() == options_.period_s.has_value()) {
    *error = "exactly one observer delivery cadence is required";
    return false;
  }
  if ((options_.every_n_steps && *options_.every_n_steps == 0) ||
      (options_.period_s && (!std::isfinite(*options_.period_s) || *options_.period_s <= 0.0))) {
    *error = "observer delivery cadence must be positive";
    return false;
  }
  return true;
}

bool ObserverClient::Impl::Start(std::string* error) {
  if (error == nullptr) return false;
  auto lifecycle_operation = lifecycle_.Acquire();
  error->clear();
  {
    std::lock_guard lock(mutex_);
    if (session_.started()) {
      *error = "observer client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error = "observer client has been stopped";
    return false;
  }
  if (!ValidateOptions(error)) return false;
  ecal_runtime_ =
      internal::AcquireEcalRuntime("mujoco-sim-observer-" + options_.observer_id, error);
  if (!ecal_runtime_) return false;
  try {
    if (!transport_) {
      transport_ = internal::MakeEcalAdapterTransport(
          options_.server_name, options_.service_timeout_ms, &stop_requested_, false);
    }
  } catch (const std::exception& exception) {
    *error = std::string("failed to create Observer transport: ") + exception.what();
    return false;
  } catch (...) {
    *error = "failed to create Observer transport: unknown exception";
    return false;
  }
  if (!Register(error) || !StartObservationReception(error) || !FinishStart(error) ||
      !StartHeartbeat(error)) {
    AbortStart();
    return false;
  }
  return true;
}

bool ObserverClient::Impl::Register(std::string* error) {
  mujoco_sim::v4::RegisterAdapterRequest request;
  request.set_adapter_id(options_.observer_id);
  request.set_role(mujoco_sim::v4::OBSERVER);
  request.set_session_timeout_ms(options_.session_timeout_ms);
  for (const auto& name : options_.observation_names)
    request.add_observation_names(name);
  if (options_.every_n_steps)
    request.mutable_delivery()->set_every_n_steps(*options_.every_n_steps);
  else
    request.mutable_delivery()->set_period_s(*options_.period_s);
  if (!transport_->Register(request, &wire_registration_, error)) return false;
  if (wire_registration_.code() != mujoco_sim::v4::OK) {
    *error = wire_registration_.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    session_.Registered({wire_registration_.session_id(), wire_registration_.epoch()});
  }
  if (!ValidateRegistration(error)) return false;
  ResetSessionData();
  return true;
}

bool ObserverClient::Impl::ValidateRegistration(std::string* error) {
  const auto& wire = wire_registration_;
  if (wire.adapter_id() != options_.observer_id || wire.role() != mujoco_sim::v4::OBSERVER ||
      wire.session_id() == 0 || wire.epoch() == 0 || wire.state() != mujoco_sim::v4::PROVISIONAL ||
      !mujoco_sim::protocol::ValidateLayoutHash(wire.layout(), error) ||
      wire.layout().command_scalar_count() != 0) {
    if (error->empty()) *error = "invalid observer registration response";
    return false;
  }
  std::vector<ResourceInfo> observations;
  for (const auto& slot : wire.layout().observations())
    observations.push_back(MapResource(slot));
  registration_.adapter_id = wire.adapter_id();
  registration_.session_id = wire.session_id();
  registration_.epoch = wire.epoch();
  registration_.layout =
      ControllerLayout(std::move(observations), {}, wire.layout().observation_scalar_count(), 0);
  layout_fingerprint_ = {ReadWord(wire.layout().layout_fingerprint(), 0),
                         ReadWord(wire.layout().layout_fingerprint(), 8)};
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

bool ObserverClient::Impl::FinishStart(std::string* error) {
  std::lock_guard lock(mutex_);
  if (stop_requested_) {
    *error = "observer start was cancelled";
    return false;
  }
  monitor_ecal_ = true;
  session_.StartCompleted();
  return true;
}

bool ObserverClient::Impl::Activate(std::string* error) {
  auto lifecycle_operation = lifecycle_.Acquire();
  if (error == nullptr) return false;
  error->clear();
  {
    std::lock_guard lock(mutex_);
    if (!session_.started() || !session_.has_session()) {
      *error = "observer client must be started before activation";
      return false;
    }
    if (session_.active()) {
      *error = "observer client has already been activated";
      return false;
    }
  }
  if (!WaitBeforeActivation(activation_delay_)) {
    *error = "observer activation was cancelled";
    return false;
  }
  internal::SessionIdentity identity;
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_ || !session_.has_session()) {
      *error = "observer activation was cancelled";
      return false;
    }
    identity = session_.identity();
    session_.BeginActivation();
  }
  mujoco_sim::v4::ActivateAdapterRequest request;
  request.set_session_id(identity.session_id);
  request.set_epoch(identity.epoch);
  mujoco_sim::v4::ActivateAdapterResponse response;
  const bool called = transport_->Activate(request, &response, error);
  if (!called || response.code() != mujoco_sim::v4::OK) {
    if (called) *error = response.message();
    std::lock_guard lock(mutex_);
    session_.ActivationRejected();
    return false;
  }
  bool cancelled = false;
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      *error = "observer activation was cancelled";
      cancelled = true;
    } else if (!session_.Activated()) {
      *error = "observer activation could not be committed locally";
      cancelled = true;
    }
  }
  if (cancelled) {
    AbortActivation(error);
    return false;
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  return true;
}

bool ObserverClient::Impl::WaitBeforeActivation(std::chrono::milliseconds delay) {
  std::unique_lock lock(mutex_);
  condition_.wait_for(lock, delay, [this] { return stop_requested_.load(); });
  return !stop_requested_;
}

ObservationWaitResult ObserverClient::Impl::WaitForObservation(std::chrono::milliseconds timeout,
                                                               ObservationLease* frame,
                                                               std::string* error) {
  if (frame == nullptr || error == nullptr || timeout.count() < 0) {
    if (error) *error = "frame, error_message, and timeout must be valid";
    return ObservationWaitResult::kError;
  }
  error->clear();
  std::shared_ptr<const ObservationLease::Data> data;
  std::unique_lock lock(mutex_);
  if (!session_.active() && terminal_error_.empty() && !stop_requested_) {
    *error = "observer client must be activated before waiting";
    return ObservationWaitResult::kError;
  }
  const auto result = observations_.Wait(
      lock, condition_, timeout,
      [this] { return stop_requested_.load() || (monitor_ecal_ && !eCAL::Ok()); },
      [this] { return terminal_error_; }, &data, error);
  if (result == internal::MailboxWaitResult::kConcurrentWait)
    *error = "another thread is already waiting for an observation";
  if (result == internal::MailboxWaitResult::kReceived) {
    ObservationLease received;
    received.data_ = std::move(data);
    *frame = std::move(received);
  }
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
}

bool ObserverClient::Impl::Unregister(std::string* error) {
  auto lifecycle_operation = lifecycle_.Acquire();
  return UnregisterSession(error);
}

bool ObserverClient::Impl::UnregisterSession(std::string* error) {
  if (error == nullptr) return false;
  error->clear();
  mujoco_sim::v4::UnregisterAdapterRequest request;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return true;
    request.set_session_id(session_.identity().session_id);
    request.set_epoch(session_.identity().epoch);
  }
  mujoco_sim::v4::UnregisterAdapterResponse response;
  if (!transport_ || !transport_->Unregister(request, &response, error)) return false;
  return CompleteUnregister(response, error);
}

bool ObserverClient::Impl::CompleteUnregister(
    const mujoco_sim::v4::UnregisterAdapterResponse& response, std::string* error) {
  if (response.code() != mujoco_sim::v4::OK && response.code() != mujoco_sim::v4::NOT_FOUND &&
      response.code() != mujoco_sim::v4::INVALID_SESSION) {
    *error = response.message();
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
}

bool ObserverClient::Impl::StartHeartbeat(std::string* error) {
  try {
    if (!StartHeartbeatThread()) {
      *error = "observer start was cancelled before heartbeat startup";
      return false;
    }
  } catch (const std::exception& exception) {
    *error = std::string("failed to start observer heartbeat: ") + exception.what();
    return false;
  } catch (...) {
    *error = "failed to start observer heartbeat: unknown exception";
    return false;
  }
  if (stop_requested_) {
    *error = "observer start was cancelled during heartbeat startup";
    return false;
  }
  return true;
}

void ObserverClient::Impl::AbortStart() {
  Stop();
  JoinHeartbeatThread();
  StopObservationReception();
  std::string ignored;
  UnregisterSession(&ignored);
}

void ObserverClient::Impl::AbortActivation(std::string* error) {
  const std::string cause = *error;
  Stop();
  JoinHeartbeatThread();
  StopObservationReception();
  std::string cleanup_error;
  if (!UnregisterSession(&cleanup_error)) {
    *error = cause + "; compensating unregister failed: " + cleanup_error;
    RecordLifecycleWarning(*error);
  }
}

ObserverClient::ObserverClient(ObserverOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}
ObserverClient::~ObserverClient() {
  impl_->Stop();
  impl_->JoinHeartbeatThread();
  impl_->StopObservationReception();
  std::string ignored;
  impl_->Unregister(&ignored);
}
bool ObserverClient::Start(std::string* error) {
  return impl_->Start(error);
}
bool ObserverClient::Activate(std::string* error) {
  return impl_->Activate(error);
}
ObservationWaitResult ObserverClient::WaitForObservation(std::chrono::milliseconds timeout,
                                                         ObservationLease* frame,
                                                         std::string* error) {
  return impl_->WaitForObservation(timeout, frame, error);
}
void ObserverClient::Stop() {
  impl_->Stop();
}
bool ObserverClient::Unregister(std::string* error) {
  return impl_->Unregister(error);
}
std::uint64_t ObserverClient::dropped_frames() const {
  return impl_->dropped_frames();
}
std::string ObserverClient::lifecycle_warning() const {
  return impl_->lifecycle_warning();
}
const AdapterRegistration& ObserverClient::registration() const {
  return impl_->registration_;
}
const ControllerLayout& ObserverClient::layout() const {
  return impl_->registration_.layout;
}

}  // namespace mujoco_sim::adapter_sdk
