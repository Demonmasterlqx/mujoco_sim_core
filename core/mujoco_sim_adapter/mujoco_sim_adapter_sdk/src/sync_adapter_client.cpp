#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "ecal_runtime.hpp"
#include "internal/adapter_status_monitor.hpp"
#include "internal/callback_boundary.hpp"
#include "internal/sync_adapter_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

using mujoco_sim::v3::OK;

std::uint64_t ReadWord(std::string_view bytes, std::size_t offset) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(
                 static_cast<unsigned char>(bytes[offset + index]))
             << (8U * index);
  }
  return value;
}

ResourceInfo MapResource(const mujoco_sim::v3::ResourceSlot& slot) {
  ResourceInfo result;
  result.name = slot.resource().name();
  result.scalars = {slot.scalar_offset(), slot.resource().dimension()};
  result.has_command_range = slot.resource().has_ctrl_range();
  result.command_min = slot.resource().ctrl_min();
  result.command_max = slot.resource().ctrl_max();
  return result;
}

}  // namespace

TickLease::TickLease() = default;
TickLease::~TickLease() = default;
TickLease::TickLease(TickLease&&) noexcept = default;
TickLease& TickLease::operator=(TickLease&&) noexcept = default;
std::uint64_t TickLease::tick_id() const noexcept {
  return data_ ? data_->header.tick_id : 0;
}
double TickLease::simulation_time_s() const noexcept {
  return data_ ? data_->header.simulation_time_s : 0.0;
}
bool TickLease::recovery_proof() const noexcept {
  return data_ &&
         (data_->header.flags & mujoco_sim::protocol::kDataFrameRecoveryProof);
}
std::span<const double> TickLease::observations() const noexcept {
  return data_ ? std::span<const double>(data_->scalars) : std::span<const double>{};
}
TickLease::operator bool() const noexcept { return data_ != nullptr; }

SyncAdapterClient::Impl::Impl(AdapterOptions options)
    : options_(std::move(options)), callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

SyncAdapterClient::Impl::~Impl() {
  Stop();
  JoinHeartbeatThread();
  StopTickReception();
  std::string ignored;
  Unregister(&ignored);
  transport_.reset();
  ecal_runtime_.reset();
}

bool SyncAdapterClient::Impl::ValidateOptions(std::string* error) const {
  if (options_.adapter_id.empty()) {
    *error = "adapter_id is required";
    return false;
  }
  if (!mujoco_sim::protocol::ValidateIdentifier(
          options_.server_name, mujoco_sim::protocol::IdentifierKind::kServer,
          error) ||
      !mujoco_sim::protocol::ValidateIdentifier(
          options_.adapter_id, mujoco_sim::protocol::IdentifierKind::kAdapter,
          error)) {
    return false;
  }
  if (options_.command_names.empty()) {
    *error = "at least one command actuator is required";
    return false;
  }
  if (options_.commit_timeout_ms == 0 || options_.session_timeout_ms == 0 ||
      options_.heartbeat_period_ms == 0 ||
      options_.heartbeat_period_ms > options_.session_timeout_ms / 3) {
    *error = "timeouts must be positive and heartbeat_period_ms must not exceed session_timeout_ms / 3";
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::Start(std::string* error) {
  if (error == nullptr) return false;
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  error->clear();
  {
    std::lock_guard lock(mutex_);
    if (session_.started()) {
      *error = "adapter client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error = "adapter client has been stopped";
    return false;
  }
  if (!ValidateOptions(error)) return false;
  ecal_runtime_ = internal::AcquireEcalRuntime(
      "mujoco-sim-adapter-" + options_.adapter_id, error);
  if (!ecal_runtime_) return false;
  if (!transport_) {
    transport_ = internal::MakeEcalAdapterTransport(
        options_.server_name, options_.service_timeout_ms, &stop_requested_, true);
  }
  if (!Register(error)) {
    std::string ignored;
    UnregisterSession(&ignored);
    return false;
  }
  if (!StartTickReception(error)) {
    StopTickReception();
    std::string ignored;
    UnregisterSession(&ignored);
    return false;
  }
  if (!FinishStart(error)) return false;
  try {
    StartHeartbeatThread();
  } catch (const std::exception& exception) {
    *error = std::string("failed to start controller heartbeat: ") + exception.what();
    Stop();
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::StartTickReception(std::string* error) {
  try {
    {
      std::lock_guard callback_lock(callback_state_->mutex);
      callback_state_->client = this;
      callback_state_->enabled = true;
    }
    const auto state = callback_state_;
    const auto weak_self = weak_from_this();
    tick_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::TickTopicName(options_.server_name, options_.adapter_id));
    tick_subscriber_->SetReceiveCallback(
        [state, weak_self](const auto&, const auto&, const auto& data) {
          const auto self = weak_self.lock();
          if (!self) return;
          CallbackLease callback(state);
          if (callback.client() != nullptr) {
            internal::InvokeCallbackBoundary(
                [&] { self->HandleTickBytes(data.buffer, data.buffer_size); },
                [&](const char* message) {
                  self->SetTerminalError(std::string("Tick callback failed: ") + message);
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
    *error = std::string("failed to create controller endpoints: ") + exception.what();
    return false;
  }
  return true;
}

bool SyncAdapterClient::Impl::Register(std::string* error) {
  mujoco_sim::v3::RegisterAdapterRequest request;
  request.set_adapter_id(options_.adapter_id);
  request.set_role(mujoco_sim::v3::CONTROLLER);
  request.set_commit_timeout_ms(options_.commit_timeout_ms);
  request.set_session_timeout_ms(options_.session_timeout_ms);
  for (const auto& name : options_.observation_names) request.add_observation_names(name);
  for (const auto& name : options_.command_names) request.add_command_names(name);
  if (!transport_->Register(request, &wire_registration_, error)) return false;
  if (wire_registration_.code() != OK) {
    *error = wire_registration_.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    session_.Registered({wire_registration_.session_id(), wire_registration_.epoch()});
  }
  if (!ValidateRegistration(error)) return false;
  std::lock_guard lock(mutex_);
  queued_tick_.reset();
  outstanding_tick_.reset();
  committed_tick_.reset();
  cached_commit_.reset();
  terminal_error_.clear();
  lifecycle_warning_.clear();
  last_commit_publish_ = std::chrono::steady_clock::now();
  last_heartbeat_publish_ = last_commit_publish_;
  return true;
}

bool SyncAdapterClient::Impl::ValidateRegistration(std::string* error) {
  const auto& wire = wire_registration_;
  if (wire.adapter_id() != options_.adapter_id ||
      wire.role() != mujoco_sim::v3::CONTROLLER || wire.session_id() == 0 ||
      wire.epoch() == 0 || wire.state() != mujoco_sim::v3::PROVISIONAL ||
      !mujoco_sim::protocol::ValidateLayoutHash(wire.layout(), error)) {
    if (error->empty()) *error = "invalid controller registration response";
    return false;
  }
  std::vector<ResourceInfo> observations;
  std::vector<ResourceInfo> commands;
  for (const auto& slot : wire.layout().observations()) observations.push_back(MapResource(slot));
  for (const auto& slot : wire.layout().commands()) commands.push_back(MapResource(slot));
  registration_.adapter_id = wire.adapter_id();
  registration_.session_id = wire.session_id();
  registration_.epoch = wire.epoch();
  registration_.layout = ControllerLayout(
      std::move(observations), std::move(commands),
      wire.layout().observation_scalar_count(), wire.layout().command_scalar_count());
  layout_fingerprint_ = {ReadWord(wire.layout().layout_fingerprint(), 0),
                         ReadWord(wire.layout().layout_fingerprint(), 8)};
  return true;
}

bool SyncAdapterClient::Impl::FinishStart(std::string* error) {
  bool cancelled;
  {
    std::lock_guard lock(mutex_);
    cancelled = stop_requested_;
    if (!cancelled) {
      monitor_ecal_ = true;
      session_.StartCompleted();
    }
  }
  if (!cancelled) return true;
  StopTickReception();
  std::string ignored;
  UnregisterSession(&ignored);
  *error = "adapter start was cancelled";
  return false;
}

bool SyncAdapterClient::Impl::Activate(std::string* error) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  if (error == nullptr) return false;
  error->clear();
  {
    std::lock_guard lock(mutex_);
    if (!session_.started() || !session_.has_session()) {
      *error = "adapter client must be started before activation";
      return false;
    }
    if (session_.active()) {
      *error = "adapter client has already been activated";
      return false;
    }
    if (stop_requested_) {
      *error = "adapter client has been stopped";
      return false;
    }
  }
  if (!WaitBeforeActivation(activation_delay_)) {
    *error = "adapter activation was cancelled";
    return false;
  }
  mujoco_sim::v3::ActivateAdapterRequest request;
  request.set_session_id(session_.identity().session_id);
  request.set_epoch(session_.identity().epoch);
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      *error = "adapter activation was cancelled";
      return false;
    }
    session_.BeginActivation();
  }
  mujoco_sim::v3::ActivateAdapterResponse response;
  const bool called = transport_->Activate(request, &response, error);
  if (!called || response.code() != OK) {
    if (called) *error = response.message();
    std::lock_guard lock(mutex_);
    session_.ActivationRejected();
    queued_tick_.reset();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      queued_tick_.reset();
      *error = "adapter activation was cancelled";
      return false;
    }
    session_.Activated();
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  return true;
}

bool SyncAdapterClient::Impl::WaitBeforeActivation(std::chrono::milliseconds delay) {
  std::unique_lock lock(mutex_);
  condition_.wait_for(lock, delay, [this] { return stop_requested_.load(); });
  return !stop_requested_;
}

void SyncAdapterClient::Impl::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    session_.Stop();
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
}

bool SyncAdapterClient::Impl::Unregister(std::string* error) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  return UnregisterSession(error);
}

bool SyncAdapterClient::Impl::UnregisterSession(std::string* error) {
  if (error == nullptr) return false;
  error->clear();
  mujoco_sim::v3::UnregisterAdapterRequest request;
  {
    std::lock_guard lock(mutex_);
    if (!session_.has_session()) return true;
    request.set_session_id(session_.identity().session_id);
    request.set_epoch(session_.identity().epoch);
  }
  mujoco_sim::v3::UnregisterAdapterResponse response;
  if (!transport_ || !transport_->Unregister(request, &response, error)) return false;
  return CompleteUnregister(response, error);
}

bool SyncAdapterClient::Impl::CompleteUnregister(
    const mujoco_sim::v3::UnregisterAdapterResponse& response, std::string* error) {
  if (response.code() != OK && response.code() != mujoco_sim::v3::NOT_FOUND &&
      response.code() != mujoco_sim::v3::INVALID_SESSION) {
    *error = response.message();
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
  heartbeat_condition_.notify_all();
}

void SyncAdapterClient::Impl::AddLifecycleWarning(const std::string& warning) {
  if (warning.empty()) return;
  std::lock_guard lock(mutex_);
  if (lifecycle_warning_.empty()) lifecycle_warning_ = warning;
  else if (lifecycle_warning_.find(warning) == std::string::npos)
    lifecycle_warning_ += "; " + warning;
}

void SyncAdapterClient::Impl::SetTerminalError(const std::string& error) {
  {
    std::lock_guard lock(mutex_);
    session_.Fail();
    if (terminal_error_.empty()) terminal_error_ = error;
    if (lifecycle_warning_.empty()) lifecycle_warning_ = error;
    else if (lifecycle_warning_.find(error) == std::string::npos)
      lifecycle_warning_ += "; " + error;
  }
  stop_requested_ = true;
  condition_.notify_all();
  heartbeat_condition_.notify_all();
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
  if (tick_subscriber_) tick_subscriber_->RemoveReceiveCallback();
  if (status_subscriber_) status_subscriber_->RemoveReceiveCallback();
  CallbackGate::WaitForOtherCallbacks(callback_state_);
  tick_subscriber_.reset();
  status_subscriber_.reset();
}

SyncAdapterClient::SyncAdapterClient(AdapterOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}
SyncAdapterClient::~SyncAdapterClient() {
  impl_->Stop();
  impl_->JoinHeartbeatThread();
  impl_->StopTickReception();
  std::string ignored;
  impl_->Unregister(&ignored);
}
bool SyncAdapterClient::Start(std::string* error) { return impl_->Start(error); }
bool SyncAdapterClient::Activate(std::string* error) { return impl_->Activate(error); }
TickWaitResult SyncAdapterClient::WaitForTick(
    std::chrono::milliseconds timeout, TickLease* tick, std::string* error) {
  return impl_->WaitForTick(timeout, tick, error);
}
bool SyncAdapterClient::Commit(const TickLease& tick,
                               std::span<const double> commands,
                               std::string* error) {
  return impl_->Commit(tick, commands, error);
}
bool SyncAdapterClient::FailTick(const TickLease& tick, FaultCode code,
                                 std::string_view message, std::string* error) {
  return impl_->FailTick(tick, code, message, error);
}
void SyncAdapterClient::Stop() { impl_->Stop(); }
bool SyncAdapterClient::Unregister(std::string* error) { return impl_->Unregister(error); }
std::string SyncAdapterClient::lifecycle_warning() const { return impl_->lifecycle_warning(); }
std::uint64_t SyncAdapterClient::layout_hash_mismatches() const {
  return impl_->layout_hash_mismatches();
}
const AdapterRegistration& SyncAdapterClient::registration() const {
  return impl_->registration_;
}
const ControllerLayout& SyncAdapterClient::layout() const {
  return impl_->registration_.layout;
}

}  // namespace mujoco_sim::adapter_sdk
