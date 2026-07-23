#include <mujoco_sim/adapter_sdk/controller_client.hpp>

#include <exception>
#include <utility>

#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>

#include "ecal_runtime.hpp"
#include "internal/controller_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

std::uint64_t ReadFingerprintWord(std::string_view bytes, std::size_t offset) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8 && offset + index < bytes.size(); ++index) {
    value |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[offset + index]))
             << (8U * index);
  }
  return value;
}

ResourceInfo MapResource(const mujoco_sim::v4::ResourceSlot& slot) {
  return {slot.resource().name(),
          {slot.scalar_offset(), slot.resource().dimension()},
          slot.resource().has_ctrl_range(),
          slot.resource().ctrl_min(),
          slot.resource().ctrl_max()};
}

bool UnregisterCompleted(mujoco_sim::v4::ErrorCode code) {
  return code == mujoco_sim::v4::OK || code == mujoco_sim::v4::NOT_FOUND ||
         code == mujoco_sim::v4::INVALID_SESSION;
}

}  // namespace

StateLease::StateLease() = default;
StateLease::~StateLease() = default;
StateLease::StateLease(StateLease&&) noexcept = default;
StateLease& StateLease::operator=(StateLease&&) noexcept = default;

std::uint64_t StateLease::sequence() const noexcept {
  return data_ ? data_->header.sequence : 0;
}

std::uint64_t StateLease::tick_id() const noexcept {
  return data_ ? data_->header.tick_id : 0;
}

double StateLease::simulation_time_s() const noexcept {
  return data_ ? data_->header.simulation_time_s : 0.0;
}

std::span<const double> StateLease::observations() const noexcept {
  return data_ ? std::span<const double>(data_->scalars) : std::span<const double>{};
}

StateLease::operator bool() const noexcept {
  return data_ != nullptr;
}

ControllerClient::Impl::Impl(ControllerOptions controller_options)
    : options(std::move(controller_options)), callback_state(std::make_shared<CallbackState>()) {
  callback_state->client = this;
}

ControllerClient::Impl::~Impl() {
  Stop();
  JoinPublisher();
  StopReception();
  std::string ignored;
  Unregister(&ignored);
  transport.reset();
  runtime.reset();
}

bool ControllerClient::Impl::Validate(std::string* error_message) {
  if (error_message == nullptr) return false;
  if (options.adapter_id.empty() || options.command_names.empty() ||
      options.service_timeout_ms == 0) {
    *error_message = "adapter_id, command resources, and service timeout are required";
    return false;
  }
  return mujoco_sim::protocol::ValidateIdentifier(
             options.server_name, mujoco_sim::protocol::IdentifierKind::kServer, error_message) &&
         mujoco_sim::protocol::ValidateIdentifier(
             options.adapter_id, mujoco_sim::protocol::IdentifierKind::kAdapter, error_message);
}

mujoco_sim::v4::RegisterAdapterRequest ControllerClient::Impl::RegistrationRequest() const {
  mujoco_sim::v4::RegisterAdapterRequest request;
  request.set_adapter_id(options.adapter_id);
  request.set_role(mujoco_sim::v4::CONTROLLER);
  for (const auto& name : options.observation_names)
    request.add_observation_names(name);
  for (const auto& name : options.command_names)
    request.add_command_names(name);
  return request;
}

bool ControllerClient::Impl::AcceptRegistration(std::string* error_message) {
  if (wire.adapter_id() != options.adapter_id) {
    *error_message = "registration adapter ID mismatch";
    return false;
  }
  if (wire.role() != mujoco_sim::v4::CONTROLLER) {
    *error_message = "registration role mismatch";
    return false;
  }
  if (wire.session_id() == 0 || wire.epoch() == 0) {
    *error_message = "registration identity is invalid";
    return false;
  }
  if (wire.state() != mujoco_sim::v4::PROVISIONAL) {
    *error_message = "registration state is not PROVISIONAL";
    return false;
  }
  if (!wire.has_controller_policy()) {
    *error_message = "registration is missing Controller policy";
    return false;
  }
  if (!mujoco_sim::protocol::ValidateLayoutHash(wire.layout(), error_message)) return false;

  std::vector<ResourceInfo> observations;
  std::vector<ResourceInfo> commands;
  for (const auto& slot : wire.layout().observations())
    observations.push_back(MapResource(slot));
  for (const auto& slot : wire.layout().commands())
    commands.push_back(MapResource(slot));
  registration = {wire.adapter_id(), wire.session_id(), wire.epoch(),
                  ControllerLayout(std::move(observations), std::move(commands),
                                   wire.layout().observation_scalar_count(),
                                   wire.layout().command_scalar_count())};
  fingerprint = {ReadFingerprintWord(wire.layout().layout_fingerprint(), 0),
                 ReadFingerprintWord(wire.layout().layout_fingerprint(), 8)};
  keepalive = std::chrono::milliseconds(wire.controller_policy().command_keepalive_ms());
  if (keepalive.count() <= 0) {
    *error_message = "invalid command keepalive policy";
    return false;
  }
  return true;
}

bool ControllerClient::Impl::Start(std::string* error_message) {
  if (error_message == nullptr) return false;
  auto lifecycle_operation = lifecycle.Acquire();
  error_message->clear();
  if (started || remote_registered || stop) {
    *error_message = "controller client cannot be started";
    return false;
  }
  if (!Validate(error_message)) return false;
  runtime =
      internal::AcquireEcalRuntime("mujoco-sim-controller-" + options.adapter_id, error_message);
  if (!runtime) return false;
  try {
    if (!transport) {
      transport = internal::MakeEcalAdapterTransport(options.server_name,
                                                     options.service_timeout_ms, &stop, true);
    }
  } catch (const std::exception& exception) {
    *error_message = std::string("failed to create Controller transport: ") + exception.what();
    return false;
  } catch (...) {
    *error_message = "failed to create Controller transport: unknown exception";
    return false;
  }
  if (!transport->Register(RegistrationRequest(), &wire, error_message) ||
      wire.code() != mujoco_sim::v4::OK) {
    if (error_message->empty()) *error_message = wire.message();
    return false;
  }
  remote_registered = true;
  unregistered = false;
  return FinishStart(error_message);
}

bool ControllerClient::Impl::FinishStart(std::string* error_message) {
  if (!AcceptRegistration(error_message)) {
    CompensateRegistration(error_message);
    return false;
  }
  if (stop) {
    *error_message = "controller start was cancelled";
    CompensateRegistration(error_message);
    return false;
  }
  if (!StartReception(error_message)) {
    StopReception();
    CompensateRegistration(error_message);
    return false;
  }
  {
    std::lock_guard lock(mutex);
    if (stop) {
      *error_message = "controller start was cancelled";
    } else {
      started = true;
    }
  }
  if (!error_message->empty()) {
    StopReception();
    CompensateRegistration(error_message);
    return false;
  }
  return true;
}

void ControllerClient::Impl::CompensateRegistration(std::string* error_message) {
  const std::string cause = *error_message;
  mujoco_sim::v4::UnregisterAdapterRequest request;
  request.set_session_id(wire.session_id());
  request.set_epoch(wire.epoch());
  mujoco_sim::v4::UnregisterAdapterResponse response;
  std::string cleanup_error;
  if (transport->Unregister(request, &response, &cleanup_error) &&
      UnregisterCompleted(response.code())) {
    remote_registered = false;
    unregistered = true;
    return;
  }
  if (cleanup_error.empty()) cleanup_error = response.message();
  *error_message = cause + "; compensating unregister failed: " + cleanup_error;
}

void ControllerClient::Impl::AbortActivation(std::string* error_message) {
  StopLocal();
  CompensateRegistration(error_message);
  StopReception();
}

bool ControllerClient::Impl::Activate(std::string* error_message) {
  if (error_message == nullptr) return false;
  auto lifecycle_operation = lifecycle.Acquire();
  error_message->clear();
  {
    std::lock_guard lock(mutex);
    if (!started || active || stop) {
      *error_message = "controller client is not ready to activate";
      return false;
    }
  }
  mujoco_sim::v4::ActivateAdapterRequest request;
  request.set_session_id(registration.session_id);
  request.set_epoch(registration.epoch);
  mujoco_sim::v4::ActivateAdapterResponse response;
  if (!transport->Activate(request, &response, error_message) ||
      response.code() != mujoco_sim::v4::OK ||
      response.state() != mujoco_sim::v4::ACTIVE_NO_COMMAND) {
    if (error_message->empty()) *error_message = response.message();
    return false;
  }

  bool cancelled = false;
  {
    std::lock_guard lock(mutex);
    if (stop) {
      *error_message = "controller activation was cancelled";
      cancelled = true;
    } else {
      active = true;
    }
  }
  if (cancelled) {
    AbortActivation(error_message);
    return false;
  }
  try {
    if (before_publisher_start) before_publisher_start();
    publisher = std::thread([this] { RunPublishLoop(); });
  } catch (const std::exception& exception) {
    *error_message = std::string("failed to start Controller publisher: ") + exception.what();
    AbortActivation(error_message);
    return false;
  } catch (...) {
    *error_message = "failed to start Controller publisher: unknown exception";
    AbortActivation(error_message);
    return false;
  }
  return true;
}

void ControllerClient::Impl::StopLocal() {
  std::lock_guard lock(mutex);
  active = false;
  stop = true;
  condition.notify_all();
}

void ControllerClient::Impl::Stop() {
  StopLocal();
  auto lifecycle_operation = lifecycle.Acquire();
  JoinPublisher();
  StopReception();
}

void ControllerClient::Impl::JoinPublisher() {
  if (publisher.joinable() && publisher.get_id() != std::this_thread::get_id()) publisher.join();
}

bool ControllerClient::Impl::Unregister(std::string* error_message) {
  if (error_message == nullptr) return false;
  auto lifecycle_operation = lifecycle.Acquire();
  error_message->clear();
  if (!remote_registered || unregistered) return true;
  mujoco_sim::v4::UnregisterAdapterRequest request;
  request.set_session_id(wire.session_id());
  request.set_epoch(wire.epoch());
  mujoco_sim::v4::UnregisterAdapterResponse response;
  if (!transport->Unregister(request, &response, error_message) ||
      !UnregisterCompleted(response.code())) {
    if (error_message->empty()) *error_message = response.message();
    return false;
  }
  remote_registered = false;
  unregistered = true;
  StopLocal();
  JoinPublisher();
  StopReception();
  return true;
}

ControllerClient::ControllerClient(ControllerOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}

ControllerClient::~ControllerClient() = default;

bool ControllerClient::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

bool ControllerClient::Activate(std::string* error_message) {
  return impl_->Activate(error_message);
}

StateReadResult ControllerClient::ReadLatestState(StateLease* state, std::string* error_message) {
  return impl_->Read(state, error_message, false, std::chrono::milliseconds(0));
}

StateReadResult ControllerClient::WaitForNewState(std::chrono::milliseconds timeout,
                                                  StateLease* state, std::string* error_message) {
  return impl_->Read(state, error_message, true, timeout);
}

bool ControllerClient::SetCommand(std::span<const double> values, std::string* error_message) {
  return impl_->SetCommand(values, error_message);
}

bool ControllerClient::ReportFault(FaultCode code, std::string_view message,
                                   std::string* error_message) {
  return impl_->ReportFault(code, message, error_message);
}

void ControllerClient::Stop() {
  impl_->Stop();
}

bool ControllerClient::Unregister(std::string* error_message) {
  return impl_->Unregister(error_message);
}

const AdapterRegistration& ControllerClient::registration() const {
  return impl_->registration;
}

const ControllerLayout& ControllerClient::layout() const {
  return impl_->registration.layout;
}

}  // namespace mujoco_sim::adapter_sdk
