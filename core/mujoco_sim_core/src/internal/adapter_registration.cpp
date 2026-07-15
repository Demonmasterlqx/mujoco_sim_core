#include "simulation_core_impl.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <utility>

#include "domain_support.hpp"

namespace mujoco_sim::core {
namespace {

using mujoco_sim::core::ALREADY_EXISTS;
using mujoco_sim::core::ERROR_PAUSED;
using mujoco_sim::core::FAILED_PRECONDITION;
using mujoco_sim::core::INVALID_ARGUMENT;
using mujoco_sim::core::NOT_FOUND;
using mujoco_sim::core::OBSERVER;
using mujoco_sim::core::OK;
using mujoco_sim::core::PROVISIONAL;
using mujoco_sim::core::RESOURCE_CONFLICT;
using mujoco_sim::core::TIMED_OUT;
using mujoco_sim::core::VERSION_MISMATCH;
using internal::WireState;

bool HasDuplicates(const std::vector<std::string>& names,
                   std::string* duplicate) {
  std::set<std::string> seen;
  for (const auto& name : names) {
    if (!seen.insert(name).second) {
      if (duplicate != nullptr) {
        *duplicate = name;
      }
      return true;
    }
  }
  return false;
}

void CopyDescriptor(const mujoco_sim::core::ResourceDescriptor& from,
                    mujoco_sim::core::ResourceDescriptor* to) {
  *to = from;
}

template <typename Response>
Response ErrorResponse(mujoco_sim::core::ErrorCode code,
                       const std::string& message) {
  Response response;
  response.set_code(code);
  response.set_message(message);
  return response;
}

std::optional<mujoco_sim::core::RegisterAdapterResponse>
ValidateObserverDelivery(const ObservationDelivery& delivery) {
  if (delivery.has_every_n_steps() && delivery.every_n_steps() == 0) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "every_n_steps must be >= 1");
  }
  if (delivery.has_period_s() &&
      (!std::isfinite(delivery.period_s()) || delivery.period_s() <= 0.0)) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "period_s must be finite and > 0");
  }
  if (delivery.kind == DeliveryKind::kUnset) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "observer delivery cadence is required");
  }
  return std::nullopt;
}

}  // namespace

std::optional<mujoco_sim::core::RegisterAdapterResponse>
SimulationCore::Impl::ValidateRegistrationRequest(
    const mujoco_sim::core::RegisterAdapterRequest& request) const {
  if (request.role() != mujoco_sim::core::CONTROLLER &&
      request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  if (!IsModelLoaded()) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "model is not loaded");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::core::STOPPED) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting registrations");
  }
  std::string identifier_error;
  if (request.adapter_id().empty() ||
      !internal::ValidateAdapterId(request.adapter_id(), &identifier_error)) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, request.adapter_id().empty()
                              ? "adapter_id is required"
                              : identifier_error);
  }
  return std::nullopt;
}

std::optional<mujoco_sim::core::RegisterAdapterResponse>
SimulationCore::Impl::ValidateControllerRequest(
    const mujoco_sim::core::RegisterAdapterRequest& request,
    Controller** existing, bool* replacing) const {
  if (request.command_names().empty()) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "at least one command actuator is required");
  }
  if (request.delivery().kind != DeliveryKind::kUnset) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "controller adapters cannot set delivery");
  }
  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate) ||
      HasDuplicates(request.command_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }
  if (adapter_registry_->HasObserver(request.adapter_id())) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        ALREADY_EXISTS, "adapter_id is already registered as an observer");
  }
  *existing = adapter_registry_->FindController(request.adapter_id());
  *replacing = *existing != nullptr &&
               WireState((*existing)->phase) == TIMED_OUT &&
               options_.allow_timed_out_adapter_replacement;
  if (*existing != nullptr && !*replacing) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        ALREADY_EXISTS,
        WireState((*existing)->phase) == TIMED_OUT
            ? "timed-out controller replacement is disabled"
            : "adapter_id is already registered");
  }
  return std::nullopt;
}

std::optional<mujoco_sim::core::RegisterAdapterResponse>
SimulationCore::Impl::PopulateControllerLayout(
    const mujoco_sim::core::RegisterAdapterRequest& request, bool replacing,
    Controller* controller) const {
  for (const auto& name : request.observation_names()) {
    const auto* descriptor = resource_catalog_->FindObservation(name);
    if (descriptor == nullptr) {
      return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
          NOT_FOUND, "unknown observation sensor: " + name);
    }
    CopyDescriptor(*descriptor, controller->layout.add_observations());
  }
  for (const auto& name : request.command_names()) {
    const auto* descriptor = resource_catalog_->FindCommand(name);
    if (descriptor == nullptr) {
      return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
          NOT_FOUND, "unknown command actuator: " + name);
    }
    const std::string* owner = adapter_registry_->ActuatorOwner(name);
    if (owner != nullptr && (!replacing || *owner != request.adapter_id())) {
      return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
          RESOURCE_CONFLICT,
          "actuator already owned by controller: " + *owner);
    }
    CopyDescriptor(*descriptor, controller->layout.add_commands());
  }
  internal::FinalizeLayout(&controller->layout);
  return std::nullopt;
}

mujoco_sim::core::RegisterAdapterResponse
SimulationCore::Impl::RegisterControlAdapter(
    const mujoco_sim::core::RegisterAdapterRequest& request) {
  Controller* existing = nullptr;
  bool replacing = false;
  if (auto error = ValidateControllerRequest(request, &existing, &replacing)) {
    return std::move(*error);
  }

  Controller controller;
  controller.id = request.adapter_id();
  controller.session_id = adapter_registry_->AllocateSession();
  controller.epoch = epoch_;
  controller.commit_timeout_ms =
      request.commit_timeout_ms() == 0 ? options_.default_commit_timeout_ms
                                       : request.commit_timeout_ms();
  controller.session_timeout_ms =
      request.session_timeout_ms() == 0 ? options_.default_session_timeout_ms
                                        : request.session_timeout_ms();
  controller.registered_at = std::chrono::steady_clock::now();
  controller.last_presence_at = controller.registered_at;
  controller.state_since_tick = current_tick_id_;
  if (auto error = PopulateControllerLayout(request, replacing, &controller)) {
    return std::move(*error);
  }

  if (replacing) {
    ZeroControllerCommands(*existing);
    ReleaseControllerResources(*existing);
    InvalidateIssuedTick(existing->id);
    barrier_coordinator_->RemoveController(existing->id);
    adapter_registry_->EraseController(existing->id);
  }
  for (const auto& command : controller.layout.commands()) {
    adapter_registry_->ClaimActuator(command.name(), controller.id);
  }

  mujoco_sim::core::RegisterAdapterResponse response;
  response.set_code(OK);
  response.set_adapter_id(controller.id);
  response.set_role(mujoco_sim::core::CONTROLLER);
  response.set_session_id(controller.session_id);
  response.set_epoch(controller.epoch);
  response.set_state(PROVISIONAL);
  response.set_message(replacing ? "replaced timed-out controller session"
                                 : "");
  *response.mutable_layout() = controller.layout;
  adapter_registry_->InsertController(std::move(controller));
  return response;
}

std::optional<mujoco_sim::core::RegisterAdapterResponse>
SimulationCore::Impl::ValidateObserverRequest(
    const mujoco_sim::core::RegisterAdapterRequest& request,
    Observer** existing, bool* replacing) const {
  if (request.observation_names().empty()) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "at least one observation sensor is required");
  }
  if (!request.command_names().empty()) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "observer adapters cannot claim actuators");
  }
  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }
  if (auto error = ValidateObserverDelivery(request.delivery())) {
    return error;
  }
  if (adapter_registry_->HasController(request.adapter_id())) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        ALREADY_EXISTS, "adapter_id is already registered as a controller");
  }
  *existing = adapter_registry_->FindObserver(request.adapter_id());
  *replacing = *existing != nullptr &&
               WireState((*existing)->phase) == TIMED_OUT &&
               options_.allow_timed_out_adapter_replacement;
  if (*existing != nullptr && !*replacing) {
    return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
        ALREADY_EXISTS,
        WireState((*existing)->phase) == TIMED_OUT
            ? "timed-out observer replacement is disabled"
            : "adapter_id is already registered");
  }
  return std::nullopt;
}

mujoco_sim::core::RegisterAdapterResponse
SimulationCore::Impl::RegisterObserverAdapter(
    const mujoco_sim::core::RegisterAdapterRequest& request) {
  Observer* existing = nullptr;
  bool replacing = false;
  if (auto error = ValidateObserverRequest(request, &existing, &replacing)) {
    return std::move(*error);
  }

  Observer observer;
  observer.id = request.adapter_id();
  observer.epoch = epoch_;
  observer.session_timeout_ms =
      request.session_timeout_ms() == 0 ? options_.default_session_timeout_ms
                                        : request.session_timeout_ms();
  observer.registered_at = std::chrono::steady_clock::now();
  observer.last_heartbeat_at = observer.registered_at;
  observer.state_since_tick = current_tick_id_;
  observer.delivery = request.delivery();
  for (const auto& name : request.observation_names()) {
    const auto* descriptor = resource_catalog_->FindObservation(name);
    if (descriptor == nullptr) {
      return ErrorResponse<mujoco_sim::core::RegisterAdapterResponse>(
          NOT_FOUND, "unknown observation sensor: " + name);
    }
    CopyDescriptor(*descriptor, observer.layout.add_observations());
  }
  observer.session_id = adapter_registry_->AllocateSession();
  internal::FinalizeLayout(&observer.layout);
  if (replacing) adapter_registry_->EraseObserver(existing->id);

  mujoco_sim::core::RegisterAdapterResponse response;
  response.set_code(OK);
  response.set_adapter_id(observer.id);
  response.set_role(OBSERVER);
  response.set_session_id(observer.session_id);
  response.set_epoch(observer.epoch);
  response.set_state(PROVISIONAL);
  response.set_message(replacing ? "replaced timed-out observer session" : "");
  *response.mutable_layout() = observer.layout;
  adapter_registry_->InsertObserver(std::move(observer));
  return response;
}

mujoco_sim::core::RegisterAdapterResponse SimulationCore::Impl::DoRegisterAdapter(
    const mujoco_sim::core::RegisterAdapterRequest& request) {
  if (auto error = ValidateRegistrationRequest(request)) {
    return std::move(*error);
  }
  return request.role() == mujoco_sim::core::CONTROLLER
             ? RegisterControlAdapter(request)
             : RegisterObserverAdapter(request);
}

}  // namespace mujoco_sim::core
