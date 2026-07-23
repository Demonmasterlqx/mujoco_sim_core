#include "internal/controller_client_impl.hpp"

#include <cmath>
#include <string>

#include "internal/callback_boundary.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

mujoco_sim::v4::FaultCode ToWireFault(FaultCode code) {
  switch (code) {
    case FaultCode::kControllerError:
      return mujoco_sim::v4::CONTROLLER_ERROR;
    case FaultCode::kInvalidOutput:
      return mujoco_sim::v4::INVALID_OUTPUT;
    case FaultCode::kInternalControllerError:
      return mujoco_sim::v4::INTERNAL_CONTROLLER_ERROR;
  }
  return mujoco_sim::v4::FAULT_CODE_UNSPECIFIED;
}

bool IsValidFault(FaultCode code) {
  return code == FaultCode::kControllerError || code == FaultCode::kInvalidOutput ||
         code == FaultCode::kInternalControllerError;
}

}  // namespace

bool ControllerClient::Impl::CommandsValid(std::span<const double> values) const {
  for (const auto& resource : registration.layout.commands()) {
    const auto scalar_span = resource.scalars;
    for (std::size_t index = scalar_span.offset; index < scalar_span.offset + scalar_span.count;
         ++index) {
      if (!std::isfinite(values[index])) return false;
      if (resource.has_command_range &&
          (values[index] < resource.command_min || values[index] > resource.command_max)) {
        return false;
      }
    }
  }
  return true;
}

bool ControllerClient::Impl::SetCommand(std::span<const double> values,
                                        std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  std::lock_guard lock(mutex);
  if (!active || stop || !terminal.empty()) {
    *error_message = "controller is not active";
    return false;
  }
  if (values.size() != registration.layout.command_scalar_count()) {
    *error_message = "command scalar count mismatch";
    return false;
  }
  if (!CommandsValid(values)) {
    *error_message = "command contains an invalid value";
    return false;
  }
  latest_command.assign(values.begin(), values.end());
  ++generation;
  condition.notify_all();
  return true;
}

bool ControllerClient::Impl::ReportFault(FaultCode code, std::string_view message,
                                         std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  if (message.size() > 4096 || !IsValidFault(code)) {
    *error_message = "invalid fault";
    return false;
  }
  auto lifecycle_operation = lifecycle.Acquire();
  {
    std::lock_guard lock(mutex);
    if (!started || !active || unregistered || stop || !terminal.empty()) {
      *error_message = "controller is not active";
      return false;
    }
  }

  mujoco_sim::v4::ControllerFault fault;
  fault.set_session_id(registration.session_id);
  fault.set_epoch(registration.epoch);
  fault.set_code(ToWireFault(code));
  fault.set_message(std::string(message));
  if (!transport) {
    *error_message = "failed to publish ControllerFault: transport is unavailable";
    return false;
  }
  const auto result = transport->PublishFault(fault);
  if (!result.error.empty()) {
    *error_message = "failed to publish ControllerFault: " + result.error;
    return false;
  }
  if (!result.sent) {
    *error_message = "failed to publish ControllerFault: eCAL rejected the payload";
    return false;
  }
  return true;
}

void ControllerClient::Impl::PublishLoop() {
  std::unique_lock lock(mutex);
  std::uint64_t sent_generation = 0;
  std::uint64_t pending_generation = 0;
  auto next = Clock::now() + keepalive;
  std::string pending;
  while (!stop) {
    condition.wait_until(lock, next, [&] { return stop || generation > sent_generation; });
    if (stop) break;
    if (latest_command.empty()) {
      next = Clock::now() + keepalive;
      continue;
    }
    if (pending.empty()) {
      mujoco_sim::protocol::DataFrameHeader header;
      header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
      header.session_id = registration.session_id;
      header.epoch = registration.epoch;
      header.layout = fingerprint;
      header.sequence = next_sequence++;
      mujoco_sim::protocol::DataFrameError error;
      if (!mujoco_sim::protocol::EncodeDataFrame(header, latest_command, &pending, &error)) {
        active = false;
        terminal = error.message;
        stop = true;
        condition.notify_all();
        break;
      }
      pending_generation = generation;
    }

    const auto bytes = pending;
    lock.unlock();
    const auto result = transport->PublishCommand(bytes);
    if (!result.error.empty()) {
      SetTerminalError("failed to publish ControllerCommand: " + result.error);
      return;
    }
    lock.lock();
    if (result.sent) {
      pending.clear();
      sent_generation = pending_generation;
      next = Clock::now() + keepalive;
    } else {
      condition.wait_for(lock, std::chrono::milliseconds(10), [&] { return stop.load(); });
    }
  }
}

void ControllerClient::Impl::RunPublishLoop() noexcept {
  internal::InvokeCallbackBoundary(
      [this] { PublishLoop(); },
      [this](const char* message) {
        SetTerminalError(std::string("Controller publisher thread failed: ") + message);
      });
}

}  // namespace mujoco_sim::adapter_sdk
