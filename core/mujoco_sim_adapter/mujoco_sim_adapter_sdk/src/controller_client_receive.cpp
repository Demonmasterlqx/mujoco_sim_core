#include "internal/controller_client_impl.hpp"

#include <exception>
#include <limits>
#include <string>
#include <utility>

#include <mujoco_sim/protocol/topics.hpp>

#include "internal/callback_boundary.hpp"

namespace mujoco_sim::adapter_sdk {

bool ControllerClient::Impl::StartReception(std::string* error_message) {
  try {
    {
      std::lock_guard callback_lock(callback_state->mutex);
      callback_state->client = this;
      callback_state->enabled = true;
    }
    const auto state = callback_state;
    const auto weak_self = weak_from_this();
    state_sub = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::ControllerStateTopicName(options.server_name, options.adapter_id));
    state_sub->SetReceiveCallback([state, weak_self](
                                      const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
                                      const eCAL::SReceiveCallbackData& data) noexcept {
      internal::InvokeCallbackBoundary(
          [&] {
            const auto self = weak_self.lock();
            if (!self) return;
            CallbackLease callback(state);
            if (callback.client() != nullptr) {
              self->OnState(data.buffer, data.buffer_size);
            }
          },
          [&](const char* message) {
            if (const auto self = weak_self.lock()) {
              self->SetTerminalError(std::string("controller state callback failed: ") + message);
            }
          });
    });
    status_sub = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::StatusTopicName(options.server_name));
    status_sub->SetReceiveCallback([state, weak_self](
                                       const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
                                       const eCAL::SReceiveCallbackData& data) noexcept {
      internal::InvokeCallbackBoundary(
          [&] {
            const auto self = weak_self.lock();
            if (!self) return;
            CallbackLease callback(state);
            if (callback.client() != nullptr) {
              self->OnStatus(data.buffer, data.buffer_size);
            }
          },
          [&](const char* message) {
            if (const auto self = weak_self.lock()) {
              self->SetTerminalError(std::string("controller status callback failed: ") + message);
            }
          });
    });
    return true;
  } catch (const std::exception& exception) {
    *error_message =
        std::string("failed to create Controller receive endpoints: ") + exception.what();
  } catch (...) {
    *error_message = "failed to create Controller receive endpoints: unknown exception";
  }
  return false;
}

void ControllerClient::Impl::StopReception() noexcept {
  CallbackGate::Disable(callback_state);
  const auto remove_callback = [this](auto& subscriber, const char* endpoint) {
    if (!subscriber) return;
    internal::InvokeCallbackBoundary([&] { subscriber->RemoveReceiveCallback(); },
                                     [this, endpoint](const char* message) {
                                       SetTerminalError(std::string("failed to remove ") +
                                                        endpoint + " receive callback: " + message);
                                     });
  };
  remove_callback(state_sub, "controller state");
  remove_callback(status_sub, "controller status");
  CallbackGate::WaitForOtherCallbacks(callback_state);
  state_sub.reset();
  status_sub.reset();
}

bool ControllerClient::Impl::StateReady(bool require_new) const {
  return stop || !terminal.empty() ||
         (latest && (!require_new || latest->header.sequence > last_read));
}

bool ControllerClient::Impl::WaitForState(std::unique_lock<std::mutex>& lock, bool wait,
                                          std::chrono::milliseconds timeout) {
  if (StateReady(wait)) return true;
  if (!wait) return false;
  const auto ready = [this, wait] {
    return StateReady(wait);
  };
  if (timeout == std::chrono::milliseconds::max()) {
    condition.wait(lock, ready);
    return true;
  }
  return condition.wait_for(lock, timeout, ready);
}

StateReadResult ControllerClient::Impl::Read(StateLease* output, std::string* error_message,
                                             bool wait, std::chrono::milliseconds timeout) {
  if (output == nullptr || error_message == nullptr || timeout.count() < 0) {
    if (error_message != nullptr) *error_message = "invalid state read arguments";
    return StateReadResult::kError;
  }
  error_message->clear();
  std::unique_lock lock(mutex);
  if (waiting) {
    *error_message = "another state read is already in progress";
    return StateReadResult::kError;
  }
  waiting = true;
  if (!WaitForState(lock, wait, timeout)) {
    waiting = false;
    return StateReadResult::kTimeout;
  }
  if (!terminal.empty()) {
    *error_message = terminal;
    waiting = false;
    return StateReadResult::kError;
  }
  if (stop) {
    waiting = false;
    return StateReadResult::kStopped;
  }

  StateLease lease;
  lease.data_ = latest;
  last_read = latest->header.sequence;
  *output = std::move(lease);
  waiting = false;
  return StateReadResult::kReceived;
}

void ControllerClient::Impl::SetTerminalError(std::string message) {
  {
    std::lock_guard lock(mutex);
    active = false;
    if (terminal.empty()) terminal = std::move(message);
  }
  stop = true;
  condition.notify_all();
}

void ControllerClient::Impl::OnState(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) return;
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError error;
  if (!mujoco_sim::protocol::ParseDataFrame(std::span(static_cast<const std::byte*>(data), size),
                                            &parsed, &error) ||
      parsed.header.kind != mujoco_sim::protocol::DataFrameKind::kControllerState ||
      parsed.header.session_id != registration.session_id ||
      parsed.header.epoch != registration.epoch || parsed.header.layout != fingerprint ||
      parsed.scalar_count != registration.layout.observation_scalar_count()) {
    return;
  }

  auto value = std::make_shared<StateLease::Data>();
  value->header = parsed.header;
  value->scalars.resize(parsed.scalar_count);
  if (!mujoco_sim::protocol::DecodeDataFrameScalars(parsed, value->scalars, &error)) return;
  std::lock_guard lock(mutex);
  if (latest && value->header.sequence <= latest->header.sequence) return;
  latest = std::move(value);
  condition.notify_all();
}

void ControllerClient::Impl::OnStatus(const void* data, std::size_t size) {
  mujoco_sim::v4::ServerStatus status;
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !status.ParseFromArray(data, static_cast<int>(size)) ||
      status.epoch() != registration.epoch) {
    return;
  }

  const mujoco_sim::v4::AdapterStatus* session = nullptr;
  for (const auto& adapter : status.adapters()) {
    if (adapter.session_id() == registration.session_id) {
      session = &adapter;
      break;
    }
  }
  std::lock_guard lock(mutex);
  if (session != nullptr) {
    seen_status = true;
    if (session->state() == mujoco_sim::v4::ERROR ||
        session->state() == mujoco_sim::v4::UNREGISTERED) {
      terminal = session->message().empty() ? "controller session terminated" : session->message();
    }
  } else if (seen_status) {
    terminal = "controller session disappeared from ServerStatus";
  }
  if (!terminal.empty()) condition.notify_all();
}

}  // namespace mujoco_sim::adapter_sdk
