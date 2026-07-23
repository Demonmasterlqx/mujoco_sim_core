#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <ecal/ecal.h>
#include <ecal/pubsub/subscriber.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v4/sync.pb.h>
#include "ecal_runtime.hpp"
#include "internal/adapter_transport.hpp"
#include "internal/callback_boundary.hpp"
#include "internal/callback_gate.hpp"
#include "internal/controller_client_test_peer.hpp"
#include "internal/lifecycle_operation.hpp"
namespace mujoco_sim::adapter_sdk {
namespace {
std::uint64_t Word(std::string_view b, std::size_t o) {
  std::uint64_t v = 0;
  for (std::size_t i = 0; i < 8 && o + i < b.size(); ++i)
    v |= static_cast<std::uint64_t>(static_cast<unsigned char>(b[o + i])) << (8 * i);
  return v;
}
ResourceInfo Resource(const mujoco_sim::v4::ResourceSlot& s) {
  return {s.resource().name(),
          {s.scalar_offset(), s.resource().dimension()},
          s.resource().has_ctrl_range(),
          s.resource().ctrl_min(),
          s.resource().ctrl_max()};
}
mujoco_sim::v4::FaultCode WireFault(FaultCode c) {
  switch (c) {
    case FaultCode::kControllerError:
      return mujoco_sim::v4::CONTROLLER_ERROR;
    case FaultCode::kInvalidOutput:
      return mujoco_sim::v4::INVALID_OUTPUT;
    case FaultCode::kInternalControllerError:
      return mujoco_sim::v4::INTERNAL_CONTROLLER_ERROR;
  }
  return mujoco_sim::v4::FAULT_CODE_UNSPECIFIED;
}
bool ValidFault(FaultCode c) {
  return c == FaultCode::kControllerError || c == FaultCode::kInvalidOutput ||
         c == FaultCode::kInternalControllerError;
}
bool UnregisterCompleted(mujoco_sim::v4::ErrorCode code) {
  return code == mujoco_sim::v4::OK || code == mujoco_sim::v4::NOT_FOUND ||
         code == mujoco_sim::v4::INVALID_SESSION;
}
}  // namespace
struct StateLease::Data {
  mujoco_sim::protocol::DataFrameHeader header;
  std::vector<double> scalars;
};
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
  return data_ ? data_->header.simulation_time_s : 0;
}
std::span<const double> StateLease::observations() const noexcept {
  return data_ ? std::span<const double>(data_->scalars) : std::span<const double>{};
}
StateLease::operator bool() const noexcept {
  return bool(data_);
}

class ControllerClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  friend class ControllerClientTestPeer;
  explicit Impl(ControllerOptions o)
      : options(std::move(o)), callback_state(std::make_shared<CallbackState>()) {
    callback_state->client = this;
  }
  ~Impl() {
    Stop();
    JoinPublisher();
    StopReception();
    std::string ignored;
    Unregister(&ignored);
    transport.reset();
    runtime.reset();
  }
  bool Validate(std::string* e) {
    if (!e) return false;
    if (options.adapter_id.empty() || options.command_names.empty() ||
        options.service_timeout_ms == 0) {
      *e = "adapter_id, command resources, and service timeout are required";
      return false;
    }
    return mujoco_sim::protocol::ValidateIdentifier(
               options.server_name, mujoco_sim::protocol::IdentifierKind::kServer, e) &&
           mujoco_sim::protocol::ValidateIdentifier(
               options.adapter_id, mujoco_sim::protocol::IdentifierKind::kAdapter, e);
  }
  mujoco_sim::v4::RegisterAdapterRequest RegistrationRequest() const {
    mujoco_sim::v4::RegisterAdapterRequest req;
    req.set_adapter_id(options.adapter_id);
    req.set_role(mujoco_sim::v4::CONTROLLER);
    for (const auto& name : options.observation_names)
      req.add_observation_names(name);
    for (const auto& name : options.command_names)
      req.add_command_names(name);
    return req;
  }
  bool AcceptRegistration(std::string* e) {
    if (wire.adapter_id() != options.adapter_id) {
      *e = "registration adapter ID mismatch";
      return false;
    }
    if (wire.role() != mujoco_sim::v4::CONTROLLER) {
      *e = "registration role mismatch";
      return false;
    }
    if (wire.session_id() == 0 || wire.epoch() == 0) {
      *e = "registration identity is invalid";
      return false;
    }
    if (wire.state() != mujoco_sim::v4::PROVISIONAL) {
      *e = "registration state is not PROVISIONAL";
      return false;
    }
    if (!wire.has_controller_policy()) {
      *e = "registration is missing Controller policy";
      return false;
    }
    if (!mujoco_sim::protocol::ValidateLayoutHash(wire.layout(), e)) return false;
    std::vector<ResourceInfo> obs, cmd;
    for (const auto& slot : wire.layout().observations())
      obs.push_back(Resource(slot));
    for (const auto& slot : wire.layout().commands())
      cmd.push_back(Resource(slot));
    registration = {
        wire.adapter_id(), wire.session_id(), wire.epoch(),
        ControllerLayout(std::move(obs), std::move(cmd), wire.layout().observation_scalar_count(),
                         wire.layout().command_scalar_count())};
    fingerprint = {Word(wire.layout().layout_fingerprint(), 0),
                   Word(wire.layout().layout_fingerprint(), 8)};
    keepalive = std::chrono::milliseconds(wire.controller_policy().command_keepalive_ms());
    if (keepalive.count() <= 0) {
      *e = "invalid command keepalive policy";
      return false;
    }
    return true;
  }
  bool Start(std::string* e) {
    if (!e) return false;
    auto life = lifecycle.Acquire();
    e->clear();
    if (started || remote_registered || stop) {
      *e = "controller client cannot be started";
      return false;
    }
    if (!Validate(e)) return false;
    runtime = internal::AcquireEcalRuntime("mujoco-sim-controller-" + options.adapter_id, e);
    if (!runtime) return false;
    try {
      if (!transport) {
        transport = internal::MakeEcalAdapterTransport(options.server_name,
                                                       options.service_timeout_ms, &stop, true);
      }
    } catch (const std::exception& exception) {
      *e = std::string("failed to create Controller transport: ") + exception.what();
      return false;
    } catch (...) {
      *e = "failed to create Controller transport: unknown exception";
      return false;
    }
    if (!transport->Register(RegistrationRequest(), &wire, e) ||
        wire.code() != mujoco_sim::v4::OK) {
      if (e->empty()) *e = wire.message();
      return false;
    }
    remote_registered = true;
    unregistered = false;
    return FinishStart(e);
  }
  bool FinishStart(std::string* e) {
    if (!AcceptRegistration(e)) {
      CompensateRegistration(e);
      return false;
    }
    if (stop) {
      *e = "controller start was cancelled";
      CompensateRegistration(e);
      return false;
    }
    if (!StartReception(e)) {
      StopReception();
      CompensateRegistration(e);
      return false;
    }
    {
      std::lock_guard lock(mutex);
      if (stop) {
        *e = "controller start was cancelled";
      } else {
        started = true;
      }
    }
    if (!e->empty()) {
      StopReception();
      CompensateRegistration(e);
      return false;
    }
    return true;
  }
  void CompensateRegistration(std::string* e) {
    const std::string cause = *e;
    mujoco_sim::v4::UnregisterAdapterRequest req;
    req.set_session_id(wire.session_id());
    req.set_epoch(wire.epoch());
    mujoco_sim::v4::UnregisterAdapterResponse res;
    std::string cleanup_error;
    if (transport->Unregister(req, &res, &cleanup_error) && UnregisterCompleted(res.code())) {
      remote_registered = false;
      unregistered = true;
    } else {
      if (cleanup_error.empty()) cleanup_error = res.message();
      *e = cause + "; compensating unregister failed: " + cleanup_error;
    }
  }
  void AbortActivation(std::string* e) {
    StopLocal();
    CompensateRegistration(e);
    StopReception();
  }
  bool StartReception(std::string* e) {
    try {
      {
        std::lock_guard callback_lock(callback_state->mutex);
        callback_state->client = this;
        callback_state->enabled = true;
      }
      const auto state = callback_state;
      auto weak = weak_from_this();
      state_sub = std::make_unique<eCAL::CSubscriber>(
          mujoco_sim::protocol::ControllerStateTopicName(options.server_name, options.adapter_id));
      state_sub->SetReceiveCallback([state, weak](const eCAL::STopicId&,
                                                  const eCAL::SDataTypeInformation&,
                                                  const eCAL::SReceiveCallbackData& d) noexcept {
        internal::InvokeCallbackBoundary(
            [&] {
              const auto self = weak.lock();
              if (!self) return;
              CallbackLease callback(state);
              if (callback.client() != nullptr) self->OnState(d.buffer, d.buffer_size);
            },
            [&](const char* message) {
              if (const auto self = weak.lock()) {
                self->SetTerminalError(std::string("controller state callback failed: ") + message);
              }
            });
      });
      status_sub = std::make_unique<eCAL::CSubscriber>(
          mujoco_sim::protocol::StatusTopicName(options.server_name));
      status_sub->SetReceiveCallback([state, weak](const eCAL::STopicId&,
                                                   const eCAL::SDataTypeInformation&,
                                                   const eCAL::SReceiveCallbackData& d) noexcept {
        internal::InvokeCallbackBoundary(
            [&] {
              const auto self = weak.lock();
              if (!self) return;
              CallbackLease callback(state);
              if (callback.client() != nullptr) self->OnStatus(d.buffer, d.buffer_size);
            },
            [&](const char* message) {
              if (const auto self = weak.lock()) {
                self->SetTerminalError(std::string("controller status callback failed: ") +
                                       message);
              }
            });
      });
      return true;
    } catch (const std::exception& x) {
      *e = x.what();
      return false;
    } catch (...) {
      *e = "failed to create Controller receive endpoints: unknown exception";
      return false;
    }
  }
  void StopReception() noexcept {
    CallbackGate::Disable(callback_state);
    const auto remove_callback = [this](auto& subscriber, const char* endpoint) {
      if (!subscriber) return;
      internal::InvokeCallbackBoundary([&] { subscriber->RemoveReceiveCallback(); },
                                       [this, endpoint](const char* message) {
                                         SetTerminalError(std::string("failed to remove ") +
                                                          endpoint +
                                                          " receive callback: " + message);
                                       });
    };
    if (state_sub) {
      remove_callback(state_sub, "controller state");
    }
    if (status_sub) {
      remove_callback(status_sub, "controller status");
    }
    CallbackGate::WaitForOtherCallbacks(callback_state);
    state_sub.reset();
    status_sub.reset();
  }
  bool Activate(std::string* e) {
    if (!e) return false;
    auto life = lifecycle.Acquire();
    {
      std::lock_guard lock(mutex);
      if (!started || active || stop) {
        *e = "controller client is not ready to activate";
        return false;
      }
    }
    mujoco_sim::v4::ActivateAdapterRequest req;
    req.set_session_id(registration.session_id);
    req.set_epoch(registration.epoch);
    mujoco_sim::v4::ActivateAdapterResponse res;
    if (!transport->Activate(req, &res, e) || res.code() != mujoco_sim::v4::OK ||
        res.state() != mujoco_sim::v4::ACTIVE_NO_COMMAND) {
      if (e->empty()) *e = res.message();
      return false;
    }
    bool cancelled = false;
    {
      std::lock_guard lock(mutex);
      if (stop) {
        *e = "controller activation was cancelled";
        cancelled = true;
      } else {
        active = true;
      }
    }
    if (cancelled) {
      AbortActivation(e);
      return false;
    }
    try {
      if (before_publisher_start) before_publisher_start();
      publisher = std::thread([this] { RunPublishLoop(); });
    } catch (const std::exception& exception) {
      *e = std::string("failed to start Controller publisher: ") + exception.what();
      AbortActivation(e);
      return false;
    } catch (...) {
      *e = "failed to start Controller publisher: unknown exception";
      AbortActivation(e);
      return false;
    }
    return true;
  }
  bool StateReady(bool require_new) const {
    return stop || !terminal.empty() ||
           (latest && (!require_new || latest->header.sequence > last_read));
  }
  bool WaitForState(std::unique_lock<std::mutex>& lock, bool wait,
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
  StateReadResult Read(StateLease* out, std::string* e, bool wait,
                       std::chrono::milliseconds timeout) {
    if (!out || !e || timeout.count() < 0) {
      if (e) *e = "invalid state read arguments";
      return StateReadResult::kError;
    }
    std::unique_lock lock(mutex);
    if (waiting) {
      *e = "another state read is already in progress";
      return StateReadResult::kError;
    }
    waiting = true;
    if (!WaitForState(lock, wait, timeout)) {
      waiting = false;
      return StateReadResult::kTimeout;
    }
    if (!terminal.empty()) {
      *e = terminal;
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
    *out = std::move(lease);
    waiting = false;
    return StateReadResult::kReceived;
  }
  bool CommandsValid(std::span<const double> values) const {
    for (const auto& resource : registration.layout.commands()) {
      const auto span = resource.scalars;
      for (std::size_t index = span.offset; index < span.offset + span.count; ++index) {
        if (!std::isfinite(values[index])) return false;
        if (resource.has_command_range &&
            (values[index] < resource.command_min || values[index] > resource.command_max))
          return false;
      }
    }
    return true;
  }
  bool SetCommand(std::span<const double> values, std::string* e) {
    if (!e) return false;
    std::lock_guard lock(mutex);
    if (!active || stop || !terminal.empty()) {
      *e = "controller is not active";
      return false;
    }
    if (values.size() != registration.layout.command_scalar_count()) {
      *e = "command scalar count mismatch";
      return false;
    }
    if (!CommandsValid(values)) {
      *e = "command contains an invalid value";
      return false;
    }
    latest_command.assign(values.begin(), values.end());
    ++generation;
    condition.notify_all();
    return true;
  }
  bool ReportFault(FaultCode code, std::string_view message, std::string* e) {
    if (!e || message.size() > 4096 || !ValidFault(code)) {
      if (e) *e = "invalid fault";
      return false;
    }
    auto life = lifecycle.Acquire();
    {
      std::lock_guard lock(mutex);
      if (!started || !active || unregistered || stop || !terminal.empty()) {
        *e = "controller is not active";
        return false;
      }
    }
    mujoco_sim::v4::ControllerFault fault;
    fault.set_session_id(registration.session_id);
    fault.set_epoch(registration.epoch);
    fault.set_code(WireFault(code));
    fault.set_message(std::string(message));
    if (!transport) {
      *e = "failed to publish ControllerFault: transport is unavailable";
      return false;
    }
    const auto result = transport->PublishFault(fault);
    if (!result.error.empty()) {
      *e = "failed to publish ControllerFault: " + result.error;
      return false;
    }
    if (!result.sent) {
      *e = "failed to publish ControllerFault: eCAL rejected the payload";
      return false;
    }
    return result.sent;
  }
  void PublishLoop() {
    std::unique_lock lock(mutex);
    std::uint64_t sent_generation = 0, pending_generation = 0;
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
        mujoco_sim::protocol::DataFrameHeader h;
        h.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
        h.session_id = registration.session_id;
        h.epoch = registration.epoch;
        h.layout = fingerprint;
        h.sequence = next_sequence++;
        mujoco_sim::protocol::DataFrameError error;
        if (!mujoco_sim::protocol::EncodeDataFrame(h, latest_command, &pending, &error)) {
          terminal = error.message;
          condition.notify_all();
          break;
        }
        pending_generation = generation;
      }
      auto bytes = pending;
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
  void RunPublishLoop() noexcept {
    internal::InvokeCallbackBoundary(
        [this] { PublishLoop(); },
        [this](const char* message) {
          SetTerminalError(std::string("Controller publisher thread failed: ") + message);
        });
  }
  void SetTerminalError(std::string message) {
    {
      std::lock_guard lock(mutex);
      active = false;
      if (terminal.empty()) terminal = std::move(message);
    }
    stop = true;
    condition.notify_all();
  }
  void OnState(const void* data, std::size_t size) {
    if (!data || !size) return;
    mujoco_sim::protocol::ParsedDataFrame parsed;
    mujoco_sim::protocol::DataFrameError error;
    if (!mujoco_sim::protocol::ParseDataFrame(std::span(static_cast<const std::byte*>(data), size),
                                              &parsed, &error) ||
        parsed.header.kind != mujoco_sim::protocol::DataFrameKind::kControllerState ||
        parsed.header.session_id != registration.session_id ||
        parsed.header.epoch != registration.epoch || parsed.header.layout != fingerprint ||
        parsed.scalar_count != registration.layout.observation_scalar_count())
      return;
    auto value = std::make_shared<StateLease::Data>();
    value->header = parsed.header;
    value->scalars.resize(parsed.scalar_count);
    if (!mujoco_sim::protocol::DecodeDataFrameScalars(parsed, value->scalars, &error)) return;
    std::lock_guard lock(mutex);
    if (latest && value->header.sequence <= latest->header.sequence) return;
    latest = std::move(value);
    condition.notify_all();
  }
  void OnStatus(const void* data, std::size_t size) {
    mujoco_sim::v4::ServerStatus status;
    if (!data || !size || size > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        !status.ParseFromArray(data, static_cast<int>(size)) ||
        status.epoch() != registration.epoch)
      return;
    bool found = false;
    const mujoco_sim::v4::AdapterStatus* session = nullptr;
    for (const auto& a : status.adapters())
      if (a.session_id() == registration.session_id) {
        found = true;
        session = &a;
        break;
      }
    std::lock_guard lock(mutex);
    if (found) {
      seen_status = true;
      if (session->state() == mujoco_sim::v4::ERROR ||
          session->state() == mujoco_sim::v4::UNREGISTERED)
        terminal =
            session->message().empty() ? "controller session terminated" : session->message();
    } else if (seen_status) {
      terminal = "controller session disappeared from ServerStatus";
    }
    if (!terminal.empty()) condition.notify_all();
  }
  void StopLocal() {
    std::lock_guard lock(mutex);
    active = false;
    stop = true;
    condition.notify_all();
  }
  void Stop() {
    StopLocal();
    auto life = lifecycle.Acquire();
    JoinPublisher();
    StopReception();
  }
  void JoinPublisher() {
    if (publisher.joinable() && publisher.get_id() != std::this_thread::get_id()) publisher.join();
  }
  bool Unregister(std::string* e) {
    if (!e) return false;
    auto life = lifecycle.Acquire();
    if (!remote_registered || unregistered) return true;
    mujoco_sim::v4::UnregisterAdapterRequest req;
    req.set_session_id(wire.session_id());
    req.set_epoch(wire.epoch());
    mujoco_sim::v4::UnregisterAdapterResponse res;
    if (!transport->Unregister(req, &res, e) || !UnregisterCompleted(res.code())) {
      if (e->empty()) *e = res.message();
      return false;
    }
    remote_registered = false;
    unregistered = true;
    StopLocal();
    JoinPublisher();
    StopReception();
    return true;
  }
  using Clock = std::chrono::steady_clock;
  ControllerOptions options;
  mujoco_sim::v4::RegisterAdapterResponse wire;
  AdapterRegistration registration;
  mujoco_sim::protocol::LayoutFingerprint fingerprint{};
  std::chrono::milliseconds keepalive{100};
  std::shared_ptr<internal::AdapterTransport> transport;
  std::shared_ptr<void> runtime;
  using CallbackGate = internal::CallbackGate<Impl>;
  using CallbackState = CallbackGate::State;
  using CallbackLease = CallbackGate::Lease;
  std::shared_ptr<CallbackState> callback_state;
  std::unique_ptr<eCAL::CSubscriber> state_sub, status_sub;
  internal::LifecycleOperation lifecycle;
  std::mutex mutex;
  std::condition_variable condition;
  std::thread publisher;
  std::function<void()> before_publisher_start;
  std::atomic<bool> stop{false};
  bool started = false, active = false, remote_registered = false, unregistered = false,
       waiting = false, seen_status = false;
  std::shared_ptr<const StateLease::Data> latest;
  std::vector<double> latest_command;
  std::uint64_t generation = 0, next_sequence = 1, last_read = 0;
  std::string terminal;
};
ControllerClient::ControllerClient(ControllerOptions o)
    : impl_(std::make_shared<Impl>(std::move(o))) {}
ControllerClient::~ControllerClient() = default;
bool ControllerClient::Start(std::string* e) {
  return impl_->Start(e);
}
bool ControllerClient::Activate(std::string* e) {
  return impl_->Activate(e);
}
StateReadResult ControllerClient::ReadLatestState(StateLease* s, std::string* e) {
  return impl_->Read(s, e, false, std::chrono::milliseconds(0));
}
StateReadResult ControllerClient::WaitForNewState(std::chrono::milliseconds t, StateLease* s,
                                                  std::string* e) {
  return impl_->Read(s, e, true, t);
}
bool ControllerClient::SetCommand(std::span<const double> v, std::string* e) {
  return impl_->SetCommand(v, e);
}
bool ControllerClient::ReportFault(FaultCode c, std::string_view m, std::string* e) {
  return impl_->ReportFault(c, m, e);
}
void ControllerClient::Stop() {
  impl_->Stop();
}
bool ControllerClient::Unregister(std::string* e) {
  return impl_->Unregister(e);
}
const AdapterRegistration& ControllerClient::registration() const {
  return impl_->registration;
}
const ControllerLayout& ControllerClient::layout() const {
  return impl_->registration.layout;
}

void ControllerClientTestPeer::Configure(ControllerClient* client,
                                         std::shared_ptr<internal::AdapterTransport> transport,
                                         std::chrono::milliseconds keepalive) {
  auto& impl = *client->impl_;
  std::lock_guard lock(impl.mutex);
  impl.stop = false;
  impl.started = true;
  impl.active = true;
  impl.remote_registered = false;
  impl.unregistered = false;
  impl.waiting = false;
  impl.seen_status = false;
  impl.registration.adapter_id = "controller";
  impl.registration.session_id = 42;
  impl.registration.epoch = 7;
  impl.registration.layout = ControllerLayout(
      {{"position", {0, 1}, false, 0.0, 0.0}, {"velocity", {1, 1}, false, 0.0, 0.0}},
      {{"actuator", {0, 2}, true, -10.0, 10.0}}, 2, 2);
  impl.fingerprint = {11, 12};
  impl.keepalive = keepalive;
  impl.transport = std::move(transport);
  impl.latest.reset();
  impl.latest_command.clear();
  impl.generation = 0;
  impl.next_sequence = 1;
  impl.last_read = 0;
  impl.terminal.clear();
}

void ControllerClientTestPeer::HandleState(ControllerClient* client, std::uint64_t sequence,
                                           std::uint64_t tick_id,
                                           std::span<const double> observations) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerState;
  header.session_id = 42;
  header.epoch = 7;
  header.tick_id = tick_id;
  header.layout = {11, 12};
  header.sequence = sequence;
  header.simulation_time_s = 0.25;
  std::string bytes;
  mujoco_sim::protocol::DataFrameError error;
  if (mujoco_sim::protocol::EncodeDataFrame(header, observations, &bytes, &error)) {
    client->impl_->OnState(bytes.data(), bytes.size());
  }
}

void ControllerClientTestPeer::StartPublisher(ControllerClient* client) {
  auto& impl = *client->impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  if (!impl.publisher.joinable()) {
    impl.publisher = std::thread([&impl] { impl.RunPublishLoop(); });
  }
}

void ControllerClientTestPeer::EnableRemoteRegistration(ControllerClient* client) {
  auto& impl = *client->impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  impl.wire.set_session_id(impl.registration.session_id);
  impl.wire.set_epoch(impl.registration.epoch);
  impl.remote_registered = true;
  impl.unregistered = false;
}

void ControllerClientTestPeer::FailPublisherStart(ControllerClient* client) {
  auto& impl = *client->impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  impl.before_publisher_start = [] {
    throw std::runtime_error("injected publisher thread creation failure");
  };
}

bool ControllerClientTestPeer::IsWaiting(ControllerClient* client) {
  std::lock_guard lock(client->impl_->mutex);
  return client->impl_->waiting;
}

void ControllerClientTestPeer::PrepareActivation(ControllerClient* client) {
  auto& impl = *client->impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  std::lock_guard lock(impl.mutex);
  impl.started = true;
  impl.active = false;
  impl.stop = false;
}

bool ControllerClientTestPeer::StopRequested(ControllerClient* client) {
  return client->impl_->stop.load();
}
}  // namespace mujoco_sim::adapter_sdk
