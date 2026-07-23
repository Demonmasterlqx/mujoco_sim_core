#include "simulation_loop.hpp"

#include <ecal/ecal.h>
#include <spdlog/logger.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <optional>
#include <utility>

#include <mujoco_sim/protocol/data_frame.hpp>

#include "adapter_registration.hpp"
#include "publish_diagnostics.hpp"
#include "sensor_runtime.hpp"
#include "server_logging.hpp"
#include "server_runtime.hpp"
#include "sim_viewer.hpp"

namespace mujoco_sim::server {
namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kPollPeriod = std::chrono::milliseconds(5);

void LogLoopBoundaryFailure(const std::shared_ptr<spdlog::logger>& logger,
                            std::string_view operation, std::string_view error) noexcept {
  try {
    if (logger) {
      logger->error("event=simulation_loop_boundary_failure operation={} error={}",
                    LogValue(operation), LogValue(error));
    }
  } catch (...) {
  }
}

std::optional<std::string> ObserverDeliveryError(
    const mujoco_sim::core::RegisterAdapterRequest& request, const ServerConfig& config,
    double timestep_s) {
  if (request.role() != mujoco_sim::core::OBSERVER) return std::nullopt;
  const auto& delivery = request.delivery();
  double rate_hz = 0.0;
  if (delivery.has_every_n_steps() && delivery.every_n_steps() > 0) {
    if (!config.allow_every_step_observer && delivery.every_n_steps() == 1) {
      return "every-step Observer delivery is disabled";
    }
    rate_hz = 1.0 / (timestep_s * static_cast<double>(delivery.every_n_steps()));
  } else if (delivery.has_period_s() && std::isfinite(delivery.period_s()) &&
             delivery.period_s() > 0.0) {
    rate_hz = 1.0 / delivery.period_s();
  } else {
    return std::nullopt;
  }
  if (rate_hz > config.observer_max_rate_hz) {
    return "Observer delivery rate exceeds observer_max_rate_hz";
  }
  return std::nullopt;
}

template <class R>
R QueueError(std::string message) {
  R response;
  response.set_code(mujoco_sim::v4::INTERNAL_ERROR);
  response.set_message(std::move(message));
  return response;
}

void CopyDescriptor(const mujoco_sim::core::ResourceDescriptor& source,
                    mujoco_sim::v4::ResourceDescriptor* target) {
  target->set_name(source.name());
  target->set_type(static_cast<mujoco_sim::v4::ResourceType>(source.type()));
  target->set_dimension(source.dimension());
  target->set_mujoco_id(source.mujoco_id());
  target->set_mujoco_type(source.mujoco_type());
  target->set_mujoco_datatype(source.mujoco_datatype());
  target->set_has_ctrl_range(source.has_ctrl_range());
  target->set_ctrl_min(source.ctrl_min());
  target->set_ctrl_max(source.ctrl_max());
  target->set_mujoco_stage(source.mujoco_stage());
  target->set_interval_period_s(source.interval_period_s());
  target->set_interval_phase_s(source.interval_phase_s());
  target->set_delay_s(source.delay_s());
  target->set_frame_id(source.frame_id());
  target->set_unit(source.unit());
  target->set_object_name(source.object_name());
  target->set_object_type(source.object_type());
  target->set_interpolation(source.interpolation());
}

std::string FingerprintBytes(mujoco_sim::core::LayoutFingerprint fingerprint) {
  std::string bytes(16, '\0');
  for (std::size_t i = 0; i < 8; ++i) {
    bytes[i] = static_cast<char>((fingerprint.low >> (i * 8)) & 255);
    bytes[i + 8] = static_cast<char>((fingerprint.high >> (i * 8)) & 255);
  }
  return bytes;
}

void CopyLayout(const mujoco_sim::core::ResourceLayout& source,
                mujoco_sim::v4::ResourceLayout* target) {
  for (std::size_t i = 0; i < source.observations_.size(); ++i) {
    auto* entry = target->add_observations();
    CopyDescriptor(source.observations_[i], entry->mutable_resource());
    entry->set_scalar_offset(source.observation_spans[i].offset);
  }
  for (std::size_t i = 0; i < source.commands_.size(); ++i) {
    auto* entry = target->add_commands();
    CopyDescriptor(source.commands_[i], entry->mutable_resource());
    entry->set_scalar_offset(source.command_spans[i].offset);
  }
  target->set_layout_fingerprint(FingerprintBytes(source.fingerprint));
  target->set_observation_scalar_count(source.observation_scalar_count);
  target->set_command_scalar_count(source.command_scalar_count);
}

mujoco_sim::v4::ControlPolicy WirePolicy(mujoco_sim::core::ControlPolicy policy) {
  return policy == mujoco_sim::core::ControlPolicy::kZero ? mujoco_sim::v4::ZERO
                                                          : mujoco_sim::v4::HOLD_LAST;
}

void CopyDelivery(const mujoco_sim::core::ObservationDelivery& source,
                  mujoco_sim::v4::ObservationDelivery* target) {
  if (source.has_every_n_steps()) {
    target->set_every_n_steps(source.every_n_steps());
  } else if (source.has_period_s()) {
    target->set_period_s(source.period_s());
  }
}

mujoco_sim::core::RegisterAdapterRequest ToDomain(
    const mujoco_sim::v4::RegisterAdapterRequest& source) {
  mujoco_sim::core::RegisterAdapterRequest target;
  target.set_adapter_id(source.adapter_id());
  target.set_role(static_cast<mujoco_sim::core::AdapterRole>(source.role()));
  for (const auto& name : source.observation_names()) {
    target.add_observation_names(name);
  }
  for (const auto& name : source.command_names()) {
    target.add_command_names(name);
  }
  target.set_session_timeout_ms(source.session_timeout_ms());
  if (source.has_delivery()) {
    if (source.delivery().has_every_n_steps()) {
      target.mutable_delivery()->set_every_n_steps(source.delivery().every_n_steps());
    } else if (source.delivery().has_period_s()) {
      target.mutable_delivery()->set_period_s(source.delivery().period_s());
    }
  }
  return target;
}

mujoco_sim::v4::GetAvailableResourcesResponse ToWire(
    const mujoco_sim::core::GetAvailableResourcesResponse& source) {
  mujoco_sim::v4::GetAvailableResourcesResponse target;
  target.set_code(static_cast<mujoco_sim::v4::ErrorCode>(source.code()));
  target.set_message(source.message());
  target.set_epoch(source.epoch());
  for (const auto& descriptor : source.observations()) {
    CopyDescriptor(descriptor, target.add_observations());
  }
  for (const auto& descriptor : source.commands()) {
    CopyDescriptor(descriptor, target.add_commands());
  }
  return target;
}

mujoco_sim::v4::RegisterAdapterResponse ToWire(
    const mujoco_sim::core::RegisterAdapterResponse& source) {
  mujoco_sim::v4::RegisterAdapterResponse target;
  target.set_code(static_cast<mujoco_sim::v4::ErrorCode>(source.code()));
  target.set_message(source.message());
  target.set_adapter_id(source.adapter_id());
  target.set_role(static_cast<mujoco_sim::v4::AdapterRole>(source.role()));
  target.set_session_id(source.session_id());
  target.set_epoch(source.epoch());
  target.set_state(static_cast<mujoco_sim::v4::AdapterState>(source.state()));
  CopyLayout(source.layout(), target.mutable_layout());
  if (source.role() == mujoco_sim::core::CONTROLLER) {
    auto* policy = target.mutable_controller_policy();
    policy->set_command_timeout_ms(source.controller_policy.command_timeout_ms);
    policy->set_command_keepalive_ms(source.controller_policy.command_keepalive_ms);
    CopyDelivery(source.controller_policy.state_delivery, policy->mutable_state_delivery());
    policy->set_startup_policy(WirePolicy(source.controller_policy.startup_policy));
    policy->set_offline_policy(WirePolicy(source.controller_policy.offline_policy));
    policy->set_error_policy(WirePolicy(source.controller_policy.error_policy));
    policy->set_release_policy(WirePolicy(source.controller_policy.release_policy));
    policy->set_offline_session_expire_ms(source.controller_policy.offline_session_expire_ms);
  }
  return target;
}

template <class WireResponse>
WireResponse ToWire(const mujoco_sim::core::AdapterSessionResponse& source) {
  WireResponse target;
  target.set_code(static_cast<mujoco_sim::v4::ErrorCode>(source.code()));
  target.set_message(source.message());
  target.set_state(static_cast<mujoco_sim::v4::AdapterState>(source.state()));
  return target;
}

mujoco_sim::v4::ServerStatus ToWire(const mujoco_sim::core::ServerStatus& source) {
  mujoco_sim::v4::ServerStatus target;
  target.set_epoch(source.epoch);
  target.set_tick_id(source.tick_id);
  target.set_simulation_time_s(source.simulation_time_s);
  target.set_state(static_cast<mujoco_sim::v4::ServerState>(source.state));
  target.set_message(source.message);
  for (const auto& value : source.adapters) {
    auto* adapter = target.add_adapters();
    adapter->set_adapter_id(value.adapter_id);
    adapter->set_role(static_cast<mujoco_sim::v4::AdapterRole>(value.role));
    adapter->set_session_id(value.session_id);
    adapter->set_state(static_cast<mujoco_sim::v4::AdapterState>(value.state));
    adapter->set_message(value.message);
    adapter->set_state_since_tick_id(value.state_since_tick_id);
    adapter->set_timeout_count(value.timeout_count);
    adapter->set_last_command_sequence(value.last_command_sequence);
    adapter->set_last_observation_sequence(value.last_observation_sequence);
    if (value.last_delivery_tick_id) {
      adapter->set_last_delivery_tick_id(*value.last_delivery_tick_id);
    }
  }
  return target;
}

mujoco_sim::core::CoreOptions CoreOptionsFor(const ServerConfig& config) {
  mujoco_sim::core::CoreOptions options;
  auto policy = [](const std::string& value) {
    return value == "zero" ? mujoco_sim::core::ControlPolicy::kZero
                           : mujoco_sim::core::ControlPolicy::kHoldLast;
  };
  auto delivery = [](std::uint32_t steps, double period) {
    mujoco_sim::core::ObservationDelivery value;
    if (period > 0.0) {
      value.set_period_s(period);
    } else {
      value.set_every_n_steps(steps);
    }
    return value;
  };
  options.server_name = config.server_name;
  options.command_timeout_ms = config.command_timeout_ms;
  options.command_keepalive_ms = config.command_keepalive_ms;
  options.observer_session_timeout_ms = config.session_timeout_ms;
  options.activation_timeout_ms = config.activation_timeout_ms;
  options.offline_session_expire_ms = config.offline_session_expire_ms;
  options.controller_state_delivery =
      delivery(config.controller_state_every_n_steps, config.controller_state_period_s);
  options.controller_state_max_rate_hz = config.controller_state_max_rate_hz;
  options.startup_policy = policy(config.startup_ctrl_policy);
  options.offline_policy = policy(config.offline_ctrl_policy);
  options.error_policy = policy(config.error_ctrl_policy);
  options.release_policy = policy(config.release_ctrl_policy);
  for (const auto& source : config.controller_overrides) {
    mujoco_sim::core::CoreOptions::ControllerOverride target;
    target.adapter_id = source.adapter_id;
    target.command_timeout_ms = source.command_timeout_ms;
    target.command_keepalive_ms = source.command_keepalive_ms;
    target.offline_session_expire_ms = source.offline_session_expire_ms;
    target.state_delivery = delivery(source.state_every_n_steps, source.state_period_s);
    target.startup_policy = policy(source.startup_ctrl_policy);
    target.offline_policy = policy(source.offline_ctrl_policy);
    target.error_policy = policy(source.error_ctrl_policy);
    target.release_policy = policy(source.release_ctrl_policy);
    options.controller_overrides.push_back(std::move(target));
  }
  return options;
}

mujoco_sim::protocol::DataFrameHeader Header(mujoco_sim::protocol::DataFrameKind kind,
                                             const mujoco_sim::core::FrameIdentity& identity,
                                             std::uint64_t sequence) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = kind;
  header.session_id = identity.session.session_id;
  header.epoch = identity.session.epoch;
  header.tick_id = identity.tick_id;
  header.layout = {identity.layout.low, identity.layout.high};
  header.sequence = sequence;
  header.simulation_time_s = identity.simulation_time_s;
  return header;
}

const char* AdapterEventName(mujoco_sim::core::AdapterChangeKind kind) {
  switch (kind) {
    case mujoco_sim::core::AdapterChangeKind::kRegistered:
      return "adapter_registered";
    case mujoco_sim::core::AdapterChangeKind::kActivated:
      return "adapter_activated";
    case mujoco_sim::core::AdapterChangeKind::kUnregistered:
      return "adapter_unregistered";
    case mujoco_sim::core::AdapterChangeKind::kActivationExpired:
      return "adapter_activation_expired";
    case mujoco_sim::core::AdapterChangeKind::kTimedOut:
      return "adapter_timed_out";
    case mujoco_sim::core::AdapterChangeKind::kRecovered:
      return "adapter_recovered";
    case mujoco_sim::core::AdapterChangeKind::kLocalError:
      return "adapter_local_error";
    case mujoco_sim::core::AdapterChangeKind::kServerFailed:
      return "adapter_server_failed";
    case mujoco_sim::core::AdapterChangeKind::kReplaced:
      return "adapter_replaced";
  }
  return "adapter_changed";
}

}  // namespace

SimulationLoop::SimulationLoop(ServerConfig config, std::shared_ptr<spdlog::logger> logger,
                               WarningLimiter* warning_limiter, EcalServerTransport* transport,
                               SimViewer* viewer)
    : config_(std::move(config)),
      logger_(std::move(logger)),
      warning_limiter_(warning_limiter),
      transport_(transport),
      viewer_(viewer),
      core_(CoreOptionsFor(config_)),
      sensor_runtime_(std::make_unique<SensorRuntime>()) {}

SimulationLoop::~SimulationLoop() {
  std::string error;
  if (!RunWithExceptionBoundary([this] { RequestStop("server_destroyed"); }, &error)) {
    LogLoopBoundaryFailure(logger_, "destructor_request_stop", error);
  }
  if (!RunWithExceptionBoundary(
          [this] {
            if (thread_.joinable()) thread_.join();
          },
          &error)) {
    LogLoopBoundaryFailure(logger_, "destructor_join", error);
  }
}

bool SimulationLoop::Initialize(std::string* error) {
  if (!error || !transport_ || !logger_) {
    return false;
  }
  auto loaded = core_.LoadModel(config_.model_path, {config_.initial_keyframe});
  if (!loaded.response.loaded) {
    *error = loaded.response.message;
    return false;
  }
  if (!sensor_runtime_->Initialize(core_, config_, error)) {
    return false;
  }
  if (viewer_) {
    viewer_snapshot_ = core_.CreateSnapshot(error);
    if (!viewer_snapshot_ || !viewer_->QueueLoad(*viewer_snapshot_, config_.model_path, error)) {
      return false;
    }
  }
  initialized_ = true;
  return true;
}

template <class Event, class Request>
typename Event::response_type SimulationLoop::SubmitRpc(Request request) {
  Event event;
  event.request = std::move(request);
  auto future = event.completion.get_future();
  if (!queue_.TryPush(ServerEvent(std::move(event)))) {
    return QueueError<typename Event::response_type>("server event queue unavailable");
  }
  try {
    return future.get();
  } catch (const std::exception& error) {
    return QueueError<typename Event::response_type>(error.what());
  }
}

EcalServerTransport::Handlers SimulationLoop::MakeHandlers() {
  EcalServerTransport::Handlers handlers;
  handlers.get_available_resources = [this](auto request) {
    return SubmitRpc<GetAvailableResourcesEvent>(std::move(request));
  };
  handlers.register_adapter = [this](auto request) {
    return SubmitRpc<RegisterAdapterEvent>(std::move(request));
  };
  handlers.activate_adapter = [this](auto request) {
    return SubmitRpc<ActivateAdapterEvent>(std::move(request));
  };
  handlers.unregister_adapter = [this](auto request) {
    return SubmitRpc<UnregisterAdapterEvent>(std::move(request));
  };
  handlers.get_sensor_streams = [this](auto request) {
    return SubmitRpc<GetSensorStreamsEvent>(std::move(request));
  };
  handlers.command = [this](auto command) {
    if (command_inbox_.Publish(std::move(command))) {
      queue_.Notify();
    }
  };
  handlers.heartbeat = [this](auto heartbeat) {
    queue_.TryPush(HeartbeatEvent{std::move(heartbeat)});
  };
  handlers.fault = [this](auto fault) {
    if (command_inbox_.PublishFault(std::move(fault))) {
      queue_.Notify();
    }
  };
  handlers.malformed = [this](std::string_view, std::size_t) {
    ++malformed_pending_;
  };
  return handlers;
}

bool SimulationLoop::Start(std::string* error) {
  if (!error || !initialized_ || thread_.joinable()) {
    if (error) {
      *error = "simulation loop is not ready";
    }
    return false;
  }
  std::string thread_error;
  if (!RunWithExceptionBoundary(
          [this] { thread_ = std::jthread([this](std::stop_token stop) { Run(stop); }); },
          &thread_error)) {
    *error = "failed to start simulation loop thread";
    if (!thread_error.empty()) {
      error->append(": ").append(thread_error);
    }
    return false;
  }
  return true;
}

void SimulationLoop::RequestStop(std::string reason) {
  bool first = false;
  {
    std::lock_guard lock(lifecycle_mutex_);
    if (!stop_requested_.load(std::memory_order_relaxed)) {
      stop_reason_ = std::move(reason);
      if (exit_code_.load(std::memory_order_relaxed) != 2) {
        exit_code_.store(0, std::memory_order_relaxed);
      }
      stop_requested_.store(true, std::memory_order_release);
      first = true;
    }
  }
  if (first) {
    command_inbox_.RequestClose();
    queue_.Close();
    if (sensor_runtime_) {
      sensor_runtime_->RequestStop();
    }
    if (viewer_) {
      viewer_->RequestClose();
    }
  }
}

int SimulationLoop::Join() {
  if (thread_.joinable()) {
    thread_.join();
  }
  return exit_code_.load(std::memory_order_acquire);
}

void SimulationLoop::Run(std::stop_token stop) noexcept {
  try {
    run_stop_token_ = stop;
    auto now = Clock::now();
    auto state = core_.state();
    const double factor = config_.realtime_factor > 0 ? config_.realtime_factor : 1.0;
    step_period_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(state.timestep_s / factor, 1e-6)));
    next_step_ = now;
    next_status_ = now;
    next_status_allowed_ = now;
    next_viewer_sync_ = now;
    next_transport_sample_ = now;
    summary_wall_anchor_ = now;
    summary_simulation_anchor_ = state.simulation_time_s;
    summary_tick_anchor_ = state.current_tick_id;
    next_summary_ = config_.log_summary_period_ms
                        ? now + std::chrono::milliseconds(config_.log_summary_period_ms)
                        : Clock::time_point::max();

    while (!stop_requested_.load(std::memory_order_acquire) && !stop.stop_requested() &&
           eCAL::Ok()) {
      const bool unthrottled = config_.pacing_mode == "unthrottled";
      auto deadline = unthrottled ? Clock::now() : std::min(next_step_, Clock::now() + kPollPeriod);
      for (auto& event : queue_.WaitUntil(deadline)) {
        HandleEvent(std::move(event));
      }
      now = Clock::now();
      DrainControllerInputs(now);
      ApplyEffects(core_.CheckDeadlines(now));
      if (unthrottled || now >= next_step_) {
        ApplyEffects(core_.Step());
        if (unthrottled) {
          next_step_ = now;
        } else if (now - next_step_ > step_period_) {
          next_step_ = now + step_period_;
        } else {
          next_step_ += step_period_;
        }
      }
      ProcessTimers(Clock::now());
    }
    int final_code = 1;
    std::string final_reason = "ecal_stopped";
    {
      std::lock_guard lock(lifecycle_mutex_);
      if (stop_requested_.load(std::memory_order_relaxed)) {
        final_code = exit_code_.load(std::memory_order_relaxed);
        final_reason = stop_reason_;
      }
    }
    Finish(final_code, std::move(final_reason));
  } catch (const std::exception& error) {
    LogLoopBoundaryFailure(logger_, "run", error.what());
    Finish(2, "run_loop_exception");
  } catch (...) {
    LogLoopBoundaryFailure(logger_, "run", "non-standard exception");
    Finish(2, "run_loop_exception");
  }
}

void SimulationLoop::HandleEvent(ServerEvent event) {
  std::visit([this](auto value) { Handle(std::move(value)); }, std::move(event));
}

void SimulationLoop::Handle(GetAvailableResourcesEvent event) {
  event.completion.set_value(ToWire(core_.GetAvailableResources({})));
}

void SimulationLoop::Handle(RegisterAdapterEvent event) {
  if (stop_requested_.load(std::memory_order_acquire)) {
    event.completion.set_value(
        QueueError<mujoco_sim::v4::RegisterAdapterResponse>("server is stopping"));
    return;
  }
  auto request = ToDomain(event.request);
  if (auto error = ObserverDeliveryError(request, config_, core_.state().timestep_s)) {
    auto response = QueueError<mujoco_sim::v4::RegisterAdapterResponse>(std::move(*error));
    response.set_code(mujoco_sim::v4::INVALID_ARGUMENT);
    event.completion.set_value(std::move(response));
    return;
  }
  constexpr std::uint64_t kPendingSessionId = 0;
  auto [pending_entry, reserved] =
      sessions_.try_emplace(kPendingSessionId, SessionInfo{request.adapter_id(), request.role()});
  if (!reserved) {
    event.completion.set_value(QueueError<mujoco_sim::v4::RegisterAdapterResponse>(
        "adapter session reservation is already active"));
    RequestStop("adapter_session_reservation_failed");
    return;
  }
  auto pending_session = sessions_.extract(pending_entry);

  auto result = RegisterAdapterAtomically(
      &core_, &command_inbox_, request, Clock::now(),
      [this](std::string_view adapter_id, mujoco_sim::core::AdapterRole role) {
        PublisherPreparation preparation;
        preparation.ready = transport_->PrepareAdapterPublisher(
            adapter_id, static_cast<mujoco_sim::v4::AdapterRole>(role), &preparation.error,
            &preparation.created);
        return preparation;
      },
      [this](std::string_view adapter_id, mujoco_sim::core::AdapterRole role) {
        transport_->RemoveAdapterPublisher(adapter_id,
                                           static_cast<mujoco_sim::v4::AdapterRole>(role));
      });
  auto response = result.response;
  if (response.code() == mujoco_sim::core::OK) {
    pending_session.key() = response.session_id();
    if (!sessions_.insert(std::move(pending_session)).inserted) {
      ApplyEffects(std::move(result.effects));
      event.completion.set_value(QueueError<mujoco_sim::v4::RegisterAdapterResponse>(
          "failed to commit adapter session metadata"));
      RequestStop("adapter_session_commit_failed");
      return;
    }
  }
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(ToWire(response));
}

void SimulationLoop::Handle(ActivateAdapterEvent event) {
  mujoco_sim::core::ActivateAdapterRequest request;
  request.set_session_id(event.request.session_id());
  request.set_epoch(event.request.epoch());
  auto result = core_.ActivateAdapter(request, Clock::now());
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(ToWire<mujoco_sim::v4::ActivateAdapterResponse>(result.response));
}

void SimulationLoop::Handle(UnregisterAdapterEvent event) {
  mujoco_sim::core::UnregisterAdapterRequest request;
  request.set_session_id(event.request.session_id());
  request.set_epoch(event.request.epoch());
  auto result = core_.UnregisterAdapter(request);
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(ToWire<mujoco_sim::v4::UnregisterAdapterResponse>(result.response));
}

void SimulationLoop::Handle(GetSensorStreamsEvent event) {
  event.completion.set_value(sensor_runtime_->GetCatalog(event.request));
}

void SimulationLoop::Handle(HeartbeatEvent event) {
  ApplyEffects(core_.AcceptHeartbeat(event.message, Clock::now()));
}

void SimulationLoop::DrainControllerInputs(Clock::time_point now) {
  for (auto& command : command_inbox_.Drain()) {
    ApplyEffects(core_.SubmitCommand(command, now).effects);
  }
  for (auto& fault : command_inbox_.DrainFaults()) {
    ApplyEffects(core_.SubmitFault(fault).effects);
  }
}

void SimulationLoop::ApplyEffects(mujoco_sim::core::EngineEffects effects) {
  if (effects.status_urgency == mujoco_sim::core::EngineEffects::StatusUrgency::kLifecycle ||
      effects.status_urgency == mujoco_sim::core::EngineEffects::StatusUrgency::kTerminal) {
    status_dirty_ = true;
  }

  bool has_tombstone = false;
  for (const auto& change : effects.adapter_changes) {
    has_tombstone = has_tombstone || change.new_state == mujoco_sim::core::UNREGISTERED;
  }
  if (has_tombstone) {
    auto status = ToWire(core_.MakeStatus());
    for (const auto& change : effects.adapter_changes) {
      if (change.new_state != mujoco_sim::core::UNREGISTERED) {
        continue;
      }
      auto* tombstone = status.add_adapters();
      tombstone->set_adapter_id(change.adapter_id);
      tombstone->set_role(static_cast<mujoco_sim::v4::AdapterRole>(change.role));
      tombstone->set_session_id(change.session_id);
      tombstone->set_state(mujoco_sim::v4::UNREGISTERED);
      tombstone->set_message(change.message);
      tombstone->set_state_since_tick_id(change.tick_id);
    }
    sensor_runtime_->AppendStatus(&status);
    ReportPublishResult(logger_, warning_limiter_, {"ServerStatus", {}, status.tick_id(), 0},
                        transport_->PublishStatus(status));
  }

  for (const auto& change : effects.adapter_changes) {
    logger_->info(
        "event={} adapter_id={} role={} session_id={} old_state={} "
        "new_state={} tick_id={} message={}",
        AdapterEventName(change.kind), LogValue(change.adapter_id), static_cast<int>(change.role),
        change.session_id, static_cast<int>(change.old_state), static_cast<int>(change.new_state),
        change.tick_id, LogValue(change.message));
    if (change.new_state == mujoco_sim::core::UNREGISTERED) {
      command_inbox_.Remove(change.session_id);
      sessions_.erase(change.session_id);
      const bool has_replacement =
          std::any_of(sessions_.begin(), sessions_.end(), [&](const auto& item) {
            return item.second.adapter_id == change.adapter_id && item.second.role == change.role;
          });
      if (!has_replacement) {
        transport_->RemoveAdapterPublisher(change.adapter_id,
                                           static_cast<mujoco_sim::v4::AdapterRole>(change.role));
      }
    }
  }
  if (effects.step_completed) {
    ApplyCompletedStep(std::move(*effects.step_completed));
  }
}

void SimulationLoop::ApplyCompletedStep(mujoco_sim::core::StepCompletedEvent event) {
  sensor_runtime_->OnStepCompleted(core_);
  sensor_runtime_->CompleteBlockingEnqueues(run_stop_token_);
  PublishControllerStates(event.controller_states);
  PublishObservations(event.observer_frames);
}

void SimulationLoop::PublishControllerStates(
    const std::vector<mujoco_sim::core::ControllerStateFrame>& frames) {
  for (const auto& frame : frames) {
    auto session = sessions_.find(frame.identity.session.session_id);
    if (session == sessions_.end()) {
      continue;
    }
    std::string bytes;
    mujoco_sim::protocol::DataFrameError error;
    auto header = Header(mujoco_sim::protocol::DataFrameKind::kControllerState, frame.identity,
                         frame.sequence);
    const PublishDiagnosticContext context{"ControllerState", session->second.adapter_id,
                                           frame.identity.tick_id, frame.sequence};
    if (mujoco_sim::protocol::EncodeDataFrame(header, frame.observations, &bytes, &error)) {
      ReportPublishResult(logger_, warning_limiter_, context,
                          transport_->PublishControllerState(session->second.adapter_id, bytes));
    } else {
      ReportDataFrameEncodingFailure(logger_, warning_limiter_, context, error);
    }
  }
}

void SimulationLoop::PublishObservations(
    const std::vector<mujoco_sim::core::ObservationFrame>& frames) {
  for (const auto& frame : frames) {
    auto session = sessions_.find(frame.identity.session.session_id);
    if (session == sessions_.end()) {
      continue;
    }
    std::string bytes;
    mujoco_sim::protocol::DataFrameError error;
    auto header =
        Header(mujoco_sim::protocol::DataFrameKind::kObservation, frame.identity, frame.sequence);
    const PublishDiagnosticContext context{"Observation", session->second.adapter_id,
                                           frame.identity.tick_id, frame.sequence};
    if (mujoco_sim::protocol::EncodeDataFrame(header, frame.observations, &bytes, &error)) {
      ReportPublishResult(logger_, warning_limiter_, context,
                          transport_->PublishObservation(session->second.adapter_id, bytes));
    } else {
      ReportDataFrameEncodingFailure(logger_, warning_limiter_, context, error);
    }
  }
}

void SimulationLoop::PublishStatus(bool, bool) {
  auto status = ToWire(core_.MakeStatus());
  sensor_runtime_->AppendStatus(&status);
  ReportPublishResult(logger_, warning_limiter_, {"ServerStatus", {}, status.tick_id(), 0},
                      transport_->PublishStatus(status));
}

void SimulationLoop::PublishSummary(Clock::time_point now,
                                    const mujoco_sim::core::CoreState& state) {
  const double wall_s = std::chrono::duration<double>(now - summary_wall_anchor_).count();
  const double simulation_s = state.simulation_time_s - summary_simulation_anchor_;
  const auto steps = state.current_tick_id - summary_tick_anchor_;
  const double rate = wall_s > 0.0 ? simulation_s / wall_s : 0.0;
  const double step_rate = wall_s > 0.0 ? static_cast<double>(steps) / wall_s : 0.0;
  logger_->info(
      "event=server_summary tick_id={} interval_steps={} step_rate_hz={} "
      "simulation_time_s={} interval_simulation_s={} interval_wall_s={} "
      "realtime_factor_rate={} target_realtime_factor={} pacing_mode={} "
      "active_controllers={} online_controllers={} offline_controllers={} "
      "error_controllers={} online_observers={} transport_endpoints={} "
      "malformed_messages={}",
      state.current_tick_id, steps, step_rate, state.simulation_time_s, simulation_s, wall_s, rate,
      config_.realtime_factor, config_.pacing_mode, state.active_controller_count,
      state.controller_count(mujoco_sim::core::ONLINE),
      state.controller_count(mujoco_sim::core::OFFLINE),
      state.controller_count(mujoco_sim::core::ERROR),
      state.observer_count(mujoco_sim::core::ONLINE), transport_endpoint_count_,
      malformed_pending_.exchange(0));
  summary_wall_anchor_ = now;
  summary_simulation_anchor_ = state.simulation_time_s;
  summary_tick_anchor_ = state.current_tick_id;
  next_summary_ = now + std::chrono::milliseconds(config_.log_summary_period_ms);
}

void SimulationLoop::ProcessTimers(Clock::time_point now) {
  auto state = core_.state();
  if (config_.max_ticks && state.current_tick_id >= config_.max_ticks) {
    RequestStop("max_ticks");
    return;
  }
  if (viewer_ && now >= next_viewer_sync_) {
    std::string error;
    if (!viewer_->Synchronize(core_, *viewer_snapshot_, &error) && !viewer_->close_requested()) {
      {
        std::lock_guard lock(lifecycle_mutex_);
        exit_code_.store(2, std::memory_order_relaxed);
      }
      RequestStop("viewer_sync_failed");
      return;
    }
    next_viewer_sync_ = now + std::chrono::milliseconds(16);
  }
  if (now >= next_transport_sample_) {
    transport_endpoint_count_ = transport_->SampleSubscriberCounts();
    next_transport_sample_ =
        now + std::chrono::milliseconds(config_.transport_subscriber_sample_period_ms);
  }
  const bool periodic_status_due = now >= next_status_;
  const bool lifecycle_status_due = status_dirty_ && now >= next_status_allowed_;
  if (periodic_status_due || lifecycle_status_due) {
    PublishStatus(status_dirty_);
    status_dirty_ = false;
    next_status_allowed_ = now + std::chrono::milliseconds(config_.status_min_interval_ms);
    if (periodic_status_due) {
      next_status_ = now + std::chrono::milliseconds(config_.status_period_ms);
    }
  }
  if (now >= next_summary_) {
    PublishSummary(now, state);
  }
}

void SimulationLoop::Finish(int code, std::string reason) noexcept {
  int final_code = code;
  auto finish_stage = [this, &final_code](std::string_view operation, auto&& action) noexcept {
    std::string error;
    if (!RunWithExceptionBoundary(std::forward<decltype(action)>(action), &error)) {
      final_code = 2;
      LogLoopBoundaryFailure(logger_, operation, error.empty() ? "non-standard exception" : error);
    }
  };

  finish_stage("command_inbox_close", [this] { command_inbox_.Close(); });
  finish_stage("event_queue_close", [this] { queue_.Close("server simulation loop stopped"); });
  finish_stage("sensor_runtime_stop", [this] { sensor_runtime_->Stop(); });
  finish_stage("core_mark_stopped", [this, &reason] { ApplyEffects(core_.MarkStopped(reason)); });
  finish_stage("terminal_status_publish", [this] { PublishStatus(true, true); });
  finish_stage("server_stopped_log", [this, &reason, &final_code] {
    logger_->info("event=server_stopped reason={} exit_code={} tick_id={}", LogValue(reason),
                  final_code, core_.state().current_tick_id);
  });

  std::string state_error;
  if (!RunWithExceptionBoundary(
          [this, &reason] {
            std::lock_guard lock(lifecycle_mutex_);
            stop_reason_ = std::move(reason);
          },
          &state_error)) {
    final_code = 2;
    LogLoopBoundaryFailure(logger_, "record_stop_reason",
                           state_error.empty() ? "non-standard exception" : state_error);
  }
  exit_code_.store(final_code, std::memory_order_release);
}

}  // namespace mujoco_sim::server
