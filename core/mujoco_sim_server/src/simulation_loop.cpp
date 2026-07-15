#include "simulation_loop.hpp"

#include <ecal/ecal.h>
#include <spdlog/logger.h>
#include <mujoco_sim/protocol/data_frame.hpp>

#include "sensor_runtime.hpp"
#include "server_logging.hpp"
#include "sim_viewer.hpp"

#include <algorithm>
#include <bit>
#include <future>
#include <iterator>
#include <string_view>
#include <utility>

namespace mujoco_sim::server {
namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kPollPeriod = std::chrono::milliseconds(5);

template <typename Response>
Response QueueError(std::string message) {
  Response response;
  response.set_code(mujoco_sim::v3::INTERNAL_ERROR);
  response.set_message(std::move(message));
  return response;
}

std::string ErrorCodeName(mujoco_sim::core::ErrorCode code) {
  return mujoco_sim::v3::ErrorCode_Name(
      static_cast<mujoco_sim::v3::ErrorCode>(code));
}

std::string ServerStateName(mujoco_sim::core::ServerState state) {
  return mujoco_sim::v3::ServerState_Name(
      static_cast<mujoco_sim::v3::ServerState>(state));
}

void CopyDescriptor(const mujoco_sim::core::ResourceDescriptor& source,
                    mujoco_sim::v3::ResourceDescriptor* target) {
  target->set_name(source.name());
  target->set_type(static_cast<mujoco_sim::v3::ResourceType>(source.type()));
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
  for (std::size_t index = 0; index < 8; ++index) {
    bytes[index] = static_cast<char>((fingerprint.low >> (index * 8U)) & 0xffU);
    bytes[index + 8] =
        static_cast<char>((fingerprint.high >> (index * 8U)) & 0xffU);
  }
  return bytes;
}

void CopyLayout(const mujoco_sim::core::ResourceLayout& source,
                mujoco_sim::v3::ResourceLayout* target) {
  for (std::size_t index = 0; index < source.observations_.size(); ++index) {
    auto* slot = target->add_observations();
    CopyDescriptor(source.observations_[index], slot->mutable_resource());
    slot->set_scalar_offset(source.observation_spans[index].offset);
  }
  for (std::size_t index = 0; index < source.commands_.size(); ++index) {
    auto* slot = target->add_commands();
    CopyDescriptor(source.commands_[index], slot->mutable_resource());
    slot->set_scalar_offset(source.command_spans[index].offset);
  }
  target->set_layout_fingerprint(FingerprintBytes(source.fingerprint));
  target->set_observation_scalar_count(source.observation_scalar_count);
  target->set_command_scalar_count(source.command_scalar_count);
}

mujoco_sim::v3::GetAvailableResourcesResponse ToWire(
    const mujoco_sim::core::GetAvailableResourcesResponse& source) {
  mujoco_sim::v3::GetAvailableResourcesResponse target;
  target.set_code(static_cast<mujoco_sim::v3::ErrorCode>(source.code()));
  target.set_message(source.message());
  target.set_epoch(source.epoch());
  for (const auto& value : source.observations())
    CopyDescriptor(value, target.add_observations());
  for (const auto& value : source.commands())
    CopyDescriptor(value, target.add_commands());
  return target;
}

mujoco_sim::core::RegisterAdapterRequest ToDomain(
    const mujoco_sim::v3::RegisterAdapterRequest& source) {
  mujoco_sim::core::RegisterAdapterRequest target;
  target.set_adapter_id(source.adapter_id());
  target.set_role(static_cast<mujoco_sim::core::AdapterRole>(source.role()));
  for (const auto& name : source.observation_names()) target.add_observation_names(name);
  for (const auto& name : source.command_names()) target.add_command_names(name);
  target.set_commit_timeout_ms(source.commit_timeout_ms());
  target.set_session_timeout_ms(source.session_timeout_ms());
  if (source.has_delivery()) {
    if (source.delivery().has_every_n_steps())
      target.mutable_delivery()->set_every_n_steps(source.delivery().every_n_steps());
    else if (source.delivery().has_period_s())
      target.mutable_delivery()->set_period_s(source.delivery().period_s());
  }
  return target;
}

mujoco_sim::v3::RegisterAdapterResponse ToWire(
    const mujoco_sim::core::RegisterAdapterResponse& source) {
  mujoco_sim::v3::RegisterAdapterResponse target;
  target.set_code(static_cast<mujoco_sim::v3::ErrorCode>(source.code()));
  target.set_message(source.message());
  target.set_adapter_id(source.adapter_id());
  target.set_role(static_cast<mujoco_sim::v3::AdapterRole>(source.role()));
  target.set_session_id(source.session_id());
  target.set_epoch(source.epoch());
  target.set_state(static_cast<mujoco_sim::v3::ControllerState>(source.state()));
  CopyLayout(source.layout(), target.mutable_layout());
  return target;
}

template <typename WireResponse>
WireResponse ToWire(const mujoco_sim::core::AdapterSessionResponse& source) {
  WireResponse target;
  target.set_code(static_cast<mujoco_sim::v3::ErrorCode>(source.code()));
  target.set_message(source.message());
  target.set_state(static_cast<mujoco_sim::v3::ControllerState>(source.state()));
  return target;
}

mujoco_sim::v3::ServerStatus ToWire(
    const mujoco_sim::core::ServerStatus& source) {
  mujoco_sim::v3::ServerStatus target;
  target.set_epoch(source.epoch);
  target.set_tick_id(source.tick_id);
  target.set_simulation_time_s(source.simulation_time_s);
  target.set_state(static_cast<mujoco_sim::v3::ServerState>(source.state));
  target.set_message(source.message);
  for (const auto& value : source.adapters) {
    auto* adapter = target.add_adapters();
    adapter->set_adapter_id(value.adapter_id);
    adapter->set_role(static_cast<mujoco_sim::v3::AdapterRole>(value.role));
    adapter->set_session_id(value.session_id);
    adapter->set_state(static_cast<mujoco_sim::v3::ControllerState>(value.state));
    adapter->set_message(value.message);
    adapter->set_state_since_tick_id(value.state_since_tick_id);
    adapter->set_timeout_count(value.timeout_count);
    if (value.last_delivery_tick_id)
      adapter->set_last_delivery_tick_id(*value.last_delivery_tick_id);
    adapter->set_last_observation_sequence(value.last_observation_sequence);
    if (value.last_heartbeat_tick_id)
      adapter->set_last_heartbeat_tick_id(*value.last_heartbeat_tick_id);
    if (value.last_commit_tick_id)
      adapter->set_last_commit_tick_id(*value.last_commit_tick_id);
    adapter->set_required_for_barrier(value.required_for_barrier);
  }
  return target;
}

mujoco_sim::protocol::DataFrameHeader DataHeader(
    mujoco_sim::protocol::DataFrameKind kind,
    const mujoco_sim::core::FrameIdentity& identity) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = kind;
  header.session_id = identity.session.session_id;
  header.epoch = identity.session.epoch;
  header.tick_id = identity.tick_id;
  header.layout = {identity.layout.low, identity.layout.high};
  header.simulation_time_s = identity.simulation_time_s;
  return header;
}

}  // namespace

SimulationLoop::SimulationLoop(ServerConfig config,
                               std::shared_ptr<spdlog::logger> logger,
                               WarningLimiter* warning_limiter,
                               EcalServerTransport* transport,
                               SimViewer* viewer)
    : config_(std::move(config)),
      logger_(std::move(logger)),
      warning_limiter_(warning_limiter),
      transport_(transport),
      viewer_(viewer),
      core_(mujoco_sim::core::CoreOptions{
          config_.server_name, config_.tick_timeout_ms,
          config_.session_timeout_ms,
          config_.activation_timeout_ms,
          config_.allow_timed_out_adapter_replacement}),
      sensor_runtime_(std::make_unique<SensorRuntime>()) {}

SimulationLoop::~SimulationLoop() {
  RequestStop("server_destroyed");
  if (thread_.joinable()) thread_.join();
}

bool SimulationLoop::Initialize(std::string* error_message) {
  if (error_message == nullptr || transport_ == nullptr || logger_ == nullptr) {
    if (error_message) *error_message = "simulation loop dependencies are invalid";
    return false;
  }
  auto loaded = core_.LoadModel(
      config_.model_path,
      mujoco_sim::core::ModelLoadOptions{config_.initial_keyframe});
  if (!loaded.response.loaded) {
    *error_message = std::move(loaded.response.message);
    return false;
  }
  ApplyEffects(std::move(loaded.effects));
  const auto resources = core_.resources();
  const auto state = core_.state();
  logger_->info(
      "event=model_loaded model_path={} observations={} commands={} "
      "simulation_time_s={:.6f}",
      LogValue(config_.model_path), resources.observations.size(),
      resources.commands.size(), state.simulation_time_s);
  if (!sensor_runtime_->Initialize(core_, config_, error_message)) return false;
  logger_->info("event=sensor_runtime_initialized streams={}",
                config_.sensor_streams.size());
  if (viewer_ != nullptr) {
    viewer_snapshot_ = core_.CreateSnapshot(error_message);
    if (!viewer_snapshot_ ||
        !viewer_->QueueLoad(*viewer_snapshot_, config_.model_path,
                            error_message)) {
      return false;
    }
  }
  initialized_ = true;
  return true;
}

template <typename Event, typename Request>
typename Event::response_type SimulationLoop::SubmitRpc(Request request) {
  using Response = typename Event::response_type;
  Event event;
  event.request = std::move(request);
  auto response = event.completion.get_future();
  if (!queue_.TryPush(ServerEvent(std::move(event)))) {
    RecordQueueRejection("rpc");
    return QueueError<Response>(queue_.closed() ? "server is stopping"
                                                : "server event queue is full");
  }
  try {
    return response.get();
  } catch (const std::exception& exception) {
    return QueueError<Response>(std::string("server event failed: ") +
                                exception.what());
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
  handlers.commit = [this](auto commit) {
    if (!queue_.TryPush(CommitEvent{std::move(commit)})) {
      RecordQueueRejection("AdapterCommit");
    }
  };
  handlers.heartbeat = [this](auto heartbeat) {
    if (!queue_.TryPush(HeartbeatEvent{std::move(heartbeat)})) {
      RecordQueueRejection("AdapterHeartbeat");
    }
  };
  handlers.fault = [this](auto fault) {
    if (!queue_.TryPush(ControllerFaultEvent{std::move(fault)})) {
      RecordQueueRejection("ControllerFault");
    }
  };
  handlers.malformed = [this](std::string_view operation, std::size_t bytes) {
    RecordMalformed(operation, bytes);
  };
  return handlers;
}

bool SimulationLoop::Start(std::string* error_message) {
  if (error_message == nullptr || !initialized_ || thread_.joinable()) {
    if (error_message) *error_message = "simulation loop is not ready to start";
    return false;
  }
  try {
    thread_ = std::jthread(
        [this](std::stop_token stop_token) { Run(stop_token); });
  } catch (const std::exception& exception) {
    *error_message = std::string("failed to start simulation loop: ") +
                     exception.what();
    return false;
  }
  return true;
}

void SimulationLoop::RequestStop(std::string reason) {
  bool first = false;
  {
    std::lock_guard lock(lifecycle_mutex_);
    if (!stop_requested_.exchange(true)) {
      stop_reason_ = std::move(reason);
      if (exit_code_ != 2) exit_code_ = 0;
      first = true;
    }
  }
  if (first) {
    queue_.Close("server is stopping");
    if (thread_.joinable()) thread_.request_stop();
    if (sensor_runtime_) sensor_runtime_->RequestStop();
    if (viewer_) viewer_->RequestClose();
  }
}

int SimulationLoop::Join() {
  if (thread_.joinable()) thread_.join();
  std::lock_guard lock(lifecycle_mutex_);
  return exit_code_;
}

void SimulationLoop::Run(std::stop_token stop_token) noexcept {
  try {
    const auto started_at = Clock::now();
    const auto state = core_.state();
    step_period_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(state.timestep_s, 1e-6)));
    next_tick_ = started_at;
    next_status_ = started_at;
    next_status_allowed_ = started_at;
    next_viewer_sync_ = started_at;
    next_subscriber_sample_ = started_at;
    metrics_.Initialize(started_at, state.current_tick_id,
                        state.simulation_time_s);
    logger_->info("event=run_loop_started");

    while (!stop_requested_.load() && !stop_token.stop_requested() &&
           eCAL::Ok()) {
      auto batch = queue_.WaitUntil(NextWakeDeadline(Clock::now()));
      for (auto& event : batch) {
        HandleEvent(std::move(event));
        if (stop_requested_.load() || stop_token.stop_requested()) break;
      }
      if (!stop_requested_.load() && !stop_token.stop_requested()) {
        ProcessTimers(Clock::now());
      }
    }

    std::string reason;
    int code = 0;
    {
      std::lock_guard lock(lifecycle_mutex_);
      reason = stop_requested_.load() ? stop_reason_ : "ecal_stopped";
      code = stop_requested_.load() ? exit_code_ : 1;
    }
    Finish(code, std::move(reason));
  } catch (const std::exception& exception) {
    logger_->error("event=simulation_loop_exception error={}",
                   LogValue(exception.what()));
    Finish(2, "run_loop_exception");
  } catch (...) {
    logger_->error("event=simulation_loop_exception error=non-standard");
    Finish(2, "run_loop_exception");
  }
}

void SimulationLoop::HandleEvent(ServerEvent event) {
  std::visit([this](auto value) { Handle(std::move(value)); },
             std::move(event));
}

void SimulationLoop::Handle(GetAvailableResourcesEvent event) {
  event.completion.set_value(ToWire(core_.GetAvailableResources({})));
}

void SimulationLoop::Handle(RegisterAdapterEvent event) {
  if (event.request.role() == mujoco_sim::v3::OBSERVER) {
    std::optional<std::uint32_t> every_n_steps;
    std::optional<double> period_s;
    if (event.request.has_delivery()) {
      if (event.request.delivery().has_every_n_steps()) {
        every_n_steps = event.request.delivery().every_n_steps();
      } else if (event.request.delivery().has_period_s()) {
        period_s = event.request.delivery().period_s();
      }
    }
    std::string cadence_error;
    if (!ValidateObserverCadenceRate(
            every_n_steps, period_s, core_.state().timestep_s,
            config_.observer_max_rate_hz, config_.allow_every_step_observer,
            &cadence_error)) {
      mujoco_sim::v3::RegisterAdapterResponse response;
      response.set_code(mujoco_sim::v3::INVALID_ARGUMENT);
      response.set_message(std::move(cadence_error));
      event.completion.set_value(std::move(response));
      return;
    }
  }
  auto result = core_.RegisterAdapter(ToDomain(event.request));
  auto& response = result.response;
  if (response.code() == mujoco_sim::core::OK) {
    std::string publisher_error;
    if (!transport_->PrepareAdapterPublisher(
            response.adapter_id(),
            static_cast<mujoco_sim::v3::AdapterRole>(response.role()),
            &publisher_error)) {
      logger_->error(
          "event=adapter_publisher_prepare_failed adapter_id={} error={}",
          LogValue(response.adapter_id()), LogValue(publisher_error));
      ApplyEffects(core_.MarkFailed("adapter publisher creation failed: " +
                                    publisher_error));
    }
    const bool replaced = std::any_of(
        result.effects.adapter_changes.begin(),
        result.effects.adapter_changes.end(), [](const auto& change) {
          return change.kind ==
                 mujoco_sim::core::AdapterChangeKind::kReplaced;
        });
    logger_->info(
        "event=adapter_registered adapter_id={} role={} session_id={} "
        "epoch={} observations={} commands={} replaced={}",
        LogValue(response.adapter_id()),
        mujoco_sim::v3::AdapterRole_Name(
            static_cast<mujoco_sim::v3::AdapterRole>(response.role())),
        response.session_id(),
        response.epoch(), response.layout().observations_size(),
        response.layout().commands_size(), replaced);
  } else {
    logger_->warn(
        "event=adapter_request_rejected operation=register adapter_id={} "
        "role={} code={} message={}",
        LogValue(event.request.adapter_id()),
        mujoco_sim::v3::AdapterRole_Name(event.request.role()),
        ErrorCodeName(response.code()), LogValue(response.message()));
  }
  ApplyEffects(std::move(result.effects));
  if (response.code() == mujoco_sim::core::OK) {
    const std::uint32_t commit_timeout_ms =
        event.request.commit_timeout_ms() == 0
            ? config_.tick_timeout_ms
            : event.request.commit_timeout_ms();
    sessions_.insert_or_assign(response.session_id(),
                               SessionInfo{response.adapter_id(), response.role(),
                                           commit_timeout_ms});
  }
  event.completion.set_value(ToWire(response));
}

void SimulationLoop::Handle(ActivateAdapterEvent event) {
  const auto session = sessions_.find(event.request.session_id());
  mujoco_sim::core::ActivateAdapterRequest request;
  request.set_session_id(event.request.session_id());
  request.set_epoch(event.request.epoch());
  if (session != sessions_.end()) {
    request.set_adapter_id(session->second.adapter_id);
    request.set_role(session->second.role);
  }
  auto result = core_.ActivateAdapter(request);
  auto& response = result.response;
  if (response.code() == mujoco_sim::core::OK) {
    if (request.role() == mujoco_sim::core::CONTROLLER) {
      active_controllers_.insert(request.adapter_id());
      metrics_.RecordSubscriberQueries(transport_->SampleSubscriberCounts());
    }
    logger_->info(
        "event=adapter_activated adapter_id={} role={} session_id={} epoch={}",
        LogValue(request.adapter_id()),
        mujoco_sim::v3::AdapterRole_Name(static_cast<mujoco_sim::v3::AdapterRole>(request.role())),
        event.request.session_id(), event.request.epoch());
  } else {
    logger_->warn(
        "event=adapter_request_rejected operation=activate adapter_id={} "
        "role={} code={} message={}",
        LogValue(request.adapter_id()),
        mujoco_sim::v3::AdapterRole_Name(static_cast<mujoco_sim::v3::AdapterRole>(request.role())),
        ErrorCodeName(response.code()), LogValue(response.message()));
  }
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(
      ToWire<mujoco_sim::v3::ActivateAdapterResponse>(response));
}

void SimulationLoop::Handle(UnregisterAdapterEvent event) {
  const auto session = sessions_.find(event.request.session_id());
  mujoco_sim::core::UnregisterAdapterRequest request;
  request.set_session_id(event.request.session_id());
  request.set_epoch(event.request.epoch());
  if (session != sessions_.end()) {
    request.set_adapter_id(session->second.adapter_id);
    request.set_role(session->second.role);
  }
  auto result = core_.UnregisterAdapter(request);
  const bool stepped = result.effects.step_completed.has_value();
  if (result.response.code() == mujoco_sim::core::OK) {
    logger_->info(
        "event=adapter_unregistered adapter_id={} role={} session_id={} "
        "epoch={} stepped={}",
        LogValue(request.adapter_id()),
        mujoco_sim::v3::AdapterRole_Name(static_cast<mujoco_sim::v3::AdapterRole>(request.role())),
        event.request.session_id(), event.request.epoch(), stepped);
  } else {
    logger_->warn(
        "event=adapter_request_rejected operation=unregister adapter_id={} "
        "role={} code={} message={}",
        LogValue(request.adapter_id()),
        mujoco_sim::v3::AdapterRole_Name(static_cast<mujoco_sim::v3::AdapterRole>(request.role())),
        ErrorCodeName(result.response.code()),
        LogValue(result.response.message()));
  }
  ApplyEffects(std::move(result.effects));
  if (result.response.code() == mujoco_sim::core::OK) {
    sessions_.erase(event.request.session_id());
  }
  event.completion.set_value(
      ToWire<mujoco_sim::v3::UnregisterAdapterResponse>(result.response));
}

void SimulationLoop::Handle(GetSensorStreamsEvent event) {
  event.completion.set_value(sensor_runtime_->GetCatalog(event.request));
}

void SimulationLoop::Handle(CommitEvent event) {
  const auto validate_started = Clock::now();
  auto result = core_.SubmitCommit(event.message);
  const double validate_us =
      std::chrono::duration<double, std::micro>(Clock::now() - validate_started)
          .count();
  metrics_.RecordCommit(result.response.duplicate,
                        result.response.code != mujoco_sim::core::OK,
                        result.effects.step_completed.has_value(), Clock::now(),
                        validate_us);
  const auto session = sessions_.find(event.message.identity.session.session_id);
  const std::string adapter_id = session == sessions_.end()
                                     ? std::string("unknown")
                                     : session->second.adapter_id;
  if (result.response.code != mujoco_sim::core::OK) {
    logger_->warn(
        "event=commit_rejected adapter_id={} tick_id={} code={} message={}",
        LogValue(adapter_id), event.message.identity.tick_id,
        ErrorCodeName(result.response.code), LogValue(result.response.message));
  } else {
    tick_retries_.erase(adapter_id);
    logger_->trace("event=commit_accepted adapter_id={} tick_id={} stepped={}",
                   LogValue(adapter_id), event.message.identity.tick_id,
                   result.effects.step_completed.has_value());
  }
  ApplyEffects(std::move(result.effects));
}

void SimulationLoop::Handle(HeartbeatEvent event) {
  const auto session = sessions_.find(event.message.session.session_id);
  metrics_.RecordHeartbeat(
      session != sessions_.end() &&
      session->second.role == mujoco_sim::core::OBSERVER);
  ApplyEffects(core_.AcceptHeartbeat(event.message));
}

void SimulationLoop::Handle(ControllerFaultEvent event) {
  ApplyEffects(core_.SubmitFault(event.message).effects);
}

void SimulationLoop::ProcessTimers(Clock::time_point now) {
  ApplyEffects(core_.CheckDeadlines(now));

  if (now >= next_subscriber_sample_) {
    metrics_.RecordSubscriberQueries(transport_->SampleSubscriberCounts());
    next_subscriber_sample_ =
        now + std::chrono::milliseconds(
                  config_.transport_subscriber_sample_period_ms);
  }

  auto state = core_.state();
  if (state.server_state == mujoco_sim::core::ERROR_PAUSED) {
    {
      std::lock_guard lock(lifecycle_mutex_);
      exit_code_ = 2;
    }
    RequestStop("core_error_paused");
    return;
  }
  if (config_.max_ticks > 0 && state.current_tick_id >= config_.max_ticks) {
    {
      std::lock_guard lock(lifecycle_mutex_);
      exit_code_ = 0;
    }
    RequestStop("max_ticks");
    return;
  }

  if (!state.tick_open && now >= next_tick_ && TickSubscribersReady()) {
    auto ticks = OpenTicks();
    PublishTicks(ticks, false);
    InitializeTickRetries(ticks, now);
    ApplyEffects(core_.CompleteEmptyBarrier());
  } else if (state.tick_open) {
    PublishDueTickRetries(now);
  }

  if (viewer_ != nullptr && now >= next_viewer_sync_) {
    std::string error;
    if (!viewer_->Synchronize(core_, *viewer_snapshot_, &error) &&
        !viewer_->close_requested()) {
      ApplyEffects(core_.MarkFailed("viewer synchronization failed: " + error));
      {
        std::lock_guard lock(lifecycle_mutex_);
        exit_code_ = 2;
      }
      RequestStop("viewer_sync_failed");
      return;
    }
    next_viewer_sync_ = now + std::chrono::milliseconds(16);
  }

  state = core_.state();
  if (state.server_state != last_logged_state_) {
    logger_->info("event=server_state_changed old_state={} new_state={} message={}",
                  ServerStateName(last_logged_state_),
                  ServerStateName(state.server_state),
                  LogValue(core_.MakeStatus().message));
    last_logged_state_ = state.server_state;
  }
  if (now >= next_status_ ||
      (status_dirty_ && now >= next_status_allowed_)) {
    PublishStatus(status_dirty_);
    status_dirty_ = false;
    next_status_ = now + std::chrono::milliseconds(config_.status_period_ms);
    next_status_allowed_ =
        now + std::chrono::milliseconds(config_.status_min_interval_ms);
  }
  MaybeLogSummary(now, false);
}

void SimulationLoop::ApplyEffects(mujoco_sim::core::EngineEffects effects) {
  status_dirty_ |= effects.status_urgency ==
                       mujoco_sim::core::EngineEffects::StatusUrgency::kLifecycle ||
                   effects.status_urgency ==
                       mujoco_sim::core::EngineEffects::StatusUrgency::kTerminal;
  for (const auto& change : effects.adapter_changes) {
    if (change.role == mujoco_sim::core::CONTROLLER &&
        (change.kind == mujoco_sim::core::AdapterChangeKind::kTimedOut ||
         change.kind == mujoco_sim::core::AdapterChangeKind::kLocalError ||
         change.kind == mujoco_sim::core::AdapterChangeKind::kUnregistered ||
         change.kind == mujoco_sim::core::AdapterChangeKind::kReplaced ||
         change.kind == mujoco_sim::core::AdapterChangeKind::kServerFailed)) {
      active_controllers_.erase(change.adapter_id);
      const auto retry = tick_retries_.find(change.adapter_id);
      if (retry != tick_retries_.end()) {
        encoded_ticks_.erase(retry->second.issue_generation);
        tick_retries_.erase(retry);
      }
    }
    switch (change.kind) {
      case mujoco_sim::core::AdapterChangeKind::kTimedOut:
        metrics_.RecordTimeout();
        logger_->warn(
            "event=adapter_timed_out adapter_id={} session_id={} tick_id={} "
            "message={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id,
            LogValue(change.message));
        break;
      case mujoco_sim::core::AdapterChangeKind::kActivationExpired:
        metrics_.RecordActivationExpiration();
        logger_->warn(
            "event=adapter_activation_expired adapter_id={} session_id={} "
            "tick_id={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id);
        break;
      case mujoco_sim::core::AdapterChangeKind::kRecoveryPending:
        metrics_.RecordRecovery();
        logger_->info(
            "event=adapter_recovery_pending adapter_id={} session_id={} "
            "tick_id={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id);
        break;
      case mujoco_sim::core::AdapterChangeKind::kRecovered:
        logger_->info(
            "event=adapter_recovered adapter_id={} session_id={} tick_id={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id);
        break;
      case mujoco_sim::core::AdapterChangeKind::kLocalError:
        metrics_.RecordLocalError();
        logger_->error(
            "event=adapter_local_error adapter_id={} session_id={} tick_id={} "
            "state={} message={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id,
            mujoco_sim::v3::ControllerState_Name(
                static_cast<mujoco_sim::v3::ControllerState>(change.new_state)),
            LogValue(change.message));
        break;
      case mujoco_sim::core::AdapterChangeKind::kServerFailed:
        logger_->error(
            "event=adapter_server_failed adapter_id={} session_id={} "
            "tick_id={} state={} message={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id,
            mujoco_sim::v3::ControllerState_Name(
                static_cast<mujoco_sim::v3::ControllerState>(change.new_state)),
            LogValue(change.message));
        break;
      case mujoco_sim::core::AdapterChangeKind::kReplaced:
        metrics_.RecordReplacement();
        break;
      case mujoco_sim::core::AdapterChangeKind::kUnregistered:
        break;
      case mujoco_sim::core::AdapterChangeKind::kRegistered:
      case mujoco_sim::core::AdapterChangeKind::kActivated:
        break;
    }
  }
  if (effects.step_completed) {
    ApplyCompletedStep(std::move(*effects.step_completed));
  }
}

void SimulationLoop::ApplyCompletedStep(
    mujoco_sim::core::StepCompletedEvent event) {
  sensor_runtime_->OnStepCompleted(core_);
  const auto wait_started = Clock::now();
  sensor_runtime_->CompleteBlockingEnqueues(thread_.get_stop_token());
  metrics_.RecordBlockingWait(
      std::chrono::duration<double, std::milli>(Clock::now() - wait_started)
          .count());
  PublishObservations(event.observer_frames);
  if (event.source != mujoco_sim::core::StepCompletionSource::kCommit) {
    metrics_.RecordBarrierCompleted(Clock::now());
  }
  next_tick_ = Clock::now() + step_period_;
  tick_retries_.clear();
  encoded_ticks_.clear();
  logger_->debug(
      "event=scheduler_step_completed completed_tick_id={} next_tick_id={} "
      "simulation_time_s={:.6f} source={}",
      event.completed_tick_id, event.next_tick_id, event.simulation_time_s,
      static_cast<int>(event.source));
}

std::vector<mujoco_sim::core::IssuedTickHandle> SimulationLoop::OpenTicks() {
  const bool was_open = core_.state().tick_open;
  auto result = core_.OpenIssuedTicks();
  auto ticks = std::move(result.response);
  if (!was_open && core_.state().tick_open) {
    metrics_.RecordTickOpened(Clock::now());
  }
  ApplyEffects(std::move(result.effects));
  return ticks;
}

void SimulationLoop::PublishTicks(
    const std::vector<mujoco_sim::core::IssuedTickHandle>& ticks,
    bool retransmission) {
  std::uint64_t attempted = 0;
  std::uint64_t sent = 0;
  std::uint64_t rejected = 0;
  std::uint64_t no_subscribers = 0;
  std::uint64_t bytes_sent = 0;
  std::uint64_t cache_hits = 0;
  std::uint64_t cache_misses = 0;
  std::uint64_t serializations = 0;
  std::uint64_t cached_bytes = 0;
  double encode_us = 0.0;
  for (const auto& issued : ticks) {
    if (!core_.IsIssuedTickCurrent(issued->identity)) continue;
    auto encoded = encoded_ticks_.find(issued->identity.issue_generation);
    if (encoded == encoded_ticks_.end()) {
      ++cache_misses;
      auto header = DataHeader(mujoco_sim::protocol::DataFrameKind::kTick,
                               issued->frame.identity);
      if (issued->frame.recovery_proof) {
        header.flags = mujoco_sim::protocol::kDataFrameRecoveryProof;
      }
      std::string bytes;
      mujoco_sim::protocol::DataFrameError error;
      const auto encode_started = Clock::now();
      if (!mujoco_sim::protocol::EncodeDataFrame(
              header, issued->frame.observations, &bytes, &error)) {
        ApplyEffects(core_.MarkFailed("failed to encode Tick: " + error.message));
        continue;
      }
      encode_us += std::chrono::duration<double, std::micro>(
                       Clock::now() - encode_started)
                       .count();
      ++serializations;
      cached_bytes += bytes.size();
      encoded = encoded_ticks_
                    .emplace(issued->identity.issue_generation, std::move(bytes))
                    .first;
    } else {
      ++cache_hits;
    }
    const auto result = transport_->PublishTick(
        issued->identity.adapter_id, encoded->second);
    if (!result.attempted) continue;
    ++attempted;
    bytes_sent += result.bytes;
    const auto outcome = result.sent ? SendOutcome::kSent
                                     : SendOutcome::kRejectedWithSubscribers;
    if (outcome == SendOutcome::kSent) {
      ++sent;
      logger_->trace("event=tick_published adapter_id={} tick_id={} bytes={}",
                     LogValue(issued->identity.adapter_id),
                     issued->identity.tick_id, result.bytes);
    } else if (outcome == SendOutcome::kNoSubscribers) {
      ++no_subscribers;
    } else {
      ++rejected;
    }
  }
  metrics_.RecordTickPublishBatch(attempted, 0, sent, rejected,
                                  no_subscribers,
                                  retransmission ? attempted : 0U, bytes_sent);
  metrics_.RecordTickCache(serializations, cache_hits, cache_misses,
                           cached_bytes, encoded_ticks_.size(), 0, encode_us);
}

void SimulationLoop::InitializeTickRetries(
    const std::vector<mujoco_sim::core::IssuedTickHandle>& ticks,
    Clock::time_point now) {
  tick_retries_.clear();
  if (config_.tick_retry_max_attempts == 0) return;
  const auto initial =
      std::chrono::milliseconds(config_.tick_retry_initial_ms);
  for (const auto& tick : ticks) {
    if (!tick) continue;
    const auto session = sessions_.find(tick->frame.identity.session.session_id);
    const std::uint32_t commit_timeout_ms =
        session == sessions_.end() || session->second.commit_timeout_ms == 0
            ? config_.tick_timeout_ms
            : session->second.commit_timeout_ms;
    const auto deadline =
        now + std::chrono::milliseconds(commit_timeout_ms);
    const auto schedule = MakeTickRetrySchedule(now, deadline, initial);
    if (!schedule) continue;
    tick_retries_.emplace(
        tick->identity.adapter_id,
        TickRetryState{*schedule, tick->identity.issue_generation});
  }
}

void SimulationLoop::PublishDueTickRetries(Clock::time_point now) {
  auto ticks = OpenTicks();
  std::vector<mujoco_sim::core::IssuedTickHandle> due;
  due.reserve(ticks.size());
  for (const auto& tick : ticks) {
    if (!tick) continue;
    auto retry = tick_retries_.find(tick->identity.adapter_id);
    if (retry == tick_retries_.end() ||
        !AdvanceTickRetry(
            &retry->second.schedule, now, config_.tick_retry_max_attempts,
            config_.tick_retry_backoff_multiplier,
            std::chrono::milliseconds(config_.tick_retry_max_delay_ms))) {
      continue;
    }
    due.push_back(tick);
  }
  if (!due.empty()) PublishTicks(due, true);
}

void SimulationLoop::PublishObservations(
    const std::vector<mujoco_sim::core::ObservationFrame>& frames) {
  std::uint64_t published = 0;
  std::uint64_t bytes_published = 0;
  for (const auto& frame : frames) {
    const auto session = sessions_.find(frame.identity.session.session_id);
    if (session == sessions_.end()) continue;
    auto header = DataHeader(
        mujoco_sim::protocol::DataFrameKind::kObservation, frame.identity);
    header.sequence = frame.sequence;
    std::string bytes;
    mujoco_sim::protocol::DataFrameError error;
    if (!mujoco_sim::protocol::EncodeDataFrame(
            header, frame.observations, &bytes, &error)) {
      logger_->error(
          "event=message_serialize_failed message_type=ObservationFrame "
          "adapter_id={} sequence={}",
          LogValue(session->second.adapter_id), frame.sequence);
      continue;
    }
    const auto result =
        transport_->PublishObservation(session->second.adapter_id, bytes);
    if (result.attempted) {
      ++published;
      bytes_published += result.bytes;
    }
  }
  metrics_.RecordObservations(published, bytes_published);
}

void SimulationLoop::PublishStatus(bool lifecycle, bool terminal) {
  auto status = ToWire(core_.MakeStatus());
  sensor_runtime_->AppendStatus(&status);
  const auto result = transport_->PublishStatus(status);
  if (result.serialization_failed) {
    logger_->error("event=message_serialize_failed message_type=ServerStatus");
  }
  metrics_.RecordStatus(lifecycle, terminal);
}

bool SimulationLoop::TickSubscribersReady() const {
  return std::all_of(active_controllers_.begin(), active_controllers_.end(),
                     [this](const auto& adapter_id) {
                       return transport_->HasTickSubscriber(adapter_id);
                     });
}

void SimulationLoop::MaybeLogSummary(Clock::time_point now, bool final) {
  const auto malformed = malformed_pending_.exchange(0);
  for (std::uint64_t i = 0; i < malformed; ++i) {
    metrics_.RecordMalformedMessage();
  }
  if (config_.log_summary_period_ms == 0 || !metrics_.initialized() ||
      (!final && !metrics_.SummaryDue(
                     now, std::chrono::milliseconds(
                              config_.log_summary_period_ms)))) {
    return;
  }
  const auto state = core_.state();
  const auto summary = metrics_.TakeSummary(
      now, state.current_tick_id, state.simulation_time_s);
  logger_->info(
      "event=tick_summary final={} window_s={:.3f} state={} tick_id={} "
      "simulation_time_s={:.6f} ticks_opened={} commits={} timeouts={} "
      "recoveries={} replacements={} local_errors={} malformed_messages={} "
      "tick_frames={} tick_bytes={} tick_encode_max_us={:.3f} "
      "commit_validate_max_us={:.3f} observation_frames={} observation_bytes={} "
      "controller_heartbeats={} observer_heartbeats={} status_periodic={} "
      "status_lifecycle={} status_terminal={} subscriber_count_queries={} "
      "tick_cache_hits={} tick_cache_misses={} tick_cache_entries={} "
      "tick_serializations={} tick_retransmissions={} queue_rejections={} queue_high_watermark={} step_rate_hz={:.3f} "
      "real_time_factor={:.3f} barrier_latency_avg_ms={:.3f} "
      "barrier_latency_max_ms={:.3f}",
      final, summary.window_seconds, ServerStateName(state.server_state),
      summary.tick_id, summary.simulation_time, summary.ticks_opened,
      summary.commits, summary.timeouts, summary.recoveries,
      summary.replacements, summary.local_errors,
      summary.malformed_messages, summary.tick_publish_attempts,
      summary.tick_bytes, summary.tick_encode_max_us,
      summary.commit_validate_max_us, summary.observation_frames,
      summary.observation_bytes, summary.controller_heartbeats,
      summary.observer_heartbeats, summary.status_periodic,
      summary.status_lifecycle, summary.status_terminal,
      summary.subscriber_count_queries, summary.tick_cache_hits,
      summary.tick_cache_misses, summary.tick_cache_entries,
      summary.tick_serializations, summary.tick_retransmissions,
      queue_rejections_.load(),
      queue_.high_watermark(), summary.rates.step_rate_hz,
      summary.rates.real_time_factor, summary.rates.barrier_latency_avg_ms,
      summary.rates.barrier_latency_max_ms);
}

void SimulationLoop::RecordMalformed(std::string_view operation,
                                     std::size_t bytes) {
  ++malformed_pending_;
  std::uint64_t suppressed = 0;
  if (!ShouldLogWarning(std::string("malformed_") + std::string(operation),
                        &suppressed)) {
    return;
  }
  if (operation == "Commit" || operation == "SessionHeartbeat" ||
      operation == "ControllerFault") {
    logger_->warn(
        "event=topic_payload_malformed message_type={} bytes={} max_bytes={} "
        "suppressed={}",
        operation, bytes, kMaxProtocolPayloadBytes, suppressed);
  } else {
    logger_->warn(
        "event=service_request_malformed method={} bytes={} max_bytes={} "
        "suppressed={}",
        operation, bytes, kMaxProtocolPayloadBytes, suppressed);
  }
}

void SimulationLoop::RecordQueueRejection(std::string_view operation) {
  ++queue_rejections_;
  std::uint64_t suppressed = 0;
  if (ShouldLogWarning(std::string("queue_full_") + std::string(operation),
                       &suppressed)) {
    logger_->warn(
        "event=server_event_rejected operation={} reason={} queue_size={} "
        "queue_payload_bytes={} suppressed={}",
        operation, queue_.closed() ? "stopping" : "queue_full", queue_.size(),
        queue_.payload_bytes(), suppressed);
  }
}

bool SimulationLoop::ShouldLogWarning(std::string_view key,
                                      std::uint64_t* suppressed) {
  if (warning_limiter_ == nullptr || suppressed == nullptr) return true;
  const auto decision = warning_limiter_->Admit(key);
  *suppressed = decision.suppressed;
  return decision.emit;
}

Clock::time_point SimulationLoop::NextWakeDeadline(Clock::time_point now) const {
  auto deadline = now + kPollPeriod;
  deadline = std::min(deadline, next_status_);
  if (status_dirty_) deadline = std::min(deadline, next_status_allowed_);
  if (viewer_ != nullptr) deadline = std::min(deadline, next_viewer_sync_);
  deadline = std::min(deadline, next_subscriber_sample_);
  if (core_.state().tick_open) {
    for (const auto& [_, retry] : tick_retries_) {
      if (retry.schedule.attempts < config_.tick_retry_max_attempts) {
        deadline = std::min(deadline,
                            std::min(retry.schedule.next_retry,
                                     retry.schedule.deadline));
      }
    }
  } else {
    deadline = std::min(deadline, next_tick_);
  }
  return deadline;
}

void SimulationLoop::Finish(int exit_code, std::string reason) {
  queue_.Close("server stopped");
  sensor_runtime_->RequestStop();
  if (exit_code == 0) {
    ApplyEffects(core_.MarkStopped(reason));
  } else if (core_.state().server_state != mujoco_sim::core::ERROR_PAUSED) {
    ApplyEffects(core_.MarkFailed(reason));
  }
  PublishStatus(false, true);
  MaybeLogSummary(Clock::now(), true);
  if (viewer_) viewer_->RequestClose();
  {
    std::lock_guard lock(lifecycle_mutex_);
    exit_code_ = exit_code;
    stop_reason_ = std::move(reason);
  }
  const auto final_state = core_.state();
  if (exit_code == 0) {
    logger_->info("event=server_stopped reason={} exit_code={} tick_id={}",
                  stop_reason_, exit_code, final_state.current_tick_id);
  } else {
    logger_->error("event=server_stopped reason={} exit_code={} tick_id={}",
                   stop_reason_, exit_code, final_state.current_tick_id);
  }
}

}  // namespace mujoco_sim::server
