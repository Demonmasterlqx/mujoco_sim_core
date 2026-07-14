#include "simulation_loop.hpp"

#include <ecal/ecal.h>
#include <spdlog/logger.h>

#include "sensor_runtime.hpp"
#include "server_logging.hpp"
#include "sim_viewer.hpp"

#include <algorithm>
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
  response.set_code(mujoco_sim::v2::INTERNAL_ERROR);
  response.set_message(std::move(message));
  return response;
}

std::string ErrorCodeName(mujoco_sim::v2::ErrorCode code) {
  return mujoco_sim::v2::ErrorCode_Name(code);
}

std::string ServerStateName(mujoco_sim::v2::ServerState state) {
  return mujoco_sim::v2::ServerState_Name(state);
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
    next_retransmission_ = started_at;
    next_status_ = started_at;
    next_viewer_sync_ = started_at;
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
  event.completion.set_value(core_.GetAvailableResources(event.request));
}

void SimulationLoop::Handle(RegisterAdapterEvent event) {
  auto result = core_.RegisterAdapter(event.request);
  auto& response = result.response;
  if (response.code() == mujoco_sim::v2::OK) {
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
        mujoco_sim::v2::AdapterRole_Name(response.role()), response.session_id(),
        response.epoch(), response.layout().observations_size(),
        response.layout().commands_size(), replaced);
  } else {
    logger_->warn(
        "event=adapter_request_rejected operation=register adapter_id={} "
        "role={} code={} message={}",
        LogValue(event.request.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(event.request.role()),
        ErrorCodeName(response.code()), LogValue(response.message()));
  }
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(std::move(response));
}

void SimulationLoop::Handle(ActivateAdapterEvent event) {
  auto result = core_.ActivateAdapter(event.request);
  auto& response = result.response;
  if (response.code() == mujoco_sim::v2::OK) {
    logger_->info(
        "event=adapter_activated adapter_id={} role={} session_id={} epoch={}",
        LogValue(event.request.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(event.request.role()),
        event.request.session_id(), event.request.epoch());
  } else {
    logger_->warn(
        "event=adapter_request_rejected operation=activate adapter_id={} "
        "role={} code={} message={}",
        LogValue(event.request.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(event.request.role()),
        ErrorCodeName(response.code()), LogValue(response.message()));
  }
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(std::move(response));
}

void SimulationLoop::Handle(UnregisterAdapterEvent event) {
  auto result = core_.UnregisterAdapter(event.request);
  const bool stepped = result.effects.step_completed.has_value();
  if (result.response.code() == mujoco_sim::v2::OK) {
    logger_->info(
        "event=adapter_unregistered adapter_id={} role={} session_id={} "
        "epoch={} stepped={}",
        LogValue(event.request.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(event.request.role()),
        event.request.session_id(), event.request.epoch(), stepped);
  } else {
    logger_->warn(
        "event=adapter_request_rejected operation=unregister adapter_id={} "
        "role={} code={} message={}",
        LogValue(event.request.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(event.request.role()),
        ErrorCodeName(result.response.code()),
        LogValue(result.response.message()));
  }
  ApplyEffects(std::move(result.effects));
  event.completion.set_value(std::move(result.response));
}

void SimulationLoop::Handle(GetSensorStreamsEvent event) {
  event.completion.set_value(sensor_runtime_->GetCatalog(event.request));
}

void SimulationLoop::Handle(CommitEvent event) {
  auto result = core_.SubmitCommit(event.message);
  metrics_.RecordCommit(result.response.duplicate,
                        result.response.code != mujoco_sim::v2::OK,
                        result.effects.step_completed.has_value(), Clock::now());
  if (result.response.code != mujoco_sim::v2::OK) {
    logger_->warn(
        "event=commit_rejected adapter_id={} tick_id={} code={} message={}",
        LogValue(event.message.adapter_id()), event.message.header().tick_id(),
        ErrorCodeName(result.response.code), LogValue(result.response.message));
  } else {
    logger_->trace("event=commit_accepted adapter_id={} tick_id={} stepped={}",
                   LogValue(event.message.adapter_id()),
                   event.message.header().tick_id(),
                   result.effects.step_completed.has_value());
  }
  ApplyEffects(std::move(result.effects));
}

void SimulationLoop::Handle(HeartbeatEvent event) {
  ApplyEffects(core_.AcceptHeartbeat(event.message));
}

void SimulationLoop::ProcessTimers(Clock::time_point now) {
  ApplyEffects(core_.CheckDeadlines(now));

  auto state = core_.state();
  if (state.server_state == mujoco_sim::v2::ERROR_PAUSED) {
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

  if (!state.tick_open && now >= next_tick_) {
    auto ticks = OpenTicks();
    PublishTicks(ticks, false);
    next_retransmission_ = now + kPollPeriod;
    ApplyEffects(core_.CompleteEmptyBarrier());
  } else if (state.tick_open && now >= next_retransmission_) {
    PublishTicks(OpenTicks(), true);
    next_retransmission_ = now + kPollPeriod;
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
                  LogValue(core_.MakeStatus().message()));
    last_logged_state_ = state.server_state;
  }
  if (status_dirty_ || now >= next_status_) {
    PublishStatus();
    status_dirty_ = false;
    next_status_ = now + std::chrono::milliseconds(config_.status_period_ms);
  }
  MaybeLogSummary(now, false);
}

void SimulationLoop::ApplyEffects(mujoco_sim::core::EngineEffects effects) {
  status_dirty_ |= effects.status_changed;
  for (const auto& change : effects.adapter_changes) {
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
            mujoco_sim::v2::ControllerState_Name(change.new_state),
            LogValue(change.message));
        break;
      case mujoco_sim::core::AdapterChangeKind::kServerFailed:
        logger_->error(
            "event=adapter_server_failed adapter_id={} session_id={} "
            "tick_id={} state={} message={}",
            LogValue(change.adapter_id), change.session_id, change.tick_id,
            mujoco_sim::v2::ControllerState_Name(change.new_state),
            LogValue(change.message));
        break;
      case mujoco_sim::core::AdapterChangeKind::kReplaced:
        metrics_.RecordReplacement();
        break;
      case mujoco_sim::core::AdapterChangeKind::kUnregistered:
        transport_->EraseAdapterPublisher(change.adapter_id, change.role);
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
  status_dirty_ = true;
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
  std::uint64_t bytes = 0;
  std::uint64_t layout_bytes = 0;
  for (const auto& tick : ticks) {
    bytes += tick->serialized_bytes.size();
    layout_bytes += tick->layout_bytes_avoided;
  }
  metrics_.RecordTickCache(was_open ? 0U : ticks.size(),
                           was_open ? ticks.size() : 0U, 0U, bytes,
                           layout_bytes);
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
  for (const auto& issued : ticks) {
    if (!core_.IsIssuedTickCurrent(issued->identity)) continue;
    const auto result = transport_->PublishTick(issued);
    if (!result.attempted) continue;
    ++attempted;
    const auto outcome = ClassifySendOutcome(
        result.sent, result.subscribers_before, result.subscribers_after);
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
                                  retransmission ? attempted : 0U);
}

void SimulationLoop::PublishObservations(
    const std::vector<mujoco_sim::v2::ObservationFrame>& frames) {
  for (const auto& frame : frames) {
    const auto result = transport_->PublishObservation(frame);
    if (result.serialization_failed) {
      logger_->error(
          "event=message_serialize_failed message_type=ObservationFrame "
          "adapter_id={} sequence={}",
          LogValue(frame.adapter_id()), frame.sequence());
    }
  }
}

void SimulationLoop::PublishStatus() {
  auto status = core_.MakeStatus();
  sensor_runtime_->AppendStatus(&status);
  const auto result = transport_->PublishStatus(status);
  if (result.serialization_failed) {
    logger_->error("event=message_serialize_failed message_type=ServerStatus");
  }
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
      "queue_rejections={} queue_high_watermark={} step_rate_hz={:.3f} "
      "real_time_factor={:.3f} barrier_latency_avg_ms={:.3f} "
      "barrier_latency_max_ms={:.3f}",
      final, summary.window_seconds, ServerStateName(state.server_state),
      summary.tick_id, summary.simulation_time, summary.ticks_opened,
      summary.commits, summary.timeouts, summary.recoveries,
      summary.replacements, summary.local_errors,
      summary.malformed_messages, queue_rejections_.load(),
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
  if (operation == "AdapterCommit" || operation == "AdapterHeartbeat") {
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
  if (viewer_ != nullptr) deadline = std::min(deadline, next_viewer_sync_);
  deadline = std::min(deadline,
                      core_.state().tick_open ? next_retransmission_
                                              : next_tick_);
  return deadline;
}

void SimulationLoop::Finish(int exit_code, std::string reason) {
  queue_.Close("server stopped");
  sensor_runtime_->RequestStop();
  if (exit_code == 0) {
    ApplyEffects(core_.MarkStopped(reason));
  } else if (core_.state().server_state != mujoco_sim::v2::ERROR_PAUSED) {
    ApplyEffects(core_.MarkFailed(reason));
  }
  PublishStatus();
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
