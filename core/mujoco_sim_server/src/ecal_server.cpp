#include <mujoco_sim/server/ecal_server.hpp>

#include "server_logging.hpp"
#include "server_runtime.hpp"
#include "sim_viewer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v1/sync.pb.h>

namespace mujoco_sim::server {

namespace {

constexpr auto kWarningPeriod = std::chrono::seconds(1);
class CallbackGuard {
 public:
  explicit CallbackGuard(std::shared_ptr<CallbackState> state)
      : state_(std::move(state)), entered_(state_->Enter()) {}

  ~CallbackGuard() {
    if (entered_) {
      state_->Leave();
    }
  }

  explicit operator bool() const { return entered_; }

 private:
  std::shared_ptr<CallbackState> state_;
  bool entered_ = false;
};

std::string ErrorCodeName(mujoco_sim::v1::ErrorCode code) {
  return mujoco_sim::v1::ErrorCode_Name(code);
}

std::string ServerStateName(mujoco_sim::v1::ServerState state) {
  return mujoco_sim::v1::ServerState_Name(state);
}

struct SummarySnapshot {
  RuntimeMetricsSnapshot runtime;
  std::size_t provisional_controllers = 0;
  std::size_t active_controllers = 0;
  std::size_t timed_out_controllers = 0;
  std::size_t error_controllers = 0;
  mujoco_sim::v1::ServerState state =
      mujoco_sim::v1::SERVER_STATE_UNSPECIFIED;
};

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

template <typename Response>
int SerializeResponse(const Response& message, std::string* response) {
  if (!message.SerializeToString(response)) {
    return 1;
  }
  return 0;
}

template <typename Response>
int ParseError(mujoco_sim::v1::ErrorCode code, const std::string& message,
               std::string* response) {
  Response out;
  out.set_code(code);
  out.set_message(message);
  return SerializeResponse(out, response);
}

}  // namespace

EcalServer::EcalServer(ServerConfig config)
    : config_(std::move(config)),
      logger_(std::make_unique<ServerLogger>()),
      warning_limiter_(std::make_unique<WarningLimiter>(kWarningPeriod)),
      metrics_(std::make_unique<RuntimeMetrics>()),
      callback_state_(std::make_shared<CallbackState>()),
      core_(mujoco_sim::core::CoreOptions{
          config_.server_name, config_.tick_timeout_ms,
          config_.activation_timeout_ms,
          config_.allow_timed_out_adapter_replacement}) {}

EcalServer::~EcalServer() {
  Stop();
  ShutdownTransport();
  if (owns_ecal_) {
    if (const auto log = logger_->get()) {
      log->debug("event=ecal_finalizing");
    }
    eCAL::Finalize();
  }
  logger_->Flush();
}

bool EcalServer::Start(std::string* error_message) {
  if (!logger_->Initialize(config_, error_message)) {
    std::cerr << *error_message << '\n';
    return false;
  }
  const auto log = logger_->get();
  log->info(
      "event=server_starting server_name={} model_path={} initial_keyframe={} "
      "mode={} max_ticks={} tick_timeout_ms={} startup_timeout_ms={} "
      "activation_timeout_ms={} allow_timed_out_adapter_replacement={} "
      "status_period_ms={} log_level={} log_file={} "
      "log_summary_period_ms={}",
      LogValue(config_.server_name), LogValue(config_.model_path),
      LogValue(config_.initial_keyframe),
      config_.headless ? "headless" : "gui", config_.max_ticks,
      config_.tick_timeout_ms, config_.startup_timeout_ms,
      config_.activation_timeout_ms,
      config_.allow_timed_out_adapter_replacement,
      config_.status_period_ms, config_.log_level, LogValue(config_.log_file),
      config_.log_summary_period_ms);
  log->warn(
      "event=deprecated_config field=startup_timeout_ms behavior=ignored");

  if (!eCAL::IsInitialized()) {
    owns_ecal_ = eCAL::Initialize("mujoco-sim-server");
    if (!owns_ecal_) {
      *error_message = "failed to initialize eCAL";
      log->error("event=server_start_failed stage=ecal error={}",
                 LogValue(*error_message));
      return false;
    }
    log->info("event=ecal_initialized ownership=server");
  } else {
    log->info("event=ecal_initialized ownership=external");
  }

  if (!core_.LoadModel(
          config_.model_path,
          mujoco_sim::core::ModelLoadOptions{config_.initial_keyframe},
          error_message)) {
    log->error("event=server_start_failed stage=model_load model_path={} "
               "error={}",
               LogValue(config_.model_path), LogValue(*error_message));
    return false;
  }
  log->info(
      "event=model_loaded model_path={} observations={} commands={} "
      "simulation_time_s={:.6f}",
      LogValue(config_.model_path), core_.observations().size(),
      core_.commands().size(), core_.simulation_time());

  if (!config_.headless) {
    viewer_snapshot_ = core_.CreateSnapshot(error_message);
    if (viewer_snapshot_ == nullptr) {
      log->error("event=server_start_failed stage=viewer_snapshot error={}",
                 LogValue(*error_message));
      return false;
    }
    viewer_ = std::make_unique<SimViewer>();
    if (!viewer_->Initialize(error_message)) {
      log->error("event=server_start_failed stage=viewer_initialize "
                 "error={}",
                 LogValue(*error_message));
      viewer_.reset();
      viewer_snapshot_.reset();
      return false;
    }
    log->info("event=viewer_initialized mode=gui");
  } else {
    log->info("event=viewer_initialized mode=headless");
  }

  if (!RegisterServices(error_message)) {
    log->error("event=server_start_failed stage=registry error={}",
               LogValue(*error_message));
    return false;
  }
  RegisterSubscribersAndPublishers();
  log->info("event=server_started server_name={}",
            LogValue(config_.server_name));
  return true;
}

bool EcalServer::RegisterServices(std::string* error_message) {
  const std::string service_name =
      mujoco_sim::protocol::RegistryServiceName(config_.server_name);
  registry_ = std::make_unique<eCAL::CServiceServer>(
      service_name);

  if (!registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kGetAvailableResourcesMethod),
      [this, callback_state = callback_state_](
          const eCAL::SServiceMethodInformation&, const std::string& request,
          std::string& response) {
        CallbackGuard guard(callback_state);
        if (!guard) {
          return 1;
        }
        return HandleGetAvailableResources(request, &response);
      })) {
    *error_message = "failed to register method: " +
                     std::string(
                         mujoco_sim::protocol::kGetAvailableResourcesMethod);
    return false;
  }
  if (!registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kRegisterControllerMethod),
      [this, callback_state = callback_state_](
          const eCAL::SServiceMethodInformation&, const std::string& request,
          std::string& response) {
        CallbackGuard guard(callback_state);
        if (!guard) {
          return 1;
        }
        return HandleRegisterController(request, &response);
      })) {
    *error_message = "failed to register method: " +
                     std::string(
                         mujoco_sim::protocol::kRegisterControllerMethod);
    return false;
  }
  if (!registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kActivateControllerMethod),
      [this, callback_state = callback_state_](
          const eCAL::SServiceMethodInformation&, const std::string& request,
          std::string& response) {
        CallbackGuard guard(callback_state);
        if (!guard) {
          return 1;
        }
        return HandleActivateController(request, &response);
      })) {
    *error_message = "failed to register method: " +
                     std::string(
                         mujoco_sim::protocol::kActivateControllerMethod);
    return false;
  }
  if (!registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kUnregisterControllerMethod),
      [this, callback_state = callback_state_](
          const eCAL::SServiceMethodInformation&, const std::string& request,
          std::string& response) {
        CallbackGuard guard(callback_state);
        if (!guard) {
          return 1;
        }
        return HandleUnregisterController(request, &response);
      })) {
    *error_message = "failed to register method: " +
                     std::string(
                         mujoco_sim::protocol::kUnregisterControllerMethod);
    return false;
  }
  logger_->get()->info(
      "event=registry_registered service={} methods=4",
      LogValue(service_name));
  return true;
}

void EcalServer::RegisterSubscribersAndPublishers() {
  const std::string commit_topic =
      mujoco_sim::protocol::CommitTopicName(config_.server_name);
  const std::string heartbeat_topic =
      mujoco_sim::protocol::HeartbeatTopicName(config_.server_name);
  const std::string status_topic =
      mujoco_sim::protocol::StatusTopicName(config_.server_name);
  commit_subscriber_ = std::make_unique<eCAL::CSubscriber>(commit_topic);
  commit_subscriber_->SetReceiveCallback(
      [this, callback_state = callback_state_](
          const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
          const eCAL::SReceiveCallbackData& data) {
        CallbackGuard guard(callback_state);
        if (!guard) {
          return;
        }
        HandleCommitBytes(data.buffer, data.buffer_size);
      });

  heartbeat_subscriber_ =
      std::make_unique<eCAL::CSubscriber>(heartbeat_topic);
  heartbeat_subscriber_->SetReceiveCallback(
      [this, callback_state = callback_state_](
          const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
          const eCAL::SReceiveCallbackData& data) {
        CallbackGuard guard(callback_state);
        if (!guard) {
          return;
        }
        HandleHeartbeatBytes(data.buffer, data.buffer_size);
      });

  status_publisher_ = std::make_unique<eCAL::CPublisher>(status_topic);
  logger_->get()->info(
      "event=runtime_endpoints_registered commit_topic={} heartbeat_topic={} "
      "status_topic={}",
      LogValue(commit_topic), LogValue(heartbeat_topic),
      LogValue(status_topic));
}

void EcalServer::ShutdownTransport() {
  callback_state_->StopAccepting();
  if (commit_subscriber_) {
    commit_subscriber_->RemoveReceiveCallback();
  }
  if (heartbeat_subscriber_) {
    heartbeat_subscriber_->RemoveReceiveCallback();
  }
  if (registry_) {
    registry_->RemoveMethodCallback(
        mujoco_sim::protocol::kGetAvailableResourcesMethod);
    registry_->RemoveMethodCallback(
        mujoco_sim::protocol::kRegisterControllerMethod);
    registry_->RemoveMethodCallback(
        mujoco_sim::protocol::kActivateControllerMethod);
    registry_->RemoveMethodCallback(
        mujoco_sim::protocol::kUnregisterControllerMethod);
  }
  callback_state_->WaitForIdle();
  commit_subscriber_.reset();
  heartbeat_subscriber_.reset();
  registry_.reset();
  {
    std::lock_guard publisher_lock(publisher_mutex_);
    tick_publishers_.clear();
  }
  status_publisher_.reset();
  if (logger_->get()) {
    logger_->get()->debug("event=transport_stopped");
  }
}

int EcalServer::Run() {
  if (viewer_ != nullptr) {
    return RunGraphical();
  }
  return RunLoop();
}

int EcalServer::RunGraphical() {
  int result = 1;
  std::thread server_thread([this, &result]() {
    std::string error;
    if (!viewer_->QueueLoad(viewer_snapshot_.get(), config_.model_path,
                            &error)) {
      const bool viewer_closed = viewer_->close_requested();
      if (!error.empty()) {
        logger_->get()->error(
            "event=viewer_load_failed model_path={} error={}",
            LogValue(config_.model_path), LogValue(error));
      }
      result = FinishRun(viewer_closed ? 0 : 1,
                         viewer_closed ? "viewer_closed"
                                       : "viewer_load_failed");
      viewer_->RequestClose();
      return;
    }
    logger_->get()->info("event=viewer_model_loaded model_path={}",
                         LogValue(config_.model_path));
    result = RunLoop();
    viewer_->RequestClose();
  });

  viewer_->RenderLoop();
  StopWithReason("viewer_closed");
  server_thread.join();
  return result;
}

int EcalServer::RunLoop() {
  const auto started_at = std::chrono::steady_clock::now();
  const auto step_period = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(core_.timestep(), 1e-6)));
  auto next_tick = started_at;
  auto next_status = started_at;
  auto next_viewer_sync = started_at;
  std::uint64_t observed_tick_id = core_.current_tick_id();

  {
    std::lock_guard lock(mutex_);
    metrics_->Initialize(started_at, core_.current_tick_id(),
                         core_.simulation_time());
  }
  logger_->get()->info("event=run_loop_started");

  while (!stop_requested_ && eCAL::Ok()) {
    std::vector<mujoco_sim::v1::Tick> ticks;
    bool should_stop_success = false;
    bool should_stop_error = false;
    bool tick_retransmission = false;
    bool tick_opened = false;
    bool deadline_completed_barrier = false;
    bool scheduled_completed_barrier = false;
    bool publish_status = false;
    bool state_changed = false;
    mujoco_sim::v1::ServerState old_state =
        mujoco_sim::v1::SERVER_STATE_UNSPECIFIED;
    mujoco_sim::v1::ServerState new_state =
        mujoco_sim::v1::SERVER_STATE_UNSPECIFIED;
    std::string stop_reason;
    std::string state_message;
    std::string viewer_error;
    double deadline_barrier_latency_ms = 0.0;
    double scheduled_barrier_latency_ms = 0.0;
    std::vector<mujoco_sim::core::ControllerTransition> transitions;
    const auto now = std::chrono::steady_clock::now();

    {
      std::unique_lock lock(mutex_);
      auto deadlines = core_.CheckControllerDeadlines(now);
      transitions = std::move(deadlines.transitions);
      if (deadlines.stepped) {
        deadline_completed_barrier = true;
        deadline_barrier_latency_ms = metrics_->RecordBarrierCompleted(
            std::chrono::steady_clock::now());
      }
      for (const auto& transition : transitions) {
        if (transition.new_state == mujoco_sim::v1::TIMED_OUT) {
          metrics_->RecordTimeout();
        } else if (transition.old_state == mujoco_sim::v1::PROVISIONAL &&
                   transition.new_state == mujoco_sim::v1::UNREGISTERED) {
          metrics_->RecordActivationExpiration();
        }
      }
      if (!transitions.empty() || deadlines.stepped) {
        status_dirty_ = true;
      }
      if (core_.current_tick_id() != observed_tick_id) {
        observed_tick_id = core_.current_tick_id();
        next_tick = std::chrono::steady_clock::now() + step_period;
      }

      if (core_.server_state() == mujoco_sim::v1::ERROR_PAUSED) {
        should_stop_error = true;
        stop_reason = "core_error_paused";
      } else if (config_.max_ticks > 0 &&
                 core_.current_tick_id() >= config_.max_ticks) {
        should_stop_success = true;
        stop_reason = "max_ticks";
      } else if (!core_.tick_open() && now >= next_tick) {
        ticks = OpenTickLocked();
        tick_opened = true;
      } else if (core_.tick_open()) {
        tick_retransmission = true;
        ticks = OpenTickLocked();
      }

      if (viewer_ != nullptr && now >= next_viewer_sync) {
        if (viewer_->Synchronize(&core_, viewer_snapshot_.get(),
                                 &viewer_error)) {
          next_viewer_sync = now +
                             std::chrono::milliseconds(16);
        } else if (!viewer_->close_requested()) {
          core_.EnterFatalError("viewer synchronization failed: " +
                                viewer_error);
          should_stop_error = true;
          stop_reason = "viewer_sync_failed";
        }
      }

      new_state = core_.server_state();
      if (new_state != last_logged_state_) {
        old_state = last_logged_state_;
        last_logged_state_ = new_state;
        state_changed = true;
        state_message = core_.MakeStatus().message();
      }
      if (status_dirty_ || now >= next_status) {
        publish_status = true;
        status_dirty_ = false;
        next_status = now +
                      std::chrono::milliseconds(config_.status_period_ms);
      }
    }

    if (!ticks.empty()) {
      PublishTicks(ticks, tick_retransmission);
    }
    if (tick_opened) {
      std::lock_guard lock(mutex_);
      if (core_.StepIfBarrierComplete()) {
        const auto completed_at = std::chrono::steady_clock::now();
        observed_tick_id = core_.current_tick_id();
        next_tick = completed_at + step_period;
        scheduled_completed_barrier = true;
        scheduled_barrier_latency_ms =
            metrics_->RecordBarrierCompleted(completed_at);
      }
    }
    for (const auto& transition : transitions) {
      if (transition.new_state == mujoco_sim::v1::TIMED_OUT) {
        logger_->get()->warn(
            "event=controller_timed_out controller_id={} session_id={} "
            "tick_id={} message={}",
            LogValue(transition.controller_id), transition.session_id,
            transition.tick_id, LogValue(transition.message));
      } else if (transition.old_state == mujoco_sim::v1::PROVISIONAL &&
                 transition.new_state == mujoco_sim::v1::UNREGISTERED) {
        logger_->get()->warn(
            "event=controller_activation_expired controller_id={} "
            "session_id={} tick_id={}",
            LogValue(transition.controller_id), transition.session_id,
            transition.tick_id);
        std::lock_guard publisher_lock(publisher_mutex_);
        tick_publishers_.erase(transition.controller_id);
      }
    }
    if (state_changed) {
      logger_->get()->info(
          "event=server_state_changed old_state={} new_state={} message={}",
          ServerStateName(old_state), ServerStateName(new_state),
          LogValue(state_message));
    }
    if (deadline_completed_barrier) {
      logger_->get()->trace(
          "event=barrier_completed source=deadline completed_tick_id={} "
          "barrier_latency_ms={:.3f}",
          observed_tick_id == 0 ? 0 : observed_tick_id - 1,
          deadline_barrier_latency_ms);
    }
    if (scheduled_completed_barrier) {
      logger_->get()->trace(
          "event=barrier_completed source=scheduler completed_tick_id={} "
          "barrier_latency_ms={:.3f}",
          observed_tick_id == 0 ? 0 : observed_tick_id - 1,
          scheduled_barrier_latency_ms);
    }
    if (!viewer_error.empty()) {
      logger_->get()->error("event=viewer_sync_failed error={}",
                            LogValue(viewer_error));
    }

    if (publish_status) {
      PublishStatus();
    }
    MaybeLogSummary(std::chrono::steady_clock::now(), false);

    if (should_stop_success) {
      {
        std::lock_guard lock(mutex_);
        core_.MarkStopped(stop_reason);
      }
      PublishStatus();
      return FinishRun(0, stop_reason);
    }
    if (should_stop_error) {
      PublishStatus();
      return FinishRun(2, stop_reason);
    }

    std::unique_lock lock(mutex_);
    const auto wait_now = std::chrono::steady_clock::now();
    const auto wake_deadline = ComputeLoopWakeDeadline(
        wait_now, next_tick, core_.tick_open(), std::chrono::milliseconds(5));
    condition_.wait_until(lock, wake_deadline);
  }
  if (stop_requested_) {
    std::string reason;
    {
      std::lock_guard lock(mutex_);
      reason = stop_reason_;
    }
    return FinishRun(0, reason);
  }
  return FinishRun(1, "ecal_stopped");
}

void EcalServer::MaybeLogSummary(
    std::chrono::steady_clock::time_point now, bool final) {
  if (config_.log_summary_period_ms == 0) {
    return;
  }

  SummarySnapshot summary;
  {
    std::lock_guard lock(mutex_);
    if (!metrics_->initialized()) {
      return;
    }
    if (!final && !metrics_->SummaryDue(
                      now, std::chrono::milliseconds(
                               config_.log_summary_period_ms))) {
      return;
    }
    summary.provisional_controllers =
        core_.controller_count(mujoco_sim::v1::PROVISIONAL);
    summary.active_controllers =
        core_.controller_count(mujoco_sim::v1::ACTIVE);
    summary.timed_out_controllers =
        core_.controller_count(mujoco_sim::v1::TIMED_OUT);
    summary.error_controllers =
        core_.controller_count(mujoco_sim::v1::ERROR);
    summary.state = core_.server_state();
    summary.runtime = metrics_->TakeSummary(
        now, core_.current_tick_id(), core_.simulation_time());
  }

  const auto& runtime = summary.runtime;
  logger_->get()->info(
      "event=tick_summary final={} window_s={:.3f} state={} tick_id={} "
      "simulation_time_s={:.6f} provisional_controllers={} "
      "active_controllers={} timed_out_controllers={} error_controllers={} "
      "ticks_opened={} "
      "tick_publish_attempts={} tick_serialization_failures={} "
      "tick_messages_sent={} "
      "tick_send_rejected_with_subscribers={} tick_no_subscribers={} "
      "tick_retransmissions={} commits={} "
      "duplicate_commits={} rejected_commits={} timeouts={} recoveries={} "
      "replacements={} local_errors={} activation_expirations={} "
      "malformed_messages={} suppressed_warnings={} step_rate_hz={:.3f} "
      "real_time_factor={:.3f} "
      "barrier_latency_avg_ms={:.3f} barrier_latency_max_ms={:.3f}",
      final, runtime.window_seconds, ServerStateName(summary.state),
      runtime.tick_id, runtime.simulation_time,
      summary.provisional_controllers, summary.active_controllers,
      summary.timed_out_controllers, summary.error_controllers,
      runtime.ticks_opened, runtime.tick_publish_attempts,
      runtime.tick_serialization_failures, runtime.tick_messages_sent,
      runtime.tick_send_rejected_with_subscribers,
      runtime.tick_no_subscribers, runtime.tick_retransmissions,
      runtime.commits, runtime.duplicate_commits, runtime.rejected_commits,
      runtime.timeouts, runtime.recoveries, runtime.replacements,
      runtime.local_errors, runtime.activation_expirations,
      runtime.malformed_messages,
      runtime.suppressed_warnings, runtime.rates.step_rate_hz,
      runtime.rates.real_time_factor,
      runtime.rates.barrier_latency_avg_ms,
      runtime.rates.barrier_latency_max_ms);
}

int EcalServer::FinishRun(int exit_code, const std::string& reason) {
  callback_state_->StopAccepting();
  callback_state_->WaitForIdle();
  {
    std::lock_guard lock(mutex_);
    if (exit_code == 0) {
      core_.MarkStopped(reason);
    } else if (core_.server_state() != mujoco_sim::v1::ERROR_PAUSED) {
      core_.EnterFatalError(reason);
    }
  }
  PublishStatus();
  MaybeLogSummary(std::chrono::steady_clock::now(), true);
  std::uint64_t tick_id = 0;
  {
    std::lock_guard lock(mutex_);
    tick_id = core_.current_tick_id();
    stop_reason_ = reason;
    stop_requested_.store(true);
  }
  if (exit_code == 0) {
    logger_->get()->info(
        "event=server_stopped reason={} exit_code={} tick_id={}", reason,
        exit_code, tick_id);
  } else {
    logger_->get()->error(
        "event=server_stopped reason={} exit_code={} tick_id={}", reason,
        exit_code, tick_id);
  }
  return exit_code;
}

void EcalServer::StopWithReason(const std::string& reason) {
  bool changed = false;
  bool should_log = false;
  {
    std::lock_guard lock(mutex_);
    if (!stop_requested_.exchange(true)) {
      stop_reason_ = reason;
      changed = true;
      should_log = metrics_->initialized();
    }
  }
  if (changed && should_log && logger_->get()) {
    logger_->get()->debug("event=stop_requested reason={}", reason);
  }
  condition_.notify_all();
}

void EcalServer::Stop() { StopWithReason("external_stop"); }

bool EcalServer::ShouldLogWarning(const char* key,
                                  std::uint64_t* suppressed) {
  const WarningDecision decision = warning_limiter_->Admit(key);
  *suppressed = decision.suppressed;
  if (!decision.emit) {
    std::lock_guard lock(mutex_);
    metrics_->RecordSuppressedWarning();
  }
  return decision.emit;
}

void EcalServer::RecordMalformedMessage() {
  std::lock_guard lock(mutex_);
  metrics_->RecordMalformedMessage();
}

std::vector<mujoco_sim::v1::Tick> EcalServer::OpenTickLocked() {
  const bool was_open = core_.tick_open();
  std::unordered_map<std::string, std::uint64_t> timed_out_sessions;
  if (!was_open) {
    const auto before = core_.MakeStatus();
    for (const auto& controller : before.controllers()) {
      if (controller.state() == mujoco_sim::v1::TIMED_OUT) {
        timed_out_sessions.emplace(controller.controller_id(),
                                   controller.session_id());
      }
    }
  }
  auto ticks = core_.OpenTick();
  if (!was_open && core_.tick_open()) {
    metrics_->RecordTickOpened(std::chrono::steady_clock::now());
    const auto after = core_.MakeStatus();
    for (const auto& controller : after.controllers()) {
      const auto previous = timed_out_sessions.find(controller.controller_id());
      if (previous != timed_out_sessions.end() &&
          previous->second == controller.session_id() &&
          controller.state() == mujoco_sim::v1::ACTIVE) {
        status_dirty_ = true;
        logger_->get()->info(
            "event=controller_recovered controller_id={} session_id={} "
            "tick_id={}",
            LogValue(controller.controller_id()), controller.session_id(),
            core_.current_tick_id());
      }
    }
  }
  return ticks;
}

void EcalServer::PublishTicks(
    const std::vector<mujoco_sim::v1::Tick>& ticks, bool retransmission) {
  struct PublishAttempt {
    const mujoco_sim::v1::Tick* tick = nullptr;
    std::string bytes;
    std::string topic;
    std::size_t subscribers_before = 0;
    std::size_t subscribers_after = 0;
    bool attempted = false;
    bool sent = false;
    bool publisher_created = false;
  };

  std::uint64_t attempted_count = 0;
  std::uint64_t sent_count = 0;
  std::uint64_t serialization_failure_count = 0;
  std::uint64_t failure_count = 0;
  std::uint64_t no_subscriber_count = 0;
  std::vector<PublishAttempt> attempts;
  attempts.reserve(ticks.size());
  for (const auto& tick : ticks) {
    PublishAttempt attempt;
    attempt.tick = &tick;
    if (!tick.SerializeToString(&attempt.bytes)) {
      logger_->get()->error(
          "event=message_serialize_failed message_type=Tick "
          "controller_id={} tick_id={}",
          LogValue(tick.controller_id()), tick.header().tick_id());
      ++serialization_failure_count;
      continue;
    }
    attempts.push_back(std::move(attempt));
  }

  {
    std::unique_lock core_lock(mutex_);
    std::lock_guard publisher_lock(publisher_mutex_);
    for (auto& attempt : attempts) {
      const auto& tick = *attempt.tick;
      if (!core_.IsTickIssued(tick)) {
        continue;
      }
      auto& publisher = tick_publishers_[tick.controller_id()];
      if (!publisher) {
        attempt.topic = mujoco_sim::protocol::TickTopicName(
            config_.server_name, tick.controller_id());
        publisher = std::make_unique<eCAL::CPublisher>(attempt.topic);
        attempt.publisher_created = true;
      }
      attempt.subscribers_before = publisher->GetSubscriberCount();
      attempt.sent = publisher->Send(attempt.bytes);
      attempt.subscribers_after = publisher->GetSubscriberCount();
      attempt.attempted = true;
    }
  }

  for (const auto& attempt : attempts) {
    if (!attempt.attempted) {
      continue;
    }
    const auto& tick = *attempt.tick;
    if (attempt.publisher_created) {
      logger_->get()->debug(
          "event=tick_publisher_created controller_id={} topic={}",
          LogValue(tick.controller_id()), LogValue(attempt.topic));
    }
    ++attempted_count;
    const SendOutcome outcome = ClassifySendOutcome(
        attempt.sent, attempt.subscribers_before, attempt.subscribers_after);
    if (outcome != SendOutcome::kSent) {
      if (outcome == SendOutcome::kNoSubscribers) {
        ++no_subscriber_count;
        logger_->get()->trace(
            "event=message_not_sent message_type=Tick reason=no_subscribers "
            "controller_id={} tick_id={} bytes={} subscribers_before={} "
            "subscribers_after={}",
            LogValue(tick.controller_id()), tick.header().tick_id(),
            attempt.bytes.size(), attempt.subscribers_before,
            attempt.subscribers_after);
        continue;
      }
      std::uint64_t suppressed = 0;
      if (ShouldLogWarning("tick_send_rejected", &suppressed)) {
        logger_->get()->warn(
            "event=message_send_rejected message_type=Tick controller_id={} "
            "tick_id={} bytes={} subscribers_before={} subscribers_after={} "
            "suppressed={}",
            LogValue(tick.controller_id()), tick.header().tick_id(),
            attempt.bytes.size(), attempt.subscribers_before,
            attempt.subscribers_after, suppressed);
      }
      ++failure_count;
      continue;
    }
    ++sent_count;
    logger_->get()->trace(
        "event=tick_published controller_id={} tick_id={} bytes={}",
        LogValue(tick.controller_id()), tick.header().tick_id(),
        attempt.bytes.size());
  }
  if (attempted_count > 0 || serialization_failure_count > 0) {
    std::lock_guard lock(mutex_);
    metrics_->RecordTickPublishBatch(
        attempted_count, serialization_failure_count, sent_count, failure_count,
        no_subscriber_count,
        retransmission ? attempted_count : 0U);
  }
}

void EcalServer::PublishStatus() {
  if (!status_publisher_) {
    return;
  }
  mujoco_sim::v1::ServerStatus status;
  {
    std::lock_guard lock(mutex_);
    status = core_.MakeStatus();
  }
  std::string bytes;
  if (!status.SerializeToString(&bytes)) {
    logger_->get()->error(
        "event=message_serialize_failed message_type=ServerStatus");
    return;
  }
  const std::size_t subscribers_before =
      status_publisher_->GetSubscriberCount();
  const bool sent = status_publisher_->Send(bytes);
  const std::size_t subscribers_after =
      status_publisher_->GetSubscriberCount();
  const SendOutcome outcome =
      ClassifySendOutcome(sent, subscribers_before, subscribers_after);
  if (outcome != SendOutcome::kSent) {
    if (outcome == SendOutcome::kNoSubscribers) {
      logger_->get()->trace(
          "event=message_not_sent message_type=ServerStatus "
          "reason=no_subscribers tick_id={} bytes={} subscribers_before={} "
          "subscribers_after={}",
          status.header().tick_id(), bytes.size(), subscribers_before,
          subscribers_after);
      return;
    }
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("status_send_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=message_send_rejected message_type=ServerStatus tick_id={} "
          "bytes={} subscribers_before={} subscribers_after={} suppressed={}",
          status.header().tick_id(), bytes.size(), subscribers_before,
          subscribers_after, suppressed);
    }
    return;
  }
  logger_->get()->debug(
      "event=status_published state={} tick_id={} controllers={} bytes={}",
      ServerStateName(status.state()), status.header().tick_id(),
      status.controllers_size(), bytes.size());
}

int EcalServer::HandleGetAvailableResources(const std::string& request,
                                            std::string* response) {
  mujoco_sim::v1::GetAvailableResourcesRequest parsed;
  if (!IsProtocolPayloadSizeAllowed(request.size()) ||
      !parsed.ParseFromString(request)) {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("resources_request_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_malformed method=GetAvailableResources "
          "bytes={} max_bytes={} suppressed={}",
          request.size(), kMaxProtocolPayloadBytes, suppressed);
    }
    const int result = ParseError<mujoco_sim::v1::GetAvailableResourcesResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
    if (result != 0) {
      logger_->get()->error(
          "event=message_serialize_failed "
          "message_type=GetAvailableResourcesResponse");
    }
    return result;
  }
  mujoco_sim::v1::GetAvailableResourcesResponse result;
  {
    std::lock_guard lock(mutex_);
    result = core_.GetAvailableResources(parsed);
  }
  if (result.code() == mujoco_sim::v1::OK) {
    logger_->get()->debug(
        "event=resources_requested code={} observations={} commands={}",
        ErrorCodeName(result.code()), result.observations_size(),
        result.commands_size());
  } else {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("resources_request_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_rejected method=GetAvailableResources "
          "code={} message={} suppressed={}",
          ErrorCodeName(result.code()), LogValue(result.message()), suppressed);
    }
  }
  const int serialized = SerializeResponse(result, response);
  if (serialized != 0) {
    logger_->get()->error(
        "event=message_serialize_failed "
        "message_type=GetAvailableResourcesResponse");
  }
  return serialized;
}

int EcalServer::HandleRegisterController(const std::string& request,
                                         std::string* response) {
  mujoco_sim::v1::RegisterControllerRequest parsed;
  if (!IsProtocolPayloadSizeAllowed(request.size()) ||
      !parsed.ParseFromString(request)) {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("register_request_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_malformed method=RegisterController "
          "bytes={} max_bytes={} suppressed={}",
          request.size(), kMaxProtocolPayloadBytes, suppressed);
    }
    const int result = ParseError<mujoco_sim::v1::RegisterControllerResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
    if (result != 0) {
      logger_->get()->error(
          "event=message_serialize_failed "
          "message_type=RegisterControllerResponse");
    }
    return result;
  }
  mujoco_sim::v1::RegisterControllerResponse result;
  bool replaced = false;
  {
    std::lock_guard lock(mutex_);
    result = core_.RegisterController(parsed);
    if (result.code() == mujoco_sim::v1::OK) {
      status_dirty_ = true;
      replaced = result.message() == "replaced timed-out controller session";
      if (replaced) {
        metrics_->RecordReplacement();
      }
    }
  }
  if (result.code() == mujoco_sim::v1::OK) {
    logger_->get()->info(
        "event=controller_registered controller_id={} session_id={} epoch={} "
        "observations={} commands={} timeout_ms={}",
        LogValue(result.controller_id()), result.session_id(), result.epoch(),
        result.layout().observations_size(), result.layout().commands_size(),
        parsed.timeout_ms() == 0 ? config_.tick_timeout_ms
                                 : parsed.timeout_ms());
    if (replaced) {
      logger_->get()->info(
          "event=controller_replaced controller_id={} new_session_id={} "
          "epoch={}",
          LogValue(result.controller_id()), result.session_id(), result.epoch());
    }
  } else {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("register_request_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=controller_request_rejected operation=register "
          "controller_id={} code={} message={} suppressed={}",
          LogValue(parsed.controller_id()), ErrorCodeName(result.code()),
          LogValue(result.message()), suppressed);
    }
  }
  const int serialized = SerializeResponse(result, response);
  if (serialized != 0) {
    logger_->get()->error(
        "event=message_serialize_failed "
        "message_type=RegisterControllerResponse");
  }
  condition_.notify_all();
  return serialized;
}

int EcalServer::HandleActivateController(const std::string& request,
                                         std::string* response) {
  mujoco_sim::v1::ActivateControllerRequest parsed;
  if (!IsProtocolPayloadSizeAllowed(request.size()) ||
      !parsed.ParseFromString(request)) {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("activate_request_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_malformed method=ActivateController "
          "bytes={} max_bytes={} suppressed={}",
          request.size(), kMaxProtocolPayloadBytes, suppressed);
    }
    const int result = ParseError<mujoco_sim::v1::ActivateControllerResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
    if (result != 0) {
      logger_->get()->error(
          "event=message_serialize_failed "
          "message_type=ActivateControllerResponse");
    }
    return result;
  }
  mujoco_sim::v1::ActivateControllerResponse result;
  {
    std::lock_guard lock(mutex_);
    result = core_.ActivateController(parsed);
    if (result.code() == mujoco_sim::v1::OK) {
      status_dirty_ = true;
    }
  }
  if (result.code() == mujoco_sim::v1::OK) {
    logger_->get()->info(
        "event=controller_activated controller_id={} session_id={} epoch={}",
        LogValue(parsed.controller_id()), parsed.session_id(), parsed.epoch());
  } else {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("activate_request_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=controller_request_rejected operation=activate "
          "controller_id={} code={} message={} suppressed={}",
          LogValue(parsed.controller_id()), ErrorCodeName(result.code()),
          LogValue(result.message()), suppressed);
    }
  }
  const int serialized = SerializeResponse(result, response);
  if (serialized != 0) {
    logger_->get()->error(
        "event=message_serialize_failed "
        "message_type=ActivateControllerResponse");
  }
  condition_.notify_all();
  return serialized;
}

int EcalServer::HandleUnregisterController(const std::string& request,
                                           std::string* response) {
  mujoco_sim::v1::UnregisterControllerRequest parsed;
  if (!IsProtocolPayloadSizeAllowed(request.size()) ||
      !parsed.ParseFromString(request)) {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("unregister_request_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_malformed method=UnregisterController "
          "bytes={} max_bytes={} suppressed={}",
          request.size(), kMaxProtocolPayloadBytes, suppressed);
    }
    const int result = ParseError<mujoco_sim::v1::UnregisterControllerResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
    if (result != 0) {
      logger_->get()->error(
          "event=message_serialize_failed "
          "message_type=UnregisterControllerResponse");
    }
    return result;
  }
  mujoco_sim::v1::UnregisterControllerResponse result;
  bool stepped = false;
  std::uint64_t completed_tick_id = 0;
  double barrier_latency_ms = 0.0;
  {
    std::lock_guard lock(mutex_);
    const auto tick_id_before = core_.current_tick_id();
    result = core_.UnregisterController(parsed);
    if (result.code() == mujoco_sim::v1::OK) {
      status_dirty_ = true;
      stepped = core_.current_tick_id() != tick_id_before;
      if (stepped) {
        completed_tick_id = core_.current_tick_id() - 1;
        barrier_latency_ms = metrics_->RecordBarrierCompleted(
            std::chrono::steady_clock::now());
      }
    }
  }
  if (result.code() == mujoco_sim::v1::OK) {
    logger_->get()->info(
        "event=controller_unregistered controller_id={} session_id={} epoch={}",
        LogValue(parsed.controller_id()), parsed.session_id(), parsed.epoch());
    if (stepped) {
      logger_->get()->trace(
          "event=barrier_completed source=unregister completed_tick_id={} "
          "barrier_latency_ms={:.3f}",
          completed_tick_id, barrier_latency_ms);
    }
    std::lock_guard publisher_lock(publisher_mutex_);
    tick_publishers_.erase(parsed.controller_id());
  } else {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("unregister_request_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=controller_request_rejected operation=unregister "
          "controller_id={} code={} message={} suppressed={}",
          LogValue(parsed.controller_id()), ErrorCodeName(result.code()),
          LogValue(result.message()), suppressed);
    }
  }
  const int serialized = SerializeResponse(result, response);
  if (serialized != 0) {
    logger_->get()->error(
        "event=message_serialize_failed "
        "message_type=UnregisterControllerResponse");
  }
  condition_.notify_all();
  return serialized;
}

void EcalServer::HandleCommitBytes(const void* data, std::size_t size) {
  const auto record_malformed = [this, size]() {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("commit_payload_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=topic_payload_malformed message_type=AdapterCommit bytes={} "
          "max_bytes={} suppressed={}",
          size, kMaxProtocolPayloadBytes, suppressed);
    }
  };
  if (data == nullptr || size == 0 ||
      !IsProtocolPayloadSizeAllowed(size)) {
    record_malformed();
    return;
  }
  mujoco_sim::v1::AdapterCommit commit;
  if (!commit.ParseFromArray(data, static_cast<int>(size))) {
    record_malformed();
    return;
  }

  mujoco_sim::core::CommitResult result;
  double barrier_latency_ms = 0.0;
  {
    std::lock_guard lock(mutex_);
    result = core_.SubmitCommit(commit);
    if (result.controller_state_changed || result.recovery_pending) {
      status_dirty_ = true;
    }
    barrier_latency_ms = metrics_->RecordCommit(
        result.duplicate, result.code != mujoco_sim::v1::OK, result.stepped,
        std::chrono::steady_clock::now());
    if (result.controller_state_changed) {
      metrics_->RecordLocalError();
    }
    if (result.recovery_pending) {
      metrics_->RecordRecovery();
    }
  }
  if (result.code != mujoco_sim::v1::OK) {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("commit_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=commit_rejected controller_id={} tick_id={} session_id={} "
          "epoch={} code={} message={} suppressed={}",
          LogValue(commit.controller_id()), commit.header().tick_id(),
          commit.header().session_id(), commit.header().epoch(),
          ErrorCodeName(result.code), LogValue(result.message), suppressed);
    }
  } else if (result.duplicate) {
    logger_->get()->debug(
        "event=commit_duplicate controller_id={} tick_id={} message={}",
        LogValue(commit.controller_id()), commit.header().tick_id(),
        LogValue(result.message));
  } else {
    logger_->get()->trace(
        "event=commit_accepted controller_id={} tick_id={} stepped={} "
        "commands={}",
        LogValue(commit.controller_id()), commit.header().tick_id(),
        result.stepped, commit.commands_size());
  }
  if (result.controller_state_changed) {
    logger_->get()->error(
        "event=controller_local_error controller_id={} session_id={} "
        "tick_id={} state={} message={}",
        LogValue(commit.controller_id()), commit.header().session_id(),
        commit.header().tick_id(),
        mujoco_sim::v1::ControllerState_Name(result.controller_state),
        LogValue(result.message));
  } else if (result.recovery_pending) {
    logger_->get()->info(
        "event=controller_recovery_pending controller_id={} session_id={} "
        "tick_id={}",
        LogValue(commit.controller_id()), commit.header().session_id(),
        commit.header().tick_id());
  }
  if (result.stepped) {
    logger_->get()->trace(
        "event=barrier_completed completed_tick_id={} "
        "barrier_latency_ms={:.3f}",
        commit.header().tick_id(), barrier_latency_ms);
  }
  condition_.notify_all();
}

void EcalServer::HandleHeartbeatBytes(const void* data, std::size_t size) {
  const auto record_malformed = [this, size]() {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("heartbeat_payload_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=topic_payload_malformed message_type=Heartbeat bytes={} "
          "max_bytes={} suppressed={}",
          size, kMaxProtocolPayloadBytes, suppressed);
    }
  };
  if (data == nullptr || size == 0 ||
      !IsProtocolPayloadSizeAllowed(size)) {
    record_malformed();
    return;
  }
  mujoco_sim::v1::Heartbeat heartbeat;
  if (!heartbeat.ParseFromArray(data, static_cast<int>(size))) {
    record_malformed();
    return;
  }
  {
    std::lock_guard lock(mutex_);
    core_.AcceptHeartbeat(heartbeat);
  }
  logger_->get()->trace(
      "event=heartbeat_received controller_id={} tick_id={} session_id={} "
      "epoch={}",
      LogValue(heartbeat.controller_id()), heartbeat.header().tick_id(),
      heartbeat.header().session_id(), heartbeat.header().epoch());
  condition_.notify_all();
}

}  // namespace mujoco_sim::server
