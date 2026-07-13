#include <mujoco_sim/server/server_application.hpp>

#include "server_logging.hpp"
#include "server_runtime.hpp"
#include "endpoints.hpp"
#include "sensor_runtime.hpp"
#include "simulation_scheduler.hpp"
#include "sim_viewer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v2/sync.pb.h>

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

std::string ErrorCodeName(mujoco_sim::v2::ErrorCode code) {
  return mujoco_sim::v2::ErrorCode_Name(code);
}

std::string ServerStateName(mujoco_sim::v2::ServerState state) {
  return mujoco_sim::v2::ServerState_Name(state);
}

struct SummarySnapshot {
  RuntimeMetricsSnapshot runtime;
  std::size_t provisional_controllers = 0;
  std::size_t active_controllers = 0;
  std::size_t timed_out_controllers = 0;
  std::size_t error_controllers = 0;
  mujoco_sim::v2::ServerState state =
      mujoco_sim::v2::SERVER_STATE_UNSPECIFIED;
};

void LogBoundaryFailure(const std::shared_ptr<spdlog::logger>& logger,
                        std::string_view boundary, std::string_view operation,
                        std::string_view error) noexcept {
  try {
    if (logger) {
      logger->error("event=ecal_exception boundary={} operation={} error={}",
                    boundary, LogValue(operation), LogValue(error));
    }
  } catch (...) {
  }
}

template <typename Action>
int InvokeServiceCallback(const std::shared_ptr<spdlog::logger>& logger,
                          std::string_view method, Action&& action) noexcept {
  int result = 1;
  std::string error;
  if (!RunWithExceptionBoundary(
          [&] { result = std::forward<Action>(action)(); }, &error)) {
    LogBoundaryFailure(logger, "service_callback", method, error);
  }
  return result;
}

template <typename Action>
void InvokeSubscriberCallback(const std::shared_ptr<spdlog::logger>& logger,
                              std::string_view topic,
                              Action&& action) noexcept {
  std::string error;
  if (!RunWithExceptionBoundary(std::forward<Action>(action), &error)) {
    LogBoundaryFailure(logger, "subscriber_callback", topic, error);
  }
}

template <typename Action>
int InvokeGuardedServiceCallback(
    const std::shared_ptr<CallbackState>& callback_state,
    const std::shared_ptr<spdlog::logger>& logger, std::string_view method,
    Action&& action) noexcept {
  int result = 1;
  std::string error;
  if (!RunWithExceptionBoundary(
          [&] {
            CallbackGuard guard(callback_state);
            if (guard) {
              result = InvokeServiceCallback(
                  logger, method, std::forward<Action>(action));
            }
          },
          &error)) {
    LogBoundaryFailure(logger, "service_callback_guard", method, error);
  }
  return result;
}

template <typename Action>
void InvokeGuardedSubscriberCallback(
    const std::shared_ptr<CallbackState>& callback_state,
    const std::shared_ptr<spdlog::logger>& logger, std::string_view topic,
    Action&& action) noexcept {
  std::string error;
  if (!RunWithExceptionBoundary(
          [&] {
            CallbackGuard guard(callback_state);
            if (guard) {
              InvokeSubscriberCallback(logger, topic,
                                       std::forward<Action>(action));
            }
          },
          &error)) {
    LogBoundaryFailure(logger, "subscriber_callback_guard", topic, error);
  }
}

template <typename Response>
int ParseError(mujoco_sim::v2::ErrorCode code, const std::string& message,
               const RegistryEndpoint& endpoint, std::string* response) {
  Response out;
  out.set_code(code);
  out.set_message(message);
  return endpoint.Serialize(out, response);
}

}  // namespace

ServerApplication::ServerApplication(ServerConfig config)
    : config_(std::move(config)),
      logger_(std::make_unique<ServerLogger>()),
      warning_limiter_(std::make_unique<WarningLimiter>(kWarningPeriod)),
      metrics_(std::make_unique<RuntimeMetrics>()),
      callback_state_(std::make_shared<CallbackState>()),
      core_(mujoco_sim::core::CoreOptions{
          config_.server_name, config_.tick_timeout_ms,
          config_.activation_timeout_ms,
          config_.allow_timed_out_adapter_replacement}),
      sensor_runtime_(std::make_unique<SensorRuntime>()),
      control_endpoint_(std::make_unique<ControlEndpoint>()),
      observation_endpoint_(std::make_unique<ObservationEndpoint>()),
      scheduler_(std::make_unique<SimulationScheduler>()) {}

ServerApplication::~ServerApplication() {
  std::string error;
  if (!RunWithExceptionBoundary([&] { RequestStop(); }, &error)) {
    LogBoundaryFailure(logger_->get(), "server_destructor", "Stop", error);
  }
  if (!RunWithExceptionBoundary([&] { ShutdownTransport(); }, &error)) {
    LogBoundaryFailure(logger_->get(), "server_destructor",
                       "ShutdownTransport", error);
  }
  if (owns_ecal_ && transport_shutdown_succeeded_) {
    if (const auto log = logger_->get()) {
      try {
        log->debug("event=ecal_finalizing");
      } catch (...) {
      }
    }
    if (!RunWithExceptionBoundary([] { eCAL::Finalize(); }, &error)) {
      LogBoundaryFailure(logger_->get(), "server_destructor", "eCAL::Finalize",
                         error);
    }
  } else if (owns_ecal_) {
    try {
      if (const auto log = logger_->get()) {
        log->error(
            "event=ecal_finalize_skipped reason=transport_cleanup_failed");
      }
    } catch (...) {
    }
  }
  RunWithExceptionBoundary([&] { logger_->Flush(); }, nullptr);
}

bool ServerApplication::Start(std::string* error_message) {
  if (error_message == nullptr ||
      !ValidateServerConfig(config_, error_message)) {
    if (error_message != nullptr) {
      std::cerr << *error_message << '\n';
    }
    return false;
  }
  if (!logger_->Initialize(config_, error_message)) {
    std::cerr << *error_message << '\n';
    return false;
  }
  const auto log = logger_->get();
  log->info(
      "event=server_starting server_name={} model_path={} initial_keyframe={} "
      "mode={} max_ticks={} tick_timeout_ms={} "
      "activation_timeout_ms={} allow_timed_out_adapter_replacement={} "
      "status_period_ms={} log_level={} log_file={} "
      "log_summary_period_ms={}",
      LogValue(config_.server_name), LogValue(config_.model_path),
      LogValue(config_.initial_keyframe),
      config_.headless ? "headless" : "gui", config_.max_ticks,
      config_.tick_timeout_ms,
      config_.activation_timeout_ms,
      config_.allow_timed_out_adapter_replacement,
      config_.status_period_ms, config_.log_level, LogValue(config_.log_file),
      config_.log_summary_period_ms);
  bool ecal_was_initialized = false;
  bool ecal_initialized = false;
  std::string ecal_error;
  if (!RunWithExceptionBoundary(
          [&] {
            ecal_was_initialized = eCAL::IsInitialized();
            if (!ecal_was_initialized) {
              owns_ecal_ = eCAL::Initialize("mujoco-sim-server");
              ecal_initialized = owns_ecal_;
            } else {
              ecal_initialized = true;
            }
          },
          &ecal_error)) {
    *error_message = "eCAL initialization threw: " + ecal_error;
    log->error("event=server_start_failed stage=ecal error={}",
               LogValue(*error_message));
    return false;
  }
  if (!ecal_was_initialized) {
    if (!ecal_initialized) {
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

  if (!sensor_runtime_->Initialize(&core_, config_, error_message)) {
    log->error("event=server_start_failed stage=sensor_runtime error={}",
               LogValue(*error_message));
    return false;
  }
  log->info("event=sensor_runtime_initialized streams={}",
            config_.sensor_streams.size());

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

  bool services_registered = false;
  std::string transport_error;
  if (!RunWithExceptionBoundary(
          [&] { services_registered = RegisterServices(error_message); },
          &transport_error)) {
    *error_message = "eCAL registry setup threw: " + transport_error;
    log->error("event=server_start_failed stage=registry error={}",
               LogValue(*error_message));
    return false;
  }
  if (!services_registered) {
    log->error("event=server_start_failed stage=registry error={}",
               LogValue(*error_message));
    return false;
  }
  if (!RunWithExceptionBoundary(
          [&] { RegisterSubscribersAndPublishers(); }, &transport_error)) {
    *error_message = "eCAL endpoint setup threw: " + transport_error;
    log->error("event=server_start_failed stage=runtime_endpoints error={}",
               LogValue(*error_message));
    return false;
  }
  log->info("event=server_started server_name={}",
            LogValue(config_.server_name));
  return true;
}

bool ServerApplication::RegisterServices(std::string* error_message) {
  const std::string service_name =
      mujoco_sim::protocol::RegistryServiceName(config_.server_name);
  registry_endpoint_ = std::make_unique<RegistryEndpoint>(service_name);
  const auto callback_logger = logger_->get();
  using Handler = int (ServerApplication::*)(const std::string&, std::string*);
  const auto register_method =
      [this, callback_logger, error_message](const char* method,
                                             Handler handler) {
        const bool registered = registry_endpoint_->RegisterMethod(
            method,
            [this, callback_state = callback_state_, callback_logger, method,
             handler](const std::string& request, std::string* response) {
              return InvokeGuardedServiceCallback(
                  callback_state, callback_logger, method,
                  [&] { return (this->*handler)(request, response); });
            });
        if (!registered) {
          *error_message = "failed to register method: " +
                           std::string(method);
        }
        return registered;
      };
  if (!register_method(mujoco_sim::protocol::kGetAvailableResourcesMethod,
                       &ServerApplication::HandleGetAvailableResources) ||
      !register_method(mujoco_sim::protocol::kRegisterAdapterMethod,
                       &ServerApplication::HandleRegisterAdapter) ||
      !register_method(mujoco_sim::protocol::kActivateAdapterMethod,
                       &ServerApplication::HandleActivateAdapter) ||
      !register_method(mujoco_sim::protocol::kUnregisterAdapterMethod,
                       &ServerApplication::HandleUnregisterAdapter) ||
      !register_method(mujoco_sim::protocol::kGetSensorStreamsMethod,
                       &ServerApplication::HandleGetSensorStreams)) {
    return false;
  }
  logger_->get()->info(
      "event=registry_registered service={} methods=5",
      LogValue(service_name));
  return true;
}

void ServerApplication::RegisterSubscribersAndPublishers() {
  const std::string commit_topic =
      mujoco_sim::protocol::CommitTopicName(config_.server_name);
  const std::string status_topic =
      mujoco_sim::protocol::StatusTopicName(config_.server_name);
  const std::string adapter_heartbeat_topic =
      mujoco_sim::protocol::AdapterHeartbeatTopicName(config_.server_name);
  const auto callback_logger = logger_->get();
  control_endpoint_->SubscribeCommits(
      commit_topic,
      [this, callback_state = callback_state_, callback_logger](
          const void* data, std::size_t size) noexcept {
        InvokeGuardedSubscriberCallback(
            callback_state, callback_logger, "AdapterCommit",
            [&] { HandleCommitBytes(data, size); });
      });

  registry_endpoint_->SubscribeHeartbeat(
      adapter_heartbeat_topic,
      [this, callback_state = callback_state_, callback_logger](
          const void* data, std::size_t size) noexcept {
        InvokeGuardedSubscriberCallback(
            callback_state, callback_logger, "AdapterHeartbeat", [&] {
              HandleAdapterHeartbeatBytes(data, size);
            });
      });

  status_endpoint_ = std::make_unique<StatusEndpoint>(status_topic);
  logger_->get()->info(
      "event=runtime_endpoints_registered commit_topic={} "
      "adapter_heartbeat_topic={} status_topic={}",
      LogValue(commit_topic), LogValue(adapter_heartbeat_topic),
      LogValue(status_topic));
}

void ServerApplication::ShutdownTransport() {
  BestEffortCleanup cleanup(
      [this](std::string_view operation, const std::string& error) {
        LogBoundaryFailure(logger_->get(), "transport_cleanup", operation,
                           error);
      });
  cleanup.Run("sensor_runtime.RequestStop", [this] {
    if (sensor_runtime_) sensor_runtime_->RequestStop();
  });
  cleanup.Run("callback_state.StopAccepting",
              [this] { callback_state_->StopAccepting(); });
  cleanup.Run("commit_subscriber.stop", [this] {
    if (control_endpoint_) control_endpoint_->StopSubscriber();
  });
  cleanup.Run("registry_callbacks.stop", [this] {
    if (registry_endpoint_) registry_endpoint_->StopCallbacks();
  });
  cleanup.Run("callback_state.WaitForIdle",
              [this] { callback_state_->WaitForIdle(); });
  cleanup.Run("sensor_runtime.Stop", [this] {
    if (sensor_runtime_) sensor_runtime_->Stop();
  });
  cleanup.Run("registry.reset", [this] {
    if (registry_endpoint_) registry_endpoint_->ResetService();
  });
  cleanup.Run("adapter_publishers.clear", [this] {
    if (control_endpoint_) control_endpoint_->ClearPublishers();
    if (observation_endpoint_) observation_endpoint_->ClearPublishers();
  });
  cleanup.Run("status_publisher.reset",
              [this] {
                if (status_endpoint_) status_endpoint_->Reset();
              });
  transport_shutdown_succeeded_ = cleanup.succeeded();
  if (logger_->get()) {
    logger_->get()->debug("event=transport_stopped");
  }
}

int ServerApplication::Run() {
  try {
    if (viewer_ != nullptr) {
      return RunGraphical();
    }
    return RunLoop();
  } catch (const std::exception& exception) {
    LogBoundaryFailure(logger_->get(), "server_run", "Run",
                       exception.what());
  } catch (...) {
    LogBoundaryFailure(logger_->get(), "server_run", "Run",
                       "non-standard exception");
  }
  try {
    StopWithReason("run_loop_exception");
    return FinishRun(2, "run_loop_exception");
  } catch (...) {
    return 2;
  }
}

int ServerApplication::RunGraphical() {
  int result = 1;
  std::thread server_thread([this, &result]() noexcept {
    try {
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
    } catch (const std::exception& exception) {
      result = 2;
      LogBoundaryFailure(logger_->get(), "gui_server_thread", "RunLoop",
                         exception.what());
      try {
        StopWithReason("gui_server_thread_exception");
        viewer_->RequestClose();
      } catch (...) {
      }
    } catch (...) {
      result = 2;
      LogBoundaryFailure(logger_->get(), "gui_server_thread", "RunLoop",
                         "non-standard exception");
      try {
        StopWithReason("gui_server_thread_exception");
        viewer_->RequestClose();
      } catch (...) {
      }
    }
  });

  std::string render_error;
  const bool render_succeeded = RunWithExceptionBoundary(
      [&] { viewer_->RenderLoop(); }, &render_error);
  if (!render_succeeded) {
    LogBoundaryFailure(logger_->get(), "gui_render_thread", "RenderLoop",
                       render_error);
  }
  StopWithReason(render_succeeded ? "viewer_closed"
                                  : "viewer_render_exception");
  server_thread.join();
  return render_succeeded ? result : 2;
}

int ServerApplication::RunLoop() {
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
    std::vector<mujoco_sim::core::IssuedTickHandle> ticks;
    std::vector<mujoco_sim::v2::ObservationFrame> observation_frames;
    std::optional<mujoco_sim::core::StepCompletedEvent> scheduled_event;
    bool should_stop_success = false;
    bool should_stop_error = false;
    bool tick_retransmission = false;
    bool tick_opened = false;
    bool deadline_completed_barrier = false;
    bool scheduled_completed_barrier = false;
    bool publish_status = false;
    bool state_changed = false;
    mujoco_sim::v2::ServerState old_state =
        mujoco_sim::v2::SERVER_STATE_UNSPECIFIED;
    mujoco_sim::v2::ServerState new_state =
        mujoco_sim::v2::SERVER_STATE_UNSPECIFIED;
    std::string stop_reason;
    std::string state_message;
    std::string viewer_error;
    double deadline_barrier_latency_ms = 0.0;
    double scheduled_barrier_latency_ms = 0.0;
    std::vector<mujoco_sim::core::AdapterTransition> transitions;
    const auto now = std::chrono::steady_clock::now();

    if (scheduler_->step_pipeline_busy()) {
      std::unique_lock lock(mutex_);
      const auto deadline = now + std::chrono::milliseconds(1);
      scheduler_->WaitUntil(lock, deadline);
      metrics_->RecordSchedulerWakeup(
          std::chrono::steady_clock::now() > deadline);
      continue;
    }

    {
      std::unique_lock simulation_lock(simulation_mutex_);
      std::unique_lock lock(mutex_);
      auto deadlines = core_.CheckDeadlines(now);
      transitions = std::move(deadlines.transitions);
      if (deadlines.step_completed) {
        if (!scheduler_->BeginStepPipeline()) {
          core_.EnterFatalError("step pipeline overlap after deadline");
          should_stop_error = true;
          stop_reason = "step_pipeline_overlap";
        }
        deadline_completed_barrier = true;
        deadline_barrier_latency_ms = metrics_->RecordBarrierCompleted(
            std::chrono::steady_clock::now());
        observation_frames =
            std::move(deadlines.step_completed->observer_frames);
        sensor_runtime_->OnStepCompleted(&core_);
      }
      for (const auto& transition : transitions) {
        if (transition.new_state == mujoco_sim::v2::TIMED_OUT) {
          metrics_->RecordTimeout();
        } else if (transition.old_state == mujoco_sim::v2::PROVISIONAL &&
                   transition.new_state == mujoco_sim::v2::UNREGISTERED) {
          metrics_->RecordActivationExpiration();
        }
      }
      if (!transitions.empty() ||
          deadlines.step_completed.has_value()) {
        status_dirty_ = true;
      }
      if (core_.current_tick_id() != observed_tick_id) {
        observed_tick_id = core_.current_tick_id();
        next_tick = std::chrono::steady_clock::now() + step_period;
      }

      if (core_.server_state() == mujoco_sim::v2::ERROR_PAUSED) {
        should_stop_error = true;
        stop_reason = "core_error_paused";
      } else if (config_.max_ticks > 0 &&
                 core_.current_tick_id() >= config_.max_ticks) {
        should_stop_success = true;
        stop_reason = "max_ticks";
      } else if (!scheduler_->step_pipeline_busy() && !core_.tick_open() &&
                 now >= next_tick) {
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
      lock.unlock();
      simulation_lock.unlock();
      if (should_stop_success || should_stop_error) {
        sensor_runtime_->RequestStop();
      }
      if (deadlines.step_completed) {
        CompleteSensorFence();
      }
      if (!observation_frames.empty()) {
        PublishObservationFrames(observation_frames);
        observation_frames.clear();
      }
      if (deadlines.step_completed) {
        FinishStepPipeline(*deadlines.step_completed);
      }
      for (const auto& transition : transitions) {
        if (transition.new_state == mujoco_sim::v2::UNREGISTERED) {
          if (transition.role == mujoco_sim::v2::OBSERVER) {
            observation_endpoint_->ErasePublisher(transition.adapter_id);
          } else {
            control_endpoint_->ErasePublisher(transition.adapter_id);
          }
        }
      }
    }

    if (!ticks.empty()) {
      PublishTicks(ticks, tick_retransmission);
    }
    if (tick_opened) {
      std::unique_lock simulation_lock(simulation_mutex_);
      {
        std::lock_guard lock(mutex_);
        auto empty_barrier = core_.CompleteEmptyBarrier();
        if (empty_barrier.step_completed) {
          scheduled_event = std::move(empty_barrier.step_completed);
          if (!scheduler_->BeginStepPipeline()) {
            core_.EnterFatalError("step pipeline overlap after empty barrier");
          }
          const auto completed_at = std::chrono::steady_clock::now();
          observed_tick_id = core_.current_tick_id();
          next_tick = completed_at + step_period;
          scheduled_completed_barrier = true;
          scheduled_barrier_latency_ms =
              metrics_->RecordBarrierCompleted(completed_at);
          auto scheduled_frames =
              std::move(scheduled_event->observer_frames);
          sensor_runtime_->OnStepCompleted(&core_);
          observation_frames.insert(
              observation_frames.end(),
              std::make_move_iterator(scheduled_frames.begin()),
              std::make_move_iterator(scheduled_frames.end()));
        }
      }
      simulation_lock.unlock();
      if (scheduled_event) {
        CompleteSensorFence();
      }
      if (!observation_frames.empty()) {
        PublishObservationFrames(observation_frames);
        observation_frames.clear();
      }
      if (scheduled_event) {
        FinishStepPipeline(*scheduled_event);
      }
    }
    for (const auto& transition : transitions) {
      if (transition.new_state == mujoco_sim::v2::TIMED_OUT) {
        logger_->get()->warn(
            "event=adapter_timed_out adapter_id={} session_id={} "
            "tick_id={} message={}",
            LogValue(transition.adapter_id), transition.session_id,
            transition.tick_id, LogValue(transition.message));
      } else if (transition.old_state == mujoco_sim::v2::PROVISIONAL &&
                 transition.new_state == mujoco_sim::v2::UNREGISTERED) {
        logger_->get()->warn(
            "event=adapter_activation_expired adapter_id={} "
            "session_id={} tick_id={}",
            LogValue(transition.adapter_id), transition.session_id,
            transition.tick_id);
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
    scheduler_->WaitUntil(lock, wake_deadline);
    metrics_->RecordSchedulerWakeup(
        std::chrono::steady_clock::now() > wake_deadline);
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

void ServerApplication::MaybeLogSummary(
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
        core_.controller_count(mujoco_sim::v2::PROVISIONAL);
    summary.active_controllers =
        core_.controller_count(mujoco_sim::v2::ACTIVE);
    summary.timed_out_controllers =
        core_.controller_count(mujoco_sim::v2::TIMED_OUT);
    summary.error_controllers =
        core_.controller_count(mujoco_sim::v2::ERROR);
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
      "tick_retransmissions={} tick_serializations={} tick_cache_hits={} "
      "tick_cache_misses={} tick_cached_bytes={} layout_bytes_avoided={} "
      "layout_hash_mismatches={} blocking_wait_count={} "
      "blocking_wait_total_ms={:.3f} blocking_wait_max_ms={:.3f} "
      "scheduler_wakeups={} scheduler_late_wakeups={} commits={} "
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
      runtime.tick_serializations, runtime.tick_cache_hits,
      runtime.tick_cache_misses, runtime.tick_cached_bytes,
      runtime.layout_bytes_avoided, runtime.layout_hash_mismatches,
      runtime.blocking_wait_count, runtime.blocking_wait_total_ms,
      runtime.blocking_wait_max_ms, runtime.scheduler_wakeups,
      runtime.scheduler_late_wakeups,
      runtime.commits, runtime.duplicate_commits, runtime.rejected_commits,
      runtime.timeouts, runtime.recoveries, runtime.replacements,
      runtime.local_errors, runtime.activation_expirations,
      runtime.malformed_messages,
      runtime.suppressed_warnings, runtime.rates.step_rate_hz,
      runtime.rates.real_time_factor,
      runtime.rates.barrier_latency_avg_ms,
      runtime.rates.barrier_latency_max_ms);
}

int ServerApplication::FinishRun(int exit_code, const std::string& reason) {
  if (sensor_runtime_) {
    sensor_runtime_->RequestStop();
  }
  callback_state_->StopAccepting();
  callback_state_->WaitForIdle();
  {
    std::lock_guard lock(mutex_);
    if (exit_code == 0) {
      core_.MarkStopped(reason);
    } else if (core_.server_state() != mujoco_sim::v2::ERROR_PAUSED) {
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

void ServerApplication::StopWithReason(const std::string& reason) {
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
  if (sensor_runtime_) {
    sensor_runtime_->RequestStop();
  }
  scheduler_->NotifyLifecycleChanged();
  if (changed && should_log && logger_->get()) {
    try {
      logger_->get()->debug("event=stop_requested reason={}", reason);
    } catch (...) {
    }
  }
}

void ServerApplication::RequestStop(std::string reason) {
  StopWithReason(reason);
}

bool ServerApplication::ShouldLogWarning(const char* key,
                                  std::uint64_t* suppressed) {
  const WarningDecision decision = warning_limiter_->Admit(key);
  *suppressed = decision.suppressed;
  if (!decision.emit) {
    std::lock_guard lock(mutex_);
    metrics_->RecordSuppressedWarning();
  }
  return decision.emit;
}

void ServerApplication::RecordMalformedMessage() {
  std::lock_guard lock(mutex_);
  metrics_->RecordMalformedMessage();
}

void ServerApplication::FinishStepPipeline(
    const mujoco_sim::core::StepCompletedEvent& event) {
  logger_->get()->debug(
      "event=tick_cache_invalidated completed_tick_id={} next_tick_id={}",
      event.completed_tick_id, event.next_tick_id);
  logger_->get()->debug(
      "event=scheduler_step_completed completed_tick_id={} next_tick_id={} "
      "simulation_time_s={:.6f} source={}",
      event.completed_tick_id, event.next_tick_id, event.simulation_time_s,
      static_cast<int>(event.source));
  scheduler_->CompleteStepPipeline();
}

void ServerApplication::CompleteSensorFence() {
  const auto started = std::chrono::steady_clock::now();
  logger_->get()->trace("event=blocking_sensor_fence_started");
  sensor_runtime_->CompleteBlockingEnqueues(
      [this] { return stop_requested_.load() || !eCAL::Ok(); });
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started)
          .count();
  {
    std::lock_guard lock(mutex_);
    metrics_->RecordBlockingWait(elapsed_ms);
  }
  logger_->get()->trace(
      "event=blocking_sensor_fence_completed elapsed_ms={:.3f}", elapsed_ms);
}

std::vector<mujoco_sim::core::IssuedTickHandle> ServerApplication::OpenTickLocked() {
  if (scheduler_->step_pipeline_busy()) {
    core_.EnterFatalError("attempted to open Tick while step pipeline is busy");
    return {};
  }
  const bool was_open = core_.tick_open();
  std::unordered_map<std::string, std::uint64_t> timed_out_sessions;
  if (!was_open) {
    const auto before = core_.MakeStatus();
    for (const auto& adapter : before.adapters()) {
      if (adapter.role() == mujoco_sim::v2::CONTROLLER &&
          adapter.state() == mujoco_sim::v2::TIMED_OUT) {
        timed_out_sessions.emplace(adapter.adapter_id(),
                                   adapter.session_id());
      }
    }
  }
  auto ticks = core_.OpenIssuedTicks();
  std::uint64_t cached_bytes = 0;
  std::uint64_t layout_bytes_avoided = 0;
  for (const auto& tick : ticks) {
    cached_bytes += tick->serialized_bytes.size();
    layout_bytes_avoided += tick->layout_bytes_avoided;
  }
  metrics_->RecordTickCache(was_open ? 0U : ticks.size(),
                            was_open ? ticks.size() : 0U, 0U, cached_bytes,
                            layout_bytes_avoided);
  if (!was_open && core_.tick_open()) {
    metrics_->RecordTickOpened(std::chrono::steady_clock::now());
    for (const auto& tick : ticks) {
      logger_->get()->debug(
          "event=tick_cache_created adapter_id={} tick_id={} bytes={} "
          "layout_bytes_avoided={} serializations={}",
          LogValue(tick->identity.adapter_id), tick->identity.tick_id,
          tick->serialized_bytes.size(), tick->layout_bytes_avoided,
          tick->serialization_count);
    }
    const auto after = core_.MakeStatus();
    for (const auto& adapter : after.adapters()) {
      if (adapter.role() != mujoco_sim::v2::CONTROLLER) {
        continue;
      }
      const auto previous = timed_out_sessions.find(adapter.adapter_id());
      if (previous != timed_out_sessions.end() &&
          previous->second == adapter.session_id() &&
          adapter.state() == mujoco_sim::v2::ACTIVE) {
        status_dirty_ = true;
        logger_->get()->info(
            "event=adapter_recovered adapter_id={} session_id={} "
            "tick_id={}",
            LogValue(adapter.adapter_id()), adapter.session_id(),
            core_.current_tick_id());
      }
    }
  }
  return ticks;
}

void ServerApplication::PublishTicks(
    const std::vector<mujoco_sim::core::IssuedTickHandle>& ticks,
    bool retransmission) {
  struct PublishAttempt {
    mujoco_sim::core::IssuedTickHandle issued;
    std::string topic;
    std::size_t subscribers_before = 0;
    std::size_t subscribers_after = 0;
    bool attempted = false;
    bool sent = false;
    bool publisher_created = false;
    std::string transport_error;
  };

  std::uint64_t attempted_count = 0;
  std::uint64_t sent_count = 0;
  std::uint64_t serialization_failure_count = 0;
  std::uint64_t failure_count = 0;
  std::uint64_t no_subscriber_count = 0;
  std::vector<PublishAttempt> attempts;
  attempts.reserve(ticks.size());
  for (const auto& issued : ticks) {
    PublishAttempt attempt;
    attempt.issued = issued;
    attempts.push_back(std::move(attempt));
  }

  for (auto& attempt : attempts) {
    bool current = false;
    {
      std::lock_guard lock(mutex_);
      current = core_.IsIssuedTickCurrent(attempt.issued->identity);
    }
    if (!current) continue;
    const auto result =
        control_endpoint_->Publish(attempt.issued, config_.server_name);
    attempt.topic = result.topic;
    attempt.subscribers_before = result.subscribers_before;
    attempt.subscribers_after = result.subscribers_after;
    attempt.attempted = result.attempted;
    attempt.sent = result.sent;
    attempt.publisher_created = result.publisher_created;
    attempt.transport_error = result.transport_error;
  }

  for (const auto& attempt : attempts) {
    if (!attempt.attempted) {
      continue;
    }
    const auto& tick = attempt.issued->message;
    if (attempt.publisher_created) {
      logger_->get()->debug(
          "event=tick_publisher_created adapter_id={} topic={}",
          LogValue(tick.adapter_id()), LogValue(attempt.topic));
    }
    ++attempted_count;
    if (!attempt.transport_error.empty()) {
      std::uint64_t suppressed = 0;
      if (ShouldLogWarning("tick_transport_exception", &suppressed)) {
        logger_->get()->error(
            "event=ecal_exception boundary=publisher operation=Tick "
            "adapter_id={} tick_id={} error={} suppressed={}",
            LogValue(tick.adapter_id()), tick.header().tick_id(),
            LogValue(attempt.transport_error), suppressed);
      }
      ++failure_count;
      continue;
    }
    const SendOutcome outcome = ClassifySendOutcome(
        attempt.sent, attempt.subscribers_before, attempt.subscribers_after);
    if (outcome != SendOutcome::kSent) {
      if (outcome == SendOutcome::kNoSubscribers) {
        ++no_subscriber_count;
        logger_->get()->trace(
            "event=message_not_sent message_type=Tick reason=no_subscribers "
            "adapter_id={} tick_id={} bytes={} subscribers_before={} "
            "subscribers_after={}",
            LogValue(tick.adapter_id()), tick.header().tick_id(),
            attempt.issued->serialized_bytes.size(), attempt.subscribers_before,
            attempt.subscribers_after);
        continue;
      }
      std::uint64_t suppressed = 0;
      if (ShouldLogWarning("tick_send_rejected", &suppressed)) {
        logger_->get()->warn(
            "event=message_send_rejected message_type=Tick adapter_id={} "
            "tick_id={} bytes={} subscribers_before={} subscribers_after={} "
            "suppressed={}",
            LogValue(tick.adapter_id()), tick.header().tick_id(),
            attempt.issued->serialized_bytes.size(), attempt.subscribers_before,
            attempt.subscribers_after, suppressed);
      }
      ++failure_count;
      continue;
    }
    ++sent_count;
    logger_->get()->trace(
        "event=tick_published adapter_id={} tick_id={} bytes={}",
        LogValue(tick.adapter_id()), tick.header().tick_id(),
        attempt.issued->serialized_bytes.size());
  }
  if (attempted_count > 0 || serialization_failure_count > 0) {
    std::lock_guard lock(mutex_);
    metrics_->RecordTickPublishBatch(
        attempted_count, serialization_failure_count, sent_count, failure_count,
        no_subscriber_count,
        retransmission ? attempted_count : 0U);
  }
}

void ServerApplication::PublishObservationFrames(
    const std::vector<mujoco_sim::v2::ObservationFrame>& frames) {
  for (const auto& frame : frames) {
    const auto publish =
        observation_endpoint_->Publish(frame, config_.server_name);
    if (publish.serialization_failed) {
      logger_->get()->error(
          "event=message_serialize_failed message_type=ObservationFrame "
          "adapter_id={} sequence={}",
          LogValue(frame.adapter_id()), frame.sequence());
      continue;
    }
    const bool sent = publish.sent;
    const auto subscribers_before = publish.subscribers_before;
    const auto subscribers_after = publish.subscribers_after;
    const auto& transport_error = publish.transport_error;
    if (!transport_error.empty()) {
      std::uint64_t suppressed = 0;
      if (ShouldLogWarning("observation_transport_exception", &suppressed)) {
        logger_->get()->error(
            "event=ecal_exception boundary=publisher "
            "operation=ObservationFrame adapter_id={} sequence={} error={} "
            "suppressed={}",
            LogValue(frame.adapter_id()), frame.sequence(),
            LogValue(transport_error), suppressed);
      }
      continue;
    }
    const auto outcome = ClassifySendOutcome(
        sent, subscribers_before, subscribers_after);
    if (outcome == SendOutcome::kSent) {
      logger_->get()->trace(
          "event=observation_published adapter_id={} sequence={} tick_id={} "
          "bytes={}",
          LogValue(frame.adapter_id()), frame.sequence(),
          frame.header().tick_id(), publish.bytes);
    } else if (outcome == SendOutcome::kRejectedWithSubscribers) {
      std::uint64_t suppressed = 0;
      if (ShouldLogWarning("observation_send_rejected", &suppressed)) {
        logger_->get()->warn(
            "event=message_send_rejected message_type=ObservationFrame "
            "adapter_id={} sequence={} subscribers_before={} "
            "subscribers_after={} suppressed={}",
            LogValue(frame.adapter_id()), frame.sequence(),
            subscribers_before, subscribers_after, suppressed);
      }
    }
  }
}

void ServerApplication::PublishStatus() {
  if (!status_endpoint_ || !status_endpoint_->available()) {
    return;
  }
  mujoco_sim::v2::ServerStatus status;
  {
    std::lock_guard lock(mutex_);
    status = core_.MakeStatus();
  }
  sensor_runtime_->AppendStatus(&status);
  const auto publish = status_endpoint_->Publish(status);
  if (publish.serialization_failed) {
    logger_->get()->error(
        "event=message_serialize_failed message_type=ServerStatus");
    return;
  }
  const auto subscribers_before = publish.subscribers_before;
  const auto subscribers_after = publish.subscribers_after;
  const bool sent = publish.sent;
  const auto& transport_error = publish.transport_error;
  if (!transport_error.empty()) {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("status_transport_exception", &suppressed)) {
      logger_->get()->error(
          "event=ecal_exception boundary=publisher operation=ServerStatus "
          "tick_id={} error={} suppressed={}",
          status.header().tick_id(), LogValue(transport_error), suppressed);
    }
    return;
  }
  const SendOutcome outcome =
      ClassifySendOutcome(sent, subscribers_before, subscribers_after);
  if (outcome != SendOutcome::kSent) {
    if (outcome == SendOutcome::kNoSubscribers) {
      logger_->get()->trace(
          "event=message_not_sent message_type=ServerStatus "
          "reason=no_subscribers tick_id={} bytes={} subscribers_before={} "
          "subscribers_after={}",
          status.header().tick_id(), publish.bytes, subscribers_before,
          subscribers_after);
      return;
    }
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("status_send_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=message_send_rejected message_type=ServerStatus tick_id={} "
          "bytes={} subscribers_before={} subscribers_after={} suppressed={}",
          status.header().tick_id(), publish.bytes, subscribers_before,
          subscribers_after, suppressed);
    }
    return;
  }
  logger_->get()->debug(
      "event=status_published state={} tick_id={} adapters={} bytes={}",
      ServerStateName(status.state()), status.header().tick_id(),
      status.adapters_size(), publish.bytes);
}

int ServerApplication::HandleGetAvailableResources(const std::string& request,
                                            std::string* response) {
  mujoco_sim::v2::GetAvailableResourcesRequest parsed;
  if (!registry_endpoint_->Parse(request, &parsed)) {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("resources_request_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_malformed method=GetAvailableResources "
          "bytes={} max_bytes={} suppressed={}",
          request.size(), kMaxProtocolPayloadBytes, suppressed);
    }
    const int result = ParseError<mujoco_sim::v2::GetAvailableResourcesResponse>(
        mujoco_sim::v2::INVALID_ARGUMENT, "invalid request",
        *registry_endpoint_, response);
    if (result != 0) {
      logger_->get()->error(
          "event=message_serialize_failed "
          "message_type=GetAvailableResourcesResponse");
    }
    return result;
  }
  mujoco_sim::v2::GetAvailableResourcesResponse result;
  {
    std::lock_guard lock(mutex_);
    result = core_.GetAvailableResources(parsed);
  }
  if (result.code() == mujoco_sim::v2::OK) {
    logger_->get()->debug(
        "event=resources_requested code={} observations={} commands={}",
        ErrorCodeName(result.code()), result.observations_size(),
        result.commands_size());
  } else {
    if (result.code() == mujoco_sim::v2::VERSION_MISMATCH) {
      logger_->get()->warn(
          "event=protocol_version_rejected method=GetAvailableResources "
          "requested_major={} requested_minor={} expected_major=2 "
          "expected_minor=0",
          parsed.protocol_major(), parsed.protocol_minor());
    }
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("resources_request_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_rejected method=GetAvailableResources "
          "code={} message={} suppressed={}",
          ErrorCodeName(result.code()), LogValue(result.message()), suppressed);
    }
  }
  const int serialized = registry_endpoint_->Serialize(result, response);
  if (serialized != 0) {
    logger_->get()->error(
        "event=message_serialize_failed "
        "message_type=GetAvailableResourcesResponse");
  }
  return serialized;
}

int ServerApplication::HandleGetSensorStreams(const std::string& request,
                                       std::string* response) {
  mujoco_sim::v2::GetSensorStreamsRequest parsed;
  if (!registry_endpoint_->Parse(request, &parsed)) {
    RecordMalformedMessage();
    return ParseError<mujoco_sim::v2::GetSensorStreamsResponse>(
        mujoco_sim::v2::INVALID_ARGUMENT, "invalid request",
        *registry_endpoint_, response);
  }
  const auto result = sensor_runtime_->GetCatalog(parsed);
  if (result.code() == mujoco_sim::v2::OK) {
    logger_->get()->debug("event=sensor_streams_requested streams={}",
                          result.streams_size());
  } else {
    if (result.code() == mujoco_sim::v2::VERSION_MISMATCH) {
      logger_->get()->warn(
          "event=protocol_version_rejected method=GetSensorStreams "
          "requested_major={} requested_minor={} expected_major=2 "
          "expected_minor=0",
          parsed.protocol_major(), parsed.protocol_minor());
    }
    logger_->get()->warn(
        "event=service_request_rejected method=GetSensorStreams code={} "
        "message={}",
        ErrorCodeName(result.code()), LogValue(result.message()));
  }
  return registry_endpoint_->Serialize(result, response);
}

 int ServerApplication::HandleRegisterAdapter(const std::string& request,
                                      std::string* response) {
  mujoco_sim::v2::RegisterAdapterRequest parsed;
  if (!registry_endpoint_->Parse(request, &parsed)) {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("register_adapter_request_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=service_request_malformed method=RegisterAdapter bytes={} "
          "max_bytes={} suppressed={}",
          request.size(), kMaxProtocolPayloadBytes, suppressed);
    }
    return ParseError<mujoco_sim::v2::RegisterAdapterResponse>(
        mujoco_sim::v2::INVALID_ARGUMENT, "invalid request",
        *registry_endpoint_, response);
  }
  mujoco_sim::v2::RegisterAdapterResponse result;
  bool replaced = false;
  {
    std::lock_guard lock(mutex_);
    result = core_.RegisterAdapter(parsed);
    if (result.code() == mujoco_sim::v2::OK) {
      status_dirty_ = true;
      replaced = result.message().find("replaced") != std::string::npos;
      if (replaced) {
        metrics_->RecordReplacement();
      }
    }
  }
  if (result.code() == mujoco_sim::v2::OK) {
    logger_->get()->debug(
        "event=layout_hash_validated adapter_id={} hash_bytes={}",
        LogValue(result.adapter_id()), result.layout().layout_hash().size());
    logger_->get()->info(
        "event=adapter_registered adapter_id={} role={} session_id={} "
        "epoch={} observations={} commands={} timeout_ms={} replaced={}",
        LogValue(result.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(result.role()), result.session_id(),
        result.epoch(), result.layout().observations_size(),
        result.layout().commands_size(),
        parsed.timeout_ms() == 0 ? config_.tick_timeout_ms
                                 : parsed.timeout_ms(),
        replaced);
  } else {
    if (result.code() == mujoco_sim::v2::VERSION_MISMATCH) {
      logger_->get()->warn(
          "event=protocol_version_rejected method=RegisterAdapter "
          "requested_major={} requested_minor={} expected_major=2 "
          "expected_minor=0",
          parsed.protocol_major(), parsed.protocol_minor());
    }
    logger_->get()->warn(
        "event=adapter_request_rejected operation=register adapter_id={} "
        "role={} code={} message={}",
        LogValue(parsed.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(parsed.role()),
        ErrorCodeName(result.code()), LogValue(result.message()));
  }
  scheduler_->NotifyLifecycleChanged();
  return registry_endpoint_->Serialize(result, response);
}

int ServerApplication::HandleActivateAdapter(const std::string& request,
                                      std::string* response) {
  mujoco_sim::v2::ActivateAdapterRequest parsed;
  if (!registry_endpoint_->Parse(request, &parsed)) {
    RecordMalformedMessage();
    return ParseError<mujoco_sim::v2::ActivateAdapterResponse>(
        mujoco_sim::v2::INVALID_ARGUMENT, "invalid request",
        *registry_endpoint_, response);
  }
  mujoco_sim::v2::ActivateAdapterResponse result;
  {
    std::lock_guard lock(mutex_);
    result = core_.ActivateAdapter(parsed);
    if (result.code() == mujoco_sim::v2::OK) {
      status_dirty_ = true;
    }
  }
  if (result.code() == mujoco_sim::v2::OK) {
    logger_->get()->info(
        "event=adapter_activated adapter_id={} role={} session_id={} epoch={}",
        LogValue(parsed.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(parsed.role()), parsed.session_id(),
        parsed.epoch());
  } else {
    logger_->get()->warn(
        "event=adapter_request_rejected operation=activate adapter_id={} "
        "role={} code={} message={}",
        LogValue(parsed.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(parsed.role()),
        ErrorCodeName(result.code()), LogValue(result.message()));
  }
  scheduler_->NotifyLifecycleChanged();
  return registry_endpoint_->Serialize(result, response);
}

int ServerApplication::HandleUnregisterAdapter(const std::string& request,
                                        std::string* response) {
  mujoco_sim::v2::UnregisterAdapterRequest parsed;
  if (!registry_endpoint_->Parse(request, &parsed)) {
    RecordMalformedMessage();
    return ParseError<mujoco_sim::v2::UnregisterAdapterResponse>(
        mujoco_sim::v2::INVALID_ARGUMENT, "invalid request",
        *registry_endpoint_, response);
  }
  mujoco_sim::v2::UnregisterAdapterResponse result;
  std::vector<mujoco_sim::v2::ObservationFrame> observation_frames;
  std::optional<mujoco_sim::core::StepCompletedEvent> step_event;
  bool stepped = false;
  std::unique_lock simulation_lock(simulation_mutex_);
  {
    std::lock_guard lock(mutex_);
    auto unregister_result = core_.UnregisterAdapter(parsed);
    result = std::move(unregister_result.response);
    stepped = unregister_result.step_completed.has_value();
    step_event = std::move(unregister_result.step_completed);
    if (result.code() == mujoco_sim::v2::OK) {
      status_dirty_ = true;
      if (stepped) {
        if (!scheduler_->BeginStepPipeline()) {
          core_.EnterFatalError("step pipeline overlap after unregister");
        }
        metrics_->RecordBarrierCompleted(std::chrono::steady_clock::now());
        observation_frames =
            std::move(step_event->observer_frames);
        sensor_runtime_->OnStepCompleted(&core_);
      }
    }
  }
  simulation_lock.unlock();
  if (step_event) {
    CompleteSensorFence();
  }
  if (!observation_frames.empty()) {
    PublishObservationFrames(observation_frames);
  }
  if (step_event) {
    FinishStepPipeline(*step_event);
  }
  if (result.code() == mujoco_sim::v2::OK) {
    logger_->get()->info(
        "event=adapter_unregistered adapter_id={} role={} session_id={} "
        "epoch={} stepped={}",
        LogValue(parsed.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(parsed.role()), parsed.session_id(),
        parsed.epoch(), stepped);
    if (parsed.role() == mujoco_sim::v2::OBSERVER) {
      observation_endpoint_->ErasePublisher(parsed.adapter_id());
    } else {
      control_endpoint_->ErasePublisher(parsed.adapter_id());
    }
  } else {
    logger_->get()->warn(
        "event=adapter_request_rejected operation=unregister adapter_id={} "
        "role={} code={} message={}",
        LogValue(parsed.adapter_id()),
        mujoco_sim::v2::AdapterRole_Name(parsed.role()),
        ErrorCodeName(result.code()), LogValue(result.message()));
  }
  scheduler_->NotifyLifecycleChanged();
  return registry_endpoint_->Serialize(result, response);
}

void ServerApplication::HandleCommitBytes(const void* data, std::size_t size) {
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
  mujoco_sim::v2::AdapterCommit commit;
  if (!control_endpoint_->ParseCommit(data, size, &commit)) {
    record_malformed();
    return;
  }

  mujoco_sim::core::CommitResult result;
  double barrier_latency_ms = 0.0;
  std::vector<mujoco_sim::v2::ObservationFrame> observation_frames;
  std::unique_lock simulation_lock(simulation_mutex_);
  {
    std::lock_guard lock(mutex_);
    result = core_.SubmitCommit(commit);
    if (result.controller_state_changed || result.recovery_pending) {
      status_dirty_ = true;
    }
    barrier_latency_ms = metrics_->RecordCommit(
        result.duplicate, result.code != mujoco_sim::v2::OK,
        result.step_completed.has_value(),
        std::chrono::steady_clock::now());
    if (result.controller_state_changed) {
      metrics_->RecordLocalError();
    }
    if (result.recovery_pending) {
      metrics_->RecordRecovery();
    }
    if (result.step_completed) {
      if (!scheduler_->BeginStepPipeline()) {
        core_.EnterFatalError("step pipeline overlap after commit");
      }
      observation_frames = std::move(result.step_completed->observer_frames);
      sensor_runtime_->OnStepCompleted(&core_);
    }
  }
  simulation_lock.unlock();
  if (result.step_completed) {
    CompleteSensorFence();
  }
  if (!observation_frames.empty()) {
    PublishObservationFrames(observation_frames);
  }
  if (result.step_completed) FinishStepPipeline(*result.step_completed);
  if (result.code != mujoco_sim::v2::OK) {
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("commit_rejected", &suppressed)) {
      logger_->get()->warn(
          "event=commit_rejected adapter_id={} tick_id={} session_id={} "
          "epoch={} code={} message={} suppressed={}",
          LogValue(commit.adapter_id()), commit.header().tick_id(),
          commit.header().session_id(), commit.header().epoch(),
          ErrorCodeName(result.code), LogValue(result.message), suppressed);
    }
  } else if (result.duplicate) {
    logger_->get()->debug(
        "event=commit_duplicate adapter_id={} tick_id={} message={}",
        LogValue(commit.adapter_id()), commit.header().tick_id(),
        LogValue(result.message));
  } else {
    logger_->get()->trace(
        "event=commit_accepted adapter_id={} tick_id={} stepped={} "
        "commands={}",
        LogValue(commit.adapter_id()), commit.header().tick_id(),
        result.step_completed.has_value(), commit.commands_size());
  }
  if (result.controller_state_changed) {
    logger_->get()->error(
        "event=adapter_local_error adapter_id={} session_id={} "
        "tick_id={} state={} message={}",
        LogValue(commit.adapter_id()), commit.header().session_id(),
        commit.header().tick_id(),
        mujoco_sim::v2::ControllerState_Name(result.controller_state),
        LogValue(result.message));
  } else if (result.recovery_pending) {
    logger_->get()->info(
        "event=adapter_recovery_pending adapter_id={} session_id={} "
        "tick_id={}",
        LogValue(commit.adapter_id()), commit.header().session_id(),
        commit.header().tick_id());
  }
  if (result.step_completed) {
    logger_->get()->trace(
        "event=barrier_completed completed_tick_id={} "
        "barrier_latency_ms={:.3f}",
        commit.header().tick_id(), barrier_latency_ms);
  }
  scheduler_->NotifyLifecycleChanged();
}

 void ServerApplication::HandleAdapterHeartbeatBytes(const void* data,
                                             std::size_t size) {
  const auto record_malformed = [this, size]() {
    RecordMalformedMessage();
    std::uint64_t suppressed = 0;
    if (ShouldLogWarning("adapter_heartbeat_payload_malformed", &suppressed)) {
      logger_->get()->warn(
          "event=topic_payload_malformed message_type=AdapterHeartbeat "
          "bytes={} max_bytes={} suppressed={}",
          size, kMaxProtocolPayloadBytes, suppressed);
    }
  };
  mujoco_sim::v2::AdapterHeartbeat heartbeat;
  if (!registry_endpoint_->ParseHeartbeat(data, size, &heartbeat)) {
    record_malformed();
    return;
  }
  {
    std::lock_guard lock(mutex_);
    core_.AcceptHeartbeat(heartbeat);
  }
  logger_->get()->trace(
      "event=adapter_heartbeat_received adapter_id={} role={} sequence={} "
      "session_id={} epoch={}",
      LogValue(heartbeat.adapter_id()),
      mujoco_sim::v2::AdapterRole_Name(heartbeat.role()),
      heartbeat.last_observation_sequence(), heartbeat.header().session_id(),
      heartbeat.header().epoch());
  scheduler_->NotifyLifecycleChanged();
}

}  // namespace mujoco_sim::server
