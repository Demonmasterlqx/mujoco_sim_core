#include <mujoco_sim/server/server_application.hpp>

#include <ecal/ecal.h>
#include <spdlog/logger.h>

#include "ecal_server_transport.hpp"
#include "server_logging.hpp"
#include "server_runtime.hpp"
#include "server_application_test_harness.hpp"
#include "sim_viewer.hpp"
#include "simulation_loop.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <string_view>
#include <utility>

namespace mujoco_sim::server {
namespace {

constexpr auto kWarningPeriod = std::chrono::seconds(1);

void LogBoundaryFailure(const std::shared_ptr<spdlog::logger>& logger, std::string_view boundary,
                        std::string_view operation, std::string_view error) noexcept {
  try {
    if (logger) {
      logger->error("event=boundary_failure boundary={} operation={} error={}", boundary,
                    LogValue(operation), LogValue(error));
    }
  } catch (...) {
  }
}

}  // namespace

class detail::ServerApplicationImpl {
 public:
  explicit ServerApplicationImpl(ServerConfig server_config)
      : config(std::move(server_config)),
        logger(std::make_unique<ServerLogger>()),
        warning_limiter(std::make_unique<WarningLimiter>(kWarningPeriod)) {}

  ~ServerApplicationImpl() { Shutdown(); }

  bool Start(std::string* error_message) {
    if (error_message == nullptr || started) {
      if (error_message) *error_message = "server has already been started";
      return false;
    }
    if (!ValidateServerConfig(config, error_message)) {
      if (error_message) std::cerr << *error_message << '\n';
      return false;
    }
    if (!logger->Initialize(config, error_message)) {
      std::cerr << *error_message << '\n';
      return false;
    }
    const auto log = logger->get();
    std::string boundary_error;
    if (!RunWithExceptionBoundary(
            [&] {
              log->info(
                  "event=server_starting server_name={} model_path={} initial_keyframe={} "
                  "mode={} pacing_mode={} realtime_factor={} max_ticks={} "
                  "command_timeout_ms={} command_keepalive_ms={} "
                  "activation_timeout_ms={} status_period_ms={} "
                  "viewer_sync_period_ms={} viewer_control_input_enabled={} "
                  "viewer_mouse_force_enabled={} viewer_mouse_force_timeout_ms={} "
                  "log_level={} log_file={} "
                  "log_summary_period_ms={}",
                  LogValue(config.server_name), LogValue(config.model_path),
                  LogValue(config.initial_keyframe), config.headless ? "headless" : "gui",
                  config.pacing_mode, config.realtime_factor, config.max_ticks,
                  config.command_timeout_ms, config.command_keepalive_ms,
                  config.activation_timeout_ms, config.status_period_ms,
                  config.viewer_sync_period_ms, config.viewer_control_input_enabled,
                  config.viewer_mouse_force_enabled, config.viewer_mouse_force_timeout_ms,
                  config.log_level, LogValue(config.log_file), config.log_summary_period_ms);
            },
            &boundary_error)) {
      *error_message = "failed to write server startup log: " + boundary_error;
      return false;
    }
    if (!logger->healthy()) {
      *error_message = "server logging failed during startup";
      return false;
    }

    bool initialized = false;
    if (!RunWithExceptionBoundary(
            [&] {
              const bool already_initialized = eCAL::IsInitialized();
              if (already_initialized) {
                initialized = true;
              } else {
                owns_ecal = eCAL::Initialize("mujoco-sim-server");
                initialized = owns_ecal;
              }
            },
            &boundary_error) ||
        !initialized) {
      *error_message = boundary_error.empty() ? "failed to initialize eCAL"
                                              : "eCAL initialization threw: " + boundary_error;
      return false;
    }

    if (!config.headless) {
      viewer = std::make_unique<SimViewer>(SimViewer::InputOptions{
          .control_enabled = config.viewer_control_input_enabled,
          .mouse_force_enabled = config.viewer_mouse_force_enabled,
      });
      if (!viewer->Initialize(error_message)) return false;
      if (!RunWithExceptionBoundary([&] { log->info("event=viewer_initialized mode=gui"); },
                                    &boundary_error) ||
          !logger->healthy()) {
        *error_message = boundary_error.empty()
                             ? "server logging failed during viewer startup"
                             : "failed to write viewer startup log: " + boundary_error;
        return false;
      }
    } else {
      if (!RunWithExceptionBoundary([&] { log->info("event=viewer_initialized mode=headless"); },
                                    &boundary_error) ||
          !logger->healthy()) {
        *error_message = boundary_error.empty()
                             ? "server logging failed during viewer startup"
                             : "failed to write viewer startup log: " + boundary_error;
        return false;
      }
    }

    transport = std::make_unique<EcalServerTransport>(config.server_name, logger->get());
    loop = std::make_unique<SimulationLoop>(config, logger->get(), warning_limiter.get(),
                                            transport.get(), viewer.get());
    if (!loop->Initialize(error_message)) return false;
    if (!transport->Start(loop->MakeHandlers(), error_message)) return false;
    if (!loop->Start(error_message)) return false;

    if (!RunWithExceptionBoundary(
            [&] { log->info("event=server_started server_name={}", LogValue(config.server_name)); },
            &boundary_error) ||
        !logger->healthy()) {
      *error_message = boundary_error.empty()
                           ? "server logging failed while recording startup"
                           : "failed to write server started log: " + boundary_error;
      RequestStop("logging_startup_failure");
      return false;
    }
    started = true;
    return true;
  }

  int Run() {
    {
      std::lock_guard lock(mutex);
      if (!started) return 1;
      if (run_completed) return exit_code;
    }

    bool render_succeeded = true;
    if (viewer) {
      std::string render_error;
      std::string boundary_error;
      const bool render_boundary_succeeded = RunWithExceptionBoundary(
          [this, &render_succeeded, &render_error] {
            render_succeeded = viewer->RenderLoop(&render_error);
          },
          &boundary_error);
      if (!render_boundary_succeeded) {
        render_succeeded = false;
        render_error = std::move(boundary_error);
      }
      if (!render_succeeded) {
        LogBoundaryFailure(logger->get(), "gui_render_thread", "RenderLoop", render_error);
        loop->RequestFailure("viewer_sync_failed");
      } else {
        RequestStop("viewer_closed");
      }
    }

    int result = 2;
    std::string join_error;
    if (!RunWithExceptionBoundary([&] { result = loop->Join(); }, &join_error)) {
      LogBoundaryFailure(logger->get(), "server_run", "simulation_loop_join", join_error);
    }
    if (!render_succeeded) result = 2;
    std::string transport_error;
    if (!RunWithExceptionBoundary([this] { transport->StopInputs(); }, &transport_error)) {
      LogBoundaryFailure(logger->get(), "server_run", "transport_stop_inputs", transport_error);
      result = 2;
    }
    if (!RunWithExceptionBoundary([this] { transport->Reset(); }, &transport_error)) {
      LogBoundaryFailure(logger->get(), "server_run", "transport_reset", transport_error);
      result = 2;
    }
    std::string flush_error;
    if (!RunWithExceptionBoundary([this] { logger->Flush(); }, &flush_error)) {
      LogBoundaryFailure(logger->get(), "server_run", "logger_flush", flush_error);
      result = 2;
    }
    if (!logger->healthy()) result = 2;
    {
      std::lock_guard lock(mutex);
      exit_code = result;
      run_completed = true;
    }
    return result;
  }

  void RequestStop(std::string reason) noexcept {
    std::string error;
    if (!RunWithExceptionBoundary(
            [this, &reason] {
              if (loop) loop->RequestStop(std::move(reason));
            },
            &error)) {
      LogBoundaryFailure(logger ? logger->get() : nullptr, "server_request_stop", "simulation_loop",
                         error);
    }
  }

  bool RequestScreenshot() {
    SimViewer* active_viewer = nullptr;
    {
      std::lock_guard lock(mutex);
      if (!started || run_completed || !viewer) return false;
      active_viewer = viewer.get();
    }
    active_viewer->RequestScreenshot();
    return true;
  }

  bool RequestViewerSyncFailureForTesting() noexcept {
    SimViewer* active_viewer = nullptr;
    {
      std::lock_guard lock(mutex);
      if (!started || run_completed || !viewer) return false;
      active_viewer = viewer.get();
    }
    active_viewer->RequestSyncFailureForTesting();
    return true;
  }

  void Shutdown() noexcept {
    std::string error;
    if (loop &&
        !RunWithExceptionBoundary([this] { loop->RequestStop("server_destroyed"); }, &error)) {
      LogBoundaryFailure(logger ? logger->get() : nullptr, "server_destructor",
                         "simulation_loop_stop", error);
    }
    if (loop && !RunWithExceptionBoundary([this] { loop->Join(); }, &error)) {
      LogBoundaryFailure(logger ? logger->get() : nullptr, "server_destructor",
                         "simulation_loop_join", error);
    }
    if (transport && !RunWithExceptionBoundary([this] { transport->StopInputs(); }, &error)) {
      LogBoundaryFailure(logger ? logger->get() : nullptr, "server_destructor",
                         "transport_stop_inputs", error);
    }
    if (transport && !RunWithExceptionBoundary([this] { transport->Reset(); }, &error)) {
      LogBoundaryFailure(logger ? logger->get() : nullptr, "server_destructor", "transport_reset",
                         error);
    }
    loop.reset();
    viewer.reset();
    transport.reset();
    if (owns_ecal) {
      bool finalized = false;
      std::string finalize_error;
      const bool completed =
          RunWithExceptionBoundary([&] { finalized = eCAL::Finalize(); }, &finalize_error);
      if (!completed || !finalized) {
        LogBoundaryFailure(logger ? logger->get() : nullptr, "server_destructor", "eCAL::Finalize",
                           finalize_error.empty() ? "eCAL returned failure" : finalize_error);
      }
      owns_ecal = false;
    }
    if (logger) RunWithExceptionBoundary([this] { logger->Flush(); }, nullptr);
  }

  ServerConfig config;
  std::unique_ptr<ServerLogger> logger;
  std::unique_ptr<WarningLimiter> warning_limiter;
  std::unique_ptr<EcalServerTransport> transport;
  std::unique_ptr<SimViewer> viewer;
  std::unique_ptr<SimulationLoop> loop;
  std::mutex mutex;
  int exit_code = 1;
  bool owns_ecal = false;
  bool started = false;
  bool run_completed = false;
};

ServerApplication::ServerApplication(ServerConfig config)
    : impl_(std::make_unique<detail::ServerApplicationImpl>(std::move(config))) {}

ServerApplication::~ServerApplication() = default;

bool ServerApplication::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

int ServerApplication::Run() {
  return impl_->Run();
}

void ServerApplication::RequestStop(std::string reason) noexcept {
  impl_->RequestStop(std::move(reason));
}

bool ServerApplication::RequestScreenshot() {
  return impl_->RequestScreenshot();
}

const ServerConfig& ServerApplication::config() const noexcept {
  return impl_->config;
}

ServerApplicationTestHarness::ServerApplicationTestHarness(ServerConfig config)
    : impl_(std::make_unique<detail::ServerApplicationImpl>(std::move(config))) {}

ServerApplicationTestHarness::~ServerApplicationTestHarness() = default;

bool ServerApplicationTestHarness::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

int ServerApplicationTestHarness::Run() {
  return impl_->Run();
}

bool ServerApplicationTestHarness::RequestViewerSyncFailure() noexcept {
  return impl_->RequestViewerSyncFailureForTesting();
}

}  // namespace mujoco_sim::server
