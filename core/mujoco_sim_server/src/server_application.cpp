#include <mujoco_sim/server/server_application.hpp>

#include <ecal/ecal.h>
#include <spdlog/logger.h>

#include "ecal_server_transport.hpp"
#include "server_logging.hpp"
#include "server_runtime.hpp"
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

}  // namespace

class ServerApplication::Impl {
 public:
  explicit Impl(ServerConfig server_config)
      : config(std::move(server_config)),
        logger(std::make_unique<ServerLogger>()),
        warning_limiter(std::make_unique<WarningLimiter>(kWarningPeriod)) {}

  ~Impl() { Shutdown(); }

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
    log->info(
        "event=server_starting server_name={} model_path={} initial_keyframe={} "
        "mode={} max_ticks={} tick_timeout_ms={} activation_timeout_ms={} "
        "allow_timed_out_adapter_replacement={} status_period_ms={} "
        "log_level={} log_file={} log_summary_period_ms={}",
        LogValue(config.server_name), LogValue(config.model_path),
        LogValue(config.initial_keyframe), config.headless ? "headless" : "gui",
        config.max_ticks, config.tick_timeout_ms, config.activation_timeout_ms,
        config.allow_timed_out_adapter_replacement, config.status_period_ms,
        config.log_level, LogValue(config.log_file),
        config.log_summary_period_ms);

    std::string boundary_error;
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
      *error_message = boundary_error.empty()
                           ? "failed to initialize eCAL"
                           : "eCAL initialization threw: " + boundary_error;
      return false;
    }

    if (!config.headless) {
      viewer = std::make_unique<SimViewer>();
      if (!viewer->Initialize(error_message)) return false;
      log->info("event=viewer_initialized mode=gui");
    } else {
      log->info("event=viewer_initialized mode=headless");
    }

    transport =
        std::make_unique<EcalServerTransport>(config.server_name, logger->get());
    loop = std::make_unique<SimulationLoop>(
        config, logger->get(), warning_limiter.get(), transport.get(),
        viewer.get());
    if (!loop->Initialize(error_message)) return false;
    if (!transport->Start(loop->MakeHandlers(), error_message)) return false;
    if (!loop->Start(error_message)) return false;

    started = true;
    log->info("event=server_started server_name={}",
              LogValue(config.server_name));
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
      std::string error;
      render_succeeded = RunWithExceptionBoundary(
          [this] { viewer->RenderLoop(); }, &error);
      if (!render_succeeded) {
        LogBoundaryFailure(logger->get(), "gui_render_thread", "RenderLoop",
                           error);
      }
      RequestStop(render_succeeded ? "viewer_closed"
                                   : "viewer_render_exception");
    }

    int result = loop->Join();
    if (!render_succeeded) result = 2;
    transport->StopInputs();
    transport->Reset();
    logger->Flush();
    {
      std::lock_guard lock(mutex);
      exit_code = result;
      run_completed = true;
    }
    return result;
  }

  void RequestStop(std::string reason) {
    if (loop) loop->RequestStop(std::move(reason));
  }

  void Shutdown() noexcept {
    std::string error;
    RunWithExceptionBoundary(
        [this] {
          if (loop) {
            loop->RequestStop("server_destroyed");
            loop->Join();
          }
        },
        &error);
    if (!error.empty()) {
      LogBoundaryFailure(logger ? logger->get() : nullptr, "server_destructor",
                         "simulation_loop", error);
    }
    RunWithExceptionBoundary(
        [this] {
          if (transport) {
            transport->StopInputs();
            transport->Reset();
          }
        },
        nullptr);
    loop.reset();
    viewer.reset();
    transport.reset();
    if (owns_ecal) {
      RunWithExceptionBoundary([] { eCAL::Finalize(); }, nullptr);
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
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ServerApplication::~ServerApplication() = default;

bool ServerApplication::Start(std::string* error_message) {
  return impl_->Start(error_message);
}

int ServerApplication::Run() { return impl_->Run(); }

void ServerApplication::RequestStop(std::string reason) {
  impl_->RequestStop(std::move(reason));
}

const ServerConfig& ServerApplication::config() const noexcept {
  return impl_->config;
}

}  // namespace mujoco_sim::server
