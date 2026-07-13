#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>

namespace mujoco_sim::server {

class SimViewer;
class CallbackState;
class ServerLogger;
class WarningLimiter;
class SensorRuntime;
class RegistryEndpoint;
class ControlEndpoint;
class ObservationEndpoint;
class StatusEndpoint;
class SimulationScheduler;
struct RuntimeMetrics;

class ServerApplication {
 public:
  explicit ServerApplication(ServerConfig config);
  ~ServerApplication();

  ServerApplication(const ServerApplication&) = delete;
  ServerApplication& operator=(const ServerApplication&) = delete;

  bool Start(std::string* error_message);
  int Run();
  void RequestStop(std::string reason = "external_stop");

  const ServerConfig& config() const noexcept { return config_; }

 private:
  int RunLoop();
  int RunGraphical();
  bool RegisterServices(std::string* error_message);
  void RegisterSubscribersAndPublishers();
  void ShutdownTransport();
  void PublishTicks(
      const std::vector<mujoco_sim::core::IssuedTickHandle>& ticks,
                    bool retransmission);
  void PublishObservationFrames(
      const std::vector<mujoco_sim::v2::ObservationFrame>& frames);
  void PublishStatus();
  std::vector<mujoco_sim::core::IssuedTickHandle> OpenTickLocked();
  void MaybeLogSummary(std::chrono::steady_clock::time_point now, bool final);
  int FinishRun(int exit_code, const std::string& reason);
  void StopWithReason(const std::string& reason);
  bool ShouldLogWarning(const char* key, std::uint64_t* suppressed);
  void RecordMalformedMessage();
  void FinishStepPipeline(
      const mujoco_sim::core::StepCompletedEvent& event);
  void CompleteSensorFence();

  int HandleGetAvailableResources(const std::string& request,
                                  std::string* response);
  int HandleRegisterAdapter(const std::string& request,
                            std::string* response);
  int HandleActivateAdapter(const std::string& request,
                            std::string* response);
  int HandleUnregisterAdapter(const std::string& request,
                              std::string* response);
  int HandleGetSensorStreams(const std::string& request,
                             std::string* response);
  void HandleCommitBytes(const void* data, std::size_t size);
  void HandleAdapterHeartbeatBytes(const void* data, std::size_t size);

  ServerConfig config_;
  std::unique_ptr<ServerLogger> logger_;
  std::unique_ptr<WarningLimiter> warning_limiter_;
  std::unique_ptr<RuntimeMetrics> metrics_;
  std::shared_ptr<CallbackState> callback_state_;
  mujoco_sim::core::SimulationCore core_;
  std::unique_ptr<SensorRuntime> sensor_runtime_;
  std::unique_ptr<mujoco_sim::core::SimulationSnapshot> viewer_snapshot_;
  std::unique_ptr<SimViewer> viewer_;
  std::unique_ptr<RegistryEndpoint> registry_endpoint_;
  std::unique_ptr<ControlEndpoint> control_endpoint_;
  std::unique_ptr<ObservationEndpoint> observation_endpoint_;
  std::unique_ptr<StatusEndpoint> status_endpoint_;
  std::unique_ptr<SimulationScheduler> scheduler_;
  std::mutex simulation_mutex_;
  std::mutex mutex_;
  std::atomic<bool> stop_requested_{false};
  std::string stop_reason_ = "external_stop";
  bool status_dirty_ = true;
  mujoco_sim::v2::ServerState last_logged_state_ =
      mujoco_sim::v2::SERVER_STATE_UNSPECIFIED;
  bool owns_ecal_ = false;
  bool transport_shutdown_succeeded_ = false;
};

}  // namespace mujoco_sim::server
