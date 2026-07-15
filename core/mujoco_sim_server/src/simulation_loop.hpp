#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>

#include "ecal_server_transport.hpp"
#include "server_event_queue.hpp"
#include "server_runtime.hpp"

namespace spdlog {
class logger;
}

namespace mujoco_sim::server {

class SensorRuntime;
class SimViewer;
class WarningLimiter;

class SimulationLoop {
 public:
  SimulationLoop(ServerConfig config, std::shared_ptr<spdlog::logger> logger,
                 WarningLimiter* warning_limiter,
                 EcalServerTransport* transport, SimViewer* viewer);
  ~SimulationLoop();

  SimulationLoop(const SimulationLoop&) = delete;
  SimulationLoop& operator=(const SimulationLoop&) = delete;

  bool Initialize(std::string* error_message);
  EcalServerTransport::Handlers MakeHandlers();
  bool Start(std::string* error_message);
  void RequestStop(std::string reason);
  int Join();

 private:
  void Run(std::stop_token stop_token) noexcept;
  void HandleEvent(ServerEvent event);
  void Handle(GetAvailableResourcesEvent event);
  void Handle(RegisterAdapterEvent event);
  void Handle(ActivateAdapterEvent event);
  void Handle(UnregisterAdapterEvent event);
  void Handle(GetSensorStreamsEvent event);
  void Handle(CommitEvent event);
  void Handle(HeartbeatEvent event);
  void Handle(ControllerFaultEvent event);
  void ProcessTimers(std::chrono::steady_clock::time_point now);
  void ApplyEffects(mujoco_sim::core::EngineEffects effects);
  void ApplyCompletedStep(mujoco_sim::core::StepCompletedEvent event);
  std::vector<mujoco_sim::core::IssuedTickHandle> OpenTicks();
  void PublishTicks(
      const std::vector<mujoco_sim::core::IssuedTickHandle>& ticks,
      bool retransmission);
  void InitializeTickRetries(
      const std::vector<mujoco_sim::core::IssuedTickHandle>& ticks,
      std::chrono::steady_clock::time_point now);
  void PublishDueTickRetries(std::chrono::steady_clock::time_point now);
  void PublishObservations(
      const std::vector<mujoco_sim::core::ObservationFrame>& frames);
  void PublishStatus(bool lifecycle, bool terminal = false);
  bool TickSubscribersReady() const;
  void MaybeLogSummary(std::chrono::steady_clock::time_point now, bool final);
  void RecordMalformed(std::string_view operation, std::size_t bytes);
  void RecordQueueRejection(std::string_view operation);
  bool ShouldLogWarning(std::string_view key, std::uint64_t* suppressed);
  std::chrono::steady_clock::time_point NextWakeDeadline(
      std::chrono::steady_clock::time_point now) const;
  void Finish(int exit_code, std::string reason);

  template <typename Event, typename Request>
  typename Event::response_type SubmitRpc(Request request);

  ServerConfig config_;
  std::shared_ptr<spdlog::logger> logger_;
  WarningLimiter* warning_limiter_ = nullptr;
  EcalServerTransport* transport_ = nullptr;
  SimViewer* viewer_ = nullptr;
  mujoco_sim::core::SimulationCore core_;
  std::unique_ptr<SensorRuntime> sensor_runtime_;
  std::unique_ptr<mujoco_sim::core::SimulationSnapshot> viewer_snapshot_;
  RuntimeMetrics metrics_;
  ServerEventQueue queue_;
  std::jthread thread_;

  std::chrono::steady_clock::duration step_period_{};
  std::chrono::steady_clock::time_point next_tick_{};
  std::chrono::steady_clock::time_point next_status_{};
  std::chrono::steady_clock::time_point next_status_allowed_{};
  std::chrono::steady_clock::time_point next_viewer_sync_{};
  std::chrono::steady_clock::time_point next_subscriber_sample_{};
  mujoco_sim::core::ServerState last_logged_state_ =
      mujoco_sim::core::SERVER_STATE_UNSPECIFIED;
  bool status_dirty_ = true;
  struct TickRetryState {
    TickRetrySchedule schedule;
    std::uint64_t issue_generation = 0;
  };
  std::unordered_map<std::string, TickRetryState> tick_retries_;
  struct SessionInfo {
    std::string adapter_id;
    mujoco_sim::core::AdapterRole role =
        mujoco_sim::core::ADAPTER_ROLE_UNSPECIFIED;
    std::uint32_t commit_timeout_ms = 0;
  };
  std::unordered_map<std::uint64_t, SessionInfo> sessions_;
  std::unordered_set<std::string> active_controllers_;
  std::unordered_map<std::uint64_t, std::string> encoded_ticks_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> malformed_pending_{0};
  std::atomic<std::uint64_t> queue_rejections_{0};
  mutable std::mutex lifecycle_mutex_;
  std::string stop_reason_ = "external_stop";
  int exit_code_ = 1;
  bool initialized_ = false;
};

}  // namespace mujoco_sim::server
