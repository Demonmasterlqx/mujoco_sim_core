#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>

#include "controller_command_inbox.hpp"
#include "ecal_server_transport.hpp"
#include "server_event_queue.hpp"

namespace spdlog {
class logger;
}

namespace mujoco_sim::server {

class SensorRuntime;
class SimViewer;
class WarningLimiter;

class SimulationLoop {
 public:
  SimulationLoop(ServerConfig, std::shared_ptr<spdlog::logger>, WarningLimiter*,
                 EcalServerTransport*, SimViewer*);
  ~SimulationLoop();

  SimulationLoop(const SimulationLoop&) = delete;
  SimulationLoop& operator=(const SimulationLoop&) = delete;

  bool Initialize(std::string*);
  EcalServerTransport::Handlers MakeHandlers();
  bool Start(std::string*);
  void RequestStop(std::string);
  int Join();

 private:
  void Run(std::stop_token) noexcept;
  void HandleEvent(ServerEvent);
  void Handle(GetAvailableResourcesEvent);
  void Handle(RegisterAdapterEvent);
  void Handle(ActivateAdapterEvent);
  void Handle(UnregisterAdapterEvent);
  void Handle(GetSensorStreamsEvent);
  void Handle(HeartbeatEvent);
  void DrainControllerInputs(std::chrono::steady_clock::time_point);
  void ApplyEffects(mujoco_sim::core::EngineEffects);
  void ApplyCompletedStep(mujoco_sim::core::StepCompletedEvent);
  void PublishControllerStates(const std::vector<mujoco_sim::core::ControllerStateFrame>&);
  void PublishObservations(const std::vector<mujoco_sim::core::ObservationFrame>&);
  void PublishStatus(bool, bool = false);
  void PublishSummary(std::chrono::steady_clock::time_point, const mujoco_sim::core::CoreState&);
  void ProcessTimers(std::chrono::steady_clock::time_point);
  void Finish(int, std::string);

  template <class Event, class Request>
  typename Event::response_type SubmitRpc(Request);

  struct SessionInfo {
    std::string adapter_id;
    mujoco_sim::core::AdapterRole role = mujoco_sim::core::ADAPTER_ROLE_UNSPECIFIED;
  };

  ServerConfig config_;
  std::shared_ptr<spdlog::logger> logger_;
  WarningLimiter* warning_limiter_ = nullptr;
  EcalServerTransport* transport_ = nullptr;
  SimViewer* viewer_ = nullptr;
  mujoco_sim::core::SimulationCore core_;
  std::unique_ptr<SensorRuntime> sensor_runtime_;
  std::unique_ptr<mujoco_sim::core::SimulationSnapshot> viewer_snapshot_;
  ServerEventQueue queue_;
  ControllerCommandInbox command_inbox_;
  std::jthread thread_;
  std::stop_token run_stop_token_;
  std::unordered_map<std::uint64_t, SessionInfo> sessions_;
  std::chrono::steady_clock::duration step_period_{};
  std::chrono::steady_clock::time_point next_step_{};
  std::chrono::steady_clock::time_point next_status_{};
  std::chrono::steady_clock::time_point next_status_allowed_{};
  std::chrono::steady_clock::time_point next_viewer_sync_{};
  std::chrono::steady_clock::time_point next_transport_sample_{};
  std::chrono::steady_clock::time_point next_summary_{};
  std::chrono::steady_clock::time_point summary_wall_anchor_{};
  double summary_simulation_anchor_ = 0.0;
  std::uint64_t summary_tick_anchor_ = 0;
  std::size_t transport_endpoint_count_ = 0;
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> malformed_pending_{0};
  mutable std::mutex lifecycle_mutex_;
  std::string stop_reason_ = "external_stop";
  int exit_code_ = 1;
  bool initialized_ = false;
  bool status_dirty_ = true;
};

}  // namespace mujoco_sim::server
