#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stop_token>
#include <string>
#include <utility>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>
#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::server {

struct SensorRuntimePublisher {
  std::shared_ptr<void> lifetime;
  std::function<std::size_t()> subscriber_count;
  std::function<bool(const std::string&)> send;
};

struct SensorRuntimeTestHooks {
  SensorRuntimeTestHooks(std::function<SensorRuntimePublisher(const std::string&)> publisher = {},
                         std::function<void(const std::string&)> capture = {},
                         std::function<void(const std::string&)> blocking_wait = {},
                         std::function<void(mujoco_sim::v4::StreamFrameHeader*)> serialize = {},
                         std::function<void(const std::string&)> initialize = {})
      : make_publisher(std::move(publisher)),
        before_capture(std::move(capture)),
        on_blocking_wait_started(std::move(blocking_wait)),
        before_serialize(std::move(serialize)),
        before_initialize(std::move(initialize)) {}

  std::function<SensorRuntimePublisher(const std::string& topic)> make_publisher;
  std::function<void(const std::string& source_name)> before_capture;
  std::function<void(const std::string& source_name)> on_blocking_wait_started;
  std::function<void(mujoco_sim::v4::StreamFrameHeader*)> before_serialize;
  std::function<void(const std::string& source_name)> before_initialize;
};

class SensorRuntime {
 public:
  SensorRuntime();
  explicit SensorRuntime(SensorRuntimeTestHooks hooks);
  ~SensorRuntime();

  SensorRuntime(const SensorRuntime&) = delete;
  SensorRuntime& operator=(const SensorRuntime&) = delete;

  bool Initialize(mujoco_sim::core::SimulationCore& core, const ServerConfig& config,
                  std::string* error_message);
  void OnStepCompleted(mujoco_sim::core::SimulationCore& core);
  /**
   * @brief Advances wall-clock Sensor management without requiring a physics step.
   *
   * The SimulationLoop calls this from its ordinary timer path in every Server
   * state. Due subscriber samples are requested asynchronously from each
   * worker; this function performs no capture, publication, or wait.
   *
   * 在不要求物理步进的情况下推进基于墙钟的 Sensor 管理。
   *
   * SimulationLoop 在所有 Server 状态下从普通定时器路径调用本函数。到期的 subscriber
   * 采样会异步请求给每个 worker；本函数不执行采集、发布或等待。
   *
   * @param now Explicit steady-clock timer boundary. 显式 steady clock 定时器边界。
   * @par Thread safety / 线程安全
   * Call only from SimulationLoop while workers may run concurrently.
   * 仅可由 SimulationLoop 调用；Sensor worker 可以并发运行。
   */
  void ProcessTimers(std::chrono::steady_clock::time_point now);
  /**
   * @brief Invalidates queued pre-Reset captures and rebuilds simulation-time cadence.
   *
   * The SimulationLoop owner thread calls this immediately after Core Reset.
   * It never waits for active capture or publication; workers use a generation
   * barrier before starting further external work.
   *
   * 使排队中的 Reset 前采集失效，并重建仿真时间节拍。
   *
   * SimulationLoop 所有者线程在 Core Reset 后立即调用。本函数不等待正在进行的采集或
   * 发布；worker 在开始后续外部工作前使用 generation 屏障。
   *
   * @param state Owned Core state after Reset. Reset 后的自有 Core 状态快照。
   * @par Thread safety / 线程安全
   * Call only from SimulationLoop while workers may run concurrently.
   * 仅可由 SimulationLoop 线程调用；Sensor worker 可以并发运行。
   */
  void OnSimulationReset(const mujoco_sim::core::CoreState& state);
  void CompleteBlockingEnqueues(std::stop_token stop_token = {});
  void RequestStop();
  void Stop();

  mujoco_sim::v4::GetSensorStreamsResponse GetCatalog(
      const mujoco_sim::v4::GetSensorStreamsRequest& request) const;
  void AppendStatus(mujoco_sim::v4::ServerStatus* status) const;

 private:
  struct Impl;
  SensorRuntimeTestHooks hooks_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
