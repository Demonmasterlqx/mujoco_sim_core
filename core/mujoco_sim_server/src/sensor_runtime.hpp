#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>
#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::server {

struct SensorRuntimePublisher {
  std::shared_ptr<void> lifetime;
  std::function<std::size_t()> subscriber_count;
  std::function<bool(const std::string&)> send;
};

struct SensorRuntimeTestHooks {
  std::function<SensorRuntimePublisher(const std::string& topic)>
      make_publisher;
  std::function<void(const std::string& source_name)> before_capture;
  std::function<void(const std::string& source_name)>
      on_blocking_wait_started;
  std::function<void(mujoco_sim::v2::StreamFrameHeader*)>
      before_serialize;
  std::function<void(const std::string& source_name)> before_initialize;
};

class SensorRuntime {
 public:
  SensorRuntime();
  explicit SensorRuntime(SensorRuntimeTestHooks hooks);
  ~SensorRuntime();

  SensorRuntime(const SensorRuntime&) = delete;
  SensorRuntime& operator=(const SensorRuntime&) = delete;

  bool Initialize(mujoco_sim::core::SimulationCore* core,
                  const ServerConfig& config, std::string* error_message);
  void OnStepCompleted(mujoco_sim::core::SimulationCore* core);
  void CompleteBlockingEnqueues(
      const std::function<bool()>& should_stop = {});
  void RequestStop();
  void Stop();

  mujoco_sim::v2::GetSensorStreamsResponse GetCatalog(
      const mujoco_sim::v2::GetSensorStreamsRequest& request) const;
  void AppendStatus(mujoco_sim::v2::ServerStatus* status) const;

 private:
  struct Impl;
  SensorRuntimeTestHooks hooks_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
