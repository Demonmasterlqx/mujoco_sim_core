#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <stop_token>
#include <string>
#include <utility>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>
#include <mujoco_sim/v3/sync.pb.h>

namespace mujoco_sim::server {

struct SensorRuntimePublisher {
  std::shared_ptr<void> lifetime;
  std::function<std::size_t()> subscriber_count;
  std::function<bool(const std::string&)> send;
};

struct SensorRuntimeTestHooks {
  SensorRuntimeTestHooks(
      std::function<SensorRuntimePublisher(const std::string&)> publisher = {},
      std::function<void(const std::string&)> capture = {},
      std::function<void(const std::string&)> blocking_wait = {},
      std::function<void(mujoco_sim::v3::StreamFrameHeader*)> serialize = {},
      std::function<void(const std::string&)> initialize = {})
      : make_publisher(std::move(publisher)),
        before_capture(std::move(capture)),
        on_blocking_wait_started(std::move(blocking_wait)),
        before_serialize(std::move(serialize)),
        before_initialize(std::move(initialize)) {}

  std::function<SensorRuntimePublisher(const std::string& topic)>
      make_publisher;
  std::function<void(const std::string& source_name)> before_capture;
  std::function<void(const std::string& source_name)>
      on_blocking_wait_started;
  std::function<void(mujoco_sim::v3::StreamFrameHeader*)>
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

  bool Initialize(mujoco_sim::core::SimulationCore& core,
                  const ServerConfig& config, std::string* error_message);
  void OnStepCompleted(mujoco_sim::core::SimulationCore& core);
  void CompleteBlockingEnqueues(std::stop_token stop_token = {});
  void RequestStop();
  void Stop();

  mujoco_sim::v3::GetSensorStreamsResponse GetCatalog(
      const mujoco_sim::v3::GetSensorStreamsRequest& request) const;
  void AppendStatus(mujoco_sim::v3::ServerStatus* status) const;

 private:
  struct Impl;
  SensorRuntimeTestHooks hooks_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
