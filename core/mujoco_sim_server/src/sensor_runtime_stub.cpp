#include "sensor_runtime.hpp"

#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::server {

struct SensorRuntime::Impl {};

SensorRuntime::SensorRuntime() : impl_(std::make_unique<Impl>()) {}
SensorRuntime::SensorRuntime(SensorRuntimeTestHooks hooks)
    : hooks_(std::move(hooks)), impl_(std::make_unique<Impl>()) {}
SensorRuntime::~SensorRuntime() = default;

bool SensorRuntime::Initialize(core::SimulationCore*, const ServerConfig& config,
                               std::string* error_message) {
  if (config.sensor_streams.empty()) return true;
  if (error_message != nullptr) {
    *error_message = "this build does not include sensor streams";
  }
  return false;
}
void SensorRuntime::OnStepCompleted(core::SimulationCore*) {}
void SensorRuntime::CompleteBlockingEnqueues(const std::function<bool()>&) {}
void SensorRuntime::RequestStop() {}
void SensorRuntime::Stop() {}
mujoco_sim::v2::GetSensorStreamsResponse SensorRuntime::GetCatalog(
    const mujoco_sim::v2::GetSensorStreamsRequest&) const {
  mujoco_sim::v2::GetSensorStreamsResponse response;
  response.set_code(mujoco_sim::v2::OK);
  return response;
}
void SensorRuntime::AppendStatus(mujoco_sim::v2::ServerStatus*) const {}

}  // namespace mujoco_sim::server
