#include <mujoco_sim/core/simulation_core.hpp>

#include <memory>
#include <utility>

#include "internal/simulation_core_impl.hpp"

namespace mujoco_sim::core {

SimulationCore::SimulationCore(CoreOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

SimulationCore::~SimulationCore() = default;
SimulationCore::SimulationCore(SimulationCore&&) noexcept = default;
SimulationCore& SimulationCore::operator=(SimulationCore&&) noexcept = default;

EngineResult<ModelLoadReceipt> SimulationCore::LoadModel(
    const std::string& model_path, const ModelLoadOptions& options) {
  return impl_->LoadModelWithEffects(model_path, options);
}

bool SimulationCore::IsModelLoaded() const { return impl_->IsModelLoaded(); }

std::unique_ptr<SimulationSnapshot> SimulationCore::CreateSnapshot(
    std::string* error_message) const {
  return impl_->CreateSnapshot(error_message);
}

std::unique_ptr<SimulationSnapshot> SimulationCore::CloneSnapshot(
    const SimulationSnapshot& source, std::string* error_message) const {
  return impl_->CloneSnapshot(source, error_message);
}

bool SimulationCore::UpdateSnapshot(SimulationSnapshot& snapshot,
                                    std::string* error_message) const {
  return impl_->UpdateSnapshot(&snapshot, error_message);
}

CoreResources SimulationCore::resources() const { return impl_->resources(); }

mujoco_sim::v2::GetAvailableResourcesResponse
SimulationCore::GetAvailableResources(
    const mujoco_sim::v2::GetAvailableResourcesRequest& request) const {
  return impl_->GetAvailableResources(request);
}

EngineResult<mujoco_sim::v2::RegisterAdapterResponse>
SimulationCore::RegisterAdapter(
    const mujoco_sim::v2::RegisterAdapterRequest& request) {
  return impl_->RegisterAdapterWithEffects(request);
}

EngineResult<mujoco_sim::v2::ActivateAdapterResponse>
SimulationCore::ActivateAdapter(
    const mujoco_sim::v2::ActivateAdapterRequest& request) {
  return impl_->ActivateAdapterWithEffects(request);
}

EngineResult<mujoco_sim::v2::UnregisterAdapterResponse>
SimulationCore::UnregisterAdapter(
    const mujoco_sim::v2::UnregisterAdapterRequest& request) {
  return impl_->UnregisterAdapterWithEffects(request);
}

EngineResult<std::vector<IssuedTickHandle>>
SimulationCore::OpenIssuedTicks() {
  return impl_->OpenIssuedTicksWithEffects();
}

bool SimulationCore::IsIssuedTickCurrent(
    const IssuedTickIdentity& identity) const {
  return impl_->IsIssuedTickCurrent(identity);
}

EngineEffects SimulationCore::CompleteEmptyBarrier() {
  return impl_->CompleteEmptyBarrierWithEffects();
}

EngineResult<CommitReceipt> SimulationCore::SubmitCommit(
    const mujoco_sim::v2::AdapterCommit& commit) {
  return impl_->SubmitCommitWithEffects(commit);
}

EngineEffects SimulationCore::AcceptHeartbeat(
    const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  return impl_->AcceptHeartbeatWithEffects(heartbeat, now);
}

EngineEffects SimulationCore::CheckDeadlines(
    std::chrono::steady_clock::time_point now) {
  return impl_->CheckDeadlinesWithEffects(now);
}

EngineEffects SimulationCore::MarkFailed(const std::string& message) {
  return impl_->MarkFailedWithEffects(message);
}

EngineEffects SimulationCore::MarkStopped(const std::string& message) {
  return impl_->MarkStoppedWithEffects(message);
}

mujoco_sim::v2::ServerStatus SimulationCore::MakeStatus() const {
  return impl_->MakeStatus();
}

CoreState SimulationCore::state() const { return impl_->state(); }

}  // namespace mujoco_sim::core
