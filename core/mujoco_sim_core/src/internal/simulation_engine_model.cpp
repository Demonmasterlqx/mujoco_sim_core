#include "simulation_core_impl.hpp"

#include <string>
#include <utility>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::core::FAILED_PRECONDITION;
using mujoco_sim::core::NOT_FOUND;
using mujoco_sim::core::OK;
using mujoco_sim::core::RUNNING;
using mujoco_sim::core::VERSION_MISMATCH;

void CopyDescriptor(const mujoco_sim::core::ResourceDescriptor& from,
                    mujoco_sim::core::ResourceDescriptor* to) {
  *to = from;
}

}  // namespace

SimulationCore::Impl::Impl(CoreOptions options)
    : options_(std::move(options)),
      model_state_(std::make_unique<ModelRuntime>()),
      resource_catalog_(std::make_unique<ResourceCatalog>()),
      snapshot_manager_(std::make_unique<SnapshotManager>()),
      observation_scheduler_(std::make_unique<ObservationScheduler>()),
      status_projection_(std::make_unique<StatusProjection>()),
      adapter_registry_(std::make_unique<AdapterRegistry>()),
      barrier_coordinator_(std::make_unique<BarrierCoordinator>()),
      tick_store_(std::make_unique<TickStore>()) {}

bool SimulationCore::Impl::DoLoadModel(const std::string& model_path,
                               const ModelLoadOptions& options,
                               std::string* error_message) {
  auto prepared = ModelRuntime::Prepare(model_path, options, error_message);
  if (!prepared) return false;
  ResourceCatalog prepared_catalog =
      ResourceCatalog::Build(*prepared->model);

  model_state_->Commit(std::move(*prepared));
  *resource_catalog_ = std::move(prepared_catalog);
  snapshot_manager_->Reset();

  adapter_registry_->Clear();
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  epoch_++;
  current_tick_id_ = 0;
  server_state_ = RUNNING;
  tick_open_ = false;
  status_message_.clear();
  return true;
}

bool SimulationCore::Impl::IsModelLoaded() const { return model_state_->loaded(); }

std::unique_ptr<SimulationSnapshot> SimulationCore::Impl::CreateSnapshot(
    std::string* error_message) const {
  return snapshot_manager_->Create(*model_state_, error_message);
}

std::unique_ptr<SimulationSnapshot> SimulationCore::Impl::CloneSnapshot(
    const SimulationSnapshot& source, std::string* error_message) const {
  return snapshot_manager_->Clone(*model_state_, source, error_message);
}

bool SimulationCore::Impl::UpdateSnapshot(SimulationSnapshot* snapshot,
                                    std::string* error_message) const {
  return snapshot_manager_->Update(*model_state_, snapshot, error_message);
}

const std::vector<mujoco_sim::core::ResourceDescriptor>&
SimulationCore::Impl::observations() const {
  return resource_catalog_->observations();
}

const std::vector<mujoco_sim::core::ResourceDescriptor>&
SimulationCore::Impl::commands() const {
  return resource_catalog_->commands();
}

CoreResources SimulationCore::Impl::resources() const {
  return {observations(), commands()};
}

mujoco_sim::core::GetAvailableResourcesResponse
SimulationCore::Impl::GetAvailableResources(
    const mujoco_sim::core::GetAvailableResourcesRequest&) const {
  mujoco_sim::core::GetAvailableResourcesResponse response;
  if (!IsModelLoaded()) {
    response.set_code(FAILED_PRECONDITION);
    response.set_message("model is not loaded");
    return response;
  }
  response.set_code(OK);
  response.set_epoch(epoch_);
  for (const auto& observation : resource_catalog_->observations()) {
    CopyDescriptor(observation, response.add_observations());
  }
  for (const auto& command : resource_catalog_->commands()) {
    CopyDescriptor(command, response.add_commands());
  }
  return response;
}


}  // namespace mujoco_sim::core
