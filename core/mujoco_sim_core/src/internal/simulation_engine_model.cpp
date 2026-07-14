#include "simulation_core_impl.hpp"

#include <string>
#include <utility>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v2::FAILED_PRECONDITION;
using mujoco_sim::v2::NOT_FOUND;
using mujoco_sim::v2::OK;
using mujoco_sim::v2::RUNNING;
using mujoco_sim::v2::VERSION_MISMATCH;

void CopyDescriptor(const mujoco_sim::v2::ResourceDescriptor& from,
                    mujoco_sim::v2::ResourceDescriptor* to) {
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

const std::vector<mujoco_sim::v2::ResourceDescriptor>&
SimulationCore::Impl::observations() const {
  return resource_catalog_->observations();
}

const std::vector<mujoco_sim::v2::ResourceDescriptor>&
SimulationCore::Impl::commands() const {
  return resource_catalog_->commands();
}

CoreResources SimulationCore::Impl::resources() const {
  return {observations(), commands()};
}

mujoco_sim::v2::GetAvailableResourcesResponse
SimulationCore::Impl::GetAvailableResources(
    const mujoco_sim::v2::GetAvailableResourcesRequest& request) const {
  mujoco_sim::v2::GetAvailableResourcesResponse response;
  if (!IsModelLoaded()) {
    response.set_code(FAILED_PRECONDITION);
    response.set_message("model is not loaded");
    return response;
  }
  if (!request.server_name().empty() &&
      request.server_name() != options_.server_name) {
    response.set_code(NOT_FOUND);
    response.set_message("server name mismatch");
    return response;
  }
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor ||
      request.protocol_minor() != mujoco_sim::protocol::kProtocolMinor) {
    response.set_code(VERSION_MISMATCH);
    response.set_message("protocol version mismatch; expected 2.0");
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
