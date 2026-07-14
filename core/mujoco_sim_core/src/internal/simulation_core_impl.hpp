#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>

#include "adapter_registry.hpp"
#include "barrier_coordinator.hpp"
#include "model_runtime.hpp"
#include "observation_scheduler.hpp"
#include "resource_catalog.hpp"
#include "snapshot_manager.hpp"
#include "status_projection.hpp"
#include "tick_store.hpp"

namespace mujoco_sim::core {

using Controller = internal::ControllerRecord;
using Observer = internal::ObserverRecord;
using internal::AdapterRegistry;
using internal::BarrierCoordinator;
using internal::ModelRuntime;
using internal::ObservationScheduler;
using internal::ResourceCatalog;
using detail::SnapshotManager;
using internal::StatusProjection;
using internal::TickStore;

struct AdapterInfo {
  std::string adapter_id;
  mujoco_sim::v2::AdapterRole role;
  std::uint64_t session_id;
  mujoco_sim::v2::ControllerState state;
};

struct SimulationCore::Impl {
  explicit Impl(CoreOptions options);

  EngineResult<ModelLoadReceipt> LoadModelWithEffects(
      const std::string& model_path, const ModelLoadOptions& options);
  EngineResult<mujoco_sim::v2::RegisterAdapterResponse>
  RegisterAdapterWithEffects(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  EngineResult<mujoco_sim::v2::ActivateAdapterResponse>
  ActivateAdapterWithEffects(
      const mujoco_sim::v2::ActivateAdapterRequest& request);
  EngineResult<mujoco_sim::v2::UnregisterAdapterResponse>
  UnregisterAdapterWithEffects(
      const mujoco_sim::v2::UnregisterAdapterRequest& request);
  EngineResult<std::vector<IssuedTickHandle>> OpenIssuedTicksWithEffects();
  EngineEffects CompleteEmptyBarrierWithEffects();
  EngineResult<CommitReceipt> SubmitCommitWithEffects(
      const mujoco_sim::v2::AdapterCommit& commit);
  EngineEffects AcceptHeartbeatWithEffects(
      const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
      std::chrono::steady_clock::time_point now);
  EngineEffects CheckDeadlinesWithEffects(
      std::chrono::steady_clock::time_point now);
  EngineEffects MarkFailedWithEffects(const std::string& message);
  EngineEffects MarkStoppedWithEffects(const std::string& message);

  bool DoLoadModel(const std::string& model_path,
                   const ModelLoadOptions& options,
                   std::string* error_message);
  bool IsModelLoaded() const;
  std::unique_ptr<SimulationSnapshot> CreateSnapshot(
      std::string* error_message) const;
  std::unique_ptr<SimulationSnapshot> CloneSnapshot(
      const SimulationSnapshot& source, std::string* error_message) const;
  bool UpdateSnapshot(SimulationSnapshot* snapshot,
                      std::string* error_message) const;
  const std::vector<mujoco_sim::v2::ResourceDescriptor>& observations() const;
  const std::vector<mujoco_sim::v2::ResourceDescriptor>& commands() const;
  CoreResources resources() const;
  mujoco_sim::v2::GetAvailableResourcesResponse GetAvailableResources(
      const mujoco_sim::v2::GetAvailableResourcesRequest& request) const;
  mujoco_sim::v2::RegisterAdapterResponse DoRegisterAdapter(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  mujoco_sim::v2::ActivateAdapterResponse DoActivateAdapter(
      const mujoco_sim::v2::ActivateAdapterRequest& request);
  EngineResult<mujoco_sim::v2::UnregisterAdapterResponse> DoUnregisterAdapter(
      const mujoco_sim::v2::UnregisterAdapterRequest& request);
  std::vector<IssuedTickHandle> DoOpenIssuedTicks();
  bool OpenNewBarrier();
  bool IsIssuedTickCurrent(const IssuedTickIdentity& identity) const;
  std::optional<StepCompletedEvent> DoCompleteEmptyBarrier();
  EngineResult<CommitReceipt> DoSubmitCommit(
      const mujoco_sim::v2::AdapterCommit& commit);
  void DoAcceptHeartbeat(const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
                       std::chrono::steady_clock::time_point now);
  void AcceptControllerHeartbeat(
      const mujoco_sim::v2::AdapterHeartbeat& heartbeat);
  void AcceptObserverHeartbeat(
      const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
      std::chrono::steady_clock::time_point now);
  EngineEffects DoCheckDeadlines(
      std::chrono::steady_clock::time_point now);
  void ExpireProvisionalControllers(
      std::chrono::steady_clock::time_point now,
      std::vector<AdapterChange>* changes);
  void UpdateObserverDeadlines(
      std::chrono::steady_clock::time_point now,
      std::vector<AdapterChange>* changes);
  std::optional<StepCompletedEvent> UpdateBarrierDeadlines(
      std::chrono::steady_clock::time_point now,
      std::vector<AdapterChange>* changes);
  void EnterFatalError(const std::string& message);
  void MarkStopped(const std::string& message);
  mujoco_sim::v2::ServerStatus MakeStatus() const;
  std::uint64_t epoch() const;
  std::uint64_t current_tick_id() const;
  double simulation_time() const;
  double timestep() const;
  mujoco_sim::v2::ServerState server_state() const;
  bool tick_open() const;
  CoreState state() const;
  std::optional<AdapterInfo> FindAdapterInfo(
      const std::string& adapter_id,
      std::optional<mujoco_sim::v2::AdapterRole> role = std::nullopt) const;
  std::vector<AdapterInfo> AdapterInfos() const;
  std::vector<AdapterInfo> RecoveryPendingControllers() const;
  std::uint64_t ControllerHeartbeatTick(const std::string& adapter_id) const;
  std::chrono::steady_clock::time_point ObserverHeartbeatTime(
      const std::string& adapter_id) const;

  mujoco_sim::v2::RegisterAdapterResponse RegisterControlAdapter(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  mujoco_sim::v2::RegisterAdapterResponse RegisterObserverAdapter(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  std::optional<mujoco_sim::v2::RegisterAdapterResponse>
  ValidateRegistrationRequest(
      const mujoco_sim::v2::RegisterAdapterRequest& request) const;
  std::optional<mujoco_sim::v2::RegisterAdapterResponse>
  ValidateControllerRequest(
      const mujoco_sim::v2::RegisterAdapterRequest& request,
      Controller** existing, bool* replacing) const;
  std::optional<mujoco_sim::v2::RegisterAdapterResponse>
  PopulateControllerLayout(
      const mujoco_sim::v2::RegisterAdapterRequest& request, bool replacing,
      Controller* controller) const;
  std::optional<mujoco_sim::v2::RegisterAdapterResponse>
  ValidateObserverRequest(
      const mujoco_sim::v2::RegisterAdapterRequest& request,
      Observer** existing, bool* replacing) const;
  mujoco_sim::v2::ActivateAdapterResponse ActivateControlAdapter(
      const mujoco_sim::v2::ActivateAdapterRequest& request);
  EngineResult<mujoco_sim::v2::UnregisterAdapterResponse>
  UnregisterControlAdapter(
      const mujoco_sim::v2::UnregisterAdapterRequest& request);
  mujoco_sim::v2::Tick BuildTickFor(const Controller& controller) const;
  mujoco_sim::v2::ObservationFrame BuildObservationFrame(
      Observer* observer) const;
  std::vector<mujoco_sim::v2::ObservationFrame>
  BuildObservationFramesForCompletedStep();
  std::optional<StepCompletedEvent> CompleteBarrier(
      StepCompletionSource source);
  mujoco_sim::v2::Header MakeHeader(std::uint64_t session_id) const;
  void EnterErrorPaused(const std::string& message);
  bool ValidateCommands(const Controller& controller,
                        const mujoco_sim::v2::AdapterCommit& commit,
                        std::string* message) const;
  bool ValidateCommitEnvelope(const Controller& controller,
                              const mujoco_sim::v2::AdapterCommit& commit,
                              EngineResult<CommitReceipt>* result) const;
  EngineResult<CommitReceipt> HandleRecoveryCommit(
      Controller* controller, const mujoco_sim::v2::AdapterCommit& commit);
  EngineResult<CommitReceipt> HandleActiveCommit(
      Controller* controller, const mujoco_sim::v2::AdapterCommit& commit);
  EngineResult<CommitReceipt> HandleStaleCommit(
      Controller* controller, const mujoco_sim::v2::AdapterCommit& commit);
  std::optional<CommitReceipt> ValidateCurrentCommitContext(
      const Controller& controller,
      const mujoco_sim::v2::AdapterCommit& commit) const;
  EngineResult<CommitReceipt> HandlePendingCommit(
      Controller* controller, const mujoco_sim::v2::AdapterCommit& commit,
      const mujoco_sim::v2::AdapterCommit& pending);
  EngineResult<CommitReceipt> RejectActiveCommit(
      Controller* controller, const std::string& message);
  EngineResult<CommitReceipt> AcceptActiveCommit(
      Controller* controller, const mujoco_sim::v2::AdapterCommit& commit);
  void ZeroControllerCommands(const Controller& controller);
  std::optional<StepCompletedEvent> EnterControllerError(
      Controller* controller, const std::string& message);
  void RecordCommitChange(EngineResult<CommitReceipt>* result,
                          AdapterChangeKind kind,
                          const Controller& controller,
                          mujoco_sim::v2::ControllerState old_state,
                          mujoco_sim::v2::ControllerState new_state) const;
  void ReleaseControllerResources(const Controller& controller);
  bool ValidateCommonHeader(const mujoco_sim::v2::Header& header,
                            const Controller& controller,
                            std::string* message) const;
  void InvalidateIssuedTick(const std::string& adapter_id);
  void InvalidateAllIssuedTicks();

  CoreOptions options_;
  std::unique_ptr<ModelRuntime> model_state_;
  std::unique_ptr<ResourceCatalog> resource_catalog_;
  std::unique_ptr<SnapshotManager> snapshot_manager_;
  std::unique_ptr<ObservationScheduler> observation_scheduler_;
  std::unique_ptr<StatusProjection> status_projection_;
  std::unique_ptr<AdapterRegistry> adapter_registry_;
  std::unique_ptr<BarrierCoordinator> barrier_coordinator_;
  std::unique_ptr<TickStore> tick_store_;
  std::uint64_t epoch_ = 1;
  std::uint64_t current_tick_id_ = 0;
  mujoco_sim::v2::ServerState server_state_ = mujoco_sim::v2::IDLE;
  bool tick_open_ = false;
  std::chrono::steady_clock::time_point tick_started_at_{};
  std::string status_message_;
};

}  // namespace mujoco_sim::core
