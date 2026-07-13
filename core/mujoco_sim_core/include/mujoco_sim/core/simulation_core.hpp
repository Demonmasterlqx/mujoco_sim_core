#pragma once

#include <chrono>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/v2/sync.pb.h>

struct mjData_;
struct mjModel_;

namespace mujoco_sim::core {

class SimulationSnapshot;

namespace detail {
struct Controller;
struct Observer;
class AdapterRegistry;
class BarrierCoordinator;
class TickStore;
class ModelState;
class ResourceCatalog;
class SnapshotManager;
class ObservationScheduler;
class StatusBuilder;
}  // namespace detail

struct CoreOptions {
  std::string server_name = "default";
  std::uint32_t default_timeout_ms = 1000;
  std::uint32_t activation_timeout_ms = 5000;
  bool allow_timed_out_adapter_replacement = true;
};

struct ModelLoadOptions {
  std::string initial_keyframe;
};

enum class StepCompletionSource {
  kCommit,
  kDeadline,
  kUnregister,
  kLocalError,
  kEmptyBarrier,
};

struct StepCompletedEvent {
  std::uint64_t completed_tick_id = 0;
  std::uint64_t next_tick_id = 0;
  double simulation_time_s = 0.0;
  StepCompletionSource source = StepCompletionSource::kCommit;
  std::vector<mujoco_sim::v2::ObservationFrame> observer_frames;
};

struct CommitResult {
  mujoco_sim::v2::ErrorCode code = mujoco_sim::v2::OK;
  std::string message;
  bool duplicate = false;
  std::optional<StepCompletedEvent> step_completed;
  bool recovery_pending = false;
  bool controller_state_changed = false;
  mujoco_sim::v2::ControllerState controller_state =
      mujoco_sim::v2::CONTROLLER_STATE_UNSPECIFIED;
};

struct AdapterTransition {
  std::string adapter_id;
  mujoco_sim::v2::AdapterRole role =
      mujoco_sim::v2::ADAPTER_ROLE_UNSPECIFIED;
  std::uint64_t session_id = 0;
  mujoco_sim::v2::ControllerState old_state =
      mujoco_sim::v2::CONTROLLER_STATE_UNSPECIFIED;
  mujoco_sim::v2::ControllerState new_state =
      mujoco_sim::v2::CONTROLLER_STATE_UNSPECIFIED;
  std::string message;
  std::uint64_t tick_id = 0;
};

struct DeadlineResult {
  std::vector<AdapterTransition> transitions;
  std::optional<StepCompletedEvent> step_completed;
};

struct UnregisterAdapterResult {
  mujoco_sim::v2::UnregisterAdapterResponse response;
  std::optional<StepCompletedEvent> step_completed;
};

struct EmptyBarrierResult {
  std::optional<StepCompletedEvent> step_completed;
};

struct IssuedTickIdentity {
  std::string adapter_id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint64_t tick_id = 0;
  std::uint64_t issue_generation = 0;
};

struct IssuedTick {
  mujoco_sim::v2::Tick message;
  std::string serialized_bytes;
  std::size_t layout_bytes_avoided = 0;
  std::uint32_t serialization_count = 0;
  IssuedTickIdentity identity;
  mutable std::atomic<bool> current{true};
};

using IssuedTickHandle = std::shared_ptr<const IssuedTick>;


class SimulationSnapshot {
 public:
  ~SimulationSnapshot();

  SimulationSnapshot(const SimulationSnapshot&) = delete;
  SimulationSnapshot& operator=(const SimulationSnapshot&) = delete;
  SimulationSnapshot(SimulationSnapshot&&) noexcept;
  SimulationSnapshot& operator=(SimulationSnapshot&&) noexcept;

  const mjModel_* model() const;
  mjData_* data() const;

 private:
  struct Impl;
  SimulationSnapshot();

  std::unique_ptr<Impl> impl_;

  friend class SimulationCore;
  friend class detail::SnapshotManager;
};

class SimulationCore {
 public:
  explicit SimulationCore(CoreOptions options = {});
  ~SimulationCore();

  SimulationCore(const SimulationCore&) = delete;
  SimulationCore& operator=(const SimulationCore&) = delete;
  SimulationCore(SimulationCore&&) noexcept;
  SimulationCore& operator=(SimulationCore&&) noexcept;

  bool LoadModel(const std::string& model_path, std::string* error_message);
  bool LoadModel(const std::string& model_path,
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

  mujoco_sim::v2::GetAvailableResourcesResponse GetAvailableResources(
      const mujoco_sim::v2::GetAvailableResourcesRequest& request) const;
  mujoco_sim::v2::RegisterAdapterResponse RegisterAdapter(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  mujoco_sim::v2::ActivateAdapterResponse ActivateAdapter(
      const mujoco_sim::v2::ActivateAdapterRequest& request);
  UnregisterAdapterResult UnregisterAdapter(
      const mujoco_sim::v2::UnregisterAdapterRequest& request);

  std::vector<IssuedTickHandle> OpenIssuedTicks();
  bool IsIssuedTickCurrent(const IssuedTickIdentity& identity) const;
  EmptyBarrierResult CompleteEmptyBarrier();
  CommitResult SubmitCommit(const mujoco_sim::v2::AdapterCommit& commit);
  void AcceptHeartbeat(
      const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now());
  DeadlineResult CheckDeadlines(
      std::chrono::steady_clock::time_point now);

  void EnterFatalError(const std::string& message);
  void MarkStopped(const std::string& message = {});

  mujoco_sim::v2::ServerStatus MakeStatus() const;

  std::uint64_t epoch() const;
  std::uint64_t current_tick_id() const;
  double simulation_time() const;
  double timestep() const;
  mujoco_sim::v2::ServerState server_state() const;
  bool tick_open() const;
  std::size_t active_controller_count() const;
  std::size_t controller_count(mujoco_sim::v2::ControllerState state) const;
  std::size_t observer_count(mujoco_sim::v2::ControllerState state) const;

 private:
  void BuildCatalogs();
  mujoco_sim::v2::RegisterAdapterResponse RegisterControlAdapter(
      const mujoco_sim::v2::RegisterAdapterRequest& request);
  mujoco_sim::v2::ActivateAdapterResponse ActivateControlAdapter(
      const mujoco_sim::v2::ActivateAdapterRequest& request);
  UnregisterAdapterResult UnregisterControlAdapter(
      const mujoco_sim::v2::UnregisterAdapterRequest& request);
  mujoco_sim::v2::Tick BuildTickFor(
      const detail::Controller& controller) const;
  mujoco_sim::v2::ObservationFrame BuildObservationFrame(
      detail::Observer* observer) const;
  std::vector<mujoco_sim::v2::ObservationFrame>
  BuildObservationFramesForCompletedStep();
  std::optional<StepCompletedEvent> CompleteBarrier(
      StepCompletionSource source);
  mujoco_sim::v2::Header MakeHeader(std::uint64_t session_id) const;
  void EnterErrorPaused(const std::string& message);
  bool ValidateCommands(const detail::Controller& controller,
                        const mujoco_sim::v2::AdapterCommit& commit,
                        std::string* message) const;
  void ZeroControllerCommands(const detail::Controller& controller);
  void SetControllerState(detail::Controller* controller,
                          mujoco_sim::v2::ControllerState state,
                          const std::string& message);
  std::optional<StepCompletedEvent> EnterControllerError(
      detail::Controller* controller, const std::string& message);
  void ReleaseControllerResources(const detail::Controller& controller);
  bool ValidateCommonHeader(const mujoco_sim::v2::Header& header,
                            const detail::Controller& controller,
                            std::string* message) const;
  void InvalidateIssuedTick(const std::string& adapter_id);
  void InvalidateAllIssuedTicks();
  void EnsureInternalComponents();

  CoreOptions options_;
  std::unique_ptr<detail::ModelState> model_state_;
  std::unique_ptr<detail::ResourceCatalog> resource_catalog_;
  std::unique_ptr<detail::SnapshotManager> snapshot_manager_;
  std::unique_ptr<detail::ObservationScheduler> observation_scheduler_;
  std::unique_ptr<detail::StatusBuilder> status_builder_;
  std::unique_ptr<detail::AdapterRegistry> adapter_registry_;
  std::unique_ptr<detail::BarrierCoordinator> barrier_coordinator_;
  std::unique_ptr<detail::TickStore> tick_store_;
  std::uint64_t epoch_ = 1;
  std::uint64_t current_tick_id_ = 0;
  mujoco_sim::v2::ServerState server_state_ = mujoco_sim::v2::IDLE;
  bool tick_open_ = false;
  std::chrono::steady_clock::time_point tick_started_at_{};
  std::string status_message_;
};

}  // namespace mujoco_sim::core
