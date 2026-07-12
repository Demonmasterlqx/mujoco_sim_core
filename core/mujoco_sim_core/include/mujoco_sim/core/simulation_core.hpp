#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/v1/sync.pb.h>

struct mjData_;
struct mjModel_;

namespace mujoco_sim::core {

struct CoreOptions {
  std::string server_name = "default";
  std::uint32_t default_timeout_ms = 1000;
  std::uint32_t activation_timeout_ms = 5000;
  bool allow_timed_out_adapter_replacement = true;
};

struct ModelLoadOptions {
  std::string initial_keyframe;
};

struct CommitResult {
  mujoco_sim::v1::ErrorCode code = mujoco_sim::v1::OK;
  std::string message;
  bool duplicate = false;
  bool stepped = false;
  bool recovery_pending = false;
  bool controller_state_changed = false;
  mujoco_sim::v1::ControllerState controller_state =
      mujoco_sim::v1::CONTROLLER_STATE_UNSPECIFIED;
};

struct ControllerTransition {
  std::string controller_id;
  std::uint64_t session_id = 0;
  mujoco_sim::v1::ControllerState old_state =
      mujoco_sim::v1::CONTROLLER_STATE_UNSPECIFIED;
  mujoco_sim::v1::ControllerState new_state =
      mujoco_sim::v1::CONTROLLER_STATE_UNSPECIFIED;
  std::string message;
  std::uint64_t tick_id = 0;
};

struct DeadlineResult {
  std::vector<ControllerTransition> transitions;
  struct AdapterTransition {
    std::string adapter_id;
    mujoco_sim::v1::AdapterRole role =
        mujoco_sim::v1::ADAPTER_ROLE_UNSPECIFIED;
    std::uint64_t session_id = 0;
    mujoco_sim::v1::ControllerState old_state =
        mujoco_sim::v1::CONTROLLER_STATE_UNSPECIFIED;
    mujoco_sim::v1::ControllerState new_state =
        mujoco_sim::v1::CONTROLLER_STATE_UNSPECIFIED;
    std::string message;
    std::uint64_t tick_id = 0;
  };
  std::vector<AdapterTransition> adapter_transitions;
  bool stepped = false;
};

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

  const std::vector<mujoco_sim::v1::ResourceDescriptor>& observations() const;
  const std::vector<mujoco_sim::v1::ResourceDescriptor>& commands() const;

  mujoco_sim::v1::GetAvailableResourcesResponse GetAvailableResources(
      const mujoco_sim::v1::GetAvailableResourcesRequest& request) const;
  mujoco_sim::v1::RegisterControllerResponse RegisterController(
      const mujoco_sim::v1::RegisterControllerRequest& request);
  mujoco_sim::v1::ActivateControllerResponse ActivateController(
      const mujoco_sim::v1::ActivateControllerRequest& request);
  mujoco_sim::v1::UnregisterControllerResponse UnregisterController(
      const mujoco_sim::v1::UnregisterControllerRequest& request);
  mujoco_sim::v1::RegisterAdapterResponse RegisterAdapter(
      const mujoco_sim::v1::RegisterAdapterRequest& request);
  mujoco_sim::v1::ActivateAdapterResponse ActivateAdapter(
      const mujoco_sim::v1::ActivateAdapterRequest& request);
  mujoco_sim::v1::UnregisterAdapterResponse UnregisterAdapter(
      const mujoco_sim::v1::UnregisterAdapterRequest& request);

  std::vector<mujoco_sim::v1::Tick> OpenTick();
  std::vector<mujoco_sim::v1::ObservationFrame> CollectObservationFrames();
  bool IsTickIssued(const mujoco_sim::v1::Tick& tick) const;
  bool StepIfBarrierComplete();
  CommitResult SubmitCommit(const mujoco_sim::v1::AdapterCommit& commit);
  void AcceptHeartbeat(const mujoco_sim::v1::Heartbeat& heartbeat);
  void AcceptAdapterHeartbeat(
      const mujoco_sim::v1::AdapterHeartbeat& heartbeat,
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now());
  DeadlineResult CheckControllerDeadlines(
      std::chrono::steady_clock::time_point now);
  bool CheckTickTimeout(std::chrono::steady_clock::time_point now);

  void EnterFatalError(const std::string& message);
  void MarkStopped(const std::string& message = {});

  mujoco_sim::v1::ServerStatus MakeStatus() const;

  std::uint64_t epoch() const;
  std::uint64_t current_tick_id() const;
  double simulation_time() const;
  double timestep() const;
  mujoco_sim::v1::ServerState server_state() const;
  bool tick_open() const;
  std::size_t active_controller_count() const;
  std::size_t controller_count(mujoco_sim::v1::ControllerState state) const;
  std::size_t observer_count(mujoco_sim::v1::ControllerState state) const;

 private:
  struct Controller;
  struct Observer;
  struct MjModelDeleter {
    void operator()(mjModel_* model) const;
  };
  struct MjDataDeleter {
    void operator()(mjData_* data) const;
  };

  void BuildCatalogs();
  mujoco_sim::v1::Tick BuildTickFor(const Controller& controller) const;
  mujoco_sim::v1::ObservationFrame BuildObservationFrame(
      Observer* observer) const;
  void QueueObservationFramesForCompletedStep();
  mujoco_sim::v1::Header MakeHeader(std::uint64_t session_id) const;
  void EnterErrorPaused(const std::string& message);
  bool ValidateCommands(const Controller& controller,
                        const mujoco_sim::v1::AdapterCommit& commit,
                        std::string* message) const;
  void ZeroControllerCommands(const Controller& controller);
  void SetControllerState(Controller* controller,
                          mujoco_sim::v1::ControllerState state,
                          const std::string& message);
  bool EnterControllerError(Controller* controller,
                            const std::string& message);
  void ReleaseControllerResources(const Controller& controller);
  bool ValidateCommonHeader(const mujoco_sim::v1::Header& header,
                            const Controller& controller,
                            std::string* message) const;

  CoreOptions options_;
  std::unique_ptr<mjModel_, MjModelDeleter> model_;
  std::unique_ptr<mjData_, MjDataDeleter> data_;
  std::vector<mujoco_sim::v1::ResourceDescriptor> observations_;
  std::vector<mujoco_sim::v1::ResourceDescriptor> commands_;
  std::unordered_map<std::string, mujoco_sim::v1::ResourceDescriptor>
      observations_by_name_;
  std::unordered_map<std::string, mujoco_sim::v1::ResourceDescriptor>
      commands_by_name_;
  std::unordered_map<std::string, Controller> controllers_;
  std::unordered_map<std::string, Observer> observers_;
  std::unordered_map<std::string, std::string> actuator_owner_;
  std::unordered_map<std::string, mujoco_sim::v1::AdapterCommit>
      pending_commits_;
  std::unordered_map<std::string, mujoco_sim::v1::Tick> open_ticks_;
  std::unordered_map<std::string, bool> tick_required_controllers_;
  std::unordered_map<std::string, bool> tick_recipient_controllers_;
  std::vector<mujoco_sim::v1::ObservationFrame>
      pending_observation_frames_;
  std::uint64_t epoch_ = 1;
  std::uint64_t current_tick_id_ = 0;
  std::uint64_t next_session_id_ = 1;
  double simulation_time_ = 0.0;
  mujoco_sim::v1::ServerState server_state_ = mujoco_sim::v1::IDLE;
  bool tick_open_ = false;
  std::chrono::steady_clock::time_point tick_started_at_{};
  std::string status_message_;
  std::shared_ptr<void> snapshot_identity_ = std::make_shared<char>();
  std::uint64_t model_generation_ = 0;
};

}  // namespace mujoco_sim::core
