#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
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
};

struct CommitResult {
  mujoco_sim::v1::ErrorCode code = mujoco_sim::v1::OK;
  std::string message;
  bool duplicate = false;
  bool stepped = false;
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
  bool IsModelLoaded() const;

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

  std::vector<mujoco_sim::v1::Tick> OpenTick();
  CommitResult SubmitCommit(const mujoco_sim::v1::AdapterCommit& commit);
  void AcceptHeartbeat(const mujoco_sim::v1::Heartbeat& heartbeat);
  bool CheckTickTimeout(std::chrono::steady_clock::time_point now);

  mujoco_sim::v1::ServerStatus MakeStatus() const;

  std::uint64_t epoch() const;
  std::uint64_t current_tick_id() const;
  double simulation_time() const;
  mujoco_sim::v1::ServerState server_state() const;
  bool tick_open() const;
  std::size_t active_controller_count() const;

 private:
  struct Controller;
  struct MjModelDeleter {
    void operator()(mjModel_* model) const;
  };
  struct MjDataDeleter {
    void operator()(mjData_* data) const;
  };

  void BuildCatalogs();
  mujoco_sim::v1::Tick BuildTickFor(const Controller& controller) const;
  mujoco_sim::v1::Header MakeHeader(std::uint64_t session_id) const;
  void EnterErrorPaused(const std::string& message);
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
  std::unordered_map<std::string, std::string> actuator_owner_;
  std::unordered_map<std::string, mujoco_sim::v1::AdapterCommit>
      pending_commits_;
  std::uint64_t epoch_ = 1;
  std::uint64_t current_tick_id_ = 0;
  std::uint64_t next_session_id_ = 1;
  double simulation_time_ = 0.0;
  mujoco_sim::v1::ServerState server_state_ = mujoco_sim::v1::IDLE;
  bool tick_open_ = false;
  std::chrono::steady_clock::time_point tick_started_at_{};
  std::string status_message_;
};

}  // namespace mujoco_sim::core
