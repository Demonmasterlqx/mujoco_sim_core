#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

struct mjData_;
struct mjModel_;

namespace mujoco_sim::core {

struct ModelLoadOptions;

namespace internal {

struct ModelDeleter {
  void operator()(mjModel_* model) const;
};

struct DataDeleter {
  void operator()(mjData_* data) const;
};

struct ValidatedActuatorWrite {
  int actuator_id = -1;
  double value = 0.0;
};

class ModelRuntime {
 public:
  struct PreparedModel {
    std::unique_ptr<mjModel_, ModelDeleter> model;
    std::unique_ptr<mjData_, DataDeleter> data;
  };

  static std::optional<PreparedModel> Prepare(
      const std::string& model_path, const ModelLoadOptions& options,
      std::string* error_message);

  void Commit(PreparedModel prepared);
  bool loaded() const noexcept;
  void ApplyActuatorWrites(
      const std::vector<ValidatedActuatorWrite>& writes);
  void ZeroActuators(const std::vector<int>& actuator_ids);
  void ZeroAllControls();
  void Step();

  const mjModel_* model_ptr() const noexcept { return model_.get(); }
  const mjData_* data_ptr() const noexcept { return data_.get(); }
  double simulation_time() const noexcept { return simulation_time_; }
  double timestep() const noexcept;

 private:
  std::unique_ptr<mjModel_, ModelDeleter> model_;
  std::unique_ptr<mjData_, DataDeleter> data_;
  double simulation_time_ = 0.0;
  std::uint64_t generation_ = 0;
};

}  // namespace internal
}  // namespace mujoco_sim::core
