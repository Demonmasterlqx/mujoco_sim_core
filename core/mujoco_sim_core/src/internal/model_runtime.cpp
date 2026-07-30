#include "model_runtime.hpp"

#include <algorithm>
#include <cmath>

#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

namespace mujoco_sim::core::internal {

void ModelDeleter::operator()(mjModel_* model) const {
  if (model != nullptr) mj_deleteModel(model);
}

void DataDeleter::operator()(mjData_* data) const {
  if (data != nullptr) mj_deleteData(data);
}

std::optional<ModelRuntime::PreparedModel> ModelRuntime::Prepare(const std::string& model_path,
                                                                 const ModelLoadOptions& options,
                                                                 std::string* error_message) {
  char load_error[1024] = {};
  PreparedModel prepared;
  prepared.model.reset(mj_loadXML(model_path.c_str(), nullptr, load_error, sizeof(load_error)));
  if (!prepared.model) {
    if (error_message != nullptr) {
      *error_message = load_error[0] == '\0' ? "failed to load MuJoCo model" : load_error;
    }
    return std::nullopt;
  }
  prepared.data.reset(mj_makeData(prepared.model.get()));
  if (!prepared.data) {
    if (error_message != nullptr) {
      *error_message = "failed to allocate MuJoCo data";
    }
    return std::nullopt;
  }
  if (options.initial_keyframe.empty()) {
    mj_resetData(prepared.model.get(), prepared.data.get());
  } else {
    const int keyframe_id =
        mj_name2id(prepared.model.get(), mjOBJ_KEY, options.initial_keyframe.c_str());
    if (keyframe_id < 0) {
      if (error_message != nullptr) {
        *error_message =
            "initial keyframe '" + options.initial_keyframe + "' was not found in MuJoCo model";
      }
      return std::nullopt;
    }
    mj_resetDataKeyframe(prepared.model.get(), prepared.data.get(), keyframe_id);
  }
  if (prepared.model->nu > 0) {
    mju_zero(prepared.data->ctrl, prepared.model->nu);
  }
  mj_forward(prepared.model.get(), prepared.data.get());
  return prepared;
}

void ModelRuntime::Commit(PreparedModel prepared) {
  model_ = std::move(prepared.model);
  data_ = std::move(prepared.data);
  simulation_time_ = data_->time;
  ++generation_;
}

bool ModelRuntime::loaded() const noexcept {
  return model_ != nullptr && data_ != nullptr;
}

void ModelRuntime::ApplyActuatorWrites(const std::vector<ValidatedActuatorWrite>& writes) {
  if (data_ == nullptr) return;
  for (const auto& write : writes) {
    data_->ctrl[write.actuator_id] = write.value;
  }
}

void ModelRuntime::ApplyInteractiveActuatorWrites(
    std::span<const InteractiveActuatorWrite> writes) {
  if (!loaded()) return;
  for (const auto& write : writes) {
    if (write.actuator_id < 0 || write.actuator_id >= model_->nu || !std::isfinite(write.value)) {
      continue;
    }
    data_->ctrl[write.actuator_id] = static_cast<mjtNum>(write.value);
  }
}

void ModelRuntime::ReplaceInteractiveBodyWrenches(std::span<const InteractiveBodyWrench> wrenches) {
  if (!loaded()) return;

  if (interactive_wrench_generation_ == generation_) {
    for (const int body_id : interactive_wrench_body_ids_) {
      mju_zero(data_->xfrc_applied + 6 * body_id, 6);
    }
  }
  interactive_wrench_body_ids_.clear();
  interactive_wrench_generation_ = generation_;

  for (const auto& write : wrenches) {
    const bool valid_body = write.body_id > 0 && write.body_id < model_->nbody &&
                            model_->body_dofnum[write.body_id] > 0;
    const bool finite =
        std::ranges::all_of(write.force, [](double value) { return std::isfinite(value); }) &&
        std::ranges::all_of(write.torque, [](double value) { return std::isfinite(value); });
    if (!valid_body || !finite) continue;

    mjtNum* target = data_->xfrc_applied + 6 * write.body_id;
    std::copy(write.force.begin(), write.force.end(), target);
    std::copy(write.torque.begin(), write.torque.end(), target + 3);
    if (std::ranges::find(interactive_wrench_body_ids_, write.body_id) ==
        interactive_wrench_body_ids_.end()) {
      interactive_wrench_body_ids_.push_back(write.body_id);
    }
  }
}

void ModelRuntime::ZeroActuators(const std::vector<int>& actuator_ids) {
  if (data_ == nullptr) return;
  for (const int actuator_id : actuator_ids)
    data_->ctrl[actuator_id] = 0.0;
}

void ModelRuntime::ZeroAllControls() {
  if (loaded() && model_->nu > 0) mju_zero(data_->ctrl, model_->nu);
}

void ModelRuntime::Step() {
  mj_step(model_.get(), data_.get());
  simulation_time_ = data_->time;
}

double ModelRuntime::timestep() const noexcept {
  return model_ == nullptr ? 0.0 : model_->opt.timestep;
}

}  // namespace mujoco_sim::core::internal
