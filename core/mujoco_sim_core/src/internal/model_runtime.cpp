#include "model_runtime.hpp"

#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

namespace mujoco_sim::core::internal {

void ModelDeleter::operator()(mjModel_* model) const {
  if (model != nullptr) mj_deleteModel(model);
}

void DataDeleter::operator()(mjData_* data) const {
  if (data != nullptr) mj_deleteData(data);
}

std::optional<ModelRuntime::PreparedModel> ModelRuntime::Prepare(
    const std::string& model_path, const ModelLoadOptions& options,
    std::string* error_message) {
  char load_error[1024] = {};
  PreparedModel prepared;
  prepared.model.reset(mj_loadXML(model_path.c_str(), nullptr, load_error,
                                  sizeof(load_error)));
  if (!prepared.model) {
    if (error_message != nullptr) {
      *error_message = load_error[0] == '\0' ? "failed to load MuJoCo model"
                                             : load_error;
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
    const int keyframe_id = mj_name2id(prepared.model.get(), mjOBJ_KEY,
                                      options.initial_keyframe.c_str());
    if (keyframe_id < 0) {
      if (error_message != nullptr) {
        *error_message = "initial keyframe '" + options.initial_keyframe +
                         "' was not found in MuJoCo model";
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

void ModelRuntime::ApplyActuatorWrites(
    const std::vector<ValidatedActuatorWrite>& writes) {
  if (data_ == nullptr) return;
  for (const auto& write : writes) {
    data_->ctrl[write.actuator_id] = write.value;
  }
}

void ModelRuntime::ZeroActuators(const std::vector<int>& actuator_ids) {
  if (data_ == nullptr) return;
  for (const int actuator_id : actuator_ids) data_->ctrl[actuator_id] = 0.0;
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
