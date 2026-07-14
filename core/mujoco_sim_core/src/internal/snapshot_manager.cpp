#include "snapshot_manager.hpp"

#include <memory>
#include <utility>

#include <mujoco/mujoco.h>

#include "model_runtime.hpp"

namespace mujoco_sim::core {

using internal::ModelRuntime;

void detail::SnapshotManager::Reset() {
  identity_ = std::make_shared<char>();
  ++model_generation_;
}

struct SimulationSnapshot::Impl {
  ~Impl() {
    if (data != nullptr) {
      mj_deleteData(data);
    }
  }

  std::shared_ptr<mjModel> model;
  mjData* data = nullptr;
  std::weak_ptr<void> owner;
  std::uint64_t model_generation = 0;
};

std::unique_ptr<SimulationSnapshot> detail::SnapshotManager::Create(
    const ModelRuntime& model_state, std::string* error_message) const {
  if (!model_state.loaded()) {
    if (error_message != nullptr) {
      *error_message = "cannot create snapshot before loading a model";
    }
    return nullptr;
  }
  auto snapshot =
      std::unique_ptr<SimulationSnapshot>(new SimulationSnapshot());
  snapshot->impl_->owner = identity_;
  snapshot->impl_->model_generation = model_generation_;
  snapshot->impl_->model = std::shared_ptr<mjModel>(
      mj_copyModel(nullptr, model_state.model_ptr()),
      [](mjModel* model) { mj_deleteModel(model); });
  if (snapshot->impl_->model == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo model for visualization";
    }
    return nullptr;
  }
  snapshot->impl_->data = mj_copyData(nullptr, snapshot->impl_->model.get(),
                                      model_state.data_ptr());
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo data for visualization";
    }
    return nullptr;
  }
  return snapshot;
}

std::unique_ptr<SimulationSnapshot> detail::SnapshotManager::Clone(
    const ModelRuntime& model_state, const SimulationSnapshot& source,
    std::string* error_message) const {
  if (!model_state.loaded() || source.impl_ == nullptr ||
      source.impl_->model == nullptr || source.impl_->data == nullptr ||
      source.impl_->owner.lock() != identity_ ||
      source.impl_->model_generation != model_generation_) {
    if (error_message != nullptr) {
      *error_message = "cannot clone an invalid or stale simulation snapshot";
    }
    return nullptr;
  }
  auto snapshot =
      std::unique_ptr<SimulationSnapshot>(new SimulationSnapshot());
  snapshot->impl_->owner = identity_;
  snapshot->impl_->model_generation = model_generation_;
  snapshot->impl_->model = source.impl_->model;
  snapshot->impl_->data = mj_copyData(nullptr, snapshot->impl_->model.get(),
                                      source.impl_->data);
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to allocate cloned MuJoCo snapshot data";
    }
    return nullptr;
  }
  return snapshot;
}

bool detail::SnapshotManager::Update(const ModelRuntime& model_state,
                                     SimulationSnapshot* snapshot,
                                     std::string* error_message) const {
  if (!model_state.loaded() || snapshot == nullptr ||
      snapshot->impl_ == nullptr || snapshot->impl_->model == nullptr ||
      snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) *error_message = "invalid visualization snapshot";
    return false;
  }
  if (snapshot->impl_->owner.lock() != identity_ ||
      snapshot->impl_->model_generation != model_generation_) {
    if (error_message != nullptr) {
      *error_message = "visualization snapshot does not belong to this model";
    }
    return false;
  }
  if (snapshot->impl_->model->nbuffer != model_state.model_ptr()->nbuffer) {
    if (error_message != nullptr) {
      *error_message = "visualization snapshot model is incompatible";
    }
    return false;
  }
  mj_copyData(snapshot->impl_->data, snapshot->impl_->model.get(),
              model_state.data_ptr());
  return true;
}

SimulationSnapshot::SimulationSnapshot() : impl_(std::make_unique<Impl>()) {}
SimulationSnapshot::~SimulationSnapshot() = default;
SimulationSnapshot::SimulationSnapshot(SimulationSnapshot&&) noexcept = default;
SimulationSnapshot& SimulationSnapshot::operator=(
    SimulationSnapshot&&) noexcept = default;

const mjModel_* SimulationSnapshot::model() const {
  return impl_->model.get();
}
mjData_* SimulationSnapshot::data() const { return impl_->data; }


}  // namespace mujoco_sim::core
