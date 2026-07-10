#include "sim_viewer.hpp"

#include <cstdio>
#include <memory>
#include <string>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_sim::server {

struct SimViewer::Impl {
  mjvCamera camera{};
  mjvOption option{};
  mjvPerturb perturb{};
  std::unique_ptr<mujoco::Simulate> simulate;
};

SimViewer::SimViewer() : impl_(std::make_unique<Impl>()) {}
SimViewer::~SimViewer() = default;

bool SimViewer::Initialize(std::string* error_message) {
  if (glfwInit() != GLFW_TRUE) {
    const char* description = nullptr;
    glfwGetError(&description);
    if (error_message != nullptr) {
      *error_message = description == nullptr
                           ? "failed to initialize GLFW"
                           : "failed to initialize GLFW: " +
                                 std::string(description);
    }
    return false;
  }

  mjv_defaultCamera(&impl_->camera);
  mjv_defaultOption(&impl_->option);
  mjv_defaultPerturb(&impl_->perturb);
  impl_->simulate = std::make_unique<mujoco::Simulate>(
      std::make_unique<mujoco::GlfwAdapter>(), &impl_->camera,
      &impl_->option, &impl_->perturb, true);
  return true;
}

bool SimViewer::QueueLoad(mujoco_sim::core::SimulationSnapshot* snapshot,
                          const std::string& displayed_filename,
                          std::string* error_message) {
  if (impl_->simulate == nullptr || snapshot == nullptr ||
      snapshot->model() == nullptr || snapshot->data() == nullptr) {
    if (error_message != nullptr) {
      *error_message = "viewer is not initialized with a valid snapshot";
    }
    return false;
  }
  const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
  impl_->simulate->mnew_ = snapshot->model();
  impl_->simulate->dnew_ = snapshot->data();
  std::snprintf(impl_->simulate->filename,
                sizeof(impl_->simulate->filename), "%s",
                displayed_filename.c_str());
  impl_->simulate->loadrequest = 2;
  return true;
}

bool SimViewer::Synchronize(mujoco_sim::core::SimulationCore* core,
                            mujoco_sim::core::SimulationSnapshot* snapshot,
                            std::string* error_message) {
  if (impl_->simulate == nullptr || core == nullptr || close_requested()) {
    return false;
  }
  const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
  if (impl_->simulate->loadrequest != 0) {
    return true;
  }
  if (!core->UpdateSnapshot(snapshot, error_message)) {
    return false;
  }
  impl_->simulate->Sync();
  return true;
}

void SimViewer::RenderLoop() {
  if (impl_->simulate != nullptr) {
    impl_->simulate->RenderLoop();
  }
}

void SimViewer::RequestScreenshot() {
  if (impl_->simulate != nullptr) {
    impl_->simulate->screenshotrequest.store(1);
  }
}

void SimViewer::RequestClose() {
  if (impl_->simulate != nullptr) {
    impl_->simulate->exitrequest.store(1);
  }
}

bool SimViewer::close_requested() const {
  return impl_->simulate != nullptr && impl_->simulate->exitrequest.load() != 0;
}

bool SimViewer::model_loaded() const {
  if (impl_->simulate == nullptr) {
    return false;
  }
  const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
  return impl_->simulate->loadrequest == 0 && impl_->simulate->m_ != nullptr;
}

}  // namespace mujoco_sim::server
