#include "sim_viewer.hpp"

namespace mujoco_sim::server {

struct SimViewer::Impl {};

SimViewer::SimViewer() : impl_(std::make_unique<Impl>()) {}
SimViewer::~SimViewer() = default;

bool SimViewer::Initialize(std::string* error_message) {
  if (error_message != nullptr) {
    *error_message = "this build does not include GUI support";
  }
  return false;
}
bool SimViewer::QueueLoad(core::SimulationSnapshot*, const std::string&,
                          std::string* error_message) {
  return Initialize(error_message);
}
bool SimViewer::Synchronize(core::SimulationCore*, core::SimulationSnapshot*,
                            std::string* error_message) {
  return Initialize(error_message);
}
void SimViewer::RenderLoop() {}
void SimViewer::RequestScreenshot() {}
void SimViewer::RequestClose() {}
bool SimViewer::close_requested() const { return true; }
bool SimViewer::model_loaded() const { return false; }

}  // namespace mujoco_sim::server
