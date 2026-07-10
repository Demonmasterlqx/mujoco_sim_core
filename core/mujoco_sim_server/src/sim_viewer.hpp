#pragma once

#include <memory>
#include <string>

namespace mujoco_sim::core {
class SimulationCore;
class SimulationSnapshot;
}

namespace mujoco_sim::server {

class SimViewer {
 public:
  SimViewer();
  ~SimViewer();

  SimViewer(const SimViewer&) = delete;
  SimViewer& operator=(const SimViewer&) = delete;

  bool Initialize(std::string* error_message);
  bool QueueLoad(mujoco_sim::core::SimulationSnapshot* snapshot,
                 const std::string& displayed_filename,
                 std::string* error_message);
  bool Synchronize(mujoco_sim::core::SimulationCore* core,
                   mujoco_sim::core::SimulationSnapshot* snapshot,
                   std::string* error_message);
  void RenderLoop();
  void RequestClose();
  bool close_requested() const;
  bool model_loaded() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
