#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace mujoco_sim::server {

class SimulationScheduler {
 public:
  bool BeginStepPipeline() noexcept;
  void CompleteStepPipeline() noexcept;
  bool step_pipeline_busy() const noexcept;

  void NotifyLifecycleChanged() noexcept;
  void NotifyCommitReceived() noexcept;
  void WaitUntil(std::unique_lock<std::mutex>& lock,
                 std::chrono::steady_clock::time_point deadline);

 private:
  std::condition_variable condition_;
  mutable std::mutex mutex_;
  bool step_pipeline_busy_ = false;
  std::uint64_t wake_generation_ = 0;
};

}  // namespace mujoco_sim::server
