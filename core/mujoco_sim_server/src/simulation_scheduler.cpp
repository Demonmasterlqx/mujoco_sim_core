#include "simulation_scheduler.hpp"

namespace mujoco_sim::server {

bool SimulationScheduler::BeginStepPipeline() noexcept {
  std::lock_guard lock(mutex_);
  if (step_pipeline_busy_) return false;
  step_pipeline_busy_ = true;
  return true;
}

void SimulationScheduler::CompleteStepPipeline() noexcept {
  {
    std::lock_guard lock(mutex_);
    step_pipeline_busy_ = false;
    ++wake_generation_;
  }
  condition_.notify_all();
}

bool SimulationScheduler::step_pipeline_busy() const noexcept {
  std::lock_guard lock(mutex_);
  return step_pipeline_busy_;
}

void SimulationScheduler::NotifyLifecycleChanged() noexcept {
  {
    std::lock_guard lock(mutex_);
    ++wake_generation_;
  }
  condition_.notify_all();
}
void SimulationScheduler::NotifyCommitReceived() noexcept {
  {
    std::lock_guard lock(mutex_);
    ++wake_generation_;
  }
  condition_.notify_all();
}

void SimulationScheduler::WaitUntil(
    std::unique_lock<std::mutex>& lock,
    std::chrono::steady_clock::time_point deadline) {
  std::unique_lock scheduler_lock(mutex_);
  const auto initial_generation = wake_generation_;
  const bool wait_for_pipeline = step_pipeline_busy_;
  lock.unlock();
  condition_.wait_until(
      scheduler_lock, deadline,
      [this, initial_generation, wait_for_pipeline] {
        return wake_generation_ != initial_generation ||
               (wait_for_pipeline && !step_pipeline_busy_);
      });
  scheduler_lock.unlock();
  lock.lock();
}

}  // namespace mujoco_sim::server
