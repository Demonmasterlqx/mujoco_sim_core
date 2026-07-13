#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <mutex>

#include "simulation_scheduler.hpp"

namespace mujoco_sim::server {
namespace {

using namespace std::chrono_literals;

TEST(SimulationSchedulerTest, SerializesStepPipelines) {
  SimulationScheduler scheduler;
  EXPECT_FALSE(scheduler.step_pipeline_busy());
  EXPECT_TRUE(scheduler.BeginStepPipeline());
  EXPECT_TRUE(scheduler.step_pipeline_busy());
  EXPECT_FALSE(scheduler.BeginStepPipeline());
  scheduler.CompleteStepPipeline();
  EXPECT_FALSE(scheduler.step_pipeline_busy());
  EXPECT_TRUE(scheduler.BeginStepPipeline());
  scheduler.CompleteStepPipeline();
}

TEST(SimulationSchedulerTest, CompletionWakesSchedulerWait) {
  SimulationScheduler scheduler;
  ASSERT_TRUE(scheduler.BeginStepPipeline());
  std::mutex mutex;
  std::promise<void> entered;
  auto waiting = std::async(std::launch::async, [&] {
    std::unique_lock lock(mutex);
    entered.set_value();
    scheduler.WaitUntil(lock, std::chrono::steady_clock::now() + 5s);
  });
  entered.get_future().wait();
  scheduler.CompleteStepPipeline();
  EXPECT_EQ(waiting.wait_for(1s), std::future_status::ready);
}

TEST(SimulationSchedulerTest, IdleSchedulerWaitsUntilDeadline) {
  SimulationScheduler scheduler;
  std::mutex mutex;
  const auto started = std::chrono::steady_clock::now();
  {
    std::unique_lock lock(mutex);
    scheduler.WaitUntil(lock, started + 50ms);
  }
  EXPECT_GE(std::chrono::steady_clock::now() - started, 40ms);
}

TEST(SimulationSchedulerTest, NotificationWakesIdleScheduler) {
  SimulationScheduler scheduler;
  std::mutex mutex;
  std::promise<void> entered;
  auto waiting = std::async(std::launch::async, [&] {
    std::unique_lock lock(mutex);
    entered.set_value();
    scheduler.WaitUntil(lock, std::chrono::steady_clock::now() + 5s);
  });
  entered.get_future().wait();
  scheduler.NotifyCommitReceived();
  EXPECT_EQ(waiting.wait_for(1s), std::future_status::ready);
}

}  // namespace
}  // namespace mujoco_sim::server
