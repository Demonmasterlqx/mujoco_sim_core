#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "server_event_queue.hpp"

namespace mujoco_sim::server {
namespace {

TEST(ServerEventQueueTest, CommandBurstsDoNotConsumeGeneralQueue) {
  ServerEventQueue queue;
  for (std::uint64_t i = 0; i < 100; ++i) {
    ASSERT_TRUE(queue.TryPush(HeartbeatEvent{{{i + 1, 1}, i}}));
  }
  EXPECT_EQ(queue.size(), 100U);
  auto batch = queue.WaitUntil(std::chrono::steady_clock::now());
  EXPECT_EQ(batch.size(), ServerEventQueue::kMaxBatchSize);
}

TEST(ServerEventQueueTest, NotifyWakesWaiterWithoutFifoPayload) {
  using namespace std::chrono_literals;
  ServerEventQueue queue;
  auto waiter = std::async(std::launch::async,
                           [&] { return queue.WaitUntil(std::chrono::steady_clock::now() + 2s); });

  queue.Notify();

  ASSERT_EQ(waiter.wait_for(200ms), std::future_status::ready);
  EXPECT_TRUE(waiter.get().empty());
  EXPECT_EQ(queue.size(), 0U);
}

TEST(ServerEventQueueTest, NotificationBeforeWaitIsNotLost) {
  using namespace std::chrono_literals;
  ServerEventQueue queue;
  queue.Notify();
  auto waiter = std::async(std::launch::async,
                           [&] { return queue.WaitUntil(std::chrono::steady_clock::now() + 2s); });

  ASSERT_EQ(waiter.wait_for(200ms), std::future_status::ready);
  EXPECT_TRUE(waiter.get().empty());
}

TEST(ServerEventQueueTest, CloseCancelsWaitAndRejectsPush) {
  ServerEventQueue queue;
  queue.Close();
  EXPECT_TRUE(queue.closed());
  EXPECT_FALSE(queue.TryPush(HeartbeatEvent{}));
  EXPECT_TRUE(queue.WaitUntil(std::chrono::steady_clock::now()).empty());
}

}  // namespace
}  // namespace mujoco_sim::server
