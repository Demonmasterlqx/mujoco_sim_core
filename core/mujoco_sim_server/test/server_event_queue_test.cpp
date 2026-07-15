#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "server_event_queue.hpp"

namespace mujoco_sim::server {
namespace {

using namespace std::chrono_literals;

CommitEvent Commit(std::uint64_t tick_id, std::size_t payload_bytes = 0) {
  CommitEvent event;
  event.message.identity.tick_id = tick_id;
  if (payload_bytes > 0) {
    event.message.commands.resize(
        (payload_bytes + sizeof(double) - 1) / sizeof(double));
  }
  return event;
}

TEST(ServerEventQueueTest, PreservesFifoOrderAndBatchLimit) {
  ServerEventQueue queue;
  for (std::size_t i = 0; i < ServerEventQueue::kMaxBatchSize + 2; ++i) {
    ASSERT_TRUE(queue.TryPush(Commit(i)));
  }

  const auto first = queue.WaitUntil(std::chrono::steady_clock::now());
  ASSERT_EQ(first.size(), ServerEventQueue::kMaxBatchSize);
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(std::get<CommitEvent>(first[i]).message.identity.tick_id, i);
  }
  EXPECT_EQ(queue.size(), 2U);
}

TEST(ServerEventQueueTest, WaitWakesForAnEvent) {
  ServerEventQueue queue;
  auto waiting = std::async(std::launch::async, [&] {
    return queue.WaitUntil(std::chrono::steady_clock::now() + 5s);
  });
  std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(queue.TryPush(Commit(7)));
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get().size(), 1U);
}

TEST(ServerEventQueueTest, EnforcesEventAndPayloadLimits) {
  ServerEventQueue count_limited;
  for (std::size_t i = 0; i < ServerEventQueue::kMaxEvents; ++i) {
    ASSERT_TRUE(count_limited.TryPush(Commit(i)));
  }
  EXPECT_FALSE(count_limited.TryPush(Commit(9999)));
  EXPECT_EQ(count_limited.high_watermark(), ServerEventQueue::kMaxEvents);

  ServerEventQueue byte_limited;
  EXPECT_FALSE(byte_limited.TryPush(
      Commit(1, ServerEventQueue::kMaxPayloadBytes + 1)));
}

TEST(ServerEventQueueTest, CloseRejectsInputAndCompletesQueuedRpc) {
  ServerEventQueue queue;
  RegisterAdapterEvent event;
  event.request.set_adapter_id("controller");
  auto response = event.completion.get_future();
  ASSERT_TRUE(queue.TryPush(std::move(event)));

  queue.Close("test shutdown");

  ASSERT_EQ(response.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(response.get().code(), mujoco_sim::v3::INTERNAL_ERROR);
  EXPECT_TRUE(queue.closed());
  EXPECT_FALSE(queue.TryPush(Commit(1)));
  EXPECT_EQ(queue.payload_bytes(), 0U);
}

TEST(ServerEventQueueTest, SupportsConcurrentProducers) {
  ServerEventQueue queue;
  constexpr std::size_t kPerProducer = 100;
  std::vector<std::thread> producers;
  for (std::size_t producer = 0; producer < 4; ++producer) {
    producers.emplace_back([&, producer] {
      for (std::size_t i = 0; i < kPerProducer; ++i) {
        EXPECT_TRUE(queue.TryPush(Commit(producer * kPerProducer + i)));
      }
    });
  }
  for (auto& producer : producers) producer.join();
  EXPECT_EQ(queue.size(), 4U * kPerProducer);
}

}  // namespace
}  // namespace mujoco_sim::server
