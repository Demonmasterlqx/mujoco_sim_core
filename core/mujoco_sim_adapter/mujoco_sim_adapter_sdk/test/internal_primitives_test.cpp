#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "internal/adapter_session.hpp"
#include "internal/blocking_mailbox.hpp"
#include "internal/callback_gate.hpp"

namespace mujoco_sim::adapter_sdk::internal {
namespace {

using namespace std::chrono_literals;

TEST(AdapterSessionTest, EnforcesTransitionOrder) {
  AdapterSession session;
  EXPECT_FALSE(session.BeginActivation());
  EXPECT_TRUE(session.Registered({11, 7}));
  EXPECT_TRUE(session.has_session());
  EXPECT_EQ(session.identity().session_id, 11u);
  EXPECT_EQ(session.identity().epoch, 7u);
  EXPECT_TRUE(session.StartCompleted());
  EXPECT_TRUE(session.BeginActivation());
  EXPECT_TRUE(session.Activated());
  EXPECT_TRUE(session.active());
  EXPECT_FALSE(session.StartCompleted());
}

TEST(AdapterSessionTest, StopCannotBeOverwrittenByLateResponses) {
  AdapterSession session;
  session.Stop();
  EXPECT_FALSE(session.Registered({11, 7}));
  EXPECT_TRUE(session.has_session());
  EXPECT_TRUE(session.stopped());
  EXPECT_FALSE(session.StartCompleted());
  EXPECT_FALSE(session.BeginActivation());
  EXPECT_FALSE(session.Activated());
  session.ReleaseSession();
  EXPECT_FALSE(session.has_session());
  EXPECT_TRUE(session.stopped());
}

TEST(AdapterSessionTest, TerminalErrorSurvivesStopAndRemoteCleanup) {
  AdapterSession session;
  EXPECT_TRUE(session.Registered({11, 7}));
  session.Fail();
  session.Stop();
  EXPECT_TRUE(session.terminal_error());
  EXPECT_TRUE(session.has_session());
  session.ReleaseSession();
  EXPECT_TRUE(session.terminal_error());
  EXPECT_FALSE(session.has_session());
}

TEST(BlockingMailboxTest, DropsOldestAtCapacity) {
  BlockingMailbox<int> mailbox;
  EXPECT_EQ(mailbox.Push(1, 2), 0u);
  EXPECT_EQ(mailbox.Push(2, 2), 0u);
  EXPECT_EQ(mailbox.Push(3, 2), 1u);

  std::mutex mutex;
  std::condition_variable condition;
  std::unique_lock lock(mutex);
  int output = 0;
  std::string error;
  EXPECT_EQ(mailbox.Wait(lock, condition, 0ms, [] { return false; },
                         [] { return std::string{}; }, &output, &error),
            MailboxWaitResult::kReceived);
  EXPECT_EQ(output, 2);
}

TEST(BlockingMailboxTest, TerminalErrorPrecedesStopAndQueuedData) {
  BlockingMailbox<int> mailbox;
  mailbox.Push(7, 1);
  std::mutex mutex;
  std::condition_variable condition;
  std::unique_lock lock(mutex);
  int output = 0;
  std::string error;
  EXPECT_EQ(mailbox.Wait(lock, condition, 0ms, [] { return true; },
                         [] { return std::string{"terminal"}; }, &output,
                         &error),
            MailboxWaitResult::kError);
  EXPECT_EQ(error, "terminal");
  EXPECT_EQ(output, 0);
}

struct CallbackTarget {};
using TestCallbackGate = CallbackGate<CallbackTarget>;

TEST(CallbackGateTest, DisableRejectsLateCallbacks) {
  CallbackTarget target;
  auto state = std::make_shared<TestCallbackGate::State>();
  state->client = &target;
  state->enabled = true;
  TestCallbackGate::Disable(state);
  TestCallbackGate::Lease lease(state);
  EXPECT_EQ(lease.client(), nullptr);
}

TEST(CallbackGateTest, CurrentCallbackDoesNotWaitForItself) {
  CallbackTarget target;
  auto state = std::make_shared<TestCallbackGate::State>();
  state->client = &target;
  state->enabled = true;
  TestCallbackGate::Lease lease(state);
  TestCallbackGate::Disable(state);
  TestCallbackGate::WaitForOtherCallbacks(state);
  EXPECT_EQ(lease.client(), &target);
}

TEST(CallbackGateTest, WaitsForCallbackOnAnotherThread) {
  CallbackTarget target;
  auto state = std::make_shared<TestCallbackGate::State>();
  state->client = &target;
  state->enabled = true;
  std::promise<void> leased;
  std::promise<void> release;
  auto release_future = release.get_future();
  std::thread worker([&] {
    TestCallbackGate::Lease lease(state);
    leased.set_value();
    release_future.wait();
  });
  leased.get_future().wait();
  TestCallbackGate::Disable(state);

  auto waiter = std::async(std::launch::async, [state] {
    TestCallbackGate::WaitForOtherCallbacks(state);
  });
  EXPECT_EQ(waiter.wait_for(20ms), std::future_status::timeout);
  release.set_value();
  EXPECT_EQ(waiter.wait_for(1s), std::future_status::ready);
  worker.join();
}

}  // namespace
}  // namespace mujoco_sim::adapter_sdk::internal
