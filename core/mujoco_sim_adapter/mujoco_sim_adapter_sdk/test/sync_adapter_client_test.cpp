#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::adapter_sdk {

class SyncAdapterClientTestPeer {
 public:
  static void Configure(SyncAdapterClient* client,
                        const mujoco_sim::v2::Tick& tick,
                        bool activated = true) {
    client->stop_requested_ = false;
    {
      std::lock_guard lock(client->mutex_);
      client->registration_.set_code(mujoco_sim::v2::OK);
      client->registration_.set_adapter_id(tick.adapter_id());
      client->registration_.set_session_id(tick.header().session_id());
      client->registration_.set_epoch(tick.header().epoch());
      auto* observation = client->registration_.mutable_layout()->add_observations();
      observation->set_name("position");
      observation->set_dimension(1);
      auto* motor_a = client->registration_.mutable_layout()->add_commands();
      motor_a->set_name("motor_a");
      motor_a->set_dimension(1);
      motor_a->set_has_ctrl_range(true);
      motor_a->set_ctrl_min(-1.0);
      motor_a->set_ctrl_max(1.0);
      auto* motor_b = client->registration_.mutable_layout()->add_commands();
      motor_b->set_name("motor_b");
      motor_b->set_dimension(1);
      client->registration_.mutable_layout()->set_layout_hash(
          mujoco_sim::protocol::ComputeLayoutHash(client->registration_.layout()));
      client->started_ = true;
      client->activated_ = activated;
      client->registered_ = false;
      client->accepting_ticks_ = activated;
      client->monitor_ecal_ = false;
    }
    {
      std::lock_guard lock(client->callback_state_->mutex);
      client->callback_state_->client = client;
      client->callback_state_->enabled = true;
    }
  }

  static void HandleTick(
      SyncAdapterClient* client, const mujoco_sim::v2::Tick& tick,
      const std::function<void(const mujoco_sim::v2::AdapterHeartbeat&)>&
          publish_heartbeat,
      const std::function<void(const mujoco_sim::v2::AdapterCommit&)>&
          publish_commit) {
    client->HandleTick(tick, publish_heartbeat, publish_commit);
  }

  static void SetFirstCommandDimension(SyncAdapterClient* client,
                                       std::uint32_t dimension) {
    client->registration_.mutable_layout()->mutable_commands(0)->set_dimension(
        dimension);
  }

  static void HandleTickBytes(SyncAdapterClient* client,
                              const mujoco_sim::v2::Tick& tick) {
    const std::string bytes = tick.SerializeAsString();
    client->HandleTickBytes(bytes.data(), bytes.size());
  }

  static void HandleStatusBytes(SyncAdapterClient* client,
                                const mujoco_sim::v2::ServerStatus& status) {
    const std::string bytes = status.SerializeAsString();
    client->HandleStatusBytes(bytes.data(), bytes.size());
  }

  static bool CommitWith(
      SyncAdapterClient* client, const mujoco_sim::v2::Tick& tick,
      const std::vector<CommandValue>& commands,
      const std::function<bool(const mujoco_sim::v2::AdapterCommit&)>& publish,
      std::string* error) {
    return client->CommitWith(tick, commands, publish, error);
  }

  static bool CommitSerializedWith(
      SyncAdapterClient* client, const mujoco_sim::v2::Tick& tick,
      const std::vector<CommandValue>& commands,
      const std::function<bool(const mujoco_sim::v2::AdapterCommit&)>& publish,
      std::string* error) {
    std::lock_guard lifecycle_lock(client->lifecycle_mutex_);
    return client->CommitWith(tick, commands, publish, error);
  }

  static bool IsExpectedTick(SyncAdapterClient* client,
                             const mujoco_sim::v2::Tick& tick) {
    return client->IsExpectedTick(tick);
  }

  static bool WaitBeforeActivation(SyncAdapterClient* client,
                                   std::chrono::milliseconds delay) {
    return client->WaitBeforeActivation(delay);
  }

  static void PrepareStartCompletion(SyncAdapterClient* client) {
    client->stop_requested_ = false;
    {
      std::lock_guard lock(client->mutex_);
      client->started_ = false;
      client->registered_ = true;
      client->monitor_ecal_ = false;
    }
    {
      std::lock_guard lock(client->callback_state_->mutex);
      client->callback_state_->client = client;
      client->callback_state_->enabled = true;
    }
  }

  static bool FinishStart(SyncAdapterClient* client,
                          const std::function<void()>& unregister,
                          std::string* error) {
    std::lock_guard lifecycle_lock(client->lifecycle_mutex_);
    return client->FinishStart(unregister, error);
  }

  static bool IsStarted(SyncAdapterClient* client) {
    std::lock_guard lock(client->mutex_);
    return client->started_;
  }

  static bool ActivateWith(
      SyncAdapterClient* client,
      const std::function<bool(
          const mujoco_sim::v2::ActivateAdapterRequest&,
          mujoco_sim::v2::ActivateAdapterResponse*, std::string*)>& call,
      std::chrono::milliseconds discovery_delay, std::string* error) {
    std::lock_guard lifecycle_lock(client->lifecycle_mutex_);
    return client->ActivateWith(call, discovery_delay, error);
  }

  static void SetRegistered(SyncAdapterClient* client, bool registered) {
    std::lock_guard lock(client->mutex_);
    client->registered_ = registered;
  }

  static bool IsRegistered(SyncAdapterClient* client) {
    std::lock_guard lock(client->mutex_);
    return client->registered_;
  }

  static bool WaitInProgress(SyncAdapterClient* client) {
    std::lock_guard lock(client->mutex_);
    return client->wait_in_progress_;
  }

  static void HoldStoppedTransitionBeforeNotify(
      SyncAdapterClient* client, std::promise<void>* transitioned,
      std::shared_future<void> release) {
    std::unique_lock lock(client->mutex_);
    client->MarkStoppedLocked();
    transitioned->set_value();
    release.wait();
    lock.unlock();
    client->condition_.notify_all();
  }

  static bool UnregisterWith(
      SyncAdapterClient* client,
      const std::function<bool(
          const mujoco_sim::v2::UnregisterAdapterRequest&,
          mujoco_sim::v2::UnregisterAdapterResponse*, std::string*)>& call,
      std::string* error) {
    return client->UnregisterWith(call, error);
  }

  static bool UnregisterSerializedWith(
      SyncAdapterClient* client,
      const std::function<bool(
          const mujoco_sim::v2::UnregisterAdapterRequest&,
          mujoco_sim::v2::UnregisterAdapterResponse*, std::string*)>& call,
      std::string* error) {
    std::lock_guard lifecycle_lock(client->lifecycle_mutex_);
    return client->UnregisterWith(call, error);
  }

  static void AddLifecycleWarning(SyncAdapterClient* client,
                                  const std::string& warning) {
    client->AddLifecycleWarning(warning);
  }

  static std::function<bool()> MakePendingCallback(
      SyncAdapterClient* client) {
    const auto state = client->callback_state_;
    return [state] {
      SyncAdapterClient::CallbackLease callback(state);
      return callback.client() != nullptr;
    };
  }

  static void HoldCallback(SyncAdapterClient* client,
                           std::promise<void>* entered,
                           std::shared_future<void> release) {
    SyncAdapterClient::CallbackLease callback(client->callback_state_);
    entered->set_value();
    release.wait();
  }

  static void StopTickReception(SyncAdapterClient* client) {
    client->StopTickReception();
  }

  static bool WaitForTickReceptionDisabled(
      SyncAdapterClient* client, std::chrono::milliseconds timeout) {
    const auto state = client->callback_state_;
    std::unique_lock lock(state->mutex);
    return state->condition.wait_for(lock, timeout, [&] {
      return !state->enabled && state->client == nullptr;
    });
  }

  static bool PublishSerialized(SyncAdapterClient* client,
                                const std::function<bool()>& publish) {
    return client->PublishSerialized(publish);
  }
};

}  // namespace mujoco_sim::adapter_sdk

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::CommandValue;
using mujoco_sim::adapter_sdk::SyncAdapterClient;
using mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer;
using mujoco_sim::adapter_sdk::TickWaitResult;

mujoco_sim::adapter_sdk::AdapterOptions MakeOptions() {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = "default";
  options.adapter_id = "arm";
  options.command_names = {"motor_a", "motor_b"};
  return options;
}

mujoco_sim::v2::Tick MakeTick(
    std::uint64_t tick_id,
    mujoco_sim::v2::ControllerState state = mujoco_sim::v2::ACTIVE) {
  mujoco_sim::v2::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(tick_id);
  tick.mutable_header()->set_timestamp_unix_nanos(1000 + tick_id);
  tick.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  tick.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  tick.set_adapter_id("arm");
  tick.set_controller_state(state);
  mujoco_sim::v2::ResourceLayout layout;
  auto* observation_descriptor = layout.add_observations();
  observation_descriptor->set_name("position");
  observation_descriptor->set_dimension(1);
  auto* observation = tick.add_observations();
  observation->set_name("position");
  observation->add_values(static_cast<double>(tick_id));

  auto* motor_a = layout.add_commands();
  motor_a->set_name("motor_a");
  motor_a->set_dimension(1);
  motor_a->set_has_ctrl_range(true);
  motor_a->set_ctrl_min(-1.0);
  motor_a->set_ctrl_max(1.0);
  auto* motor_b = layout.add_commands();
  motor_b->set_name("motor_b");
  motor_b->set_dimension(1);
  tick.set_layout_hash(mujoco_sim::protocol::ComputeLayoutHash(layout));
  return tick;
}

const std::vector<CommandValue>& ValidCommands() {
  static const std::vector<CommandValue> commands{
      {"motor_a", 0.25}, {"motor_b", -0.5}};
  return commands;
}

struct Publications {
  std::mutex mutex;
  std::vector<mujoco_sim::v2::AdapterHeartbeat> heartbeats;
  std::vector<mujoco_sim::v2::AdapterCommit> commits;

  void AdapterHeartbeat(const mujoco_sim::v2::AdapterHeartbeat& heartbeat) {
    std::lock_guard lock(mutex);
    heartbeats.push_back(heartbeat);
  }

  bool Commit(const mujoco_sim::v2::AdapterCommit& commit) {
    std::lock_guard lock(mutex);
    commits.push_back(commit);
    return true;
  }
};

void Deliver(SyncAdapterClient* client, const mujoco_sim::v2::Tick& tick,
             Publications* publications) {
  SyncAdapterClientTestPeer::HandleTick(
      client, tick,
      [publications](const auto& heartbeat) {
        publications->AdapterHeartbeat(heartbeat);
      },
      [publications](const auto& commit) { publications->Commit(commit); });
}

TEST(SyncAdapterClientTest, RequiresExplicitActivationBeforeWaiting) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick, false);

  mujoco_sim::v2::Tick output;
  std::string error;
  EXPECT_EQ(client.WaitForTick(0ms, &output, &error), TickWaitResult::kError);
  EXPECT_EQ(error, "adapter client must be activated before waiting");

  SyncAdapterClient never_started(MakeOptions());
  EXPECT_FALSE(never_started.Activate(&error));
  EXPECT_EQ(error, "adapter client must be started before activation");
}

TEST(SyncAdapterClientTest, WaitTimesOutTransactionallyAndRejectsBadArguments) {
  SyncAdapterClient client(MakeOptions());
  const auto session = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, session);

  mujoco_sim::v2::Tick output;
  output.set_adapter_id("unchanged");
  std::string error = "old";
  EXPECT_EQ(client.WaitForTick(1ms, &output, &error),
            TickWaitResult::kTimeout);
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(output.adapter_id(), "unchanged");

  EXPECT_EQ(client.WaitForTick(-1ms, &output, &error),
            TickWaitResult::kError);
  EXPECT_EQ(error, "tick wait timeout must not be negative");
  EXPECT_EQ(client.WaitForTick(0ms, nullptr, &error), TickWaitResult::kError);
  EXPECT_FALSE(client.Commit(session, ValidCommands(), nullptr));
}

TEST(SyncAdapterClientTest, EnforcesOneOutstandingTick) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  Publications publications;
  Deliver(&client, tick, &publications);

  mujoco_sim::v2::Tick received;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kReceived);
  EXPECT_EQ(received.header().tick_id(), 9U);
  EXPECT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kError);
  EXPECT_EQ(error, "the previously received Tick has not been committed");

  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(),
      [&publications](const auto& commit) {
        return publications.Commit(commit);
      },
      &error))
      << error;
  EXPECT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kTimeout);
}

TEST(SyncAdapterClientTest, DuplicateTickIsDeliveredOnceAndResendsCommit) {
  SyncAdapterClient client(MakeOptions());
  auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  Publications publications;

  Deliver(&client, tick, &publications);
  auto timestamp_only_duplicate = tick;
  timestamp_only_duplicate.mutable_header()->set_timestamp_unix_nanos(9999);
  Deliver(&client, timestamp_only_duplicate, &publications);

  mujoco_sim::v2::Tick received;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kReceived);
  EXPECT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kError);
  {
    std::lock_guard lock(publications.mutex);
    EXPECT_EQ(publications.heartbeats.size(), 2U);
    EXPECT_TRUE(publications.commits.empty());
  }

  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(),
      [&publications](const auto& commit) {
        return publications.Commit(commit);
      },
      &error))
      << error;
  Deliver(&client, timestamp_only_duplicate, &publications);
  {
    std::lock_guard lock(publications.mutex);
    ASSERT_EQ(publications.commits.size(), 2U);
    EXPECT_EQ(publications.commits[0].SerializeAsString(),
              publications.commits[1].SerializeAsString());
    EXPECT_EQ(publications.heartbeats.size(), 3U);
  }

  auto conflicting = tick;
  conflicting.mutable_observations(0)->set_values(0, 123.0);
  Deliver(&client, conflicting, &publications);
  Deliver(&client, MakeTick(8), &publications);
  {
    std::lock_guard lock(publications.mutex);
    EXPECT_EQ(publications.heartbeats.size(), 3U);
    EXPECT_EQ(publications.commits.size(), 2U);
  }
  EXPECT_NE(client.lifecycle_warning().find("conflicting payload for Tick 9"),
            std::string::npos);
}

TEST(SyncAdapterClientTest, FailedPublishIsReportedAndCommitCanBeRetried) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  Publications publications;
  Deliver(&client, tick, &publications);

  mujoco_sim::v2::Tick received;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kReceived);
  int publish_attempts = 0;
  ASSERT_FALSE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(),
      [&publish_attempts](const auto&) {
        ++publish_attempts;
        return false;
      },
      &error));
  EXPECT_EQ(error, "failed to publish Commit");
  EXPECT_EQ(publish_attempts, 1);

  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(),
      [&publications](const auto& commit) {
        return publications.Commit(commit);
      },
      &error))
      << error;

  std::lock_guard lock(publications.mutex);
  ASSERT_EQ(publications.commits.size(), 1U);
  EXPECT_EQ(publications.commits.front().header().tick_id(), 9U);
  EXPECT_EQ(publications.commits.front().commands_size(), 2);
}

TEST(SyncAdapterClientTest, TimedOutTicksUseLatestValueMailbox) {
  SyncAdapterClient client(MakeOptions());
  const auto tick9 = MakeTick(9, mujoco_sim::v2::TIMED_OUT);
  SyncAdapterClientTestPeer::Configure(&client, tick9);
  Publications publications;
  Deliver(&client, tick9, &publications);

  mujoco_sim::v2::Tick received;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kReceived);
  Deliver(&client, MakeTick(10, mujoco_sim::v2::TIMED_OUT), &publications);
  Deliver(&client, MakeTick(11, mujoco_sim::v2::TIMED_OUT), &publications);
  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(), [](const auto&) { return true; },
      &error))
      << error;

  ASSERT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kReceived);
  EXPECT_EQ(received.header().tick_id(), 11U);
  EXPECT_EQ(received.controller_state(), mujoco_sim::v2::TIMED_OUT);
  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(), [](const auto&) { return true; },
      &error))
      << error;
}

TEST(SyncAdapterClientTest, CommitCanonicalizesOrderAndIsIdempotent) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  Publications publications;
  Deliver(&client, tick, &publications);

  mujoco_sim::v2::Tick received;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kReceived);
  const std::vector<CommandValue> reversed{{"motor_b", -0.5},
                                           {"motor_a", 0.25}};
  const auto publish = [&publications](const auto& commit) {
    return publications.Commit(commit);
  };
  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, reversed, publish, &error))
      << error;
  ASSERT_TRUE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, ValidCommands(), publish, &error))
      << error;

  {
    std::lock_guard lock(publications.mutex);
    ASSERT_EQ(publications.commits.size(), 2U);
    ASSERT_EQ(publications.commits[0].commands_size(), 2);
    EXPECT_EQ(publications.commits[0].commands(0).name(), "motor_a");
    EXPECT_EQ(publications.commits[0].commands(1).name(), "motor_b");
    EXPECT_EQ(publications.commits[0].SerializeAsString(),
              publications.commits[1].SerializeAsString());
  }

  const std::vector<CommandValue> changed{{"motor_a", 0.5},
                                          {"motor_b", -0.5}};
  EXPECT_FALSE(SyncAdapterClientTestPeer::CommitWith(
      &client, received, changed, publish, &error));
  EXPECT_EQ(error, "a different Commit was already sent for this Tick");
}

TEST(SyncAdapterClientTest, InvalidCommandCanBeCorrectedBeforeCommit) {
  const auto run_case = [](std::vector<CommandValue> invalid,
                           const std::string& expected_error,
                           bool non_scalar = false) {
    auto tick = MakeTick(9);
    SyncAdapterClient client(MakeOptions());
    SyncAdapterClientTestPeer::Configure(&client, tick);
    if (non_scalar) {
      SyncAdapterClientTestPeer::SetFirstCommandDimension(&client, 2);
    }
    Publications publications;
    Deliver(&client, tick, &publications);
    mujoco_sim::v2::Tick received;
    std::string error;
    EXPECT_EQ(client.WaitForTick(0ms, &received, &error),
              TickWaitResult::kReceived);
    EXPECT_FALSE(SyncAdapterClientTestPeer::CommitWith(
        &client, received, invalid, [](const auto&) { return true; }, &error));
    EXPECT_NE(error.find(expected_error), std::string::npos) << error;
    if (!non_scalar) {
      EXPECT_TRUE(SyncAdapterClientTestPeer::CommitWith(
          &client, received, ValidCommands(), [](const auto&) { return true; },
          &error))
          << error;
    }
  };

  run_case({}, "missing actuator command");
  run_case({{"", 0.1}, {"motor_b", 0.0}}, "name must not be empty");
  run_case({{"motor_a", 0.1}}, "missing actuator command");
  run_case({{"motor_a", 0.1}, {"motor_a", 0.2}, {"motor_b", 0.0}},
           "duplicate actuator command");
  run_case({{"motor_a", 0.1}, {"motor_b", 0.0}, {"other", 0.0}},
           "unknown actuator command");
  run_case({{"motor_a", std::numeric_limits<double>::quiet_NaN()},
            {"motor_b", 0.0}},
           "must be finite");
  run_case({{"motor_a", std::numeric_limits<double>::infinity()},
            {"motor_b", 0.0}},
           "must be finite");
  run_case({{"motor_a", 1.1}, {"motor_b", 0.0}}, "outside ctrlrange");
  run_case(ValidCommands(), "unsupported non-scalar", true);
}

TEST(SyncAdapterClientTest, RejectsTickOutsideRegistrationIdentityAndLayout) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  EXPECT_TRUE(SyncAdapterClientTestPeer::IsExpectedTick(&client, tick));

  auto changed = tick;
  changed.mutable_header()->set_server_name("other");
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsExpectedTick(&client, changed));
  changed = tick;
  changed.mutable_header()->set_session_id(5);
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsExpectedTick(&client, changed));
  changed = tick;
  changed.mutable_header()->set_epoch(3);
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsExpectedTick(&client, changed));
  changed = tick;
  changed.set_layout_hash(std::string(mujoco_sim::protocol::kLayoutHashSize,
                                      '\xff'));
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsExpectedTick(&client, changed));
  changed = tick;
  changed.set_controller_state(mujoco_sim::v2::PROVISIONAL);
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsExpectedTick(&client, changed));
}

TEST(SyncAdapterClientTest, LayoutHashMismatchIsATerminalProtocolError) {
  SyncAdapterClient client(MakeOptions());
  auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  tick.set_layout_hash("short");

  SyncAdapterClientTestPeer::HandleTickBytes(&client, tick);

  mujoco_sim::v2::Tick received;
  std::string error;
  EXPECT_EQ(client.WaitForTick(0ms, &received, &error),
            TickWaitResult::kError);
  EXPECT_NE(error.find("event=layout_hash_mismatch"), std::string::npos);
  EXPECT_EQ(client.layout_hash_mismatches(), 1U);
}

TEST(SyncAdapterClientTest, ConcurrentWaitIsRejectedAndTickWakesFirstWaiter) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  Publications publications;

  mujoco_sim::v2::Tick first_tick;
  std::string first_error;
  auto first = std::async(std::launch::async, [&] {
    return client.WaitForTick(std::chrono::milliseconds::max(), &first_tick,
                              &first_error);
  });
  std::this_thread::sleep_for(20ms);

  mujoco_sim::v2::Tick second_tick;
  std::string second_error;
  EXPECT_EQ(client.WaitForTick(0ms, &second_tick, &second_error),
            TickWaitResult::kError);
  EXPECT_EQ(second_error, "another thread is already waiting for a Tick");

  Deliver(&client, tick, &publications);
  ASSERT_EQ(first.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(first.get(), TickWaitResult::kReceived);
  EXPECT_EQ(first_tick.header().tick_id(), 9U);
}

TEST(SyncAdapterClientTest, StopWakesWaiter) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9));
  mujoco_sim::v2::Tick output;
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForTick(std::chrono::milliseconds::max(), &output,
                              &error);
  });
  std::this_thread::sleep_for(20ms);
  client.Stop();
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), TickWaitResult::kStopped);
}

TEST(SyncAdapterClientTest,
     ServerStoppedTransitionIsAtomicWithConcurrentWait) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9));
  std::promise<void> transitioned_promise;
  auto transitioned = transitioned_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  auto stopping = std::async(std::launch::async, [&] {
    SyncAdapterClientTestPeer::HoldStoppedTransitionBeforeNotify(
        &client, &transitioned_promise, release);
  });
  if (transitioned.wait_for(1s) != std::future_status::ready) {
    release_promise.set_value();
    stopping.wait();
    FAIL() << "stopped transition did not acquire the client mutex";
    return;
  }

  mujoco_sim::v2::Tick output;
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForTick(std::chrono::milliseconds::max(), &output,
                              &error);
  });
  EXPECT_EQ(waiting.wait_for(20ms), std::future_status::timeout);
  release_promise.set_value();
  stopping.get();
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), TickWaitResult::kStopped);
  EXPECT_TRUE(error.empty());
}

TEST(SyncAdapterClientTest, MatchingServerErrorWakesWaiterWithReason) {
  SyncAdapterClient client(MakeOptions());
  const auto session = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, session);
  mujoco_sim::v2::Tick output;
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForTick(std::chrono::milliseconds::max(), &output,
                              &error);
  });
  std::this_thread::sleep_for(20ms);

  mujoco_sim::v2::ServerStatus status;
  status.mutable_header()->set_server_name("default");
  status.mutable_header()->set_epoch(2);
  status.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  status.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  status.set_state(mujoco_sim::v2::RUNNING);
  auto* controller = status.add_adapters();
  controller->set_adapter_id("arm");
  controller->set_role(mujoco_sim::v2::CONTROLLER);
  controller->set_session_id(4);
  controller->set_state(mujoco_sim::v2::ERROR);
  controller->set_message("invalid actuator command");
  SyncAdapterClientTestPeer::HandleStatusBytes(&client, status);

  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), TickWaitResult::kError);
  EXPECT_EQ(error, "invalid actuator command");
  EXPECT_EQ(client.lifecycle_warning(), "invalid actuator command");
}

TEST(SyncAdapterClientTest, SessionReplacementWakesWaiterWithReason) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9));
  mujoco_sim::v2::Tick output;
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForTick(std::chrono::milliseconds::max(), &output,
                              &error);
  });
  std::this_thread::sleep_for(20ms);

  mujoco_sim::v2::ServerStatus status;
  status.mutable_header()->set_server_name("default");
  status.mutable_header()->set_epoch(2);
  status.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  status.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  status.set_state(mujoco_sim::v2::RUNNING);
  auto* controller = status.add_adapters();
  controller->set_adapter_id("arm");
  controller->set_role(mujoco_sim::v2::CONTROLLER);
  controller->set_session_id(5);
  controller->set_state(mujoco_sim::v2::ACTIVE);
  SyncAdapterClientTestPeer::HandleStatusBytes(&client, status);

  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), TickWaitResult::kError);
  EXPECT_EQ(error, "server replaced adapter session");
}

TEST(SyncAdapterClientTest, ServerTerminalStatesWakeInfiniteWait) {
  const auto make_status = [](mujoco_sim::v2::ServerState state,
                              const std::string& message = {}) {
    mujoco_sim::v2::ServerStatus status;
    status.mutable_header()->set_server_name("default");
    status.mutable_header()->set_epoch(2);
    status.mutable_header()->set_protocol_major(
        mujoco_sim::protocol::kProtocolMajor);
    status.mutable_header()->set_protocol_minor(
        mujoco_sim::protocol::kProtocolMinor);
    status.set_state(state);
    status.set_message(message);
    return status;
  };

  SyncAdapterClient stopped_client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&stopped_client, MakeTick(9));
  mujoco_sim::v2::Tick stopped_output;
  std::string stopped_error;
  auto stopped_wait = std::async(std::launch::async, [&] {
    return stopped_client.WaitForTick(std::chrono::milliseconds::max(),
                                      &stopped_output, &stopped_error);
  });
  std::this_thread::sleep_for(20ms);
  SyncAdapterClientTestPeer::HandleStatusBytes(
      &stopped_client, make_status(mujoco_sim::v2::STOPPED));
  ASSERT_EQ(stopped_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(stopped_wait.get(), TickWaitResult::kStopped);
  EXPECT_TRUE(stopped_error.empty());

  SyncAdapterClient stopped_before_wait(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&stopped_before_wait, MakeTick(9));
  SyncAdapterClientTestPeer::HandleStatusBytes(
      &stopped_before_wait, make_status(mujoco_sim::v2::STOPPED));
  EXPECT_EQ(stopped_before_wait.WaitForTick(0ms, &stopped_output,
                                            &stopped_error),
            TickWaitResult::kStopped);
  EXPECT_TRUE(stopped_error.empty());

  SyncAdapterClient error_client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&error_client, MakeTick(9));
  mujoco_sim::v2::Tick error_output;
  std::string error_message;
  auto error_wait = std::async(std::launch::async, [&] {
    return error_client.WaitForTick(std::chrono::milliseconds::max(),
                                    &error_output, &error_message);
  });
  std::this_thread::sleep_for(20ms);
  SyncAdapterClientTestPeer::HandleStatusBytes(
      &error_client,
      make_status(mujoco_sim::v2::ERROR_PAUSED, "fatal simulation error"));
  ASSERT_EQ(error_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(error_wait.get(), TickWaitResult::kError);
  EXPECT_EQ(error_message, "fatal simulation error");
}

TEST(SyncAdapterClientTest, ConcurrentActivateCallsAreSerialized) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9), false);
  SyncAdapterClientTestPeer::SetRegistered(&client, true);
  std::atomic<int> calls{0};
  std::promise<void> first_entered_promise;
  auto first_entered = first_entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  const auto call = [&](const auto&, auto* response, std::string*) {
    if (++calls == 1) {
      first_entered_promise.set_value();
      release.wait();
    }
    response->set_code(mujoco_sim::v2::OK);
    return true;
  };

  std::string first_error;
  auto first = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::ActivateWith(&client, call, 0ms,
                                                    &first_error);
  });
  ASSERT_EQ(first_entered.wait_for(1s), std::future_status::ready);
  std::string second_error;
  auto second = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::ActivateWith(&client, call, 0ms,
                                                    &second_error);
  });
  EXPECT_EQ(second.wait_for(100ms), std::future_status::timeout);
  release_promise.set_value();

  EXPECT_TRUE(first.get()) << first_error;
  EXPECT_FALSE(second.get());
  EXPECT_EQ(second_error, "adapter client has already been activated");
  EXPECT_EQ(calls.load(), 1);
  SyncAdapterClientTestPeer::SetRegistered(&client, false);
}

TEST(SyncAdapterClientTest, StopWinsRaceWithSuccessfulActivateResponse) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9), false);
  SyncAdapterClientTestPeer::SetRegistered(&client, true);
  std::promise<void> call_entered_promise;
  auto call_entered = call_entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  const auto call = [&](const auto&, auto* response, std::string*) {
    call_entered_promise.set_value();
    release.wait();
    response->set_code(mujoco_sim::v2::OK);
    return true;
  };

  std::string error;
  auto activating = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::ActivateWith(&client, call, 0ms, &error);
  });
  ASSERT_EQ(call_entered.wait_for(1s), std::future_status::ready);
  client.Stop();
  release_promise.set_value();
  EXPECT_FALSE(activating.get());
  EXPECT_EQ(error, "adapter activation was cancelled");
  SyncAdapterClientTestPeer::SetRegistered(&client, false);
}

TEST(SyncAdapterClientTest, PublisherActionsAreSerialized) {
  SyncAdapterClient client(MakeOptions());
  std::promise<void> first_entered_promise;
  auto first_entered = first_entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();

  auto first = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::PublishSerialized(&client, [&] {
      first_entered_promise.set_value();
      release.wait();
      return true;
    });
  });
  ASSERT_EQ(first_entered.wait_for(1s), std::future_status::ready);
  auto second = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::PublishSerialized(
        &client, [] { return true; });
  });
  EXPECT_EQ(second.wait_for(100ms), std::future_status::timeout);
  release_promise.set_value();
  EXPECT_TRUE(first.get());
  EXPECT_TRUE(second.get());
}

TEST(SyncAdapterClientTest, StopDoesNotBlockBehindCommitAndUnregisterCleansUp) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  SyncAdapterClientTestPeer::SetRegistered(&client, true);
  Publications publications;
  Deliver(&client, tick, &publications);
  mujoco_sim::v2::Tick received;
  std::string wait_error;
  ASSERT_EQ(client.WaitForTick(0ms, &received, &wait_error),
            TickWaitResult::kReceived);

  std::promise<void> publish_entered_promise;
  auto publish_entered = publish_entered_promise.get_future();
  std::promise<void> release_publish_promise;
  auto release_publish = release_publish_promise.get_future().share();
  std::string commit_error;
  auto committing = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::CommitSerializedWith(
        &client, received, ValidCommands(),
        [&](const auto&) {
          publish_entered_promise.set_value();
          release_publish.wait();
          return true;
        },
        &commit_error);
  });
  ASSERT_EQ(publish_entered.wait_for(1s), std::future_status::ready);

  std::atomic<bool> unregister_called{false};
  std::string unregister_error;
  auto unregistering = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::UnregisterSerializedWith(
        &client,
        [&](const auto&, auto* response, std::string*) {
          unregister_called = true;
          response->set_code(mujoco_sim::v2::OK);
          return true;
        },
        &unregister_error);
  });
  EXPECT_EQ(unregistering.wait_for(100ms), std::future_status::timeout);
  EXPECT_FALSE(unregister_called.load());

  const auto stop_started = std::chrono::steady_clock::now();
  client.Stop();
  EXPECT_LT(std::chrono::steady_clock::now() - stop_started, 100ms);
  release_publish_promise.set_value();

  EXPECT_TRUE(committing.get()) << commit_error;
  EXPECT_TRUE(unregistering.get()) << unregister_error;
  EXPECT_TRUE(unregister_called.load());
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsRegistered(&client));
}

TEST(SyncAdapterClientTest, FailedUnregisterRemainsRetryableAndWarningsMerge) {
  SyncAdapterClient client(MakeOptions());
  const auto tick = MakeTick(9);
  SyncAdapterClientTestPeer::Configure(&client, tick);
  SyncAdapterClientTestPeer::SetRegistered(&client, true);
  int calls = 0;
  const auto call = [&](const auto& request, auto* response,
                        std::string* error) {
    ++calls;
    EXPECT_EQ(request.adapter_id(), "arm");
    EXPECT_EQ(request.session_id(), 4U);
    if (calls == 1) {
      *error = "transport unavailable";
      return false;
    }
    response->set_code(mujoco_sim::v2::NOT_FOUND);
    return true;
  };

  std::string error;
  EXPECT_FALSE(SyncAdapterClientTestPeer::UnregisterWith(&client, call,
                                                          &error));
  EXPECT_TRUE(SyncAdapterClientTestPeer::IsRegistered(&client));
  SyncAdapterClientTestPeer::AddLifecycleWarning(&client, "first");
  SyncAdapterClientTestPeer::AddLifecycleWarning(&client, "second");
  EXPECT_EQ(client.lifecycle_warning(), "first; second");

  EXPECT_TRUE(SyncAdapterClientTestPeer::UnregisterWith(&client, call,
                                                         &error));
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsRegistered(&client));
  EXPECT_EQ(calls, 2);
}

TEST(SyncAdapterClientTest, SuccessfulUnregisterWakesConcurrentWaitAsStopped) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9));
  SyncAdapterClientTestPeer::SetRegistered(&client, true);
  mujoco_sim::v2::Tick output;
  std::string wait_error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForTick(std::chrono::milliseconds::max(), &output,
                              &wait_error);
  });
  for (int i = 0;
       i < 100 && !SyncAdapterClientTestPeer::WaitInProgress(&client); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  if (!SyncAdapterClientTestPeer::WaitInProgress(&client)) {
    client.Stop();
    waiting.wait();
    FAIL() << "wait did not enter the blocking state";
    return;
  }

  std::string unregister_error;
  const bool unregistered = SyncAdapterClientTestPeer::UnregisterWith(
      &client,
      [](const auto&, auto* response, std::string*) {
        response->set_code(mujoco_sim::v2::OK);
        return true;
      },
      &unregister_error);
  if (!unregistered) {
    client.Stop();
    waiting.wait();
    FAIL() << unregister_error;
    return;
  }
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), TickWaitResult::kStopped);
  EXPECT_TRUE(wait_error.empty());
}

TEST(SyncAdapterClientTest, StopCancelsStartAndCleansRegisteredSession) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::PrepareStartCompletion(&client);
  auto delayed_callback =
      SyncAdapterClientTestPeer::MakePendingCallback(&client);
  ASSERT_TRUE(delayed_callback());
  client.Stop();

  bool unregistered = false;
  std::string error;
  EXPECT_FALSE(SyncAdapterClientTestPeer::FinishStart(
      &client,
      [&] {
        unregistered = true;
        SyncAdapterClientTestPeer::SetRegistered(&client, false);
      },
      &error));
  EXPECT_EQ(error, "adapter start was cancelled");
  EXPECT_TRUE(unregistered);
  EXPECT_FALSE(SyncAdapterClientTestPeer::IsStarted(&client));
  EXPECT_FALSE(delayed_callback());
}

TEST(SyncAdapterClientTest, StopInterruptsPreActivationWait) {
  SyncAdapterClient client(MakeOptions());
  const auto started_at = std::chrono::steady_clock::now();
  auto waiting = std::async(std::launch::async, [&] {
    return SyncAdapterClientTestPeer::WaitBeforeActivation(&client, 5s);
  });
  std::this_thread::sleep_for(20ms);
  client.Stop();

  ASSERT_EQ(waiting.wait_for(500ms), std::future_status::ready);
  EXPECT_FALSE(waiting.get());
  EXPECT_LT(std::chrono::steady_clock::now() - started_at, 1s);
}

TEST(SyncAdapterClientTest, StopReceptionWaitsForInflightCallback) {
  SyncAdapterClient client(MakeOptions());
  SyncAdapterClientTestPeer::Configure(&client, MakeTick(9));
  std::promise<void> callback_entered_promise;
  auto callback_entered = callback_entered_promise.get_future();
  std::promise<void> release_callback_promise;
  auto release_callback = release_callback_promise.get_future().share();
  auto callback = std::async(std::launch::async, [&] {
    SyncAdapterClientTestPeer::HoldCallback(
        &client, &callback_entered_promise, release_callback);
  });
  ASSERT_EQ(callback_entered.wait_for(1s), std::future_status::ready);

  auto stopping = std::async(std::launch::async, [&] {
    SyncAdapterClientTestPeer::StopTickReception(&client);
  });
  EXPECT_TRUE(SyncAdapterClientTestPeer::WaitForTickReceptionDisabled(
      &client, 1s));
  EXPECT_EQ(stopping.wait_for(0ms), std::future_status::timeout);
  release_callback_promise.set_value();
  callback.get();
  ASSERT_EQ(stopping.wait_for(1s), std::future_status::ready);
  stopping.get();
}

TEST(SyncAdapterClientTest, DelayedCallbackCannotReachDestroyedClient) {
  std::function<bool()> delayed_callback;
  {
    auto client = std::make_unique<SyncAdapterClient>(MakeOptions());
    SyncAdapterClientTestPeer::Configure(client.get(), MakeTick(9));
    delayed_callback =
        SyncAdapterClientTestPeer::MakePendingCallback(client.get());
    EXPECT_TRUE(delayed_callback());
  }
  EXPECT_FALSE(delayed_callback());
}

}  // namespace
