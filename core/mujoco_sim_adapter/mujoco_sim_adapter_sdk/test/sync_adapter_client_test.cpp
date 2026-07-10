#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <mutex>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::adapter_sdk {

class SyncAdapterClientTestPeer {
 public:
  static void SetHandler(SyncAdapterClient* client, TickHandler handler) {
    std::lock_guard lock(client->mutex_);
    client->handler_ = std::move(handler);
    client->accepting_ticks_ = true;
  }

  static void SetRegistration(SyncAdapterClient* client,
                              const mujoco_sim::v1::Tick& tick) {
    client->registration_.set_code(mujoco_sim::v1::OK);
    client->registration_.set_controller_id(tick.controller_id());
    client->registration_.set_session_id(tick.header().session_id());
    client->registration_.set_epoch(tick.header().epoch());
    client->registration_.set_effective_protocol_major(
        tick.header().protocol_major());
    client->registration_.set_effective_protocol_minor(
        tick.header().protocol_minor());
    *client->registration_.mutable_layout() = tick.layout();
  }

  static void HandleTick(
      SyncAdapterClient* client, const mujoco_sim::v1::Tick& tick,
      const std::function<void(const mujoco_sim::v1::Heartbeat&)>&
          publish_heartbeat,
      const std::function<void(const mujoco_sim::v1::AdapterCommit&)>&
          publish_commit) {
    client->HandleTick(tick, publish_heartbeat, publish_commit);
  }

  static std::uint64_t HandledTicks(SyncAdapterClient* client) {
    std::lock_guard lock(client->mutex_);
    return client->handled_ticks_;
  }

  static void HandleTickBytes(SyncAdapterClient* client,
                              const mujoco_sim::v1::Tick& tick) {
    const std::string bytes = tick.SerializeAsString();
    client->HandleTickBytes(bytes.data(), bytes.size());
  }

  static void StopTickReception(SyncAdapterClient* client) {
    client->StopTickReception();
  }
};

}  // namespace mujoco_sim::adapter_sdk

namespace {

using namespace std::chrono_literals;

TEST(SyncAdapterClientTest, BuildsCommitFromTick) {
  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(9);
  tick.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  tick.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  tick.set_controller_id("arm");
  tick.set_controller_state(mujoco_sim::v1::ACTIVE);

  const auto commit = mujoco_sim::adapter_sdk::BuildCommit(
      tick, {mujoco_sim::adapter_sdk::CommandValue{"hinge_motor", 0.5}});
  EXPECT_EQ(commit.controller_id(), "arm");
  EXPECT_EQ(commit.header().tick_id(), 9);
  EXPECT_EQ(commit.status(), mujoco_sim::v1::COMMIT_OK);
  ASSERT_EQ(commit.commands_size(), 1);
  EXPECT_EQ(commit.commands(0).name(), "hinge_motor");
  EXPECT_DOUBLE_EQ(commit.commands(0).values(0), 0.5);
}

TEST(SyncAdapterClientTest, BuildsHeartbeatFromTick) {
  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(9);
  tick.set_controller_id("arm");

  const auto heartbeat = mujoco_sim::adapter_sdk::BuildHeartbeat(tick);
  EXPECT_EQ(heartbeat.controller_id(), "arm");
  EXPECT_EQ(heartbeat.header().session_id(), 4);
  EXPECT_EQ(heartbeat.header().tick_id(), 9);
}

TEST(SyncAdapterClientTest, ConcurrentDuplicateTickRunsHandlerOnceAndResendsCommit) {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.controller_id = "arm";
  options.command_names = {"hinge_motor"};
  mujoco_sim::adapter_sdk::SyncAdapterClient client(std::move(options));

  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(9);
  tick.set_controller_id("arm");

  std::atomic<int> handler_calls{0};
  std::promise<void> handler_entered_promise;
  auto handler_entered = handler_entered_promise.get_future();
  std::promise<void> release_handler_promise;
  auto release_handler = release_handler_promise.get_future().share();
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::SetHandler(
      &client, [&](const mujoco_sim::v1::Tick&) {
        const int invocation = ++handler_calls;
        if (invocation == 1) {
          handler_entered_promise.set_value();
          release_handler.wait();
        }
        return std::vector<mujoco_sim::adapter_sdk::CommandValue>{
            {"hinge_motor", static_cast<double>(invocation)}};
      });

  std::mutex published_mutex;
  std::vector<mujoco_sim::v1::Heartbeat> heartbeats;
  std::vector<mujoco_sim::v1::AdapterCommit> commits;
  const auto publish_heartbeat = [&](const mujoco_sim::v1::Heartbeat& value) {
    std::lock_guard lock(published_mutex);
    heartbeats.push_back(value);
  };
  const auto publish_commit = [&](const mujoco_sim::v1::AdapterCommit& value) {
    std::lock_guard lock(published_mutex);
    commits.push_back(value);
  };

  auto first = std::async(std::launch::async, [&] {
    mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTick(
        &client, tick, publish_heartbeat, publish_commit);
  });
  if (handler_entered.wait_for(1s) != std::future_status::ready) {
    release_handler_promise.set_value();
    first.wait();
    FAIL() << "first Tick did not enter the handler";
    return;
  }

  std::promise<void> duplicate_started_promise;
  auto duplicate_started = duplicate_started_promise.get_future();
  auto repeated_tick = tick;
  repeated_tick.mutable_header()->set_timestamp_unix_nanos(12345);
  auto duplicate = std::async(std::launch::async, [&] {
    duplicate_started_promise.set_value();
    mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTick(
        &client, repeated_tick, publish_heartbeat, publish_commit);
  });
  if (duplicate_started.wait_for(1s) != std::future_status::ready) {
    release_handler_promise.set_value();
    first.wait();
    duplicate.wait();
    FAIL() << "duplicate Tick callback did not start";
    return;
  }
  EXPECT_EQ(duplicate.wait_for(100ms), std::future_status::timeout);

  release_handler_promise.set_value();
  first.get();
  duplicate.get();

  EXPECT_EQ(handler_calls.load(), 1);
  EXPECT_EQ(
      mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandledTicks(&client),
      1);
  {
    std::lock_guard lock(published_mutex);
    ASSERT_EQ(heartbeats.size(), 2);
    ASSERT_EQ(commits.size(), 2);
    EXPECT_EQ(commits[0].SerializeAsString(), commits[1].SerializeAsString());
    ASSERT_EQ(commits[0].commands_size(), 1);
    EXPECT_DOUBLE_EQ(commits[0].commands(0).values(0), 1.0);
  }

  auto old_tick = tick;
  old_tick.mutable_header()->set_tick_id(8);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTick(
      &client, old_tick, publish_heartbeat, publish_commit);
  EXPECT_EQ(handler_calls.load(), 1);
  {
    std::lock_guard lock(published_mutex);
    EXPECT_EQ(heartbeats.size(), 2);
    EXPECT_EQ(commits.size(), 2);
  }

  auto conflicting_tick = tick;
  conflicting_tick.add_observations()->set_name("unexpected_change");
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTick(
      &client, conflicting_tick, publish_heartbeat, publish_commit);
  EXPECT_EQ(handler_calls.load(), 1);

  auto next_epoch_tick = tick;
  next_epoch_tick.mutable_header()->set_epoch(3);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTick(
      &client, next_epoch_tick, publish_heartbeat, publish_commit);
  EXPECT_EQ(handler_calls.load(), 2);
  {
    std::lock_guard lock(published_mutex);
    ASSERT_EQ(commits.size(), 3);
    EXPECT_EQ(commits.back().header().epoch(), 3U);
    EXPECT_DOUBLE_EQ(commits.back().commands(0).values(0), 2.0);
  }
}

TEST(SyncAdapterClientTest, RejectsTicksOutsideRegisteredIdentityAndLayout) {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = "default";
  options.controller_id = "arm";
  options.command_names = {"hinge_motor"};
  mujoco_sim::adapter_sdk::SyncAdapterClient client(std::move(options));

  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(9);
  tick.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  tick.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  tick.set_controller_id("arm");
  tick.set_controller_state(mujoco_sim::v1::ACTIVE);
  auto* command = tick.mutable_layout()->add_commands();
  command->set_name("hinge_motor");
  command->set_type(mujoco_sim::v1::COMMAND_ACTUATOR);
  command->set_dimension(1);

  std::atomic<int> handler_calls{0};
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::SetRegistration(&client,
                                                                      tick);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::SetHandler(
      &client, [&](const mujoco_sim::v1::Tick&) {
        ++handler_calls;
        return std::vector<mujoco_sim::adapter_sdk::CommandValue>{
            {"hinge_motor", 0.0}};
      });

  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      tick);
  EXPECT_EQ(handler_calls.load(), 1);

  auto invalid = tick;
  invalid.mutable_header()->set_session_id(5);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      invalid);
  invalid = tick;
  invalid.mutable_header()->set_epoch(3);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      invalid);
  invalid = tick;
  invalid.mutable_header()->set_server_name("other");
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      invalid);
  invalid = tick;
  invalid.mutable_header()->set_protocol_major(99);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      invalid);
  invalid = tick;
  invalid.mutable_layout()->mutable_commands(0)->set_name("other_motor");
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      invalid);
  EXPECT_EQ(handler_calls.load(), 1);

  tick.mutable_header()->set_tick_id(10);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                      tick);
  EXPECT_EQ(handler_calls.load(), 2);
}

TEST(SyncAdapterClientTest, StopReceptionWaitsForInFlightTickCallback) {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = "default";
  options.controller_id = "arm";
  options.command_names = {"hinge_motor"};
  mujoco_sim::adapter_sdk::SyncAdapterClient client(std::move(options));

  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(9);
  tick.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  tick.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  tick.set_controller_id("arm");
  tick.set_controller_state(mujoco_sim::v1::ACTIVE);

  std::promise<void> handler_entered_promise;
  auto handler_entered = handler_entered_promise.get_future();
  std::promise<void> release_handler_promise;
  auto release_handler = release_handler_promise.get_future().share();
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::SetRegistration(&client,
                                                                      tick);
  mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::SetHandler(
      &client, [&](const mujoco_sim::v1::Tick&) {
        handler_entered_promise.set_value();
        client.Stop();
        release_handler.wait();
        return std::vector<mujoco_sim::adapter_sdk::CommandValue>{
            {"hinge_motor", 0.0}};
      });

  auto callback = std::async(std::launch::async, [&] {
    mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::HandleTickBytes(&client,
                                                                        tick);
  });
  if (handler_entered.wait_for(1s) != std::future_status::ready) {
    release_handler_promise.set_value();
    callback.wait();
    FAIL() << "Tick callback did not enter the handler";
    return;
  }

  auto stop = std::async(std::launch::async, [&] {
    mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer::StopTickReception(
        &client);
  });
  EXPECT_EQ(stop.wait_for(100ms), std::future_status::timeout);
  release_handler_promise.set_value();
  callback.get();
  EXPECT_EQ(stop.wait_for(1s), std::future_status::ready);
  stop.get();
}

}  // namespace
