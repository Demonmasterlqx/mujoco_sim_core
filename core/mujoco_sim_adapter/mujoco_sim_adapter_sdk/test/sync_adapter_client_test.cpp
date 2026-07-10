#include <gtest/gtest.h>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace {

TEST(SyncAdapterClientTest, BuildsCommitFromTick) {
  mujoco_sim::v1::Tick tick;
  tick.mutable_header()->set_server_name("default");
  tick.mutable_header()->set_session_id(4);
  tick.mutable_header()->set_epoch(2);
  tick.mutable_header()->set_tick_id(9);
  tick.set_controller_id("arm");

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

}  // namespace
