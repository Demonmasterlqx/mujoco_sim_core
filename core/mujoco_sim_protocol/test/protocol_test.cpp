#include <gtest/gtest.h>

#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v1/sync.pb.h>

namespace {

TEST(ProtocolTopicsTest, BuildsRegistryAndRuntimeNames) {
  EXPECT_EQ(mujoco_sim::protocol::RegistryServiceName("default"),
            "mujoco_sim.default.registry");
  EXPECT_EQ(mujoco_sim::protocol::TickTopicName("default", "arm"),
            "mujoco_sim/default/tick/arm");
  EXPECT_EQ(mujoco_sim::protocol::CommitTopicName("default"),
            "mujoco_sim/default/commit");
  EXPECT_EQ(mujoco_sim::protocol::HeartbeatTopicName("default"),
            "mujoco_sim/default/heartbeat");
  EXPECT_EQ(mujoco_sim::protocol::StatusTopicName("default"),
            "mujoco_sim/default/status");
}

TEST(ProtocolSerializationTest, RoundTripsRegisterResponse) {
  mujoco_sim::v1::RegisterControllerResponse response;
  response.set_code(mujoco_sim::v1::OK);
  response.set_controller_id("arm");
  response.set_session_id(42);
  response.set_epoch(7);
  response.set_effective_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  response.set_effective_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  response.set_state(mujoco_sim::v1::PROVISIONAL);

  auto* command = response.mutable_layout()->add_commands();
  command->set_name("hinge_motor");
  command->set_type(mujoco_sim::v1::COMMAND_ACTUATOR);
  command->set_dimension(1);
  command->set_has_ctrl_range(true);
  command->set_ctrl_min(-1.0);
  command->set_ctrl_max(1.0);

  std::string bytes;
  ASSERT_TRUE(response.SerializeToString(&bytes));

  mujoco_sim::v1::RegisterControllerResponse parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.code(), mujoco_sim::v1::OK);
  EXPECT_EQ(parsed.controller_id(), "arm");
  EXPECT_EQ(parsed.session_id(), 42);
  ASSERT_EQ(parsed.layout().commands_size(), 1);
  EXPECT_EQ(parsed.layout().commands(0).name(), "hinge_motor");
  EXPECT_DOUBLE_EQ(parsed.layout().commands(0).ctrl_max(), 1.0);
}

TEST(ProtocolSerializationTest, CommitCarriesAtomicCommands) {
  mujoco_sim::v1::AdapterCommit commit;
  commit.mutable_header()->set_server_name("default");
  commit.mutable_header()->set_session_id(10);
  commit.mutable_header()->set_epoch(1);
  commit.mutable_header()->set_tick_id(5);
  commit.set_controller_id("arm");
  commit.set_status(mujoco_sim::v1::COMMIT_OK);
  auto* command = commit.add_commands();
  command->set_name("hinge_motor");
  command->add_values(0.25);

  std::string bytes;
  ASSERT_TRUE(commit.SerializeToString(&bytes));
  mujoco_sim::v1::AdapterCommit parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().tick_id(), 5);
  ASSERT_EQ(parsed.commands_size(), 1);
  EXPECT_DOUBLE_EQ(parsed.commands(0).values(0), 0.25);
}

}  // namespace
