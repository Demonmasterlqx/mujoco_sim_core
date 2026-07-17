#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <thread>

#include "controller_command_inbox.hpp"

namespace mujoco_sim::server {
namespace {

mujoco_sim::core::ControllerCommand Command(std::uint64_t session, std::uint64_t sequence) {
  mujoco_sim::core::ControllerCommand command;
  command.session = {session, 1};
  command.sequence = sequence;
  command.commands = {static_cast<double>(sequence)};
  return command;
}

TEST(ControllerCommandInboxTest, RejectsUnknownAndKeepsLatestPerSession) {
  ControllerCommandInbox inbox;
  EXPECT_FALSE(inbox.Publish(Command(1, 1)));
  ASSERT_TRUE(inbox.Register(1));
  EXPECT_TRUE(inbox.Publish(Command(1, 1)));
  EXPECT_FALSE(inbox.Publish(Command(1, 2)));
  EXPECT_FALSE(inbox.Publish(Command(1, 1)));
  auto values = inbox.Drain();
  ASSERT_EQ(values.size(), 1U);
  EXPECT_EQ(values[0].sequence, 2U);
  EXPECT_DOUBLE_EQ(values[0].commands[0], 2.0);
}

TEST(ControllerCommandInboxTest, ConcurrentPublishAndDrainRemainsDepthOne) {
  ControllerCommandInbox inbox;
  ASSERT_TRUE(inbox.Register(7));
  std::atomic<bool> done = false;
  std::thread publisher([&] {
    for (std::uint64_t i = 1; i <= 10000; ++i) {
      inbox.Publish(Command(7, i));
    }
    done = true;
  });
  std::uint64_t newest = 0;
  while (!done) {
    for (auto& command : inbox.Drain()) {
      newest = std::max(newest, command.sequence);
    }
  }
  publisher.join();
  for (auto& command : inbox.Drain()) {
    newest = std::max(newest, command.sequence);
  }
  EXPECT_EQ(newest, 10000U);
}

TEST(ControllerCommandInboxTest, FaultsAreBoundedPerRegisteredSession) {
  ControllerCommandInbox inbox;
  mujoco_sim::core::ControllerFault fault;
  fault.session = {7, 1};
  fault.code = mujoco_sim::core::FaultCode::kControllerError;
  fault.message = "first";

  EXPECT_FALSE(inbox.PublishFault(fault));
  ASSERT_TRUE(inbox.Register(7));
  EXPECT_TRUE(inbox.PublishFault(fault));
  fault.message = "latest";
  EXPECT_FALSE(inbox.PublishFault(fault));

  auto drained = inbox.DrainFaults();
  ASSERT_EQ(drained.size(), 1U);
  EXPECT_EQ(drained.front().message, "latest");
  EXPECT_TRUE(inbox.DrainFaults().empty());

  inbox.Remove(7);
  EXPECT_FALSE(inbox.PublishFault(fault));
  ASSERT_TRUE(inbox.Register(7));
  inbox.Close();
  EXPECT_FALSE(inbox.PublishFault(fault));
}

TEST(ControllerCommandInboxTest, RemoveAndCloseRejectFurtherData) {
  ControllerCommandInbox inbox;
  inbox.Register(1);
  inbox.Remove(1);
  EXPECT_FALSE(inbox.Publish(Command(1, 1)));
  inbox.Register(2);
  inbox.Close();
  EXPECT_FALSE(inbox.Publish(Command(2, 1)));
  EXPECT_TRUE(inbox.Drain().empty());
  EXPECT_TRUE(inbox.DrainFaults().empty());
}

TEST(ControllerCommandInboxTest, ReservationMoveTransfersPlaceholderOwnership) {
  ControllerCommandInbox inbox;
  auto original = inbox.PrepareRegistration(true);
  ASSERT_TRUE(original);

  auto moved = std::move(original);
  EXPECT_FALSE(original);
  ControllerCommandInbox::RegistrationReservation assigned;
  assigned = std::move(moved);
  EXPECT_FALSE(moved);
  ASSERT_TRUE(assigned);
  ASSERT_TRUE(assigned.Commit(42, 1));

  EXPECT_TRUE(inbox.Publish(Command(42, 1)));
  auto commands = inbox.Drain();
  ASSERT_EQ(commands.size(), 1U);
  EXPECT_EQ(commands.front().session.session_id, 42U);
}

TEST(ControllerCommandInboxTest, InvalidFaultCannotReplaceValidFault) {
  ControllerCommandInbox inbox;
  ASSERT_TRUE(inbox.Register(7, 3));

  mujoco_sim::core::ControllerFault valid;
  valid.session = {7, 3};
  valid.code = mujoco_sim::core::FaultCode::kControllerError;
  valid.message = "valid";
  EXPECT_TRUE(inbox.PublishFault(valid));

  auto invalid = valid;
  invalid.session.epoch = 4;
  EXPECT_FALSE(inbox.PublishFault(invalid));
  invalid.session.epoch = 3;
  invalid.code = mujoco_sim::core::FaultCode::kUnspecified;
  EXPECT_FALSE(inbox.PublishFault(invalid));

  auto faults = inbox.DrainFaults();
  ASSERT_EQ(faults.size(), 1U);
  EXPECT_EQ(faults.front().message, "valid");
}

}  // namespace
}  // namespace mujoco_sim::server
