#include <gtest/gtest.h>

#include <mujoco_sim/sample_adapter/sample_adapter.hpp>

namespace {

TEST(SampleAdapterTest, ParsesNameList) {
  const auto names =
      mujoco_sim::sample_adapter::ParseNameList(" hinge_pos,hinge_vel ,, imu ");
  ASSERT_EQ(names.size(), 3);
  EXPECT_EQ(names[0], "hinge_pos");
  EXPECT_EQ(names[1], "hinge_vel");
  EXPECT_EQ(names[2], "imu");
}

TEST(SampleAdapterTest, BuildsConstantCommandsFromTickLayout) {
  mujoco_sim::v1::Tick tick;
  auto* command = tick.mutable_layout()->add_commands();
  command->set_name("hinge_motor");
  command->set_type(mujoco_sim::v1::COMMAND_ACTUATOR);

  const auto commands =
      mujoco_sim::sample_adapter::ConstantCommandsForTick(tick, 0.25);
  ASSERT_EQ(commands.size(), 1);
  EXPECT_EQ(commands[0].name, "hinge_motor");
  EXPECT_DOUBLE_EQ(commands[0].value, 0.25);
}

TEST(SampleAdapterTest, ParsesCommandLineDefaultsAndOverrides) {
  const char* argv[] = {"mujoco-sim-sample-adapter",
                        "--server-name",
                        "sim",
                        "--controller-id",
                        "arm",
                        "--ticks",
                        "4",
                        "--command-value",
                        "0.1"};
  mujoco_sim::sample_adapter::SampleOptions options;
  std::string error;
  ASSERT_TRUE(mujoco_sim::sample_adapter::ParseCommandLine(
      static_cast<int>(std::size(argv)), const_cast<char**>(argv), &options,
      &error))
      << error;
  EXPECT_EQ(options.adapter.server_name, "sim");
  EXPECT_EQ(options.adapter.controller_id, "arm");
  EXPECT_EQ(options.max_ticks, 4);
  EXPECT_DOUBLE_EQ(options.command_value, 0.1);
  ASSERT_EQ(options.adapter.command_names.size(), 1);
  EXPECT_EQ(options.adapter.command_names[0], "hinge_motor");
}

}  // namespace
