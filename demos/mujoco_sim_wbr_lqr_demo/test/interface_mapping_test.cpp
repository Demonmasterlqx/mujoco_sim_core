#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include <mujoco_sim/wbr_lqr_demo/controller.hpp>

#include "test_support.hpp"

namespace demo = mujoco_sim::wbr_lqr_demo;
namespace test = mujoco_sim::wbr_lqr_demo::test;

TEST(InterfaceMappingTest, DecodesDenseRegisteredSpans) {
  const auto layout = test::MakeValidLayout(true, false);
  auto tick = test::MakeValidTick(layout, 0.017);
  const std::array<double, 1> q{1.25};
  const std::array<double, 1> velocity{-7.5};
  const std::array<double, 1> effort{3.75};
  test::SetObservation(layout, "ljoint1_pos", q, &tick.observations);
  test::SetObservation(layout, "rwheel_vel", velocity, &tick.observations);
  test::SetObservation(layout, "rjoint4_tor", effort, &tick.observations);

  const demo::StateDecoder decoder(layout);
  const auto raw = decoder.Decode(tick.observations, tick.simulation_time_s);
  EXPECT_DOUBLE_EQ(raw.time, 0.017);
  EXPECT_FLOAT_EQ(raw.quat_gt[0], 1.0F);
  EXPECT_FLOAT_EQ(raw.accel[2], 9.81F);
  EXPECT_FLOAT_EQ(raw.motors[0].q, 1.25F);
  EXPECT_FLOAT_EQ(raw.motors[5].dq, -7.5F);
  EXPECT_FLOAT_EQ(raw.motors[4].tau, 3.75F);
}

TEST(InterfaceMappingTest, ValidatesLayoutAndValues) {
  auto observations = test::MakeValidLayout().observations();
  auto commands = test::MakeValidLayout().commands();
  observations.front().scalars.count = 3;
  const mujoco_sim::adapter_sdk::ControllerLayout wrong(observations, commands, 31, 6);
  EXPECT_THROW((demo::StateDecoder(wrong)), demo::InterfaceError);

  auto unbounded_commands = commands;
  unbounded_commands.front().has_command_range = false;
  const mujoco_sim::adapter_sdk::ControllerLayout unbounded(test::MakeValidLayout().observations(),
                                                            unbounded_commands, 31, 6);
  EXPECT_THROW((demo::TorqueCommandMapper(unbounded)), demo::InterfaceError);

  const auto layout = test::MakeValidLayout();
  auto tick = test::MakeValidTick(layout);
  tick.observations[layout.Observation("lwheel_vel")->offset] =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW(demo::StateDecoder(layout).Decode(tick.observations, 0.0), demo::InterfaceError);

  tick = test::MakeValidTick(layout);
  tick.observations.pop_back();
  EXPECT_THROW(demo::StateDecoder(layout).Decode(tick.observations, 0.0), demo::InterfaceError);
  const auto valid_tick = test::MakeValidTick(layout);
  EXPECT_THROW(demo::StateDecoder(layout).Decode(valid_tick.observations,
                                                 std::numeric_limits<double>::quiet_NaN()),
               demo::InterfaceError);

  auto overlapping_observations = layout.observations();
  overlapping_observations[1].scalars.offset = overlapping_observations[0].scalars.offset;
  const mujoco_sim::adapter_sdk::ControllerLayout overlapping(
      overlapping_observations, layout.commands(), layout.observation_scalar_count(),
      layout.command_scalar_count());
  EXPECT_THROW((demo::StateDecoder(overlapping)), demo::InterfaceError);

  const mujoco_sim::adapter_sdk::ControllerLayout wrong_scalar_count(
      layout.observations(), layout.commands(), layout.observation_scalar_count() - 1,
      layout.command_scalar_count());
  EXPECT_THROW((demo::StateDecoder(wrong_scalar_count)), demo::InterfaceError);
}

TEST(InterfaceMappingTest, MapsDenseCommandOrderAndClampsRanges) {
  const auto layout = test::MakeValidLayout(false, true);
  const demo::TorqueCommandMapper mapper(layout);
  const auto commands = mapper.Map({25.0F, -25.0F, 8.0F, -21.0F, 19.0F, -9.0F});
  ASSERT_EQ(commands.size(), 6U);
  EXPECT_DOUBLE_EQ(commands[layout.Command("rwheel_actuator")->offset], -5.2);
  EXPECT_DOUBLE_EQ(commands[layout.Command("rjoint4_actuator")->offset], 19.0);
  EXPECT_DOUBLE_EQ(commands[layout.Command("rjoint1_actuator")->offset], -20.0);
  EXPECT_DOUBLE_EQ(commands[layout.Command("lwheel_actuator")->offset], 5.2);
  EXPECT_DOUBLE_EQ(commands[layout.Command("ljoint4_actuator")->offset], -20.0);
  EXPECT_DOUBLE_EQ(commands[layout.Command("ljoint1_actuator")->offset], 20.0);
  EXPECT_EQ(mapper.last_clamp_count(), 5U);

  auto torque = std::array<float, 6>{};
  torque[3] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(mapper.Map(torque), demo::InterfaceError);
}

TEST(InterfaceMappingTest, BuildsAdapterRegistrationOptions) {
  demo::AdapterConfig config;
  config.server_name = "sim";
  config.adapter_id = "wbr";
  const auto options = demo::MakeControllerOptions(config);
  EXPECT_EQ(options.server_name, "sim");
  EXPECT_EQ(options.adapter_id, "wbr");
  EXPECT_EQ(options.observation_names.size(), 21U);
  EXPECT_EQ(options.command_names.size(), 6U);
}

TEST(InterfaceMappingTest, RunsInputScriptRelativeToAdapterSession) {
  const auto layout = test::MakeValidLayout();
  const auto tick = test::MakeValidTick(layout);
  control::input_snapshot_t toggle_move{};
  toggle_move.space = true;
  demo::PipelineConfig pipeline_config;
  pipeline_config.imu_mode = control::imu_mode::bypass;
  demo::WbrController controller(layout, pipeline_config,
                                 demo::InputScript({{0, 1, {}}, {1, 2, toggle_move}}));

  controller.ComputeCommand(tick.observations, 0.5);
  ASSERT_TRUE(controller.last_output().has_value());
  EXPECT_FALSE(controller.last_input().space);
  controller.ComputeCommand(tick.observations, 0.501);
  EXPECT_TRUE(controller.last_input().space);
  EXPECT_TRUE(controller.last_output()->command.move);
  controller.Reset();
  controller.ComputeCommand(tick.observations, 0.9);
  EXPECT_FALSE(controller.last_input().space);
}
