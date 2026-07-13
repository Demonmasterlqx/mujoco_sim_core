#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <string>

#include <mujoco_sim/wbr_lqr_demo/tick_controller.hpp>

#include "test_support.hpp"

namespace demo = mujoco_sim::wbr_lqr_demo;
namespace test = mujoco_sim::wbr_lqr_demo::test;

TEST(InterfaceMappingTest, DecodesTwentyOneBlocksByName) {
  const auto layout = test::MakeValidLayout(true, false);
  auto tick = test::MakeValidTick(layout, 17, false);
  test::FindBlock(&tick, "ljoint1_pos")->set_values(0, 1.25);
  test::FindBlock(&tick, "rwheel_vel")->set_values(0, -7.5);
  test::FindBlock(&tick, "rjoint4_tor")->set_values(0, 3.75);

  const demo::TickDecoder decoder(layout);
  const auto raw = decoder.Decode(tick);
  EXPECT_DOUBLE_EQ(raw.time, 0.017000000807456672);
  EXPECT_FLOAT_EQ(raw.quat_gt[0], 1.0F);
  EXPECT_FLOAT_EQ(raw.accel[2], 9.81F);
  EXPECT_FLOAT_EQ(raw.motors[0].q, 1.25F);
  EXPECT_FLOAT_EQ(raw.motors[5].dq, -7.5F);
  EXPECT_FLOAT_EQ(raw.motors[4].tau, 3.75F);
}

TEST(InterfaceMappingTest, ValidatesLayoutDimensionsAndCompleteness) {
  auto wrong_dimension = test::MakeValidLayout();
  wrong_dimension.mutable_observations(0)->set_dimension(3);
  EXPECT_THROW((demo::TickDecoder(wrong_dimension)), demo::InterfaceError);

  auto missing = test::MakeValidLayout();
  missing.mutable_observations()->RemoveLast();
  EXPECT_THROW((demo::TickDecoder(missing)), demo::InterfaceError);

  auto duplicate = test::MakeValidLayout();
  duplicate.mutable_observations(1)->set_name(
      duplicate.observations(0).name());
  duplicate.mutable_observations(1)->set_dimension(
      duplicate.observations(0).dimension());
  EXPECT_THROW((demo::TickDecoder(duplicate)), demo::InterfaceError);

  auto unbounded = test::MakeValidLayout();
  unbounded.mutable_commands(0)->set_has_ctrl_range(false);
  EXPECT_THROW((demo::TorqueCommandMapper(unbounded)), demo::InterfaceError);
}

TEST(InterfaceMappingTest, RejectsBadTickDimensionsAndNonFiniteValues) {
  const auto layout = test::MakeValidLayout();
  const demo::TickDecoder decoder(layout);

  auto wrong_dimension = test::MakeValidTick(layout);
  test::FindBlock(&wrong_dimension, "imu_gyro")->mutable_values()->RemoveLast();
  EXPECT_THROW(decoder.Decode(wrong_dimension), demo::InterfaceError);

  auto non_finite = test::MakeValidTick(layout);
  test::FindBlock(&non_finite, "lwheel_vel")
      ->set_values(0, std::numeric_limits<double>::infinity());
  EXPECT_THROW(decoder.Decode(non_finite), demo::InterfaceError);

  auto changed_layout = test::MakeValidTick(layout);
  changed_layout.set_layout_hash(std::string(16, '\0'));
  EXPECT_THROW(decoder.Decode(changed_layout), demo::InterfaceError);
}

TEST(InterfaceMappingTest, MapsByActuatorNameAndClampsRegisteredRanges) {
  const auto layout = test::MakeValidLayout(false, true);
  const demo::TorqueCommandMapper mapper(layout);
  const auto commands = mapper.Map({25.0F, -25.0F, 8.0F, -21.0F, 19.0F,
                                    -9.0F});

  ASSERT_EQ(commands.size(), 6U);
  EXPECT_EQ(commands[0].name, "rwheel_actuator");
  EXPECT_DOUBLE_EQ(commands[0].value, -5.2);
  EXPECT_EQ(commands[1].name, "rjoint4_actuator");
  EXPECT_DOUBLE_EQ(commands[1].value, 19.0);
  EXPECT_EQ(commands[2].name, "rjoint1_actuator");
  EXPECT_DOUBLE_EQ(commands[2].value, -20.0);
  EXPECT_EQ(commands[3].name, "lwheel_actuator");
  EXPECT_DOUBLE_EQ(commands[3].value, 5.2);
  EXPECT_EQ(commands[4].name, "ljoint4_actuator");
  EXPECT_DOUBLE_EQ(commands[4].value, -20.0);
  EXPECT_EQ(commands[5].name, "ljoint1_actuator");
  EXPECT_DOUBLE_EQ(commands[5].value, 20.0);
  EXPECT_EQ(mapper.last_clamp_count(), 5U);

  auto torque = std::array<float, 6>{};
  torque[3] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(mapper.Map(torque), demo::InterfaceError);
}

TEST(InterfaceMappingTest, DecodesAndMapsFullyRenamedResources) {
  auto config = demo::MakeUpstreamWbrInterfaceConfig();
  config.imu.quaternion = "robot/imu/orientation";
  config.imu.angular_velocity = "robot/imu/angular_velocity";
  config.imu.linear_acceleration = "robot/imu/linear_acceleration";
  for (std::size_t i = 0; i < config.motors.size(); ++i) {
    const std::string prefix = "robot/motor_" + std::to_string(i);
    config.motors[i].position_state = prefix + "/position";
    config.motors[i].velocity_state = prefix + "/velocity";
    config.motors[i].effort_state = prefix + "/effort";
    config.motors[i].effort_command = prefix + "/command";
  }

  auto observations = demo::WbrObservationNames(config);
  auto commands = demo::WbrCommandNames(config);
  std::reverse(observations.begin(), observations.end());
  std::reverse(commands.begin(), commands.end());

  mujoco_sim::v2::ResourceLayout layout;
  for (const auto& name : observations) {
    auto* descriptor = layout.add_observations();
    descriptor->set_name(name);
    descriptor->set_type(mujoco_sim::v2::OBSERVATION_SENSOR);
    descriptor->set_dimension(name == config.imu.quaternion
                                  ? 4
                                  : (name == config.imu.angular_velocity ||
                                             name == config.imu.linear_acceleration
                                         ? 3
                                         : 1));
  }
  for (const auto& name : commands) {
    const auto motor = std::find_if(
        config.motors.begin(), config.motors.end(),
        [&name](const demo::MotorInterfaces& candidate) {
          return candidate.effort_command == name;
        });
    ASSERT_NE(motor, config.motors.end());
    const std::size_t index =
        static_cast<std::size_t>(motor - config.motors.begin());
    auto* descriptor = layout.add_commands();
    descriptor->set_name(name);
    descriptor->set_type(mujoco_sim::v2::COMMAND_ACTUATOR);
    descriptor->set_dimension(1);
    descriptor->set_has_ctrl_range(true);
    descriptor->set_ctrl_min(-static_cast<double>(index + 1));
    descriptor->set_ctrl_max(static_cast<double>(index + 1));
  }

  layout.set_layout_hash(mujoco_sim::protocol::ComputeLayoutHash(layout));
  mujoco_sim::v2::Tick tick;
  tick.mutable_header()->set_tick_id(23);
  tick.set_layout_hash(layout.layout_hash());
  for (const auto& name : observations) {
    auto* block = tick.add_observations();
    block->set_name(name);
    if (name == config.imu.quaternion) {
      for (const double value : {0.5, 0.5, -0.5, -0.5}) {
        block->add_values(value);
      }
    } else if (name == config.imu.angular_velocity) {
      for (const double value : {1.0, 2.0, 3.0}) block->add_values(value);
    } else if (name == config.imu.linear_acceleration) {
      for (const double value : {4.0, 5.0, 6.0}) block->add_values(value);
    } else {
      for (std::size_t i = 0; i < config.motors.size(); ++i) {
        if (name == config.motors[i].position_state) {
          block->add_values(10.0 + static_cast<double>(i));
        } else if (name == config.motors[i].velocity_state) {
          block->add_values(20.0 + static_cast<double>(i));
        } else if (name == config.motors[i].effort_state) {
          block->add_values(30.0 + static_cast<double>(i));
        }
      }
    }
  }

  const demo::TickDecoder decoder(config, layout);
  const auto raw = decoder.Decode(tick);
  EXPECT_FLOAT_EQ(raw.quat_gt[2], -0.5F);
  EXPECT_FLOAT_EQ(raw.gyro[1], 2.0F);
  EXPECT_FLOAT_EQ(raw.accel[2], 6.0F);
  for (std::size_t i = 0; i < config.motors.size(); ++i) {
    EXPECT_FLOAT_EQ(raw.motors[i].q, 10.0F + static_cast<float>(i));
    EXPECT_FLOAT_EQ(raw.motors[i].dq, 20.0F + static_cast<float>(i));
    EXPECT_FLOAT_EQ(raw.motors[i].tau, 30.0F + static_cast<float>(i));
  }

  const demo::TorqueCommandMapper mapper(config, layout);
  const auto mapped =
      mapper.Map({10.0F, -10.0F, 10.0F, -10.0F, 10.0F, -10.0F});
  ASSERT_EQ(mapped.size(), config.motors.size());
  for (std::size_t order = 0; order < mapped.size(); ++order) {
    const std::size_t motor_index = mapped.size() - order - 1;
    EXPECT_EQ(mapped[order].name,
              config.motors[motor_index].effort_command);
    const double magnitude = static_cast<double>(motor_index + 1);
    EXPECT_DOUBLE_EQ(mapped[order].value,
                     motor_index % 2 == 0 ? magnitude : -magnitude);
  }
  EXPECT_EQ(mapper.last_clamp_count(), 6U);
}

TEST(InterfaceMappingTest, BuildsFixedAdapterRegistrationOptions) {
  demo::AdapterConfig config;
  config.server_name = "sim";
  config.adapter_id = "wbr";
  const auto options = demo::MakeAdapterOptions(config);
  EXPECT_EQ(options.server_name, "sim");
  EXPECT_EQ(options.adapter_id, "wbr");
  EXPECT_EQ(options.observation_names.size(), 21U);
  EXPECT_EQ(options.command_names.size(), 6U);
}

TEST(InterfaceMappingTest, RunsInputScriptRelativeToAdapterSession) {
  const auto layout = test::MakeValidLayout();
  control::input_snapshot_t toggle_move{};
  toggle_move.space = true;
  demo::PipelineConfig pipeline_config;
  pipeline_config.imu_mode = control::imu_mode::bypass;
  demo::TickController controller(
      layout, pipeline_config,
      demo::InputScript({{0, 1, {}}, {1, 2, toggle_move}}));

  controller.HandleTick(test::MakeValidTick(layout, 500));
  ASSERT_TRUE(controller.last_output().has_value());
  EXPECT_FALSE(controller.last_input().space);
  EXPECT_FALSE(controller.last_output()->command.move);

  controller.HandleTick(test::MakeValidTick(layout, 501));
  ASSERT_TRUE(controller.last_output().has_value());
  EXPECT_TRUE(controller.last_input().space);
  EXPECT_TRUE(controller.last_output()->command.move);

  controller.Reset();
  controller.HandleTick(test::MakeValidTick(layout, 900));
  ASSERT_TRUE(controller.last_output().has_value());
  EXPECT_FALSE(controller.last_input().space);
  EXPECT_FALSE(controller.last_output()->command.move);
}
