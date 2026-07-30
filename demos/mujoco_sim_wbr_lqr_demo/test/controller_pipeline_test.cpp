#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <mujoco_sim/wbr_lqr_demo/controller_pipeline.hpp>

#include "test_support.hpp"

namespace demo = mujoco_sim::wbr_lqr_demo;
namespace test = mujoco_sim::wbr_lqr_demo::test;

TEST(ControllerPipelineTest, FirstTickUsesEmptyPreviousPendulum) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  demo::ControllerPipeline pipeline(config);

  auto raw = test::MakeHomeObservation();
  raw.motors[2].dq = 10.0F;
  raw.motors[5].dq = -10.0F;
  const auto first = pipeline.Step(raw, {});

  EXPECT_EQ(first.tick_index, 0U);
  EXPECT_EQ(pipeline.tick_count(), 1U);
  EXPECT_FLOAT_EQ(first.fusion_pendulum.x, 0.0F);
  EXPECT_FLOAT_EQ(first.command.x, 0.0F);
  EXPECT_EQ(first.fsm_state, control::chassis_state::relax);
  for (const float torque : first.raw_torque) {
    EXPECT_FLOAT_EQ(torque, 0.0F);
  }

  const auto second = pipeline.Step(raw, {});
  EXPECT_FLOAT_EQ(second.fusion_pendulum.x, first.pendulum.x);
  EXPECT_FLOAT_EQ(second.command.x, -0.00025F);
  EXPECT_GT(second.command.x, first.pendulum.x);
}

TEST(ControllerPipelineTest, DirectionInputSelectsFixedSignedVelocityAndYawRate) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  demo::ControllerPipeline pipeline(config);
  const auto raw = test::MakeHomeObservation();

  control::input_snapshot_t forward{};
  forward.w = true;
  const auto forward_output = pipeline.Step(raw, forward);
  EXPECT_FLOAT_EQ(forward_output.command.v, 2.0F);

  control::input_snapshot_t backward{};
  backward.s = true;
  const auto backward_output = pipeline.Step(raw, backward);
  EXPECT_FLOAT_EQ(backward_output.command.v, -2.0F);

  control::input_snapshot_t left{};
  left.a = true;
  const auto left_output = pipeline.Step(raw, left);
  EXPECT_FLOAT_EQ(left_output.command.dyaw, 1000.0F);

  control::input_snapshot_t right{};
  right.d = true;
  const auto right_output = pipeline.Step(raw, right);
  EXPECT_FLOAT_EQ(right_output.command.dyaw, -1000.0F);

  const auto released = pipeline.Step(raw, {});
  EXPECT_FLOAT_EQ(released.command.v, 0.0F);
  EXPECT_FLOAT_EQ(released.command.dyaw, 0.0F);
}

TEST(ControllerPipelineTest, OpposingDirectionInputsAreNeutral) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  demo::ControllerPipeline pipeline(config);
  const auto raw = test::MakeHomeObservation();

  control::input_snapshot_t opposing{};
  opposing.w = true;
  opposing.s = true;
  opposing.a = true;
  opposing.d = true;
  const auto output = pipeline.Step(raw, opposing);
  EXPECT_FLOAT_EQ(output.command.v, 0.0F);
  EXPECT_FLOAT_EQ(output.command.dyaw, 0.0F);
}

TEST(ControllerPipelineTest, ResetRestoresFirstTickState) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  demo::ControllerPipeline pipeline(config);
  const auto raw = test::MakeHomeObservation();

  pipeline.Step(raw, {});
  pipeline.Reset();
  const auto after_reset = pipeline.Step(raw, {});
  EXPECT_EQ(after_reset.tick_index, 0U);
  EXPECT_FLOAT_EQ(after_reset.fusion_pendulum.x, 0.0F);
  EXPECT_EQ(after_reset.fsm_state, control::chassis_state::relax);
}

TEST(ControllerPipelineTest, RejectsNonFiniteObservationAndWrongDt) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  demo::ControllerPipeline pipeline(config);
  auto raw = test::MakeHomeObservation();
  raw.motors[4].dq = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(pipeline.Step(raw, {}), demo::PipelineError);

  config.chassis.control_dt = 0.002F;
  EXPECT_THROW((demo::ControllerPipeline(config)), demo::PipelineError);
}

TEST(ControllerPipelineTest, MahonyConsumesCurrentObservation) {
  demo::ControllerPipeline pipeline;
  auto raw = test::MakeHomeObservation();
  raw.quat_gt[0] = 0.0F;
  raw.quat_gt[1] = 1.0F;

  const auto output = pipeline.Step(raw, {});
  EXPECT_NEAR(output.imu.quaternion[0], 1.0F, 1e-6F);
  EXPECT_NEAR(output.imu.quaternion[1], 0.0F, 1e-6F);
  EXPECT_NEAR(output.imu.pitch, 0.0F, 1e-6F);
}
