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
  EXPECT_FLOAT_EQ(first.fusion_pendulum.v, 0.0F);
  EXPECT_FALSE(first.fusion_pendulum.normal);
  EXPECT_FLOAT_EQ(first.command.x, 0.0F);
  EXPECT_EQ(first.fsm_state, control::chassis_state::relax);
  for (const float torque : first.raw_torque) {
    EXPECT_FLOAT_EQ(torque, 0.0F);
  }

  const auto second = pipeline.Step(raw, {});
  EXPECT_FLOAT_EQ(second.fusion_pendulum.x, first.pendulum.x);
  EXPECT_FLOAT_EQ(second.fusion_pendulum.v, first.pendulum.v);
  EXPECT_FLOAT_EQ(second.command.x, first.pendulum.x);
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
