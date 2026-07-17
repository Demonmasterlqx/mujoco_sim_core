#include <gtest/gtest.h>

#include <control/balance.hpp>
#include <mujoco_sim/wbr_lqr_demo/controller_pipeline.hpp>

#include "test_support.hpp"

namespace demo = mujoco_sim::wbr_lqr_demo;
namespace test = mujoco_sim::wbr_lqr_demo::test;

TEST(PipelineGoldenTest, HomePoseRelaxSnapshotMatchesUpstream) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  demo::ControllerPipeline pipeline(config);

  const auto output = pipeline.Step(test::MakeHomeObservation(), {});

  // Pinned from a standalone build against upstream commit 6a8b375.
  EXPECT_NEAR(output.left.length, 0.334131330F, 1e-6F);
  EXPECT_NEAR(output.left.phi, 2.94079638F, 1e-6F);
  EXPECT_NEAR(output.left.alpha, 1.37000000F, 1e-6F);
  EXPECT_NEAR(output.left.alpha_equilibrium, 0.0504172891F, 1e-6F);
  EXPECT_NEAR(output.left.spring_force, 22.5799942F, 1e-5F);
  EXPECT_NEAR(output.left.normal_force, 6.56367302F, 1e-5F);
  EXPECT_NEAR(output.right.length, output.left.length, 1e-7F);
  EXPECT_NEAR(output.right.normal_force, output.left.normal_force, 1e-6F);
  EXPECT_NEAR(output.odometry.a_z, 9.81000042F, 1e-6F);
  EXPECT_NEAR(output.pendulum.N, 13.1273460F, 1e-5F);
}

TEST(PipelineGoldenTest, NormalStepRawTorqueMatchesUpstreamSequence) {
  demo::PipelineConfig config;
  config.imu_mode = control::imu_mode::bypass;
  config.chassis.fsm.enter_normal_ticks = 1;
  demo::ControllerPipeline pipeline(config);

  const auto flat = test::MakeHomeObservation();
  auto neutral = test::MakeHomeObservation();
  neutral.motors[0].q = 0.18F;
  neutral.motors[1].q = -0.44F;
  neutral.motors[3].q = -0.18F;
  neutral.motors[4].q = 0.44F;

  control::input_snapshot_t toggle_move{};
  toggle_move.space = true;
  pipeline.Step(flat, {});
  pipeline.Step(flat, toggle_move);
  pipeline.Step(flat, {});
  pipeline.Step(neutral, {});
  const auto normal = pipeline.Step(neutral, {});

  ASSERT_EQ(normal.fsm_state, control::chassis_state::normal);
  EXPECT_NEAR(normal.left.length, 0.200398296F, 1e-6F);
  EXPECT_NEAR(normal.left.alpha, 0.0F, 1e-6F);
  EXPECT_NEAR(normal.pendulum.N, 82.4239273F, 1e-4F);
  const float expected[6] = {3.54397225F, -3.54397154F, -0.0F, -3.54397225F, 3.54397154F, -0.0F};
  for (std::size_t i = 0; i < normal.raw_torque.size(); ++i) {
    EXPECT_NEAR(normal.raw_torque[i], expected[i], 2e-5F) << i;
  }
}

TEST(PipelineGoldenTest, LqrTableMatchesPinnedUpstreamOutput) {
  control::lqr_solver solver;
  const float reference[10] = {0.1F, -0.2F, 0.3F, -0.4F, 0.05F, 0.1F, -0.1F, 0.2F, -0.05F, 0.15F};
  const float observation[10] = {-0.2F, 0.4F,  -0.1F,  0.3F, -0.2F,
                                 0.5F,  0.25F, -0.35F, 0.1F, -0.45F};
  solver.update(0.21F, 0.29F, reference, observation, demo::MakeUpstreamWbrChassisConfig());

  EXPECT_NEAR(solver.tout[0], -0.467866302F, 1e-5F);
  EXPECT_NEAR(solver.tout[1], 0.623082042F, 1e-5F);
  EXPECT_NEAR(solver.tout[2], -9.65930843F, 1e-5F);
  EXPECT_NEAR(solver.tout[3], 2.17984962F, 1e-5F);
}
