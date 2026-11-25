#include <gtest/gtest.h>

#include <chrono>

#include "viewer_input_state.hpp"

namespace {

using namespace std::chrono_literals;
using mujoco_sim::server::SimViewer;
using mujoco_sim::server::ViewerInputState;

TEST(ViewerInputStateTest, MergesControlPerActuatorUntilCompletedStep) {
  ViewerInputState state(100ms);
  SimViewer::ViewerInputBatch first;
  first.actuator_writes = {
      {.actuator_id = 0, .value = 0.2},
      {.actuator_id = 1, .value = 0.3},
  };
  state.Merge(first);
  SimViewer::ViewerInputBatch second;
  second.actuator_writes = {{.actuator_id = 0, .value = 0.4}};
  state.Merge(second);

  auto prepared = state.PrepareStep({});
  ASSERT_EQ(prepared.input.actuator_writes.size(), 2U);
  EXPECT_EQ(prepared.input.actuator_writes[0].actuator_id, 0);
  EXPECT_DOUBLE_EQ(prepared.input.actuator_writes[0].value, 0.4);
  EXPECT_EQ(prepared.input.actuator_writes[1].actuator_id, 1);
  EXPECT_DOUBLE_EQ(prepared.input.actuator_writes[1].value, 0.3);

  state.CommitCompletedStep();
  EXPECT_TRUE(state.PrepareStep({}).input.actuator_writes.empty());
}

TEST(ViewerInputStateTest, ReplacesClearsAndExpiresWrenchFromGenerationTime) {
  ViewerInputState state(100ms);
  const auto generated = std::chrono::steady_clock::time_point(10ms);
  SimViewer::ViewerInputBatch active;
  active.wrench_frame.emplace();
  active.wrench_frame->sequence = 1;
  active.wrench_frame->generated_at = generated;
  active.wrench_frame->body_wrenches = {
      {.body_id = 1, .force = {1.0, 0.0, 0.0}},
  };
  state.Merge(active);

  auto prepared = state.PrepareStep(generated + 100ms);
  EXPECT_FALSE(prepared.wrench_expired);
  EXPECT_TRUE(prepared.input.replace_body_wrenches);
  ASSERT_EQ(prepared.input.body_wrenches.size(), 1U);
  state.CommitCompletedStep();
  EXPECT_FALSE(state.PrepareStep(generated + 100ms).input.replace_body_wrenches);

  prepared = state.PrepareStep(generated + 101ms);
  EXPECT_TRUE(prepared.wrench_expired);
  EXPECT_EQ(prepared.expired_body_count, 1U);
  EXPECT_EQ(prepared.wrench_age, 101ms);
  EXPECT_TRUE(prepared.input.replace_body_wrenches);
  EXPECT_TRUE(prepared.input.body_wrenches.empty());
  state.CommitCompletedStep();
  EXPECT_FALSE(state.PrepareStep(generated + 200ms).wrench_expired);

  active.wrench_frame->sequence = 2;
  active.wrench_frame->generated_at = generated + 200ms;
  state.Merge(active);
  prepared = state.PrepareStep(generated + 201ms);
  EXPECT_FALSE(prepared.wrench_expired);
  EXPECT_EQ(prepared.input.body_wrenches.size(), 1U);

  SimViewer::ViewerInputBatch clear;
  clear.wrench_frame.emplace();
  clear.wrench_frame->sequence = 3;
  clear.wrench_frame->generated_at = generated + 202ms;
  state.Merge(clear);
  prepared = state.PrepareStep(generated + 203ms);
  EXPECT_TRUE(prepared.input.replace_body_wrenches);
  EXPECT_TRUE(prepared.input.body_wrenches.empty());
  EXPECT_FALSE(prepared.wrench_expired);
}

TEST(ViewerInputStateTest, MergesLifecycleActionsUsingLatestAndLatchedRules) {
  ViewerInputState state(100ms);
  SimViewer::ViewerInputBatch first;
  first.simulation_actions.running = false;
  first.simulation_actions.reset_requested = true;
  state.Merge(first);

  SimViewer::ViewerInputBatch second;
  second.simulation_actions.running = true;
  state.Merge(second);

  const auto actions = state.TakeSimulationActions();
  ASSERT_TRUE(actions.running.has_value());
  EXPECT_TRUE(*actions.running);
  EXPECT_TRUE(actions.reset_requested);

  const auto consumed = state.TakeSimulationActions();
  EXPECT_FALSE(consumed.running.has_value());
  EXPECT_FALSE(consumed.reset_requested);
}

TEST(ViewerInputStateTest, ResetAfterSimulationResetClearsAllDataInput) {
  ViewerInputState state(100ms);
  SimViewer::ViewerInputBatch input;
  input.actuator_writes = {{.actuator_id = 0, .value = 0.7}};
  input.wrench_frame.emplace();
  input.wrench_frame->generated_at = std::chrono::steady_clock::time_point(10ms);
  input.wrench_frame->body_wrenches = {
      {.body_id = 1, .force = {1.0, 0.0, 0.0}},
  };
  state.Merge(input);
  ASSERT_FALSE(
      state.PrepareStep(std::chrono::steady_clock::time_point(20ms)).input.actuator_writes.empty());

  state.ResetAfterSimulationReset();
  const auto prepared = state.PrepareStep(std::chrono::steady_clock::time_point(1s));
  EXPECT_TRUE(prepared.input.actuator_writes.empty());
  EXPECT_TRUE(prepared.input.body_wrenches.empty());
  EXPECT_FALSE(prepared.input.replace_body_wrenches);
  EXPECT_FALSE(prepared.wrench_expired);
}

}  // namespace
