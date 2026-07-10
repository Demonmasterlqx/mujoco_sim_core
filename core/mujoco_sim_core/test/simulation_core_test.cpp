#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <string>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace {

mujoco_sim::core::SimulationCore LoadedCoreFrom(const std::string& model_path) {
  mujoco_sim::core::SimulationCore core;
  std::string error;
  EXPECT_TRUE(core.LoadModel(model_path, &error)) << error;
  return core;
}

mujoco_sim::core::SimulationCore LoadedCore() {
  return LoadedCoreFrom(MUJOCO_SIM_CORE_TEST_MODEL);
}

mujoco_sim::v1::RegisterControllerRequest BasicRegisterRequest(
    const std::string& controller_id = "arm",
    const std::string& command_name = "hinge_motor") {
  mujoco_sim::v1::RegisterControllerRequest request;
  request.set_server_name("default");
  request.set_controller_id(controller_id);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.add_observation_names("hinge_pos");
  request.add_observation_names("hinge_vel");
  request.add_command_names(command_name);
  request.set_timeout_ms(25);
  return request;
}

mujoco_sim::v1::AdapterCommit CommitFor(
    const mujoco_sim::v1::RegisterControllerResponse& registration,
    std::uint64_t tick_id, double value) {
  mujoco_sim::v1::AdapterCommit commit;
  commit.mutable_header()->set_server_name("default");
  commit.mutable_header()->set_session_id(registration.session_id());
  commit.mutable_header()->set_epoch(registration.epoch());
  commit.mutable_header()->set_tick_id(tick_id);
  commit.mutable_header()->set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  commit.mutable_header()->set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  commit.set_controller_id(registration.controller_id());
  commit.set_status(mujoco_sim::v1::COMMIT_OK);
  auto* command = commit.add_commands();
  command->set_name(registration.layout().commands(0).name());
  command->add_values(value);
  return commit;
}

mujoco_sim::v1::RegisterControllerResponse RegisterAndActivate(
    mujoco_sim::core::SimulationCore* core,
    const mujoco_sim::v1::RegisterControllerRequest& request) {
  auto registration = core->RegisterController(request);
  EXPECT_EQ(registration.code(), mujoco_sim::v1::OK);
  mujoco_sim::v1::ActivateControllerRequest activate;
  activate.set_controller_id(registration.controller_id());
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  const auto activated = core->ActivateController(activate);
  EXPECT_EQ(activated.code(), mujoco_sim::v1::OK);
  return registration;
}

mujoco_sim::v1::RegisterControllerResponse RegisterAndActivate(
    mujoco_sim::core::SimulationCore* core) {
  return RegisterAndActivate(core, BasicRegisterRequest());
}

TEST(SimulationCoreTest, LoadsModelAndBuildsCatalogs) {
  auto core = LoadedCore();
  ASSERT_TRUE(core.IsModelLoaded());
  EXPECT_EQ(core.observations().size(), 3);
  ASSERT_EQ(core.commands().size(), 1);
  EXPECT_EQ(core.commands()[0].name(), "hinge_motor");
  EXPECT_TRUE(core.commands()[0].has_ctrl_range());
  EXPECT_DOUBLE_EQ(core.commands()[0].ctrl_min(), -1.0);
  EXPECT_DOUBLE_EQ(core.commands()[0].ctrl_max(), 1.0);
}

TEST(SimulationCoreTest, ProvidesAvailableResources) {
  auto core = LoadedCore();
  mujoco_sim::v1::GetAvailableResourcesRequest request;
  request.set_server_name("default");
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  const auto response = core.GetAvailableResources(request);
  EXPECT_EQ(response.code(), mujoco_sim::v1::OK);
  EXPECT_EQ(response.observations_size(), 3);
  EXPECT_EQ(response.commands_size(), 1);
}

TEST(SimulationCoreTest, RejectsUnknownAndConflictingResources) {
  auto core = LoadedCore();
  auto bad = BasicRegisterRequest("bad");
  bad.set_observation_names(0, "missing_sensor");
  EXPECT_EQ(core.RegisterController(bad).code(), mujoco_sim::v1::NOT_FOUND);

  auto first = core.RegisterController(BasicRegisterRequest("first"));
  EXPECT_EQ(first.code(), mujoco_sim::v1::OK);
  auto conflict = core.RegisterController(BasicRegisterRequest("second"));
  EXPECT_EQ(conflict.code(), mujoco_sim::v1::RESOURCE_CONFLICT);
}

TEST(SimulationCoreTest, ActivatesAndPublishesTickWithObservation) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  const auto ticks = core.OpenTick();
  ASSERT_EQ(ticks.size(), 1);
  EXPECT_TRUE(core.tick_open());
  EXPECT_EQ(ticks[0].controller_id(), registration.controller_id());
  EXPECT_EQ(ticks[0].header().tick_id(), 0);
  EXPECT_GE(ticks[0].observations_size(), 1);
}

TEST(SimulationCoreTest, StepsWhenBarrierCompletesAndHandlesDuplicateCommits) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);

  const auto commit = CommitFor(registration, 0, 0.2);
  const auto duplicate_in_open_tick = core.SubmitCommit(commit);
  EXPECT_EQ(duplicate_in_open_tick.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(duplicate_in_open_tick.stepped);
  const auto stale_duplicate = core.SubmitCommit(commit);
  EXPECT_EQ(stale_duplicate.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(stale_duplicate.duplicate);
  EXPECT_NE(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  EXPECT_EQ(core.current_tick_id(), 1);

  ASSERT_EQ(core.OpenTick().size(), 1);
  const auto duplicate_current_tick = core.SubmitCommit(CommitFor(registration, 1, 0.2));
  EXPECT_EQ(duplicate_current_tick.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(duplicate_current_tick.stepped);
  const auto result = core.SubmitCommit(CommitFor(registration, 1, 0.2));
  EXPECT_EQ(result.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(result.duplicate);
  EXPECT_EQ(core.current_tick_id(), 2);
}

TEST(SimulationCoreTest, RejectsSameTickDuplicateCommitWithDifferentPayload) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second =
      RegisterAndActivate(&core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.active_controller_count(), 2);
  ASSERT_EQ(core.OpenTick().size(), 2);

  const auto first_commit = CommitFor(first, 0, 0.2);
  const auto accepted = core.SubmitCommit(first_commit);
  EXPECT_EQ(accepted.code, mujoco_sim::v1::OK);
  EXPECT_FALSE(accepted.stepped);

  const auto same_duplicate = core.SubmitCommit(first_commit);
  EXPECT_EQ(same_duplicate.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(same_duplicate.duplicate);
  EXPECT_NE(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);

  const auto different_duplicate = core.SubmitCommit(CommitFor(first, 0, 0.3));
  EXPECT_EQ(different_duplicate.code, mujoco_sim::v1::INVALID_COMMAND);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);

  (void)second;
}

TEST(SimulationCoreTest, RejectsInvalidCommandsAndErrorPauses) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);

  auto commit = CommitFor(registration, 0, std::numeric_limits<double>::quiet_NaN());
  const auto result = core.SubmitCommit(commit);
  EXPECT_EQ(result.code, mujoco_sim::v1::INVALID_COMMAND);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
}

TEST(SimulationCoreTest, RejectsOutOfRangeCommands) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);

  const auto result = core.SubmitCommit(CommitFor(registration, 0, 2.0));
  EXPECT_EQ(result.code, mujoco_sim::v1::INVALID_COMMAND);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
}

TEST(SimulationCoreTest, RejectsTickSessionAndEpochMismatches) {
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    ASSERT_EQ(core.OpenTick().size(), 1);

    auto stale = CommitFor(registration, 1, 0.0);
    EXPECT_EQ(core.SubmitCommit(stale).code, mujoco_sim::v1::INVALID_TICK);
    EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  }
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    ASSERT_EQ(core.OpenTick().size(), 1);

    auto wrong_session = CommitFor(registration, 0, 0.0);
    wrong_session.mutable_header()->set_session_id(registration.session_id() +
                                                   100);
    EXPECT_EQ(core.SubmitCommit(wrong_session).code,
              mujoco_sim::v1::INVALID_SESSION);
    EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  }
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    ASSERT_EQ(core.OpenTick().size(), 1);

    auto wrong_epoch = CommitFor(registration, 0, 0.0);
    wrong_epoch.mutable_header()->set_epoch(registration.epoch() + 1);
    EXPECT_EQ(core.SubmitCommit(wrong_epoch).code,
              mujoco_sim::v1::INVALID_EPOCH);
    EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  }
}

TEST(SimulationCoreTest, RejectsSessionEpochAndTickMismatchesAfterTickCloses) {
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    ASSERT_EQ(core.OpenTick().size(), 1);
    EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).stepped);

    auto wrong_session = CommitFor(registration, 0, 0.1);
    wrong_session.mutable_header()->set_session_id(registration.session_id() +
                                                   100);
    EXPECT_EQ(core.SubmitCommit(wrong_session).code,
              mujoco_sim::v1::INVALID_SESSION);
    EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  }
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    ASSERT_EQ(core.OpenTick().size(), 1);
    EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).stepped);

    auto wrong_epoch = CommitFor(registration, 0, 0.1);
    wrong_epoch.mutable_header()->set_epoch(registration.epoch() + 1);
    EXPECT_EQ(core.SubmitCommit(wrong_epoch).code,
              mujoco_sim::v1::INVALID_EPOCH);
    EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  }
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    ASSERT_EQ(core.OpenTick().size(), 1);
    EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).stepped);

    EXPECT_EQ(core.SubmitCommit(CommitFor(registration, 2, 0.1)).code,
              mujoco_sim::v1::INVALID_TICK);
    EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
  }
}

TEST(SimulationCoreTest, TimesOutOpenTick) {
  auto core = LoadedCore();
  RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);

  const bool timed_out =
      core.CheckTickTimeout(std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(100));
  EXPECT_TRUE(timed_out);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::ERROR_PAUSED);
}

}  // namespace
