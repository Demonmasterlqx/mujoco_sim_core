#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
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

mujoco_sim::v1::ControllerStatus ControllerStatusFor(
    const mujoco_sim::core::SimulationCore& core,
    const std::string& controller_id) {
  const auto server_status = core.MakeStatus();
  for (const auto& status : server_status.controllers()) {
    if (status.controller_id() == controller_id) return status;
  }
  ADD_FAILURE() << "missing controller status for " << controller_id;
  return {};
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
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
  EXPECT_GT(core.timestep(), 0.0);
}

TEST(SimulationCoreTest, StepsWithAnEmptyBarrier) {
  auto core = LoadedCore();
  const double before = core.simulation_time();
  EXPECT_TRUE(core.OpenTick().empty());
  EXPECT_TRUE(core.tick_open());
  EXPECT_TRUE(core.StepIfBarrierComplete());
  EXPECT_EQ(core.current_tick_id(), 1U);
  EXPECT_DOUBLE_EQ(core.simulation_time(), before + core.timestep());
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
}

TEST(SimulationCoreTest, LoadsModelFromInitialKeyframeAndClearsControl) {
  mujoco_sim::core::SimulationCore core;
  mujoco_sim::core::ModelLoadOptions options;
  options.initial_keyframe = "ready";
  std::string error;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, options, &error))
      << error;

  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->time, 1.25);
  EXPECT_DOUBLE_EQ(snapshot->data()->qpos[0], 0.5);
  EXPECT_DOUBLE_EQ(snapshot->data()->qvel[0], -0.25);
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.0);
  EXPECT_DOUBLE_EQ(snapshot->data()->sensordata[0], 0.5);
  EXPECT_DOUBLE_EQ(core.simulation_time(), 1.25);
}

TEST(SimulationCoreTest, EmptyModelLoadOptionsPreserveDefaultResetBehavior) {
  mujoco_sim::core::SimulationCore legacy_core;
  mujoco_sim::core::SimulationCore options_core;
  std::string error;
  ASSERT_TRUE(legacy_core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error))
      << error;
  ASSERT_TRUE(options_core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL,
                                    mujoco_sim::core::ModelLoadOptions{},
                                    &error))
      << error;

  auto legacy_snapshot = legacy_core.CreateSnapshot(&error);
  ASSERT_NE(legacy_snapshot, nullptr) << error;
  auto options_snapshot = options_core.CreateSnapshot(&error);
  ASSERT_NE(options_snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(options_snapshot->data()->time,
                   legacy_snapshot->data()->time);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->qpos[0],
                   legacy_snapshot->data()->qpos[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->qvel[0],
                   legacy_snapshot->data()->qvel[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->ctrl[0],
                   legacy_snapshot->data()->ctrl[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->sensordata[0],
                   legacy_snapshot->data()->sensordata[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->time, 0.0);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->qpos[0], 0.0);
}

TEST(SimulationCoreTest, MissingInitialKeyframeLeavesLoadedModelUnchanged) {
  auto core = LoadedCore();
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);
  const auto epoch = core.epoch();
  const auto tick_id = core.current_tick_id();
  const auto simulation_time = core.simulation_time();

  mujoco_sim::core::ModelLoadOptions options;
  options.initial_keyframe = "missing_pose";
  EXPECT_FALSE(core.LoadModel(MUJOCO_SIM_CORE_DUAL_TEST_MODEL, options, &error));
  EXPECT_EQ(error,
            "initial keyframe 'missing_pose' was not found in MuJoCo model");
  EXPECT_TRUE(core.IsModelLoaded());
  EXPECT_EQ(core.epoch(), epoch);
  EXPECT_EQ(core.current_tick_id(), tick_id);
  EXPECT_DOUBLE_EQ(core.simulation_time(), simulation_time);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
  EXPECT_TRUE(core.tick_open());
  EXPECT_EQ(core.active_controller_count(), 1);
  ASSERT_EQ(core.commands().size(), 1);
  EXPECT_EQ(core.commands()[0].name(), "hinge_motor");
  EXPECT_TRUE(core.UpdateSnapshot(snapshot.get(), &error)) << error;

  const auto result = core.SubmitCommit(CommitFor(registration, tick_id, 0.2));
  EXPECT_EQ(result.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(result.stepped);
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

TEST(SimulationCoreTest, CreatesIsolatedVisualizationSnapshot) {
  auto core = LoadedCore();
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  ASSERT_NE(snapshot->model(), nullptr);
  ASSERT_NE(snapshot->data(), nullptr);

  snapshot->data()->qpos[0] = 0.75;
  auto independent = core.CreateSnapshot(&error);
  ASSERT_NE(independent, nullptr) << error;
  EXPECT_NE(independent->data()->qpos[0], snapshot->data()->qpos[0]);

  ASSERT_TRUE(core.UpdateSnapshot(snapshot.get(), &error)) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->qpos[0], independent->data()->qpos[0]);
}

TEST(SimulationCoreTest, RefreshesVisualizationSnapshotAfterStep) {
  auto core = LoadedCore();
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.2)).stepped);
  ASSERT_TRUE(core.UpdateSnapshot(snapshot.get(), &error)) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->time, core.simulation_time());
}

TEST(SimulationCoreTest, RejectsSnapshotFromAnotherCoreOrModelGeneration) {
  auto first = LoadedCore();
  auto second = LoadedCore();
  std::string error;
  auto snapshot = first.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  EXPECT_FALSE(second.UpdateSnapshot(snapshot.get(), &error));
  EXPECT_NE(error.find("does not belong"), std::string::npos);

  ASSERT_TRUE(first.LoadModel(MUJOCO_SIM_CORE_DUAL_TEST_MODEL, &error))
      << error;
  EXPECT_FALSE(first.UpdateSnapshot(snapshot.get(), &error));
  EXPECT_NE(error.find("does not belong"), std::string::npos);
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

TEST(SimulationCoreTest, DuplicatePayloadMismatchOnlyErrorsItsController) {
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
  EXPECT_TRUE(different_duplicate.controller_state_changed);
  EXPECT_EQ(different_duplicate.controller_state, mujoco_sim::v1::ERROR);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
  EXPECT_EQ(ControllerStatusFor(core, "first").state(),
            mujoco_sim::v1::ERROR);
  EXPECT_EQ(ControllerStatusFor(core, "second").state(),
            mujoco_sim::v1::ACTIVE);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.1)).stepped);
}

TEST(SimulationCoreTest, InvalidCommandOnlyErrorsItsController) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenTick().size(), 2);

  auto commit =
      CommitFor(first, 0, std::numeric_limits<double>::quiet_NaN());
  const auto result = core.SubmitCommit(commit);
  EXPECT_EQ(result.code, mujoco_sim::v1::INVALID_COMMAND);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
  EXPECT_EQ(ControllerStatusFor(core, "first").state(),
            mujoco_sim::v1::ERROR);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.2)).stepped);

  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.0);
}

TEST(SimulationCoreTest, IdentityMismatchesDoNotChangeControllerState) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);

  auto wrong_session = CommitFor(registration, 0, 0.0);
  wrong_session.mutable_header()->set_session_id(registration.session_id() + 1);
  EXPECT_EQ(core.SubmitCommit(wrong_session).code,
            mujoco_sim::v1::INVALID_SESSION);
  auto wrong_epoch = CommitFor(registration, 0, 0.0);
  wrong_epoch.mutable_header()->set_epoch(registration.epoch() + 1);
  EXPECT_EQ(core.SubmitCommit(wrong_epoch).code, mujoco_sim::v1::INVALID_EPOCH);
  EXPECT_EQ(core.SubmitCommit(CommitFor(registration, 1, 0.0)).code,
            mujoco_sim::v1::INVALID_TICK);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
  EXPECT_EQ(ControllerStatusFor(core, "arm").state(),
            mujoco_sim::v1::ACTIVE);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).stepped);
}

TEST(SimulationCoreTest, RegistersAndActivatesWhileRunning) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  EXPECT_TRUE(core.OpenTick().empty());
  EXPECT_TRUE(core.StepIfBarrierComplete());
  EXPECT_EQ(core.current_tick_id(), 1U);

  const auto first = RegisterAndActivate(
      &core, BasicRegisterRequest("first", "hinge_motor"));
  ASSERT_EQ(core.OpenTick().size(), 1);
  const auto second_registration =
      core.RegisterController(BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(second_registration.code(), mujoco_sim::v1::OK);
  mujoco_sim::v1::ActivateControllerRequest activate;
  activate.set_controller_id("second");
  activate.set_session_id(second_registration.session_id());
  activate.set_epoch(second_registration.epoch());
  EXPECT_EQ(core.ActivateController(activate).code(), mujoco_sim::v1::OK);
  EXPECT_EQ(core.OpenTick().size(), 1);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(first, 1, 0.1)).stepped);
  EXPECT_EQ(core.OpenTick().size(), 2);
}

TEST(SimulationCoreTest, UsesNegotiatedMinorForControllerTicks) {
  auto core = LoadedCore();
  auto request = BasicRegisterRequest();
  request.set_protocol_minor(0);
  const auto registration = RegisterAndActivate(&core, request);
  ASSERT_EQ(registration.effective_protocol_minor(), 0U);
  ASSERT_EQ(core.OpenTick().size(), 1U);
  EXPECT_TRUE(core.CheckControllerDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .stepped);

  const auto recovery_ticks = core.OpenTick();
  ASSERT_EQ(recovery_ticks.size(), 1U);
  EXPECT_EQ(recovery_ticks[0].header().protocol_minor(), 0U);
  EXPECT_EQ(recovery_ticks[0].controller_state(), mujoco_sim::v1::ACTIVE);
  auto recovery = CommitFor(registration, 1, 0.1);
  recovery.mutable_header()->set_protocol_minor(0);
  const auto accepted = core.SubmitCommit(recovery);
  EXPECT_EQ(accepted.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(accepted.recovery_pending);
  const auto duplicate = core.SubmitCommit(recovery);
  EXPECT_EQ(duplicate.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(duplicate.duplicate);
}

TEST(SimulationCoreTest, UnregistersRequiredControllerDuringOpenTick) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenTick().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.2)).stepped);

  mujoco_sim::v1::UnregisterControllerRequest request;
  request.set_controller_id("second");
  request.set_session_id(second.session_id());
  request.set_epoch(second.epoch());
  EXPECT_EQ(core.UnregisterController(request).code(), mujoco_sim::v1::OK);
  EXPECT_EQ(core.current_tick_id(), 1U);
  EXPECT_EQ(core.MakeStatus().controllers_size(), 1);
}

TEST(SimulationCoreTest, ExpiresProvisionalControllerAndReleasesActuator) {
  mujoco_sim::core::CoreOptions options;
  options.activation_timeout_ms = 10;
  mujoco_sim::core::SimulationCore core(options);
  std::string error;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  EXPECT_EQ(core.RegisterController(BasicRegisterRequest("first")).code(),
            mujoco_sim::v1::OK);
  const auto deadlines = core.CheckControllerDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_EQ(deadlines.transitions[0].old_state, mujoco_sim::v1::PROVISIONAL);
  EXPECT_EQ(deadlines.transitions[0].new_state,
            mujoco_sim::v1::UNREGISTERED);
  EXPECT_EQ(core.RegisterController(BasicRegisterRequest("second")).code(),
            mujoco_sim::v1::OK);
}

TEST(SimulationCoreTest, TimesOutOneControllerAndContinuesBarrier) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenTick().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.2)).stepped);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.3)).stepped);
  ASSERT_EQ(core.OpenTick().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 1, 0.2)).stepped);

  const auto deadlines = core.CheckControllerDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_EQ(deadlines.transitions[0].controller_id, "second");
  EXPECT_EQ(deadlines.transitions[0].new_state, mujoco_sim::v1::TIMED_OUT);
  EXPECT_TRUE(deadlines.stepped);
  EXPECT_EQ(core.server_state(), mujoco_sim::v1::RUNNING);
  const auto timed_out = ControllerStatusFor(core, "second");
  EXPECT_EQ(timed_out.state(), mujoco_sim::v1::TIMED_OUT);
  EXPECT_EQ(timed_out.timeout_count(), 1U);
  EXPECT_FALSE(timed_out.required_for_barrier());
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[1], 0.0);
  EXPECT_EQ(core.OpenTick().size(), 2);
}

TEST(SimulationCoreTest, KeepsIssuedTickPayloadImmutableAfterTimeout) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  auto first_request = BasicRegisterRequest("first", "hinge_motor");
  first_request.set_timeout_ms(10);
  const auto first = RegisterAndActivate(&core, first_request);
  auto second_request = BasicRegisterRequest("second", "second_motor");
  second_request.set_timeout_ms(100);
  const auto second = RegisterAndActivate(&core, second_request);
  const auto issued = core.OpenTick();
  ASSERT_EQ(issued.size(), 2);
  const auto first_tick = std::find_if(
      issued.begin(), issued.end(),
      [](const auto& tick) { return tick.controller_id() == "first"; });
  ASSERT_NE(first_tick, issued.end());
  EXPECT_EQ(first_tick->controller_state(), mujoco_sim::v1::ACTIVE);

  const auto deadlines = core.CheckControllerDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20));
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_FALSE(deadlines.stepped);
  const auto retransmitted = core.OpenTick();
  const auto repeated_first = std::find_if(
      retransmitted.begin(), retransmitted.end(),
      [](const auto& tick) { return tick.controller_id() == "first"; });
  ASSERT_NE(repeated_first, retransmitted.end());
  EXPECT_EQ(first_tick->SerializeAsString(), repeated_first->SerializeAsString());
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.1)).stepped);
  (void)first;
}

TEST(SimulationCoreTest, RecoversTimedOutControllerOnNextTick) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenTick().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.1)).stepped);
  EXPECT_TRUE(core.CheckControllerDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .stepped);

  ASSERT_EQ(core.OpenTick().size(), 2);
  const auto recovery = core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(recovery.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(recovery.recovery_pending);
  EXPECT_FALSE(recovery.stepped);
  const auto duplicate = core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(duplicate.code, mujoco_sim::v1::OK);
  EXPECT_TRUE(duplicate.duplicate);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(first, 1, 0.1)).stepped);
  ASSERT_EQ(core.OpenTick().size(), 2);
  EXPECT_EQ(ControllerStatusFor(core, "second").state(),
            mujoco_sim::v1::ACTIVE);
  EXPECT_TRUE(ControllerStatusFor(core, "second").required_for_barrier());
}

TEST(SimulationCoreTest, KeepsHeartbeatTickMonotonic) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).stepped);
  ASSERT_EQ(core.OpenTick().size(), 1U);

  mujoco_sim::v1::Heartbeat heartbeat;
  heartbeat.set_controller_id(registration.controller_id());
  heartbeat.mutable_header()->set_server_name("default");
  heartbeat.mutable_header()->set_session_id(registration.session_id());
  heartbeat.mutable_header()->set_epoch(registration.epoch());
  heartbeat.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  heartbeat.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  heartbeat.mutable_header()->set_tick_id(1);
  core.AcceptHeartbeat(heartbeat);
  heartbeat.mutable_header()->set_tick_id(0);
  core.AcceptHeartbeat(heartbeat);

  EXPECT_EQ(ControllerStatusFor(core, "arm").last_tick_id(), 1U);
}

TEST(SimulationCoreTest, ReplacesTimedOutControllerWithNewSession) {
  auto core = LoadedCore();
  const auto old_registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);
  EXPECT_TRUE(core.CheckControllerDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .stepped);

  const auto replacement = core.RegisterController(BasicRegisterRequest());
  ASSERT_EQ(replacement.code(), mujoco_sim::v1::OK);
  EXPECT_NE(replacement.session_id(), old_registration.session_id());
  EXPECT_NE(replacement.message().find("replaced"), std::string::npos);
  EXPECT_EQ(core.SubmitCommit(CommitFor(old_registration, 0, 0.1)).code,
            mujoco_sim::v1::INVALID_SESSION);
  EXPECT_EQ(ControllerStatusFor(core, "arm").state(),
            mujoco_sim::v1::PROVISIONAL);
}

TEST(SimulationCoreTest, CanDisableTimedOutControllerReplacement) {
  mujoco_sim::core::CoreOptions options;
  options.allow_timed_out_adapter_replacement = false;
  mujoco_sim::core::SimulationCore core(options);
  std::string error;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1);
  EXPECT_TRUE(core.CheckControllerDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .stepped);
  EXPECT_EQ(core.RegisterController(BasicRegisterRequest()).code(),
            mujoco_sim::v1::ALREADY_EXISTS);
}

TEST(SimulationCoreTest, FatalErrorUpdatesControllerStateTimestamp) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenTick().size(), 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.0)).stepped);
  ASSERT_EQ(core.current_tick_id(), 1U);

  core.EnterFatalError("fatal transport error");
  const auto status = ControllerStatusFor(core, "arm");
  EXPECT_EQ(status.state(), mujoco_sim::v1::ERROR);
  EXPECT_EQ(status.state_since_tick_id(), 1U);
  EXPECT_EQ(status.message(), "fatal transport error");
}

}  // namespace
