#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

namespace {
using namespace std::chrono_literals;
using mujoco_sim::core::AdapterState;
using mujoco_sim::core::ControllerCommand;
using mujoco_sim::core::SimulationCore;

auto T(std::chrono::milliseconds value) {
  return std::chrono::steady_clock::time_point(value);
}

auto RegisterController(SimulationCore* core, std::string id = "arm") {
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id(std::move(id));
  request.set_role(mujoco_sim::core::CONTROLLER);
  request.add_observation_names("hinge_pos");
  request.add_command_names("hinge_motor");
  return core->RegisterAdapter(request, std::chrono::steady_clock::time_point{});
}

auto RegisterObserver(SimulationCore* core, std::string id, std::uint32_t every_n_steps = 1,
                      std::uint32_t timeout_ms = 0) {
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id(std::move(id));
  request.set_role(mujoco_sim::core::OBSERVER);
  request.add_observation_names("hinge_pos");
  request.mutable_delivery()->set_every_n_steps(every_n_steps);
  request.set_session_timeout_ms(timeout_ms);
  return core->RegisterAdapter(request, std::chrono::steady_clock::time_point{});
}

void Activate(SimulationCore* core, const mujoco_sim::core::RegisterAdapterResponse& registration,
              std::chrono::steady_clock::time_point now = {}) {
  mujoco_sim::core::ActivateAdapterRequest request;
  request.set_session_id(registration.session_id());
  request.set_epoch(registration.epoch());
  ASSERT_EQ(core->ActivateAdapter(request, now).response.code(), mujoco_sim::core::OK);
}

TEST(AsyncSimulationCoreTest, StepsContinuouslyWithoutControllers) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  for (int i = 0; i < 10; ++i)
    ASSERT_TRUE(core.Step().step_completed);
  EXPECT_EQ(core.state().current_tick_id, 10U);
  EXPECT_DOUBLE_EQ(core.state().simulation_time_s, 0.01);
}

TEST(AsyncSimulationCoreTest, ManualPauseIsNormalIdempotentAndDoesNotStep) {
  SimulationCore core;
  EXPECT_FALSE(core.SetPaused(true).response.applied);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);

  const auto before = core.state();
  auto pause = core.SetPaused(true);
  EXPECT_TRUE(pause.response.applied);
  EXPECT_TRUE(pause.response.state_changed);
  EXPECT_EQ(pause.effects.status_urgency,
            mujoco_sim::core::EngineEffects::StatusUrgency::kLifecycle);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::PAUSED);
  EXPECT_EQ(core.MakeStatus().message, "paused by local GUI");
  EXPECT_FALSE(core.Step().step_completed);
  EXPECT_EQ(core.state().current_tick_id, before.current_tick_id);
  EXPECT_DOUBLE_EQ(core.state().simulation_time_s, before.simulation_time_s);

  pause = core.SetPaused(true);
  EXPECT_TRUE(pause.response.applied);
  EXPECT_FALSE(pause.response.state_changed);
  EXPECT_EQ(pause.effects.status_urgency, mujoco_sim::core::EngineEffects::StatusUrgency::kNone);

  const auto run = core.SetPaused(false);
  EXPECT_TRUE(run.response.applied);
  EXPECT_TRUE(run.response.state_changed);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
  EXPECT_TRUE(core.MakeStatus().message.empty());
  EXPECT_TRUE(core.Step().step_completed);

  core.MarkFailed("failed");
  EXPECT_FALSE(core.SetPaused(false).response.applied);
  EXPECT_FALSE(core.Reset(T(1ms)).response.applied);
}

TEST(AsyncSimulationCoreTest, ResetRestoresDefaultAndConfiguredKeyframeWithoutTickRollback) {
  for (const std::string keyframe : {std::string{}, std::string{"ready"}}) {
    SimulationCore core;
    mujoco_sim::core::ModelLoadOptions options;
    options.initial_keyframe = keyframe;
    ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, options).response.loaded);
    const auto epoch = core.state().epoch;

    const std::vector<mujoco_sim::core::InteractiveActuatorWrite> controls = {
        {.actuator_id = 0, .value = 0.6}};
    const std::vector<mujoco_sim::core::InteractiveBodyWrench> wrenches = {
        {.body_id = 1, .force = {1.0, 2.0, 3.0}, .torque = {4.0, 5.0, 6.0}}};
    ASSERT_TRUE(core.Step({.actuator_writes = controls,
                           .body_wrenches = wrenches,
                           .replace_body_wrenches = true})
                    .step_completed);
    const auto tick_before_reset = core.state().current_tick_id;

    const auto reset = core.Reset(T(20ms));
    ASSERT_TRUE(reset.response.applied) << reset.response.message;
    EXPECT_FALSE(reset.response.state_changed);
    EXPECT_EQ(core.state().epoch, epoch);
    EXPECT_EQ(core.state().current_tick_id, tick_before_reset);
    EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);

    const auto snapshot = core.CreateSnapshot(nullptr);
    ASSERT_NE(snapshot, nullptr);
    EXPECT_DOUBLE_EQ(snapshot->data()->time, keyframe.empty() ? 0.0 : 1.25);
    EXPECT_DOUBLE_EQ(snapshot->data()->qpos[0], keyframe.empty() ? 0.0 : 0.5);
    EXPECT_DOUBLE_EQ(snapshot->data()->qvel[0], keyframe.empty() ? 0.0 : -0.25);
    EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.0);
    for (int index = 0; index < 6; ++index) {
      EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 + index], 0.0);
    }

    ASSERT_TRUE(core.Step().step_completed);
    EXPECT_EQ(core.state().current_tick_id, tick_before_reset + 1);
  }
}

TEST(AsyncSimulationCoreTest, ResetPreservesPausedModeAndClearsOnlineControllerCommand) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);

  ControllerCommand command{{registration.response.session_id(), registration.response.epoch()},
                            registration.response.layout().fingerprint,
                            1,
                            {0.75}};
  ASSERT_EQ(core.SubmitCommand(command, T(1ms)).response.code, mujoco_sim::core::OK);
  const auto initial_step = core.Step();
  ASSERT_TRUE(initial_step.step_completed);
  ASSERT_EQ(initial_step.step_completed->controller_states.size(), 1U);
  EXPECT_EQ(initial_step.step_completed->controller_states.front().sequence, 1U);
  ASSERT_TRUE(core.SetPaused(true).response.applied);
  const auto tick = core.state().current_tick_id;

  const auto reset = core.Reset(T(10ms));
  ASSERT_TRUE(reset.response.applied);
  ASSERT_EQ(reset.effects.adapter_changes.size(), 1U);
  EXPECT_EQ(reset.effects.adapter_changes.front().kind,
            mujoco_sim::core::AdapterChangeKind::kSimulationReset);
  EXPECT_EQ(reset.effects.adapter_changes.front().old_state, mujoco_sim::core::ONLINE);
  EXPECT_EQ(reset.effects.adapter_changes.front().new_state, mujoco_sim::core::ACTIVE_NO_COMMAND);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::PAUSED);
  EXPECT_EQ(core.state().current_tick_id, tick);
  const auto status = core.MakeStatus();
  EXPECT_EQ(status.message, "paused by local GUI");
  ASSERT_EQ(status.adapters.size(), 1U);
  EXPECT_EQ(status.adapters.front().state, mujoco_sim::core::ACTIVE_NO_COMMAND);
  EXPECT_EQ(status.adapters.front().last_command_sequence, 1U);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.0);

  EXPECT_EQ(core.SubmitCommand(command, T(11ms)).response.code, mujoco_sim::core::INVALID_COMMAND);
  command.sequence = 2;
  command.commands = {0.25};
  EXPECT_EQ(core.SubmitCommand(command, T(12ms)).response.code, mujoco_sim::core::OK);
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::ONLINE);
  EXPECT_EQ(RegisterController(&core, "competitor").response.code(),
            mujoco_sim::core::RESOURCE_CONFLICT);
  EXPECT_FALSE(core.Step().step_completed);
  ASSERT_TRUE(core.SetPaused(false).response.applied);
  const auto resumed = core.Step();
  ASSERT_TRUE(resumed.step_completed);
  ASSERT_EQ(resumed.step_completed->controller_states.size(), 1U);
  EXPECT_EQ(resumed.step_completed->controller_states.front().sequence, 2U);
  EXPECT_EQ(resumed.step_completed->controller_states.front().identity.session.session_id,
            registration.response.session_id());
  EXPECT_EQ(resumed.step_completed->controller_states.front().identity.layout,
            registration.response.layout().fingerprint);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.25);
}

TEST(AsyncSimulationCoreTest, ResetPreservesProvisionalAndRestartsActiveNoCommandLiveness) {
  mujoco_sim::core::CoreOptions options;
  options.command_timeout_ms = 10;
  SimulationCore core(options);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);

  ASSERT_TRUE(core.Reset(T(5ms)).response.applied);
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::PROVISIONAL);
  Activate(&core, registration.response, T(6ms));
  ASSERT_TRUE(core.Reset(T(10ms)).response.applied);
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::ACTIVE_NO_COMMAND);
  core.CheckDeadlines(T(19ms));
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::ACTIVE_NO_COMMAND);
  core.CheckDeadlines(T(21ms));
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::OFFLINE);
}

TEST(AsyncSimulationCoreTest, ResetPreservesOfflineControllerDiagnosticsAndSequence) {
  mujoco_sim::core::CoreOptions options;
  options.command_timeout_ms = 10;
  SimulationCore core(options);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);
  ControllerCommand command{{registration.response.session_id(), registration.response.epoch()},
                            registration.response.layout().fingerprint,
                            1,
                            {0.75}};
  ASSERT_EQ(core.SubmitCommand(command, T(1ms)).response.code, mujoco_sim::core::OK);
  ASSERT_TRUE(core.Step().step_completed);
  core.CheckDeadlines(T(12ms));
  const auto before = core.MakeStatus().adapters.front();
  ASSERT_EQ(before.state, mujoco_sim::core::OFFLINE);

  ASSERT_TRUE(core.Reset(T(20ms)).response.applied);
  const auto after = core.MakeStatus().adapters.front();
  EXPECT_EQ(after.state, mujoco_sim::core::OFFLINE);
  EXPECT_EQ(after.message, before.message);
  EXPECT_EQ(after.timeout_count, before.timeout_count);
  EXPECT_EQ(after.state_since_tick_id, before.state_since_tick_id);
  EXPECT_EQ(after.last_command_sequence, 1U);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.0);
  EXPECT_EQ(core.SubmitCommand(command, T(21ms)).response.code, mujoco_sim::core::INVALID_COMMAND);
}

TEST(AsyncSimulationCoreTest, ResetPreservesControllerErrorAndClearsCommandCache) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);
  ControllerCommand command{{registration.response.session_id(), registration.response.epoch()},
                            registration.response.layout().fingerprint,
                            1,
                            {0.75}};
  ASSERT_EQ(core.SubmitCommand(command, T(1ms)).response.code, mujoco_sim::core::OK);
  ASSERT_TRUE(core.Step().step_completed);
  mujoco_sim::core::ControllerFault fault;
  fault.session = command.session;
  fault.code = mujoco_sim::core::FaultCode::kControllerError;
  fault.message = "controller failed";
  ASSERT_EQ(core.SubmitFault(fault).response.code, mujoco_sim::core::OK);
  const auto before = core.MakeStatus().adapters.front();
  ASSERT_EQ(before.state, mujoco_sim::core::ERROR);

  ASSERT_TRUE(core.Reset(T(20ms)).response.applied);
  const auto after = core.MakeStatus().adapters.front();
  EXPECT_EQ(after.state, mujoco_sim::core::ERROR);
  EXPECT_EQ(after.message, before.message);
  EXPECT_EQ(after.last_command_sequence, before.last_command_sequence);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.0);
}

TEST(AsyncSimulationCoreTest, ResetRestartsObserverDeliveryWithoutSequenceRollback) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterObserver(&core, "observer", 3);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);

  auto step = core.Step();
  ASSERT_EQ(step.step_completed->observer_frames.size(), 1U);
  EXPECT_EQ(step.step_completed->observer_frames.front().sequence, 1U);
  ASSERT_TRUE(core.Reset(T(5ms)).response.applied);
  step = core.Step();
  ASSERT_EQ(step.step_completed->observer_frames.size(), 1U);
  EXPECT_EQ(step.step_completed->observer_frames.front().sequence, 2U);
}

TEST(AsyncSimulationCoreTest, AppliesLatestCommandAtNextStepAndKeepsLast) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_session_id(registration.response.session_id());
  activate.set_epoch(registration.response.epoch());
  ASSERT_EQ(core.ActivateAdapter(activate, {}).response.state(),
            mujoco_sim::core::ACTIVE_NO_COMMAND);

  ControllerCommand command;
  command.session = {registration.response.session_id(), registration.response.epoch()};
  command.layout = registration.response.layout().fingerprint;
  command.sequence = 1;
  command.commands = {0.25};
  ASSERT_EQ(core.SubmitCommand(command, T(1ms)).response.code, mujoco_sim::core::OK);
  command.sequence = 2;
  command.commands = {0.75};
  ASSERT_EQ(core.SubmitCommand(command, T(2ms)).response.code, mujoco_sim::core::OK);
  ASSERT_TRUE(core.Step().step_completed);
  auto snapshot = core.CreateSnapshot(nullptr);
  ASSERT_NE(snapshot, nullptr);
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.75);
  ASSERT_TRUE(core.Step().step_completed);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.75);
}

TEST(AsyncSimulationCoreTest, AppliesInteractiveControlAfterControllerAsOneShotWrite) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);

  ControllerCommand command{{registration.response.session_id(), registration.response.epoch()},
                            registration.response.layout().fingerprint,
                            1,
                            {0.75}};
  ASSERT_EQ(core.SubmitCommand(command, T(1ms)).response.code, mujoco_sim::core::OK);
  const std::vector<mujoco_sim::core::InteractiveActuatorWrite> gui_writes = {
      {.actuator_id = 0, .value = 0.2},
      {.actuator_id = 0, .value = 0.4},
  };
  ASSERT_TRUE(core.Step({.actuator_writes = gui_writes}).step_completed);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.4);

  ASSERT_TRUE(core.Step().step_completed);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.4);

  command.sequence = 2;
  command.commands = {0.6};
  ASSERT_EQ(core.SubmitCommand(command, T(2ms)).response.code, mujoco_sim::core::OK);
  ASSERT_TRUE(core.Step().step_completed);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 0.6);
}

TEST(AsyncSimulationCoreTest, IgnoresInvalidInteractiveControlsWithoutClampingValidValues) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const std::vector<mujoco_sim::core::InteractiveActuatorWrite> writes = {
      {.actuator_id = -1, .value = 0.1},
      {.actuator_id = 4, .value = 0.2},
      {.actuator_id = 0, .value = std::numeric_limits<double>::quiet_NaN()},
      {.actuator_id = 0, .value = 2.5},
  };
  ASSERT_TRUE(core.Step({.actuator_writes = writes}).step_completed);
  EXPECT_DOUBLE_EQ(core.CreateSnapshot(nullptr)->data()->ctrl[0], 2.5);
}

TEST(AsyncSimulationCoreTest, ReplacesInteractiveWrenchesAndClearsPreviousBodies) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_DUAL_TEST_MODEL).response.loaded);
  auto snapshot = core.CreateSnapshot(nullptr);
  ASSERT_NE(snapshot, nullptr);
  const int first = mj_name2id(snapshot->model(), mjOBJ_BODY, "first_link");
  const int second = mj_name2id(snapshot->model(), mjOBJ_BODY, "second_link");
  const int static_body = mj_name2id(snapshot->model(), mjOBJ_BODY, "static_link");
  ASSERT_GT(first, 0);
  ASSERT_GT(second, 0);
  ASSERT_GT(static_body, 0);

  const std::vector<mujoco_sim::core::InteractiveBodyWrench> first_frame = {
      {.body_id = first, .force = {1.0, 2.0, 3.0}, .torque = {4.0, 5.0, 6.0}},
  };
  ASSERT_TRUE(
      core.Step({.body_wrenches = first_frame, .replace_body_wrenches = true}).step_completed);
  snapshot = core.CreateSnapshot(nullptr);
  for (int index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * first + index], index + 1.0);
  }

  const std::vector<mujoco_sim::core::InteractiveBodyWrench> second_frame = {
      {.body_id = second, .force = {-1.0, 0.0, 0.0}, .torque = {0.0, 0.0, 2.0}},
      {.body_id = second, .force = {-2.0, 0.0, 0.0}, .torque = {0.0, 0.0, 3.0}},
  };
  ASSERT_TRUE(
      core.Step({.body_wrenches = second_frame, .replace_body_wrenches = true}).step_completed);
  snapshot = core.CreateSnapshot(nullptr);
  for (int index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * first + index], 0.0);
  }
  EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * second], -2.0);
  EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * second + 5], 3.0);

  ASSERT_TRUE(core.Step({.body_wrenches = {}, .replace_body_wrenches = true}).step_completed);
  snapshot = core.CreateSnapshot(nullptr);
  for (int index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * second + index], 0.0);
  }

  ASSERT_TRUE(
      core.Step({.body_wrenches = second_frame, .replace_body_wrenches = true}).step_completed);

  const std::vector<mujoco_sim::core::InteractiveBodyWrench> invalid_frame = {
      {.body_id = 0, .force = {9.0, 0.0, 0.0}},
      {.body_id = static_body, .force = {9.0, 0.0, 0.0}},
      {.body_id = second, .force = {std::numeric_limits<double>::infinity(), 0.0, 0.0}},
  };
  ASSERT_TRUE(
      core.Step({.body_wrenches = invalid_frame, .replace_body_wrenches = true}).step_completed);
  snapshot = core.CreateSnapshot(nullptr);
  for (int index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * second + index], 0.0);
  }

  ASSERT_TRUE(
      core.Step({.body_wrenches = first_frame, .replace_body_wrenches = true}).step_completed);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_DUAL_TEST_MODEL).response.loaded);
  snapshot = core.CreateSnapshot(nullptr);
  for (int body = 0; body < snapshot->model()->nbody; ++body) {
    for (int index = 0; index < 6; ++index) {
      EXPECT_DOUBLE_EQ(snapshot->data()->xfrc_applied[6 * body + index], 0.0);
    }
  }
}

TEST(AsyncSimulationCoreTest, RejectsStaleInvalidAndTimesOutWithoutError) {
  mujoco_sim::core::CoreOptions options;
  options.command_timeout_ms = 10;
  SimulationCore core(options);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  auto registration = RegisterController(&core);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_session_id(registration.response.session_id());
  activate.set_epoch(registration.response.epoch());
  core.ActivateAdapter(activate, {});
  ControllerCommand command{{registration.response.session_id(), registration.response.epoch()},
                            registration.response.layout().fingerprint,
                            1,
                            {0.5}};
  ASSERT_EQ(core.SubmitCommand(command, T(1ms)).response.code, mujoco_sim::core::OK);
  EXPECT_EQ(core.SubmitCommand(command, T(2ms)).response.code, mujoco_sim::core::INVALID_COMMAND);
  command.sequence = 2;
  command.commands = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_EQ(core.SubmitCommand(command, T(3ms)).response.code, mujoco_sim::core::INVALID_COMMAND);
  core.CheckDeadlines(T(12ms));
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::OFFLINE);
  command.commands = {0.2};
  EXPECT_EQ(core.SubmitCommand(command, T(13ms)).response.code, mujoco_sim::core::OK);
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::ONLINE);
}

TEST(AsyncSimulationCoreTest, PublishesNoInitialStateAndStopsInError) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  auto registration = RegisterController(&core);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_session_id(registration.response.session_id());
  activate.set_epoch(registration.response.epoch());
  core.ActivateAdapter(activate, {});
  auto first = core.Step();
  ASSERT_EQ(first.step_completed->controller_states.size(), 1U);
  EXPECT_EQ(first.step_completed->controller_states.front().sequence, 1U);
  EXPECT_EQ(core.MakeStatus().adapters.front().last_observation_sequence, 0U);
  mujoco_sim::core::ControllerFault fault;
  fault.session = {registration.response.session_id(), registration.response.epoch()};
  fault.message = "failed";
  EXPECT_EQ(core.SubmitFault(fault).response.code, mujoco_sim::core::INVALID_ARGUMENT);
  fault.code = static_cast<mujoco_sim::core::FaultCode>(255);
  EXPECT_EQ(core.SubmitFault(fault).response.code, mujoco_sim::core::INVALID_ARGUMENT);
  fault.code = mujoco_sim::core::FaultCode::kControllerError;
  ASSERT_EQ(core.SubmitFault(fault).response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(core.Step().step_completed->controller_states.empty());
}

TEST(AsyncSimulationCoreTest, RejectsInvalidAdapterIdentifiers) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const std::vector<std::string> invalid_ids = {
      "bad/id", ".bad", "_bad", std::string(1, static_cast<char>(0xff)), std::string(65, 'a')};
  for (const auto& id : invalid_ids) {
    mujoco_sim::core::RegisterAdapterRequest request;
    request.set_adapter_id(id);
    request.set_role(mujoco_sim::core::CONTROLLER);
    request.add_command_names("hinge_motor");
    EXPECT_EQ(core.RegisterAdapter(request, {}).response.code(),
              mujoco_sim::core::INVALID_ARGUMENT);
  }
  EXPECT_EQ(RegisterController(&core, std::string(64, 'a')).response.code(), mujoco_sim::core::OK);
}

TEST(AsyncSimulationCoreTest, AdapterIdentifiersAreUniqueAcrossRoles) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  ASSERT_EQ(RegisterController(&core, "shared").response.code(), mujoco_sim::core::OK);
  EXPECT_EQ(RegisterObserver(&core, "shared").response.code(), mujoco_sim::core::ALREADY_EXISTS);

  ASSERT_EQ(RegisterObserver(&core, "observer").response.code(), mujoco_sim::core::OK);
  EXPECT_EQ(RegisterController(&core, "observer").response.code(),
            mujoco_sim::core::ALREADY_EXISTS);
}

TEST(AsyncSimulationCoreTest, ObserverPublishesFirstStepThenAtStepCadence) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterObserver(&core, "observer", 3);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);

  auto step = core.Step();
  ASSERT_EQ(step.step_completed->observer_frames.size(), 1U);
  EXPECT_EQ(step.step_completed->observer_frames.front().sequence, 1U);
  EXPECT_TRUE(core.Step().step_completed->observer_frames.empty());
  EXPECT_TRUE(core.Step().step_completed->observer_frames.empty());
  step = core.Step();
  ASSERT_EQ(step.step_completed->observer_frames.size(), 1U);
  EXPECT_EQ(step.step_completed->observer_frames.front().sequence, 2U);
}

TEST(AsyncSimulationCoreTest, ObserverPeriodCadenceStartsAtFirstDelivery) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id("observer");
  request.set_role(mujoco_sim::core::OBSERVER);
  request.add_observation_names("hinge_pos");
  request.mutable_delivery()->set_period_s(0.003);
  const auto registration = core.RegisterAdapter(request, {});
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);

  EXPECT_EQ(core.Step().step_completed->observer_frames.size(), 1U);
  EXPECT_TRUE(core.Step().step_completed->observer_frames.empty());
  EXPECT_TRUE(core.Step().step_completed->observer_frames.empty());
  EXPECT_EQ(core.Step().step_completed->observer_frames.size(), 1U);
}

TEST(AsyncSimulationCoreTest, ObserverHeartbeatCannotAcknowledgeFutureFrame) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto future = RegisterObserver(&core, "future", 1, 10);
  const auto current = RegisterObserver(&core, "current", 1, 10);
  ASSERT_EQ(future.response.code(), mujoco_sim::core::OK);
  ASSERT_EQ(current.response.code(), mujoco_sim::core::OK);
  Activate(&core, future.response);
  Activate(&core, current.response);
  ASSERT_EQ(core.Step().step_completed->observer_frames.size(), 2U);

  core.AcceptHeartbeat({{future.response.session_id(), future.response.epoch()}, 2}, T(9ms));
  core.AcceptHeartbeat({{current.response.session_id(), current.response.epoch()}, 1}, T(9ms));
  core.CheckDeadlines(T(11ms));
  const auto status = core.MakeStatus();
  ASSERT_EQ(status.adapters.size(), 2U);
  EXPECT_EQ(status.adapters[0].adapter_id, "current");
  EXPECT_EQ(status.adapters[0].state, mujoco_sim::core::ONLINE);
  EXPECT_EQ(status.adapters[1].adapter_id, "future");
  EXPECT_EQ(status.adapters[1].state, mujoco_sim::core::OFFLINE);
}

TEST(AsyncSimulationCoreTest, OfflineObserverStopsPublishing) {
  SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterObserver(&core, "observer", 1, 10);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  Activate(&core, registration.response);
  ASSERT_EQ(core.Step().step_completed->observer_frames.size(), 1U);

  core.CheckDeadlines(T(11ms));
  EXPECT_EQ(core.MakeStatus().adapters.front().state, mujoco_sim::core::OFFLINE);
  EXPECT_TRUE(core.Step().step_completed->observer_frames.empty());
}

TEST(AsyncSimulationCoreTest, OfflineSessionExpiresAndReleasesOwnership) {
  mujoco_sim::core::CoreOptions options;
  options.command_timeout_ms = 5;
  options.offline_session_expire_ms = 5;
  SimulationCore core(options);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  auto first = RegisterController(&core);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_session_id(first.response.session_id());
  activate.set_epoch(first.response.epoch());
  core.ActivateAdapter(activate, {});
  core.CheckDeadlines(T(6ms));
  auto expired = core.CheckDeadlines(T(12ms));
  ASSERT_EQ(expired.adapter_changes.size(), 1U);
  EXPECT_EQ(expired.adapter_changes.front().new_state, mujoco_sim::core::UNREGISTERED);
  EXPECT_EQ(RegisterController(&core, "replacement").response.code(), mujoco_sim::core::OK);
}

TEST(AsyncSimulationCoreTest, GrantsRateLimitedControllerOverridePolicy) {
  mujoco_sim::core::CoreOptions options;
  options.controller_state_max_rate_hz = 100.0;
  options.controller_state_delivery.set_every_n_steps(1);
  mujoco_sim::core::CoreOptions::ControllerOverride override;
  override.adapter_id = "arm";
  override.command_timeout_ms = 250;
  override.command_keepalive_ms = 50;
  override.state_delivery.set_period_s(0.002);
  override.offline_policy = mujoco_sim::core::ControlPolicy::kZero;
  options.controller_overrides.push_back(override);

  SimulationCore core(options);
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL).response.loaded);
  const auto registration = RegisterController(&core);
  ASSERT_EQ(registration.response.code(), mujoco_sim::core::OK);
  const auto& policy = registration.response.controller_policy;
  EXPECT_EQ(policy.command_timeout_ms, 250U);
  EXPECT_EQ(policy.command_keepalive_ms, 50U);
  EXPECT_EQ(policy.offline_policy, mujoco_sim::core::ControlPolicy::kZero);
  ASSERT_TRUE(policy.state_delivery.has_period_s());
  EXPECT_DOUBLE_EQ(policy.state_delivery.period_s(), 0.01);

  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_session_id(registration.response.session_id());
  activate.set_epoch(registration.response.epoch());
  ASSERT_EQ(core.ActivateAdapter(activate, {}).response.code(), mujoco_sim::core::OK);
  for (int step = 0; step < 9; ++step) {
    EXPECT_TRUE(core.Step().step_completed->controller_states.empty());
  }
  EXPECT_EQ(core.Step().step_completed->controller_states.size(), 1U);
}

}  // namespace
