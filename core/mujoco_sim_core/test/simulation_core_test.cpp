#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

namespace {

bool LoadModel(mujoco_sim::core::SimulationCore* core,
               const std::string& model_path, std::string* error) {
  auto result = core->LoadModel(model_path);
  *error = result.response.message;
  return result.response.loaded;
}

bool LoadModel(mujoco_sim::core::SimulationCore* core,
               const std::string& model_path,
               const mujoco_sim::core::ModelLoadOptions& options,
               std::string* error) {
  auto result = core->LoadModel(model_path, options);
  *error = result.response.message;
  return result.response.loaded;
}

mujoco_sim::core::SimulationCore LoadedCoreFrom(const std::string& model_path) {
  mujoco_sim::core::SimulationCore core;
  std::string error;
  EXPECT_TRUE(LoadModel(&core, model_path, &error)) << error;
  return core;
}

mujoco_sim::core::SimulationCore LoadedCore() {
  return LoadedCoreFrom(MUJOCO_SIM_CORE_TEST_MODEL);
}

std::string SensorMetadataModelPath() {
  return (std::filesystem::path(MUJOCO_SIM_CORE_TEST_MODEL).parent_path() /
          "sensor_metadata.xml")
      .string();
}

bool HasChange(const mujoco_sim::core::EngineEffects& effects,
               mujoco_sim::core::AdapterChangeKind kind) {
  return std::any_of(effects.adapter_changes.begin(),
                     effects.adapter_changes.end(),
                     [kind](const auto& change) { return change.kind == kind; });
}

const mujoco_sim::core::ResourceDescriptor* FindObservation(
    const mujoco_sim::core::SimulationCore& core, const std::string& name) {
  const auto observations = core.resources().observations;
  const auto found = std::find_if(
      observations.begin(), observations.end(),
      [&name](const auto& descriptor) { return descriptor.name() == name; });
  return found == observations.end() ? nullptr : &*found;
}

mujoco_sim::core::RegisterAdapterRequest BasicRegisterRequest(
    const std::string& adapter_id = "arm",
    const std::string& command_name = "hinge_motor") {
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id(adapter_id);
  request.set_role(mujoco_sim::core::CONTROLLER);
  request.add_observation_names("hinge_pos");
  request.add_observation_names("hinge_vel");
  request.add_command_names(command_name);
  request.set_commit_timeout_ms(25);
  request.set_session_timeout_ms(1000);
  return request;
}

mujoco_sim::core::AdapterCommit CommitFor(
    const mujoco_sim::core::RegisterAdapterResponse& registration,
  std::uint64_t tick_id, double value) {
  mujoco_sim::core::AdapterCommit commit;
  commit.identity.session = {registration.session_id(), registration.epoch()};
  commit.identity.tick_id = tick_id;
  commit.identity.layout = registration.layout().fingerprint;
  commit.commands.push_back(value);
  return commit;
}

mujoco_sim::core::RegisterAdapterResponse RegisterAndActivate(
    mujoco_sim::core::SimulationCore* core,
    const mujoco_sim::core::RegisterAdapterRequest& request) {
  auto registration = core->RegisterAdapter(request).response;
  EXPECT_EQ(registration.code(), mujoco_sim::core::OK);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_adapter_id(registration.adapter_id());
  activate.set_role(mujoco_sim::core::CONTROLLER);
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  const auto activated = core->ActivateAdapter(activate).response;
  EXPECT_EQ(activated.code(), mujoco_sim::core::OK);
  return registration;
}

mujoco_sim::core::RegisterAdapterRequest ObserverRequest(
    const std::string& observer_id = "state-observer") {
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id(observer_id);
  request.set_role(mujoco_sim::core::OBSERVER);
  request.add_observation_names("hinge_pos");
  request.add_observation_names("hinge_vel");
  request.set_session_timeout_ms(25);
  request.mutable_delivery()->set_every_n_steps(1);
  return request;
}

mujoco_sim::core::RegisterAdapterResponse RegisterAndActivateObserver(
    mujoco_sim::core::SimulationCore* core,
    const mujoco_sim::core::RegisterAdapterRequest& request) {
  auto registration = core->RegisterAdapter(request).response;
  EXPECT_EQ(registration.code(), mujoco_sim::core::OK);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_adapter_id(registration.adapter_id());
  activate.set_role(mujoco_sim::core::OBSERVER);
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  EXPECT_EQ(core->ActivateAdapter(activate).response.code(), mujoco_sim::core::OK);
  return registration;
}

mujoco_sim::core::AdapterStatus AdapterStatusFor(
    const mujoco_sim::core::SimulationCore& core,
    const std::string& adapter_id) {
  const auto server_status = core.MakeStatus();
  for (const auto& status : server_status.adapters) {
    if (status.adapter_id == adapter_id) return status;
  }
  ADD_FAILURE() << "missing adapter status for " << adapter_id;
  return {};
}

mujoco_sim::core::RegisterAdapterResponse RegisterAndActivate(
    mujoco_sim::core::SimulationCore* core) {
  return RegisterAndActivate(core, BasicRegisterRequest());
}

TEST(SimulationCoreTest, LoadsModelAndBuildsCatalogs) {
  auto core = LoadedCore();
  ASSERT_TRUE(core.IsModelLoaded());
  const auto resources = core.resources();
  EXPECT_EQ(resources.observations.size(), 3);
  ASSERT_EQ(resources.commands.size(), 1);
  EXPECT_EQ(resources.commands[0].name(), "hinge_motor");
  EXPECT_TRUE(resources.commands[0].has_ctrl_range());
  EXPECT_DOUBLE_EQ(resources.commands[0].ctrl_min(), -1.0);
  EXPECT_DOUBLE_EQ(resources.commands[0].ctrl_max(), 1.0);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
  EXPECT_GT(core.state().timestep_s, 0.0);
  EXPECT_EQ(resources.observations[0].mujoco_stage(), mjSTAGE_POS);
  EXPECT_GE(resources.observations[0].interval_period_s(), 0.0);
  const auto* hinge_force = FindObservation(core, "hinge_force");
  ASSERT_NE(hinge_force, nullptr);
  EXPECT_EQ(hinge_force->unit(), "N*m");
}

TEST(SimulationCoreTest, BuildsSensorTimingObjectFrameAndUnitMetadata) {
  const auto core = LoadedCoreFrom(SensorMetadataModelPath());
  ASSERT_TRUE(core.IsModelLoaded());

  const auto* slide_position = FindObservation(core, "slide_position");
  ASSERT_NE(slide_position, nullptr);
  EXPECT_EQ(slide_position->mujoco_stage(), mjSTAGE_POS);
  EXPECT_DOUBLE_EQ(slide_position->interval_period_s(), 0.01);
  EXPECT_DOUBLE_EQ(slide_position->interval_phase_s(), -0.002);
  EXPECT_DOUBLE_EQ(slide_position->delay_s(), 0.003);
  EXPECT_EQ(slide_position->object_type(), mjOBJ_JOINT);
  EXPECT_EQ(slide_position->object_name(), "slide");
  EXPECT_TRUE(slide_position->frame_id().empty());
  EXPECT_EQ(slide_position->unit(), "m");
  EXPECT_EQ(slide_position->interpolation(), "linear");

  const auto* referenced_position =
      FindObservation(core, "body_position_in_reference");
  ASSERT_NE(referenced_position, nullptr);
  EXPECT_EQ(referenced_position->object_type(), mjOBJ_BODY);
  EXPECT_EQ(referenced_position->object_name(), "measured_body");
  EXPECT_EQ(referenced_position->frame_id(), "reference_site");
  EXPECT_EQ(referenced_position->unit(), "m");
  EXPECT_EQ(referenced_position->interpolation(), "cubic");

  const auto* world_orientation =
      FindObservation(core, "body_orientation_world");
  ASSERT_NE(world_orientation, nullptr);
  EXPECT_EQ(world_orientation->object_type(), mjOBJ_BODY);
  EXPECT_EQ(world_orientation->object_name(), "measured_body");
  EXPECT_EQ(world_orientation->frame_id(), "world");
  EXPECT_EQ(world_orientation->unit(), "1");

  const auto* camera_projection =
      FindObservation(core, "site_camera_projection");
  ASSERT_NE(camera_projection, nullptr);
  EXPECT_EQ(camera_projection->object_type(), mjOBJ_SITE);
  EXPECT_EQ(camera_projection->object_name(), "imu_site");
  EXPECT_EQ(camera_projection->frame_id(), "metadata_camera");
  EXPECT_EQ(camera_projection->unit(), "px");

  const auto* inside = FindObservation(core, "inside_reference_geom");
  ASSERT_NE(inside, nullptr);
  EXPECT_EQ(inside->unit(), "1");
}

TEST(SimulationCoreTest, BuildsSiteAndGeometrySensorFrameMetadata) {
  const auto core = LoadedCoreFrom(SensorMetadataModelPath());

  for (const auto* name : {"site_acceleration", "site_angular_velocity",
                           "site_force", "site_torque"}) {
    SCOPED_TRACE(name);
    const auto* sensor = FindObservation(core, name);
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(sensor->object_type(), mjOBJ_SITE);
    EXPECT_EQ(sensor->object_name(), "imu_site");
    EXPECT_EQ(sensor->frame_id(), "imu_site");
  }

  const auto* normal = FindObservation(core, "geom_normal");
  ASSERT_NE(normal, nullptr);
  EXPECT_EQ(normal->object_name(), "measured_geom");
  EXPECT_EQ(normal->frame_id(), "world");
  EXPECT_NE(normal->frame_id(), "reference_geom");

  const auto* fromto = FindObservation(core, "geom_fromto");
  ASSERT_NE(fromto, nullptr);
  EXPECT_EQ(fromto->object_name(), "measured_geom");
  EXPECT_EQ(fromto->frame_id(), "world");
  EXPECT_NE(fromto->frame_id(), "reference_geom");
}

TEST(SimulationCoreTest, BuildsJointTendonAndActuatorTransmissionUnits) {
  const auto core = LoadedCoreFrom(SensorMetadataModelPath());
  const std::vector<std::pair<const char*, const char*>> expected_units = {
      {"hinge_joint_force", "N*m"},
      {"slide_joint_force", "N"},
      {"hinge_limit_force", "N*m"},
      {"slide_limit_force", "N"},
      {"tendon_position", "m"},
      {"tendon_velocity", "m/s"},
      {"tendon_force", "N"},
      {"tendon_limit_position", "m"},
      {"tendon_limit_velocity", "m/s"},
      {"tendon_limit_force", "N"},
      {"hinge_actuator_position", "rad"},
      {"hinge_actuator_velocity", "rad/s"},
      {"hinge_actuator_force", "N*m"},
      {"slide_actuator_position", "m"},
      {"slide_actuator_velocity", "m/s"},
      {"slide_actuator_force", "N"},
      {"tendon_actuator_position", "m"},
      {"tendon_actuator_velocity", "m/s"},
      {"tendon_actuator_force", "N"},
      {"hinge_tendon_position", "rad"},
      {"hinge_tendon_velocity", "rad/s"},
      {"hinge_tendon_force", "N*m"},
      {"slide_tendon_position", "m"},
      {"slide_tendon_velocity", "m/s"},
      {"slide_tendon_force", "N"},
      {"hinge_tendon_actuator_position", "rad"},
      {"hinge_tendon_actuator_velocity", "rad/s"},
      {"hinge_tendon_actuator_force", "N*m"},
      {"slide_tendon_actuator_position", "m"},
      {"slide_tendon_actuator_velocity", "m/s"},
      {"slide_tendon_actuator_force", "N"},
  };
  for (const auto& [name, unit] : expected_units) {
    SCOPED_TRACE(name);
    const auto* sensor = FindObservation(core, name);
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(sensor->unit(), unit);
  }

  for (const auto* name : {
           "mixed_tendon_position", "mixed_tendon_velocity",
           "mixed_tendon_force", "mixed_tendon_actuator_position",
           "mixed_tendon_actuator_velocity", "mixed_tendon_actuator_force",
           "site_actuator_position", "site_actuator_velocity",
           "site_actuator_force"}) {
    SCOPED_TRACE(name);
    const auto* sensor = FindObservation(core, name);
    ASSERT_NE(sensor, nullptr);
    EXPECT_TRUE(sensor->unit().empty());
  }
}

TEST(SimulationCoreTest, UnifiedControllerAdapterPreservesSynchronousTickData) {
  auto core = LoadedCore();
  core.OpenIssuedTicks().response;
  ASSERT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());
  const auto defaults = BasicRegisterRequest();
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id(defaults.adapter_id());
  request.set_role(mujoco_sim::core::CONTROLLER);
  for (const auto& name : defaults.observation_names()) {
    request.add_observation_names(name);
  }
  for (const auto& name : defaults.command_names()) {
    request.add_command_names(name);
  }
  request.set_commit_timeout_ms(defaults.commit_timeout_ms());
  request.set_session_timeout_ms(defaults.session_timeout_ms());
  const auto registration = core.RegisterAdapter(request).response;
  ASSERT_EQ(registration.code(), mujoco_sim::core::OK);
  ASSERT_EQ(registration.layout().observations_size(), 2);

  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_adapter_id(registration.adapter_id());
  activate.set_role(mujoco_sim::core::CONTROLLER);
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  ASSERT_EQ(core.ActivateAdapter(activate).response.code(), mujoco_sim::core::OK);
  const auto ticks = core.OpenIssuedTicks().response;
  ASSERT_EQ(ticks.size(), 1U);
  EXPECT_EQ(ticks[0]->frame.observations.size(), 2);
  EXPECT_GT(ticks[0]->frame.identity.simulation_time_s, 0.0);
  EXPECT_DOUBLE_EQ(ticks[0]->frame.identity.simulation_time_s,
                   core.state().simulation_time_s);
  EXPECT_TRUE(AdapterStatusFor(core, "arm").required_for_barrier);

  mujoco_sim::core::AdapterHeartbeat heartbeat;
  heartbeat.session = {registration.session_id(), registration.epoch()};
  heartbeat.last_seen = ticks[0]->frame.identity.tick_id;
  core.AcceptHeartbeat(heartbeat);
  EXPECT_EQ(AdapterStatusFor(core, "arm").last_heartbeat_tick_id,
            ticks[0]->frame.identity.tick_id);
}

TEST(SimulationCoreTest, ObserverValidationIsStrictAndTransactional) {
  auto core = LoadedCore();
  auto request = ObserverRequest();
  request.add_command_names("hinge_motor");
  EXPECT_EQ(core.RegisterAdapter(request).response.code(),
            mujoco_sim::core::INVALID_ARGUMENT);
  EXPECT_EQ(core.state().observer_count(mujoco_sim::core::PROVISIONAL), 0U);

  request = ObserverRequest();
  request.observation_names_[0] = "missing";
  EXPECT_EQ(core.RegisterAdapter(request).response.code(), mujoco_sim::core::NOT_FOUND);
  EXPECT_EQ(core.state().observer_count(mujoco_sim::core::PROVISIONAL), 0U);

  request = ObserverRequest();
  request.mutable_delivery()->set_every_n_steps(0);
  EXPECT_EQ(core.RegisterAdapter(request).response.code(),
            mujoco_sim::core::INVALID_ARGUMENT);
  EXPECT_EQ(core.state().observer_count(mujoco_sim::core::PROVISIONAL), 0U);

  const auto valid = core.RegisterAdapter(ObserverRequest()).response;
  EXPECT_EQ(valid.code(), mujoco_sim::core::OK);
  EXPECT_EQ(valid.session_id(), 1U);
}

TEST(SimulationCoreTest, ObserverReceivesPostStepFramesWithoutJoiningBarrier) {
  auto core = LoadedCore();
  const auto registration =
      RegisterAndActivateObserver(&core, ObserverRequest());
  EXPECT_EQ(core.state().active_controller_count, 0U);
  EXPECT_EQ(core.state().observer_count(mujoco_sim::core::ACTIVE), 1U);
  EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
  auto completed = core.CompleteEmptyBarrier().step_completed;
  ASSERT_TRUE(completed.has_value());
  const auto& frames = completed->observer_frames;
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].identity.session.session_id, registration.session_id());
  EXPECT_EQ(frames[0].sequence, 1U);
  EXPECT_EQ(frames[0].identity.tick_id, 1U);
  EXPECT_DOUBLE_EQ(frames[0].identity.simulation_time_s,
                   core.state().simulation_time_s);
  ASSERT_EQ(frames[0].observations.size(), 2U);
}

TEST(SimulationCoreTest, ObserverSupportsStepAndSimulationTimeDecimation) {
  auto core = LoadedCore();
  auto request = ObserverRequest("step-observer");
  request.mutable_delivery()->set_every_n_steps(2);
  RegisterAndActivateObserver(&core, request);
  request = ObserverRequest("period-observer");
  request.mutable_delivery()->set_period_s(core.state().timestep_s * 2.0);
  RegisterAndActivateObserver(&core, request);

  for (int step = 1; step <= 3; ++step) {
    EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    const auto& frames = completed->observer_frames;
    if (step == 2) {
      EXPECT_TRUE(frames.empty());
    } else {
      EXPECT_EQ(frames.size(), 2U);
    }
  }
}

TEST(SimulationCoreTest, ObserverPeriodUsesPhaseLockedDeadlines) {
  auto core = LoadedCore();
  ASSERT_DOUBLE_EQ(core.state().timestep_s, 0.001);
  auto request = ObserverRequest("phase-observer");
  request.mutable_delivery()->set_period_s(0.0015);
  RegisterAndActivateObserver(&core, request);

  std::vector<mujoco_sim::core::ObservationFrame> frames;
  for (int step = 0; step < 10; ++step) {
    EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    frames.insert(frames.end(), completed->observer_frames.begin(),
                  completed->observer_frames.end());
  }
  const std::vector<std::uint64_t> expected_ticks = {1, 3, 4, 6, 7, 9, 10};
  ASSERT_EQ(frames.size(), expected_ticks.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    EXPECT_EQ(frames[index].identity.tick_id, expected_ticks[index]);
    EXPECT_EQ(frames[index].sequence, index + 1);
  }
}

TEST(SimulationCoreTest, ObserverPeriodRemainsPhaseLockedForLongRuns) {
  auto core = LoadedCore();
  auto request = ObserverRequest("long-phase-observer");
  request.mutable_delivery()->set_period_s(0.0015);
  RegisterAndActivateObserver(&core, request);

  std::uint64_t period_index = 1;
  for (std::uint64_t tick = 1; tick <= 20000; ++tick) {
    EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    const auto& frames = completed->observer_frames;
    const std::uint64_t expected_tick =
        period_index == 0 ? 1 : 1 + (3 * period_index + 1) / 2;
    const bool due = tick == 1 || tick == expected_tick;
    ASSERT_EQ(frames.size(), due ? 1U : 0U) << "tick=" << tick;
    if (due && tick != 1) {
      EXPECT_EQ(frames[0].identity.tick_id, tick);
      ++period_index;
    }
  }
}

TEST(SimulationCoreTest, ObserverPeriodBelowTimestepDeliversEveryStep) {
  auto core = LoadedCore();
  auto request = ObserverRequest("fast-observer");
  request.mutable_delivery()->set_period_s(
      std::numeric_limits<double>::denorm_min());
  RegisterAndActivateObserver(&core, request);

  for (std::uint64_t tick = 1; tick <= 4; ++tick) {
    EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    const auto& frames = completed->observer_frames;
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].identity.tick_id, tick);
  }
}

TEST(SimulationCoreTest, CompletedFramesSurviveUnregisterUntilCollected) {
  auto core = LoadedCore();
  const auto registration =
      RegisterAndActivateObserver(&core, ObserverRequest());
  std::vector<mujoco_sim::core::ObservationFrame> frames;
  for (int step = 0; step < 2; ++step) {
    EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    frames.insert(frames.end(), completed->observer_frames.begin(),
                  completed->observer_frames.end());
  }

  mujoco_sim::core::UnregisterAdapterRequest unregister;
  unregister.set_adapter_id(registration.adapter_id());
  unregister.set_role(mujoco_sim::core::OBSERVER);
  unregister.set_session_id(registration.session_id());
  unregister.set_epoch(registration.epoch());
  ASSERT_EQ(core.UnregisterAdapter(unregister).response.code(), mujoco_sim::core::OK);

  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].identity.tick_id, 1U);
  EXPECT_EQ(frames[0].sequence, 1U);
  EXPECT_EQ(frames[1].identity.tick_id, 2U);
  EXPECT_EQ(frames[1].sequence, 2U);
}

TEST(SimulationCoreTest, ObserverCannotActivateWhenServerIsStoppedOrPaused) {
  for (const bool stopped : {false, true}) {
    SCOPED_TRACE(stopped ? "stopped" : "error-paused");
    auto core = LoadedCore();
    const auto registration = core.RegisterAdapter(ObserverRequest()).response;
    ASSERT_EQ(registration.code(), mujoco_sim::core::OK);

    if (stopped) {
      core.MarkStopped("shutdown");
    } else {
      core.MarkFailed("transport failed");
    }

    mujoco_sim::core::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::core::OBSERVER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    EXPECT_EQ(core.ActivateAdapter(activate).response.code(),
              mujoco_sim::core::FAILED_PRECONDITION);
    EXPECT_EQ(core.state().observer_count(mujoco_sim::core::PROVISIONAL), 1U);
  }
}

TEST(SimulationCoreTest, ControllerCannotActivateWhenServerIsStoppedOrPaused) {
  for (const bool stopped : {false, true}) {
    SCOPED_TRACE(stopped ? "stopped" : "error-paused");
    auto core = LoadedCore();
    const auto registration = core.RegisterAdapter(BasicRegisterRequest()).response;
    ASSERT_EQ(registration.code(), mujoco_sim::core::OK);

    if (stopped) {
      core.MarkStopped("shutdown");
    } else {
      core.MarkFailed("transport failed");
    }

    mujoco_sim::core::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::core::CONTROLLER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    EXPECT_EQ(core.ActivateAdapter(activate).response.code(),
              mujoco_sim::core::FAILED_PRECONDITION);
    EXPECT_EQ(core.state().controller_count(mujoco_sim::core::PROVISIONAL), 1U);
  }
}

TEST(SimulationCoreTest, ObserverKeepaliveTimeoutAndReplacementAreIsolated) {
  auto core = LoadedCore();
  const auto registration =
      RegisterAndActivateObserver(&core, ObserverRequest());
  const auto future = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100);
  const auto deadlines = core.CheckDeadlines(future);
  ASSERT_EQ(deadlines.adapter_changes.size(), 1U);
  EXPECT_EQ(deadlines.adapter_changes[0].new_state,
            mujoco_sim::core::TIMED_OUT);
  EXPECT_EQ(core.state().observer_count(mujoco_sim::core::TIMED_OUT), 1U);
  EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
  EXPECT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());

  const auto replacement = core.RegisterAdapter(ObserverRequest()).response;
  ASSERT_EQ(replacement.code(), mujoco_sim::core::OK);
  EXPECT_NE(replacement.session_id(), registration.session_id());
  EXPECT_NE(replacement.message().find("replaced"), std::string::npos);
}

TEST(SimulationCoreTest, StepsWithAnEmptyBarrier) {
  auto core = LoadedCore();
  const double before = core.state().simulation_time_s;
  EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
  EXPECT_TRUE(core.state().tick_open);
  EXPECT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());
  EXPECT_EQ(core.state().current_tick_id, 1U);
  EXPECT_DOUBLE_EQ(core.state().simulation_time_s,
                   before + core.state().timestep_s);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
}

TEST(SimulationCoreTest, ReportsEveryStepCompletionSourceWithObserverFrames) {
  const auto add_observer = [](mujoco_sim::core::SimulationCore* core,
                               const std::string& id) {
    auto request = ObserverRequest(id);
    request.set_session_timeout_ms(10000);
    RegisterAndActivateObserver(core, request);
  };

  {
    auto core = LoadedCore();
    add_observer(&core, "empty_observer");
    core.OpenIssuedTicks().response;
    const auto result = core.CompleteEmptyBarrier();
    ASSERT_TRUE(result.step_completed.has_value());
    EXPECT_EQ(result.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kEmptyBarrier);
    EXPECT_EQ(result.step_completed->observer_frames.size(), 1U);
  }
  {
    auto core = LoadedCore();
    add_observer(&core, "commit_observer");
    const auto registration = RegisterAndActivate(&core);
    core.OpenIssuedTicks().response;
    const auto result = core.SubmitCommit(CommitFor(registration, 0, 0.1));
    ASSERT_TRUE(result.effects.step_completed.has_value());
    EXPECT_EQ(result.effects.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kCommit);
    EXPECT_EQ(result.effects.step_completed->observer_frames.size(), 1U);
  }
  {
    auto core = LoadedCore();
    add_observer(&core, "deadline_observer");
    RegisterAndActivate(&core);
    core.OpenIssuedTicks().response;
    const auto result = core.CheckDeadlines(
        std::chrono::steady_clock::now() + std::chrono::seconds(2));
    ASSERT_TRUE(result.step_completed.has_value());
    EXPECT_EQ(result.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kDeadline);
    EXPECT_EQ(result.step_completed->observer_frames.size(), 1U);
  }
  {
    auto core = LoadedCore();
    add_observer(&core, "unregister_observer");
    const auto registration = RegisterAndActivate(&core);
    core.OpenIssuedTicks().response;
    mujoco_sim::core::UnregisterAdapterRequest request;
    request.set_adapter_id(registration.adapter_id());
    request.set_role(mujoco_sim::core::CONTROLLER);
    request.set_session_id(registration.session_id());
    request.set_epoch(registration.epoch());
    const auto result = core.UnregisterAdapter(request);
    ASSERT_TRUE(result.effects.step_completed.has_value());
    EXPECT_EQ(result.effects.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kUnregister);
    EXPECT_EQ(result.effects.step_completed->observer_frames.size(), 1U);
  }
  {
    auto core = LoadedCore();
    add_observer(&core, "error_observer");
    const auto registration = RegisterAndActivate(&core);
    core.OpenIssuedTicks().response;
    const auto result = core.SubmitCommit(CommitFor(
        registration, 0, std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(result.effects.step_completed.has_value());
    EXPECT_EQ(result.effects.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kLocalError);
    EXPECT_EQ(result.effects.step_completed->observer_frames.size(), 1U);
  }
}

TEST(SimulationCoreTest, InvalidatesTickHandlesAtEveryLifecycleBoundary) {
  std::string error;
  {
    auto core = LoadedCore();
    RegisterAndActivate(&core);
    const auto handle = core.OpenIssuedTicks().response.front();
    ASSERT_TRUE(LoadModel(&core, MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
    EXPECT_FALSE(handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(handle->identity));
  }
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    const auto handle = core.OpenIssuedTicks().response.front();
    mujoco_sim::core::UnregisterAdapterRequest request;
    request.set_adapter_id(registration.adapter_id());
    request.set_role(mujoco_sim::core::CONTROLLER);
    request.set_session_id(registration.session_id());
    request.set_epoch(registration.epoch());
    ASSERT_EQ(core.UnregisterAdapter(request).response.code(),
              mujoco_sim::core::OK);
    EXPECT_FALSE(handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(handle->identity));
  }
  {
    auto core = LoadedCore();
    const auto old_registration = RegisterAndActivate(&core);
    core.OpenIssuedTicks().response;
    ASSERT_TRUE(core.CheckDeadlines(std::chrono::steady_clock::now() +
                                    std::chrono::seconds(2))
                    .step_completed.has_value());
    const auto recovery_handle = core.OpenIssuedTicks().response.front();
    const auto replacement = core.RegisterAdapter(BasicRegisterRequest()).response;
    ASSERT_EQ(replacement.code(), mujoco_sim::core::OK);
    EXPECT_NE(replacement.session_id(), old_registration.session_id());
    EXPECT_FALSE(recovery_handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(recovery_handle->identity));
  }
  {
    auto core = LoadedCore();
    RegisterAndActivate(&core);
    const auto handle = core.OpenIssuedTicks().response.front();
    core.MarkStopped();
    EXPECT_FALSE(handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(handle->identity));
  }
}

TEST(SimulationCoreTest, LoadsModelFromInitialKeyframeAndClearsControl) {
  mujoco_sim::core::SimulationCore core;
  mujoco_sim::core::ModelLoadOptions options;
  options.initial_keyframe = "ready";
  std::string error;
  ASSERT_TRUE(LoadModel(&core, MUJOCO_SIM_CORE_TEST_MODEL, options, &error))
      << error;

  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->time, 1.25);
  EXPECT_DOUBLE_EQ(snapshot->data()->qpos[0], 0.5);
  EXPECT_DOUBLE_EQ(snapshot->data()->qvel[0], -0.25);
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.0);
  EXPECT_DOUBLE_EQ(snapshot->data()->sensordata[0], 0.5);
  EXPECT_DOUBLE_EQ(core.state().simulation_time_s, 1.25);
}

TEST(SimulationCoreTest, EmptyModelLoadOptionsPreserveDefaultResetBehavior) {
  mujoco_sim::core::SimulationCore source_core;
  mujoco_sim::core::SimulationCore options_core;
  std::string error;
  ASSERT_TRUE(LoadModel(&source_core, MUJOCO_SIM_CORE_TEST_MODEL, &error))
      << error;
  ASSERT_TRUE(LoadModel(&options_core, MUJOCO_SIM_CORE_TEST_MODEL,
                        mujoco_sim::core::ModelLoadOptions{}, &error))
      << error;

  auto source_snapshot = source_core.CreateSnapshot(&error);
  ASSERT_NE(source_snapshot, nullptr) << error;
  auto options_snapshot = options_core.CreateSnapshot(&error);
  ASSERT_NE(options_snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(options_snapshot->data()->time,
                   source_snapshot->data()->time);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->qpos[0],
                   source_snapshot->data()->qpos[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->qvel[0],
                   source_snapshot->data()->qvel[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->ctrl[0],
                   source_snapshot->data()->ctrl[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->sensordata[0],
                   source_snapshot->data()->sensordata[0]);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->time, 0.0);
  EXPECT_DOUBLE_EQ(options_snapshot->data()->qpos[0], 0.0);
}

TEST(SimulationCoreTest, MissingInitialKeyframeLeavesLoadedModelUnchanged) {
  auto core = LoadedCore();
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);
  const auto before = core.state();

  mujoco_sim::core::ModelLoadOptions options;
  options.initial_keyframe = "missing_pose";
  EXPECT_FALSE(LoadModel(&core, MUJOCO_SIM_CORE_DUAL_TEST_MODEL, options, &error));
  EXPECT_EQ(error,
            "initial keyframe 'missing_pose' was not found in MuJoCo model");
  EXPECT_TRUE(core.IsModelLoaded());
  const auto after = core.state();
  EXPECT_EQ(after.epoch, before.epoch);
  EXPECT_EQ(after.current_tick_id, before.current_tick_id);
  EXPECT_DOUBLE_EQ(after.simulation_time_s, before.simulation_time_s);
  EXPECT_EQ(after.server_state, mujoco_sim::core::RUNNING);
  EXPECT_TRUE(after.tick_open);
  EXPECT_EQ(after.active_controller_count, 1);
  ASSERT_EQ(core.resources().commands.size(), 1);
  EXPECT_EQ(core.resources().commands[0].name(), "hinge_motor");
  EXPECT_TRUE(core.UpdateSnapshot(*snapshot, &error)) << error;

  const auto result =
      core.SubmitCommit(CommitFor(registration, before.current_tick_id, 0.2));
  EXPECT_EQ(result.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(result.effects.step_completed.has_value());
}

TEST(SimulationCoreTest, ProvidesAvailableResources) {
  auto core = LoadedCore();
  mujoco_sim::core::GetAvailableResourcesRequest request;
  const auto response = core.GetAvailableResources(request);
  EXPECT_EQ(response.code(), mujoco_sim::core::OK);
  EXPECT_EQ(response.observations().size(), 3U);
  EXPECT_EQ(response.commands().size(), 1U);
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

  ASSERT_TRUE(core.UpdateSnapshot(*snapshot, &error)) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->qpos[0], independent->data()->qpos[0]);
}

TEST(SimulationCoreTest, ClonedSnapshotsShareConstModelAndIsolateData) {
  auto core = LoadedCore();
  std::string error;
  auto source = core.CreateSnapshot(&error);
  ASSERT_NE(source, nullptr) << error;
  source->data()->qpos[0] = 0.75;
  source->data()->qvel[0] = -0.25;
  source->data()->time = 2.0;

  auto core_state = core.CreateSnapshot(&error);
  ASSERT_NE(core_state, nullptr) << error;
  ASSERT_NE(source->data()->qpos[0], core_state->data()->qpos[0]);
  ASSERT_NE(source->data()->time, core_state->data()->time);

  auto clone = core.CloneSnapshot(*source, &error);
  ASSERT_NE(clone, nullptr) << error;
  EXPECT_EQ(source->model(), clone->model());
  EXPECT_NE(source->data(), clone->data());
  EXPECT_DOUBLE_EQ(clone->data()->qpos[0], source->data()->qpos[0]);
  EXPECT_DOUBLE_EQ(clone->data()->qvel[0], source->data()->qvel[0]);
  EXPECT_DOUBLE_EQ(clone->data()->time, source->data()->time);

  clone->data()->qpos[0] = -0.5;
  EXPECT_DOUBLE_EQ(source->data()->qpos[0], 0.75);
  EXPECT_NE(clone->data()->qpos[0], core_state->data()->qpos[0]);

  ASSERT_TRUE(core.UpdateSnapshot(*clone, &error)) << error;
  EXPECT_DOUBLE_EQ(clone->data()->qpos[0], core_state->data()->qpos[0]);
  EXPECT_DOUBLE_EQ(clone->data()->time, core_state->data()->time);
  EXPECT_NE(clone->data()->qpos[0], source->data()->qpos[0]);

  ASSERT_TRUE(LoadModel(&core, MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  EXPECT_EQ(core.CloneSnapshot(*source, &error), nullptr);
  EXPECT_NE(error.find("stale"), std::string::npos);
}

TEST(SimulationCoreTest, RefreshesVisualizationSnapshotAfterStep) {
  auto core = LoadedCore();
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.2)).effects.step_completed.has_value());
  ASSERT_TRUE(core.UpdateSnapshot(*snapshot, &error)) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->time, core.state().simulation_time_s);
}

TEST(SimulationCoreTest, RejectsSnapshotFromAnotherCoreOrModelGeneration) {
  auto first = LoadedCore();
  auto second = LoadedCore();
  std::string error;
  auto snapshot = first.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  EXPECT_FALSE(second.UpdateSnapshot(*snapshot, &error));
  EXPECT_NE(error.find("does not belong"), std::string::npos);

  ASSERT_TRUE(LoadModel(&first, MUJOCO_SIM_CORE_DUAL_TEST_MODEL, &error))
      << error;
  EXPECT_FALSE(first.UpdateSnapshot(*snapshot, &error));
  EXPECT_NE(error.find("does not belong"), std::string::npos);
}

TEST(SimulationCoreTest, MoveTransfersSnapshotOwnershipToDestination) {
  auto first = LoadedCore();
  auto second = LoadedCore();
  std::string error;
  auto snapshot = first.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  auto first_destination = std::move(first);
  auto second_destination = std::move(second);
  EXPECT_TRUE(first_destination.IsModelLoaded());
  EXPECT_TRUE(second_destination.IsModelLoaded());
  EXPECT_TRUE(first_destination.UpdateSnapshot(*snapshot, &error)) << error;
  EXPECT_FALSE(second_destination.UpdateSnapshot(*snapshot, &error));
  EXPECT_NE(error.find("does not belong"), std::string::npos);
  EXPECT_EQ(second_destination.CloneSnapshot(*snapshot, &error), nullptr);
  EXPECT_NE(error.find("invalid or stale"), std::string::npos);
}

TEST(SimulationCoreTest, RejectsUnknownAndConflictingResources) {
  auto core = LoadedCore();
  auto bad = BasicRegisterRequest("bad");
  bad.observation_names_[0] = "missing_sensor";
  EXPECT_EQ(core.RegisterAdapter(bad).response.code(), mujoco_sim::core::NOT_FOUND);

  auto first = core.RegisterAdapter(BasicRegisterRequest("first")).response;
  EXPECT_EQ(first.code(), mujoco_sim::core::OK);
  auto conflict = core.RegisterAdapter(BasicRegisterRequest("second")).response;
  EXPECT_EQ(conflict.code(), mujoco_sim::core::RESOURCE_CONFLICT);
}

TEST(SimulationCoreTest, ActivatesAndPublishesTickWithObservation) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  const auto ticks = core.OpenIssuedTicks().response;
  ASSERT_EQ(ticks.size(), 1);
  EXPECT_TRUE(core.state().tick_open);
  EXPECT_EQ(ticks[0]->identity.adapter_id, registration.adapter_id());
  EXPECT_EQ(ticks[0]->frame.identity.tick_id, 0);
  EXPECT_GE(ticks[0]->frame.observations.size(), 1);
}

TEST(SimulationCoreTest, ControllerDeliveryStatusDistinguishesUnsentTickZero) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  auto status = AdapterStatusFor(core, registration.adapter_id());
  EXPECT_FALSE(status.last_delivery_tick_id.has_value());

  const auto issued = core.OpenIssuedTicks();
  ASSERT_EQ(issued.response.size(), 1U);
  status = AdapterStatusFor(core, registration.adapter_id());
  ASSERT_TRUE(status.last_delivery_tick_id.has_value());
  EXPECT_EQ(*status.last_delivery_tick_id, 0U);
}

TEST(SimulationCoreTest, StepsWhenBarrierCompletesAndHandlesDuplicateCommits) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);

  const auto commit = CommitFor(registration, 0, 0.2);
  const auto duplicate_in_open_tick = core.SubmitCommit(commit);
  EXPECT_EQ(duplicate_in_open_tick.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(duplicate_in_open_tick.effects.step_completed.has_value());
  const auto stale_duplicate = core.SubmitCommit(commit);
  EXPECT_EQ(stale_duplicate.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(stale_duplicate.response.duplicate);
  EXPECT_NE(core.state().server_state, mujoco_sim::core::ERROR_PAUSED);
  EXPECT_EQ(core.state().current_tick_id, 1);

  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);
  const auto duplicate_current_tick = core.SubmitCommit(CommitFor(registration, 1, 0.2));
  EXPECT_EQ(duplicate_current_tick.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(duplicate_current_tick.effects.step_completed.has_value());
  const auto result = core.SubmitCommit(CommitFor(registration, 1, 0.2));
  EXPECT_EQ(result.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(result.response.duplicate);
  EXPECT_EQ(core.state().current_tick_id, 2);
}

TEST(SimulationCoreTest, DuplicatePayloadMismatchOnlyErrorsItsController) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second =
      RegisterAndActivate(&core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.state().active_controller_count, 2);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);

  const auto first_commit = CommitFor(first, 0, 0.2);
  const auto accepted = core.SubmitCommit(first_commit);
  EXPECT_EQ(accepted.response.code, mujoco_sim::core::OK);
  EXPECT_FALSE(accepted.effects.step_completed.has_value());

  const auto same_duplicate = core.SubmitCommit(first_commit);
  EXPECT_EQ(same_duplicate.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(same_duplicate.response.duplicate);
  EXPECT_NE(core.state().server_state, mujoco_sim::core::ERROR_PAUSED);

  const auto different_duplicate = core.SubmitCommit(CommitFor(first, 0, 0.3));
  EXPECT_EQ(different_duplicate.response.code, mujoco_sim::core::INVALID_COMMAND);
  EXPECT_TRUE(HasChange(different_duplicate.effects,
                        mujoco_sim::core::AdapterChangeKind::kLocalError));
  EXPECT_EQ(different_duplicate.effects.adapter_changes.front().new_state,
            mujoco_sim::core::ERROR);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
  EXPECT_EQ(AdapterStatusFor(core, "first").state,
            mujoco_sim::core::ERROR);
  EXPECT_EQ(AdapterStatusFor(core, "second").state,
            mujoco_sim::core::ACTIVE);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.1)).effects.step_completed.has_value());
}

TEST(SimulationCoreTest, StaleDuplicatePayloadMismatchErrorsItsController) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  const auto issued = core.OpenIssuedTicks();
  ASSERT_EQ(issued.response.size(), 1U);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.2)).effects.step_completed.has_value());

  const auto mismatch = core.SubmitCommit(CommitFor(registration, 0, 0.3));
  EXPECT_EQ(mismatch.response.code, mujoco_sim::core::INVALID_COMMAND);
  EXPECT_TRUE(HasChange(mismatch.effects,
                        mujoco_sim::core::AdapterChangeKind::kLocalError));
  EXPECT_EQ(mismatch.effects.adapter_changes.front().new_state,
            mujoco_sim::core::ERROR);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
}

TEST(SimulationCoreTest, UnverifiableOlderCommitIsRejected) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1U);
  const auto first_commit = CommitFor(registration, 0, 0.2);
  ASSERT_TRUE(core.SubmitCommit(first_commit).effects.step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1U);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 1, 0.1)).effects.step_completed.has_value());

  const auto stale = core.SubmitCommit(first_commit);
  EXPECT_EQ(stale.response.code, mujoco_sim::core::INVALID_TICK);
  EXPECT_TRUE(stale.effects.adapter_changes.empty());
  EXPECT_EQ(AdapterStatusFor(core, registration.adapter_id()).state,
            mujoco_sim::core::ACTIVE);
}

TEST(SimulationCoreTest, InvalidCommandOnlyErrorsItsController) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);

  auto commit =
      CommitFor(first, 0, std::numeric_limits<double>::quiet_NaN());
  const auto result = core.SubmitCommit(commit);
  EXPECT_EQ(result.response.code, mujoco_sim::core::INVALID_COMMAND);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
  EXPECT_EQ(AdapterStatusFor(core, "first").state,
            mujoco_sim::core::ERROR);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.2)).effects.step_completed.has_value());

  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.0);
}

TEST(SimulationCoreTest, IdentityMismatchesDoNotChangeControllerState) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);

  auto wrong_session = CommitFor(registration, 0, 0.0);
  wrong_session.identity.session.session_id = registration.session_id() + 1;
  EXPECT_EQ(core.SubmitCommit(wrong_session).response.code,
            mujoco_sim::core::INVALID_SESSION);
  auto wrong_epoch = CommitFor(registration, 0, 0.0);
  wrong_epoch.identity.session.epoch = registration.epoch() + 1;
  EXPECT_EQ(core.SubmitCommit(wrong_epoch).response.code, mujoco_sim::core::INVALID_EPOCH);
  EXPECT_EQ(core.SubmitCommit(CommitFor(registration, 1, 0.0)).response.code,
            mujoco_sim::core::INVALID_TICK);
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
  EXPECT_EQ(AdapterStatusFor(core, "arm").state,
            mujoco_sim::core::ACTIVE);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).effects.step_completed.has_value());
}

TEST(SimulationCoreTest, RegistersAndActivatesWhileRunning) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  EXPECT_TRUE(core.OpenIssuedTicks().response.empty());
  EXPECT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());
  EXPECT_EQ(core.state().current_tick_id, 1U);

  const auto first = RegisterAndActivate(
      &core, BasicRegisterRequest("first", "hinge_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);
  const auto second_registration =
      core.RegisterAdapter(BasicRegisterRequest("second", "second_motor"))
          .response;
  ASSERT_EQ(second_registration.code(), mujoco_sim::core::OK);
  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_adapter_id("second");
  activate.set_role(mujoco_sim::core::CONTROLLER);
  activate.set_session_id(second_registration.session_id());
  activate.set_epoch(second_registration.epoch());
  EXPECT_EQ(core.ActivateAdapter(activate).response.code(), mujoco_sim::core::OK);
  EXPECT_EQ(core.OpenIssuedTicks().response.size(), 1);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(first, 1, 0.1)).effects.step_completed.has_value());
  EXPECT_EQ(core.OpenIssuedTicks().response.size(), 2);
}

TEST(SimulationCoreTest, UnregistersRequiredControllerDuringOpenTick) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.2)).effects.step_completed.has_value());

  mujoco_sim::core::UnregisterAdapterRequest request;
  request.set_adapter_id("second");
  request.set_role(mujoco_sim::core::CONTROLLER);
  request.set_session_id(second.session_id());
  request.set_epoch(second.epoch());
  EXPECT_EQ(core.UnregisterAdapter(request).response.code(), mujoco_sim::core::OK);
  EXPECT_EQ(core.state().current_tick_id, 1U);
  EXPECT_EQ(core.MakeStatus().adapters.size(), 1U);
}

TEST(SimulationCoreTest, DefersUnregisterControlWriteUntilBarrierCompletes) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2U);
  ASSERT_FALSE(
      core.SubmitCommit(CommitFor(first, 0, 0.2)).effects.step_completed.has_value());
  ASSERT_TRUE(
      core.SubmitCommit(CommitFor(second, 0, 0.3)).effects.step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2U);

  mujoco_sim::core::UnregisterAdapterRequest request;
  request.set_adapter_id("first");
  request.set_role(mujoco_sim::core::CONTROLLER);
  request.set_session_id(first.session_id());
  request.set_epoch(first.epoch());
  const auto unregistered = core.UnregisterAdapter(request);
  ASSERT_EQ(unregistered.response.code(), mujoco_sim::core::OK);
  EXPECT_FALSE(unregistered.effects.step_completed.has_value());

  std::string error;
  auto before_completion = core.CreateSnapshot(&error);
  ASSERT_NE(before_completion, nullptr) << error;
  EXPECT_DOUBLE_EQ(before_completion->data()->ctrl[0], 0.2);

  ASSERT_TRUE(
      core.SubmitCommit(CommitFor(second, 1, 0.1)).effects.step_completed.has_value());
  auto after_completion = core.CreateSnapshot(&error);
  ASSERT_NE(after_completion, nullptr) << error;
  EXPECT_DOUBLE_EQ(after_completion->data()->ctrl[0], 0.0);
}

TEST(SimulationCoreTest, ExpiresProvisionalControllerAndReleasesActuator) {
  mujoco_sim::core::CoreOptions options;
  options.activation_timeout_ms = 10;
  mujoco_sim::core::SimulationCore core(options);
  std::string error;
  ASSERT_TRUE(LoadModel(&core, MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  EXPECT_EQ(core.RegisterAdapter(BasicRegisterRequest("first")).response.code(),
            mujoco_sim::core::OK);
  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  ASSERT_EQ(deadlines.adapter_changes.size(), 1U);
  EXPECT_EQ(deadlines.adapter_changes[0].old_state, mujoco_sim::core::PROVISIONAL);
  EXPECT_EQ(deadlines.adapter_changes[0].new_state,
            mujoco_sim::core::UNREGISTERED);
  EXPECT_EQ(core.RegisterAdapter(BasicRegisterRequest("second")).response.code(),
            mujoco_sim::core::OK);
}

TEST(SimulationCoreTest, TimesOutOneControllerAndContinuesBarrier) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.2)).effects.step_completed.has_value());
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.3)).effects.step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 1, 0.2)).effects.step_completed.has_value());

  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  ASSERT_EQ(deadlines.adapter_changes.size(), 1U);
  EXPECT_EQ(deadlines.adapter_changes[0].adapter_id, "second");
  EXPECT_EQ(deadlines.adapter_changes[0].new_state, mujoco_sim::core::TIMED_OUT);
  EXPECT_TRUE(deadlines.step_completed.has_value());
  EXPECT_EQ(core.state().server_state, mujoco_sim::core::RUNNING);
  const auto timed_out = AdapterStatusFor(core, "second");
  EXPECT_EQ(timed_out.state, mujoco_sim::core::TIMED_OUT);
  EXPECT_EQ(timed_out.timeout_count, 1U);
  EXPECT_FALSE(timed_out.required_for_barrier);
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[1], 0.0);
  EXPECT_EQ(core.OpenIssuedTicks().response.size(), 2);
}

TEST(SimulationCoreTest, KeepsIssuedTickPayloadImmutableAfterTimeout) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  auto first_request = BasicRegisterRequest("first", "hinge_motor");
  first_request.set_commit_timeout_ms(10);
  const auto first = RegisterAndActivate(&core, first_request);
  auto second_request = BasicRegisterRequest("second", "second_motor");
  second_request.set_commit_timeout_ms(100);
  const auto second = RegisterAndActivate(&core, second_request);
  const auto issued = core.OpenIssuedTicks().response;
  ASSERT_EQ(issued.size(), 2);
  const auto first_tick = std::find_if(
      issued.begin(), issued.end(),
      [](const auto& tick) { return tick->identity.adapter_id == "first"; });
  ASSERT_NE(first_tick, issued.end());
  const auto frozen_identity = (*first_tick)->frame.identity;
  const auto frozen_observations = (*first_tick)->frame.observations;

  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20));
  ASSERT_EQ(deadlines.adapter_changes.size(), 1U);
  EXPECT_FALSE(deadlines.step_completed.has_value());
  const auto retransmitted = core.OpenIssuedTicks().response;
  const auto repeated_first = std::find_if(
      retransmitted.begin(), retransmitted.end(),
      [](const auto& tick) { return tick->identity.adapter_id == "first"; });
  ASSERT_NE(repeated_first, retransmitted.end());
  EXPECT_EQ((*first_tick).get(), (*repeated_first).get());
  EXPECT_EQ((*repeated_first)->frame.identity.session.session_id,
            frozen_identity.session.session_id);
  EXPECT_EQ((*repeated_first)->frame.identity.tick_id, frozen_identity.tick_id);
  EXPECT_EQ((*repeated_first)->frame.observations, frozen_observations);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.1)).effects.step_completed.has_value());
  EXPECT_FALSE((*first_tick)->current.load());
  (void)first;
}

TEST(SimulationCoreTest, RecoversTimedOutControllerOnNextTick) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.1)).effects.step_completed.has_value());
  EXPECT_TRUE(core.CheckDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .step_completed.has_value());

  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);
  const auto recovery = core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(recovery.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(HasChange(recovery.effects,
                        mujoco_sim::core::AdapterChangeKind::kRecoveryPending));
  EXPECT_FALSE(recovery.effects.step_completed.has_value());
  const auto duplicate = core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(duplicate.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(duplicate.response.duplicate);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(first, 1, 0.1)).effects.step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 2);
  EXPECT_EQ(AdapterStatusFor(core, "second").state,
            mujoco_sim::core::ACTIVE);
  EXPECT_TRUE(AdapterStatusFor(core, "second").required_for_barrier);
  const auto delayed_duplicate =
      core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(delayed_duplicate.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(delayed_duplicate.response.duplicate);
  EXPECT_EQ(AdapterStatusFor(core, "second").state,
            mujoco_sim::core::ACTIVE);
}

TEST(SimulationCoreTest, KeepsHeartbeatTickMonotonic) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).effects.step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1U);

  mujoco_sim::core::AdapterHeartbeat heartbeat;
  heartbeat.session = {registration.session_id(), registration.epoch()};
  heartbeat.last_seen = 1;
  core.AcceptHeartbeat(heartbeat);
  heartbeat.last_seen = 0;
  core.AcceptHeartbeat(heartbeat);

  EXPECT_EQ(AdapterStatusFor(core, "arm").last_heartbeat_tick_id, 1U);
}

TEST(SimulationCoreTest, HeartbeatDoesNotExtendBarrierCommitDeadline) {
  auto core = LoadedCore();
  auto request = BasicRegisterRequest();
  request.set_commit_timeout_ms(10);
  request.set_session_timeout_ms(1000);
  const auto registration = RegisterAndActivate(&core, request);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1U);

  const auto now = std::chrono::steady_clock::now();
  mujoco_sim::core::AdapterHeartbeat heartbeat;
  heartbeat.session = {registration.session_id(), registration.epoch()};
  heartbeat.last_seen = 0;
  core.AcceptHeartbeat(heartbeat, now + std::chrono::milliseconds(100));

  const auto deadlines =
      core.CheckDeadlines(now + std::chrono::milliseconds(20));
  ASSERT_TRUE(deadlines.step_completed.has_value());
  EXPECT_EQ(deadlines.step_completed->source,
            mujoco_sim::core::StepCompletionSource::kDeadline);
  EXPECT_EQ(AdapterStatusFor(core, "arm").state,
            mujoco_sim::core::TIMED_OUT);
}

TEST(SimulationCoreTest, ControllerPresenceTimeoutIsIndependentOfBarrier) {
  auto core = LoadedCore();
  auto request = BasicRegisterRequest();
  request.set_commit_timeout_ms(1000);
  request.set_session_timeout_ms(10);
  RegisterAndActivate(&core, request);

  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20));
  EXPECT_FALSE(deadlines.step_completed.has_value());
  EXPECT_TRUE(HasChange(deadlines,
                        mujoco_sim::core::AdapterChangeKind::kTimedOut));
  EXPECT_EQ(AdapterStatusFor(core, "arm").state,
            mujoco_sim::core::TIMED_OUT);
}

TEST(SimulationCoreTest, ReplacesTimedOutControllerWithNewSession) {
  auto core = LoadedCore();
  const auto old_registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);
  EXPECT_TRUE(core.CheckDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .step_completed.has_value());

  const auto replacement = core.RegisterAdapter(BasicRegisterRequest()).response;
  ASSERT_EQ(replacement.code(), mujoco_sim::core::OK);
  EXPECT_NE(replacement.session_id(), old_registration.session_id());
  EXPECT_NE(replacement.message().find("replaced"), std::string::npos);
  EXPECT_EQ(core.SubmitCommit(CommitFor(old_registration, 0, 0.1)).response.code,
            mujoco_sim::core::INVALID_SESSION);
  EXPECT_EQ(AdapterStatusFor(core, "arm").state,
            mujoco_sim::core::PROVISIONAL);
}

TEST(SimulationCoreTest, CanDisableTimedOutControllerReplacement) {
  mujoco_sim::core::CoreOptions options;
  options.allow_timed_out_adapter_replacement = false;
  mujoco_sim::core::SimulationCore core(options);
  std::string error;
  ASSERT_TRUE(LoadModel(&core, MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1);
  EXPECT_TRUE(core.CheckDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .step_completed.has_value());
  EXPECT_EQ(core.RegisterAdapter(BasicRegisterRequest()).response.code(),
            mujoco_sim::core::ALREADY_EXISTS);
}

TEST(SimulationCoreTest, FatalErrorUpdatesControllerStateTimestamp) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().response.size(), 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.0)).effects.step_completed.has_value());
  ASSERT_EQ(core.state().current_tick_id, 1U);

  const auto effects = core.MarkFailed("fatal transport error");
  ASSERT_EQ(effects.adapter_changes.size(), 1U);
  EXPECT_EQ(effects.adapter_changes.front().kind,
            mujoco_sim::core::AdapterChangeKind::kServerFailed);
  EXPECT_EQ(effects.adapter_changes.front().old_state,
            mujoco_sim::core::ACTIVE);
  EXPECT_EQ(effects.adapter_changes.front().new_state,
            mujoco_sim::core::ERROR);
  const auto status = AdapterStatusFor(core, "arm");
  EXPECT_EQ(status.state, mujoco_sim::core::ERROR);
  EXPECT_EQ(status.state_since_tick_id, 1U);
  EXPECT_EQ(status.message, "fatal transport error");
}

TEST(SimulationCoreTest, EngineEffectsDescribeLifecycleTransitions) {
  auto core = LoadedCore();

  const auto registered = core.RegisterAdapter(BasicRegisterRequest());
  ASSERT_EQ(registered.response.code(), mujoco_sim::core::OK);
  EXPECT_TRUE(HasChange(
      registered.effects, mujoco_sim::core::AdapterChangeKind::kRegistered));

  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_adapter_id(registered.response.adapter_id());
  activate.set_role(mujoco_sim::core::CONTROLLER);
  activate.set_session_id(registered.response.session_id());
  activate.set_epoch(registered.response.epoch());
  const auto activated = core.ActivateAdapter(activate);
  ASSERT_EQ(activated.response.code(), mujoco_sim::core::OK);
  EXPECT_TRUE(HasChange(
      activated.effects, mujoco_sim::core::AdapterChangeKind::kActivated));

  const auto issued = core.OpenIssuedTicks();
  ASSERT_EQ(issued.response.size(), 1U);
  const auto timed_out = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::seconds(1));
  EXPECT_TRUE(HasChange(
      timed_out, mujoco_sim::core::AdapterChangeKind::kTimedOut));

  const auto recovery = core.SubmitCommit(
      CommitFor(registered.response, issued.response.front()->identity.tick_id,
                0.25));
  ASSERT_EQ(recovery.response.code, mujoco_sim::core::OK);
  EXPECT_TRUE(HasChange(
      recovery.effects,
      mujoco_sim::core::AdapterChangeKind::kRecoveryPending));
  const auto reopened = core.OpenIssuedTicks();
  EXPECT_TRUE(HasChange(
      reopened.effects, mujoco_sim::core::AdapterChangeKind::kRecovered));

  mujoco_sim::core::UnregisterAdapterRequest unregister;
  unregister.set_adapter_id(registered.response.adapter_id());
  unregister.set_role(mujoco_sim::core::CONTROLLER);
  unregister.set_session_id(registered.response.session_id());
  unregister.set_epoch(registered.response.epoch());
  const auto unregistered = core.UnregisterAdapter(unregister);
  ASSERT_EQ(unregistered.response.code(), mujoco_sim::core::OK);
  EXPECT_TRUE(HasChange(
      unregistered.effects,
      mujoco_sim::core::AdapterChangeKind::kUnregistered));

  const auto provisional = core.RegisterAdapter(BasicRegisterRequest("late"));
  ASSERT_EQ(provisional.response.code(), mujoco_sim::core::OK);
  const auto expired = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::seconds(10));
  EXPECT_TRUE(HasChange(
      expired, mujoco_sim::core::AdapterChangeKind::kActivationExpired));

  const auto observer = RegisterAndActivateObserver(&core, ObserverRequest());
  const auto observer_timeout = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::seconds(1));
  ASSERT_TRUE(HasChange(
      observer_timeout, mujoco_sim::core::AdapterChangeKind::kTimedOut));
  const auto replacement = core.RegisterAdapter(ObserverRequest());
  ASSERT_EQ(replacement.response.code(), mujoco_sim::core::OK);
  EXPECT_TRUE(HasChange(
      replacement.effects, mujoco_sim::core::AdapterChangeKind::kReplaced));
  ASSERT_EQ(replacement.effects.adapter_changes.size(), 1U);
  EXPECT_EQ(replacement.effects.adapter_changes.front().previous_session_id,
            observer.session_id());
}

}  // namespace
