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

std::string SensorMetadataModelPath() {
  return (std::filesystem::path(MUJOCO_SIM_CORE_TEST_MODEL).parent_path() /
          "sensor_metadata.xml")
      .string();
}

const mujoco_sim::v2::ResourceDescriptor* FindObservation(
    const mujoco_sim::core::SimulationCore& core, const std::string& name) {
  const auto found = std::find_if(
      core.observations().begin(), core.observations().end(),
      [&name](const auto& descriptor) { return descriptor.name() == name; });
  return found == core.observations().end() ? nullptr : &*found;
}

mujoco_sim::v2::RegisterAdapterRequest BasicRegisterRequest(
    const std::string& adapter_id = "arm",
    const std::string& command_name = "hinge_motor") {
  mujoco_sim::v2::RegisterAdapterRequest request;
  request.set_server_name("default");
  request.set_adapter_id(adapter_id);
  request.set_role(mujoco_sim::v2::CONTROLLER);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.add_observation_names("hinge_pos");
  request.add_observation_names("hinge_vel");
  request.add_command_names(command_name);
  request.set_timeout_ms(25);
  return request;
}

mujoco_sim::v2::AdapterCommit CommitFor(
    const mujoco_sim::v2::RegisterAdapterResponse& registration,
    std::uint64_t tick_id, double value) {
  mujoco_sim::v2::AdapterCommit commit;
  commit.mutable_header()->set_server_name("default");
  commit.mutable_header()->set_session_id(registration.session_id());
  commit.mutable_header()->set_epoch(registration.epoch());
  commit.mutable_header()->set_tick_id(tick_id);
  commit.mutable_header()->set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  commit.mutable_header()->set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  commit.set_adapter_id(registration.adapter_id());
  commit.set_status(mujoco_sim::v2::COMMIT_OK);
  auto* command = commit.add_commands();
  command->set_name(registration.layout().commands(0).name());
  command->add_values(value);
  return commit;
}

mujoco_sim::v2::RegisterAdapterResponse RegisterAndActivate(
    mujoco_sim::core::SimulationCore* core,
    const mujoco_sim::v2::RegisterAdapterRequest& request) {
  auto registration = core->RegisterAdapter(request);
  EXPECT_EQ(registration.code(), mujoco_sim::v2::OK);
  mujoco_sim::v2::ActivateAdapterRequest activate;
  activate.set_adapter_id(registration.adapter_id());
  activate.set_role(mujoco_sim::v2::CONTROLLER);
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  const auto activated = core->ActivateAdapter(activate);
  EXPECT_EQ(activated.code(), mujoco_sim::v2::OK);
  return registration;
}

mujoco_sim::v2::RegisterAdapterRequest ObserverRequest(
    const std::string& observer_id = "state-observer") {
  mujoco_sim::v2::RegisterAdapterRequest request;
  request.set_server_name("default");
  request.set_adapter_id(observer_id);
  request.set_role(mujoco_sim::v2::OBSERVER);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.add_observation_names("hinge_pos");
  request.add_observation_names("hinge_vel");
  request.set_timeout_ms(25);
  request.mutable_delivery()->set_every_n_steps(1);
  return request;
}

mujoco_sim::v2::RegisterAdapterResponse RegisterAndActivateObserver(
    mujoco_sim::core::SimulationCore* core,
    const mujoco_sim::v2::RegisterAdapterRequest& request) {
  auto registration = core->RegisterAdapter(request);
  EXPECT_EQ(registration.code(), mujoco_sim::v2::OK);
  mujoco_sim::v2::ActivateAdapterRequest activate;
  activate.set_adapter_id(registration.adapter_id());
  activate.set_role(mujoco_sim::v2::OBSERVER);
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  EXPECT_EQ(core->ActivateAdapter(activate).code(), mujoco_sim::v2::OK);
  return registration;
}

mujoco_sim::v2::AdapterStatus AdapterStatusFor(
    const mujoco_sim::core::SimulationCore& core,
    const std::string& adapter_id) {
  const auto server_status = core.MakeStatus();
  for (const auto& status : server_status.adapters()) {
    if (status.adapter_id() == adapter_id) return status;
  }
  ADD_FAILURE() << "missing adapter status for " << adapter_id;
  return {};
}

mujoco_sim::v2::RegisterAdapterResponse RegisterAndActivate(
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
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
  EXPECT_GT(core.timestep(), 0.0);
  EXPECT_EQ(core.observations()[0].mujoco_stage(), mjSTAGE_POS);
  EXPECT_GE(core.observations()[0].interval_period_s(), 0.0);
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
  core.OpenIssuedTicks();
  ASSERT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());
  const auto defaults = BasicRegisterRequest();
  mujoco_sim::v2::RegisterAdapterRequest request;
  request.set_server_name(defaults.server_name());
  request.set_adapter_id(defaults.adapter_id());
  request.set_role(mujoco_sim::v2::CONTROLLER);
  request.set_protocol_major(defaults.protocol_major());
  request.set_protocol_minor(defaults.protocol_minor());
  request.mutable_observation_names()->CopyFrom(defaults.observation_names());
  request.mutable_command_names()->CopyFrom(defaults.command_names());
  request.set_timeout_ms(defaults.timeout_ms());
  const auto registration = core.RegisterAdapter(request);
  ASSERT_EQ(registration.code(), mujoco_sim::v2::OK);
  ASSERT_EQ(registration.layout().observations_size(), 2);

  mujoco_sim::v2::ActivateAdapterRequest activate;
  activate.set_adapter_id(registration.adapter_id());
  activate.set_role(mujoco_sim::v2::CONTROLLER);
  activate.set_session_id(registration.session_id());
  activate.set_epoch(registration.epoch());
  ASSERT_EQ(core.ActivateAdapter(activate).code(), mujoco_sim::v2::OK);
  const auto ticks = core.OpenIssuedTicks();
  ASSERT_EQ(ticks.size(), 1U);
  EXPECT_EQ(ticks[0]->message.observations_size(), 2);
  EXPECT_TRUE(ticks[0]->message.header().has_simulation_time_s());
  EXPECT_DOUBLE_EQ(ticks[0]->message.header().simulation_time_s(),
                   core.simulation_time());
  EXPECT_TRUE(AdapterStatusFor(core, "arm").required_for_barrier());

  mujoco_sim::v2::AdapterHeartbeat heartbeat;
  *heartbeat.mutable_header() = ticks[0]->message.header();
  heartbeat.set_adapter_id(registration.adapter_id());
  heartbeat.set_role(mujoco_sim::v2::CONTROLLER);
  core.AcceptHeartbeat(heartbeat);
  EXPECT_EQ(AdapterStatusFor(core, "arm").last_heartbeat_tick_id(),
            ticks[0]->message.header().tick_id());
}

TEST(SimulationCoreTest, ObserverValidationIsStrictAndTransactional) {
  auto core = LoadedCore();
  auto request = ObserverRequest();
  request.add_command_names("hinge_motor");
  EXPECT_EQ(core.RegisterAdapter(request).code(),
            mujoco_sim::v2::INVALID_ARGUMENT);
  EXPECT_EQ(core.observer_count(mujoco_sim::v2::PROVISIONAL), 0U);

  request = ObserverRequest();
  request.set_observation_names(0, "missing");
  EXPECT_EQ(core.RegisterAdapter(request).code(), mujoco_sim::v2::NOT_FOUND);
  EXPECT_EQ(core.observer_count(mujoco_sim::v2::PROVISIONAL), 0U);

  request = ObserverRequest();
  request.mutable_delivery()->set_every_n_steps(0);
  EXPECT_EQ(core.RegisterAdapter(request).code(),
            mujoco_sim::v2::INVALID_ARGUMENT);
  EXPECT_EQ(core.observer_count(mujoco_sim::v2::PROVISIONAL), 0U);

  request = ObserverRequest();
  request.set_protocol_minor(1);
  EXPECT_EQ(core.RegisterAdapter(request).code(),
            mujoco_sim::v2::VERSION_MISMATCH);
  EXPECT_EQ(core.observer_count(mujoco_sim::v2::PROVISIONAL), 0U);

  const auto valid = core.RegisterAdapter(ObserverRequest());
  EXPECT_EQ(valid.code(), mujoco_sim::v2::OK);
  EXPECT_EQ(valid.session_id(), 1U);
}

TEST(SimulationCoreTest, ObserverReceivesPostStepFramesWithoutJoiningBarrier) {
  auto core = LoadedCore();
  const auto registration =
      RegisterAndActivateObserver(&core, ObserverRequest());
  EXPECT_EQ(core.active_controller_count(), 0U);
  EXPECT_EQ(core.observer_count(mujoco_sim::v2::ACTIVE), 1U);
  EXPECT_TRUE(core.OpenIssuedTicks().empty());
  auto completed = core.CompleteEmptyBarrier().step_completed;
  ASSERT_TRUE(completed.has_value());
  const auto& frames = completed->observer_frames;
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].adapter_id(), registration.adapter_id());
  EXPECT_EQ(frames[0].sequence(), 1U);
  EXPECT_EQ(frames[0].header().tick_id(), 1U);
  EXPECT_DOUBLE_EQ(frames[0].header().simulation_time_s(),
                   core.simulation_time());
  ASSERT_EQ(frames[0].observations_size(), 2);
  EXPECT_EQ(frames[0].observations(0).name(), "hinge_pos");
}

TEST(SimulationCoreTest, ObserverSupportsStepAndSimulationTimeDecimation) {
  auto core = LoadedCore();
  auto request = ObserverRequest("step-observer");
  request.mutable_delivery()->set_every_n_steps(2);
  RegisterAndActivateObserver(&core, request);
  request = ObserverRequest("period-observer");
  request.mutable_delivery()->set_period_s(core.timestep() * 2.0);
  RegisterAndActivateObserver(&core, request);

  for (int step = 1; step <= 3; ++step) {
    EXPECT_TRUE(core.OpenIssuedTicks().empty());
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
  ASSERT_DOUBLE_EQ(core.timestep(), 0.001);
  auto request = ObserverRequest("phase-observer");
  request.mutable_delivery()->set_period_s(0.0015);
  RegisterAndActivateObserver(&core, request);

  std::vector<mujoco_sim::v2::ObservationFrame> frames;
  for (int step = 0; step < 10; ++step) {
    EXPECT_TRUE(core.OpenIssuedTicks().empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    frames.insert(frames.end(), completed->observer_frames.begin(),
                  completed->observer_frames.end());
  }
  const std::vector<std::uint64_t> expected_ticks = {1, 3, 4, 6, 7, 9, 10};
  ASSERT_EQ(frames.size(), expected_ticks.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    EXPECT_EQ(frames[index].header().tick_id(), expected_ticks[index]);
    EXPECT_EQ(frames[index].sequence(), index + 1);
  }
}

TEST(SimulationCoreTest, ObserverPeriodRemainsPhaseLockedForLongRuns) {
  auto core = LoadedCore();
  auto request = ObserverRequest("long-phase-observer");
  request.mutable_delivery()->set_period_s(0.0015);
  RegisterAndActivateObserver(&core, request);

  std::uint64_t period_index = 1;
  for (std::uint64_t tick = 1; tick <= 20000; ++tick) {
    EXPECT_TRUE(core.OpenIssuedTicks().empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    const auto& frames = completed->observer_frames;
    const std::uint64_t expected_tick =
        period_index == 0 ? 1 : 1 + (3 * period_index + 1) / 2;
    const bool due = tick == 1 || tick == expected_tick;
    ASSERT_EQ(frames.size(), due ? 1U : 0U) << "tick=" << tick;
    if (due && tick != 1) {
      EXPECT_EQ(frames[0].header().tick_id(), tick);
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
    EXPECT_TRUE(core.OpenIssuedTicks().empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    const auto& frames = completed->observer_frames;
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].header().tick_id(), tick);
  }
}

TEST(SimulationCoreTest, CompletedFramesSurviveUnregisterUntilCollected) {
  auto core = LoadedCore();
  const auto registration =
      RegisterAndActivateObserver(&core, ObserverRequest());
  std::vector<mujoco_sim::v2::ObservationFrame> frames;
  for (int step = 0; step < 2; ++step) {
    EXPECT_TRUE(core.OpenIssuedTicks().empty());
    auto completed = core.CompleteEmptyBarrier().step_completed;
    ASSERT_TRUE(completed.has_value());
    frames.insert(frames.end(), completed->observer_frames.begin(),
                  completed->observer_frames.end());
  }

  mujoco_sim::v2::UnregisterAdapterRequest unregister;
  unregister.set_adapter_id(registration.adapter_id());
  unregister.set_role(mujoco_sim::v2::OBSERVER);
  unregister.set_session_id(registration.session_id());
  unregister.set_epoch(registration.epoch());
  ASSERT_EQ(core.UnregisterAdapter(unregister).response.code(), mujoco_sim::v2::OK);

  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].header().tick_id(), 1U);
  EXPECT_EQ(frames[0].sequence(), 1U);
  EXPECT_EQ(frames[1].header().tick_id(), 2U);
  EXPECT_EQ(frames[1].sequence(), 2U);
}

TEST(SimulationCoreTest, ObserverCannotActivateWhenServerIsStoppedOrPaused) {
  for (const bool stopped : {false, true}) {
    SCOPED_TRACE(stopped ? "stopped" : "error-paused");
    auto core = LoadedCore();
    const auto registration = core.RegisterAdapter(ObserverRequest());
    ASSERT_EQ(registration.code(), mujoco_sim::v2::OK);

    if (stopped) {
      core.MarkStopped("shutdown");
    } else {
      core.EnterFatalError("transport failed");
    }

    mujoco_sim::v2::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::v2::OBSERVER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    EXPECT_EQ(core.ActivateAdapter(activate).code(),
              mujoco_sim::v2::FAILED_PRECONDITION);
    EXPECT_EQ(core.observer_count(mujoco_sim::v2::PROVISIONAL), 1U);
  }
}

TEST(SimulationCoreTest, ControllerCannotActivateWhenServerIsStoppedOrPaused) {
  for (const bool stopped : {false, true}) {
    SCOPED_TRACE(stopped ? "stopped" : "error-paused");
    auto core = LoadedCore();
    const auto registration = core.RegisterAdapter(BasicRegisterRequest());
    ASSERT_EQ(registration.code(), mujoco_sim::v2::OK);

    if (stopped) {
      core.MarkStopped("shutdown");
    } else {
      core.EnterFatalError("transport failed");
    }

    mujoco_sim::v2::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::v2::CONTROLLER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    EXPECT_EQ(core.ActivateAdapter(activate).code(),
              mujoco_sim::v2::FAILED_PRECONDITION);
    EXPECT_EQ(core.controller_count(mujoco_sim::v2::PROVISIONAL), 1U);
  }
}

TEST(SimulationCoreTest, ObserverKeepaliveTimeoutAndReplacementAreIsolated) {
  auto core = LoadedCore();
  const auto registration =
      RegisterAndActivateObserver(&core, ObserverRequest());
  const auto future = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100);
  const auto deadlines = core.CheckDeadlines(future);
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_EQ(deadlines.transitions[0].new_state,
            mujoco_sim::v2::TIMED_OUT);
  EXPECT_EQ(core.observer_count(mujoco_sim::v2::TIMED_OUT), 1U);
  EXPECT_TRUE(core.OpenIssuedTicks().empty());
  EXPECT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());

  const auto replacement = core.RegisterAdapter(ObserverRequest());
  ASSERT_EQ(replacement.code(), mujoco_sim::v2::OK);
  EXPECT_NE(replacement.session_id(), registration.session_id());
  EXPECT_NE(replacement.message().find("replaced"), std::string::npos);
}

TEST(SimulationCoreTest, StepsWithAnEmptyBarrier) {
  auto core = LoadedCore();
  const double before = core.simulation_time();
  EXPECT_TRUE(core.OpenIssuedTicks().empty());
  EXPECT_TRUE(core.tick_open());
  EXPECT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());
  EXPECT_EQ(core.current_tick_id(), 1U);
  EXPECT_DOUBLE_EQ(core.simulation_time(), before + core.timestep());
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
}

TEST(SimulationCoreTest, ReportsEveryStepCompletionSourceWithObserverFrames) {
  const auto add_observer = [](mujoco_sim::core::SimulationCore* core,
                               const std::string& id) {
    auto request = ObserverRequest(id);
    request.set_timeout_ms(10000);
    RegisterAndActivateObserver(core, request);
  };

  {
    auto core = LoadedCore();
    add_observer(&core, "empty_observer");
    core.OpenIssuedTicks();
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
    core.OpenIssuedTicks();
    const auto result = core.SubmitCommit(CommitFor(registration, 0, 0.1));
    ASSERT_TRUE(result.step_completed.has_value());
    EXPECT_EQ(result.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kCommit);
    EXPECT_EQ(result.step_completed->observer_frames.size(), 1U);
  }
  {
    auto core = LoadedCore();
    add_observer(&core, "deadline_observer");
    RegisterAndActivate(&core);
    core.OpenIssuedTicks();
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
    core.OpenIssuedTicks();
    mujoco_sim::v2::UnregisterAdapterRequest request;
    request.set_adapter_id(registration.adapter_id());
    request.set_role(mujoco_sim::v2::CONTROLLER);
    request.set_session_id(registration.session_id());
    request.set_epoch(registration.epoch());
    const auto result = core.UnregisterAdapter(request);
    ASSERT_TRUE(result.step_completed.has_value());
    EXPECT_EQ(result.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kUnregister);
    EXPECT_EQ(result.step_completed->observer_frames.size(), 1U);
  }
  {
    auto core = LoadedCore();
    add_observer(&core, "error_observer");
    const auto registration = RegisterAndActivate(&core);
    core.OpenIssuedTicks();
    const auto result = core.SubmitCommit(CommitFor(
        registration, 0, std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(result.step_completed.has_value());
    EXPECT_EQ(result.step_completed->source,
              mujoco_sim::core::StepCompletionSource::kLocalError);
    EXPECT_EQ(result.step_completed->observer_frames.size(), 1U);
  }
}

TEST(SimulationCoreTest, InvalidatesTickHandlesAtEveryLifecycleBoundary) {
  std::string error;
  {
    auto core = LoadedCore();
    RegisterAndActivate(&core);
    const auto handle = core.OpenIssuedTicks().front();
    ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
    EXPECT_FALSE(handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(handle->identity));
  }
  {
    auto core = LoadedCore();
    const auto registration = RegisterAndActivate(&core);
    const auto handle = core.OpenIssuedTicks().front();
    mujoco_sim::v2::UnregisterAdapterRequest request;
    request.set_adapter_id(registration.adapter_id());
    request.set_role(mujoco_sim::v2::CONTROLLER);
    request.set_session_id(registration.session_id());
    request.set_epoch(registration.epoch());
    ASSERT_EQ(core.UnregisterAdapter(request).response.code(),
              mujoco_sim::v2::OK);
    EXPECT_FALSE(handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(handle->identity));
  }
  {
    auto core = LoadedCore();
    const auto old_registration = RegisterAndActivate(&core);
    core.OpenIssuedTicks();
    ASSERT_TRUE(core.CheckDeadlines(std::chrono::steady_clock::now() +
                                    std::chrono::seconds(2))
                    .step_completed.has_value());
    const auto recovery_handle = core.OpenIssuedTicks().front();
    const auto replacement = core.RegisterAdapter(BasicRegisterRequest());
    ASSERT_EQ(replacement.code(), mujoco_sim::v2::OK);
    EXPECT_NE(replacement.session_id(), old_registration.session_id());
    EXPECT_FALSE(recovery_handle->current.load());
    EXPECT_FALSE(core.IsIssuedTickCurrent(recovery_handle->identity));
  }
  {
    auto core = LoadedCore();
    RegisterAndActivate(&core);
    const auto handle = core.OpenIssuedTicks().front();
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
  mujoco_sim::core::SimulationCore source_core;
  mujoco_sim::core::SimulationCore options_core;
  std::string error;
  ASSERT_TRUE(source_core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error))
      << error;
  ASSERT_TRUE(options_core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL,
                                    mujoco_sim::core::ModelLoadOptions{},
                                    &error))
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
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);
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
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
  EXPECT_TRUE(core.tick_open());
  EXPECT_EQ(core.active_controller_count(), 1);
  ASSERT_EQ(core.commands().size(), 1);
  EXPECT_EQ(core.commands()[0].name(), "hinge_motor");
  EXPECT_TRUE(core.UpdateSnapshot(snapshot.get(), &error)) << error;

  const auto result = core.SubmitCommit(CommitFor(registration, tick_id, 0.2));
  EXPECT_EQ(result.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(result.step_completed.has_value());
}

TEST(SimulationCoreTest, ProvidesAvailableResources) {
  auto core = LoadedCore();
  mujoco_sim::v2::GetAvailableResourcesRequest request;
  request.set_server_name("default");
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  const auto response = core.GetAvailableResources(request);
  EXPECT_EQ(response.code(), mujoco_sim::v2::OK);
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

  ASSERT_TRUE(core.UpdateSnapshot(clone.get(), &error)) << error;
  EXPECT_DOUBLE_EQ(clone->data()->qpos[0], core_state->data()->qpos[0]);
  EXPECT_DOUBLE_EQ(clone->data()->time, core_state->data()->time);
  EXPECT_NE(clone->data()->qpos[0], source->data()->qpos[0]);

  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  EXPECT_EQ(core.CloneSnapshot(*source, &error), nullptr);
  EXPECT_NE(error.find("stale"), std::string::npos);
}

TEST(SimulationCoreTest, RefreshesVisualizationSnapshotAfterStep) {
  auto core = LoadedCore();
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.2)).step_completed.has_value());
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

TEST(SimulationCoreTest, ReusedMovedFromCoresKeepSnapshotOwnershipDistinct) {
  auto first = LoadedCore();
  auto second = LoadedCore();
  auto first_destination = std::move(first);
  auto second_destination = std::move(second);
  EXPECT_TRUE(first_destination.IsModelLoaded());
  EXPECT_TRUE(second_destination.IsModelLoaded());

  std::string error;
  ASSERT_TRUE(first.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  ASSERT_TRUE(second.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  auto snapshot = first.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  EXPECT_FALSE(second.UpdateSnapshot(snapshot.get(), &error));
  EXPECT_NE(error.find("does not belong"), std::string::npos);
  EXPECT_EQ(second.CloneSnapshot(*snapshot, &error), nullptr);
  EXPECT_NE(error.find("invalid or stale"), std::string::npos);
}

TEST(SimulationCoreTest, RejectsUnknownAndConflictingResources) {
  auto core = LoadedCore();
  auto bad = BasicRegisterRequest("bad");
  bad.set_observation_names(0, "missing_sensor");
  EXPECT_EQ(core.RegisterAdapter(bad).code(), mujoco_sim::v2::NOT_FOUND);

  auto first = core.RegisterAdapter(BasicRegisterRequest("first"));
  EXPECT_EQ(first.code(), mujoco_sim::v2::OK);
  auto conflict = core.RegisterAdapter(BasicRegisterRequest("second"));
  EXPECT_EQ(conflict.code(), mujoco_sim::v2::RESOURCE_CONFLICT);
}

TEST(SimulationCoreTest, ActivatesAndPublishesTickWithObservation) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  const auto ticks = core.OpenIssuedTicks();
  ASSERT_EQ(ticks.size(), 1);
  EXPECT_TRUE(core.tick_open());
  EXPECT_EQ(ticks[0]->message.adapter_id(), registration.adapter_id());
  EXPECT_EQ(ticks[0]->message.header().tick_id(), 0);
  EXPECT_GE(ticks[0]->message.observations_size(), 1);
}

TEST(SimulationCoreTest, ControllerDeliveryStatusDistinguishesUnsentTickZero) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  auto status = AdapterStatusFor(core, registration.adapter_id());
  EXPECT_FALSE(status.has_last_delivery_tick_id());

  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);
  status = AdapterStatusFor(core, registration.adapter_id());
  ASSERT_TRUE(status.has_last_delivery_tick_id());
  EXPECT_EQ(status.last_delivery_tick_id(), 0U);
}

TEST(SimulationCoreTest, StepsWhenBarrierCompletesAndHandlesDuplicateCommits) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);

  const auto commit = CommitFor(registration, 0, 0.2);
  const auto duplicate_in_open_tick = core.SubmitCommit(commit);
  EXPECT_EQ(duplicate_in_open_tick.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(duplicate_in_open_tick.step_completed.has_value());
  const auto stale_duplicate = core.SubmitCommit(commit);
  EXPECT_EQ(stale_duplicate.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(stale_duplicate.duplicate);
  EXPECT_NE(core.server_state(), mujoco_sim::v2::ERROR_PAUSED);
  EXPECT_EQ(core.current_tick_id(), 1);

  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);
  const auto duplicate_current_tick = core.SubmitCommit(CommitFor(registration, 1, 0.2));
  EXPECT_EQ(duplicate_current_tick.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(duplicate_current_tick.step_completed.has_value());
  const auto result = core.SubmitCommit(CommitFor(registration, 1, 0.2));
  EXPECT_EQ(result.code, mujoco_sim::v2::OK);
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
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);

  const auto first_commit = CommitFor(first, 0, 0.2);
  const auto accepted = core.SubmitCommit(first_commit);
  EXPECT_EQ(accepted.code, mujoco_sim::v2::OK);
  EXPECT_FALSE(accepted.step_completed.has_value());

  const auto same_duplicate = core.SubmitCommit(first_commit);
  EXPECT_EQ(same_duplicate.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(same_duplicate.duplicate);
  EXPECT_NE(core.server_state(), mujoco_sim::v2::ERROR_PAUSED);

  const auto different_duplicate = core.SubmitCommit(CommitFor(first, 0, 0.3));
  EXPECT_EQ(different_duplicate.code, mujoco_sim::v2::INVALID_COMMAND);
  EXPECT_TRUE(different_duplicate.controller_state_changed);
  EXPECT_EQ(different_duplicate.controller_state, mujoco_sim::v2::ERROR);
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
  EXPECT_EQ(AdapterStatusFor(core, "first").state(),
            mujoco_sim::v2::ERROR);
  EXPECT_EQ(AdapterStatusFor(core, "second").state(),
            mujoco_sim::v2::ACTIVE);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.1)).step_completed.has_value());
}

TEST(SimulationCoreTest, StaleDuplicatePayloadMismatchErrorsItsController) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.2)).step_completed.has_value());

  const auto mismatch = core.SubmitCommit(CommitFor(registration, 0, 0.3));
  EXPECT_EQ(mismatch.code, mujoco_sim::v2::INVALID_COMMAND);
  EXPECT_TRUE(mismatch.controller_state_changed);
  EXPECT_EQ(mismatch.controller_state, mujoco_sim::v2::ERROR);
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
}

TEST(SimulationCoreTest, UnverifiableOlderCommitIsRejected) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);
  const auto first_commit = CommitFor(registration, 0, 0.2);
  ASSERT_TRUE(core.SubmitCommit(first_commit).step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);
  ASSERT_TRUE(core.SubmitCommit(CommitFor(registration, 1, 0.1)).step_completed.has_value());

  const auto stale = core.SubmitCommit(first_commit);
  EXPECT_EQ(stale.code, mujoco_sim::v2::INVALID_TICK);
  EXPECT_FALSE(stale.controller_state_changed);
  EXPECT_EQ(AdapterStatusFor(core, registration.adapter_id()).state(),
            mujoco_sim::v2::ACTIVE);
}

TEST(SimulationCoreTest, InvalidCommandOnlyErrorsItsController) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);

  auto commit =
      CommitFor(first, 0, std::numeric_limits<double>::quiet_NaN());
  const auto result = core.SubmitCommit(commit);
  EXPECT_EQ(result.code, mujoco_sim::v2::INVALID_COMMAND);
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
  EXPECT_EQ(AdapterStatusFor(core, "first").state(),
            mujoco_sim::v2::ERROR);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.2)).step_completed.has_value());

  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[0], 0.0);
}

TEST(SimulationCoreTest, IdentityMismatchesDoNotChangeControllerState) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);

  auto wrong_session = CommitFor(registration, 0, 0.0);
  wrong_session.mutable_header()->set_session_id(registration.session_id() + 1);
  EXPECT_EQ(core.SubmitCommit(wrong_session).code,
            mujoco_sim::v2::INVALID_SESSION);
  auto wrong_epoch = CommitFor(registration, 0, 0.0);
  wrong_epoch.mutable_header()->set_epoch(registration.epoch() + 1);
  EXPECT_EQ(core.SubmitCommit(wrong_epoch).code, mujoco_sim::v2::INVALID_EPOCH);
  EXPECT_EQ(core.SubmitCommit(CommitFor(registration, 1, 0.0)).code,
            mujoco_sim::v2::INVALID_TICK);
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
  EXPECT_EQ(AdapterStatusFor(core, "arm").state(),
            mujoco_sim::v2::ACTIVE);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).step_completed.has_value());
}

TEST(SimulationCoreTest, RegistersAndActivatesWhileRunning) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  EXPECT_TRUE(core.OpenIssuedTicks().empty());
  EXPECT_TRUE(core.CompleteEmptyBarrier().step_completed.has_value());
  EXPECT_EQ(core.current_tick_id(), 1U);

  const auto first = RegisterAndActivate(
      &core, BasicRegisterRequest("first", "hinge_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);
  const auto second_registration =
      core.RegisterAdapter(BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(second_registration.code(), mujoco_sim::v2::OK);
  mujoco_sim::v2::ActivateAdapterRequest activate;
  activate.set_adapter_id("second");
  activate.set_role(mujoco_sim::v2::CONTROLLER);
  activate.set_session_id(second_registration.session_id());
  activate.set_epoch(second_registration.epoch());
  EXPECT_EQ(core.ActivateAdapter(activate).code(), mujoco_sim::v2::OK);
  EXPECT_EQ(core.OpenIssuedTicks().size(), 1);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(first, 1, 0.1)).step_completed.has_value());
  EXPECT_EQ(core.OpenIssuedTicks().size(), 2);
}

TEST(SimulationCoreTest, UnregistersRequiredControllerDuringOpenTick) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.2)).step_completed.has_value());

  mujoco_sim::v2::UnregisterAdapterRequest request;
  request.set_adapter_id("second");
  request.set_role(mujoco_sim::v2::CONTROLLER);
  request.set_session_id(second.session_id());
  request.set_epoch(second.epoch());
  EXPECT_EQ(core.UnregisterAdapter(request).response.code(), mujoco_sim::v2::OK);
  EXPECT_EQ(core.current_tick_id(), 1U);
  EXPECT_EQ(core.MakeStatus().adapters_size(), 1);
}

TEST(SimulationCoreTest, DefersUnregisterControlWriteUntilBarrierCompletes) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2U);
  ASSERT_FALSE(
      core.SubmitCommit(CommitFor(first, 0, 0.2)).step_completed.has_value());
  ASSERT_TRUE(
      core.SubmitCommit(CommitFor(second, 0, 0.3)).step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2U);

  mujoco_sim::v2::UnregisterAdapterRequest request;
  request.set_adapter_id("first");
  request.set_role(mujoco_sim::v2::CONTROLLER);
  request.set_session_id(first.session_id());
  request.set_epoch(first.epoch());
  const auto unregistered = core.UnregisterAdapter(request);
  ASSERT_EQ(unregistered.response.code(), mujoco_sim::v2::OK);
  EXPECT_FALSE(unregistered.step_completed.has_value());

  std::string error;
  auto before_completion = core.CreateSnapshot(&error);
  ASSERT_NE(before_completion, nullptr) << error;
  EXPECT_DOUBLE_EQ(before_completion->data()->ctrl[0], 0.2);

  ASSERT_TRUE(
      core.SubmitCommit(CommitFor(second, 1, 0.1)).step_completed.has_value());
  auto after_completion = core.CreateSnapshot(&error);
  ASSERT_NE(after_completion, nullptr) << error;
  EXPECT_DOUBLE_EQ(after_completion->data()->ctrl[0], 0.0);
}

TEST(SimulationCoreTest, ExpiresProvisionalControllerAndReleasesActuator) {
  mujoco_sim::core::CoreOptions options;
  options.activation_timeout_ms = 10;
  mujoco_sim::core::SimulationCore core(options);
  std::string error;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  EXPECT_EQ(core.RegisterAdapter(BasicRegisterRequest("first")).code(),
            mujoco_sim::v2::OK);
  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_EQ(deadlines.transitions[0].old_state, mujoco_sim::v2::PROVISIONAL);
  EXPECT_EQ(deadlines.transitions[0].new_state,
            mujoco_sim::v2::UNREGISTERED);
  EXPECT_EQ(core.RegisterAdapter(BasicRegisterRequest("second")).code(),
            mujoco_sim::v2::OK);
}

TEST(SimulationCoreTest, TimesOutOneControllerAndContinuesBarrier) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.2)).step_completed.has_value());
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.3)).step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 1, 0.2)).step_completed.has_value());

  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_EQ(deadlines.transitions[0].adapter_id, "second");
  EXPECT_EQ(deadlines.transitions[0].new_state, mujoco_sim::v2::TIMED_OUT);
  EXPECT_TRUE(deadlines.step_completed.has_value());
  EXPECT_EQ(core.server_state(), mujoco_sim::v2::RUNNING);
  const auto timed_out = AdapterStatusFor(core, "second");
  EXPECT_EQ(timed_out.state(), mujoco_sim::v2::TIMED_OUT);
  EXPECT_EQ(timed_out.timeout_count(), 1U);
  EXPECT_FALSE(timed_out.required_for_barrier());
  std::string error;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[1], 0.0);
  EXPECT_EQ(core.OpenIssuedTicks().size(), 2);
}

TEST(SimulationCoreTest, KeepsIssuedTickPayloadImmutableAfterTimeout) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  auto first_request = BasicRegisterRequest("first", "hinge_motor");
  first_request.set_timeout_ms(10);
  const auto first = RegisterAndActivate(&core, first_request);
  auto second_request = BasicRegisterRequest("second", "second_motor");
  second_request.set_timeout_ms(100);
  const auto second = RegisterAndActivate(&core, second_request);
  const auto issued = core.OpenIssuedTicks();
  ASSERT_EQ(issued.size(), 2);
  const auto first_tick = std::find_if(
      issued.begin(), issued.end(),
      [](const auto& tick) { return tick->message.adapter_id() == "first"; });
  ASSERT_NE(first_tick, issued.end());
  EXPECT_EQ((*first_tick)->message.controller_state(), mujoco_sim::v2::ACTIVE);
  EXPECT_EQ((*first_tick)->serialized_bytes,
            (*first_tick)->message.SerializeAsString());
  EXPECT_EQ((*first_tick)->serialization_count, 1U);
  EXPECT_GT((*first_tick)->layout_bytes_avoided, 0U);

  const auto deadlines = core.CheckDeadlines(
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20));
  ASSERT_EQ(deadlines.transitions.size(), 1U);
  EXPECT_FALSE(deadlines.step_completed.has_value());
  const auto retransmitted = core.OpenIssuedTicks();
  const auto repeated_first = std::find_if(
      retransmitted.begin(), retransmitted.end(),
      [](const auto& tick) { return tick->message.adapter_id() == "first"; });
  ASSERT_NE(repeated_first, retransmitted.end());
  EXPECT_EQ((*first_tick).get(), (*repeated_first).get());
  EXPECT_EQ((*first_tick)->serialized_bytes,
            (*repeated_first)->serialized_bytes);
  EXPECT_EQ((*repeated_first)->serialization_count, 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(second, 0, 0.1)).step_completed.has_value());
  EXPECT_FALSE((*first_tick)->current.load());
  (void)first;
}

TEST(SimulationCoreTest, RecoversTimedOutControllerOnNextTick) {
  auto core = LoadedCoreFrom(MUJOCO_SIM_CORE_DUAL_TEST_MODEL);
  const auto first =
      RegisterAndActivate(&core, BasicRegisterRequest("first", "hinge_motor"));
  const auto second = RegisterAndActivate(
      &core, BasicRegisterRequest("second", "second_motor"));
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);
  EXPECT_FALSE(core.SubmitCommit(CommitFor(first, 0, 0.1)).step_completed.has_value());
  EXPECT_TRUE(core.CheckDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .step_completed.has_value());

  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);
  const auto recovery = core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(recovery.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(recovery.recovery_pending);
  EXPECT_FALSE(recovery.step_completed.has_value());
  const auto duplicate = core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(duplicate.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(duplicate.duplicate);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(first, 1, 0.1)).step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().size(), 2);
  EXPECT_EQ(AdapterStatusFor(core, "second").state(),
            mujoco_sim::v2::ACTIVE);
  EXPECT_TRUE(AdapterStatusFor(core, "second").required_for_barrier());
  const auto delayed_duplicate =
      core.SubmitCommit(CommitFor(second, 1, 0.3));
  EXPECT_EQ(delayed_duplicate.code, mujoco_sim::v2::OK);
  EXPECT_TRUE(delayed_duplicate.duplicate);
  EXPECT_EQ(AdapterStatusFor(core, "second").state(),
            mujoco_sim::v2::ACTIVE);
}

TEST(SimulationCoreTest, KeepsHeartbeatTickMonotonic) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.1)).step_completed.has_value());
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);

  mujoco_sim::v2::AdapterHeartbeat heartbeat;
  heartbeat.set_adapter_id(registration.adapter_id());
  heartbeat.set_role(mujoco_sim::v2::CONTROLLER);
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

  EXPECT_EQ(AdapterStatusFor(core, "arm").last_heartbeat_tick_id(), 1U);
}

TEST(SimulationCoreTest, ReplacesTimedOutControllerWithNewSession) {
  auto core = LoadedCore();
  const auto old_registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);
  EXPECT_TRUE(core.CheckDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .step_completed.has_value());

  const auto replacement = core.RegisterAdapter(BasicRegisterRequest());
  ASSERT_EQ(replacement.code(), mujoco_sim::v2::OK);
  EXPECT_NE(replacement.session_id(), old_registration.session_id());
  EXPECT_NE(replacement.message().find("replaced"), std::string::npos);
  EXPECT_EQ(core.SubmitCommit(CommitFor(old_registration, 0, 0.1)).code,
            mujoco_sim::v2::INVALID_SESSION);
  EXPECT_EQ(AdapterStatusFor(core, "arm").state(),
            mujoco_sim::v2::PROVISIONAL);
}

TEST(SimulationCoreTest, CanDisableTimedOutControllerReplacement) {
  mujoco_sim::core::CoreOptions options;
  options.allow_timed_out_adapter_replacement = false;
  mujoco_sim::core::SimulationCore core(options);
  std::string error;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_CORE_TEST_MODEL, &error)) << error;
  RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1);
  EXPECT_TRUE(core.CheckDeadlines(
                      std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(100))
                  .step_completed.has_value());
  EXPECT_EQ(core.RegisterAdapter(BasicRegisterRequest()).code(),
            mujoco_sim::v2::ALREADY_EXISTS);
}

TEST(SimulationCoreTest, FatalErrorUpdatesControllerStateTimestamp) {
  auto core = LoadedCore();
  const auto registration = RegisterAndActivate(&core);
  ASSERT_EQ(core.OpenIssuedTicks().size(), 1U);
  EXPECT_TRUE(core.SubmitCommit(CommitFor(registration, 0, 0.0)).step_completed.has_value());
  ASSERT_EQ(core.current_tick_id(), 1U);

  core.EnterFatalError("fatal transport error");
  const auto status = AdapterStatusFor(core, "arm");
  EXPECT_EQ(status.state(), mujoco_sim::v2::ERROR);
  EXPECT_EQ(status.state_since_tick_id(), 1U);
  EXPECT_EQ(status.message(), "fatal transport error");
}

}  // namespace
