#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include <mujoco_sim/core/simulation_core.hpp>

#include "adapter_registration.hpp"

namespace mujoco_sim::server {
namespace {

mujoco_sim::core::RegisterAdapterRequest ControllerRequest() {
  mujoco_sim::core::RegisterAdapterRequest request;
  request.set_adapter_id("arm");
  request.set_role(mujoco_sim::core::CONTROLLER);
  request.add_observation_names("hinge_pos");
  request.add_command_names("hinge_motor");
  return request;
}

auto RegisterErrorController(mujoco_sim::core::SimulationCore* core) {
  auto registration = core->RegisterAdapter(ControllerRequest(), {});
  EXPECT_EQ(registration.response.code(), mujoco_sim::core::OK);

  mujoco_sim::core::ActivateAdapterRequest activate;
  activate.set_session_id(registration.response.session_id());
  activate.set_epoch(registration.response.epoch());
  EXPECT_EQ(core->ActivateAdapter(activate, {}).response.code(), mujoco_sim::core::OK);

  mujoco_sim::core::ControllerFault fault;
  fault.session = {registration.response.session_id(), registration.response.epoch()};
  fault.code = mujoco_sim::core::FaultCode::kControllerError;
  EXPECT_EQ(core->SubmitFault(fault).response.code, mujoco_sim::core::OK);
  return registration.response;
}

void ExpectSessionCanUnregister(mujoco_sim::core::SimulationCore* core,
                                const mujoco_sim::core::RegisterAdapterResponse& registration) {
  mujoco_sim::core::UnregisterAdapterRequest unregister;
  unregister.set_session_id(registration.session_id());
  unregister.set_epoch(registration.epoch());
  EXPECT_EQ(core->UnregisterAdapter(unregister).response.code(), mujoco_sim::core::OK);
}

TEST(AdapterRegistrationTest, PublisherFailurePreservesReplaceableSession) {
  mujoco_sim::core::SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_SERVER_TEST_MODEL).response.loaded);
  const auto old_registration = RegisterErrorController(&core);
  ControllerCommandInbox inbox;
  ASSERT_TRUE(inbox.Register(old_registration.session_id()));
  int rollback_count = 0;

  auto result = RegisterAdapterAtomically(
      &core, &inbox, ControllerRequest(), {},
      [](std::string_view, mujoco_sim::core::AdapterRole) {
        return PublisherPreparation{false, false, "publisher failed"};
      },
      [&](std::string_view, mujoco_sim::core::AdapterRole) { ++rollback_count; });

  EXPECT_EQ(result.response.code(), mujoco_sim::core::INTERNAL_ERROR);
  EXPECT_EQ(result.response.message(), "publisher failed");
  EXPECT_EQ(rollback_count, 0);
  ExpectSessionCanUnregister(&core, old_registration);
}

TEST(AdapterRegistrationTest, ClosedInboxPreservesReplaceableSession) {
  mujoco_sim::core::SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_SERVER_TEST_MODEL).response.loaded);
  const auto old_registration = RegisterErrorController(&core);
  ControllerCommandInbox inbox;
  inbox.Close();
  int rollback_count = 0;

  auto result = RegisterAdapterAtomically(
      &core, &inbox, ControllerRequest(), {},
      [](std::string_view, mujoco_sim::core::AdapterRole) {
        return PublisherPreparation{true, true, {}};
      },
      [&](std::string_view, mujoco_sim::core::AdapterRole) { ++rollback_count; });

  EXPECT_EQ(result.response.code(), mujoco_sim::core::INTERNAL_ERROR);
  EXPECT_EQ(result.response.message(), "controller command inbox is closed");
  EXPECT_EQ(rollback_count, 1);
  ExpectSessionCanUnregister(&core, old_registration);
}

TEST(AdapterRegistrationTest, SuccessfulSetupCommitsReplacement) {
  mujoco_sim::core::SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_SERVER_TEST_MODEL).response.loaded);
  const auto old_registration = RegisterErrorController(&core);
  ControllerCommandInbox inbox;
  ASSERT_TRUE(inbox.Register(old_registration.session_id()));

  auto result = RegisterAdapterAtomically(
      &core, &inbox, ControllerRequest(), {},
      [](std::string_view, mujoco_sim::core::AdapterRole) {
        return PublisherPreparation{true, false, {}};
      },
      [](std::string_view, mujoco_sim::core::AdapterRole) {});

  ASSERT_EQ(result.response.code(), mujoco_sim::core::OK);
  EXPECT_NE(result.response.session_id(), old_registration.session_id());
  EXPECT_FALSE(
      inbox.Publish({{old_registration.session_id(), old_registration.epoch()}, {}, 1, {0.0}}));
  EXPECT_TRUE(
      inbox.Publish({{result.response.session_id(), result.response.epoch()}, {}, 1, {0.0}}));
}

TEST(AdapterRegistrationTest, InvalidRoleDoesNotPrepareRuntimeResources) {
  mujoco_sim::core::SimulationCore core;
  ASSERT_TRUE(core.LoadModel(MUJOCO_SIM_SERVER_TEST_MODEL).response.loaded);
  ControllerCommandInbox inbox;
  auto request = ControllerRequest();
  request.set_role(mujoco_sim::core::ADAPTER_ROLE_UNSPECIFIED);
  int prepare_count = 0;

  auto result = RegisterAdapterAtomically(
      &core, &inbox, request, {},
      [&](std::string_view, mujoco_sim::core::AdapterRole) {
        ++prepare_count;
        return PublisherPreparation{true, true, {}};
      },
      [](std::string_view, mujoco_sim::core::AdapterRole) {});

  EXPECT_EQ(result.response.code(), mujoco_sim::core::INVALID_ARGUMENT);
  EXPECT_EQ(prepare_count, 0);
}

}  // namespace
}  // namespace mujoco_sim::server
