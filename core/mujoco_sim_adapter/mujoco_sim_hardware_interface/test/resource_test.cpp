#include "resource.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using mujoco_sim::adapter_sdk::ControllerLayout;
using mujoco_sim::adapter_sdk::ResourceInfo;
using mujoco_sim::adapter_sdk::ScalarSpan;
using mujoco_sim::hardware::internal::InterfaceRegistry;
using mujoco_sim::hardware::internal::MakeResource;
using mujoco_sim::hardware::internal::ParameterMap;
using mujoco_sim::hardware::internal::ResourceInterface;
using mujoco_sim::hardware::internal::ValidateLayout;

hardware_interface::ComponentInfo Component(std::string name, std::string type) {
  hardware_interface::ComponentInfo component;
  component.name = std::move(name);
  component.type = std::move(type);
  return component;
}

ResourceInfo Observation(std::string name, std::size_t offset, std::size_t count) {
  return {std::move(name), {offset, count}, false, 0.0, 0.0};
}

ResourceInfo Command(std::string name, std::size_t offset, double minimum, double maximum) {
  return {std::move(name), {offset, 1}, true, minimum, maximum};
}

void RegisterInterfaces(ResourceInterface& resource) {
  InterfaceRegistry registry;
  std::string error;
  ASSERT_TRUE(resource.RegisterInterfaces(registry, error)) << error;
  (void)registry.ReleaseManifest();
}

TEST(HardwareOptionsTest, ParsesStrictPublicContractTransactionally) {
  ParameterMap parameters{{"server_id", "sim"},
                          {"adapter_id", "ros2_control"},
                          {"server_time_out_ms", "42"},
                          {"publish_sim_time", "true"},
                          {"debug", "false"}};
  mujoco_sim::hardware::internal::HardwareOptions options;
  std::string error;
  ASSERT_TRUE(mujoco_sim::hardware::internal::ParseHardwareOptions(parameters, options, error))
      << error;
  EXPECT_EQ(options.server_id, "sim");
  EXPECT_EQ(options.adapter_id, "ros2_control");
  EXPECT_EQ(options.server_timeout_ms, 42U);
  EXPECT_TRUE(options.publish_sim_time);

  parameters["publish_sim_time"] = "TRUE";
  EXPECT_FALSE(mujoco_sim::hardware::internal::ParseHardwareOptions(parameters, options, error));
  EXPECT_NE(error.find("exactly"), std::string::npos);
}

TEST(HardwareOptionsTest, RejectsUnknownOverflowAndNonAsciiIdentifiers) {
  mujoco_sim::hardware::internal::HardwareOptions options;
  options.adapter_id = "unchanged";
  std::string error;
  EXPECT_FALSE(mujoco_sim::hardware::internal::ParseHardwareOptions(
      {{"adapter_id", "controller"}, {"unknown", "value"}}, options, error));
  EXPECT_EQ(options.adapter_id, "unchanged");
  EXPECT_NE(error.find("unknown"), std::string::npos);

  EXPECT_FALSE(mujoco_sim::hardware::internal::ParseHardwareOptions(
      {{"adapter_id", "controller"}, {"server_time_out_ms", "4294967296"}}, options, error));
  EXPECT_NE(error.find("UINT32_MAX"), std::string::npos);

  EXPECT_FALSE(mujoco_sim::hardware::internal::ParseHardwareOptions(
      {{"adapter_id", "contr\xC3\xB4ller"}}, options, error));
  EXPECT_NE(error.find("adapter_id"), std::string::npos);
}

TEST(ResourceFactoryTest, ExportsAllSixJointMappingKinds) {
  ParameterMap parameters{
      {"actuator_position", "position_motor"}, {"actuator_velocity", "velocity_motor"},
      {"actuator_effort", "effort_motor"},     {"sensor_position", "position_sensor"},
      {"sensor_velocity", "velocity_sensor"},  {"sensor_effort", "effort_sensor"}};
  std::string error;
  auto resource = MakeResource(Component("joint", "joint"), parameters, error);
  ASSERT_NE(resource, nullptr) << error;
  ASSERT_EQ(resource->state_bindings().size(), 3U);
  ASSERT_EQ(resource->command_bindings().size(), 3U);
  EXPECT_EQ(resource->state_bindings()[0].FullName(), "joint/position");
  EXPECT_EQ(resource->state_bindings()[1].FullName(), "joint/velocity");
  EXPECT_EQ(resource->state_bindings()[2].FullName(), "joint/effort");
  EXPECT_EQ(resource->command_bindings()[0].FullName(), "joint/position");
  EXPECT_EQ(resource->command_bindings()[1].FullName(), "joint/velocity");
  EXPECT_EQ(resource->command_bindings()[2].FullName(), "joint/effort");
}

TEST(ResourceFactoryTest, RejectsEmptyUnknownAndInvalidSensorTypes) {
  std::string error;
  EXPECT_EQ(MakeResource(Component("joint", "joint"),
                         {{"actuator_velocity", ""}, {"sensor_position", "position"}}, error),
            nullptr);
  EXPECT_NE(error.find("must not be empty"), std::string::npos);

  EXPECT_EQ(MakeResource(Component("joint", "joint"),
                         {{"actuator_velocity", "motor"}, {"typo", "value"}}, error),
            nullptr);
  EXPECT_NE(error.find("unknown"), std::string::npos);

  EXPECT_EQ(MakeResource(Component("sensor", "sensor"),
                         {{"sensor_type", "other"},
                          {"sensor_name", "generic"},
                          {"sensor_dim", "1"},
                          {"sensor_dim_names", "x"},
                          {"accelerometer", "accel"}},
                         error),
            nullptr);
  EXPECT_NE(error.find("unknown parameter 'accelerometer'"), std::string::npos);

  EXPECT_EQ(
      MakeResource(Component("sensor", "sensor"),
                   {{"sensor_name", "generic"}, {"sensor_dim", "1"}, {"sensor_dim_names", "x"}},
                   error),
      nullptr);
  EXPECT_NE(error.find("requires sensor_type"), std::string::npos);

  EXPECT_EQ(MakeResource(Component("sensor", "sensor"), {{"sensor_type", "generic"}}, error),
            nullptr);
  EXPECT_NE(error.find("'imu' or 'other'"), std::string::npos);
}

TEST(ResourceFactoryTest, BuildsAndBindsJointMappingsWithClamping) {
  ParameterMap parameters{{"adapter_id", "adapter"},
                          {"sensor_position", "joint_pos"},
                          {"sensor_velocity", "joint_vel"},
                          {"actuator_velocity", "joint_motor"},
                          {"debug", "false"}};
  std::string error;
  auto resource = MakeResource(Component("wheel", "joint"), parameters, error);
  ASSERT_NE(resource, nullptr) << error;
  RegisterInterfaces(*resource);
  ControllerLayout layout({Observation("joint_vel", 0, 1), Observation("joint_pos", 1, 1)},
                          {Command("joint_motor", 0, -2.0, 2.0)}, 2, 1);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;

  const std::array observations{3.0, 4.0};
  std::array<double, 2> staged{};
  ASSERT_TRUE(resource->StageState(observations, staged, error)) << error;
  EXPECT_DOUBLE_EQ(staged[0], 4.0);
  EXPECT_DOUBLE_EQ(staged[1], 3.0);

  std::array ros_commands{9.0};
  std::array<double, 1> dense{};
  ASSERT_TRUE(resource->StageCommand(ros_commands, dense, error)) << error;
  EXPECT_DOUBLE_EQ(dense[0], 2.0);
  EXPECT_DOUBLE_EQ(ros_commands[0], 2.0);

  std::array nan_command{std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(resource->StageCommand(nan_command, dense, error));
  EXPECT_NE(error.find("non-finite"), std::string::npos);

  std::array infinite_command{std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(resource->StageCommand(infinite_command, dense, error));
}

TEST(ResourceFactoryTest, RejectsInvalidGenericSensorDimensions) {
  ParameterMap parameters{{"sensor_type", "other"},
                          {"sensor_name", "force"},
                          {"sensor_dim", "3"},
                          {"sensor_dim_names", "x y"}};
  std::string error;
  EXPECT_EQ(MakeResource(Component("force_sensor", "sensor"), parameters, error), nullptr);
  EXPECT_NE(error.find("exactly"), std::string::npos);
}

TEST(ResourceFactoryTest, GroupsGenericSensorInConfiguredOrder) {
  ParameterMap parameters{{"sensor_type", "other"},
                          {"sensor_name", "force"},
                          {"sensor_dim", "3"},
                          {"sensor_dim_names", "x y z"}};
  std::string error;
  auto resource = MakeResource(Component("force_sensor", "sensor"), parameters, error);
  ASSERT_NE(resource, nullptr) << error;
  RegisterInterfaces(*resource);
  ControllerLayout layout({Observation("force", 0, 3)}, {}, 3, 0);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;
  const std::array observations{1.0, 2.0, 3.0};
  std::array<double, 3> staged{};
  ASSERT_TRUE(resource->StageState(observations, staged, error)) << error;
  EXPECT_EQ(staged, observations);
}

TEST(ResourceFactoryTest, ConvertsAndNormalizesMujocoQuaternionForImu) {
  ParameterMap parameters{{"sensor_type", "imu"},   {"accelerometer", "accel"},
                          {"gyro", "gyro"},         {"target_link", "imu_link"},
                          {"father_link", "world"}, {"orientation", "quat"},
                          {"position", "position"}};
  std::string error;
  auto resource = MakeResource(Component("imu", "sensor"), parameters, error);
  ASSERT_NE(resource, nullptr) << error;
  RegisterInterfaces(*resource);
  ControllerLayout layout({Observation("accel", 0, 3), Observation("gyro", 3, 3),
                           Observation("quat", 6, 4), Observation("position", 10, 3)},
                          {}, 13, 0);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;
  const std::array observations{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 2.0, 0.0, 0.0, 0.0, 7.0, 8.0, 9.0};
  std::array<double, 13> staged{};
  ASSERT_TRUE(resource->StageState(observations, staged, error)) << error;
  EXPECT_DOUBLE_EQ(staged[6], 0.0);
  EXPECT_DOUBLE_EQ(staged[9], 1.0);
  EXPECT_DOUBLE_EQ(staged[10], 7.0);
  EXPECT_DOUBLE_EQ(staged[11], 8.0);
  EXPECT_DOUBLE_EQ(staged[12], 9.0);

  const auto previous = staged;
  auto invalid_observations = observations;
  invalid_observations[6] = 0.0;
  EXPECT_FALSE(resource->StageState(invalid_observations, staged, error));
  EXPECT_NE(error.find("quaternion"), std::string::npos);
  EXPECT_EQ(staged, previous);
}

TEST(ResourceFactoryTest, RejectsNonFiniteStateWithoutPartialSuccess) {
  ParameterMap parameters{{"sensor_type", "other"},
                          {"sensor_name", "force"},
                          {"sensor_dim", "3"},
                          {"sensor_dim_names", "x y z"}};
  std::string error;
  auto resource = MakeResource(Component("force_sensor", "sensor"), parameters, error);
  ASSERT_NE(resource, nullptr) << error;
  RegisterInterfaces(*resource);
  ControllerLayout layout({Observation("force", 0, 3)}, {}, 3, 0);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;
  const std::array observations{1.0, std::numeric_limits<double>::quiet_NaN(), 3.0};
  std::array<double, 3> staged{9.0, 9.0, 9.0};
  EXPECT_FALSE(resource->StageState(observations, staged, error));
  EXPECT_NE(error.find("non-finite"), std::string::npos);
  EXPECT_EQ(staged, (std::array<double, 3>{9.0, 9.0, 9.0}));
}

TEST(LayoutTest, AcceptsReorderedCompleteDenseSpans) {
  std::string error;
  std::vector<std::unique_ptr<ResourceInterface>> resources;
  resources.push_back(MakeResource(Component("joint", "joint"),
                                   {{"sensor_position", "position"},
                                    {"sensor_velocity", "velocity"},
                                    {"actuator_velocity", "motor"}},
                                   error));
  ASSERT_NE(resources.front(), nullptr) << error;
  ControllerLayout layout({Observation("velocity", 0, 1), Observation("position", 1, 1)},
                          {Command("motor", 0, -2.0, 2.0)}, 2, 1);
  EXPECT_TRUE(ValidateLayout(layout, resources, error)) << error;
}

TEST(LayoutTest, RejectsMissingWrongOutOfBoundsOverlappingAndHoleySpans) {
  std::string error;
  std::vector<std::unique_ptr<ResourceInterface>> resources;
  resources.push_back(MakeResource(Component("joint", "joint"),
                                   {{"sensor_position", "position"},
                                    {"sensor_velocity", "velocity"},
                                    {"actuator_velocity", "motor"}},
                                   error));
  ASSERT_NE(resources.front(), nullptr) << error;

  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 1)}, {Command("motor", 0, -1.0, 1.0)}, 1, 1),
      resources, error));
  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 2), Observation("velocity", 2, 1)},
                       {Command("motor", 0, -1.0, 1.0)}, 3, 1),
      resources, error));
  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 1), Observation("velocity", 2, 1)},
                       {Command("motor", 0, -1.0, 1.0)}, 2, 1),
      resources, error));
  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 1), Observation("velocity", 0, 1)},
                       {Command("motor", 0, -1.0, 1.0)}, 2, 1),
      resources, error));
  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 1), Observation("velocity", 2, 1)},
                       {Command("motor", 0, -1.0, 1.0)}, 3, 1),
      resources, error));
}

TEST(LayoutTest, RejectsDuplicateNamesAndInvalidCommandRanges) {
  std::string error;
  std::vector<std::unique_ptr<ResourceInterface>> resources;
  resources.push_back(MakeResource(Component("joint", "joint"),
                                   {{"sensor_position", "position"},
                                    {"sensor_velocity", "velocity"},
                                    {"actuator_velocity", "motor"}},
                                   error));
  ASSERT_NE(resources.front(), nullptr) << error;

  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 1), Observation("position", 1, 1)},
                       {Command("motor", 0, -1.0, 1.0)}, 2, 1),
      resources, error));
  EXPECT_NE(error.find("duplicates"), std::string::npos);

  ResourceInfo invalid_range = Command("motor", 0, 2.0, -2.0);
  EXPECT_FALSE(ValidateLayout(
      ControllerLayout({Observation("position", 0, 1), Observation("velocity", 1, 1)},
                       {invalid_range}, 2, 1),
      resources, error));
  EXPECT_NE(error.find("invalid range"), std::string::npos);
}

}  // namespace
