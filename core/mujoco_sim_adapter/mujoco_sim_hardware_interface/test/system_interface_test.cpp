#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <mujoco_sim_hardware_interface/mujoco_sim_system.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareComponentParams;
using hardware_interface::HardwareInfo;
using mujoco_sim::hardware::MujocoSimSystem;

class RosEnvironment final : public testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

[[maybe_unused]] const auto* kRosEnvironment =
    testing::AddGlobalTestEnvironment(new RosEnvironment);

ComponentInfo Joint(std::string name, std::string command, std::string position,
                    std::string velocity = {}) {
  ComponentInfo joint;
  joint.name = std::move(name);
  joint.type = "joint";
  if (!command.empty()) joint.parameters["actuator_velocity"] = std::move(command);
  if (!position.empty()) joint.parameters["sensor_position"] = std::move(position);
  if (!velocity.empty()) joint.parameters["sensor_velocity"] = std::move(velocity);
  return joint;
}

ComponentInfo GenericSensor(std::string component, std::string resource,
                            std::string dimension_names) {
  ComponentInfo sensor;
  sensor.name = std::move(component);
  sensor.type = "sensor";
  sensor.parameters = {{"sensor_type", "other"},
                       {"sensor_name", std::move(resource)},
                       {"sensor_dim", "1"},
                       {"sensor_dim_names", std::move(dimension_names)}};
  return sensor;
}

HardwareInfo BaseInfo() {
  HardwareInfo info;
  info.name = "test_hardware";
  info.type = "system";
  info.rw_rate = 100;
  info.is_async = false;
  info.thread_priority = 50;
  info.hardware_plugin_name = "mujoco_sim_hardware_interface/MujocoSimSystem";
  info.hardware_parameters = {{"adapter_id", "test_controller"}};
  return info;
}

HardwareComponentParams Params(HardwareInfo info) {
  HardwareComponentParams params;
  params.hardware_info = std::move(info);
  params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  params.logger = rclcpp::get_logger("mujoco_sim_hardware_interface_test");
  return params;
}

TEST(SystemInterfaceTest, AutoExportsMappedInterfacesAndUsesValidInitialValues) {
  auto info = BaseInfo();
  info.joints.push_back(Joint("wheel", "wheel_motor", "wheel_pos", "wheel_vel"));
  MujocoSimSystem system;
  ASSERT_EQ(system.init(Params(std::move(info))), CallbackReturn::SUCCESS);

  const auto states = system.on_export_state_interfaces();
  const auto commands = system.on_export_command_interfaces();
  ASSERT_EQ(states.size(), 2U);
  ASSERT_EQ(commands.size(), 1U);
  EXPECT_EQ(states[0]->get_name(), "wheel/position");
  EXPECT_EQ(states[1]->get_name(), "wheel/velocity");
  EXPECT_EQ(commands[0]->get_name(), "wheel/velocity");
  EXPECT_DOUBLE_EQ(states[0]->get_optional<double>().value(), 0.0);
  EXPECT_DOUBLE_EQ(commands[0]->get_optional<double>().value(), 0.0);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(std::isnan(states[0]->get_optional<double>().value()));
  EXPECT_TRUE(std::isnan(states[1]->get_optional<double>().value()));
  EXPECT_DOUBLE_EQ(commands[0]->get_optional<double>().value(), 0.0);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(SystemInterfaceTest, LocalDebugParameterOverridesGlobalValue) {
  auto info = BaseInfo();
  info.hardware_parameters["debug"] = "true";
  auto joint = Joint("wheel", "wheel_motor", "wheel_pos");
  joint.parameters["debug"] = "false";
  info.joints.push_back(std::move(joint));
  MujocoSimSystem system;
  ASSERT_EQ(system.init(Params(std::move(info))), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_export_state_interfaces().size(), 1U);
  ASSERT_EQ(system.on_export_command_interfaces().size(), 1U);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(SystemInterfaceTest, RejectsComponentOverridesOfSystemOnlyParameters) {
  for (const auto* parameter :
       {"server_id", "adapter_id", "server_time_out_ms", "publish_sim_time"}) {
    auto info = BaseInfo();
    auto joint = Joint("wheel", "wheel_motor", "wheel_pos");
    joint.parameters[parameter] = "invalid-even-when-the-value-would-otherwise-be-valid";
    info.joints.push_back(std::move(joint));
    MujocoSimSystem system;
    EXPECT_EQ(system.init(Params(std::move(info))), CallbackReturn::ERROR) << parameter;
  }
}

TEST(SystemInterfaceTest, RejectsExplicitRosInterfaces) {
  auto info = BaseInfo();
  auto joint = Joint("wheel", "wheel_motor", "wheel_pos");
  hardware_interface::InterfaceInfo explicit_state;
  explicit_state.name = "position";
  joint.state_interfaces.push_back(std::move(explicit_state));
  info.joints.push_back(std::move(joint));
  MujocoSimSystem system;
  EXPECT_EQ(system.init(Params(std::move(info))), CallbackReturn::ERROR);
}

TEST(SystemInterfaceTest, RejectsDuplicateObservationAndCommandMappings) {
  auto observation_info = BaseInfo();
  observation_info.joints.push_back(Joint("left", "left_motor", "shared_position"));
  observation_info.joints.push_back(Joint("right", "right_motor", "shared_position"));
  MujocoSimSystem observation_system;
  EXPECT_EQ(observation_system.init(Params(std::move(observation_info))), CallbackReturn::ERROR);

  auto command_info = BaseInfo();
  command_info.joints.push_back(Joint("left", "shared_motor", "left_position"));
  command_info.joints.push_back(Joint("right", "shared_motor", "right_position"));
  MujocoSimSystem command_system;
  EXPECT_EQ(command_system.init(Params(std::move(command_info))), CallbackReturn::ERROR);
}

TEST(SystemInterfaceTest, RejectsOneObservationMappedToTwoImuGroups) {
  auto info = BaseInfo();
  info.joints.push_back(Joint("wheel", "wheel_motor", "wheel_position"));
  ComponentInfo imu;
  imu.name = "imu";
  imu.type = "sensor";
  imu.parameters = {{"sensor_type", "imu"},
                    {"accelerometer", "shared_vector"},
                    {"gyro", "shared_vector"},
                    {"father_link", "world"},
                    {"target_link", "imu_link"}};
  info.sensors.push_back(std::move(imu));
  MujocoSimSystem system;
  EXPECT_EQ(system.init(Params(std::move(info))), CallbackReturn::ERROR);
}

TEST(SystemInterfaceTest, RejectsDuplicateGroupedDebugTopics) {
  auto info = BaseInfo();
  info.hardware_parameters["debug"] = "true";
  info.joints.push_back(Joint("wheel", "wheel_motor", "wheel_position"));
  info.sensors.push_back(GenericSensor("grouped", "sensor_a", "a"));
  info.sensors.push_back(GenericSensor("grouped", "sensor_b", "b"));
  MujocoSimSystem system;
  EXPECT_EQ(system.init(Params(std::move(info))), CallbackReturn::ERROR);
}

TEST(SystemInterfaceTest, RejectsPureSensorAndGpioConfigurations) {
  auto sensor_only = BaseInfo();
  sensor_only.sensors.push_back(GenericSensor("grouped", "sensor_a", "a"));
  MujocoSimSystem sensor_system;
  EXPECT_EQ(sensor_system.init(Params(std::move(sensor_only))), CallbackReturn::ERROR);

  auto gpio_info = BaseInfo();
  gpio_info.joints.push_back(Joint("wheel", "wheel_motor", "wheel_position"));
  ComponentInfo gpio;
  gpio.name = "gpio";
  gpio.type = "gpio";
  gpio_info.gpios.push_back(std::move(gpio));
  MujocoSimSystem gpio_system;
  EXPECT_EQ(gpio_system.init(Params(std::move(gpio_info))), CallbackReturn::ERROR);
}

TEST(ResourceManagerTest, DiscoversAllGeneratedJazzyInterfaces) {
  const std::string urdf = R"(
<robot name="resource_manager_test">
  <link name="base_link"/>
  <link name="wheel_link"/>
  <joint name="wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  <ros2_control name="mujoco_test" type="system">
    <hardware>
      <plugin>mujoco_sim_hardware_interface/MujocoSimSystem</plugin>
      <param name="adapter_id">resource_manager_test</param>
    </hardware>
    <joint name="wheel">
      <param name="actuator_velocity">wheel_motor</param>
      <param name="sensor_position">wheel_position</param>
      <param name="sensor_velocity">wheel_velocity</param>
    </joint>
    <sensor name="force_sensor">
      <param name="sensor_type">other</param>
      <param name="sensor_name">contact_force</param>
      <param name="sensor_dim">3</param>
      <param name="sensor_dim_names">x y z</param>
    </sensor>
    <sensor name="imu_sensor">
      <param name="sensor_type">imu</param>
      <param name="accelerometer">acceleration</param>
      <param name="gyro">angular_velocity</param>
      <param name="father_link">world</param>
      <param name="target_link">imu_link</param>
    </sensor>
  </ros2_control>
</robot>)";
  hardware_interface::ResourceManager manager(
      urdf, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME),
      rclcpp::get_logger("mujoco_sim_resource_manager_test"), false);

  auto state_keys = manager.state_interface_keys();
  auto command_keys = manager.command_interface_keys();
  std::sort(state_keys.begin(), state_keys.end());
  std::sort(command_keys.begin(), command_keys.end());
  const std::vector<std::string> expected_states = {"force_sensor/x",
                                                    "force_sensor/y",
                                                    "force_sensor/z",
                                                    "imu_sensor/angular_velocity.x",
                                                    "imu_sensor/angular_velocity.y",
                                                    "imu_sensor/angular_velocity.z",
                                                    "imu_sensor/linear_acceleration.x",
                                                    "imu_sensor/linear_acceleration.y",
                                                    "imu_sensor/linear_acceleration.z",
                                                    "wheel/position",
                                                    "wheel/velocity"};
  EXPECT_EQ(state_keys, expected_states);
  EXPECT_EQ(command_keys, (std::vector<std::string>{"wheel/velocity"}));
}

}  // namespace
