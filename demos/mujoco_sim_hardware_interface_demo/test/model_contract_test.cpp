#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>
#include <hardware_interface/component_parser.hpp>

TEST(ModelContractTest, UrdfUsesAutomaticResourceMappings) {
  std::ifstream input(DEMO_URDF);
  ASSERT_TRUE(input.is_open());
  const std::string urdf((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const auto hardware = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(hardware.size(), 1U);
  const auto& info = hardware.front();
  EXPECT_EQ(info.type, "system");
  EXPECT_EQ(info.hardware_plugin_name, "mujoco_sim_hardware_interface/MujocoSimSystem");
  ASSERT_EQ(info.joints.size(), 2U);
  ASSERT_EQ(info.sensors.size(), 2U);
  for (const auto& joint : info.joints) {
    EXPECT_TRUE(joint.state_interfaces.empty());
    EXPECT_TRUE(joint.command_interfaces.empty());
    EXPECT_TRUE(joint.parameters.contains("actuator_velocity"));
    EXPECT_TRUE(joint.parameters.contains("sensor_position"));
    EXPECT_TRUE(joint.parameters.contains("sensor_velocity"));
  }
  EXPECT_EQ(info.sensors[0].parameters.at("sensor_type"), "imu");
  EXPECT_EQ(info.sensors[0].parameters.at("accelerometer"), "imu_acceleration");
  EXPECT_EQ(info.sensors[1].parameters.at("sensor_type"), "other");
  EXPECT_EQ(info.sensors[1].parameters.at("sensor_dim_names"), "x y z");
}
