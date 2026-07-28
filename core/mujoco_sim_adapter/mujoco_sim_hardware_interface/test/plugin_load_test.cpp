#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

TEST(PluginLoadTest, DiscoversAndConstructsMujocoSimSystem) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  const std::string plugin = "mujoco_sim_hardware_interface/MujocoSimSystem";
  EXPECT_TRUE(loader.isClassAvailable(plugin));
  auto instance = loader.createSharedInstance(plugin);
  EXPECT_NE(instance, nullptr);
}
