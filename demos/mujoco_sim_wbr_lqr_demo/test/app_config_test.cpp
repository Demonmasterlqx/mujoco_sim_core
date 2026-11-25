#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "app_config.hpp"

#include <mujoco_sim/wbr_lqr_demo/controller.hpp>

namespace demo = mujoco_sim::wbr_lqr_demo;

TEST(AppConfigTest, LoadsShippedGeneratedConfig) {
  demo::generated::WbrConfig generated;
  std::string error;
  ASSERT_TRUE(demo::generated::Load(std::filesystem::path(WBR_LQR_CONFIG_DIR) / "wbr_lqr.yaml",
                                    &generated, &error))
      << error;

  demo::AppConfig config;
  ASSERT_TRUE(demo::BuildAppConfig(generated, &config, &error)) << error;
  EXPECT_EQ(config.adapter.server_name, "wbr");
  EXPECT_EQ(config.adapter.max_ticks, 0u);
  EXPECT_EQ(config.adapter.interfaces.motors[0].semantic_name, "left_joint_1");
  EXPECT_EQ(config.adapter.interfaces.motors[5].effort_command, "rwheel_actuator");
  EXPECT_EQ(config.input.mode, demo::InputMode::kScript);
  EXPECT_EQ(config.input.cin_hold_ticks, 200U);
  EXPECT_TRUE(config.input_script.InputAt(1).space);
  EXPECT_FALSE(config.input_script.InputAt(2000).e);
  EXPECT_FALSE(config.input_script.InputAt(4000).q);
  EXPECT_EQ(config.pipeline.chassis.leg_pid.len.kp, 6000.0F);
  EXPECT_FLOAT_EQ(config.pipeline.chassis.twheel_max, 5.2F);
}

TEST(AppConfigTest, SelectsCinModeThroughCliPrecedence) {
  demo::generated::WbrConfig generated;
  std::string error;
  const char* arguments[] = {"wbr-adapter", "--input-mode",      "cin", "--cin-hold-ticks",
                             "17",          "--log-every-ticks", "0"};
  ASSERT_TRUE(
      demo::generated::Parse(static_cast<int>(std::size(arguments)), arguments, &generated, &error))
      << error;
  demo::AppConfig config;
  ASSERT_TRUE(demo::BuildAppConfig(generated, &config, &error)) << error;
  EXPECT_EQ(config.input.mode, demo::InputMode::kCin);
  EXPECT_EQ(config.input.cin_hold_ticks, 17U);
  EXPECT_EQ(config.log_every_ticks, 0U);
}

TEST(AppConfigTest, RejectsInvalidInputModeAndUnknownCliField) {
  demo::generated::WbrConfig generated;
  std::string error;
  const char* invalid_mode[] = {"wbr-adapter", "--input-mode", "evdev"};
  EXPECT_FALSE(demo::generated::Parse(static_cast<int>(std::size(invalid_mode)), invalid_mode,
                                      &generated, &error));
  EXPECT_NE(error.find("input"), std::string::npos);

  const char* unknown_field[] = {"wbr-adapter", "--unknown-keyboard-field", "value"};
  EXPECT_FALSE(demo::generated::Parse(static_cast<int>(std::size(unknown_field)), unknown_field,
                                      &generated, &error));
  EXPECT_FALSE(error.empty());

  const char* zero_hold[] = {"wbr-adapter", "--cin-hold-ticks", "0"};
  EXPECT_FALSE(demo::generated::Parse(static_cast<int>(std::size(zero_hold)), zero_hold, &generated,
                                      &error));
  EXPECT_NE(error.find("hold_ticks"), std::string::npos);
}

TEST(AppConfigTest, RejectsUnknownYamlInputField) {
  const auto path = std::filesystem::temp_directory_path() / "wbr_lqr_unknown_input.yaml";
  {
    std::ofstream output(path);
    ASSERT_TRUE(output.is_open());
    output << "input:\n  unknown_keyboard_field: true\n";
  }

  demo::generated::WbrConfig generated;
  std::string error;
  const std::string path_string = path.string();
  const char* arguments[] = {"wbr-adapter", "--config", path_string.c_str()};
  EXPECT_FALSE(demo::generated::Parse(static_cast<int>(std::size(arguments)), arguments, &generated,
                                      &error));
  EXPECT_FALSE(error.empty());
  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

TEST(AppConfigTest, ResourceNamesRemainConfigurableAcrossReordering) {
  demo::generated::WbrConfig generated;
  std::reverse(generated.interfaces.motors.begin(), generated.interfaces.motors.end());
  generated.interfaces.imu.quaternion = "custom_quaternion";
  for (auto& motor : generated.interfaces.motors) {
    motor.position_state = "position_" + motor.semantic_name;
    motor.velocity_state = "velocity_" + motor.semantic_name;
    motor.effort_state = "effort_" + motor.semantic_name;
    motor.effort_command = "command_" + motor.semantic_name;
  }

  demo::AppConfig config;
  std::string error;
  ASSERT_TRUE(demo::BuildAppConfig(generated, &config, &error)) << error;
  const auto options = demo::MakeControllerOptions(config.adapter);
  EXPECT_EQ(options.observation_names.front(), "custom_quaternion");
  EXPECT_NE(std::find(options.observation_names.begin(), options.observation_names.end(),
                      "position_left_joint_1"),
            options.observation_names.end());
  EXPECT_EQ(options.command_names.front(), "command_left_joint_1");
  EXPECT_EQ(options.command_names.back(), "command_right_wheel");
}

TEST(AppConfigTest, RejectsSemanticAndSpaceScriptErrorsTransactionally) {
  demo::generated::WbrConfig generated;
  generated.interfaces.motors[1].semantic_name = generated.interfaces.motors[0].semantic_name;
  demo::AppConfig config;
  config.adapter.server_name = "unchanged";
  std::string error;
  EXPECT_FALSE(demo::BuildAppConfig(generated, &config, &error));
  EXPECT_NE(error.find("duplicate motor semantic_name"), std::string::npos);
  EXPECT_EQ(config.adapter.server_name, "unchanged");

  generated = {};
  generated.input_script = {{0, 1, false, false, false, false, false, false, false, true}};
  EXPECT_FALSE(demo::BuildAppConfig(generated, &config, &error));
  EXPECT_NE(error.find("after tick zero"), std::string::npos);
  EXPECT_EQ(config.adapter.server_name, "unchanged");
}
