#include <mujoco_sim/server/server_config.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include <gtest/gtest.h>

namespace mujoco_sim::server {
namespace {

class TemporaryConfig {
 public:
  explicit TemporaryConfig(std::string_view contents) {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("mujoco_sim_server_config_" + std::to_string(suffix) + ".yaml");
    std::ofstream stream(path_);
    stream << contents;
  }

  ~TemporaryConfig() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

ServerConfig ValidConfig() {
  ServerConfig config;
  config.model_path = "model.xml";
  config.headless = true;
  return config;
}

TEST(ServerConfigTest, LoadsPeriodCadenceAndCompleteControllerOverride) {
  TemporaryConfig file(R"(
model_path: model.xml
headless: true
controller_state_period_s: 0.01
controller_state_max_rate_hz: 100.0
controller_overrides:
  - adapter_id: arm
    command_timeout_ms: 800
    command_keepalive_ms: 80
    offline_session_expire_ms: 3000
    state_every_n_steps: 4
    state_period_s: 0.02
    startup_ctrl_policy: hold_last
    offline_ctrl_policy: zero
    error_ctrl_policy: hold_last
    release_ctrl_policy: hold_last
)");
  auto config = ValidConfig();
  std::string error;

  ASSERT_TRUE(LoadConfigFile(file.path().string(), &config, &error)) << error;
  ASSERT_TRUE(ValidateServerConfig(config, &error)) << error;
  EXPECT_DOUBLE_EQ(config.controller_state_period_s, 0.01);
  EXPECT_DOUBLE_EQ(config.controller_state_max_rate_hz, 100.0);
  ASSERT_EQ(config.controller_overrides.size(), 1U);
  const auto& override = config.controller_overrides.front();
  EXPECT_EQ(override.adapter_id, "arm");
  EXPECT_EQ(override.command_timeout_ms, 800U);
  EXPECT_EQ(override.command_keepalive_ms, 80U);
  EXPECT_EQ(override.offline_session_expire_ms, 3000U);
  EXPECT_EQ(override.state_every_n_steps, 4U);
  EXPECT_DOUBLE_EQ(override.state_period_s, 0.02);
  EXPECT_EQ(override.startup_ctrl_policy, "hold_last");
  EXPECT_EQ(override.offline_ctrl_policy, "zero");
  EXPECT_EQ(override.error_ctrl_policy, "hold_last");
  EXPECT_EQ(override.release_ctrl_policy, "hold_last");
}

TEST(ServerConfigTest, RejectsDuplicateControllerOverrides) {
  auto config = ValidConfig();
  config.controller_overrides = {
      {.adapter_id = "arm"},
      {.adapter_id = "arm"},
  };
  std::string error;
  EXPECT_FALSE(ValidateServerConfig(config, &error));
  EXPECT_NE(error.find("override"), std::string::npos);
}

TEST(ServerConfigTest, RejectsInvalidControllerTimingAndRate) {
  std::string error;
  auto config = ValidConfig();
  config.command_keepalive_ms = config.command_timeout_ms;
  EXPECT_FALSE(ValidateServerConfig(config, &error));

  config = ValidConfig();
  config.controller_state_max_rate_hz = 0.0;
  EXPECT_FALSE(ValidateServerConfig(config, &error));

  config = ValidConfig();
  config.controller_overrides.push_back(
      {.adapter_id = "arm", .command_timeout_ms = 50, .command_keepalive_ms = 50});
  EXPECT_FALSE(ValidateServerConfig(config, &error));
}

class ObsoleteControllerKeyTest : public testing::TestWithParam<const char*> {};

TEST_P(ObsoleteControllerKeyTest, IsRejectedBySchema) {
  TemporaryConfig file(std::string("model_path: model.xml\nheadless: true\n") + GetParam() +
                       ": 1\n");
  auto config = ValidConfig();
  std::string error;
  EXPECT_FALSE(LoadConfigFile(file.path().string(), &config, &error));
  EXPECT_FALSE(error.empty());
}

INSTANTIATE_TEST_SUITE_P(V3Keys, ObsoleteControllerKeyTest,
                         testing::Values("tick_timeout_ms", "controller_heartbeat_period_ms",
                                         "tick_retry_initial_delay_ms", "required_for_barrier",
                                         "barrier_replacement_policy"));

}  // namespace
}  // namespace mujoco_sim::server
