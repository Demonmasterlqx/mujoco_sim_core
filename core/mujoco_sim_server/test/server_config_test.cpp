#include <mujoco_sim/server/server_config.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

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

TEST(ServerConfigTest, LoadsViewerSyncPeriodAtInclusiveLimits) {
  TemporaryConfig minimum_file(R"(
model_path: model.xml
headless: true
viewer_sync_period_ms: 1
)");
  auto config = ValidConfig();
  std::string error;
  ASSERT_TRUE(LoadConfigFile(minimum_file.path().string(), &config, &error)) << error;
  EXPECT_EQ(config.viewer_sync_period_ms, 1U);

  TemporaryConfig maximum_file(R"(
model_path: model.xml
headless: true
viewer_sync_period_ms: 4294967295
)");
  config = ValidConfig();
  ASSERT_TRUE(LoadConfigFile(maximum_file.path().string(), &config, &error)) << error;
  EXPECT_EQ(config.viewer_sync_period_ms, std::numeric_limits<std::uint32_t>::max());
}

TEST(ServerConfigTest, ViewerSyncPeriodDefaultsToSixteenMilliseconds) {
  const ServerConfig config;
  EXPECT_EQ(config.viewer_sync_period_ms, 16U);
}

TEST(ServerConfigTest, RejectsZeroAndUnknownViewerSyncFields) {
  TemporaryConfig zero_file(R"(
model_path: model.xml
headless: true
viewer_sync_period_ms: 0
)");
  auto config = ValidConfig();
  std::string error;
  EXPECT_FALSE(LoadConfigFile(zero_file.path().string(), &config, &error));
  EXPECT_NE(error.find("viewer_sync_period_ms"), std::string::npos);

  TemporaryConfig unknown_file(R"(
model_path: model.xml
headless: true
viewer_sync_period: 16
)");
  config = ValidConfig();
  EXPECT_FALSE(LoadConfigFile(unknown_file.path().string(), &config, &error));
  EXPECT_NE(error.find("viewer_sync_period"), std::string::npos);

  config = ValidConfig();
  config.viewer_sync_period_ms = 0;
  EXPECT_FALSE(ValidateServerConfig(config, &error));
  EXPECT_NE(error.find("viewer_sync_period_ms"), std::string::npos);
}

TEST(ServerConfigTest, ViewerSyncCliOverridesYamlInHeadlessMode) {
  TemporaryConfig file(R"(
model_path: model.xml
headless: false
viewer_sync_period_ms: 25
)");
  std::vector<std::string> arguments = {"mujoco-sim-server",       "--config",
                                        file.path().string(),      "--headless",
                                        "--viewer-sync-period-ms", "7"};
  std::vector<char*> argv;
  argv.reserve(arguments.size());
  for (auto& argument : arguments) {
    argv.push_back(argument.data());
  }
  ServerConfig config;
  std::string error;
  ASSERT_TRUE(ParseCommandLine(static_cast<int>(argv.size()), argv.data(), &config, &error))
      << error;
  EXPECT_TRUE(config.headless);
  EXPECT_EQ(config.viewer_sync_period_ms, 7U);
}

TEST(ServerConfigTest, RejectsZeroViewerSyncPeriodFromCommandLine) {
  TemporaryConfig file(R"(
model_path: model.xml
headless: true
)");
  std::vector<std::string> arguments = {"mujoco-sim-server", "--config", file.path().string(),
                                        "--viewer-sync-period-ms", "0"};
  std::vector<char*> argv;
  argv.reserve(arguments.size());
  for (auto& argument : arguments) {
    argv.push_back(argument.data());
  }
  ServerConfig config;
  std::string error;
  EXPECT_FALSE(ParseCommandLine(static_cast<int>(argv.size()), argv.data(), &config, &error));
  EXPECT_NE(error.find("viewer_sync_period_ms"), std::string::npos);
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
