#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/server/server_config.hpp>

namespace {

class ScopedConfigFile {
 public:
  ScopedConfigFile(std::string name, const std::string& contents)
      : path_(std::filesystem::temp_directory_path() / std::move(name)) {
    std::ofstream output(path_);
    output << contents;
  }

  ~ScopedConfigFile() {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  const std::string path() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

bool ParseArgs(std::vector<std::string> arguments,
               mujoco_sim::server::ServerConfig* config,
               std::string* error_message) {
  std::vector<char*> argv;
  argv.reserve(arguments.size());
  for (auto& argument : arguments) {
    argv.push_back(argument.data());
  }
  return mujoco_sim::server::ParseCommandLine(
      static_cast<int>(argv.size()), argv.data(), config, error_message);
}

void ExpectConfigEquals(const mujoco_sim::server::ServerConfig& actual,
                        const mujoco_sim::server::ServerConfig& expected) {
  EXPECT_EQ(actual.server_name, expected.server_name);
  EXPECT_EQ(actual.model_path, expected.model_path);
  EXPECT_EQ(actual.initial_keyframe, expected.initial_keyframe);
  EXPECT_EQ(actual.max_ticks, expected.max_ticks);
  EXPECT_EQ(actual.tick_timeout_ms, expected.tick_timeout_ms);
  EXPECT_EQ(actual.startup_timeout_ms, expected.startup_timeout_ms);
  EXPECT_EQ(actual.status_period_ms, expected.status_period_ms);
  EXPECT_EQ(actual.headless, expected.headless);
}

TEST(ServerConfigTest, ParsesCommandLine) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  ASSERT_TRUE(ParseArgs({"mujoco-sim-server",
                         "--model",
                         "robot.xml",
                         "--server-name",
                         "sim",
                         "--initial-keyframe",
                         "home",
                         "--max-ticks",
                         "10",
                         "--tick-timeout-ms",
                         "50",
                         "--startup-timeout-ms",
                         "250",
                         "--headless"},
                        &config, &error))
      << error;
  EXPECT_EQ(config.model_path, "robot.xml");
  EXPECT_EQ(config.server_name, "sim");
  EXPECT_EQ(config.initial_keyframe, "home");
  EXPECT_EQ(config.max_ticks, 10);
  EXPECT_EQ(config.tick_timeout_ms, 50);
  EXPECT_EQ(config.startup_timeout_ms, 250);
  EXPECT_EQ(config.status_period_ms, 100);
  EXPECT_TRUE(config.headless);
}

TEST(ServerConfigTest, KeepsDefaultsForUnspecifiedCommandLineOptions) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  ASSERT_TRUE(ParseArgs({"mujoco-sim-server", "--model", "robot.xml"},
                        &config, &error))
      << error;
  EXPECT_EQ(config.server_name, "default");
  EXPECT_EQ(config.max_ticks, 0);
  EXPECT_EQ(config.tick_timeout_ms, 1000);
  EXPECT_EQ(config.startup_timeout_ms, 5000);
  EXPECT_EQ(config.status_period_ms, 100);
  EXPECT_FALSE(config.headless);
}

TEST(ServerConfigTest, PreservesLegacyAggregateInitializationOrder) {
  const mujoco_sim::server::ServerConfig config{
      "legacy", "robot.xml", 7, 8, 9, 10, true};
  EXPECT_EQ(config.server_name, "legacy");
  EXPECT_EQ(config.model_path, "robot.xml");
  EXPECT_EQ(config.max_ticks, 7U);
  EXPECT_EQ(config.tick_timeout_ms, 8U);
  EXPECT_EQ(config.startup_timeout_ms, 9U);
  EXPECT_EQ(config.status_period_ms, 10U);
  EXPECT_TRUE(config.headless);
  EXPECT_TRUE(config.initial_keyframe.empty());
}

TEST(ServerConfigTest, ReturnsGeneratedHelp) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  EXPECT_FALSE(ParseArgs({"mujoco-sim-server", "--help"}, &config, &error));
  EXPECT_EQ(error, mujoco_sim::server::Usage());
  EXPECT_NE(error.find("--startup-timeout-ms"), std::string::npos);
  EXPECT_NE(error.find("MuJoCo MJCF model path"), std::string::npos);
  EXPECT_NE(error.find("--initial-keyframe"), std::string::npos);
  EXPECT_NE(error.find("--headless"), std::string::npos);
  EXPECT_NE(error.find("--gui"), std::string::npos);
}

TEST(ServerConfigTest, RejectsInvalidCommandLine) {
  const std::vector<std::vector<std::string>> invalid_arguments = {
      {"mujoco-sim-server", "--model", "robot.xml", "--unknown"},
      {"mujoco-sim-server", "--model"},
      {"mujoco-sim-server", "--model", "robot.xml", "--max-ticks", "bad"},
      {"mujoco-sim-server", "--model", "robot.xml", "--max-ticks", "-1"},
      {"mujoco-sim-server", "--model", "robot.xml", "--max-ticks",
       "18446744073709551616"},
      {"mujoco-sim-server", "--model", "robot.xml", "--tick-timeout-ms",
       "4294967296"},
      {"mujoco-sim-server", "--model", "one.xml", "--model", "two.xml"},
      {"mujoco-sim-server", "--model", "robot.xml", "--headless", "--gui"},
  };

  for (const auto& arguments : invalid_arguments) {
    SCOPED_TRACE(testing::PrintToString(arguments));
    mujoco_sim::server::ServerConfig config;
    std::string error;
    EXPECT_FALSE(ParseArgs(arguments, &config, &error));
    EXPECT_FALSE(error.empty());
  }
}

TEST(ServerConfigTest, RequiresModelAndNonEmptyServerName) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  EXPECT_FALSE(ParseArgs({"mujoco-sim-server"}, &config, &error));
  EXPECT_EQ(error, "model path is required");

  config.server_name.clear();
  EXPECT_FALSE(ParseArgs({"mujoco-sim-server", "--model", "robot.xml"},
                         &config, &error));
  EXPECT_EQ(error, "server name is required");
}

TEST(ServerConfigTest, EmptyYamlModelPathRemainsEmptyAndRequired) {
  ScopedConfigFile file("mujoco_sim_server_empty_model.yaml",
                        "model_path: \"\"\n");
  mujoco_sim::server::ServerConfig config;
  config.model_path = "unchanged.xml";
  std::string error;

  EXPECT_FALSE(ParseArgs({"mujoco-sim-server", "--config", file.path()},
                         &config, &error));
  EXPECT_EQ(error, "model path is required");
  EXPECT_EQ(config.model_path, "unchanged.xml");
}

TEST(ServerConfigTest, LoadsStrictYamlWithAllFields) {
  ScopedConfigFile file(
      "mujoco_sim_server_all_fields.yaml",
      "server_name: \"sim:#1\" # quoted colon and hash\n"
      "model_path: robot.xml\n"
      "initial_keyframe: home\n"
      "max_ticks: 18446744073709551615\n"
      "tick_timeout_ms: 4294967295\n"
      "startup_timeout_ms: 6000\n"
      "status_period_ms: 25\n"
      "headless: true\n");

  mujoco_sim::server::ServerConfig config;
  std::string error;
  ASSERT_TRUE(
      mujoco_sim::server::LoadConfigFile(file.path(), &config, &error))
      << error;
  EXPECT_EQ(config.server_name, "sim:#1");
  EXPECT_EQ(config.model_path,
            (std::filesystem::temp_directory_path() / "robot.xml").string());
  EXPECT_EQ(config.initial_keyframe, "home");
  EXPECT_EQ(config.max_ticks, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(config.tick_timeout_ms,
            std::numeric_limits<std::uint32_t>::max());
  EXPECT_EQ(config.startup_timeout_ms, 6000);
  EXPECT_EQ(config.status_period_ms, 25);
  EXPECT_TRUE(config.headless);
}

TEST(ServerConfigTest, AcceptsEmptyYamlWithoutChangingConfig) {
  ScopedConfigFile file("mujoco_sim_server_empty.yaml", "");
  mujoco_sim::server::ServerConfig config;
  config.server_name = "unchanged";
  config.model_path = "robot.xml";
  const auto expected = config;
  std::string error;

  ASSERT_TRUE(
      mujoco_sim::server::LoadConfigFile(file.path(), &config, &error))
      << error;
  ExpectConfigEquals(config, expected);
}

TEST(ServerConfigTest, RejectsInvalidYamlDocuments) {
  struct InvalidYaml {
    std::string filename;
    std::string contents;
    std::string expected_error;
  };
  const std::vector<InvalidYaml> cases = {
      {"mujoco_sim_server_syntax.yaml", "server_name: [\n", "end of sequence"},
      {"mujoco_sim_server_root.yaml", "- server_name\n- sim\n",
       "root must be a mapping"},
      {"mujoco_sim_server_documents.yaml",
       "server_name: one\n---\nserver_name: two\n",
       "multiple YAML documents"},
  };

  for (const auto& test_case : cases) {
    SCOPED_TRACE(test_case.filename);
    ScopedConfigFile file(test_case.filename, test_case.contents);
    mujoco_sim::server::ServerConfig config;
    std::string error;
    EXPECT_FALSE(
        mujoco_sim::server::LoadConfigFile(file.path(), &config, &error));
    EXPECT_NE(error.find(file.path()), std::string::npos);
    EXPECT_NE(error.find(test_case.expected_error), std::string::npos)
        << error;
  }
}

TEST(ServerConfigTest, RejectsUnknownDuplicateAndWrongTypeYamlFields) {
  struct InvalidYaml {
    std::string filename;
    std::string contents;
    std::string expected_error;
  };
  const std::vector<InvalidYaml> cases = {
      {"mujoco_sim_server_unknown.yaml", "model_path: robot.xml\nmodle: x\n",
       "unknown field: modle"},
      {"mujoco_sim_server_duplicate.yaml",
       "model_path: one.xml\nmodel_path: two.xml\n",
       "duplicate field: model_path"},
      {"mujoco_sim_server_nested.yaml", "model_path:\n  nested: robot.xml\n",
       "model_path must be a string"},
      {"mujoco_sim_server_null.yaml", "server_name: null\n",
       "server_name must be a string"},
      {"mujoco_sim_server_numeric_string.yaml", "server_name: 123\n",
       "server_name must be a string"},
      {"mujoco_sim_server_boolean_string.yaml", "server_name: true\n",
       "server_name must be a string"},
      {"mujoco_sim_server_tagged_string.yaml", "model_path: !!int 123\n",
       "model_path must be a string"},
      {"mujoco_sim_server_quoted_number.yaml", "max_ticks: \"12\"\n",
       "max_ticks must be an unsigned 64-bit integer"},
      {"mujoco_sim_server_negative.yaml", "tick_timeout_ms: -1\n",
       "tick_timeout_ms must be an unsigned 32-bit integer in range"},
      {"mujoco_sim_server_overflow.yaml", "status_period_ms: 4294967296\n",
       "status_period_ms must be an unsigned 32-bit integer in range"},
      {"mujoco_sim_server_quoted_bool.yaml", "headless: \"true\"\n",
       "headless must be a boolean"},
      {"mujoco_sim_server_invalid_bool.yaml", "headless: enabled\n",
       "headless must be a boolean"},
  };

  for (const auto& test_case : cases) {
    SCOPED_TRACE(test_case.filename);
    ScopedConfigFile file(test_case.filename, test_case.contents);
    mujoco_sim::server::ServerConfig config;
    std::string error;
    EXPECT_FALSE(
        mujoco_sim::server::LoadConfigFile(file.path(), &config, &error));
    EXPECT_NE(error.find(test_case.expected_error), std::string::npos)
        << error;
  }
}

TEST(ServerConfigTest, CommandLineOverridesYamlRegardlessOfArgumentOrder) {
  ScopedConfigFile file(
      "mujoco_sim_server_precedence.yaml",
      "server_name: yaml\n"
      "model_path: yaml.xml\n"
      "max_ticks: 10\n"
      "tick_timeout_ms: 20\n"
      "startup_timeout_ms: 30\n"
      "status_period_ms: 40\n"
      "headless: false\n");

  const std::vector<std::vector<std::string>> argument_orders = {
      {"mujoco-sim-server", "--config", file.path(), "--model", "cli.xml",
       "--server-name", "cli", "--max-ticks", "11", "--tick-timeout-ms",
       "21", "--startup-timeout-ms", "31"},
      {"mujoco-sim-server", "--model", "cli.xml", "--server-name", "cli",
       "--max-ticks", "11", "--tick-timeout-ms", "21",
       "--startup-timeout-ms", "31", "--config", file.path(),
       "--headless"},
  };

  for (const auto& arguments : argument_orders) {
    mujoco_sim::server::ServerConfig config;
    std::string error;
    ASSERT_TRUE(ParseArgs(arguments, &config, &error)) << error;
    EXPECT_EQ(config.model_path, "cli.xml");
    EXPECT_EQ(config.server_name, "cli");
    EXPECT_EQ(config.max_ticks, 11);
    EXPECT_EQ(config.tick_timeout_ms, 21);
    EXPECT_EQ(config.startup_timeout_ms, 31);
    EXPECT_EQ(config.status_period_ms, 40);
    EXPECT_EQ(config.headless, arguments.back() == "--headless");
  }
}

TEST(ServerConfigTest, CommandLineGuiOverridesYamlHeadless) {
  ScopedConfigFile file("mujoco_sim_server_gui_override.yaml",
                        "model_path: robot.xml\nheadless: true\n");
  mujoco_sim::server::ServerConfig config;
  std::string error;
  ASSERT_TRUE(ParseArgs({"mujoco-sim-server", "--config", file.path(),
                         "--gui"},
                        &config, &error))
      << error;
  EXPECT_FALSE(config.headless);
}

TEST(ServerConfigTest, LoadFailureDoesNotModifyConfig) {
  ScopedConfigFile file("mujoco_sim_server_transaction.yaml",
                        "server_name: changed\nunknown: value\n");
  mujoco_sim::server::ServerConfig config;
  config.server_name = "original";
  config.model_path = "original.xml";
  config.max_ticks = 7;
  const auto expected = config;
  std::string error;

  EXPECT_FALSE(
      mujoco_sim::server::LoadConfigFile(file.path(), &config, &error));
  ExpectConfigEquals(config, expected);
}

TEST(ServerConfigTest, CommandLineFailureDoesNotModifyConfig) {
  mujoco_sim::server::ServerConfig config;
  config.server_name = "original";
  config.model_path = "original.xml";
  config.max_ticks = 7;
  const auto expected = config;
  std::string error;

  EXPECT_FALSE(ParseArgs({"mujoco-sim-server", "--model", "changed.xml",
                          "--tick-timeout-ms", "4294967296"},
                         &config, &error));
  ExpectConfigEquals(config, expected);
}

TEST(ServerConfigTest, ReportsYamlErrorLocation) {
  ScopedConfigFile file("mujoco_sim_server_location.yaml",
                        "server_name: sim\nunknown: value\n");
  mujoco_sim::server::ServerConfig config;
  std::string error;
  EXPECT_FALSE(
      mujoco_sim::server::LoadConfigFile(file.path(), &config, &error));
  EXPECT_NE(error.find(file.path() + ":2:"), std::string::npos) << error;
}

TEST(ServerTopicsTest, UsesDocumentedRegistryName) {
  EXPECT_EQ(mujoco_sim::protocol::RegistryServiceName("sim"),
            "mujoco_sim.sim.registry");
  EXPECT_EQ(mujoco_sim::protocol::CommitTopicName("sim"),
            "mujoco_sim/sim/commit");
}

}  // namespace
