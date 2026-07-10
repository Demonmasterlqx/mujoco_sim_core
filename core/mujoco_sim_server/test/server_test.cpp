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
  EXPECT_EQ(actual.max_ticks, expected.max_ticks);
  EXPECT_EQ(actual.tick_timeout_ms, expected.tick_timeout_ms);
  EXPECT_EQ(actual.startup_timeout_ms, expected.startup_timeout_ms);
  EXPECT_EQ(actual.status_period_ms, expected.status_period_ms);
}

TEST(ServerConfigTest, ParsesCommandLine) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  ASSERT_TRUE(ParseArgs({"mujoco-sim-server",
                         "--model",
                         "robot.xml",
                         "--server-name",
                         "sim",
                         "--max-ticks",
                         "10",
                         "--tick-timeout-ms",
                         "50",
                         "--startup-timeout-ms",
                         "250"},
                        &config, &error))
      << error;
  EXPECT_EQ(config.model_path, "robot.xml");
  EXPECT_EQ(config.server_name, "sim");
  EXPECT_EQ(config.max_ticks, 10);
  EXPECT_EQ(config.tick_timeout_ms, 50);
  EXPECT_EQ(config.startup_timeout_ms, 250);
  EXPECT_EQ(config.status_period_ms, 100);
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
}

TEST(ServerConfigTest, ReturnsGeneratedHelp) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  EXPECT_FALSE(ParseArgs({"mujoco-sim-server", "--help"}, &config, &error));
  EXPECT_EQ(error, mujoco_sim::server::Usage());
  EXPECT_NE(error.find("--startup-timeout-ms"), std::string::npos);
  EXPECT_NE(error.find("MuJoCo MJCF model path"), std::string::npos);
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

TEST(ServerConfigTest, LoadsStrictYamlWithAllFields) {
  ScopedConfigFile file(
      "mujoco_sim_server_all_fields.yaml",
      "server_name: \"sim:#1\" # quoted colon and hash\n"
      "model_path: robot.xml\n"
      "max_ticks: 18446744073709551615\n"
      "tick_timeout_ms: 4294967295\n"
      "startup_timeout_ms: 6000\n"
      "status_period_ms: 25\n");

  mujoco_sim::server::ServerConfig config;
  std::string error;
  ASSERT_TRUE(
      mujoco_sim::server::LoadConfigFile(file.path(), &config, &error))
      << error;
  EXPECT_EQ(config.server_name, "sim:#1");
  EXPECT_EQ(config.model_path, "robot.xml");
  EXPECT_EQ(config.max_ticks, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(config.tick_timeout_ms,
            std::numeric_limits<std::uint32_t>::max());
  EXPECT_EQ(config.startup_timeout_ms, 6000);
  EXPECT_EQ(config.status_period_ms, 25);
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
       "max_ticks must be an unsigned integer"},
      {"mujoco_sim_server_negative.yaml", "tick_timeout_ms: -1\n",
       "tick_timeout_ms must be an unsigned integer in range"},
      {"mujoco_sim_server_overflow.yaml", "status_period_ms: 4294967296\n",
       "status_period_ms must be an unsigned integer in range"},
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
      "status_period_ms: 40\n");

  const std::vector<std::vector<std::string>> argument_orders = {
      {"mujoco-sim-server", "--config", file.path(), "--model", "cli.xml",
       "--server-name", "cli", "--max-ticks", "11", "--tick-timeout-ms",
       "21", "--startup-timeout-ms", "31"},
      {"mujoco-sim-server", "--model", "cli.xml", "--server-name", "cli",
       "--max-ticks", "11", "--tick-timeout-ms", "21",
       "--startup-timeout-ms", "31", "--config", file.path()},
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
  }
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
