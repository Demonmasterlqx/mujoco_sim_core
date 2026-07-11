#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/server/server_config.hpp>
#include <spdlog/sinks/sink.h>

#include "server_logging.hpp"
#include "server_runtime.hpp"

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

class ScopedTempDirectory {
 public:
  explicit ScopedTempDirectory(std::string name)
      : path_(std::filesystem::temp_directory_path() /
              (std::move(name) + "_" +
               std::to_string(std::chrono::steady_clock::now()
                                  .time_since_epoch()
                                  .count()))) {
    std::filesystem::create_directories(path_);
  }

  ~ScopedTempDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

class ThrowingSink final : public spdlog::sinks::sink {
 public:
  void log(const spdlog::details::log_msg&) override {
    throw std::runtime_error("simulated sink failure");
  }
  void flush() override { throw std::runtime_error("simulated flush failure"); }
  void set_pattern(const std::string&) override {}
  void set_formatter(std::unique_ptr<spdlog::formatter>) override {}
};

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

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
  EXPECT_EQ(actual.log_level, expected.log_level);
  EXPECT_EQ(actual.log_file, expected.log_file);
  EXPECT_EQ(actual.log_file_max_size_mb, expected.log_file_max_size_mb);
  EXPECT_EQ(actual.log_file_max_files, expected.log_file_max_files);
  EXPECT_EQ(actual.log_summary_period_ms, expected.log_summary_period_ms);
  EXPECT_EQ(actual.activation_timeout_ms, expected.activation_timeout_ms);
  EXPECT_EQ(actual.allow_timed_out_adapter_replacement,
            expected.allow_timed_out_adapter_replacement);
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
                         "--activation-timeout-ms",
                         "750",
                         "--deny-timed-out-adapter-replacement",
                         "--log-level",
                         "debug",
                         "--log-file",
                         "server.log",
                         "--log-file-max-size-mb",
                         "20",
                         "--log-file-max-files",
                         "7",
                         "--log-summary-period-ms",
                         "2500",
                         "--headless"},
                        &config, &error))
      << error;
  EXPECT_EQ(config.model_path, "robot.xml");
  EXPECT_EQ(config.server_name, "sim");
  EXPECT_EQ(config.initial_keyframe, "home");
  EXPECT_EQ(config.max_ticks, 10);
  EXPECT_EQ(config.tick_timeout_ms, 50);
  EXPECT_EQ(config.startup_timeout_ms, 250);
  EXPECT_EQ(config.activation_timeout_ms, 750U);
  EXPECT_FALSE(config.allow_timed_out_adapter_replacement);
  EXPECT_EQ(config.status_period_ms, 100);
  EXPECT_TRUE(config.headless);
  EXPECT_EQ(config.log_level, "debug");
  EXPECT_EQ(config.log_file, "server.log");
  EXPECT_EQ(config.log_file_max_size_mb, 20U);
  EXPECT_EQ(config.log_file_max_files, 7U);
  EXPECT_EQ(config.log_summary_period_ms, 2500U);
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
  EXPECT_EQ(config.activation_timeout_ms, 5000U);
  EXPECT_TRUE(config.allow_timed_out_adapter_replacement);
  EXPECT_EQ(config.status_period_ms, 100);
  EXPECT_FALSE(config.headless);
  EXPECT_EQ(config.log_level, "info");
  EXPECT_TRUE(config.log_file.empty());
  EXPECT_EQ(config.log_file_max_size_mb, 10U);
  EXPECT_EQ(config.log_file_max_files, 5U);
  EXPECT_EQ(config.log_summary_period_ms, 5000U);
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
  EXPECT_EQ(config.log_level, "info");
}

TEST(ServerConfigTest, ReturnsGeneratedHelp) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  EXPECT_FALSE(ParseArgs({"mujoco-sim-server", "--help"}, &config, &error));
  EXPECT_EQ(error, mujoco_sim::server::Usage());
  EXPECT_NE(error.find("--startup-timeout-ms"), std::string::npos);
  EXPECT_NE(error.find("--activation-timeout-ms"), std::string::npos);
  EXPECT_NE(error.find("--allow-timed-out-adapter-replacement"),
            std::string::npos);
  EXPECT_NE(error.find("--deny-timed-out-adapter-replacement"),
            std::string::npos);
  EXPECT_NE(error.find("MuJoCo MJCF model path"), std::string::npos);
  EXPECT_NE(error.find("--initial-keyframe"), std::string::npos);
  EXPECT_NE(error.find("--headless"), std::string::npos);
  EXPECT_NE(error.find("--gui"), std::string::npos);
  EXPECT_NE(error.find("--log-level"), std::string::npos);
  EXPECT_NE(error.find("--log-summary-period-ms"), std::string::npos);
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
      {"mujoco-sim-server", "--model", "robot.xml", "--log-level",
       "verbose"},
      {"mujoco-sim-server", "--model", "robot.xml",
       "--log-file-max-size-mb", "0"},
      {"mujoco-sim-server", "--model", "robot.xml",
       "--log-file-max-files", "200001"},
      {"mujoco-sim-server", "--model", "robot.xml",
       "--activation-timeout-ms", "0"},
      {"mujoco-sim-server", "--model", "robot.xml",
       "--allow-timed-out-adapter-replacement",
       "--deny-timed-out-adapter-replacement"},
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
      "activation_timeout_ms: 7000\n"
      "allow_timed_out_adapter_replacement: false\n"
      "status_period_ms: 25\n"
      "headless: true\n"
      "log_level: trace\n"
      "log_file: logs/server.log\n"
      "log_file_max_size_mb: 20\n"
      "log_file_max_files: 7\n"
      "log_summary_period_ms: 2500\n");

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
  EXPECT_EQ(config.activation_timeout_ms, 7000U);
  EXPECT_FALSE(config.allow_timed_out_adapter_replacement);
  EXPECT_EQ(config.status_period_ms, 25);
  EXPECT_TRUE(config.headless);
  EXPECT_EQ(config.log_level, "trace");
  EXPECT_EQ(config.log_file,
            (std::filesystem::temp_directory_path() / "logs/server.log")
                .string());
  EXPECT_EQ(config.log_file_max_size_mb, 20U);
  EXPECT_EQ(config.log_file_max_files, 7U);
  EXPECT_EQ(config.log_summary_period_ms, 2500U);
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
      {"mujoco_sim_server_log_level.yaml", "log_level: verbose\n",
       "log_level must be one of"},
      {"mujoco_sim_server_log_size.yaml", "log_file_max_size_mb: 0\n",
       "log_file_max_size_mb must be >= 1"},
      {"mujoco_sim_server_log_files.yaml", "log_file_max_files: 200001\n",
       "log_file_max_files must be <= 200000"},
      {"mujoco_sim_server_activation_timeout.yaml",
       "activation_timeout_ms: 0\n", "activation_timeout_ms must be >= 1"},
      {"mujoco_sim_server_replacement_bool.yaml",
       "allow_timed_out_adapter_replacement: \"true\"\n",
       "allow_timed_out_adapter_replacement must be a boolean"},
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
      "headless: false\n"
      "log_level: warn\n"
      "log_file: yaml.log\n"
      "log_summary_period_ms: 9000\n");

  const std::vector<std::vector<std::string>> argument_orders = {
      {"mujoco-sim-server", "--config", file.path(), "--model", "cli.xml",
       "--server-name", "cli", "--max-ticks", "11", "--tick-timeout-ms",
       "21", "--startup-timeout-ms", "31"},
      {"mujoco-sim-server", "--model", "cli.xml", "--server-name", "cli",
       "--max-ticks", "11", "--tick-timeout-ms", "21",
       "--startup-timeout-ms", "31", "--config", file.path(),
       "--headless", "--log-level", "debug", "--log-file", "cli.log",
       "--log-summary-period-ms", "2500"},
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
    EXPECT_EQ(config.headless, arguments.back() != "31");
    if (arguments.back() == "31") {
      EXPECT_EQ(config.log_level, "warn");
      EXPECT_EQ(config.log_file,
                (std::filesystem::temp_directory_path() / "yaml.log").string());
      EXPECT_EQ(config.log_summary_period_ms, 9000U);
    } else {
      EXPECT_EQ(config.log_level, "debug");
      EXPECT_EQ(config.log_file, "cli.log");
      EXPECT_EQ(config.log_summary_period_ms, 2500U);
    }
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

TEST(ServerLoggingTest, EscapesAndBoundsStringValues) {
  EXPECT_EQ(mujoco_sim::server::LogValue("a\n\"b\\c"),
            "\"a\\n\\\"b\\\\c\"");
  EXPECT_EQ(mujoco_sim::server::LogValue("abcdef", 3), "\"abc...\"");
}

TEST(ServerLoggingTest, RateLimitsWarningsByStableKey) {
  using namespace std::chrono_literals;
  mujoco_sim::server::WarningLimiter limiter(1s);
  const auto start = std::chrono::steady_clock::time_point(10s);

  auto decision = limiter.AdmitAt("publish", start);
  EXPECT_TRUE(decision.emit);
  EXPECT_EQ(decision.suppressed, 0U);
  EXPECT_FALSE(limiter.AdmitAt("publish", start + 100ms).emit);
  EXPECT_FALSE(limiter.AdmitAt("publish", start + 200ms).emit);
  EXPECT_TRUE(limiter.AdmitAt("other", start + 200ms).emit);

  decision = limiter.AdmitAt("publish", start + 1s);
  EXPECT_TRUE(decision.emit);
  EXPECT_EQ(decision.suppressed, 2U);
}

TEST(ServerRuntimeTest, EnforcesProtocolPayloadBoundary) {
  EXPECT_TRUE(mujoco_sim::server::IsProtocolPayloadSizeAllowed(
      mujoco_sim::server::kMaxProtocolPayloadBytes - 1));
  EXPECT_TRUE(mujoco_sim::server::IsProtocolPayloadSizeAllowed(
      mujoco_sim::server::kMaxProtocolPayloadBytes));
  EXPECT_FALSE(mujoco_sim::server::IsProtocolPayloadSizeAllowed(
      mujoco_sim::server::kMaxProtocolPayloadBytes + 1));
}

TEST(ServerRuntimeTest, ClassifiesObservableSendOutcomes) {
  using mujoco_sim::server::SendOutcome;
  EXPECT_EQ(mujoco_sim::server::ClassifySendOutcome(true, 0, 0),
            SendOutcome::kSent);
  EXPECT_EQ(mujoco_sim::server::ClassifySendOutcome(false, 0, 0),
            SendOutcome::kNoSubscribers);
  EXPECT_EQ(mujoco_sim::server::ClassifySendOutcome(false, 1, 0),
            SendOutcome::kNoSubscribers);
  EXPECT_EQ(mujoco_sim::server::ClassifySendOutcome(false, 0, 1),
            SendOutcome::kNoSubscribers);
  EXPECT_EQ(mujoco_sim::server::ClassifySendOutcome(false, 1, 1),
            SendOutcome::kRejectedWithSubscribers);
}

TEST(ServerRuntimeTest, PreservesSubMillisecondWakeDeadlines) {
  using namespace std::chrono_literals;
  const auto now = std::chrono::steady_clock::time_point(10s);
  const auto next_tick = now + 250us;
  EXPECT_EQ(mujoco_sim::server::ComputeLoopWakeDeadline(
                now, next_tick, false, 5ms),
            next_tick);
  EXPECT_EQ(mujoco_sim::server::ComputeLoopWakeDeadline(
                now, next_tick, true, 5ms),
            now + 5ms);
  EXPECT_EQ(mujoco_sim::server::ComputeLoopWakeDeadline(
                now, now - 1us, false, 5ms),
            now - 1us);
}

TEST(ServerRuntimeTest, CalculatesRatesAndSuccessfulBarrierLatency) {
  const auto rates = mujoco_sim::server::CalculateRuntimeRates(
      2.0, 10, 20, 1.0, 1.5, 4, 12.0, 5.0);
  EXPECT_DOUBLE_EQ(rates.step_rate_hz, 5.0);
  EXPECT_DOUBLE_EQ(rates.real_time_factor, 0.25);
  EXPECT_DOUBLE_EQ(rates.barrier_latency_avg_ms, 3.0);
  EXPECT_DOUBLE_EQ(rates.barrier_latency_max_ms, 5.0);

  const auto empty = mujoco_sim::server::CalculateRuntimeRates(
      0.0, 20, 20, 1.5, 1.5, 0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(empty.step_rate_hz, 0.0);
  EXPECT_DOUBLE_EQ(empty.real_time_factor, 0.0);
  EXPECT_DOUBLE_EQ(empty.barrier_latency_avg_ms, 0.0);
}

TEST(ServerRuntimeTest, AggregatesAndResetsDeterministicWindows) {
  using namespace std::chrono_literals;
  using Metrics = mujoco_sim::server::RuntimeMetrics;
  const auto start = Metrics::TimePoint(10s);
  Metrics metrics;
  metrics.Initialize(start, 10, 1.0);
  EXPECT_FALSE(metrics.SummaryDue(start + 1999ms, 2s));
  EXPECT_TRUE(metrics.SummaryDue(start + 2s, 2s));

  metrics.RecordTickOpened(start + 100ms);
  metrics.RecordTickPublishBatch(10, 2, 4, 1, 3, 8);
  EXPECT_DOUBLE_EQ(metrics.RecordCommit(true, false, false, start + 150ms),
                   0.0);
  EXPECT_DOUBLE_EQ(metrics.RecordCommit(false, true, false, start + 200ms),
                   0.0);
  EXPECT_DOUBLE_EQ(metrics.RecordCommit(false, false, true, start + 400ms),
                   300.0);
  metrics.RecordTimeout();
  metrics.RecordMalformedMessage();
  metrics.RecordMalformedMessage();
  metrics.RecordSuppressedWarning();

  const auto first = metrics.TakeSummary(start + 2s, 15, 1.5);
  EXPECT_DOUBLE_EQ(first.window_seconds, 2.0);
  EXPECT_EQ(first.tick_id, 15U);
  EXPECT_DOUBLE_EQ(first.simulation_time, 1.5);
  EXPECT_EQ(first.ticks_opened, 1U);
  EXPECT_EQ(first.tick_publish_attempts, 10U);
  EXPECT_EQ(first.tick_serialization_failures, 2U);
  EXPECT_EQ(first.tick_messages_sent, 4U);
  EXPECT_EQ(first.tick_send_rejected_with_subscribers, 1U);
  EXPECT_EQ(first.tick_no_subscribers, 3U);
  EXPECT_EQ(first.tick_retransmissions, 8U);
  EXPECT_EQ(first.commits, 3U);
  EXPECT_EQ(first.duplicate_commits, 1U);
  EXPECT_EQ(first.rejected_commits, 1U);
  EXPECT_EQ(first.timeouts, 1U);
  EXPECT_EQ(first.malformed_messages, 2U);
  EXPECT_EQ(first.suppressed_warnings, 1U);
  EXPECT_EQ(first.completed_barriers, 1U);
  EXPECT_DOUBLE_EQ(first.rates.step_rate_hz, 2.5);
  EXPECT_DOUBLE_EQ(first.rates.real_time_factor, 0.25);
  EXPECT_DOUBLE_EQ(first.rates.barrier_latency_avg_ms, 300.0);
  EXPECT_DOUBLE_EQ(first.rates.barrier_latency_max_ms, 300.0);
  EXPECT_FALSE(metrics.SummaryDue(start + 2s, 2s));

  metrics.RecordTickOpened(start + 2100ms);
  const auto second = metrics.TakeSummary(start + 3s, 15, 1.5);
  EXPECT_EQ(second.ticks_opened, 1U);
  EXPECT_EQ(second.completed_barriers, 0U);
  EXPECT_DOUBLE_EQ(
      metrics.RecordCommit(false, false, true, start + 3100ms), 1000.0);
  const auto third = metrics.TakeSummary(start + 4s, 16, 1.6);
  EXPECT_EQ(third.ticks_opened, 0U);
  EXPECT_EQ(third.commits, 1U);
  EXPECT_EQ(third.completed_barriers, 1U);
  EXPECT_DOUBLE_EQ(third.rates.barrier_latency_avg_ms, 1000.0);
  EXPECT_DOUBLE_EQ(third.rates.barrier_latency_max_ms, 1000.0);
}

TEST(ServerRuntimeTest, RecordsBarrierCompletionWithoutACommit) {
  using namespace std::chrono_literals;
  using Metrics = mujoco_sim::server::RuntimeMetrics;
  const auto start = Metrics::TimePoint(10s);
  Metrics metrics;
  metrics.Initialize(start, 0, 0.0);
  metrics.RecordTickOpened(start + 100us);
  EXPECT_DOUBLE_EQ(metrics.RecordBarrierCompleted(start + 350us), 0.25);

  const auto summary = metrics.TakeSummary(start + 1s, 1, 0.001);
  EXPECT_EQ(summary.commits, 0U);
  EXPECT_EQ(summary.completed_barriers, 1U);
  EXPECT_DOUBLE_EQ(summary.rates.barrier_latency_avg_ms, 0.25);
  EXPECT_DOUBLE_EQ(summary.rates.barrier_latency_max_ms, 0.25);
}

TEST(ServerRuntimeTest, StopsNewCallbacksAndWaitsForInflightWork) {
  using namespace std::chrono_literals;
  mujoco_sim::server::CallbackState state;
  ASSERT_TRUE(state.Enter());
  std::atomic<bool> waiting = false;
  std::atomic<bool> finished = false;
  std::thread teardown([&]() {
    state.StopAccepting();
    waiting.store(true);
    state.WaitForIdle();
    finished.store(true);
  });

  while (!waiting.load()) {
    std::this_thread::yield();
  }
  std::this_thread::sleep_for(20ms);
  EXPECT_FALSE(finished.load());
  EXPECT_FALSE(state.Enter());
  state.Leave();
  teardown.join();
  EXPECT_TRUE(finished.load());
}

TEST(ServerLoggingTest, RejectsInvalidDirectConfiguration) {
  const std::vector<mujoco_sim::server::ServerConfig> invalid = []() {
    std::vector<mujoco_sim::server::ServerConfig> values(3);
    values[0].log_level = "verbose";
    values[1].log_file_max_size_mb = 0;
    values[2].log_file_max_files = 200001;
    return values;
  }();

  for (const auto& config : invalid) {
    mujoco_sim::server::ServerLogger logger;
    std::string error;
    EXPECT_FALSE(logger.Initialize(config, &error));
    EXPECT_FALSE(error.empty());
  }
}

TEST(ServerLoggingTest, WritesConfiguredLevelsAndCreatesDirectories) {
  ScopedTempDirectory directory("mujoco_sim_server_logging");
  const auto log_path = directory.path() / "nested" / "server.log";
  mujoco_sim::server::ServerConfig config;
  config.log_level = "info";
  config.log_file = log_path.string();

  testing::internal::CaptureStderr();
  {
    mujoco_sim::server::ServerLogger logger;
    std::string error;
    ASSERT_TRUE(logger.Initialize(config, &error)) << error;
    logger.get()->debug("event=hidden");
    logger.get()->info("event=visible value={}", 7);
    logger.Flush();
  }
  const std::string standard_error = testing::internal::GetCapturedStderr();
  const std::string file = ReadFile(log_path);
  EXPECT_NE(standard_error.find("event=visible value=7"), std::string::npos);
  EXPECT_EQ(standard_error.find("event=hidden"), std::string::npos);
  EXPECT_NE(file.find("event=visible value=7"), std::string::npos);
  EXPECT_EQ(file.find("event=hidden"), std::string::npos);
  EXPECT_EQ(file.find("\x1b["), std::string::npos);
}

TEST(ServerLoggingTest, FailsWhenLogParentCannotBeCreated) {
  ScopedTempDirectory directory("mujoco_sim_server_logging_failure");
  const auto regular_file = directory.path() / "parent";
  std::ofstream(regular_file) << "not a directory";

  mujoco_sim::server::ServerConfig config;
  config.log_file = (regular_file / "server.log").string();
  mujoco_sim::server::ServerLogger logger;
  std::string error;
  EXPECT_FALSE(logger.Initialize(config, &error));
  EXPECT_NE(error.find("failed to initialize server logging"),
            std::string::npos);
}

TEST(ServerLoggingTest, SupportsConcurrentWrites) {
  ScopedTempDirectory directory("mujoco_sim_server_logging_concurrent");
  const auto log_path = directory.path() / "server.log";
  mujoco_sim::server::ServerConfig config;
  config.log_file = log_path.string();

  testing::internal::CaptureStderr();
  {
    mujoco_sim::server::ServerLogger logger;
    std::string error;
    ASSERT_TRUE(logger.Initialize(config, &error)) << error;
    std::vector<std::thread> threads;
    for (int thread = 0; thread < 4; ++thread) {
      threads.emplace_back([log = logger.get(), thread]() {
        for (int line = 0; line < 25; ++line) {
          log->info("event=concurrent thread={} line={}", thread, line);
        }
      });
    }
    for (auto& thread : threads) {
      thread.join();
    }
    logger.Flush();
  }
  testing::internal::GetCapturedStderr();

  const std::string file = ReadFile(log_path);
  std::size_t count = 0;
  std::size_t position = 0;
  while ((position = file.find("event=concurrent", position)) !=
         std::string::npos) {
    ++count;
    position += 1;
  }
  EXPECT_EQ(count, 100U);
}

TEST(ServerLoggingTest, ReportsRuntimeSinkFailures) {
  mujoco_sim::server::ServerConfig config;
  testing::internal::CaptureStderr();
  mujoco_sim::server::ServerLogger logger;
  std::string error;
  ASSERT_TRUE(logger.Initialize(config, &error)) << error;
  logger.get()->sinks().push_back(std::make_shared<ThrowingSink>());
  logger.get()->info("event=trigger_sink_failure");
  EXPECT_FALSE(logger.healthy());
  logger.get()->sinks().pop_back();
  const std::string standard_error = testing::internal::GetCapturedStderr();
  EXPECT_NE(standard_error.find("event=log_sink_error"), std::string::npos);
  EXPECT_NE(standard_error.find("simulated sink failure"), std::string::npos);
}

TEST(ServerLoggingTest, RotatesWithinConfiguredTotalFileCount) {
  ScopedTempDirectory directory("mujoco_sim_server_logging_rotation");
  const auto log_path = directory.path() / "server.log";
  mujoco_sim::server::ServerConfig config;
  config.log_file = log_path.string();
  config.log_file_max_size_mb = 1;
  config.log_file_max_files = 2;

  testing::internal::CaptureStderr();
  {
    mujoco_sim::server::ServerLogger logger;
    std::string error;
    ASSERT_TRUE(logger.Initialize(config, &error)) << error;
    const std::string payload(600000, 'x');
    for (int index = 0; index < 4; ++index) {
      logger.get()->info("event=rotation index={} payload={}", index, payload);
    }
    logger.Flush();
  }
  testing::internal::GetCapturedStderr();

  EXPECT_TRUE(std::filesystem::exists(log_path));
  EXPECT_TRUE(std::filesystem::exists(directory.path() / "server.1.log"));
  EXPECT_FALSE(std::filesystem::exists(directory.path() / "server.2.log"));
}

TEST(ServerTopicsTest, UsesDocumentedRegistryName) {
  EXPECT_EQ(mujoco_sim::protocol::RegistryServiceName("sim"),
            "mujoco_sim.sim.registry");
  EXPECT_EQ(mujoco_sim::protocol::CommitTopicName("sim"),
            "mujoco_sim/sim/commit");
}

}  // namespace
