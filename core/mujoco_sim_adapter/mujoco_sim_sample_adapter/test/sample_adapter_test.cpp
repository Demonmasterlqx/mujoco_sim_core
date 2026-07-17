#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/sample_adapter/sample_adapter.hpp>

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

  std::string path() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

bool ParseArgs(std::vector<std::string> arguments,
               mujoco_sim::sample_adapter::SampleOptions* options, std::string* error) {
  std::vector<char*> argv;
  argv.reserve(arguments.size());
  for (auto& argument : arguments) {
    argv.push_back(argument.data());
  }
  return mujoco_sim::sample_adapter::ParseCommandLine(static_cast<int>(argv.size()), argv.data(),
                                                      options, error);
}

void ExpectOptionsEqual(const mujoco_sim::sample_adapter::SampleOptions& actual,
                        const mujoco_sim::sample_adapter::SampleOptions& expected) {
  EXPECT_EQ(actual.adapter.server_name, expected.adapter.server_name);
  EXPECT_EQ(actual.adapter.adapter_id, expected.adapter.adapter_id);
  EXPECT_EQ(actual.adapter.observation_names, expected.adapter.observation_names);
  EXPECT_EQ(actual.adapter.command_names, expected.adapter.command_names);
  EXPECT_EQ(actual.adapter.service_timeout_ms, expected.adapter.service_timeout_ms);
  EXPECT_EQ(actual.max_ticks, expected.max_ticks);
  EXPECT_EQ(actual.command_value, expected.command_value);
}

TEST(SampleAdapterTest, ParsesNameList) {
  const auto names = mujoco_sim::sample_adapter::ParseNameList(" hinge_pos,hinge_vel ,, imu ");
  ASSERT_EQ(names.size(), 3);
  EXPECT_EQ(names[0], "hinge_pos");
  EXPECT_EQ(names[1], "hinge_vel");
  EXPECT_EQ(names[2], "imu");
}

TEST(SampleAdapterTest, BuildsConstantCommandsFromRegistrationLayout) {
  mujoco_sim::adapter_sdk::ControllerLayout layout({}, {{"hinge_motor", {0, 1}}}, 0, 1);

  const auto commands = mujoco_sim::sample_adapter::ConstantCommandsForLayout(layout, 0.25);
  ASSERT_EQ(commands.size(), 1);
  EXPECT_DOUBLE_EQ(commands[0], 0.25);
}

TEST(SampleAdapterTest, ParsesCommandLineDefaultsAndOverrides) {
  const char* argv[] = {"mujoco-sim-sample-adapter",
                        "--server-name",
                        "sim",
                        "--adapter-id",
                        "arm",
                        "--ticks",
                        "4",
                        "--command-value",
                        "0.1"};
  mujoco_sim::sample_adapter::SampleOptions options;
  std::string error;
  ASSERT_TRUE(mujoco_sim::sample_adapter::ParseCommandLine(
      static_cast<int>(std::size(argv)), const_cast<char**>(argv), &options, &error))
      << error;
  EXPECT_EQ(options.adapter.server_name, "sim");
  EXPECT_EQ(options.adapter.adapter_id, "arm");
  EXPECT_EQ(options.max_ticks, 4);
  EXPECT_DOUBLE_EQ(options.command_value, 0.1);
  ASSERT_EQ(options.adapter.command_names.size(), 1);
  EXPECT_EQ(options.adapter.command_names[0], "hinge_motor");
}

TEST(SampleAdapterTest, RejectsInvalidNumericValuesAndDuplicateOptions) {
  const std::vector<std::vector<std::string>> invalid_arguments = {
      {"mujoco-sim-sample-adapter", "--ticks", "-1"},
      {"mujoco-sim-sample-adapter", "--ticks", "18446744073709551616"},
      {"mujoco-sim-sample-adapter", "--command-timeout-ms", "100"},
      {"mujoco-sim-sample-adapter", "--service-timeout-ms", "-1"},
      {"mujoco-sim-sample-adapter", "--command-value", "nan"},
      {"mujoco-sim-sample-adapter", "--command-value", "inf"},
      {"mujoco-sim-sample-adapter", "--ticks", "1", "--ticks", "2"},
  };

  for (const auto& arguments : invalid_arguments) {
    SCOPED_TRACE(testing::PrintToString(arguments));
    mujoco_sim::sample_adapter::SampleOptions options;
    std::string error;
    EXPECT_FALSE(ParseArgs(arguments, &options, &error));
    EXPECT_FALSE(error.empty());
  }
}

TEST(SampleAdapterTest, ParsingFailureIsTransactional) {
  mujoco_sim::sample_adapter::SampleOptions options;
  options.adapter.server_name = "unchanged";
  options.adapter.adapter_id = "original";
  options.adapter.observation_names = {"one"};
  options.adapter.command_names = {"motor"};
  options.adapter.service_timeout_ms = 456;
  options.max_ticks = 7;
  options.command_value = 0.25;
  const auto expected = options;
  std::string error;

  EXPECT_FALSE(ParseArgs(
      {"mujoco-sim-sample-adapter", "--server-name", "changed", "--command-timeout-ms", "100"},
      &options, &error));
  ExpectOptionsEqual(options, expected);
}

TEST(SampleAdapterTest, ReturnsGeneratedHelpForEverySupportedOption) {
  mujoco_sim::sample_adapter::SampleOptions options;
  std::string error;
  EXPECT_FALSE(ParseArgs({"mujoco-sim-sample-adapter", "--help"}, &options, &error));
  EXPECT_EQ(error, mujoco_sim::sample_adapter::Usage());
  EXPECT_NE(error.find("--config"), std::string::npos);
  EXPECT_NE(error.find("--service-timeout-ms"), std::string::npos);
}

TEST(SampleAdapterTest, LoadsYamlAndAppliesCommandLineOverrides) {
  ScopedConfigFile file("mujoco_sim_sample_adapter.yaml",
                        "server_name: yaml_server\n"
                        "adapter_id: yaml_controller\n"
                        "observation_names: [one, two]\n"
                        "command_names: [motor_a, motor_b]\n"
                        "max_ticks: 10\n"
                        "command_value: 0.2\n"
                        "service_timeout_ms: 400\n");
  mujoco_sim::sample_adapter::SampleOptions options;
  std::string error;

  ASSERT_TRUE(ParseArgs({"mujoco-sim-sample-adapter", "--server-name", "cli_server", "--ticks",
                         "11", "--config", file.path()},
                        &options, &error))
      << error;
  EXPECT_EQ(options.adapter.server_name, "cli_server");
  EXPECT_EQ(options.adapter.adapter_id, "yaml_controller");
  EXPECT_EQ(options.adapter.observation_names, (std::vector<std::string>{"one", "two"}));
  EXPECT_EQ(options.adapter.command_names, (std::vector<std::string>{"motor_a", "motor_b"}));
  EXPECT_EQ(options.max_ticks, 11u);
  EXPECT_DOUBLE_EQ(options.command_value, 0.2);
  EXPECT_EQ(options.adapter.service_timeout_ms, 400u);
}

TEST(SampleAdapterTest, RejectsInvalidYamlTransactionally) {
  const std::vector<std::pair<std::string, std::string>> invalid = {
      {"unknown: true\n", "unknown"},
      {"command_names: motor\n", "sequence"},
      {"command_value: nan\n", "finite"},
      {"command_timeout_ms: 300\n", "unknown"},
  };

  for (std::size_t i = 0; i < invalid.size(); ++i) {
    SCOPED_TRACE(i);
    ScopedConfigFile file("mujoco_sim_sample_adapter_invalid_" + std::to_string(i) + ".yaml",
                          invalid[i].first);
    mujoco_sim::sample_adapter::SampleOptions options;
    options.adapter.server_name = "unchanged";
    options.adapter.adapter_id = "original";
    options.adapter.command_names = {"motor"};
    const auto expected = options;
    std::string error;

    EXPECT_FALSE(
        ParseArgs({"mujoco-sim-sample-adapter", "--config", file.path()}, &options, &error));
    EXPECT_NE(error.find(invalid[i].second), std::string::npos) << error;
    ExpectOptionsEqual(options, expected);
  }
}

}  // namespace
