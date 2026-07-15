#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <mujoco_sim/protocol/topics.hpp>

#include "status_tool.hpp"

namespace {
using namespace mujoco_sim::server::status_tool;

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

CommandLineResult ParseArgs(const std::vector<std::string>& arguments,
                            StatusOptions* options, std::string* message) {
  std::vector<std::string> storage = arguments;
  std::vector<char*> argv;
  for (auto& argument : storage) argv.push_back(argument.data());
  return ParseCommandLine(static_cast<int>(argv.size()), argv.data(), options,
                          message);
}

mujoco_sim::v3::ServerStatus MakeStatus(
    const std::string& = "default", std::uint64_t epoch = 1) {
  mujoco_sim::v3::ServerStatus status;
  status.set_epoch(epoch);
  status.set_tick_id(2);
  status.set_state(mujoco_sim::v3::RUNNING);
  return status;
}

StatusReport MakeReport() {
  StatusReport report;
  report.server_name = "default";
  report.status = MakeStatus();
  report.status.set_message("healthy");
  auto* zeta = report.status.add_adapters();
  zeta->set_adapter_id("zeta");
  zeta->set_role(mujoco_sim::v3::CONTROLLER);
  zeta->set_session_id(9);
  zeta->set_last_delivery_tick_id(8);
  zeta->set_state(mujoco_sim::v3::ACTIVE);
  zeta->set_last_commit_tick_id(8);
  zeta->set_state_since_tick_id(3);
  zeta->set_required_for_barrier(true);
  auto* alpha = report.status.add_adapters();
  alpha->set_adapter_id("alpha");
  alpha->set_role(mujoco_sim::v3::CONTROLLER);
  alpha->set_session_id(7);
  alpha->set_state(mujoco_sim::v3::TIMED_OUT);
  alpha->set_message("controller timed out");
  alpha->set_state_since_tick_id(5);
  alpha->set_timeout_count(2);
  alpha->set_required_for_barrier(false);
  auto* error = report.status.add_adapters();
  error->set_adapter_id("error");
  error->set_role(mujoco_sim::v3::CONTROLLER);
  error->set_session_id(8);
  error->set_state(mujoco_sim::v3::ERROR);
  error->set_message("invalid command");
  error->set_last_commit_tick_id(6);
  error->set_state_since_tick_id(7);
  error->set_required_for_barrier(false);
  auto* observer = report.status.add_adapters();
  observer->set_adapter_id("observer");
  observer->set_role(mujoco_sim::v3::OBSERVER);
  observer->set_session_id(10);
  observer->set_state(mujoco_sim::v3::ACTIVE);
  observer->set_last_delivery_tick_id(8);
  observer->set_last_observation_sequence(4);
  auto* stream = report.status.add_streams();
  stream->set_name("front_rgb");
  stream->set_state(mujoco_sim::v3::STREAM_RUNNING);
  stream->set_subscriber_count(1);
  stream->set_captures(5);
  stream->set_published(4);
  stream->set_dropped(1);
  report.resources.emplace();
  report.resources->set_code(mujoco_sim::v3::OK);
  report.resources->set_epoch(1);
  auto* z_sensor = report.resources->add_observations();
  z_sensor->set_name("z_sensor");
  z_sensor->set_dimension(2);
  auto* a_sensor = report.resources->add_observations();
  a_sensor->set_name("a_sensor");
  a_sensor->set_dimension(1);
  auto* motor = report.resources->add_commands();
  motor->set_name("motor");
  motor->set_dimension(1);
  motor->set_has_ctrl_range(true);
  motor->set_ctrl_min(-1.0);
  motor->set_ctrl_max(1.0);
  return report;
}

TEST(StatusCommandLineTest, ParsesDefaultsAndOverrides) {
  StatusOptions defaults;
  std::string message;
  EXPECT_EQ(ParseArgs({"mujoco-sim-status"}, &defaults, &message),
            CommandLineResult::kSuccess);
  EXPECT_EQ(defaults.server_name, "default");
  EXPECT_EQ(defaults.timeout_ms, 3000u);
  EXPECT_EQ(defaults.interval_ms, 1000u);
  EXPECT_FALSE(defaults.watch);
  EXPECT_FALSE(defaults.json);

  StatusOptions configured;
  EXPECT_EQ(ParseArgs({"mujoco-sim-status", "--server-name", "robot",
                       "--timeout-ms", "4500", "--interval-ms", "250",
                       "--watch", "--json"},
                      &configured, &message),
            CommandLineResult::kSuccess)
      << message;
  EXPECT_EQ(configured.server_name, "robot");
  EXPECT_EQ(configured.timeout_ms, 4500u);
  EXPECT_EQ(configured.interval_ms, 250u);
  EXPECT_TRUE(configured.watch);
  EXPECT_TRUE(configured.json);
}

TEST(StatusCommandLineTest, GeneratesHelpAndRejectsInvalidArguments) {
  StatusOptions options;
  std::string message;
  EXPECT_EQ(ParseArgs({"mujoco-sim-status", "--help"}, &options, &message),
            CommandLineResult::kHelp);
  EXPECT_NE(message.find("--watch"), std::string::npos);
  EXPECT_NE(message.find("--config"), std::string::npos);
  EXPECT_EQ(message, Usage());
  const std::vector<std::vector<std::string>> invalid = {
      {"mujoco-sim-status", "--unknown"},
      {"mujoco-sim-status", "--timeout-ms"},
      {"mujoco-sim-status", "--timeout-ms", "0"},
      {"mujoco-sim-status", "--timeout-ms", "-1"},
      {"mujoco-sim-status", "--timeout-ms", "4294967296"},
      {"mujoco-sim-status", "--interval-ms", "0"},
      {"mujoco-sim-status", "--interval-ms", "-1"},
      {"mujoco-sim-status", "--server-name", ""},
  };
  for (const auto& arguments : invalid) {
    StatusOptions original;
    original.server_name = "unchanged";
    EXPECT_EQ(ParseArgs(arguments, &original, &message),
              CommandLineResult::kError);
    EXPECT_EQ(original.server_name, "unchanged");
    EXPECT_FALSE(message.empty());
  }
}

TEST(StatusCommandLineTest, LoadsYamlAndAppliesCommandLineOverrides) {
  ScopedConfigFile file("mujoco_sim_status.yaml",
                        "server_name: yaml_server\n"
                        "timeout_ms: 4000\n"
                        "interval_ms: 200\n"
                        "watch: true\n"
                        "json: false\n");
  StatusOptions options;
  std::string message;
  EXPECT_EQ(ParseArgs({"mujoco-sim-status", "--timeout-ms", "4500",
                       "--json", "--config", file.path()},
                      &options, &message),
            CommandLineResult::kSuccess)
      << message;
  EXPECT_EQ(options.server_name, "yaml_server");
  EXPECT_EQ(options.timeout_ms, 4500u);
  EXPECT_EQ(options.interval_ms, 200u);
  EXPECT_TRUE(options.watch);
  EXPECT_TRUE(options.json);
}

TEST(StatusCommandLineTest, RejectsInvalidYamlTransactionally) {
  const std::vector<std::pair<std::string, std::string>> invalid = {
      {"unknown: true\n", "unknown"},
      {"timeout_ms: 0\n", "greater than zero"},
      {"interval_ms: 4294967296\n", "range"},
      {"watch: \"true\"\n", "boolean"},
  };
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    SCOPED_TRACE(i);
    ScopedConfigFile file(
        "mujoco_sim_status_invalid_" + std::to_string(i) + ".yaml",
        invalid[i].first);
    StatusOptions options;
    options.server_name = "unchanged";
    const auto expected = options;
    std::string message;

    EXPECT_EQ(ParseArgs({"mujoco-sim-status", "--config", file.path()},
                        &options, &message),
              CommandLineResult::kError);
    EXPECT_NE(message.find(invalid[i].second), std::string::npos) << message;
    EXPECT_EQ(options.server_name, expected.server_name);
    EXPECT_EQ(options.timeout_ms, expected.timeout_ms);
    EXPECT_EQ(options.interval_ms, expected.interval_ms);
    EXPECT_EQ(options.watch, expected.watch);
    EXPECT_EQ(options.json, expected.json);
  }
}

TEST(StatusTrackerTest, LocksPublisherUntilStaleAndRejectsOldMessages) {
  StatusTracker tracker("default", std::chrono::milliseconds(1000));
  const PublisherIdentity first{"host", 10, 100};
  const PublisherIdentity second{"host", 11, 101};
  const auto start = std::chrono::steady_clock::time_point{};
  EXPECT_TRUE(tracker.Accept(first, 1, MakeStatus(), start).accepted);
  auto update = tracker.Accept(second, 1, MakeStatus(),
                               start + std::chrono::milliseconds(500));
  EXPECT_FALSE(update.accepted);
  EXPECT_TRUE(update.publisher_conflict);
  update = tracker.Accept(second, 1, MakeStatus(),
                          start + std::chrono::milliseconds(1000));
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.publisher_changed);
  EXPECT_FALSE(tracker.Accept(second, 1, MakeStatus(),
                              start + std::chrono::milliseconds(1100))
                   .accepted);
  update = tracker.Accept(second, 2, MakeStatus("default", 2),
                          start + std::chrono::milliseconds(1200));
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.epoch_changed);
}

TEST(StatusPayloadTest, RejectsInvalidPayloadsAndHeaders) {
  const auto input = MakeStatus();
  std::string bytes;
  ASSERT_TRUE(input.SerializeToString(&bytes));
  mujoco_sim::v3::ServerStatus parsed;
  EXPECT_TRUE(ParseStatusPayload(bytes.data(), bytes.size(), "default",
                                 &parsed));
  EXPECT_TRUE(
      ParseStatusPayload(bytes.data(), bytes.size(), "other", &parsed));
  EXPECT_FALSE(ParseStatusPayload(nullptr, bytes.size(), "default", &parsed));
  const std::string malformed = "not protobuf";
  EXPECT_FALSE(ParseStatusPayload(malformed.data(), malformed.size(),
                                  "default", &parsed));
}

TEST(StatusFormattingTest, ProducesSortedHumanOutput) {
  const std::string output = FormatHuman(MakeReport());
  EXPECT_NE(output.find("Server: default"), std::string::npos);
  EXPECT_NE(output.find("State: RUNNING"), std::string::npos);
  EXPECT_NE(output.find("Adapters (4)"), std::string::npos);
  EXPECT_EQ(output.find("Controllers ("), std::string::npos);
  EXPECT_NE(output.find("observer\tOBSERVER\tACTIVE\t10\t8\t-\t4\t0\t0\t-\t-"),
            std::string::npos);
  EXPECT_NE(output.find("Sensor streams (1)"), std::string::npos);
  EXPECT_NE(output.find("front_rgb\tSTREAM_RUNNING"), std::string::npos);
  EXPECT_NE(output.find("a_sensor[1], z_sensor[2]"), std::string::npos);
  EXPECT_LT(output.find("alpha"), output.find("zeta"));
  EXPECT_NE(output.find("motor[1]"), std::string::npos);
  EXPECT_NE(output.find("ID\tRole\tState\tSession\tLast delivery\tLast commit\t"
                        "Sequence\tState since\tTimeouts\tBarrier\tMessage\n"),
            std::string::npos);
  EXPECT_NE(output.find("alpha\tCONTROLLER\tTIMED_OUT\t7\t-\t-\t-\t5\t2\tno\t"
                        "controller timed out\n"),
            std::string::npos);
  EXPECT_NE(output.find("error\tCONTROLLER\tERROR\t8\t-\t6\t-\t7\t0\tno\t"
                        "invalid command\n"),
            std::string::npos);
  EXPECT_NE(output.find("zeta\tCONTROLLER\tACTIVE\t9\t8\t8\t-\t3\t0\tyes\t-\n"),
            std::string::npos);
}

TEST(StatusFormattingTest, PreservesUint64AndEscapesJson) {
  auto report = MakeReport();
  report.status.set_tick_id(std::numeric_limits<std::uint64_t>::max());
  report.status.set_message("quote \" slash \\ newline\n");
  std::string json;
  std::string error;
  ASSERT_TRUE(FormatJson(report, &json, &error)) << error;
  google::protobuf::Struct root;
  ASSERT_TRUE(google::protobuf::util::JsonStringToMessage(json, &root).ok());
  const auto& fields = root.fields();
  EXPECT_EQ(fields.size(), 4);
  EXPECT_EQ(fields.at("server_name").string_value(), "default");
  const auto& status = fields.at("status").struct_value().fields();
  EXPECT_EQ(status.at("tick_id").string_value(), "18446744073709551615");
  EXPECT_EQ(status.at("message").string_value(),
            "quote \" slash \\ newline\n");
  EXPECT_EQ(status.count("controllers"), 0);
  const auto& adapters = status.at("adapters").list_value();
  ASSERT_EQ(adapters.values_size(), 4);
  const auto& timed_out = adapters.values(0).struct_value().fields();
  EXPECT_EQ(timed_out.at("adapter_id").string_value(), "alpha");
  EXPECT_EQ(timed_out.at("state").string_value(), "TIMED_OUT");
  EXPECT_EQ(timed_out.count("last_commit_tick_id"), 0);
  EXPECT_EQ(timed_out.at("state_since_tick_id").string_value(), "5");
  EXPECT_EQ(timed_out.at("timeout_count").string_value(), "2");
  EXPECT_FALSE(timed_out.at("required_for_barrier").bool_value());
  const auto& controller_error =
      adapters.values(1).struct_value().fields();
  EXPECT_EQ(controller_error.at("adapter_id").string_value(), "error");
  EXPECT_EQ(controller_error.at("state").string_value(), "ERROR");
  EXPECT_EQ(controller_error.at("last_commit_tick_id").string_value(), "6");
  const auto& active = adapters.values(2).struct_value().fields();
  EXPECT_EQ(active.at("adapter_id").string_value(), "zeta");
  EXPECT_TRUE(active.at("required_for_barrier").bool_value());
  EXPECT_EQ(adapters.values(3)
                .struct_value()
                .fields()
                .at("role")
                .string_value(),
            "OBSERVER");
  const auto& streams = status.at("streams").list_value();
  ASSERT_EQ(streams.values_size(), 1);
  EXPECT_EQ(streams.values(0)
                .struct_value()
                .fields()
                .at("name")
                .string_value(),
            "front_rgb");
  EXPECT_EQ(streams.values(0)
                .struct_value()
                .fields()
                .at("dropped")
                .string_value(),
            "1");
  const auto& observations = fields.at("resources")
                                 .struct_value()
                                 .fields()
                                 .at("observations")
                                 .list_value();
  EXPECT_EQ(observations.values(0)
                .struct_value()
                .fields()
                .at("name")
                .string_value(),
            "a_sensor");
  EXPECT_EQ(fields.at("resource_error").kind_case(),
            google::protobuf::Value::kNullValue);
}

TEST(StatusFormattingTest, WritesUnavailableResourcesAsNdjson) {
  auto report = MakeReport();
  report.resources.reset();
  report.resource_error = "missing \"service\"\nretry";
  std::ostringstream output;
  std::string error;
  ASSERT_TRUE(WriteReport(report, true, &output, &error));
  ASSERT_TRUE(WriteReport(report, true, &output, &error));
  std::istringstream lines(output.str());
  std::string line;
  int count = 0;
  while (std::getline(lines, line)) {
    google::protobuf::Struct root;
    ASSERT_TRUE(
        google::protobuf::util::JsonStringToMessage(line, &root).ok());
    EXPECT_EQ(root.fields().size(), 4);
    EXPECT_EQ(root.fields().at("resources").kind_case(),
              google::protobuf::Value::kNullValue);
    EXPECT_EQ(root.fields().at("resource_error").string_value(),
              report.resource_error);
    const auto& adapters = root.fields()
                                  .at("status")
                                  .struct_value()
                                  .fields()
                                  .at("adapters")
                                  .list_value();
    ASSERT_EQ(adapters.values_size(), 4);
    const auto& timed_out = adapters.values(0).struct_value().fields();
    EXPECT_EQ(timed_out.at("state").string_value(), "TIMED_OUT");
    EXPECT_EQ(timed_out.count("last_commit_tick_id"), 0);
    EXPECT_FALSE(timed_out.at("required_for_barrier").bool_value());
    const auto& controller_error =
        adapters.values(1).struct_value().fields();
    EXPECT_EQ(controller_error.at("state").string_value(), "ERROR");
    EXPECT_EQ(controller_error.at("last_commit_tick_id").string_value(),
              "6");
    EXPECT_TRUE(adapters.values(2)
                    .struct_value()
                    .fields()
                    .at("required_for_barrier")
                    .bool_value());
    ++count;
  }
  EXPECT_EQ(count, 2);
}

}  // namespace
