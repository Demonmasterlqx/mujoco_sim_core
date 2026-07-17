#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "app_config.hpp"
#include "mapper_scan_accounting.hpp"
#include "sensor_demo_generated_config.hpp"

namespace demo = mujoco_sim::sensor_demo;
namespace generated = mujoco_sim::sensor_demo::generated;
namespace {

class TemporaryFile {
 public:
  explicit TemporaryFile(const std::string& contents) {
    static std::atomic<std::uint64_t> sequence{0};
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("sensor_demo_config_" + std::to_string(ticks) + "_" +
             std::to_string(sequence.fetch_add(1)) + ".yaml");
    std::ofstream output(path_);
    output << contents;
  }
  ~TemporaryFile() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

demo::ParseAppConfigResult Parse(const std::vector<std::string>& arguments, demo::AppConfig* config,
                                 std::string* message) {
  std::vector<std::string> storage = arguments;
  std::vector<char*> argv;
  argv.reserve(storage.size());
  for (auto& argument : storage)
    argv.push_back(argument.data());
  return demo::ParseAppConfig(static_cast<int>(argv.size()), argv.data(), config, message);
}

}  // namespace

TEST(AppConfigTest, LoadsStrictYamlAndBuildsAllFourProcessConfigurations) {
  demo::AppConfig config;
  std::string message;
  ASSERT_EQ(Parse({"sensor-demo", "--config", SENSOR_DEMO_CONFIG_DIR "/sensor_demo.yaml"}, &config,
                  &message),
            demo::ParseAppConfigResult::kSuccess)
      << message;
  EXPECT_EQ(config.server_name, "sensor_demo");
  EXPECT_EQ(config.controller.adapter.observation_names.size(), 6U);
  EXPECT_EQ(config.controller.adapter.command_names.size(), 4U);
  EXPECT_EQ(config.state_observer.observer.observation_names.size(), 11U);
  EXPECT_EQ(config.mapper.observer.observation_names, std::vector<std::string>{"imu_quat"});
  EXPECT_EQ(config.camera_consumer.streams.stream_names.size(), 4U);
  EXPECT_EQ(config.mapper.streams.stream_names, std::vector<std::string>{"lidar_points"});
  EXPECT_TRUE(config.controller.control.stop_after_lap);
}

TEST(AppConfigTest, WiresEveryRenamedAndReorderedInterfaceName) {
  generated::SensorDemoConfig source;
  const std::array<std::string, 4> controller_positions = {
      "controller_position_3", "controller_position_1", "controller_position_4",
      "controller_position_2"};
  const std::array<std::string, 4> controller_commands = {"wheel_command_4", "wheel_command_2",
                                                          "wheel_command_1", "wheel_command_3"};
  source.controller.interfaces.wheel_positions = controller_positions;
  source.controller.interfaces.imu_quaternion = "controller_imu_quaternion_2";
  source.controller.interfaces.imu_angular_velocity = "controller_imu_angular_velocity_1";
  source.controller.interfaces.wheel_velocity_commands = controller_commands;

  const std::array<std::string, 4> state_positions = {"observer_position_2", "observer_position_4",
                                                      "observer_position_1", "observer_position_3"};
  const std::array<std::string, 4> state_velocities = {
      "observer_velocity_3", "observer_velocity_1", "observer_velocity_4", "observer_velocity_2"};
  source.state_observer.interfaces.wheel_positions = state_positions;
  source.state_observer.interfaces.wheel_velocities = state_velocities;
  source.state_observer.interfaces.imu_quaternion = "observer_imu_quaternion_3";
  source.state_observer.interfaces.imu_angular_velocity = "observer_imu_angular_velocity_1";
  source.state_observer.interfaces.imu_linear_acceleration = "observer_imu_linear_acceleration_2";

  source.camera_consumer.streams.rgb = "camera_stream_4_rgb";
  source.camera_consumer.streams.depth = "camera_stream_1_depth";
  source.camera_consumer.streams.segmentation = "camera_stream_3_segmentation";
  source.camera_consumer.streams.pointcloud = "camera_stream_2_pointcloud";
  source.mapper.imu_quaternion = "mapper_imu_quaternion_2";
  source.mapper.lidar_stream = "mapper_stream_1_lidar";

  demo::AppConfig config;
  std::string message;
  ASSERT_TRUE(demo::BuildAppConfig(source, &config, &message)) << message;

  EXPECT_EQ(config.controller.interfaces.wheel_positions, controller_positions);
  EXPECT_EQ(config.controller.interfaces.imu_quaternion,
            source.controller.interfaces.imu_quaternion);
  EXPECT_EQ(config.controller.interfaces.imu_angular_velocity,
            source.controller.interfaces.imu_angular_velocity);
  EXPECT_EQ(config.controller.interfaces.wheel_velocity_commands, controller_commands);
  EXPECT_EQ(config.controller.adapter.observation_names,
            (std::vector<std::string>{"controller_position_3", "controller_position_1",
                                      "controller_position_4", "controller_position_2",
                                      "controller_imu_quaternion_2",
                                      "controller_imu_angular_velocity_1"}));
  EXPECT_EQ(config.controller.adapter.command_names,
            (std::vector<std::string>{"wheel_command_4", "wheel_command_2", "wheel_command_1",
                                      "wheel_command_3"}));

  EXPECT_EQ(config.state_observer.interfaces.wheel_positions, state_positions);
  EXPECT_EQ(config.state_observer.interfaces.wheel_velocities, state_velocities);
  EXPECT_EQ(config.state_observer.interfaces.imu_quaternion,
            source.state_observer.interfaces.imu_quaternion);
  EXPECT_EQ(config.state_observer.interfaces.imu_angular_velocity,
            source.state_observer.interfaces.imu_angular_velocity);
  EXPECT_EQ(config.state_observer.interfaces.imu_linear_acceleration,
            source.state_observer.interfaces.imu_linear_acceleration);
  EXPECT_EQ(config.state_observer.observer.observation_names,
            (std::vector<std::string>{
                "observer_position_2", "observer_position_4", "observer_position_1",
                "observer_position_3", "observer_velocity_3", "observer_velocity_1",
                "observer_velocity_4", "observer_velocity_2", "observer_imu_quaternion_3",
                "observer_imu_angular_velocity_1", "observer_imu_linear_acceleration_2"}));

  EXPECT_EQ(config.camera_consumer.rgb_stream, "camera_stream_4_rgb");
  EXPECT_EQ(config.camera_consumer.depth_stream, "camera_stream_1_depth");
  EXPECT_EQ(config.camera_consumer.segmentation_stream, "camera_stream_3_segmentation");
  EXPECT_EQ(config.camera_consumer.pointcloud_stream, "camera_stream_2_pointcloud");
  EXPECT_EQ(
      config.camera_consumer.streams.stream_names,
      (std::vector<std::string>{"camera_stream_4_rgb", "camera_stream_1_depth",
                                "camera_stream_3_segmentation", "camera_stream_2_pointcloud"}));

  EXPECT_EQ(config.mapper.imu_quaternion, "mapper_imu_quaternion_2");
  EXPECT_EQ(config.mapper.observer.observation_names,
            (std::vector<std::string>{"mapper_imu_quaternion_2"}));
  EXPECT_EQ(config.mapper.lidar_stream, "mapper_stream_1_lidar");
  EXPECT_EQ(config.mapper.streams.stream_names,
            (std::vector<std::string>{"mapper_stream_1_lidar"}));
}

TEST(AppConfigTest, CliOverridesYamlRegardlessOfConfigArgumentOrder) {
  for (bool config_first : {true, false}) {
    std::vector<std::string> arguments{"sensor-demo"};
    if (config_first) {
      arguments.insert(arguments.end(), {"--config", SENSOR_DEMO_CONFIG_DIR "/sensor_demo.yaml"});
    }
    arguments.insert(arguments.end(), {"--server-name", "cli_server", "--output-dir", "cli_output",
                                       "--target-speed", "0.75"});
    if (!config_first) {
      arguments.insert(arguments.end(), {"--config", SENSOR_DEMO_CONFIG_DIR "/sensor_demo.yaml"});
    }
    demo::AppConfig config;
    std::string message;
    ASSERT_EQ(Parse(arguments, &config, &message), demo::ParseAppConfigResult::kSuccess) << message;
    EXPECT_EQ(config.server_name, "cli_server");
    EXPECT_EQ(config.output_directory, "cli_output");
    EXPECT_DOUBLE_EQ(config.controller.control.target_speed, 0.75);
  }
}

TEST(AppConfigTest, ParseFailureIsTransactional) {
  TemporaryFile invalid("common:\n  unknown_field: true\n");
  demo::AppConfig config;
  config.server_name = "sentinel";
  config.output_directory = "unchanged";
  std::string message;
  EXPECT_EQ(Parse({"sensor-demo", "--config", invalid.path().string()}, &config, &message),
            demo::ParseAppConfigResult::kError);
  EXPECT_NE(message.find("unknown"), std::string::npos);
  EXPECT_EQ(config.server_name, "sentinel");
  EXPECT_EQ(config.output_directory, "unchanged");
}

TEST(AppConfigTest, CrossFieldFailureIsTransactional) {
  generated::SensorDemoConfig generated_config;
  generated_config.mapper.adapter_id = generated_config.state_observer.adapter_id;
  demo::AppConfig config;
  config.server_name = "sentinel";
  std::string message;
  EXPECT_FALSE(demo::BuildAppConfig(generated_config, &config, &message));
  EXPECT_NE(message.find("duplicate"), std::string::npos);
  EXPECT_EQ(config.server_name, "sentinel");

  generated_config = generated::SensorDemoConfig{};
  generated_config.mapper.heartbeat_period_ms = generated_config.mapper.session_timeout_ms;
  EXPECT_FALSE(demo::BuildAppConfig(generated_config, &config, &message));
  EXPECT_NE(message.find("heartbeat_period_ms"), std::string::npos);
  EXPECT_EQ(config.server_name, "sentinel");
}

TEST(AppConfigTest, HelpComesFromGeneratedCli) {
  demo::AppConfig config;
  std::string message;
  EXPECT_EQ(Parse({"sensor-demo", "--help"}, &config, &message), demo::ParseAppConfigResult::kHelp);
  EXPECT_EQ(message, demo::AppUsage());
  EXPECT_NE(message.find("--server-name"), std::string::npos);
  EXPECT_NE(message.find("--output-dir"), std::string::npos);
}

TEST(MapperScanAccountingTest, RejectedDecodedScansExhaustFiniteBudget) {
  demo::MapperScanAccounting accounting(3);

  for (std::uint64_t scan = 1; scan <= 3; ++scan) {
    ASSERT_TRUE(accounting.CanStartScan());
    EXPECT_EQ(accounting.BeginDecodedScan(), scan);
    accounting.FinishScan(demo::MapperScanOutcome::kRejected);
  }

  EXPECT_FALSE(accounting.CanStartScan());
  EXPECT_EQ(accounting.processed_scans(), 3U);
  EXPECT_EQ(accounting.accepted_scans(), 0U);
  EXPECT_EQ(accounting.rejected_scans(), 3U);
}

TEST(MapperScanAccountingTest, DecodingConsumesBudgetBeforeDisposition) {
  demo::MapperScanAccounting accounting(1);

  EXPECT_EQ(accounting.BeginDecodedScan(), 1U);
  EXPECT_FALSE(accounting.CanStartScan());
  EXPECT_EQ(accounting.processed_scans(), 1U);
  EXPECT_EQ(accounting.accepted_scans(), 0U);
  EXPECT_EQ(accounting.rejected_scans(), 0U);

  accounting.FinishScan(demo::MapperScanOutcome::kRejected);
  EXPECT_FALSE(accounting.CanStartScan());
  EXPECT_EQ(accounting.rejected_scans(), 1U);
}

TEST(MapperScanAccountingTest, UnlimitedModePartitionsProcessedScansByOutcome) {
  demo::MapperScanAccounting accounting(0);

  accounting.BeginDecodedScan();
  accounting.FinishScan(demo::MapperScanOutcome::kAccepted);
  accounting.BeginDecodedScan();
  accounting.FinishScan(demo::MapperScanOutcome::kRejected);

  EXPECT_TRUE(accounting.CanStartScan());
  EXPECT_EQ(accounting.processed_scans(), 2U);
  EXPECT_EQ(accounting.accepted_scans(), 1U);
  EXPECT_EQ(accounting.rejected_scans(), 1U);
}

TEST(MapperScanAccountingTest, ScopeRejectsScanDuringStandardExceptionUnwind) {
  demo::MapperScanAccounting accounting(2);

  try {
    demo::MapperScanScope scan(accounting);
    EXPECT_EQ(scan.scan_number(), 1U);
    throw std::runtime_error("mapper dependency failed");
  } catch (const std::exception&) {
  }

  EXPECT_TRUE(accounting.CanStartScan());
  EXPECT_EQ(accounting.processed_scans(), 1U);
  EXPECT_EQ(accounting.accepted_scans(), 0U);
  EXPECT_EQ(accounting.rejected_scans(), 1U);
}

TEST(MapperScanAccountingTest, ExplicitScopeOutcomeIsNotCountedTwice) {
  demo::MapperScanAccounting accounting(1);
  {
    demo::MapperScanScope scan(accounting);
    scan.Finish(demo::MapperScanOutcome::kAccepted);
  }

  EXPECT_FALSE(accounting.CanStartScan());
  EXPECT_EQ(accounting.processed_scans(), 1U);
  EXPECT_EQ(accounting.accepted_scans(), 1U);
  EXPECT_EQ(accounting.rejected_scans(), 0U);
}
