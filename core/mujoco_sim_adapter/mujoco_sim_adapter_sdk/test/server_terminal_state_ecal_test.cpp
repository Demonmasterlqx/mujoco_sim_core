#include <chrono>
#include <future>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/server/server_application.hpp>

namespace mujoco_sim::adapter_sdk {
namespace {

using namespace std::chrono_literals;

TEST(ServerTerminalStateEcalTest, StoppedServerWakesAllClientRoles) {
  server::ServerConfig config;
  config.server_name =
      "sdk_terminal_" +
      std::to_string(std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count());
  config.model_path = MUJOCO_SIM_SDK_TERMINAL_MODEL;
  config.headless = true;
  config.log_level = "off";
  config.status_period_ms = 10;
  server::ServerConfig::SensorStream stream;
  stream.name = "lidar";
  stream.kind = "lidar_pointcloud";
  stream.source = "lidar_site";
  stream.frame_id = "lidar";
  stream.rate_hz = 20.0;
  stream.horizontal_samples = 16;
  stream.vertical_samples = 2;
  stream.queue_mode = "latest";
  stream.queue_depth = 1;
  config.sensor_streams = {stream};

  server::ServerApplication server(config);
  std::string error;
  ASSERT_TRUE(server.Start(&error)) << error;

  AdapterOptions controller_options;
  controller_options.server_name = config.server_name;
  controller_options.adapter_id = "controller";
  controller_options.observation_names = {"hinge_pos"};
  controller_options.command_names = {"hinge_motor"};
  controller_options.service_timeout_ms = 5000;
  SyncAdapterClient controller(controller_options);
  ASSERT_TRUE(controller.Start(&error)) << error;
  ASSERT_TRUE(controller.Activate(&error)) << error;

  ObserverOptions observer_options;
  observer_options.server_name = config.server_name;
  observer_options.observer_id = "observer";
  observer_options.observation_names = {"imu_quat", "imu_gyro"};
  observer_options.service_timeout_ms = 5000;
  observer_options.keepalive_period_ms = 20;
  ObserverClient observer(observer_options);
  ASSERT_TRUE(observer.Start(&error)) << error;
  ASSERT_TRUE(observer.Activate(&error)) << error;

  SensorStreamOptions stream_options;
  stream_options.server_name = config.server_name;
  stream_options.stream_names = {"lidar"};
  stream_options.service_timeout_ms = 5000;
  SensorStreamClient streams(stream_options);
  ASSERT_TRUE(streams.Start(&error)) << error;

  mujoco_sim::v2::Tick tick;
  std::string tick_error;
  auto tick_wait = std::async(std::launch::async, [&] {
    return controller.WaitForTick(std::chrono::milliseconds::max(), &tick,
                                  &tick_error);
  });
  mujoco_sim::v2::ObservationFrame observation;
  std::string observation_error;
  auto observation_wait = std::async(std::launch::async, [&] {
    return observer.WaitForObservation(std::chrono::milliseconds::max(),
                                       &observation, &observation_error);
  });
  OwnedStreamSample sample;
  std::string sample_error;
  auto sample_wait = std::async(std::launch::async, [&] {
    return streams.WaitForSample("lidar", std::chrono::milliseconds::max(),
                                 &sample, &sample_error);
  });

  std::this_thread::sleep_for(300ms);
  server.RequestStop();
  EXPECT_EQ(server.Run(), 0);

  ASSERT_EQ(tick_wait.wait_for(3s), std::future_status::ready);
  ASSERT_EQ(observation_wait.wait_for(3s), std::future_status::ready);
  ASSERT_EQ(sample_wait.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(tick_wait.get(), TickWaitResult::kStopped);
  EXPECT_EQ(observation_wait.get(), ObservationWaitResult::kStopped);
  EXPECT_EQ(sample_wait.get(), SampleWaitResult::kStopped);
  EXPECT_TRUE(tick_error.empty());
  EXPECT_TRUE(observation_error.empty());
  EXPECT_TRUE(sample_error.empty());
}

}  // namespace
}  // namespace mujoco_sim::adapter_sdk
