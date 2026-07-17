#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <ecal/ecal.h>
#include <ecal/pubsub/publisher.h>
#include <ecal/service/server.h>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/server/server_config.hpp>

#include "sensor_runtime.hpp"

namespace {

using namespace std::chrono_literals;

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

class EcalScope {
 public:
  explicit EcalScope(const std::string& name) : initialized_(eCAL::Initialize(name)) {}

  ~EcalScope() {
    if (initialized_) eCAL::Finalize();
  }

  bool initialized() const { return initialized_; }

 private:
  bool initialized_ = false;
};

mujoco_sim::server::ServerConfig MakeConfig(const std::string& server_name) {
  mujoco_sim::server::ServerConfig config;
  config.server_name = server_name;
  config.model_path = MUJOCO_SIM_SENSOR_RUNTIME_ECAL_MODEL;
  config.headless = true;
  config.render_backend = "auto";
  mujoco_sim::server::ServerConfig::SensorStream stream;
  stream.name = "lidar";
  stream.kind = "lidar_pointcloud";
  stream.source = "lidar_site";
  stream.frame_id = "lidar";
  stream.rate_hz = 100.0;
  stream.horizontal_samples = 72;
  stream.vertical_samples = 4;
  stream.vertical_fov_min_deg = -10.0;
  stream.vertical_fov_max_deg = 10.0;
  stream.min_range_m = 0.05;
  stream.max_range_m = 8.0;
  stream.queue_mode = "blocking";
  stream.queue_depth = 1;
  config.sensor_streams = {std::move(stream)};
  return config;
}

TEST(SensorRuntimeEcalTest, CaptureErrorReachesStreamClient) {
  EcalScope ecal("mujoco-sim-runtime-error-test");
  ASSERT_TRUE(ecal.initialized());
  const std::string server_name =
      "runtime-error-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  auto config = MakeConfig(server_name);
  mujoco_sim::core::SimulationCore core(mujoco_sim::core::CoreOptions{server_name});
  std::string error;
  const auto loaded = core.LoadModel(config.model_path);
  ASSERT_TRUE(loaded.response.loaded) << loaded.response.message;

  mujoco_sim::server::SensorRuntimeTestHooks hooks;
  hooks.before_capture = [](const std::string&) {
    throw std::runtime_error("injected runtime capture failure");
  };
  mujoco_sim::server::SensorRuntime runtime(std::move(hooks));
  ASSERT_TRUE(runtime.Initialize(core, config, &error)) << error;

  eCAL::CServiceServer registry(mujoco_sim::protocol::RegistryServiceName(server_name));
  ASSERT_TRUE(registry.SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kGetSensorStreamsMethod),
      [&runtime](const eCAL::SServiceMethodInformation&, const std::string& request_bytes,
                 std::string& response_bytes) {
        mujoco_sim::v4::GetSensorStreamsRequest request;
        if (!request.ParseFromString(request_bytes)) return 1;
        const auto response = runtime.GetCatalog(request);
        return response.SerializeToString(&response_bytes) ? 0 : 1;
      }));
  eCAL::CPublisher status_publisher(mujoco_sim::protocol::StatusTopicName(server_name));

  mujoco_sim::adapter_sdk::SensorStreamOptions options;
  options.server_name = server_name;
  options.stream_names = {"lidar"};
  options.service_timeout_ms = 5000;
  mujoco_sim::adapter_sdk::SensorStreamClient client(std::move(options));
  ASSERT_TRUE(client.Start(&error)) << error;
  const auto discovery_deadline = std::chrono::steady_clock::now() + 3s;
  while (status_publisher.GetSubscriberCount() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(status_publisher.GetSubscriberCount(), 0U);

  mujoco_sim::adapter_sdk::OwnedStreamSample sample;
  auto waiting = std::async(std::launch::async,
                            [&] { return client.WaitForSample("lidar", 5s, &sample, &error); });
  const auto error_deadline = std::chrono::steady_clock::now() + 4s;
  while (waiting.wait_for(0ms) != std::future_status::ready &&
         std::chrono::steady_clock::now() < error_deadline) {
    ASSERT_TRUE(core.Step().step_completed.has_value());
    runtime.OnStepCompleted(core);
    runtime.CompleteBlockingEnqueues();
    mujoco_sim::v4::ServerStatus status;
    status.set_epoch(core.state().epoch);
    status.set_tick_id(core.state().current_tick_id);
    status.set_simulation_time_s(core.state().simulation_time_s);
    status.set_state(mujoco_sim::v4::RUNNING);
    runtime.AppendStatus(&status);
    status_publisher.Send(status.SerializeAsString());
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_EQ(waiting.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(waiting.get(), mujoco_sim::adapter_sdk::SampleWaitResult::kError);
  EXPECT_NE(error.find("injected runtime capture failure"), std::string::npos);
  EXPECT_NE(client.stream_diagnostic("lidar").find("injected runtime capture failure"),
            std::string::npos);
  client.Stop();
  runtime.Stop();
}

}  // namespace
