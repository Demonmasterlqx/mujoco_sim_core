#include <mujoco_sim/server/server_application.hpp>

#include <ecal/ecal.h>
#include <gtest/gtest.h>

#include <string>
#include <thread>
#include <vector>

namespace mujoco_sim::server {
namespace {

TEST(ServerStopConcurrencyTest, PublishesOneCompleteStopResult) {
  ASSERT_TRUE(eCAL::Initialize("mujoco-sim-server-stop-concurrency-test"));

  ServerConfig config;
  config.server_name = "stop_concurrency_test";
  config.model_path = MUJOCO_SIM_SERVER_TEST_MODEL;
  config.headless = true;
  config.log_summary_period_ms = 0;

  {
    ServerApplication application(std::move(config));
    std::string error;
    ASSERT_TRUE(application.Start(&error)) << error;

    std::vector<std::thread> callers;
    callers.reserve(16);
    for (int index = 0; index < 16; ++index) {
      callers.emplace_back([&application, index] {
        application.RequestStop("concurrent_stop_" + std::to_string(index));
      });
    }
    for (auto& caller : callers) {
      caller.join();
    }

    EXPECT_EQ(application.Run(), 0);
  }

  eCAL::Finalize();
}

}  // namespace
}  // namespace mujoco_sim::server
