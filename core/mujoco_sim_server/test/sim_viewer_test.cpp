#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include <mujoco_sim/core/simulation_core.hpp>

#include "sim_viewer.hpp"

namespace {

TEST(SimViewerTest, LoadsSynchronizesAndClosesWindow) {
  mujoco_sim::core::SimulationCore core;
  std::string error;
  const auto model_load = core.LoadModel(MUJOCO_SIM_SERVER_TEST_MODEL);
  ASSERT_TRUE(model_load.response.loaded) << model_load.response.message;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  mujoco_sim::server::SimViewer viewer;
  ASSERT_TRUE(viewer.Initialize(&error)) << error;

  bool loaded = false;
  std::thread worker([&]() {
    if (!viewer.QueueLoad(*snapshot, MUJOCO_SIM_SERVER_TEST_MODEL,
                          &error)) {
      viewer.RequestClose();
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(2);
    while (!viewer.model_loaded() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    loaded = viewer.model_loaded();
    if (loaded) {
      loaded = viewer.Synchronize(core, *snapshot, &error);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    viewer.RequestClose();
  });

  viewer.RenderLoop();
  worker.join();
  EXPECT_TRUE(loaded) << error;
  EXPECT_TRUE(viewer.close_requested());
}

TEST(SimViewerTest, ClosesWithPendingModelLoad) {
  mujoco_sim::core::SimulationCore core;
  std::string error;
  const auto loaded = core.LoadModel(MUJOCO_SIM_SERVER_TEST_MODEL);
  ASSERT_TRUE(loaded.response.loaded) << loaded.response.message;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  mujoco_sim::server::SimViewer viewer;
  ASSERT_TRUE(viewer.Initialize(&error)) << error;
  ASSERT_TRUE(viewer.QueueLoad(*snapshot, MUJOCO_SIM_SERVER_TEST_MODEL,
                               &error))
      << error;
  viewer.RequestClose();
  viewer.RenderLoop();
  EXPECT_TRUE(viewer.close_requested());
}

}  // namespace
