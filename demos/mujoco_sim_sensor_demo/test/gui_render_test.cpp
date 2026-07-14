#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <png.h>

#include <mujoco_sim/core/simulation_core.hpp>

#include "sim_viewer.hpp"

namespace {

using namespace std::chrono_literals;

class ScopedScreenshot {
 public:
  ScopedScreenshot()
      : path_(std::filesystem::current_path() / "screenshot.png") {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  ~ScopedScreenshot() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

TEST(SensorDemoGuiRenderTest, CapturesNonBlankRoomAndVehicleFrame) {
  ScopedScreenshot screenshot;
  mujoco_sim::core::SimulationCore core;
  std::string error;
  const auto model_load = core.LoadModel(
      (std::filesystem::path(SENSOR_DEMO_MJCF_DIR) / "scene.xml").string());
  ASSERT_TRUE(model_load.response.loaded) << model_load.response.message;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  mujoco_sim::server::SimViewer viewer;
  ASSERT_TRUE(viewer.Initialize(&error)) << error;

  bool loaded = false;
  bool captured = false;
  std::thread worker([&] {
    if (!viewer.QueueLoad(*snapshot, "MuJoCo Sensor Demo", &error)) {
      viewer.RequestClose();
      return;
    }
    const auto load_deadline = std::chrono::steady_clock::now() + 3s;
    while (!viewer.model_loaded() &&
           std::chrono::steady_clock::now() < load_deadline) {
      std::this_thread::sleep_for(5ms);
    }
    loaded = viewer.model_loaded();
    if (loaded && viewer.Synchronize(core, *snapshot, &error)) {
      viewer.RequestScreenshot();
      const auto capture_deadline = std::chrono::steady_clock::now() + 3s;
      while (!std::filesystem::exists(screenshot.path()) &&
             std::chrono::steady_clock::now() < capture_deadline) {
        std::this_thread::sleep_for(5ms);
      }
      captured = std::filesystem::exists(screenshot.path());
    }
    viewer.RequestClose();
  });

  viewer.RenderLoop();
  worker.join();
  ASSERT_TRUE(loaded) << error;
  ASSERT_TRUE(captured) << error;

  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(
                &image, screenshot.path().string().c_str()),
            0)
      << image.message;
  image.format = PNG_FORMAT_RGB;
  std::vector<unsigned char> pixels(PNG_IMAGE_SIZE(image));
  ASSERT_NE(png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr),
            0)
      << image.message;
  const auto width = image.width;
  const auto height = image.height;
  png_image_free(&image);

  EXPECT_GE(width, 320U);
  EXPECT_GE(height, 240U);
  const auto [minimum, maximum] =
      std::minmax_element(pixels.begin(), pixels.end());
  ASSERT_NE(minimum, pixels.end());
  EXPECT_GT(static_cast<int>(*maximum) - static_cast<int>(*minimum), 20);
}

}  // namespace
