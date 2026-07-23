#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <png.h>

#include <mujoco_sim/server/server_application.hpp>

namespace {

using namespace std::chrono_literals;

class ScopedScreenshot {
 public:
  ScopedScreenshot() : path_(std::filesystem::current_path() / "screenshot.png") {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  ~ScopedScreenshot() {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

TEST(WbrGuiRenderTest, CapturesNonBlankModelFrame) {
  ScopedScreenshot screenshot;
  mujoco_sim::server::ServerConfig config;
  config.server_name = "wbr_gui_render_test";
  config.model_path = (std::filesystem::path(WBR_LQR_MJCF_DIR) / "scene.xml").string();
  config.initial_keyframe = "home";
  config.headless = false;
  config.log_summary_period_ms = 0;
  mujoco_sim::server::ServerApplication application(std::move(config));
  std::string error;
  ASSERT_TRUE(application.Start(&error)) << error;

  bool request_accepted = false;
  bool captured = false;
  std::thread worker([&] {
    std::this_thread::sleep_for(250ms);
    const auto capture_deadline = std::chrono::steady_clock::now() + 5s;
    while (!captured && std::chrono::steady_clock::now() < capture_deadline) {
      request_accepted = application.RequestScreenshot() || request_accepted;
      std::this_thread::sleep_for(20ms);
      captured = std::filesystem::exists(screenshot.path());
    }
    application.RequestStop(captured ? "screenshot_captured" : "screenshot_timeout");
  });

  const int run_result = application.Run();
  worker.join();
  ASSERT_EQ(run_result, 0);
  EXPECT_FALSE(application.RequestScreenshot());
  ASSERT_TRUE(request_accepted);
  ASSERT_TRUE(captured) << error;

  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(&image, screenshot.path().c_str()), 0) << image.message;
  image.format = PNG_FORMAT_RGB;
  std::vector<unsigned char> pixels(PNG_IMAGE_SIZE(image));
  ASSERT_NE(png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr), 0) << image.message;
  const auto width = image.width;
  const auto height = image.height;
  png_image_free(&image);

  EXPECT_GE(width, 320U);
  EXPECT_GE(height, 240U);
  const auto [minimum, maximum] = std::minmax_element(pixels.begin(), pixels.end());
  ASSERT_NE(minimum, pixels.end());
  EXPECT_GT(static_cast<int>(*maximum) - static_cast<int>(*minimum), 20);
}

}  // namespace
