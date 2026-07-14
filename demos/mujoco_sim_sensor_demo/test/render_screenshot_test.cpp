#include <gtest/gtest.h>

#include <png.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/sensor/offscreen_renderer.hpp>
#include <mujoco_sim/sensor_demo/stream_processing.hpp>

namespace demo = mujoco_sim::sensor_demo;
namespace sensor = mujoco_sim::sensor;

TEST(RenderScreenshotTest, EglHeadlessRenderProducesVerifiablePng) {
  mujoco_sim::core::SimulationCore core;
  std::string error;
  const auto model_load = core.LoadModel(
      (std::filesystem::path(SENSOR_DEMO_MJCF_DIR) / "scene.xml").string());
  ASSERT_TRUE(model_load.response.loaded) << model_load.response.message;
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  sensor::CameraProfile profile;
  profile.camera_name = "front_camera";
  profile.width = 160;
  profile.height = 120;
  profile.rate_hz = 10.0;
  profile.frame_id = "front_camera_optical";
  profile.outputs = {{mujoco_sim::v2::RGB8, "test_rgb"}};
  sensor::OffscreenRendererOptions options;
  options.backend = sensor::BackendPreference::kEglOnly;
  auto renderer = sensor::OffscreenRenderer::Create(*snapshot, profile,
                                                     options, &error);
  ASSERT_NE(renderer, nullptr) << error;

  sensor::CaptureMetadata metadata;
  metadata.header.set_server_name("render_test");
  metadata.header.set_epoch(1);
  metadata.header.set_tick_id(1);
  metadata.header.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  metadata.header.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  metadata.header.set_simulation_time_s(0.0);
  metadata.stream_sequence = 1;
  metadata.capture_id = 1;
  std::vector<sensor::RawFrame> frames;
  ASSERT_TRUE(renderer->Capture(*snapshot, metadata, &frames, &error))
      << error;
  ASSERT_EQ(frames.size(), 1U);

  mujoco_sim::adapter_sdk::OwnedStreamSample sample;
  sample.header = frames[0].header;
  sample.payload = frames[0].payload;
  demo::StreamValidationSummary summary;
  ASSERT_TRUE(demo::ValidateCameraSample(
      sample, renderer->descriptors()[0], demo::CameraStreamKind::kRgb,
      &summary, &error))
      << error;
  EXPECT_GT(summary.populated_values, 0U);

  static std::atomic<std::uint64_t> sequence{0};
  const auto output = std::filesystem::temp_directory_path() /
                      ("sensor_demo_egl_" +
                       std::to_string(sequence.fetch_add(1)) + ".png");
  ASSERT_TRUE(demo::SaveRgbScreenshot(output, sample, &error)) << error;
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(&image, output.string().c_str()), 0)
      << image.message;
  image.format = PNG_FORMAT_RGB;
  std::vector<std::uint8_t> pixels(PNG_IMAGE_SIZE(image));
  ASSERT_NE(png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr),
            0)
      << image.message;
  png_image_free(&image);
  const auto [minimum, maximum] =
      std::minmax_element(pixels.begin(), pixels.end());
  EXPECT_LT(*minimum, *maximum);
  EXPECT_GT(*maximum, 20U);
  std::error_code ignored;
  std::filesystem::remove(output, ignored);
}
