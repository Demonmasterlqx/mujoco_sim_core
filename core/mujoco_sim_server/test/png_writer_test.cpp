#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include <png.h>

#include "lodepng.h"

namespace {

class TemporaryPng {
 public:
  TemporaryPng()
      : path_(std::filesystem::path(testing::TempDir()) /
              "mujoco_sim_png_writer_test.png") {
    std::filesystem::remove(path_);
  }

  ~TemporaryPng() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

TEST(PngWriterTest, WritesRgbPixelsReadableByLibpng) {
  TemporaryPng file;
  const std::vector<unsigned char> expected = {
      255, 0, 0, 0, 255, 0,
      0, 0, 255, 255, 255, 255,
  };

  ASSERT_EQ(lodepng::encode(file.path().string(), expected.data(), 2, 2,
                            LCT_RGB),
            0u);

  png_image input{};
  input.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(&input, file.path().c_str()), 0)
      << input.message;
  EXPECT_EQ(input.width, 2u);
  EXPECT_EQ(input.height, 2u);
  input.format = PNG_FORMAT_RGB;
  std::vector<unsigned char> actual(PNG_IMAGE_SIZE(input));
  ASSERT_NE(png_image_finish_read(&input, nullptr, actual.data(), 0, nullptr),
            0)
      << input.message;
  png_image_free(&input);
  EXPECT_EQ(actual, expected);
}

TEST(PngWriterTest, RejectsInvalidInput) {
  TemporaryPng file;
  EXPECT_NE(lodepng::encode(file.path().string(), nullptr, 1, 1, LCT_RGB),
            0u);
  const unsigned char pixel[] = {0, 0, 0};
  EXPECT_NE(lodepng::encode(file.path().string(), pixel, 0, 1, LCT_RGB), 0u);
}

}  // namespace
