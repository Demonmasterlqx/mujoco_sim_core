#include <gtest/gtest.h>

#include <png.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <mujoco_sim/sensor_demo/map_output.hpp>

namespace demo = mujoco_sim::sensor_demo;
namespace {

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    static std::atomic<std::uint64_t> sequence{0};
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("mujoco_sensor_map_output_test_" + std::to_string(ticks) + "_" +
             std::to_string(sequence.fetch_add(1)));
  }

  ~TemporaryDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

  [[nodiscard]] const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

demo::PointCloud MakeMap() {
  demo::PointCloud map;
  for (int column = 0; column < 5; ++column) {
    const double center_x = -1.5 + 0.7 * column;
    const double center_y = (column % 2 == 0) ? -0.6 : 0.8;
    for (int height = 0; height < 16; ++height) {
      for (int facet = 0; facet < 10; ++facet) {
        const double angle = facet * 0.2 * std::numbers::pi;
        map.emplace_back(static_cast<float>(center_x + 0.2 * std::cos(angle)),
                         static_cast<float>(center_y + 0.2 * std::sin(angle)),
                         static_cast<float>(0.1 + 0.1 * height));
      }
    }
  }
  map.width = static_cast<std::uint32_t>(map.size());
  map.height = 1;
  map.is_dense = true;
  return map;
}

demo::MapperTrajectory MakeTrajectory() {
  demo::MapperTrajectory trajectory;
  for (int index = 0; index < 3; ++index) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(0.1 * index, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(0.4 * index, 0.2 * index, 0.0);
    trajectory.push_back({0.1 * index, pose, 0.001 * index});
  }
  return trajectory;
}

std::vector<char> ReadFile(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

demo::PointCloud ReadBinaryPcd(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return {};
  std::string line;
  std::size_t points = 0;
  bool binary = false;
  while (std::getline(input, line)) {
    if (line.rfind("POINTS ", 0) == 0) {
      points = static_cast<std::size_t>(std::stoull(line.substr(7)));
    }
    if (line == "DATA binary") {
      binary = true;
      break;
    }
  }
  demo::PointCloud cloud;
  if (!binary) return cloud;
  cloud.resize(points);
  for (auto& point : cloud) {
    input.read(reinterpret_cast<char*>(&point.x), sizeof(point.x));
    input.read(reinterpret_cast<char*>(&point.y), sizeof(point.y));
    input.read(reinterpret_cast<char*>(&point.z), sizeof(point.z));
    if (!input) return {};
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

}  // namespace

TEST(MapOutputTest, WritesReloadablePcdCsvAndRgbTopDownPng) {
  TemporaryDirectory temporary;
  const demo::PointCloud map = MakeMap();
  const demo::MapperTrajectory trajectory = MakeTrajectory();
  demo::MapOutputConfig config;
  config.image_width = 320;
  config.image_height = 240;
  config.padding_meters = 0.25;

  const demo::MapOutputPaths paths =
      demo::SaveMapOutputs(temporary.path(), map, trajectory, config);
  EXPECT_EQ(paths.pcd, temporary.path() / "map.pcd");
  EXPECT_TRUE(std::filesystem::is_regular_file(paths.pcd));
  EXPECT_TRUE(std::filesystem::is_regular_file(paths.trajectory_csv));
  EXPECT_TRUE(std::filesystem::is_regular_file(paths.top_down_png));

  const demo::PointCloud reloaded = ReadBinaryPcd(paths.pcd);
  ASSERT_EQ(reloaded.size(), map.size());
  EXPECT_FLOAT_EQ(reloaded[23].x, map[23].x);
  EXPECT_FLOAT_EQ(reloaded[23].y, map[23].y);
  EXPECT_FLOAT_EQ(reloaded[23].z, map[23].z);

  std::ifstream csv(paths.trajectory_csv);
  ASSERT_TRUE(csv);
  std::string line;
  ASSERT_TRUE(std::getline(csv, line));
  EXPECT_EQ(line, "timestamp,x,y,z,qx,qy,qz,qw,fitness");
  std::size_t rows = 0;
  while (std::getline(csv, line)) {
    ++rows;
  }
  EXPECT_EQ(rows, trajectory.size());

  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(&image, paths.top_down_png.string().c_str()), 0)
      << image.message;
  EXPECT_EQ(image.width, config.image_width);
  EXPECT_EQ(image.height, config.image_height);
  image.format = PNG_FORMAT_RGB;
  std::vector<std::uint8_t> pixels(PNG_IMAGE_SIZE(image));
  ASSERT_NE(png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr), 0) << image.message;
  png_image_free(&image);
  bool has_rendered_pixel = false;
  for (std::uint8_t channel : pixels) {
    has_rendered_pixel = has_rendered_pixel || channel != 248;
  }
  EXPECT_TRUE(has_rendered_pixel);
}

TEST(MapOutputTest, NonFiniteInputLeavesFilesystemUnchanged) {
  TemporaryDirectory temporary;
  const demo::PointCloud map = MakeMap();
  const demo::MapperTrajectory trajectory = MakeTrajectory();
  const demo::MapOutputPaths paths = demo::SaveMapOutputs(temporary.path(), map, trajectory);
  const std::vector<char> original_pcd = ReadFile(paths.pcd);
  const std::vector<char> original_csv = ReadFile(paths.trajectory_csv);
  const std::vector<char> original_png = ReadFile(paths.top_down_png);

  demo::PointCloud invalid_map = map;
  invalid_map[4].x = std::numeric_limits<float>::infinity();
  EXPECT_THROW(demo::SaveMapOutputs(temporary.path(), invalid_map, trajectory),
               demo::MapOutputError);
  EXPECT_EQ(ReadFile(paths.pcd), original_pcd);
  EXPECT_EQ(ReadFile(paths.trajectory_csv), original_csv);
  EXPECT_EQ(ReadFile(paths.top_down_png), original_png);

  const std::filesystem::path new_directory = temporary.path() / "new";
  invalid_map[4].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(demo::SaveMapOutputs(new_directory, invalid_map, trajectory), demo::MapOutputError);
  EXPECT_FALSE(std::filesystem::exists(new_directory));
}
