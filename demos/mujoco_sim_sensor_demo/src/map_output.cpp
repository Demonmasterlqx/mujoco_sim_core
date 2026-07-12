#include <mujoco_sim/sensor_demo/map_output.hpp>

#include <png.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace mujoco_sim::sensor_demo {
namespace {

constexpr std::size_t kMinimumImageDimension = 32;
constexpr std::size_t kMaximumImageDimension = 16384;

struct Bounds {
  double minimum_x = std::numeric_limits<double>::max();
  double maximum_x = std::numeric_limits<double>::lowest();
  double minimum_y = std::numeric_limits<double>::max();
  double maximum_y = std::numeric_limits<double>::lowest();
  double minimum_z = std::numeric_limits<double>::max();
  double maximum_z = std::numeric_limits<double>::lowest();

  void Include(double x, double y, double z) {
    minimum_x = std::min(minimum_x, x);
    maximum_x = std::max(maximum_x, x);
    minimum_y = std::min(minimum_y, y);
    maximum_y = std::max(maximum_y, y);
    minimum_z = std::min(minimum_z, z);
    maximum_z = std::max(maximum_z, z);
  }
};

struct Projection {
  double scale = 1.0;
  double left = 0.0;
  double bottom = 0.0;
  double offset_x = 0.0;
  double offset_y = 0.0;
  std::size_t width = 0;
  std::size_t height = 0;

  [[nodiscard]] std::pair<int, int> Pixel(double x, double y) const {
    const double image_x = offset_x + (x - left) * scale;
    const double image_y = offset_y + (y - bottom) * scale;
    const int pixel_x = static_cast<int>(std::llround(image_x));
    const int pixel_y = static_cast<int>(height - 1) -
                        static_cast<int>(std::llround(image_y));
    return {pixel_x, pixel_y};
  }
};

struct StagedFile {
  std::filesystem::path final_path;
  std::filesystem::path temporary_path;
  std::filesystem::path backup_path;
  bool backed_up = false;
  bool installed = false;
};

class StagingCleanup {
 public:
  explicit StagingCleanup(std::vector<StagedFile>* files) : files_(files) {}
  ~StagingCleanup() {
    std::error_code ignored;
    for (const StagedFile& file : *files_) {
      std::filesystem::remove(file.temporary_path, ignored);
    }
  }

 private:
  std::vector<StagedFile>* files_;
};

void RequireFinite(double value, const std::string& field) {
  if (!std::isfinite(value)) {
    throw MapOutputError("non-finite " + field);
  }
}

void ValidateLeafFilename(const std::string& filename,
                          const std::string& field) {
  const std::filesystem::path path(filename);
  if (filename.empty() || path.is_absolute() || path.has_parent_path() ||
      path.filename() != path || path == "." || path == "..") {
    throw MapOutputError(field + " must be a non-empty leaf filename");
  }
}

Bounds ValidateInputs(const std::filesystem::path& output_directory,
                      const PointCloud& map,
                      const MapperTrajectory& trajectory,
                      const MapOutputConfig& config) {
  if (output_directory.empty()) {
    throw MapOutputError("output directory is empty");
  }
  ValidateLeafFilename(config.pcd_filename, "pcd_filename");
  ValidateLeafFilename(config.trajectory_filename, "trajectory_filename");
  ValidateLeafFilename(config.top_down_png_filename,
                       "top_down_png_filename");
  if (config.pcd_filename == config.trajectory_filename ||
      config.pcd_filename == config.top_down_png_filename ||
      config.trajectory_filename == config.top_down_png_filename) {
    throw MapOutputError("output filenames must be distinct");
  }
  RequireFinite(config.padding_meters, "padding_meters");
  if (config.padding_meters < 0.0 ||
      config.image_width < kMinimumImageDimension ||
      config.image_height < kMinimumImageDimension ||
      config.image_width > kMaximumImageDimension ||
      config.image_height > kMaximumImageDimension ||
      config.image_width >
          std::numeric_limits<std::size_t>::max() / config.image_height / 3) {
    throw MapOutputError("invalid top-down image configuration");
  }
  if (map.empty()) {
    throw MapOutputError("cannot save an empty map");
  }
  if (trajectory.empty()) {
    throw MapOutputError("cannot save an empty trajectory");
  }

  Bounds bounds;
  for (std::size_t index = 0; index < map.size(); ++index) {
    const pcl::PointXYZ& point = map[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      throw MapOutputError("map contains a non-finite point at index " +
                           std::to_string(index));
    }
    bounds.Include(point.x, point.y, point.z);
  }

  double previous_timestamp = 0.0;
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    const MapperTrajectoryPoint& point = trajectory[index];
    RequireFinite(point.timestamp,
                  "trajectory[" + std::to_string(index) + "].timestamp");
    RequireFinite(point.fitness,
                  "trajectory[" + std::to_string(index) + "].fitness");
    if (index > 0 && point.timestamp <= previous_timestamp) {
      throw MapOutputError("trajectory timestamps must increase strictly");
    }
    if (!point.pose.matrix().allFinite()) {
      throw MapOutputError("trajectory contains a non-finite pose at index " +
                           std::to_string(index));
    }
    const Eigen::Matrix3d rotation = point.pose.rotation();
    if (!rotation.transpose().isApprox(rotation.inverse(), 1e-6) ||
        std::abs(rotation.determinant() - 1.0) > 1e-6) {
      throw MapOutputError("trajectory contains a non-rigid pose at index " +
                           std::to_string(index));
    }
    const Eigen::Vector3d translation = point.pose.translation();
    bounds.Include(translation.x(), translation.y(), translation.z());
    previous_timestamp = point.timestamp;
  }
  return bounds;
}

Projection MakeProjection(Bounds bounds, const MapOutputConfig& config) {
  if (bounds.maximum_x - bounds.minimum_x < 1e-9) {
    bounds.minimum_x -= 0.5;
    bounds.maximum_x += 0.5;
  }
  if (bounds.maximum_y - bounds.minimum_y < 1e-9) {
    bounds.minimum_y -= 0.5;
    bounds.maximum_y += 0.5;
  }
  bounds.minimum_x -= config.padding_meters;
  bounds.maximum_x += config.padding_meters;
  bounds.minimum_y -= config.padding_meters;
  bounds.maximum_y += config.padding_meters;

  const double range_x = bounds.maximum_x - bounds.minimum_x;
  const double range_y = bounds.maximum_y - bounds.minimum_y;
  if (!std::isfinite(bounds.minimum_x) ||
      !std::isfinite(bounds.maximum_x) ||
      !std::isfinite(bounds.minimum_y) ||
      !std::isfinite(bounds.maximum_y) || !std::isfinite(range_x) ||
      !std::isfinite(range_y) || range_x <= 0.0 || range_y <= 0.0) {
    throw MapOutputError("map bounds cannot be represented in the PNG");
  }
  const double available_width = static_cast<double>(config.image_width - 1);
  const double available_height = static_cast<double>(config.image_height - 1);
  const double scale =
      std::min(available_width / range_x, available_height / range_y);
  if (!std::isfinite(scale) || scale <= 0.0) {
    throw MapOutputError("map projection scale is invalid");
  }
  const double rendered_width = range_x * scale;
  const double rendered_height = range_y * scale;
  return {scale,
          bounds.minimum_x,
          bounds.minimum_y,
          0.5 * (available_width - rendered_width),
          0.5 * (available_height - rendered_height),
          config.image_width,
          config.image_height};
}

void SetPixel(std::vector<std::uint8_t>& pixels, std::size_t width,
              std::size_t height, int x, int y,
              const std::array<std::uint8_t, 3>& color) {
  if (x < 0 || y < 0 || x >= static_cast<int>(width) ||
      y >= static_cast<int>(height)) {
    return;
  }
  const std::size_t offset =
      3 * (static_cast<std::size_t>(y) * width +
           static_cast<std::size_t>(x));
  std::copy(color.begin(), color.end(), pixels.begin() + offset);
}

void DrawLine(std::vector<std::uint8_t>& pixels, std::size_t width,
              std::size_t height, std::pair<int, int> start,
              std::pair<int, int> end,
              const std::array<std::uint8_t, 3>& color) {
  int x = start.first;
  int y = start.second;
  const int delta_x = std::abs(end.first - start.first);
  const int step_x = start.first < end.first ? 1 : -1;
  const int delta_y = -std::abs(end.second - start.second);
  const int step_y = start.second < end.second ? 1 : -1;
  int error = delta_x + delta_y;
  while (true) {
    SetPixel(pixels, width, height, x, y, color);
    SetPixel(pixels, width, height, x + 1, y, color);
    SetPixel(pixels, width, height, x, y + 1, color);
    if (x == end.first && y == end.second) {
      break;
    }
    const int twice_error = 2 * error;
    if (twice_error >= delta_y) {
      error += delta_y;
      x += step_x;
    }
    if (twice_error <= delta_x) {
      error += delta_x;
      y += step_y;
    }
  }
}

std::vector<std::uint8_t> RenderTopDown(
    const PointCloud& map, const MapperTrajectory& trajectory,
    const Bounds& bounds, const MapOutputConfig& config) {
  std::vector<std::uint8_t> pixels(config.image_width * config.image_height * 3,
                                   248);
  const Projection projection = MakeProjection(bounds, config);
  const double raw_z_range = bounds.maximum_z - bounds.minimum_z;
  if (!std::isfinite(raw_z_range) || raw_z_range < 0.0) {
    throw MapOutputError("map height range cannot be represented in the PNG");
  }
  const double z_range = std::max(1e-9, raw_z_range);
  for (const pcl::PointXYZ& point : map) {
    const double height_fraction =
        std::clamp((point.z - bounds.minimum_z) / z_range, 0.0, 1.0);
    const std::array<std::uint8_t, 3> color{
        static_cast<std::uint8_t>(35.0 + 194.0 * height_fraction),
        static_cast<std::uint8_t>(92.0 + 32.0 * height_fraction),
        static_cast<std::uint8_t>(145.0 - 100.0 * height_fraction)};
    const auto pixel = projection.Pixel(point.x, point.y);
    SetPixel(pixels, config.image_width, config.image_height, pixel.first,
             pixel.second, color);
  }

  constexpr std::array<std::uint8_t, 3> kTrajectoryColor{198, 42, 48};
  for (std::size_t index = 1; index < trajectory.size(); ++index) {
    const Eigen::Vector3d previous = trajectory[index - 1].pose.translation();
    const Eigen::Vector3d current = trajectory[index].pose.translation();
    DrawLine(pixels, config.image_width, config.image_height,
             projection.Pixel(previous.x(), previous.y()),
             projection.Pixel(current.x(), current.y()), kTrajectoryColor);
  }
  for (const MapperTrajectoryPoint& point : trajectory) {
    const Eigen::Vector3d translation = point.pose.translation();
    const auto pixel = projection.Pixel(translation.x(), translation.y());
    for (int y_offset = -2; y_offset <= 2; ++y_offset) {
      for (int x_offset = -2; x_offset <= 2; ++x_offset) {
        if (x_offset * x_offset + y_offset * y_offset <= 4) {
          SetPixel(pixels, config.image_width, config.image_height,
                   pixel.first + x_offset, pixel.second + y_offset,
                   kTrajectoryColor);
        }
      }
    }
  }
  return pixels;
}

std::string BuildTrajectoryCsv(const MapperTrajectory& trajectory) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "timestamp,x,y,z,qx,qy,qz,qw,fitness\n";
  output << std::setprecision(17);
  for (const MapperTrajectoryPoint& point : trajectory) {
    Eigen::Quaterniond orientation(point.pose.rotation());
    orientation.normalize();
    const Eigen::Vector3d translation = point.pose.translation();
    output << point.timestamp << ',' << translation.x() << ','
           << translation.y() << ',' << translation.z() << ','
           << orientation.x() << ',' << orientation.y() << ','
           << orientation.z() << ',' << orientation.w() << ','
           << point.fitness << '\n';
  }
  return output.str();
}

void WriteTextFile(const std::filesystem::path& path,
                   const std::string& contents) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw MapOutputError("failed to open trajectory CSV: " + path.string());
  }
  output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  output.close();
  if (!output) {
    throw MapOutputError("failed to write trajectory CSV: " + path.string());
  }
}

void WritePcd(const std::filesystem::path& path, const PointCloud& map) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw MapOutputError("failed to open PCD: " + path.string());
  }
  output.imbue(std::locale::classic());
  output << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z\n"
         << "SIZE 4 4 4\n"
         << "TYPE F F F\n"
         << "COUNT 1 1 1\n"
         << "WIDTH " << map.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << map.size() << "\n"
         << "DATA binary\n";
  for (const pcl::PointXYZ& point : map) {
    output.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
    output.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
    output.write(reinterpret_cast<const char*>(&point.z), sizeof(point.z));
  }
  output.close();
  if (!output) {
    throw MapOutputError("failed to write PCD: " + path.string());
  }
}

void WritePng(const std::filesystem::path& path,
              const std::vector<std::uint8_t>& pixels,
              const MapOutputConfig& config) {
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  image.width = static_cast<png_uint_32>(config.image_width);
  image.height = static_cast<png_uint_32>(config.image_height);
  image.format = PNG_FORMAT_RGB;
  if (png_image_write_to_file(&image, path.string().c_str(), 0, pixels.data(),
                              0, nullptr) == 0) {
    const std::string message = image.message;
    png_image_free(&image);
    throw MapOutputError("failed to write top-down PNG: " + message);
  }
  png_image_free(&image);
}

std::string UniqueSuffix() {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  return "." + std::to_string(ticks) + "." +
         std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
}

void CommitStagedFiles(std::vector<StagedFile>& files) {
  try {
    for (StagedFile& file : files) {
      if (std::filesystem::exists(file.final_path)) {
        std::filesystem::rename(file.final_path, file.backup_path);
        file.backed_up = true;
      }
    }
    for (StagedFile& file : files) {
      std::filesystem::rename(file.temporary_path, file.final_path);
      file.installed = true;
    }
    for (StagedFile& file : files) {
      if (file.backed_up) {
        std::error_code ignored;
        if (std::filesystem::remove(file.backup_path, ignored)) {
          file.backed_up = false;
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& error) {
    std::error_code ignored;
    for (auto file = files.rbegin(); file != files.rend(); ++file) {
      if (file->installed) {
        std::filesystem::remove(file->final_path, ignored);
        file->installed = false;
      }
      if (file->backed_up) {
        std::filesystem::rename(file->backup_path, file->final_path, ignored);
        file->backed_up = false;
      }
    }
    throw MapOutputError("failed to commit map outputs: " +
                         std::string(error.what()));
  }
}

void ValidateDestinations(const std::filesystem::path& output_directory,
                          const MapOutputPaths& paths) {
  std::error_code error;
  const bool output_exists = std::filesystem::exists(output_directory, error);
  if (error) {
    throw MapOutputError("failed to inspect output directory: " +
                         error.message());
  }
  if (output_exists &&
      !std::filesystem::is_directory(output_directory, error)) {
    if (error) {
      throw MapOutputError("failed to inspect output directory: " +
                           error.message());
    }
    throw MapOutputError("output path is not a directory: " +
                         output_directory.string());
  }

  const std::array<std::filesystem::path, 3> final_paths{
      paths.pcd, paths.trajectory_csv, paths.top_down_png};
  for (const std::filesystem::path& path : final_paths) {
    error.clear();
    const bool exists = std::filesystem::exists(path, error);
    if (error) {
      throw MapOutputError("failed to inspect output destination: " +
                           error.message());
    }
    if (exists && !std::filesystem::is_regular_file(path, error)) {
      if (error) {
        throw MapOutputError("failed to inspect output destination: " +
                             error.message());
      }
      throw MapOutputError("output destination is not a regular file: " +
                           path.string());
    }
  }
}

}  // namespace

MapOutputPaths SaveMapOutputs(
    const std::filesystem::path& output_directory, const PointCloud& map,
    const MapperTrajectory& trajectory, const MapOutputConfig& config) {
  const Bounds bounds =
      ValidateInputs(output_directory, map, trajectory, config);
  const std::string trajectory_csv = BuildTrajectoryCsv(trajectory);
  const std::vector<std::uint8_t> pixels =
      RenderTopDown(map, trajectory, bounds, config);

  const MapOutputPaths paths{output_directory / config.pcd_filename,
                             output_directory / config.trajectory_filename,
                             output_directory /
                                 config.top_down_png_filename};
  ValidateDestinations(output_directory, paths);
  const std::string suffix = UniqueSuffix();
  std::vector<StagedFile> staged{
      {paths.pcd, paths.pcd.string() + ".tmp" + suffix,
       paths.pcd.string() + ".bak" + suffix},
      {paths.trajectory_csv,
       paths.trajectory_csv.string() + ".tmp" + suffix,
       paths.trajectory_csv.string() + ".bak" + suffix},
      {paths.top_down_png, paths.top_down_png.string() + ".tmp" + suffix,
       paths.top_down_png.string() + ".bak" + suffix}};
  StagingCleanup cleanup(&staged);

  try {
    std::filesystem::create_directories(output_directory);
    WritePcd(staged[0].temporary_path, map);
    WriteTextFile(staged[1].temporary_path, trajectory_csv);
    WritePng(staged[2].temporary_path, pixels, config);
    CommitStagedFiles(staged);
  } catch (const MapOutputError&) {
    throw;
  } catch (const std::exception& error) {
    throw MapOutputError("failed to save map outputs: " +
                         std::string(error.what()));
  }

  return paths;
}

MapOutputPaths SaveMapOutputs(const std::filesystem::path& output_directory,
                              const SimpleIcpMapper& mapper,
                              const MapOutputConfig& config) {
  const PointCloud::ConstPtr map = mapper.global_map();
  if (!map) {
    throw MapOutputError("mapper returned a null global map");
  }
  return SaveMapOutputs(output_directory, *map, mapper.trajectory(), config);
}

}  // namespace mujoco_sim::sensor_demo
