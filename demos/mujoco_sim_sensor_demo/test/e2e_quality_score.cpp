#include <png.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using MapPoint = std::array<float, 3>;

constexpr std::array<std::array<double, 2>, 5> kColumnCentersWorld{{
    {-3.05, -1.15},
    {-1.35, 1.20},
    {0.55, -0.45},
    {2.25, 1.25},
    {3.15, -1.05},
}};

struct Bounds {
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  void Include(double x, double y) {
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }
};

[[maybe_unused]] bool ReadPcd(const std::filesystem::path& path, std::size_t* point_count,
                              Bounds* bounds, std::vector<MapPoint>* points, std::string* error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    *error = "failed to open " + path.string();
    return false;
  }
  std::string line;
  std::size_t declared_points = 0;
  bool binary = false;
  while (std::getline(input, line)) {
    if (line.rfind("POINTS ", 0) == 0) {
      try {
        declared_points = static_cast<std::size_t>(std::stoull(line.substr(7)));
      } catch (const std::exception&) {
        *error = "invalid PCD POINTS field";
        return false;
      }
    }
    if (line == "DATA binary") {
      binary = true;
      break;
    }
  }
  if (!binary || declared_points == 0) {
    *error = "PCD is not a non-empty binary XYZ file";
    return false;
  }
  Bounds next;
  std::vector<MapPoint> parsed;
  parsed.reserve(declared_points);
  for (std::size_t index = 0; index < declared_points; ++index) {
    MapPoint point{};
    input.read(reinterpret_cast<char*>(point.data()), static_cast<std::streamsize>(sizeof(point)));
    if (!input || !std::isfinite(point[0]) || !std::isfinite(point[1]) ||
        !std::isfinite(point[2])) {
      *error = "PCD contains a truncated or non-finite point";
      return false;
    }
    next.Include(point[0], point[1]);
    parsed.push_back(point);
  }
  char extra = 0;
  if (input.read(&extra, 1)) {
    *error = "PCD has trailing payload bytes";
    return false;
  }
  *point_count = declared_points;
  *bounds = next;
  *points = std::move(parsed);
  return true;
}

bool ParseCsvRow(const std::string& line, std::array<double, 9>* values) {
  std::istringstream input(line);
  std::string token;
  for (double& value : *values) {
    if (!std::getline(input, token, ',')) return false;
    try {
      std::size_t consumed = 0;
      value = std::stod(token, &consumed);
      if (consumed != token.size() || !std::isfinite(value)) return false;
    } catch (const std::exception&) {
      return false;
    }
  }
  return !std::getline(input, token, ',');
}

[[maybe_unused]] bool ReadTrajectory(const std::filesystem::path& path, std::size_t* pose_count,
                                     double* path_length, double* closure_error, Bounds* bounds,
                                     std::vector<std::array<double, 9>>* trajectory,
                                     std::string* error) {
  std::ifstream input(path);
  if (!input) {
    *error = "failed to open " + path.string();
    return false;
  }
  std::string line;
  if (!std::getline(input, line) || line != "timestamp,x,y,z,qx,qy,qz,qw,fitness") {
    *error = "trajectory CSV header is invalid";
    return false;
  }
  std::vector<std::array<double, 9>> rows;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::array<double, 9> row{};
    if (!ParseCsvRow(line, &row)) {
      *error = "trajectory CSV row is invalid";
      return false;
    }
    if (!rows.empty() && row[0] <= rows.back()[0]) {
      *error = "trajectory timestamps are not strictly increasing";
      return false;
    }
    rows.push_back(row);
  }
  if (rows.empty()) {
    *error = "trajectory is empty";
    return false;
  }
  double length = 0.0;
  Bounds next;
  for (std::size_t index = 0; index < rows.size(); ++index) {
    next.Include(rows[index][1], rows[index][2]);
    if (index > 0) {
      length +=
          std::hypot(rows[index][1] - rows[index - 1][1], rows[index][2] - rows[index - 1][2]);
    }
  }
  *pose_count = rows.size();
  *path_length = length;
  *closure_error = std::hypot(rows.back()[1] - rows.front()[1], rows.back()[2] - rows.front()[2]);
  *bounds = next;
  *trajectory = std::move(rows);
  return true;
}

struct TruthRow {
  double timestamp = 0.0;
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{};
};

[[maybe_unused]] bool ReadTruth(const std::filesystem::path& path, std::vector<TruthRow>* rows,
                                std::string* error) {
  std::ifstream input(path);
  if (!input) {
    *error = "failed to open " + path.string();
    return false;
  }
  std::string line;
  if (!std::getline(input, line) || line != "timestamp,x,y,z,qx,qy,qz,qw") {
    *error = "truth CSV header is invalid";
    return false;
  }
  std::vector<TruthRow> parsed;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::istringstream values(line);
    std::array<double, 8> fields{};
    std::string token;
    for (double& field : fields) {
      if (!std::getline(values, token, ',')) {
        *error = "truth CSV row is incomplete";
        return false;
      }
      try {
        std::size_t consumed = 0;
        field = std::stod(token, &consumed);
        if (consumed != token.size() || !std::isfinite(field)) {
          *error = "truth CSV contains a non-finite value";
          return false;
        }
      } catch (const std::exception&) {
        *error = "truth CSV contains an invalid number";
        return false;
      }
    }
    if (std::getline(values, token, ',')) {
      *error = "truth CSV row has extra fields";
      return false;
    }
    if (!parsed.empty() && fields[0] <= parsed.back().timestamp) {
      *error = "truth timestamps are not strictly increasing";
      return false;
    }
    parsed.push_back({fields[0],
                      {fields[1], fields[2], fields[3]},
                      {fields[4], fields[5], fields[6], fields[7]}});
  }
  if (parsed.size() < 20) {
    *error = "truth trajectory has too few samples";
    return false;
  }
  *rows = std::move(parsed);
  return true;
}

double NormalizeAngle(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

double QuaternionYaw(const std::array<double, 4>& quaternion) {
  const double x = quaternion[0];
  const double y = quaternion[1];
  const double z = quaternion[2];
  const double w = quaternion[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

struct MapGeometryMetrics {
  std::size_t candidate_points = 0;
  std::size_t surface_inliers = 0;
  std::array<std::size_t, kColumnCentersWorld.size()> column_support{};
  std::array<std::size_t, kColumnCentersWorld.size()> angular_coverage{};
  double inlier_ratio = 0.0;
  double residual_p90_m = std::numeric_limits<double>::infinity();
};

bool ValidateMapGeometry(const std::vector<MapPoint>& points, const TruthRow& reference,
                         MapGeometryMetrics* metrics, std::string* error) {
  constexpr double kNominalColumnRadiusM = 0.46;
  constexpr double kSurfaceToleranceM = 0.20;
  constexpr std::size_t kAngularBins = 12;
  constexpr std::size_t kMinimumCandidatePoints = 100;
  constexpr std::size_t kMinimumColumnSupport = 12;
  constexpr std::size_t kMinimumAngularCoverage = 4;
  constexpr double kMinimumInlierRatio = 0.50;
  constexpr double kMaximumResidualP90M = 0.70;

  *metrics = MapGeometryMetrics{};
  const double yaw = QuaternionYaw(reference.quaternion);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  std::array<std::array<bool, kAngularBins>, kColumnCentersWorld.size()> occupied_bins{};
  std::vector<double> residuals;

  for (const MapPoint& point : points) {
    // The first mapper pose is the LiDAR frame identity. Its nearest truth
    // sample supplies the rigid map-to-world transform.
    const double world_x = reference.position[0] + cosine * point[0] - sine * point[1];
    const double world_y = reference.position[1] + sine * point[0] + cosine * point[1];

    // This region contains all five columns but excludes the arena walls and
    // the vehicle's perimeter path. The Z cut removes floor returns.
    if (world_x < -3.80 || world_x > 3.90 || world_y < -1.90 || world_y > 2.00 ||
        point[2] < -0.15F || point[2] > 1.80F) {
      continue;
    }

    std::size_t nearest_column = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t column = 0; column < kColumnCentersWorld.size(); ++column) {
      const double distance = std::hypot(world_x - kColumnCentersWorld[column][0],
                                         world_y - kColumnCentersWorld[column][1]);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_column = column;
      }
    }

    ++metrics->candidate_points;
    const double residual = std::abs(nearest_distance - kNominalColumnRadiusM);
    residuals.push_back(residual);
    if (residual > kSurfaceToleranceM) continue;

    ++metrics->surface_inliers;
    ++metrics->column_support[nearest_column];
    double angle = std::atan2(world_y - kColumnCentersWorld[nearest_column][1],
                              world_x - kColumnCentersWorld[nearest_column][0]);
    if (angle < 0.0) angle += 2.0 * std::numbers::pi;
    const std::size_t bin =
        std::min(kAngularBins - 1,
                 static_cast<std::size_t>(angle * kAngularBins / (2.0 * std::numbers::pi)));
    occupied_bins[nearest_column][bin] = true;
  }

  if (!residuals.empty()) {
    std::sort(residuals.begin(), residuals.end());
    metrics->residual_p90_m = residuals[(90 * (residuals.size() - 1)) / 100];
  }
  if (metrics->candidate_points > 0) {
    metrics->inlier_ratio = static_cast<double>(metrics->surface_inliers) /
                            static_cast<double>(metrics->candidate_points);
  }
  bool columns_pass = true;
  for (std::size_t column = 0; column < kColumnCentersWorld.size(); ++column) {
    metrics->angular_coverage[column] = static_cast<std::size_t>(
        std::count(occupied_bins[column].begin(), occupied_bins[column].end(), true));
    columns_pass = columns_pass && metrics->column_support[column] >= kMinimumColumnSupport &&
                   metrics->angular_coverage[column] >= kMinimumAngularCoverage;
  }

  if (metrics->candidate_points < kMinimumCandidatePoints || !columns_pass ||
      metrics->inlier_ratio < kMinimumInlierRatio ||
      metrics->residual_p90_m > kMaximumResidualP90M) {
    std::ostringstream message;
    message << "map landmark geometry gate failed: candidates=" << metrics->candidate_points
            << " inliers=" << metrics->surface_inliers << " ratio=" << metrics->inlier_ratio
            << " residual_p90=" << metrics->residual_p90_m << " support=";
    for (std::size_t column = 0; column < kColumnCentersWorld.size(); ++column) {
      if (column > 0) message << ',';
      message << metrics->column_support[column] << '/' << metrics->angular_coverage[column];
    }
    *error = message.str();
    return false;
  }
  return true;
}

std::size_t NearestTruth(const std::vector<TruthRow>& truth, double timestamp) {
  const auto upper =
      std::lower_bound(truth.begin(), truth.end(), timestamp,
                       [](const TruthRow& row, double value) { return row.timestamp < value; });
  if (upper == truth.begin()) return 0;
  if (upper == truth.end()) return truth.size() - 1;
  const std::size_t upper_index = static_cast<std::size_t>(upper - truth.begin());
  const std::size_t lower_index = upper_index - 1;
  return timestamp - truth[lower_index].timestamp <= truth[upper_index].timestamp - timestamp
             ? lower_index
             : upper_index;
}

const TruthRow& MapperOriginTruth(const std::vector<std::array<double, 9>>& trajectory,
                                  const std::vector<TruthRow>& truth) {
  return truth[NearestTruth(truth, trajectory.front()[0])];
}

struct AccuracyMetrics {
  double ate_rmse_m = 0.0;
  double translation_rpe_rmse_m = 0.0;
  double yaw_rpe_rmse_rad = 0.0;
  double truth_path_length_m = 0.0;
  double path_scale = 0.0;
  double maximum_time_offset_s = 0.0;
  double fitness_p95 = 0.0;
};

[[maybe_unused]] bool ComputeAccuracy(const std::vector<std::array<double, 9>>& trajectory,
                                      const std::vector<TruthRow>& truth, double mapper_path_length,
                                      AccuracyMetrics* metrics, std::string* error) {
  if (trajectory.size() < 2 || truth.size() < 2) {
    *error = "trajectory accuracy requires at least two poses";
    return false;
  }
  const auto& reference = MapperOriginTruth(trajectory, truth);
  const double reference_yaw = QuaternionYaw(reference.quaternion);
  const double cosine = std::cos(reference_yaw);
  const double sine = std::sin(reference_yaw);
  std::array<double, 3> previous_truth{};
  std::array<double, 3> previous_mapper{};
  double previous_truth_yaw = 0.0;
  double previous_mapper_yaw = 0.0;
  double squared_ate = 0.0;
  double squared_translation_rpe = 0.0;
  double squared_yaw_rpe = 0.0;
  std::vector<double> fitness;
  fitness.reserve(trajectory.size());

  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    const auto& pose = trajectory[index];
    const std::size_t truth_index = NearestTruth(truth, pose[0]);
    const auto& truth_pose = truth[truth_index];
    metrics->maximum_time_offset_s =
        std::max(metrics->maximum_time_offset_s, std::abs(truth_pose.timestamp - pose[0]));
    const double delta_x = truth_pose.position[0] - reference.position[0];
    const double delta_y = truth_pose.position[1] - reference.position[1];
    const std::array<double, 3> truth_in_map = {cosine * delta_x + sine * delta_y,
                                                -sine * delta_x + cosine * delta_y,
                                                truth_pose.position[2] - reference.position[2]};
    const std::array<double, 3> mapper_position = {pose[1], pose[2], pose[3]};
    const double truth_yaw = NormalizeAngle(QuaternionYaw(truth_pose.quaternion) - reference_yaw);
    const double mapper_yaw = QuaternionYaw({pose[4], pose[5], pose[6], pose[7]});
    double squared_position_error = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double error_value = mapper_position[axis] - truth_in_map[axis];
      squared_position_error += error_value * error_value;
    }
    squared_ate += squared_position_error;
    fitness.push_back(pose[8]);

    if (index > 0) {
      double truth_step_squared = 0.0;
      double translation_error_squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double truth_step = truth_in_map[axis] - previous_truth[axis];
        const double mapper_step = mapper_position[axis] - previous_mapper[axis];
        truth_step_squared += truth_step * truth_step;
        const double step_error = mapper_step - truth_step;
        translation_error_squared += step_error * step_error;
      }
      metrics->truth_path_length_m += std::sqrt(truth_step_squared);
      squared_translation_rpe += translation_error_squared;
      const double truth_yaw_step = NormalizeAngle(truth_yaw - previous_truth_yaw);
      const double mapper_yaw_step = NormalizeAngle(mapper_yaw - previous_mapper_yaw);
      const double yaw_error = NormalizeAngle(mapper_yaw_step - truth_yaw_step);
      squared_yaw_rpe += yaw_error * yaw_error;
    }
    previous_truth = truth_in_map;
    previous_mapper = mapper_position;
    previous_truth_yaw = truth_yaw;
    previous_mapper_yaw = mapper_yaw;
  }

  metrics->ate_rmse_m = std::sqrt(squared_ate / static_cast<double>(trajectory.size()));
  const double transitions = static_cast<double>(trajectory.size() - 1);
  metrics->translation_rpe_rmse_m = std::sqrt(squared_translation_rpe / transitions);
  metrics->yaw_rpe_rmse_rad = std::sqrt(squared_yaw_rpe / transitions);
  if (metrics->truth_path_length_m <= 0.0) {
    *error = "truth trajectory has zero path length";
    return false;
  }
  metrics->path_scale = mapper_path_length / metrics->truth_path_length_m;
  std::sort(fitness.begin(), fitness.end());
  metrics->fitness_p95 = fitness[(95 * (fitness.size() - 1)) / 100];
  return true;
}

[[maybe_unused]] bool ReadMapperAccounting(const std::filesystem::path& path,
                                           std::uint64_t* processed, std::uint64_t* accepted,
                                           std::uint64_t* rejected, std::string* error) {
  std::ifstream input(path);
  if (!input) {
    *error = "failed to open " + path.string();
    return false;
  }
  const std::regex summary(
      R"(mapper summary processed=([0-9]+) accepted=([0-9]+) rejected=([0-9]+))");
  std::string line;
  std::smatch match;
  while (std::getline(input, line)) {
    if (std::regex_search(line, match, summary)) {
      *processed = std::stoull(match[1].str());
      *accepted = std::stoull(match[2].str());
      *rejected = std::stoull(match[3].str());
      return true;
    }
  }
  *error = "mapper summary is missing";
  return false;
}

[[maybe_unused]] bool ValidatePng(const std::filesystem::path& path,
                                  std::size_t minimum_nonuniform_channels, std::string* error) {
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  if (png_image_begin_read_from_file(&image, path.string().c_str()) == 0) {
    *error = "failed to read PNG " + path.string() + ": " + image.message;
    return false;
  }
  image.format = PNG_FORMAT_RGB;
  std::vector<std::uint8_t> pixels(PNG_IMAGE_SIZE(image));
  if (png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr) == 0) {
    *error = "failed to decode PNG " + path.string() + ": " + image.message;
    png_image_free(&image);
    return false;
  }
  png_image_free(&image);
  const auto [minimum, maximum] = std::minmax_element(pixels.begin(), pixels.end());
  const std::size_t different = static_cast<std::size_t>(
      std::count_if(pixels.begin(), pixels.end(),
                    [reference = pixels.front()](auto value) { return value != reference; }));
  if (minimum == pixels.end() || *minimum == *maximum || different < minimum_nonuniform_channels) {
    *error = "PNG is blank or effectively uniform: " + path.string();
    return false;
  }
  return true;
}

}  // namespace

#ifndef SENSOR_DEMO_E2E_QUALITY_SCORE_NO_MAIN
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: e2e_quality_score OUTPUT_DIRECTORY\n";
    return 2;
  }
  const std::filesystem::path output = argv[1];
  std::string error;
  std::size_t map_points = 0;
  Bounds map_bounds;
  std::vector<MapPoint> map;
  if (!ReadPcd(output / "map.pcd", &map_points, &map_bounds, &map, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  std::size_t poses = 0;
  double path_length = 0.0;
  double closure_error = 0.0;
  Bounds trajectory_bounds;
  std::vector<std::array<double, 9>> trajectory;
  if (!ReadTrajectory(output / "trajectory.csv", &poses, &path_length, &closure_error,
                      &trajectory_bounds, &trajectory, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  std::vector<TruthRow> truth;
  if (!ReadTruth(output / "truth.csv", &truth, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  MapGeometryMetrics geometry;
  if (!ValidateMapGeometry(map, MapperOriginTruth(trajectory, truth), &geometry, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  AccuracyMetrics accuracy;
  if (!ComputeAccuracy(trajectory, truth, path_length, &accuracy, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  std::uint64_t processed = 0;
  std::uint64_t accepted = 0;
  std::uint64_t rejected = 0;
  if (!ReadMapperAccounting(output / "logs/mapper.log", &processed, &accepted, &rejected, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  const double acceptance_ratio =
      processed == 0 ? 0.0 : static_cast<double>(accepted) / static_cast<double>(processed);
  if (!ValidatePng(output / "map_top_down.png", 100, &error) ||
      !ValidatePng(output / "front_camera.png", 100, &error)) {
    std::cerr << error << '\n';
    return 1;
  }

  // These are post-run quality gates only. No model truth enters the mapper.
  if (map_points < 1000 || poses < 20 || path_length < 15.0 ||
      trajectory_bounds.max_x - trajectory_bounds.min_x < 6.0 ||
      trajectory_bounds.max_y - trajectory_bounds.min_y < 3.0 || closure_error > 3.0 ||
      map_bounds.max_x - map_bounds.min_x < 7.0 || map_bounds.max_y - map_bounds.min_y < 4.0 ||
      processed != accepted + rejected || acceptance_ratio < 0.60 || accuracy.ate_rmse_m > 2.0 ||
      accuracy.translation_rpe_rmse_m > 0.40 || accuracy.yaw_rpe_rmse_rad > 0.35 ||
      accuracy.path_scale < 0.65 || accuracy.path_scale > 1.35 ||
      accuracy.maximum_time_offset_s > 0.06 || accuracy.fitness_p95 > 0.20) {
    std::cerr << "map quality gate failed: points=" << map_points << " poses=" << poses
              << " path_length=" << path_length << " closure=" << closure_error
              << " trajectory_span=" << trajectory_bounds.max_x - trajectory_bounds.min_x << 'x'
              << trajectory_bounds.max_y - trajectory_bounds.min_y
              << " map_span=" << map_bounds.max_x - map_bounds.min_x << 'x'
              << map_bounds.max_y - map_bounds.min_y << " acceptance=" << acceptance_ratio
              << " ate=" << accuracy.ate_rmse_m
              << " translation_rpe=" << accuracy.translation_rpe_rmse_m
              << " yaw_rpe=" << accuracy.yaw_rpe_rmse_rad << " scale=" << accuracy.path_scale
              << " time_offset=" << accuracy.maximum_time_offset_s
              << " fitness_p95=" << accuracy.fitness_p95 << '\n';
    return 1;
  }
  std::cout << "map quality passed points=" << map_points << " poses=" << poses
            << " path_length=" << path_length << " closure=" << closure_error
            << " acceptance=" << acceptance_ratio << " ate=" << accuracy.ate_rmse_m
            << " translation_rpe=" << accuracy.translation_rpe_rmse_m
            << " yaw_rpe=" << accuracy.yaw_rpe_rmse_rad << " scale=" << accuracy.path_scale
            << " fitness_p95=" << accuracy.fitness_p95
            << " landmark_ratio=" << geometry.inlier_ratio
            << " landmark_residual_p90=" << geometry.residual_p90_m << '\n';
  return 0;
}
#endif
