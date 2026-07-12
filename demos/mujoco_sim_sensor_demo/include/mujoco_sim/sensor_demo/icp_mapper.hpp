#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mujoco_sim::sensor_demo {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct SimpleIcpMapperConfig {
  double voxel_size = 0.10;
  double max_correspondence_distance = 0.75;
  int max_iterations = 30;
  std::size_t local_map_scans = 20;
  std::size_t minimum_points = 20;
  double transformation_epsilon = 1e-8;
  double euclidean_fitness_epsilon = 1e-6;
  double maximum_fitness_score = 0.25;
  double minimum_inlier_ratio = 0.15;
  double maximum_translation_step_m = 1.0;
  double maximum_rotation_step_rad = 0.8;
};

struct MapperTrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double fitness = 0.0;
};

using MapperTrajectory = std::vector<MapperTrajectoryPoint>;

struct IcpUpdateResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0.0;
  bool accepted = false;
  bool converged = false;
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  double translation_step_m = 0.0;
  double rotation_step_rad = 0.0;
  std::string rejection_reason;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  std::size_t input_points = 0;
  std::size_t downsampled_points = 0;
  std::size_t global_map_points = 0;
};

class IcpMapperError : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

// Registers XYZ scans from a 16-line lidar. Ring identity is not required by
// ICP, so callers provide an ordinary XYZ cloud. IMU orientation is interpreted
// as world-from-sensor and only relative rotations are used; the map frame is
// anchored at the first scan. Sensor and IMU axes are assumed to be aligned.
class SimpleIcpMapper {
 public:
  explicit SimpleIcpMapper(SimpleIcpMapperConfig config = {});
  ~SimpleIcpMapper();
  SimpleIcpMapper(SimpleIcpMapper&&) noexcept;
  SimpleIcpMapper& operator=(SimpleIcpMapper&&) noexcept;
  SimpleIcpMapper(const SimpleIcpMapper&) = delete;
  SimpleIcpMapper& operator=(const SimpleIcpMapper&) = delete;

  // A non-converged registration returns accepted=false and does not mutate
  // mapper state. Invalid timestamps, quaternions, or points throw likewise.
  IcpUpdateResult AddScan(double timestamp, const PointCloud& cloud,
                          const Eigen::Quaterniond& imu_orientation);
  IcpUpdateResult AddScan(double timestamp, const PointCloud::ConstPtr& cloud,
                          const Eigen::Quaterniond& imu_orientation);

  void Reset();
  [[nodiscard]] const SimpleIcpMapperConfig& config() const noexcept;
  [[nodiscard]] Eigen::Isometry3d pose() const;
  [[nodiscard]] PointCloud::ConstPtr global_map() const noexcept;
  [[nodiscard]] const MapperTrajectory& trajectory() const noexcept;
  [[nodiscard]] std::size_t scan_count() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::sensor_demo
