#include <mujoco_sim/sensor_demo/icp_mapper.hpp>

#include <cmath>
#include <deque>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

namespace mujoco_sim::sensor_demo {
namespace {

constexpr double kMinimumQuaternionNorm = 1e-12;

void RequireFinite(double value, const std::string& field) {
  if (!std::isfinite(value)) {
    throw IcpMapperError("non-finite " + field);
  }
}

void ValidateConfig(const SimpleIcpMapperConfig& config) {
  RequireFinite(config.voxel_size, "voxel_size");
  RequireFinite(config.max_correspondence_distance,
                "max_correspondence_distance");
  RequireFinite(config.transformation_epsilon, "transformation_epsilon");
  RequireFinite(config.euclidean_fitness_epsilon,
                "euclidean_fitness_epsilon");
  RequireFinite(config.maximum_fitness_score, "maximum_fitness_score");
  RequireFinite(config.minimum_inlier_ratio, "minimum_inlier_ratio");
  RequireFinite(config.maximum_translation_step_m,
                "maximum_translation_step_m");
  RequireFinite(config.maximum_rotation_step_rad,
                "maximum_rotation_step_rad");
  if (config.voxel_size <= 0.0 ||
      config.max_correspondence_distance <= 0.0 ||
      config.max_iterations <= 0 || config.local_map_scans == 0 ||
      config.minimum_points < 3 || config.transformation_epsilon < 0.0 ||
      config.euclidean_fitness_epsilon < 0.0 ||
      config.maximum_fitness_score < 0.0 ||
      config.minimum_inlier_ratio < 0.0 ||
      config.minimum_inlier_ratio > 1.0 ||
      config.maximum_translation_step_m <= 0.0 ||
      config.maximum_rotation_step_rad <= 0.0 ||
      config.maximum_rotation_step_rad > std::numbers::pi) {
    throw IcpMapperError("invalid ICP mapper configuration");
  }
}

Eigen::Quaterniond NormalizeQuaternion(
    const Eigen::Quaterniond& quaternion) {
  RequireFinite(quaternion.w(), "IMU quaternion.w");
  RequireFinite(quaternion.x(), "IMU quaternion.x");
  RequireFinite(quaternion.y(), "IMU quaternion.y");
  RequireFinite(quaternion.z(), "IMU quaternion.z");
  const double first_pair = std::hypot(quaternion.w(), quaternion.x());
  const double second_pair = std::hypot(quaternion.y(), quaternion.z());
  const double norm = std::hypot(first_pair, second_pair);
  if (!std::isfinite(norm) || norm < kMinimumQuaternionNorm) {
    throw IcpMapperError("IMU quaternion has near-zero norm");
  }
  return Eigen::Quaterniond(quaternion.w() / norm, quaternion.x() / norm,
                            quaternion.y() / norm, quaternion.z() / norm);
}

void ValidateCloud(const PointCloud& cloud, std::size_t minimum_points) {
  if (cloud.size() < minimum_points) {
    throw IcpMapperError("point cloud has too few points");
  }
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const pcl::PointXYZ& point = cloud[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      throw IcpMapperError("point cloud contains a non-finite point at index " +
                           std::to_string(index));
    }
  }
}

PointCloud::Ptr Downsample(const PointCloud& cloud,
                           const SimpleIcpMapperConfig& config) {
  auto input = pcl::make_shared<PointCloud>(cloud);
  auto output = pcl::make_shared<PointCloud>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  const float leaf_size = static_cast<float>(config.voxel_size);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setInputCloud(input);
  voxel_filter.filter(*output);
  output->is_dense = true;
  if (output->size() < config.minimum_points) {
    throw IcpMapperError("voxelized point cloud has too few points");
  }
  return output;
}

PointCloud::Ptr BuildLocalTarget(
    const std::deque<PointCloud::ConstPtr>& local_scans) {
  auto target = pcl::make_shared<PointCloud>();
  std::size_t point_count = 0;
  for (const auto& scan : local_scans) {
    point_count += scan->size();
  }
  target->reserve(point_count);
  for (const auto& scan : local_scans) {
    *target += *scan;
  }
  target->is_dense = true;
  return target;
}

bool IsFiniteRigidTransform(const Eigen::Matrix4d& transform) {
  if (!transform.allFinite()) {
    return false;
  }
  const Eigen::Vector4d expected_bottom(0.0, 0.0, 0.0, 1.0);
  if (!transform.row(3).transpose().isApprox(expected_bottom, 1e-5)) {
    return false;
  }
  const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
  return rotation.transpose().isApprox(rotation.inverse(), 1e-3) &&
         std::abs(rotation.determinant() - 1.0) <= 1e-3;
}

double FiniteFitness(double fitness) {
  return std::isfinite(fitness) ? fitness
                                : std::numeric_limits<double>::max();
}

double ComputeInlierRatio(const PointCloud& aligned,
                          const PointCloud::ConstPtr& target,
                          double maximum_distance) {
  if (aligned.empty() || !target || target->empty()) {
    return 0.0;
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  const float maximum_squared_distance = static_cast<float>(
      maximum_distance * maximum_distance);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  std::size_t inliers = 0;
  for (const auto& point : aligned) {
    if (tree.nearestKSearch(point, 1, nearest_index,
                            nearest_squared_distance) == 1 &&
        nearest_squared_distance[0] <= maximum_squared_distance) {
      ++inliers;
    }
  }
  return static_cast<double>(inliers) /
         static_cast<double>(aligned.size());
}

}  // namespace

struct SimpleIcpMapper::Impl {
  explicit Impl(SimpleIcpMapperConfig mapper_config)
      : config(std::move(mapper_config)),
        global_map(pcl::make_shared<PointCloud>()) {
    ValidateConfig(config);
  }

  SimpleIcpMapperConfig config;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond previous_imu_orientation =
      Eigen::Quaterniond::Identity();
  double previous_timestamp = 0.0;
  PointCloud::Ptr global_map;
  std::deque<PointCloud::ConstPtr> local_scans;
  MapperTrajectory trajectory;
};

SimpleIcpMapper::SimpleIcpMapper(SimpleIcpMapperConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

SimpleIcpMapper::~SimpleIcpMapper() = default;
SimpleIcpMapper::SimpleIcpMapper(SimpleIcpMapper&&) noexcept = default;
SimpleIcpMapper& SimpleIcpMapper::operator=(SimpleIcpMapper&&) noexcept =
    default;

IcpUpdateResult SimpleIcpMapper::AddScan(
    double timestamp, const PointCloud& cloud,
    const Eigen::Quaterniond& imu_orientation) {
  RequireFinite(timestamp, "timestamp");
  if (!impl_->trajectory.empty() && timestamp <= impl_->previous_timestamp) {
    throw IcpMapperError("scan timestamps must increase strictly");
  }
  ValidateCloud(cloud, impl_->config.minimum_points);
  const Eigen::Quaterniond normalized_imu =
      NormalizeQuaternion(imu_orientation);
  PointCloud::Ptr downsampled = Downsample(cloud, impl_->config);

  IcpUpdateResult result;
  result.timestamp = timestamp;
  result.input_points = cloud.size();
  result.downsampled_points = downsampled->size();

  if (impl_->trajectory.empty()) {
    auto next_global_map = pcl::make_shared<PointCloud>(*downsampled);
    std::deque<PointCloud::ConstPtr> next_local_scans;
    next_local_scans.push_back(downsampled);
    MapperTrajectory next_trajectory;
    next_trajectory.push_back(
        {timestamp, Eigen::Isometry3d::Identity(), 0.0});

    impl_->pose = Eigen::Isometry3d::Identity();
    impl_->previous_imu_orientation = normalized_imu;
    impl_->previous_timestamp = timestamp;
    impl_->global_map = std::move(next_global_map);
    impl_->local_scans = std::move(next_local_scans);
    impl_->trajectory = std::move(next_trajectory);

    result.accepted = true;
    result.converged = true;
    result.fitness = 0.0;
    result.inlier_ratio = 1.0;
    result.pose = impl_->pose;
    result.global_map_points = impl_->global_map->size();
    return result;
  }

  Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond relative_rotation =
      impl_->previous_imu_orientation.conjugate() * normalized_imu;
  relative_rotation.normalize();
  initial_guess.linear() =
      impl_->pose.linear() * relative_rotation.toRotationMatrix();
  initial_guess.translation() = impl_->pose.translation();

  if (impl_->trajectory.size() >= 2) {
    const MapperTrajectoryPoint& latest = impl_->trajectory.back();
    const MapperTrajectoryPoint& preceding =
        impl_->trajectory[impl_->trajectory.size() - 2];
    const double velocity_interval = latest.timestamp - preceding.timestamp;
    const double prediction_interval = timestamp - latest.timestamp;
    if (velocity_interval > 0.0) {
      const Eigen::Vector3d velocity =
          (latest.pose.translation() - preceding.pose.translation()) /
          velocity_interval;
      initial_guess.translation() += velocity * prediction_interval;
    }
  }
  if (!initial_guess.matrix().allFinite()) {
    throw IcpMapperError("constant-velocity initial guess is non-finite");
  }

  PointCloud::Ptr local_target = BuildLocalTarget(impl_->local_scans);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(downsampled);
  icp.setInputTarget(local_target);
  icp.setMaxCorrespondenceDistance(
      impl_->config.max_correspondence_distance);
  icp.setMaximumIterations(impl_->config.max_iterations);
  icp.setTransformationEpsilon(impl_->config.transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(
      impl_->config.euclidean_fitness_epsilon);
  icp.setRANSACIterations(0);

  PointCloud aligned;
  icp.align(aligned, initial_guess.matrix().cast<float>());
  const double fitness = FiniteFitness(icp.getFitnessScore());
  const Eigen::Matrix4d final_matrix =
      icp.getFinalTransformation().cast<double>();
  const bool valid_transform = IsFiniteRigidTransform(final_matrix);

  result.converged = icp.hasConverged() && valid_transform &&
                     fitness < std::numeric_limits<double>::max();
  result.fitness = fitness;
  result.pose = initial_guess;
  result.global_map_points = impl_->global_map->size();
  if (!result.converged) {
    result.rejection_reason = "ICP did not converge to a finite rigid pose";
    if (valid_transform) {
      result.pose.matrix() = final_matrix;
    }
    return result;
  }

  Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond final_rotation(final_matrix.topLeftCorner<3, 3>());
  final_rotation.normalize();
  final_pose.linear() = final_rotation.toRotationMatrix();
  final_pose.translation() = final_matrix.topRightCorner<3, 1>();
  result.inlier_ratio = ComputeInlierRatio(
      aligned, local_target, impl_->config.max_correspondence_distance);
  result.translation_step_m =
      (final_pose.translation() - impl_->pose.translation()).norm();
  result.rotation_step_rad =
      Eigen::AngleAxisd(impl_->pose.rotation().transpose() *
                        final_pose.rotation())
          .angle();
  if (fitness > impl_->config.maximum_fitness_score) {
    result.pose = final_pose;
    result.rejection_reason = "fitness score exceeds configured maximum";
    return result;
  }
  if (result.inlier_ratio < impl_->config.minimum_inlier_ratio) {
    result.pose = final_pose;
    result.rejection_reason = "inlier ratio is below configured minimum";
    return result;
  }
  if (result.translation_step_m >
      impl_->config.maximum_translation_step_m) {
    result.pose = final_pose;
    result.rejection_reason = "translation step exceeds configured maximum";
    return result;
  }
  if (result.rotation_step_rad > impl_->config.maximum_rotation_step_rad) {
    result.pose = final_pose;
    result.rejection_reason = "rotation step exceeds configured maximum";
    return result;
  }
  auto world_scan = pcl::make_shared<PointCloud>();
  pcl::transformPointCloud(*downsampled, *world_scan,
                           final_pose.matrix().cast<float>());
  ValidateCloud(*world_scan, impl_->config.minimum_points);
  world_scan->is_dense = true;

  auto next_global_map = pcl::make_shared<PointCloud>(*impl_->global_map);
  *next_global_map += *world_scan;
  next_global_map->is_dense = true;
  std::deque<PointCloud::ConstPtr> next_local_scans = impl_->local_scans;
  next_local_scans.push_back(world_scan);
  while (next_local_scans.size() > impl_->config.local_map_scans) {
    next_local_scans.pop_front();
  }
  MapperTrajectory next_trajectory = impl_->trajectory;
  next_trajectory.push_back({timestamp, final_pose, fitness});

  impl_->pose = final_pose;
  impl_->previous_imu_orientation = normalized_imu;
  impl_->previous_timestamp = timestamp;
  impl_->global_map = std::move(next_global_map);
  impl_->local_scans = std::move(next_local_scans);
  impl_->trajectory = std::move(next_trajectory);

  result.accepted = true;
  result.pose = final_pose;
  result.global_map_points = impl_->global_map->size();
  return result;
}

IcpUpdateResult SimpleIcpMapper::AddScan(
    double timestamp, const PointCloud::ConstPtr& cloud,
    const Eigen::Quaterniond& imu_orientation) {
  if (!cloud) {
    throw IcpMapperError("point cloud pointer is null");
  }
  return AddScan(timestamp, *cloud, imu_orientation);
}

void SimpleIcpMapper::Reset() {
  const SimpleIcpMapperConfig config = impl_->config;
  impl_ = std::make_unique<Impl>(config);
}

const SimpleIcpMapperConfig& SimpleIcpMapper::config() const noexcept {
  return impl_->config;
}

Eigen::Isometry3d SimpleIcpMapper::pose() const { return impl_->pose; }

PointCloud::ConstPtr SimpleIcpMapper::global_map() const noexcept {
  return impl_->global_map;
}

const MapperTrajectory& SimpleIcpMapper::trajectory() const noexcept {
  return impl_->trajectory;
}

std::size_t SimpleIcpMapper::scan_count() const noexcept {
  return impl_->trajectory.size();
}

}  // namespace mujoco_sim::sensor_demo
