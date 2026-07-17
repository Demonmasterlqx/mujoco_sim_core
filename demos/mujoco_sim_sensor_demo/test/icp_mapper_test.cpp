#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mujoco_sim/sensor_demo/icp_mapper.hpp>

namespace demo = mujoco_sim::sensor_demo;
namespace {

demo::PointCloud MakeRectangleAndColumnsCloud() {
  demo::PointCloud cloud;
  constexpr std::array<std::array<double, 3>, 5> kColumns{{
      {-2.1, -0.8, 0.22},
      {-0.9, 1.1, 0.27},
      {0.4, -0.3, 0.19},
      {1.5, 1.2, 0.31},
      {2.3, -1.0, 0.24},
  }};

  for (int line = 0; line < 16; ++line) {
    const double z = 0.12 + 0.11 * line;
    for (double x = -3.0; x <= 3.0001; x += 0.18) {
      cloud.emplace_back(static_cast<float>(x), -2.0F, static_cast<float>(z));
      cloud.emplace_back(static_cast<float>(x), 2.0F, static_cast<float>(z));
    }
    for (double y = -1.82; y <= 1.8201; y += 0.18) {
      cloud.emplace_back(-3.0F, static_cast<float>(y), static_cast<float>(z));
      cloud.emplace_back(3.0F, static_cast<float>(y), static_cast<float>(z));
    }
    for (const auto& column : kColumns) {
      for (int facet = 0; facet < 12; ++facet) {
        const double angle = facet * 2.0 * std::numbers::pi / 12.0;
        cloud.emplace_back(static_cast<float>(column[0] + column[2] * std::cos(angle)),
                           static_cast<float>(column[1] + column[2] * std::sin(angle)),
                           static_cast<float>(z));
      }
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

Eigen::Isometry3d MakePose(double x, double y, double z, double yaw, double pitch = 0.0) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))
                      .toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

demo::PointCloud ObserveFromPose(const demo::PointCloud& world,
                                 const Eigen::Isometry3d& sensor_pose) {
  demo::PointCloud observed;
  observed.reserve(world.size());
  const Eigen::Isometry3d world_to_sensor = sensor_pose.inverse();
  for (const pcl::PointXYZ& point : world) {
    const Eigen::Vector3d transformed =
        world_to_sensor * Eigen::Vector3d(point.x, point.y, point.z);
    observed.emplace_back(static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
                          static_cast<float>(transformed.z()));
  }
  observed.width = static_cast<std::uint32_t>(observed.size());
  observed.height = 1;
  observed.is_dense = true;
  return observed;
}

double RotationError(const Eigen::Isometry3d& actual, const Eigen::Isometry3d& expected) {
  return Eigen::AngleAxisd(actual.rotation().transpose() * expected.rotation()).angle();
}

}  // namespace

TEST(SimpleIcpMapperTest, DefaultsDocumentRequestedPclConfiguration) {
  const demo::SimpleIcpMapper mapper;
  EXPECT_DOUBLE_EQ(mapper.config().voxel_size, 0.10);
  EXPECT_DOUBLE_EQ(mapper.config().max_correspondence_distance, 0.75);
  EXPECT_EQ(mapper.config().max_iterations, 30);
  EXPECT_EQ(mapper.config().local_map_scans, 20U);
  EXPECT_DOUBLE_EQ(mapper.config().maximum_fitness_score, 0.25);
  EXPECT_DOUBLE_EQ(mapper.config().minimum_inlier_ratio, 0.15);
  EXPECT_DOUBLE_EQ(mapper.config().maximum_translation_step_m, 1.0);
  EXPECT_DOUBLE_EQ(mapper.config().maximum_rotation_step_rad, 0.8);
  EXPECT_EQ(mapper.scan_count(), 0U);
  ASSERT_NE(mapper.global_map(), nullptr);
  EXPECT_TRUE(mapper.global_map()->empty());
}

TEST(SimpleIcpMapperTest, RecoversKnownTransformsWithImuAndVelocityGuess) {
  const demo::PointCloud world = MakeRectangleAndColumnsCloud();
  demo::SimpleIcpMapper mapper;

  const auto first = mapper.AddScan(10.0, world, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(first.accepted);
  ASSERT_TRUE(first.converged);
  EXPECT_TRUE(first.pose.isApprox(Eigen::Isometry3d::Identity()));

  const Eigen::Isometry3d second_pose = MakePose(0.22, -0.13, 0.025, 0.10, 0.012);
  const auto second = mapper.AddScan(10.1, ObserveFromPose(world, second_pose),
                                     Eigen::Quaterniond(second_pose.rotation()));
  ASSERT_TRUE(second.accepted);
  ASSERT_TRUE(second.converged);
  EXPECT_LT((second.pose.translation() - second_pose.translation()).norm(), 0.025);
  EXPECT_LT(RotationError(second.pose, second_pose), 0.015);
  EXPECT_LT(second.fitness, 0.002);
  EXPECT_GT(second.inlier_ratio, 0.9);

  const Eigen::Isometry3d third_pose = MakePose(0.44, -0.26, 0.05, 0.20, 0.024);
  const auto third = mapper.AddScan(10.2, ObserveFromPose(world, third_pose),
                                    Eigen::Quaterniond(third_pose.rotation()));
  ASSERT_TRUE(third.accepted);
  ASSERT_TRUE(third.converged);
  EXPECT_LT((third.pose.translation() - third_pose.translation()).norm(), 0.025);
  EXPECT_LT(RotationError(third.pose, third_pose), 0.015);
  EXPECT_LT(third.fitness, 0.002);

  EXPECT_EQ(mapper.scan_count(), 3U);
  EXPECT_EQ(mapper.trajectory().size(), 3U);
  for (const auto& point : mapper.trajectory()) {
    const Eigen::Matrix3d rotation = point.pose.rotation();
    EXPECT_TRUE(rotation.transpose().isApprox(rotation.inverse(), 1e-10));
    EXPECT_NEAR(rotation.determinant(), 1.0, 1e-10);
  }
  EXPECT_GT(mapper.global_map()->size(), first.global_map_points);
  EXPECT_EQ(mapper.global_map()->size(), third.global_map_points);
}

TEST(SimpleIcpMapperTest, RejectsConvergedPoseJumpTransactionally) {
  demo::SimpleIcpMapperConfig config;
  config.maximum_translation_step_m = 0.05;
  demo::SimpleIcpMapper mapper(config);
  const demo::PointCloud world = MakeRectangleAndColumnsCloud();
  ASSERT_TRUE(mapper.AddScan(1.0, world, Eigen::Quaterniond::Identity()).accepted);
  const auto map_before = mapper.global_map();
  const auto pose_before = mapper.pose();

  const Eigen::Isometry3d moved_pose = MakePose(0.22, -0.13, 0.025, 0.10, 0.012);
  const auto rejected = mapper.AddScan(1.1, ObserveFromPose(world, moved_pose),
                                       Eigen::Quaterniond(moved_pose.rotation()));
  EXPECT_TRUE(rejected.converged);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_GT(rejected.translation_step_m, config.maximum_translation_step_m);
  EXPECT_NE(rejected.rejection_reason.find("translation step"), std::string::npos);
  EXPECT_EQ(mapper.global_map().get(), map_before.get());
  EXPECT_TRUE(mapper.pose().isApprox(pose_before));
  EXPECT_EQ(mapper.scan_count(), 1U);
}

TEST(SimpleIcpMapperTest, ValidatesRegistrationQualityThresholds) {
  demo::SimpleIcpMapperConfig config;
  config.minimum_inlier_ratio = 1.1;
  EXPECT_THROW(
      {
        demo::SimpleIcpMapper invalid_mapper{config};
        (void)invalid_mapper;
      },
      demo::IcpMapperError);
  config.minimum_inlier_ratio = 0.1;
  config.maximum_fitness_score = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      {
        demo::SimpleIcpMapper invalid_mapper{config};
        (void)invalid_mapper;
      },
      demo::IcpMapperError);
  config.maximum_fitness_score = 0.1;
  config.maximum_rotation_step_rad = std::numbers::pi + 0.1;
  EXPECT_THROW(
      {
        demo::SimpleIcpMapper invalid_mapper{config};
        (void)invalid_mapper;
      },
      demo::IcpMapperError);
}

TEST(SimpleIcpMapperTest, InvalidInputIsRejectedTransactionally) {
  const demo::PointCloud world = MakeRectangleAndColumnsCloud();
  demo::SimpleIcpMapper mapper;
  mapper.AddScan(1.0, world, Eigen::Quaterniond::Identity());
  const demo::PointCloud::ConstPtr map_before = mapper.global_map();
  const Eigen::Isometry3d pose_before = mapper.pose();

  demo::PointCloud non_overlapping = world;
  for (pcl::PointXYZ& point : non_overlapping) {
    point.x += 20.0F;
  }
  const auto not_converged = mapper.AddScan(1.5, non_overlapping, Eigen::Quaterniond::Identity());
  EXPECT_FALSE(not_converged.accepted);
  EXPECT_FALSE(not_converged.converged);
  EXPECT_EQ(mapper.global_map().get(), map_before.get());
  EXPECT_EQ(mapper.scan_count(), 1U);
  EXPECT_TRUE(mapper.pose().isApprox(pose_before));

  demo::PointCloud bad_cloud = world;
  bad_cloud[17].z = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(mapper.AddScan(2.0, bad_cloud, Eigen::Quaterniond::Identity()),
               demo::IcpMapperError);
  EXPECT_EQ(mapper.global_map().get(), map_before.get());
  EXPECT_EQ(mapper.scan_count(), 1U);
  EXPECT_TRUE(mapper.pose().isApprox(pose_before));

  EXPECT_THROW(mapper.AddScan(2.0, world, Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)),
               demo::IcpMapperError);
  EXPECT_EQ(mapper.global_map().get(), map_before.get());
  EXPECT_EQ(mapper.scan_count(), 1U);

  const auto accepted = mapper.AddScan(2.0, world, Eigen::Quaterniond::Identity());
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(mapper.scan_count(), 2U);

  EXPECT_THROW(mapper.AddScan(2.0, world, Eigen::Quaterniond::Identity()), demo::IcpMapperError);
  EXPECT_EQ(mapper.scan_count(), 2U);
}
