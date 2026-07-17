#include <gtest/gtest.h>

#define SENSOR_DEMO_E2E_QUALITY_SCORE_NO_MAIN
#include "e2e_quality_score.cpp"

namespace {

TruthRow MakeReference() {
  constexpr double kYaw = 0.37;
  TruthRow reference;
  reference.position = {-4.60, -2.75, 0.18};
  reference.quaternion = {0.0, 0.0, std::sin(kYaw / 2.0), std::cos(kYaw / 2.0)};
  return reference;
}

MapPoint WorldToMap(double world_x, double world_y, double map_z, const TruthRow& reference) {
  const double yaw = QuaternionYaw(reference.quaternion);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double delta_x = world_x - reference.position[0];
  const double delta_y = world_y - reference.position[1];
  return {static_cast<float>(cosine * delta_x + sine * delta_y),
          static_cast<float>(-sine * delta_x + cosine * delta_y), static_cast<float>(map_z)};
}

std::vector<MapPoint> MakeColumnMap(const TruthRow& reference) {
  std::vector<MapPoint> points;
  for (const auto& center : kColumnCentersWorld) {
    for (std::size_t height = 0; height < 6; ++height) {
      for (std::size_t sample = 0; sample < 24; ++sample) {
        const double angle = 2.0 * std::numbers::pi * sample / 24.0;
        const double radius = 0.46 + 0.015 * std::sin(3.0 * angle + height);
        points.push_back(WorldToMap(center[0] + radius * std::cos(angle),
                                    center[1] + radius * std::sin(angle),
                                    0.10 + 0.25 * static_cast<double>(height), reference));
      }
    }
  }
  return points;
}

struct GeometryResult {
  bool passes = false;
  MapGeometryMetrics metrics;
  std::string error;
};

GeometryResult EvaluateGeometry(const std::vector<MapPoint>& points, const TruthRow& reference) {
  GeometryResult result;
  result.passes = ValidateMapGeometry(points, reference, &result.metrics, &result.error);
  return result;
}

std::string Describe(const GeometryResult& result) {
  std::ostringstream description;
  description << "candidates=" << result.metrics.candidate_points
              << " inliers=" << result.metrics.surface_inliers
              << " ratio=" << result.metrics.inlier_ratio
              << " p90=" << result.metrics.residual_p90_m << " error=" << result.error;
  return description.str();
}

TEST(E2eQualityScoreGeometryTest, AcceptsRigidFiveColumnMap) {
  const TruthRow reference = MakeReference();
  const auto result = EvaluateGeometry(MakeColumnMap(reference), reference);
  EXPECT_TRUE(result.passes) << Describe(result);
}

TEST(E2eQualityScoreGeometryTest, AnchorsMapAtFirstMapperPoseTimestamp) {
  TruthRow stale = MakeReference();
  stale.timestamp = 1.0;
  stale.position[0] += 3.0;
  stale.position[1] -= 2.0;
  constexpr double kStaleYaw = -0.85;
  stale.quaternion = {0.0, 0.0, std::sin(kStaleYaw / 2.0), std::cos(kStaleYaw / 2.0)};
  TruthRow reference = MakeReference();
  reference.timestamp = 2.0;
  TruthRow next = reference;
  next.timestamp = 3.0;
  const double reference_yaw = QuaternionYaw(reference.quaternion);
  next.position[0] += std::cos(reference_yaw);
  next.position[1] += std::sin(reference_yaw);
  const std::vector<TruthRow> truth = {stale, reference, next};
  std::vector<std::array<double, 9>> trajectory(2);
  trajectory[0][0] = 1.99;
  trajectory[0][7] = 1.0;
  trajectory[0][8] = 0.01;
  trajectory[1][0] = 3.0;
  trajectory[1][1] = 1.0;
  trajectory[1][7] = 1.0;
  trajectory[1][8] = 0.01;
  const auto points = MakeColumnMap(reference);

  const auto stale_result = EvaluateGeometry(points, truth.front());
  EXPECT_FALSE(stale_result.passes) << Describe(stale_result);
  const auto result = EvaluateGeometry(points, MapperOriginTruth(trajectory, truth));
  EXPECT_TRUE(result.passes) << Describe(result);
  EXPECT_EQ(&MapperOriginTruth(trajectory, truth), &truth[1]);

  AccuracyMetrics accuracy;
  std::string error;
  ASSERT_TRUE(ComputeAccuracy(trajectory, truth, 1.0, &accuracy, &error)) << error;
  EXPECT_NEAR(accuracy.ate_rmse_m, 0.0, 1e-12);
  EXPECT_NEAR(accuracy.translation_rpe_rmse_m, 0.0, 1e-12);
  EXPECT_NEAR(accuracy.yaw_rpe_rmse_rad, 0.0, 1e-12);
  EXPECT_NEAR(accuracy.path_scale, 1.0, 1e-12);
}

TEST(E2eQualityScoreGeometryTest, RejectsMapYAxisReflection) {
  const TruthRow reference = MakeReference();
  auto points = MakeColumnMap(reference);
  for (MapPoint& point : points)
    point[1] = -point[1];
  const auto result = EvaluateGeometry(points, reference);
  EXPECT_FALSE(result.passes) << Describe(result);
}

TEST(E2eQualityScoreGeometryTest, RejectsIndependentAxisScale) {
  const TruthRow reference = MakeReference();
  auto points = MakeColumnMap(reference);
  for (MapPoint& point : points) {
    point[0] *= 1.20F;
    point[1] *= 0.82F;
  }
  const auto result = EvaluateGeometry(points, reference);
  EXPECT_FALSE(result.passes) << Describe(result);
}

TEST(E2eQualityScoreGeometryTest, RejectsMapOnlyUniformScale) {
  const TruthRow reference = MakeReference();
  auto points = MakeColumnMap(reference);
  for (MapPoint& point : points) {
    point[0] *= 1.20F;
    point[1] *= 1.20F;
  }
  const auto result = EvaluateGeometry(points, reference);
  EXPECT_FALSE(result.passes) << Describe(result);
}

TEST(E2eQualityScoreGeometryTest, RejectsPersistentAccumulatedDrift) {
  const TruthRow reference = MakeReference();
  const auto rigid = MakeColumnMap(reference);
  std::vector<MapPoint> drifted;
  drifted.reserve(rigid.size() * 8);
  for (std::size_t step = 0; step < 8; ++step) {
    for (MapPoint point : rigid) {
      point[0] += 0.22F * static_cast<float>(step);
      point[1] -= 0.11F * static_cast<float>(step);
      drifted.push_back(point);
    }
  }
  const auto result = EvaluateGeometry(drifted, reference);
  EXPECT_FALSE(result.passes) << Describe(result);
}

}  // namespace
