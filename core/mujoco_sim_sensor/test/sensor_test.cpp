#include <mujoco_sim/sensor/sensor.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

#include "../src/internal.hpp"

namespace mujoco_sim::sensor {
namespace {

template <typename T>
std::vector<T> Decode(const RawFrame& frame) {
  EXPECT_EQ(frame.payload.size() % sizeof(T), 0U);
  std::vector<T> values(frame.payload.size() / sizeof(T));
  if (!values.empty()) {
    std::memcpy(values.data(), frame.payload.data(), frame.payload.size());
  }
  return values;
}

const RawFrame& FrameWithFormat(const std::vector<RawFrame>& frames,
                                mujoco_sim::v4::StreamFormat format) {
  const auto found = std::find_if(frames.begin(), frames.end(), [format](const RawFrame& frame) {
    return frame.header.format() == format;
  });
  EXPECT_NE(found, frames.end());
  return *found;
}

CameraProfile FullCameraProfile() {
  CameraProfile profile;
  profile.camera_name = "test_camera";
  profile.width = 64;
  profile.height = 48;
  profile.rate_hz = 30.0;
  profile.frame_id = "camera_optical";
  profile.outputs = {
      {mujoco_sim::v4::RGB8, "camera/rgb"},
      {mujoco_sim::v4::DEPTH32F, "camera/depth"},
      {mujoco_sim::v4::SEGMENTATION32U, "camera/segmentation"},
      {mujoco_sim::v4::XYZ32F, "camera/points"},
  };
  return profile;
}

RayLidarProfile LidarProfile() {
  RayLidarProfile profile;
  profile.site_name = "lidar_site";
  profile.horizontal_samples = 4;
  profile.vertical_angles_rad = {0.0};
  profile.min_range_m = 0.1;
  profile.max_range_m = 4.5;
  profile.rate_hz = 10.0;
  profile.frame_id = "lidar";
  profile.stream_name = "lidar/points";
  return profile;
}

CaptureMetadata TestMetadata() {
  CaptureMetadata metadata;
  metadata.epoch = 7;
  metadata.tick_id = 11;
  metadata.simulation_time_s = 0.25;
  metadata.stream_sequence = 13;
  metadata.capture_id = 17;
  metadata.dropped_since_previous = 2;
  return metadata;
}

class SensorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string error;
    const auto loaded = core_.LoadModel(MUJOCO_SIM_SENSOR_TEST_MODEL);
    ASSERT_TRUE(loaded.response.loaded) << loaded.response.message;
    snapshot_ = core_.CreateSnapshot(&error);
    ASSERT_NE(snapshot_, nullptr) << error;
  }

  core::SimulationCore core_;
  std::unique_ptr<core::SimulationSnapshot> snapshot_;
};

TEST(SensorShapeTest, AppliesRendererAndProtocolPayloadLimits) {
  StreamShape shape;
  std::string error;
  ASSERT_TRUE(StreamShapeFor(mujoco_sim::v4::RGB8, 64, 48, &shape, &error)) << error;
  EXPECT_EQ(shape.row_stride_bytes, 64U * 3U);
  EXPECT_EQ(shape.payload_bytes, 64U * 48U * 3U);

  EXPECT_FALSE(StreamShapeFor(mujoco_sim::v4::RGB8,
                              static_cast<std::uint32_t>(std::numeric_limits<int>::max()) + 1U, 1,
                              &shape, &error));
  EXPECT_NE(error.find("integer limits"), std::string::npos);

  EXPECT_FALSE(StreamShapeFor(mujoco_sim::v4::XYZ32F, 100'000U, 500U, &shape, &error));
  EXPECT_NE(error.find("payload limits"), std::string::npos);
}

TEST_F(SensorTest, DescriptorsExposeShapesConventionsAndLabels) {
  std::vector<mujoco_sim::v4::StreamDescriptor> camera_descriptors;
  std::string error;
  ASSERT_TRUE(DescribeCameraStreams(*snapshot_, FullCameraProfile(), &camera_descriptors, &error))
      << error;
  ASSERT_EQ(camera_descriptors.size(), 4U);
  EXPECT_EQ(camera_descriptors[0].format(), mujoco_sim::v4::RGB8);
  EXPECT_EQ(camera_descriptors[0].row_stride_bytes(), 64U * 3U);
  EXPECT_EQ(camera_descriptors[0].maximum_payload_bytes(), 64U * 48U * 3U);
  EXPECT_EQ(camera_descriptors[0].frame_convention(), mujoco_sim::v4::ROS_OPTICAL);
  EXPECT_EQ(camera_descriptors[0].topic(), "");
  EXPECT_GT(camera_descriptors[0].camera_intrinsics().fx(), 0.0);
  EXPECT_DOUBLE_EQ(camera_descriptors[0].camera_intrinsics().cx(), 31.5);

  CameraProfile intrinsic_profile = FullCameraProfile();
  intrinsic_profile.camera_name = "intrinsic_camera";
  ASSERT_TRUE(DescribeCameraStreams(*snapshot_, intrinsic_profile, &camera_descriptors, &error))
      << error;
  const auto& intrinsics = camera_descriptors[0].camera_intrinsics();
  EXPECT_NEAR(intrinsics.fx(), 64.0, 1e-5);
  EXPECT_NEAR(intrinsics.fy(), 72.0, 1e-5);
  EXPECT_NEAR(intrinsics.cx(), 28.3, 1e-5);
  EXPECT_NEAR(intrinsics.cy(), 25.9, 1e-5);

  ASSERT_TRUE(DescribeCameraStreams(*snapshot_, FullCameraProfile(), &camera_descriptors, &error))
      << error;
  const auto& segmentation = camera_descriptors[2];
  ASSERT_EQ(segmentation.segmentation_labels_size(), snapshot_->model()->ngeom);
  const int target_id = mj_name2id(snapshot_->model(), mjOBJ_GEOM, "camera_target");
  ASSERT_GE(target_id, 0);
  EXPECT_EQ(segmentation.segmentation_labels(target_id).value(),
            static_cast<std::uint32_t>(target_id + 1));
  EXPECT_EQ(segmentation.segmentation_labels(target_id).name(), "camera_target");

  mujoco_sim::v4::StreamDescriptor lidar_descriptor;
  ASSERT_TRUE(DescribeRayLidarStream(*snapshot_, LidarProfile(), &lidar_descriptor, &error))
      << error;
  EXPECT_EQ(lidar_descriptor.width(), 4U);
  EXPECT_EQ(lidar_descriptor.height(), 1U);
  EXPECT_EQ(lidar_descriptor.row_stride_bytes(), 4U * 3U * sizeof(float));
  EXPECT_EQ(lidar_descriptor.frame_convention(), mujoco_sim::v4::ROS_BODY);
}

TEST_F(SensorTest, CalibratedPrincipalPointMatchesRenderedTarget) {
  CameraProfile profile = FullCameraProfile();
  profile.camera_name = "intrinsic_camera";
  profile.outputs = {{mujoco_sim::v4::RGB8, "intrinsic/rgb"}};
  std::string error;
  auto renderer = OffscreenRenderer::Create(*snapshot_, profile, &error);
  ASSERT_NE(renderer, nullptr) << error;

  std::vector<RawFrame> frames;
  ASSERT_TRUE(renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
  const auto rgb = Decode<std::uint8_t>(frames.front());

  std::uint32_t minimum_column = profile.width;
  std::uint32_t maximum_column = 0;
  std::uint32_t minimum_row = profile.height;
  std::uint32_t maximum_row = 0;
  std::size_t target_pixels = 0;
  for (std::uint32_t row = 0; row < profile.height; ++row) {
    for (std::uint32_t column = 0; column < profile.width; ++column) {
      const std::size_t offset = 3U * (static_cast<std::size_t>(row) * profile.width + column);
      if (rgb[offset] <= 2U * rgb[offset + 1] || rgb[offset] <= 2U * rgb[offset + 2] ||
          rgb[offset] <= 40U) {
        continue;
      }
      minimum_column = std::min(minimum_column, column);
      maximum_column = std::max(maximum_column, column);
      minimum_row = std::min(minimum_row, row);
      maximum_row = std::max(maximum_row, row);
      ++target_pixels;
    }
  }

  ASSERT_GT(target_pixels, 20U);
  const auto& intrinsics = renderer->descriptors().front().camera_intrinsics();
  EXPECT_NEAR(0.5 * static_cast<double>(minimum_column + maximum_column), intrinsics.cx(), 0.75);
  EXPECT_NEAR(0.5 * static_cast<double>(minimum_row + maximum_row), intrinsics.cy(), 0.75);
}

TEST_F(SensorTest, CameraCaptureProducesAllConsistentOutputs) {
  std::string error;
  auto renderer = OffscreenRenderer::Create(*snapshot_, FullCameraProfile(), &error);
  ASSERT_NE(renderer, nullptr) << error;

  std::vector<RawFrame> frames;
  const CaptureMetadata metadata = TestMetadata();
  ASSERT_TRUE(renderer->Capture(*snapshot_, metadata, &frames, &error)) << error;
  ASSERT_EQ(frames.size(), 4U);
  for (const RawFrame& frame : frames) {
    EXPECT_EQ(frame.header.epoch(), metadata.epoch);
    EXPECT_EQ(frame.header.tick_id(), metadata.tick_id);
    EXPECT_DOUBLE_EQ(frame.header.simulation_time_s(), metadata.simulation_time_s);
    EXPECT_EQ(frame.header.stream_sequence(), metadata.stream_sequence);
    EXPECT_EQ(frame.header.capture_id(), metadata.capture_id);
    EXPECT_EQ(frame.header.dropped_since_previous(), metadata.dropped_since_previous);
    EXPECT_EQ(frame.header.payload_size(), frame.payload.size());
  }

  const auto rgb = Decode<std::uint8_t>(FrameWithFormat(frames, mujoco_sim::v4::RGB8));
  ASSERT_EQ(rgb.size(), 64U * 48U * 3U);
  const auto [minimum_rgb, maximum_rgb] = std::minmax_element(rgb.begin(), rgb.end());
  EXPECT_LT(*minimum_rgb, *maximum_rgb);
  EXPECT_GT(*maximum_rgb, 20U);

  const auto depth = Decode<float>(FrameWithFormat(frames, mujoco_sim::v4::DEPTH32F));
  const auto segmentation =
      Decode<std::uint32_t>(FrameWithFormat(frames, mujoco_sim::v4::SEGMENTATION32U));
  const auto points = Decode<float>(FrameWithFormat(frames, mujoco_sim::v4::XYZ32F));
  ASSERT_EQ(depth.size(), 64U * 48U);
  ASSERT_EQ(segmentation.size(), depth.size());
  ASSERT_EQ(points.size(), depth.size() * 3U);

  const std::size_t center = 24U * 64U + 32U;
  EXPECT_NEAR(depth[center], 2.5F, 0.08F);
  const int target_id = mj_name2id(snapshot_->model(), mjOBJ_GEOM, "camera_target");
  EXPECT_EQ(segmentation[center], static_cast<std::uint32_t>(target_id + 1));
  const int far_wall_id = mj_name2id(snapshot_->model(), mjOBJ_GEOM, "far_camera_wall");
  ASSERT_GE(far_wall_id, 0);
  EXPECT_EQ(segmentation.front(), static_cast<std::uint32_t>(far_wall_id + 1));
  EXPECT_EQ(segmentation[63], static_cast<std::uint32_t>(far_wall_id + 1));
  EXPECT_FLOAT_EQ(points[3U * center + 2U], depth[center]);
  EXPECT_TRUE(std::all_of(depth.begin(), depth.end(),
                          [](float value) { return std::isfinite(value) && value > 0.0F; }));

  std::vector<RawFrame> repeated;
  ASSERT_TRUE(renderer->Capture(*snapshot_, metadata, &repeated, &error)) << error;
  ASSERT_EQ(repeated.size(), frames.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    EXPECT_EQ(repeated[index].header.SerializeAsString(), frames[index].header.SerializeAsString());
    EXPECT_EQ(repeated[index].payload, frames[index].payload);
  }
}

TEST_F(SensorTest, IntrinsicCameraPointCloudReprojectsToSourcePixels) {
  CameraProfile profile = FullCameraProfile();
  profile.camera_name = "intrinsic_camera";
  profile.outputs = {{mujoco_sim::v4::DEPTH32F, "intrinsic/depth"},
                     {mujoco_sim::v4::XYZ32F, "intrinsic/points"}};
  std::string error;
  auto renderer = OffscreenRenderer::Create(*snapshot_, profile, &error);
  ASSERT_NE(renderer, nullptr) << error;

  std::vector<RawFrame> frames;
  ASSERT_TRUE(renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
  const auto points = Decode<float>(FrameWithFormat(frames, mujoco_sim::v4::XYZ32F));
  const auto& intrinsics = renderer->descriptors().front().camera_intrinsics();
  constexpr std::size_t kColumn = 17;
  constexpr std::size_t kRow = 11;
  const std::size_t index = kRow * profile.width + kColumn;
  const double x = points[3 * index];
  const double y = points[3 * index + 1];
  const double z = points[3 * index + 2];
  ASSERT_TRUE(std::isfinite(z));
  ASSERT_GT(z, 0.0);
  EXPECT_NEAR(intrinsics.fx() * x / z + intrinsics.cx(), kColumn, 1e-4);
  EXPECT_NEAR(intrinsics.fy() * y / z + intrinsics.cy(), kRow, 1e-4);
}

TEST_F(SensorTest, SegmentationUsesTheRenderedMonoCamera) {
  CameraProfile profile = FullCameraProfile();
  profile.camera_name = "wide_ipd_camera";
  profile.outputs = {{mujoco_sim::v4::RGB8, "wide/rgb"},
                     {mujoco_sim::v4::SEGMENTATION32U, "wide/segmentation"}};
  std::string error;
  auto renderer = OffscreenRenderer::Create(*snapshot_, profile, &error);
  ASSERT_NE(renderer, nullptr) << error;

  std::vector<RawFrame> frames;
  ASSERT_TRUE(renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
  const auto rgb = Decode<std::uint8_t>(FrameWithFormat(frames, mujoco_sim::v4::RGB8));
  const auto segmentation =
      Decode<std::uint32_t>(FrameWithFormat(frames, mujoco_sim::v4::SEGMENTATION32U));
  const int target_id = mj_name2id(snapshot_->model(), mjOBJ_GEOM, "camera_target");
  ASSERT_GE(target_id, 0);
  const std::uint32_t target_label = static_cast<std::uint32_t>(target_id + 1);

  double rgb_column_sum = 0.0;
  std::size_t rgb_count = 0;
  double label_column_sum = 0.0;
  std::size_t label_count = 0;
  for (std::size_t index = 0; index < segmentation.size(); ++index) {
    const std::size_t column = index % profile.width;
    const std::size_t rgb_offset = 3 * index;
    if (rgb[rgb_offset] > 2 * rgb[rgb_offset + 1] && rgb[rgb_offset] > 2 * rgb[rgb_offset + 2] &&
        rgb[rgb_offset] > 40) {
      rgb_column_sum += static_cast<double>(column);
      ++rgb_count;
    }
    if (segmentation[index] == target_label) {
      label_column_sum += static_cast<double>(column);
      ++label_count;
    }
  }
  ASSERT_GT(rgb_count, 20U);
  ASSERT_GT(label_count, 20U);
  EXPECT_NEAR(rgb_column_sum / static_cast<double>(rgb_count),
              label_column_sum / static_cast<double>(label_count), 0.75);
}

TEST_F(SensorTest, ForcedEglIsHeadlessAndRenders) {
  std::string error;
  OffscreenRendererOptions options;
  options.backend = BackendPreference::kEglOnly;
  CameraProfile profile = FullCameraProfile();
  profile.outputs = {{mujoco_sim::v4::RGB8, "egl/rgb"}};
  auto renderer = OffscreenRenderer::Create(*snapshot_, std::move(profile), options, &error);
  ASSERT_NE(renderer, nullptr) << error;
  EXPECT_TRUE(renderer->backend() == OffscreenBackend::kEglSurfaceless ||
              renderer->backend() == OffscreenBackend::kEglPbuffer);
  std::vector<RawFrame> frames;
  ASSERT_TRUE(renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
  EXPECT_EQ(frames.size(), 1U);
}

TEST_F(SensorTest, RecreatesEglRendererAfterDestroyingPreviousContext) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    std::string error;
    OffscreenRendererOptions options;
    options.backend = BackendPreference::kEglOnly;
    CameraProfile profile = FullCameraProfile();
    profile.outputs = {{mujoco_sim::v4::RGB8, "egl/recreated/rgb"}};
    auto renderer = OffscreenRenderer::Create(*snapshot_, std::move(profile), options, &error);
    ASSERT_NE(renderer, nullptr) << error;

    std::vector<RawFrame> frames;
    ASSERT_TRUE(renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_FALSE(frames.front().payload.empty());
  }
}

TEST_F(SensorTest, ConcurrentRendererCreationUsesOneProcessBackend) {
  auto create = [this](std::string stream_name) {
    CameraProfile profile = FullCameraProfile();
    profile.outputs = {{mujoco_sim::v4::RGB8, std::move(stream_name)}};
    std::string error;
    auto renderer = OffscreenRenderer::Create(*snapshot_, std::move(profile), &error);
    return std::make_pair(std::move(renderer), std::move(error));
  };

  auto first = std::async(std::launch::async, create, "concurrent/first");
  auto second = std::async(std::launch::async, create, "concurrent/second");
  auto [first_renderer, first_error] = first.get();
  auto [second_renderer, second_error] = second.get();
  ASSERT_NE(first_renderer, nullptr) << first_error;
  ASSERT_NE(second_renderer, nullptr) << second_error;
  EXPECT_EQ(first_renderer->backend(), second_renderer->backend());
}

TEST_F(SensorTest, DestroyingOneEglRendererKeepsOtherDisplayAlive) {
  auto create = [this](const std::string& stream_name) {
    CameraProfile profile = FullCameraProfile();
    profile.outputs = {{mujoco_sim::v4::RGB8, stream_name}};
    OffscreenRendererOptions options;
    options.backend = BackendPreference::kEglOnly;
    std::string error;
    auto renderer = OffscreenRenderer::Create(*snapshot_, std::move(profile), options, &error);
    return std::make_pair(std::move(renderer), std::move(error));
  };

  auto [first_renderer, first_error] = create("shared-display/first");
  auto [second_renderer, second_error] = create("shared-display/second");
  ASSERT_NE(first_renderer, nullptr) << first_error;
  ASSERT_NE(second_renderer, nullptr) << second_error;

  first_renderer.reset();
  std::vector<RawFrame> frames;
  std::string error;
  ASSERT_TRUE(second_renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_FALSE(frames.front().payload.empty());
}

TEST(SensorEncodingTest, NumericPayloadsUseLittleEndian) {
  const std::array<float, 1> floating = {2.5F};
  const auto float_bytes = detail::ToBytes<float>(floating);
  EXPECT_EQ(float_bytes, (std::vector<std::byte>{std::byte{0x00}, std::byte{0x00}, std::byte{0x20},
                                                 std::byte{0x40}}));

  const std::array<std::uint32_t, 1> integer = {0x12345678U};
  const auto integer_bytes = detail::ToBytes<std::uint32_t>(integer);
  EXPECT_EQ(integer_bytes, (std::vector<std::byte>{std::byte{0x78}, std::byte{0x56},
                                                   std::byte{0x34}, std::byte{0x12}}));
}

TEST_F(SensorTest, ForcedOsMesaIsHeadlessAndRenders) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    std::string error;
    OffscreenRendererOptions options;
    options.backend = BackendPreference::kOsMesaOnly;
    CameraProfile profile = FullCameraProfile();
    profile.outputs = {{mujoco_sim::v4::RGB8, "osmesa/rgb"}};
    auto renderer = OffscreenRenderer::Create(*snapshot_, std::move(profile), options, &error);
    if (renderer == nullptr && error.find("already initialized") != std::string::npos) {
      GTEST_SKIP() << error;
    }
    ASSERT_NE(renderer, nullptr) << error;
    EXPECT_EQ(renderer->backend(), OffscreenBackend::kOsMesa);
    std::vector<RawFrame> frames;
    ASSERT_TRUE(renderer->Capture(*snapshot_, TestMetadata(), &frames, &error)) << error;
    const auto rgb = Decode<std::uint8_t>(frames.front());
    EXPECT_NE(*std::min_element(rgb.begin(), rgb.end()), *std::max_element(rgb.begin(), rgb.end()));
  }
}

TEST_F(SensorTest, RayLidarReturnsKnownDistancesAndNanMisses) {
  std::string error;
  auto lidar = RayLidar::Create(*snapshot_, LidarProfile(), &error);
  ASSERT_NE(lidar, nullptr) << error;
  RawFrame frame;
  ASSERT_TRUE(lidar->Capture(*snapshot_, TestMetadata(), &frame, &error)) << error;
  const auto points = Decode<float>(frame);
  ASSERT_EQ(points.size(), 12U);
  EXPECT_NEAR(points[0], 2.5F, 1e-4F);
  EXPECT_NEAR(points[1], 0.0F, 1e-4F);
  EXPECT_NEAR(points[3], 0.0F, 1e-4F);
  EXPECT_NEAR(points[4], 3.0F, 1e-4F);
  EXPECT_NEAR(points[6], -2.0F, 1e-4F);
  EXPECT_NEAR(points[7], 0.0F, 1e-4F);
  EXPECT_TRUE(std::isnan(points[9]));
  EXPECT_TRUE(std::isnan(points[10]));
  EXPECT_TRUE(std::isnan(points[11]));
}

TEST_F(SensorTest, VerticalFovBuildsInclusiveOrganizedRings) {
  RayLidarProfile profile = LidarProfile();
  profile.vertical_angles_rad.clear();
  profile.vertical_fov = VerticalFieldOfView{3, -0.2, 0.3};
  mujoco_sim::v4::StreamDescriptor descriptor;
  std::string error;
  ASSERT_TRUE(DescribeRayLidarStream(*snapshot_, profile, &descriptor, &error)) << error;
  EXPECT_EQ(descriptor.width(), 4U);
  EXPECT_EQ(descriptor.height(), 3U);
  EXPECT_EQ(descriptor.maximum_payload_bytes(), 4U * 3U * 3U * 4U);
}

TEST_F(SensorTest, InvalidProfilesAreRejectedBeforeAllocation) {
  std::string error;
  std::vector<mujoco_sim::v4::StreamDescriptor> descriptors;
  CameraProfile camera = FullCameraProfile();
  camera.width = 0;
  EXPECT_FALSE(DescribeCameraStreams(*snapshot_, camera, &descriptors, &error));
  EXPECT_FALSE(error.empty());

  camera = FullCameraProfile();
  camera.camera_name = "missing";
  EXPECT_FALSE(DescribeCameraStreams(*snapshot_, camera, &descriptors, &error));
  camera = FullCameraProfile();
  camera.camera_name = "ortho_camera";
  EXPECT_FALSE(DescribeCameraStreams(*snapshot_, camera, &descriptors, &error));
  camera = FullCameraProfile();
  camera.outputs.push_back(camera.outputs.front());
  EXPECT_FALSE(DescribeCameraStreams(*snapshot_, camera, &descriptors, &error));
  EXPECT_TRUE(descriptors.empty());
  camera = FullCameraProfile();
  camera.width = std::numeric_limits<std::uint32_t>::max();
  EXPECT_FALSE(DescribeCameraStreams(*snapshot_, camera, &descriptors, &error));

  mujoco_sim::v4::StreamDescriptor descriptor;
  RayLidarProfile lidar = LidarProfile();
  lidar.vertical_fov = VerticalFieldOfView{2, -0.1, 0.1};
  EXPECT_FALSE(DescribeRayLidarStream(*snapshot_, lidar, &descriptor, &error));
  lidar = LidarProfile();
  lidar.vertical_angles_rad = {2.0};
  EXPECT_FALSE(DescribeRayLidarStream(*snapshot_, lidar, &descriptor, &error));
  lidar = LidarProfile();
  lidar.site_name = "missing";
  EXPECT_FALSE(DescribeRayLidarStream(*snapshot_, lidar, &descriptor, &error));
  lidar = LidarProfile();
  lidar.max_range_m = lidar.min_range_m;
  EXPECT_FALSE(DescribeRayLidarStream(*snapshot_, lidar, &descriptor, &error));
  lidar = LidarProfile();
  lidar.vertical_angles_rad.clear();
  lidar.vertical_fov = VerticalFieldOfView{std::numeric_limits<std::uint32_t>::max(), -0.1, 0.1};
  EXPECT_FALSE(DescribeRayLidarStream(*snapshot_, lidar, &descriptor, &error));
}

TEST_F(SensorTest, OptionalDiagnosticsMayBeOmittedOnFailure) {
  CameraProfile camera = FullCameraProfile();
  camera.camera_name = "missing";
  std::vector<mujoco_sim::v4::StreamDescriptor> descriptors;
  EXPECT_FALSE(DescribeCameraStreams(*snapshot_, camera, &descriptors, nullptr));
  EXPECT_EQ(OffscreenRenderer::Create(*snapshot_, std::move(camera), nullptr), nullptr);

  RayLidarProfile lidar_profile = LidarProfile();
  lidar_profile.site_name = "missing";
  EXPECT_EQ(RayLidar::Create(*snapshot_, std::move(lidar_profile), nullptr), nullptr);

  StreamShape shape;
  EXPECT_FALSE(StreamShapeFor(mujoco_sim::v4::STREAM_FORMAT_UNSPECIFIED, 1, 1, &shape, nullptr));
}

TEST_F(SensorTest, CaptureRejectsInvalidMetadataAndForeignSnapshot) {
  std::string error;
  auto lidar = RayLidar::Create(*snapshot_, LidarProfile(), &error);
  ASSERT_NE(lidar, nullptr) << error;
  CaptureMetadata metadata = TestMetadata();
  metadata.simulation_time_s = std::numeric_limits<double>::quiet_NaN();
  RawFrame frame;
  EXPECT_FALSE(lidar->Capture(*snapshot_, metadata, &frame, &error));

  core::SimulationCore other_core;
  const auto loaded = other_core.LoadModel(MUJOCO_SIM_SENSOR_TEST_MODEL);
  ASSERT_TRUE(loaded.response.loaded) << loaded.response.message;
  auto other_snapshot = other_core.CreateSnapshot(&error);
  ASSERT_NE(other_snapshot, nullptr) << error;
  EXPECT_FALSE(lidar->Capture(*other_snapshot, TestMetadata(), &frame, &error));
}

}  // namespace
}  // namespace mujoco_sim::sensor
