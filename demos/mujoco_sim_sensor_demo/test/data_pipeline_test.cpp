#include <gtest/gtest.h>

#include <png.h>

#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <mujoco_sim/sensor_demo/imu_synchronizer.hpp>
#include <mujoco_sim/sensor_demo/interface_data.hpp>
#include <mujoco_sim/sensor_demo/stream_processing.hpp>

namespace demo = mujoco_sim::sensor_demo;
namespace {

void SetHeader(mujoco_sim::v2::Header* header, double time) {
  header->set_server_name("test");
  header->set_session_id(1);
  header->set_epoch(2);
  header->set_tick_id(3);
  header->set_protocol_major(1);
  header->set_protocol_minor(2);
  header->set_simulation_time_s(time);
}

void AddObservation(
    google::protobuf::RepeatedPtrField<mujoco_sim::v2::ObservationBlock>* blocks,
    const std::string& name, std::initializer_list<double> values) {
  auto* block = blocks->Add();
  block->set_name(name);
  for (double value : values) block->add_values(value);
}

demo::ControllerInterfaces ControllerNames() {
  return {{{"fl_pos", "fr_pos", "rl_pos", "rr_pos"}},
          "quat",
          "gyro",
          {{"fl_cmd", "fr_cmd", "rl_cmd", "rr_cmd"}}};
}

mujoco_sim::adapter_sdk::OwnedStreamSample MakeStreamSample(
    const std::string& name, mujoco_sim::v2::StreamFormat format,
    mujoco_sim::v2::FrameConvention convention, std::uint32_t width,
    std::uint32_t height, std::uint32_t stride,
    std::vector<std::byte> payload) {
  mujoco_sim::adapter_sdk::OwnedStreamSample sample;
  SetHeader(sample.header.mutable_header(), 1.25);
  sample.header.set_stream_name(name);
  sample.header.set_stream_sequence(1);
  sample.header.set_capture_id(2);
  sample.header.set_format(format);
  sample.header.set_width(width);
  sample.header.set_height(height);
  sample.header.set_row_stride_bytes(stride);
  sample.header.set_frame_id("frame");
  sample.header.set_frame_convention(convention);
  sample.header.set_payload_size(payload.size());
  sample.payload = std::move(payload);
  return sample;
}

mujoco_sim::v2::StreamDescriptor DescriptorFor(
    const mujoco_sim::adapter_sdk::OwnedStreamSample& sample) {
  mujoco_sim::v2::StreamDescriptor descriptor;
  descriptor.set_name(sample.header.stream_name());
  descriptor.set_format(sample.header.format());
  descriptor.set_width(sample.header.width());
  descriptor.set_height(sample.header.height());
  descriptor.set_row_stride_bytes(sample.header.row_stride_bytes());
  descriptor.set_frame_id(sample.header.frame_id());
  descriptor.set_frame_convention(sample.header.frame_convention());
  descriptor.set_rate_hz(10.0);
  descriptor.set_maximum_payload_bytes(sample.payload.size());
  return descriptor;
}

void AppendUint32LittleEndian(std::uint32_t value,
                              std::vector<std::byte>* output) {
  const std::size_t offset = output->size();
  output->resize(offset + sizeof(value));
  for (std::size_t index = 0; index < sizeof(value); ++index) {
    (*output)[offset + index] =
        static_cast<std::byte>((value >> (8U * index)) & 0xffU);
  }
}

void AppendFloat32LittleEndian(float value, std::vector<std::byte>* output) {
  AppendUint32LittleEndian(std::bit_cast<std::uint32_t>(value), output);
}

std::filesystem::path TemporaryPng() {
  static std::atomic<std::uint64_t> sequence{0};
  return std::filesystem::temp_directory_path() /
         ("sensor_demo_rgb_" +
          std::to_string(
              std::chrono::steady_clock::now().time_since_epoch().count()) +
          "_" + std::to_string(sequence.fetch_add(1)) + ".png");
}

}  // namespace

TEST(DataPipelineTest, DecodesControllerTickByConfiguredNames) {
  const auto interfaces = ControllerNames();
  mujoco_sim::v2::Tick tick;
  SetHeader(tick.mutable_header(), 4.5);
  AddObservation(tick.mutable_observations(), "gyro", {0.1, 0.2, 0.3});
  AddObservation(tick.mutable_observations(), "rr_pos", {4.0});
  AddObservation(tick.mutable_observations(), "quat", {1.0, 0.0, 0.0, 0.0});
  AddObservation(tick.mutable_observations(), "fl_pos", {1.0});
  AddObservation(tick.mutable_observations(), "rl_pos", {3.0});
  AddObservation(tick.mutable_observations(), "fr_pos", {2.0});

  demo::VehicleSample sample;
  std::string error;
  ASSERT_TRUE(demo::DecodeVehicleSample(tick, interfaces, &sample, &error))
      << error;
  EXPECT_DOUBLE_EQ(sample.simulation_time, 4.5);
  EXPECT_EQ(sample.wheel_positions, (std::array<double, 4>{1, 2, 3, 4}));
  EXPECT_DOUBLE_EQ(sample.imu_gyro[2], 0.3);
  const auto commands = demo::BuildWheelCommands(interfaces, {5, 6, 7, 8});
  ASSERT_EQ(commands.size(), 4U);
  EXPECT_EQ(commands[2].name, "rl_cmd");
  EXPECT_DOUBLE_EQ(commands[2].value, 7.0);
}

TEST(DataPipelineTest, ObservationDecodeFailureIsTransactional) {
  mujoco_sim::v2::Tick tick;
  SetHeader(tick.mutable_header(), 1.0);
  AddObservation(tick.mutable_observations(), "fl_pos", {1.0});
  demo::VehicleSample sample;
  sample.simulation_time = 99.0;
  std::string error;
  EXPECT_FALSE(
      demo::DecodeVehicleSample(tick, ControllerNames(), &sample, &error));
  EXPECT_NE(error.find("missing"), std::string::npos);
  EXPECT_DOUBLE_EQ(sample.simulation_time, 99.0);
}

TEST(DataPipelineTest, DecodesObserverJointStateAndImuWithoutBarrierData) {
  demo::StateObserverInterfaces interfaces{
      {{"fl_pos", "fr_pos", "rl_pos", "rr_pos"}},
      {{"fl_vel", "fr_vel", "rl_vel", "rr_vel"}},
      "quat",
      "gyro",
      "accel"};
  mujoco_sim::v2::ObservationFrame frame;
  SetHeader(frame.mutable_header(), 2.5);
  AddObservation(frame.mutable_observations(), "fl_pos", {1.0});
  AddObservation(frame.mutable_observations(), "fr_pos", {2.0});
  AddObservation(frame.mutable_observations(), "rl_pos", {3.0});
  AddObservation(frame.mutable_observations(), "rr_pos", {4.0});
  AddObservation(frame.mutable_observations(), "fl_vel", {5.0});
  AddObservation(frame.mutable_observations(), "fr_vel", {6.0});
  AddObservation(frame.mutable_observations(), "rl_vel", {7.0});
  AddObservation(frame.mutable_observations(), "rr_vel", {8.0});
  AddObservation(frame.mutable_observations(), "quat", {1.0, 0.0, 0.0, 0.0});
  AddObservation(frame.mutable_observations(), "gyro", {0.1, 0.2, 0.3});
  AddObservation(frame.mutable_observations(), "accel", {0.0, 0.0, 9.81});

  demo::StateSample state;
  std::string error;
  ASSERT_TRUE(
      demo::DecodeStateSample(frame, interfaces, &state, &error))
      << error;
  EXPECT_DOUBLE_EQ(state.simulation_time, 2.5);
  EXPECT_DOUBLE_EQ(state.wheel_velocities[3], 8.0);
  EXPECT_DOUBLE_EQ(state.imu_linear_acceleration[2], 9.81);

  demo::ImuOrientationSample orientation;
  ASSERT_TRUE(
      demo::DecodeImuOrientation(frame, "quat", &orientation, &error))
      << error;
  EXPECT_DOUBLE_EQ(orientation.simulation_time, 2.5);
  EXPECT_EQ(orientation.quaternion,
            (std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
}

TEST(DataPipelineTest, SynchronizesNearestNormalizedImuOrientation) {
  demo::ImuSynchronizer synchronizer(3);
  std::string error;
  ASSERT_TRUE(synchronizer.Add({1.0, Eigen::Quaterniond(2, 0, 0, 0)},
                               &error));
  ASSERT_TRUE(synchronizer.Add({1.1, Eigen::Quaterniond(0, 0, 0, 2)},
                               &error));
  ASSERT_TRUE(synchronizer.Add({1.2, Eigen::Quaterniond(1, 0, 0, 0)},
                               &error));
  ASSERT_TRUE(synchronizer.Add({1.3, Eigen::Quaterniond(1, 0, 0, 0)},
                               &error));
  EXPECT_EQ(synchronizer.size(), 3U);
  const auto found = synchronizer.FindNearest(1.11, 0.02);
  ASSERT_TRUE(found.has_value());
  EXPECT_DOUBLE_EQ(found->simulation_time, 1.1);
  EXPECT_NEAR(found->orientation.norm(), 1.0, 1e-12);

  EXPECT_FALSE(synchronizer.Add({1.3, Eigen::Quaterniond::Identity()},
                                &error));
  EXPECT_EQ(synchronizer.size(), 3U);
  EXPECT_FALSE(synchronizer.FindNearest(2.0, 0.1).has_value());
}

TEST(DataPipelineTest, ValidatesAllCameraFormatsAndWritesRgbPng) {
  std::string error;
  demo::StreamValidationSummary summary;

  std::vector<std::byte> rgb = {
      std::byte{10}, std::byte{20}, std::byte{30},
      std::byte{40}, std::byte{50}, std::byte{60}};
  auto rgb_sample = MakeStreamSample("rgb", mujoco_sim::v2::RGB8,
                                     mujoco_sim::v2::ROS_OPTICAL, 2, 1, 6,
                                     std::move(rgb));
  auto descriptor = DescriptorFor(rgb_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(
      rgb_sample, descriptor, demo::CameraStreamKind::kRgb, &summary, &error))
      << error;

  std::vector<std::byte> depth;
  AppendFloat32LittleEndian(1.0F, &depth);
  AppendFloat32LittleEndian(2.0F, &depth);
  auto depth_sample = MakeStreamSample(
      "depth", mujoco_sim::v2::DEPTH32F, mujoco_sim::v2::ROS_OPTICAL, 2, 1,
      8, std::move(depth));
  descriptor = DescriptorFor(depth_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(
      depth_sample, descriptor, demo::CameraStreamKind::kDepth, &summary,
      &error))
      << error;

  std::vector<std::byte> segmentation;
  AppendUint32LittleEndian(0, &segmentation);
  AppendUint32LittleEndian(4, &segmentation);
  auto segmentation_sample = MakeStreamSample(
      "seg", mujoco_sim::v2::SEGMENTATION32U,
      mujoco_sim::v2::ROS_OPTICAL, 2, 1, 8, std::move(segmentation));
  descriptor = DescriptorFor(segmentation_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(
      segmentation_sample, descriptor,
      demo::CameraStreamKind::kSegmentation, &summary, &error))
      << error;

  std::vector<std::byte> points;
  for (float value : {0.0F, 0.0F, 1.0F, 0.1F, -0.1F, 2.0F}) {
    AppendFloat32LittleEndian(value, &points);
  }
  auto point_sample = MakeStreamSample(
      "points", mujoco_sim::v2::XYZ32F, mujoco_sim::v2::ROS_OPTICAL, 2, 1,
      24, std::move(points));
  descriptor = DescriptorFor(point_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(
      point_sample, descriptor, demo::CameraStreamKind::kPointCloud,
      &summary, &error))
      << error;
  EXPECT_EQ(summary.sample_count, 4U);

  const auto png = TemporaryPng();
  ASSERT_TRUE(demo::SaveRgbScreenshot(png, rgb_sample, &error)) << error;
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(&image, png.string().c_str()), 0)
      << image.message;
  EXPECT_EQ(image.width, 2U);
  EXPECT_EQ(image.height, 1U);
  png_image_free(&image);
  std::error_code ignored;
  std::filesystem::remove(png, ignored);
}

TEST(DataPipelineTest, OversizedScreenshotShapesFailTransactionally) {
  const auto output = TemporaryPng();
  constexpr char kExistingContents[] = "existing screenshot";
  {
    std::ofstream file(output, std::ios::binary);
    ASSERT_TRUE(file.is_open());
    file << kExistingContents;
  }

  const std::uint32_t overflow_width =
      std::numeric_limits<std::uint32_t>::max() / 3U + 1U;
  auto width_overflow = MakeStreamSample(
      "rgb", mujoco_sim::v2::RGB8, mujoco_sim::v2::ROS_OPTICAL,
      overflow_width, 1, 2, std::vector<std::byte>(2));
  std::string error;
  EXPECT_FALSE(demo::SaveRgbScreenshot(output, width_overflow, &error));
  EXPECT_NE(error.find("invalid shape"), std::string::npos);

  auto png_limit = MakeStreamSample(
      "rgb", mujoco_sim::v2::RGB8, mujoco_sim::v2::ROS_OPTICAL,
      static_cast<std::uint32_t>(PNG_UINT_31_MAX) + 1U, 1, 3,
      std::vector<std::byte>(3));
  EXPECT_FALSE(demo::SaveRgbScreenshot(output, png_limit, &error));
  EXPECT_NE(error.find("libpng limits"), std::string::npos);

  auto oversized_payload = MakeStreamSample(
      "rgb", mujoco_sim::v2::RGB8, mujoco_sim::v2::ROS_OPTICAL, 1,
      static_cast<std::uint32_t>(PNG_UINT_31_MAX), 3,
      std::vector<std::byte>(3));
  EXPECT_FALSE(demo::SaveRgbScreenshot(output, oversized_payload, &error));
  EXPECT_NE(error.find("invalid shape"), std::string::npos);

  std::ifstream file(output, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  const std::string contents((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, kExistingContents);
  std::error_code ignored;
  std::filesystem::remove(output, ignored);
}

TEST(DataPipelineTest, LidarDecodeDropsNanMissesAndIsTransactional) {
  std::vector<std::byte> points;
  for (float value : {1.0F, 2.0F, 3.0F,
                      std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::quiet_NaN()}) {
    AppendFloat32LittleEndian(value, &points);
  }
  auto sample = MakeStreamSample("lidar", mujoco_sim::v2::XYZ32F,
                                 mujoco_sim::v2::ROS_BODY, 2, 1, 24,
                                 std::move(points));
  auto descriptor = DescriptorFor(sample);
  demo::PointCloud cloud;
  std::string error;
  ASSERT_TRUE(demo::DecodeXyzPointCloud(sample, descriptor, &cloud, &error))
      << error;
  ASSERT_EQ(cloud.size(), 1U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0F);
  EXPECT_TRUE(cloud.is_dense);

  descriptor.set_frame_convention(mujoco_sim::v2::ROS_OPTICAL);
  cloud[0].x = 42.0F;
  EXPECT_FALSE(
      demo::DecodeXyzPointCloud(sample, descriptor, &cloud, &error));
  EXPECT_FLOAT_EQ(cloud[0].x, 42.0F);
}

TEST(DataPipelineTest, DecodesNumericPayloadsAsLittleEndian) {
  std::vector<std::byte> segmentation = {
      std::byte{0x04}, std::byte{0x03}, std::byte{0x02}, std::byte{0x01}};
  auto segmentation_sample = MakeStreamSample(
      "seg", mujoco_sim::v2::SEGMENTATION32U,
      mujoco_sim::v2::ROS_OPTICAL, 1, 1, 4, std::move(segmentation));
  auto descriptor = DescriptorFor(segmentation_sample);
  demo::StreamValidationSummary summary;
  std::string error;
  ASSERT_TRUE(demo::ValidateCameraSample(
      segmentation_sample, descriptor,
      demo::CameraStreamKind::kSegmentation, &summary, &error))
      << error;
  EXPECT_DOUBLE_EQ(summary.minimum_value, 0x01020304U);
  EXPECT_DOUBLE_EQ(summary.maximum_value, 0x01020304U);

  const std::vector<std::byte> point = {
      std::byte{0x00}, std::byte{0x00}, std::byte{0x80}, std::byte{0x3f},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x40},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x40}, std::byte{0x40}};
  auto point_sample = MakeStreamSample(
      "lidar", mujoco_sim::v2::XYZ32F, mujoco_sim::v2::ROS_BODY, 1, 1, 12,
      point);
  descriptor = DescriptorFor(point_sample);
  demo::PointCloud cloud;
  ASSERT_TRUE(
      demo::DecodeXyzPointCloud(point_sample, descriptor, &cloud, &error))
      << error;
  ASSERT_EQ(cloud.size(), 1U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0F);
  EXPECT_FLOAT_EQ(cloud[0].y, 2.0F);
  EXPECT_FLOAT_EQ(cloud[0].z, 3.0F);
}
