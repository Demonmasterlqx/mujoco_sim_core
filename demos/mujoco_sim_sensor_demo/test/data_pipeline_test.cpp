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

using Resource = std::pair<std::string, std::size_t>;

mujoco_sim::adapter_sdk::ControllerLayout MakeLayout(
    std::initializer_list<Resource> observation_specs,
    std::initializer_list<std::string> command_names = {}) {
  std::vector<mujoco_sim::adapter_sdk::ResourceInfo> observations, commands;
  std::size_t offset = 0;
  for (const auto& [name, count] : observation_specs) {
    observations.push_back({name, {offset, count}});
    offset += count;
  }
  std::size_t command_offset = 0;
  for (const auto& name : command_names) {
    commands.push_back({name, {command_offset++, 1}});
  }
  return {std::move(observations), std::move(commands), offset, command_offset};
}

void SetObservation(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                    const std::string& name, std::initializer_list<double> values,
                    std::vector<double>* output) {
  const auto span = layout.Observation(name);
  ASSERT_TRUE(span.has_value());
  ASSERT_EQ(span->count, values.size());
  std::copy(values.begin(), values.end(), output->begin() + span->offset);
}

demo::ControllerInterfaces ControllerNames() {
  return {{{"fl_pos", "fr_pos", "rl_pos", "rr_pos"}},
          "quat",
          "gyro",
          {{"fl_cmd", "fr_cmd", "rl_cmd", "rr_cmd"}}};
}

mujoco_sim::adapter_sdk::OwnedStreamSample MakeStreamSample(
    const std::string& name, mujoco_sim::v4::StreamFormat format,
    mujoco_sim::v4::FrameConvention convention, std::uint32_t width, std::uint32_t height,
    std::uint32_t stride, std::vector<std::byte> payload) {
  mujoco_sim::adapter_sdk::OwnedStreamSample sample;
  sample.header.set_epoch(2);
  sample.header.set_tick_id(3);
  sample.header.set_simulation_time_s(1.25);
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

mujoco_sim::v4::StreamDescriptor DescriptorFor(
    const mujoco_sim::adapter_sdk::OwnedStreamSample& sample) {
  mujoco_sim::v4::StreamDescriptor descriptor;
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

void AppendUint32LittleEndian(std::uint32_t value, std::vector<std::byte>* output) {
  const std::size_t offset = output->size();
  output->resize(offset + sizeof(value));
  for (std::size_t index = 0; index < sizeof(value); ++index) {
    (*output)[offset + index] = static_cast<std::byte>((value >> (8U * index)) & 0xffU);
  }
}

void AppendFloat32LittleEndian(float value, std::vector<std::byte>* output) {
  AppendUint32LittleEndian(std::bit_cast<std::uint32_t>(value), output);
}

std::filesystem::path TemporaryPng() {
  static std::atomic<std::uint64_t> sequence{0};
  return std::filesystem::temp_directory_path() /
         ("sensor_demo_rgb_" +
          std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
          std::to_string(sequence.fetch_add(1)) + ".png");
}

}  // namespace

TEST(DataPipelineTest, DecodesControllerStateByConfiguredNames) {
  const auto interfaces = ControllerNames();
  const auto layout = MakeLayout(
      {{"gyro", 3}, {"rr_pos", 1}, {"quat", 4}, {"fl_pos", 1}, {"rl_pos", 1}, {"fr_pos", 1}},
      {"fl_cmd", "fr_cmd", "rl_cmd", "rr_cmd"});
  std::vector<double> values(layout.observation_scalar_count());
  SetObservation(layout, "gyro", {0.1, 0.2, 0.3}, &values);
  SetObservation(layout, "rr_pos", {4.0}, &values);
  SetObservation(layout, "quat", {1.0, 0.0, 0.0, 0.0}, &values);
  SetObservation(layout, "fl_pos", {1.0}, &values);
  SetObservation(layout, "rl_pos", {3.0}, &values);
  SetObservation(layout, "fr_pos", {2.0}, &values);

  demo::VehicleSample sample;
  std::string error;
  const demo::VehicleDecoder decoder(layout, interfaces);
  ASSERT_TRUE(decoder.Decode(values, 4.5, &sample, &error)) << error;
  EXPECT_DOUBLE_EQ(sample.simulation_time, 4.5);
  EXPECT_EQ(sample.wheel_positions, (std::array<double, 4>{1, 2, 3, 4}));
  EXPECT_DOUBLE_EQ(sample.imu_gyro[2], 0.3);
  const demo::WheelCommandMapper mapper(layout, interfaces);
  std::vector<double> commands;
  ASSERT_TRUE(mapper.Map({5, 6, 7, 8}, &commands, &error)) << error;
  ASSERT_EQ(commands.size(), 4U);
  EXPECT_DOUBLE_EQ(commands[2], 7.0);
}

TEST(DataPipelineTest, ObservationDecodeFailureIsTransactional) {
  const auto layout = MakeLayout({{"fl_pos", 1}});
  std::vector<double> values{1.0};
  demo::VehicleSample sample;
  sample.simulation_time = 99.0;
  std::string error;
  const demo::VehicleDecoder decoder(layout, ControllerNames());
  EXPECT_FALSE(decoder.Decode(values, 1.0, &sample, &error));
  EXPECT_NE(error.find("mismatch"), std::string::npos);
  EXPECT_DOUBLE_EQ(sample.simulation_time, 99.0);
}

TEST(DataPipelineTest, DenseDecodersRejectInvalidSizeTimeAndOverlappingSpans) {
  const auto interfaces = ControllerNames();
  const auto layout = MakeLayout(
      {{"fl_pos", 1}, {"fr_pos", 1}, {"rl_pos", 1}, {"rr_pos", 1}, {"quat", 4}, {"gyro", 3}},
      {"fl_cmd", "fr_cmd", "rl_cmd", "rr_cmd"});
  std::vector<double> values(layout.observation_scalar_count());
  demo::VehicleSample sample;
  sample.simulation_time = 99.0;
  std::string error = "stale";
  const demo::VehicleDecoder decoder(layout, interfaces);

  values.pop_back();
  EXPECT_FALSE(decoder.Decode(values, 1.0, &sample, &error));
  EXPECT_NE(error.find("size mismatch"), std::string::npos);
  EXPECT_DOUBLE_EQ(sample.simulation_time, 99.0);

  values.resize(layout.observation_scalar_count());
  EXPECT_FALSE(decoder.Decode(values, std::numeric_limits<double>::infinity(), &sample, &error));
  EXPECT_NE(error.find("timestamp"), std::string::npos);
  EXPECT_DOUBLE_EQ(sample.simulation_time, 99.0);

  auto overlapping_observations = layout.observations();
  overlapping_observations[1].scalars.offset = overlapping_observations[0].scalars.offset;
  const mujoco_sim::adapter_sdk::ControllerLayout overlapping_observation_layout(
      overlapping_observations, layout.commands(), layout.observation_scalar_count(),
      layout.command_scalar_count());
  const demo::VehicleDecoder overlapping_decoder(overlapping_observation_layout, interfaces);
  EXPECT_FALSE(overlapping_decoder.Decode(values, 1.0, &sample, &error));
  EXPECT_NE(error.find("overlapping"), std::string::npos);
  EXPECT_DOUBLE_EQ(sample.simulation_time, 99.0);

  const mujoco_sim::adapter_sdk::ControllerLayout oversized_observation_layout(
      layout.observations(), layout.commands(), layout.observation_scalar_count() + 1,
      layout.command_scalar_count());
  const demo::VehicleDecoder oversized_decoder(oversized_observation_layout, interfaces);
  values.resize(oversized_observation_layout.observation_scalar_count());
  EXPECT_FALSE(oversized_decoder.Decode(values, 1.0, &sample, &error));
  EXPECT_NE(error.find("scalar count"), std::string::npos);
  EXPECT_DOUBLE_EQ(sample.simulation_time, 99.0);

  auto overlapping_commands = layout.commands();
  overlapping_commands[1].scalars.offset = overlapping_commands[0].scalars.offset;
  const mujoco_sim::adapter_sdk::ControllerLayout overlapping(
      layout.observations(), overlapping_commands, layout.observation_scalar_count(),
      layout.command_scalar_count());
  const demo::WheelCommandMapper mapper(overlapping, interfaces);
  std::vector<double> commands{42.0};
  EXPECT_FALSE(mapper.Map({1.0, 2.0, 3.0, 4.0}, &commands, &error));
  EXPECT_NE(error.find("overlapping"), std::string::npos);
  EXPECT_EQ(commands, (std::vector<double>{42.0}));
}

TEST(DataPipelineTest, DecodesObserverJointStateAndImuWithoutBarrierData) {
  demo::StateObserverInterfaces interfaces{{{"fl_pos", "fr_pos", "rl_pos", "rr_pos"}},
                                           {{"fl_vel", "fr_vel", "rl_vel", "rr_vel"}},
                                           "quat",
                                           "gyro",
                                           "accel"};
  const auto layout = MakeLayout({{"fl_pos", 1},
                                  {"fr_pos", 1},
                                  {"rl_pos", 1},
                                  {"rr_pos", 1},
                                  {"fl_vel", 1},
                                  {"fr_vel", 1},
                                  {"rl_vel", 1},
                                  {"rr_vel", 1},
                                  {"quat", 4},
                                  {"gyro", 3},
                                  {"accel", 3}});
  std::vector<double> values(layout.observation_scalar_count());
  for (const auto& [name, value] : std::array<std::pair<const char*, double>, 8>{{{"fl_pos", 1},
                                                                                  {"fr_pos", 2},
                                                                                  {"rl_pos", 3},
                                                                                  {"rr_pos", 4},
                                                                                  {"fl_vel", 5},
                                                                                  {"fr_vel", 6},
                                                                                  {"rl_vel", 7},
                                                                                  {"rr_vel", 8}}}) {
    SetObservation(layout, name, {value}, &values);
  }
  SetObservation(layout, "quat", {1, 0, 0, 0}, &values);
  SetObservation(layout, "gyro", {0.1, 0.2, 0.3}, &values);
  SetObservation(layout, "accel", {0, 0, 9.81}, &values);

  demo::StateSample state;
  std::string error;
  const demo::StateDecoder state_decoder(layout, interfaces);
  ASSERT_TRUE(state_decoder.Decode(values, 2.5, &state, &error)) << error;
  EXPECT_DOUBLE_EQ(state.simulation_time, 2.5);
  EXPECT_DOUBLE_EQ(state.wheel_velocities[3], 8.0);
  EXPECT_DOUBLE_EQ(state.imu_linear_acceleration[2], 9.81);

  demo::ImuOrientationSample orientation;
  const demo::ImuOrientationDecoder imu_decoder(layout, "quat");
  ASSERT_TRUE(imu_decoder.Decode(values, 2.5, &orientation, &error)) << error;
  EXPECT_DOUBLE_EQ(orientation.simulation_time, 2.5);
  EXPECT_EQ(orientation.quaternion, (std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
}

TEST(DataPipelineTest, SynchronizesNearestNormalizedImuOrientation) {
  demo::ImuSynchronizer synchronizer(3);
  std::string error;
  ASSERT_TRUE(synchronizer.Add({1.0, Eigen::Quaterniond(2, 0, 0, 0)}, &error));
  ASSERT_TRUE(synchronizer.Add({1.1, Eigen::Quaterniond(0, 0, 0, 2)}, &error));
  ASSERT_TRUE(synchronizer.Add({1.2, Eigen::Quaterniond(1, 0, 0, 0)}, &error));
  ASSERT_TRUE(synchronizer.Add({1.3, Eigen::Quaterniond(1, 0, 0, 0)}, &error));
  EXPECT_EQ(synchronizer.size(), 3U);
  const auto found = synchronizer.FindNearest(1.11, 0.02);
  ASSERT_TRUE(found.has_value());
  EXPECT_DOUBLE_EQ(found->simulation_time, 1.1);
  EXPECT_NEAR(found->orientation.norm(), 1.0, 1e-12);

  EXPECT_FALSE(synchronizer.Add({1.3, Eigen::Quaterniond::Identity()}, &error));
  EXPECT_EQ(synchronizer.size(), 3U);
  EXPECT_FALSE(synchronizer.FindNearest(2.0, 0.1).has_value());
}

TEST(DataPipelineTest, ValidatesAllCameraFormatsAndWritesRgbPng) {
  std::string error;
  demo::StreamValidationSummary summary;

  std::vector<std::byte> rgb = {std::byte{10}, std::byte{20}, std::byte{30},
                                std::byte{40}, std::byte{50}, std::byte{60}};
  auto rgb_sample = MakeStreamSample("rgb", mujoco_sim::v4::RGB8, mujoco_sim::v4::ROS_OPTICAL, 2, 1,
                                     6, std::move(rgb));
  auto descriptor = DescriptorFor(rgb_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(rgb_sample, descriptor, demo::CameraStreamKind::kRgb,
                                         &summary, &error))
      << error;

  std::vector<std::byte> depth;
  AppendFloat32LittleEndian(1.0F, &depth);
  AppendFloat32LittleEndian(2.0F, &depth);
  auto depth_sample = MakeStreamSample("depth", mujoco_sim::v4::DEPTH32F,
                                       mujoco_sim::v4::ROS_OPTICAL, 2, 1, 8, std::move(depth));
  descriptor = DescriptorFor(depth_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(depth_sample, descriptor, demo::CameraStreamKind::kDepth,
                                         &summary, &error))
      << error;

  std::vector<std::byte> segmentation;
  AppendUint32LittleEndian(0, &segmentation);
  AppendUint32LittleEndian(4, &segmentation);
  auto segmentation_sample =
      MakeStreamSample("seg", mujoco_sim::v4::SEGMENTATION32U, mujoco_sim::v4::ROS_OPTICAL, 2, 1, 8,
                       std::move(segmentation));
  descriptor = DescriptorFor(segmentation_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(segmentation_sample, descriptor,
                                         demo::CameraStreamKind::kSegmentation, &summary, &error))
      << error;

  std::vector<std::byte> points;
  for (float value : {0.0F, 0.0F, 1.0F, 0.1F, -0.1F, 2.0F}) {
    AppendFloat32LittleEndian(value, &points);
  }
  auto point_sample = MakeStreamSample("points", mujoco_sim::v4::XYZ32F,
                                       mujoco_sim::v4::ROS_OPTICAL, 2, 1, 24, std::move(points));
  descriptor = DescriptorFor(point_sample);
  ASSERT_TRUE(demo::ValidateCameraSample(point_sample, descriptor,
                                         demo::CameraStreamKind::kPointCloud, &summary, &error))
      << error;
  EXPECT_EQ(summary.sample_count, 4U);

  const auto png = TemporaryPng();
  ASSERT_TRUE(demo::SaveRgbScreenshot(png, rgb_sample, &error)) << error;
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  ASSERT_NE(png_image_begin_read_from_file(&image, png.string().c_str()), 0) << image.message;
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

  const std::uint32_t overflow_width = std::numeric_limits<std::uint32_t>::max() / 3U + 1U;
  auto width_overflow = MakeStreamSample("rgb", mujoco_sim::v4::RGB8, mujoco_sim::v4::ROS_OPTICAL,
                                         overflow_width, 1, 2, std::vector<std::byte>(2));
  std::string error;
  EXPECT_FALSE(demo::SaveRgbScreenshot(output, width_overflow, &error));
  EXPECT_NE(error.find("invalid shape"), std::string::npos);

  auto png_limit = MakeStreamSample("rgb", mujoco_sim::v4::RGB8, mujoco_sim::v4::ROS_OPTICAL,
                                    static_cast<std::uint32_t>(PNG_UINT_31_MAX) + 1U, 1, 3,
                                    std::vector<std::byte>(3));
  EXPECT_FALSE(demo::SaveRgbScreenshot(output, png_limit, &error));
  EXPECT_NE(error.find("libpng limits"), std::string::npos);

  auto oversized_payload =
      MakeStreamSample("rgb", mujoco_sim::v4::RGB8, mujoco_sim::v4::ROS_OPTICAL, 1,
                       static_cast<std::uint32_t>(PNG_UINT_31_MAX), 3, std::vector<std::byte>(3));
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
  for (float value :
       {1.0F, 2.0F, 3.0F, std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()}) {
    AppendFloat32LittleEndian(value, &points);
  }
  auto sample = MakeStreamSample("lidar", mujoco_sim::v4::XYZ32F, mujoco_sim::v4::ROS_BODY, 2, 1,
                                 24, std::move(points));
  auto descriptor = DescriptorFor(sample);
  demo::PointCloud cloud;
  std::string error;
  ASSERT_TRUE(demo::DecodeXyzPointCloud(sample, descriptor, &cloud, &error)) << error;
  ASSERT_EQ(cloud.size(), 1U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0F);
  EXPECT_TRUE(cloud.is_dense);

  descriptor.set_frame_convention(mujoco_sim::v4::ROS_OPTICAL);
  cloud[0].x = 42.0F;
  EXPECT_FALSE(demo::DecodeXyzPointCloud(sample, descriptor, &cloud, &error));
  EXPECT_FLOAT_EQ(cloud[0].x, 42.0F);
}

TEST(DataPipelineTest, DecodesNumericPayloadsAsLittleEndian) {
  std::vector<std::byte> segmentation = {std::byte{0x04}, std::byte{0x03}, std::byte{0x02},
                                         std::byte{0x01}};
  auto segmentation_sample =
      MakeStreamSample("seg", mujoco_sim::v4::SEGMENTATION32U, mujoco_sim::v4::ROS_OPTICAL, 1, 1, 4,
                       std::move(segmentation));
  auto descriptor = DescriptorFor(segmentation_sample);
  demo::StreamValidationSummary summary;
  std::string error;
  ASSERT_TRUE(demo::ValidateCameraSample(segmentation_sample, descriptor,
                                         demo::CameraStreamKind::kSegmentation, &summary, &error))
      << error;
  EXPECT_DOUBLE_EQ(summary.minimum_value, 0x01020304U);
  EXPECT_DOUBLE_EQ(summary.maximum_value, 0x01020304U);

  const std::vector<std::byte> point = {std::byte{0x00}, std::byte{0x00}, std::byte{0x80},
                                        std::byte{0x3f}, std::byte{0x00}, std::byte{0x00},
                                        std::byte{0x00}, std::byte{0x40}, std::byte{0x00},
                                        std::byte{0x00}, std::byte{0x40}, std::byte{0x40}};
  auto point_sample =
      MakeStreamSample("lidar", mujoco_sim::v4::XYZ32F, mujoco_sim::v4::ROS_BODY, 1, 1, 12, point);
  descriptor = DescriptorFor(point_sample);
  demo::PointCloud cloud;
  ASSERT_TRUE(demo::DecodeXyzPointCloud(point_sample, descriptor, &cloud, &error)) << error;
  ASSERT_EQ(cloud.size(), 1U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0F);
  EXPECT_FLOAT_EQ(cloud[0].y, 2.0F);
  EXPECT_FLOAT_EQ(cloud[0].z, 3.0F);
}
