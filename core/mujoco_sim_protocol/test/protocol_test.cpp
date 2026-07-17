#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v4/sync.pb.h>

namespace {

std::uint8_t HexDigit(char digit) {
  if (digit >= '0' && digit <= '9') {
    return static_cast<std::uint8_t>(digit - '0');
  }
  return static_cast<std::uint8_t>(digit - 'a' + 10);
}

std::string BytesFromHex(std::string_view hex) {
  std::string bytes;
  bytes.reserve(hex.size() / 2);
  for (std::size_t index = 0; index < hex.size(); index += 2) {
    bytes.push_back(static_cast<char>((HexDigit(hex[index]) << 4U) | HexDigit(hex[index + 1])));
  }
  return bytes;
}

TEST(ProtocolV4Test, ExposesOnlyTheV4Contract) {
  EXPECT_EQ(mujoco_sim::protocol::kProtocolMajor, 4U);
  EXPECT_EQ(mujoco_sim::protocol::kProtocolMinor, 0U);

  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  EXPECT_NE(pool->FindMessageTypeByName("mujoco_sim.v4.RegisterAdapterRequest"), nullptr);
}

TEST(ProtocolTopicsTest, BuildsVersionedRegistryAndRuntimeNames) {
  EXPECT_EQ(mujoco_sim::protocol::RegistryServiceName("default"), "mujoco_sim.v4.default.registry");
  EXPECT_EQ(mujoco_sim::protocol::ControllerStateTopicName("default", "arm"),
            "mujoco_sim/v4/default/controller_state/arm");
  EXPECT_EQ(mujoco_sim::protocol::ControllerCommandTopicName("default"),
            "mujoco_sim/v4/default/controller_command");
  EXPECT_EQ(mujoco_sim::protocol::SessionHeartbeatTopicName("default"),
            "mujoco_sim/v4/default/session_heartbeat");
  EXPECT_EQ(mujoco_sim::protocol::StatusTopicName("default"), "mujoco_sim/v4/default/status");
  EXPECT_EQ(mujoco_sim::protocol::ObservationTopicName("default", "imu"),
            "mujoco_sim/v4/default/observation/imu");
  EXPECT_EQ(mujoco_sim::protocol::SensorStreamTopicName("default", "rgb"),
            "mujoco_sim/v4/default/sensor/rgb");
}

TEST(ProtocolSerializationTest, RoundTripsUnifiedAdapterContract) {
  mujoco_sim::v4::RegisterAdapterResponse response;
  response.set_code(mujoco_sim::v4::OK);
  response.set_adapter_id("arm");
  response.set_role(mujoco_sim::v4::CONTROLLER);
  response.set_session_id(42);
  response.set_epoch(7);
  response.set_state(mujoco_sim::v4::PROVISIONAL);
  response.mutable_layout()->set_layout_fingerprint(std::string(16, '\x2a'));
  auto* command = response.mutable_layout()->add_commands();
  auto* resource = command->mutable_resource();
  resource->set_name("hinge_motor");
  resource->set_type(mujoco_sim::v4::COMMAND_ACTUATOR);
  resource->set_dimension(1);
  resource->set_has_ctrl_range(true);
  resource->set_ctrl_min(-1.0);
  resource->set_ctrl_max(1.0);
  command->set_scalar_offset(0);
  response.mutable_layout()->set_command_scalar_count(1);

  mujoco_sim::v4::RegisterAdapterResponse parsed;
  ASSERT_TRUE(parsed.ParseFromString(response.SerializeAsString()));
  EXPECT_EQ(parsed.adapter_id(), "arm");
  EXPECT_EQ(parsed.role(), mujoco_sim::v4::CONTROLLER);
  EXPECT_EQ(parsed.layout().layout_fingerprint().size(), 16U);
  ASSERT_EQ(parsed.layout().commands_size(), 1);
  EXPECT_EQ(parsed.layout().commands(0).resource().name(), "hinge_motor");
  EXPECT_EQ(parsed.layout().command_scalar_count(), 1U);
}

TEST(ProtocolSerializationTest, RoundTripsUnifiedAdapterStatus) {
  mujoco_sim::v4::AdapterStatus status;
  status.set_adapter_id("arm");
  status.set_role(mujoco_sim::v4::CONTROLLER);
  status.set_session_id(12);
  status.set_state(mujoco_sim::v4::OFFLINE);
  status.set_last_command_sequence(19);
  status.set_last_delivery_tick_id(20);
  status.set_state_since_tick_id(20);
  status.set_timeout_count(2);

  mujoco_sim::v4::AdapterStatus parsed;
  ASSERT_TRUE(parsed.ParseFromString(status.SerializeAsString()));
  EXPECT_EQ(parsed.adapter_id(), "arm");
  EXPECT_EQ(parsed.state(), mujoco_sim::v4::OFFLINE);
  EXPECT_EQ(parsed.last_command_sequence(), 19U);
}

TEST(IdentifierTest, EnforcesTheEndpointGrammar) {
  std::string error;
  EXPECT_TRUE(mujoco_sim::protocol::ValidateIdentifier(
      "sim_01-test", mujoco_sim::protocol::IdentifierKind::kServer, &error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(mujoco_sim::protocol::ValidateIdentifier(
      std::string(64, 'a'), mujoco_sim::protocol::IdentifierKind::kAdapter, &error));
  const std::vector<std::string> invalid_identifiers = {
      "", "_arm", "arm/one", "arm.one", std::string(65, 'a'), "arm:x"};
  for (const auto& invalid : invalid_identifiers) {
    SCOPED_TRACE(invalid);
    EXPECT_FALSE(mujoco_sim::protocol::ValidateIdentifier(
        invalid, mujoco_sim::protocol::IdentifierKind::kAdapter, &error));
    EXPECT_FALSE(error.empty());
  }
}

TEST(LayoutHashTest, MatchesTheFixedEmptyLayoutVector) {
  mujoco_sim::v4::ResourceLayout layout;
  EXPECT_EQ(mujoco_sim::protocol::ComputeLayoutHash(layout),
            BytesFromHex("ee8033d25c155b1341ca165905404e5c"));
}

TEST(LayoutHashTest, CoversDescriptorFieldsAndOrder) {
  mujoco_sim::v4::ResourceLayout layout;
  auto* first = layout.add_observations();
  first->mutable_resource()->set_name("position");
  first->mutable_resource()->set_type(mujoco_sim::v4::OBSERVATION_SENSOR);
  first->mutable_resource()->set_dimension(1);
  first->set_scalar_offset(0);
  auto* second = layout.add_observations();
  second->mutable_resource()->set_name("velocity");
  second->mutable_resource()->set_type(mujoco_sim::v4::OBSERVATION_SENSOR);
  second->mutable_resource()->set_dimension(1);
  second->set_scalar_offset(1);
  layout.set_observation_scalar_count(2);
  auto* command = layout.add_commands();
  command->mutable_resource()->set_name("motor");
  command->mutable_resource()->set_type(mujoco_sim::v4::COMMAND_ACTUATOR);
  command->mutable_resource()->set_dimension(1);
  command->set_scalar_offset(0);
  layout.set_command_scalar_count(1);
  const std::string original = mujoco_sim::protocol::ComputeLayoutHash(layout);
  EXPECT_EQ(original.size(), mujoco_sim::protocol::kLayoutHashSize);

  auto changed = layout;
  changed.mutable_observations(0)->mutable_resource()->set_unit("rad");
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  changed = layout;
  changed.mutable_observations()->SwapElements(0, 1);
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  changed = layout;
  changed.mutable_observations(1)->set_scalar_offset(2);
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  changed = layout;
  changed.set_observation_scalar_count(3);
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  changed = layout;
  changed.mutable_commands(0)->set_scalar_offset(1);
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  changed = layout;
  changed.set_command_scalar_count(2);
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  layout.set_layout_fingerprint(original);
  std::string error;
  EXPECT_TRUE(mujoco_sim::protocol::ValidateLayoutHash(layout, &error));
  changed = layout;
  changed.mutable_observations(1)->set_scalar_offset(2);
  changed.set_layout_fingerprint(mujoco_sim::protocol::ComputeLayoutHash(changed));
  EXPECT_FALSE(mujoco_sim::protocol::ValidateLayoutHash(changed, &error));
  changed = layout;
  changed.set_observation_scalar_count(3);
  changed.set_layout_fingerprint(mujoco_sim::protocol::ComputeLayoutHash(changed));
  EXPECT_FALSE(mujoco_sim::protocol::ValidateLayoutHash(changed, &error));
  changed = layout;
  changed.mutable_commands(0)->set_scalar_offset(1);
  changed.set_layout_fingerprint(mujoco_sim::protocol::ComputeLayoutHash(changed));
  EXPECT_FALSE(mujoco_sim::protocol::ValidateLayoutHash(changed, &error));
  changed = layout;
  changed.mutable_commands(0)->mutable_resource()->set_dimension(0);
  changed.set_command_scalar_count(0);
  changed.set_layout_fingerprint(mujoco_sim::protocol::ComputeLayoutHash(changed));
  EXPECT_FALSE(mujoco_sim::protocol::ValidateLayoutHash(changed, &error));
  layout.set_layout_fingerprint(std::string(16, '\0'));
  EXPECT_FALSE(mujoco_sim::protocol::ValidateLayoutHash(layout, &error));
}

TEST(ProtocolSerializationTest, CarriesObserverLifecycleAndSimulationTime) {
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::CONTROLLER), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::OBSERVER), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::RGB8), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::DEPTH32F), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::SEGMENTATION32U), 3);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::XYZ32F), 4);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::STREAM_IDLE), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::STREAM_RUNNING), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v4::STREAM_ERROR), 3);

  mujoco_sim::v4::RegisterAdapterRequest request;
  request.set_adapter_id("state-observer");
  request.set_role(mujoco_sim::v4::OBSERVER);
  request.add_observation_names("imu_gyro");
  request.set_session_timeout_ms(1000);
  request.mutable_delivery()->set_every_n_steps(4);

  const std::string bytes = request.SerializeAsString();
  mujoco_sim::v4::RegisterAdapterRequest parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.role(), mujoco_sim::v4::OBSERVER);
  EXPECT_EQ(parsed.session_timeout_ms(), 1000U);
  EXPECT_EQ(parsed.delivery().every_n_steps(), 4U);
}

TEST(StreamFrameTest, RoundTripsHeaderAndViewsRawPayload) {
  mujoco_sim::v4::StreamFrameHeader header;
  header.set_epoch(2);
  header.set_tick_id(7);
  header.set_simulation_time_s(0.014);
  header.set_stream_name("front_rgb");
  header.set_stream_sequence(4);
  header.set_capture_id(3);
  header.set_format(mujoco_sim::v4::RGB8);
  header.set_width(2);
  header.set_height(1);
  header.set_row_stride_bytes(6);
  header.set_payload_size(6);
  const std::array<std::byte, 6> payload = {std::byte{1}, std::byte{2}, std::byte{3},
                                            std::byte{4}, std::byte{5}, std::byte{6}};

  std::string bytes;
  std::string error;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, std::span<const std::byte>(payload), &bytes, &error))
      << error;
  mujoco_sim::protocol::ParsedStreamFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error)) << error;
  EXPECT_EQ(parsed.header.stream_name(), "front_rgb");
  EXPECT_EQ(parsed.header.tick_id(), 7U);
  ASSERT_EQ(parsed.payload.size(), payload.size());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), parsed.payload.begin()));
}

TEST(StreamFrameTest, SupportsPayloadAliasingOutputBuffer) {
  std::string bytes(1024, '\0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<char>(index & 0xffU);
  }
  const std::string expected_payload = bytes;

  mujoco_sim::v4::StreamFrameHeader header;
  header.set_stream_name("aliased");
  header.set_format(mujoco_sim::v4::RGB8);
  header.set_width(1024);
  header.set_height(1);
  header.set_row_stride_bytes(1024);
  header.set_payload_size(bytes.size());
  const auto payload =
      std::span<const std::byte>(reinterpret_cast<const std::byte*>(bytes.data()), bytes.size());
  std::string error;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error)) << error;

  mujoco_sim::protocol::ParsedStreamFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error)) << error;
  ASSERT_EQ(parsed.payload.size(), expected_payload.size());
  EXPECT_TRUE(std::equal(parsed.payload.begin(), parsed.payload.end(),
                         reinterpret_cast<const std::byte*>(expected_payload.data())));
}

TEST(StreamFrameTest, SerializesFixedGoldenBytes) {
  mujoco_sim::v4::StreamFrameHeader header;
  header.set_stream_name("x");
  header.set_format(mujoco_sim::v4::RGB8);
  header.set_width(1);
  header.set_height(1);
  header.set_row_stride_bytes(3);
  header.set_payload_size(3);
  const std::array<std::byte, 3> payload = {std::byte{1}, std::byte{2}, std::byte{3}};

  std::string bytes;
  std::string error;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error)) << error;
  const std::string golden = BytesFromHex(
      "4d4a53545245414d010000000d0000000300000000000000"
      "12017828013001380140035803"
      "010203");
  EXPECT_EQ(bytes, golden);

  mujoco_sim::protocol::ParsedStreamFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(golden, &parsed, &error)) << error;
  EXPECT_EQ(parsed.header.SerializeAsString(), header.SerializeAsString());
  EXPECT_TRUE(
      std::equal(payload.begin(), payload.end(), parsed.payload.begin(), parsed.payload.end()));
}

TEST(StreamFrameTest, RejectsMalformedOrInconsistentFrames) {
  mujoco_sim::v4::StreamFrameHeader header;
  header.set_stream_name("depth");
  header.set_payload_size(2);
  const std::array<std::byte, 1> payload = {std::byte{1}};
  std::string bytes;
  std::string error;
  EXPECT_FALSE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  EXPECT_NE(error.find("payload_size"), std::string::npos);

  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(
      std::string_view("short"), static_cast<mujoco_sim::protocol::ParsedStreamFrame*>(nullptr),
      &error));

  header.set_payload_size(1);
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes[0] = 'X';
  mujoco_sim::protocol::ParsedStreamFrame parsed;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("magic"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes[8] = 2;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("major"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes[10] = 1;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("minor"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes[mujoco_sim::protocol::kStreamFramePrefixSize] = static_cast<char>(0x0f);
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("header"), std::string::npos);
  EXPECT_EQ(parsed.header.ByteSizeLong(), 0U);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes[12] = static_cast<char>(static_cast<unsigned char>(bytes[12]) + 1U);
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("size"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes[mujoco_sim::protocol::kStreamFramePrefixSize + header.ByteSizeLong() - 1] = 2;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("payload_size"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  bytes.pop_back();
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("size"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error));
  const std::uint64_t oversized = mujoco_sim::protocol::kMaximumStreamPayloadBytes + 1;
  for (std::size_t index = 0; index < sizeof(oversized); ++index) {
    bytes[16 + index] = static_cast<char>((oversized >> (8U * index)) & 0xffU);
  }
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("maximum"), std::string::npos);

  header.Clear();
  header.set_frame_id(std::string(mujoco_sim::protocol::kMaximumStreamHeaderBytes + 1U, 'x'));
  EXPECT_FALSE(mujoco_sim::protocol::SerializeStreamFrame(header, std::span<const std::byte>{},
                                                          &bytes, &error));
  EXPECT_NE(error.find("header exceeds maximum"), std::string::npos);

  header.Clear();
  header.set_payload_size(0);
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, std::span<const std::byte>{},
                                                         &bytes, &error));
  const std::uint32_t oversized_header = mujoco_sim::protocol::kMaximumStreamHeaderBytes + 1U;
  for (std::size_t index = 0; index < sizeof(oversized_header); ++index) {
    bytes[12 + index] = static_cast<char>((oversized_header >> (8U * index)) & 0xffU);
  }
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("header exceeds maximum"), std::string::npos);
}

TEST(DataFrameTest, SerializesFixedGoldenBytes) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerState;
  header.session_id = 1;
  header.epoch = 2;
  header.tick_id = 3;
  header.layout = {0x0102030405060708ULL, 0x1112131415161718ULL};
  header.sequence = 4;
  header.simulation_time_s = 1.5;
  const std::array scalars{-0.0, 2.25};
  mujoco_sim::protocol::DataFrameError error;
  std::string encoded;
  ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error))
      << error.message;

  const std::string golden = BytesFromHex(
      "4d4a44415441340002000000010050006000000002000000"
      "010000000000000002000000000000000300000000000000"
      "080706050403020118171615141312110400000000000000"
      "000000000000f83f00000000000000800000000000000240");
  EXPECT_EQ(encoded, golden);

  mujoco_sim::protocol::ParsedDataFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseDataFrame(std::string_view(golden), &parsed, &error))
      << error.message;
  EXPECT_EQ(parsed.header.kind, mujoco_sim::protocol::DataFrameKind::kControllerState);
  EXPECT_EQ(parsed.header.flags, 0U);
  EXPECT_EQ(parsed.header.session_id, 1U);
  EXPECT_EQ(parsed.header.epoch, 2U);
  EXPECT_EQ(parsed.header.tick_id, 3U);
  EXPECT_EQ(parsed.header.layout, header.layout);
  EXPECT_EQ(parsed.header.sequence, 4U);
  EXPECT_DOUBLE_EQ(parsed.header.simulation_time_s, 1.5);
  std::array<double, 2> decoded{};
  ASSERT_TRUE(mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded, &error))
      << error.message;
  EXPECT_EQ(std::bit_cast<std::uint64_t>(decoded[0]), std::bit_cast<std::uint64_t>(-0.0));
  EXPECT_DOUBLE_EQ(decoded[1], 2.25);
}

TEST(DataFrameTest, SerializesFixedControllerCommandGoldenBytes) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
  header.session_id = 0x0102030405060708ULL;
  header.epoch = 0x1112131415161718ULL;
  header.tick_id = 0x5152535455565758ULL;
  header.layout = {0x2122232425262728ULL, 0x3132333435363738ULL};
  header.sequence = 0x4142434445464748ULL;
  header.simulation_time_s = 123.5;
  const std::array scalars{-0.0, 2.25};
  mujoco_sim::protocol::DataFrameError error;
  std::string encoded;
  ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error))
      << error.message;

  const std::string golden = BytesFromHex(
      "4d4a44415441340002000000020040005000000002000000"
      "080706050403020118171615141312112827262524232221"
      "383736353433323148474645444342410000000000000080"
      "0000000000000240");
  EXPECT_EQ(encoded.size(),
            mujoco_sim::protocol::kCommandDataFrameHeaderSize + scalars.size() * sizeof(double));
  EXPECT_EQ(encoded, golden);

  mujoco_sim::protocol::ParsedDataFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseDataFrame(std::string_view(golden), &parsed, &error))
      << error.message;
  EXPECT_EQ(parsed.header.kind, mujoco_sim::protocol::DataFrameKind::kControllerCommand);
  EXPECT_EQ(parsed.header.flags, 0U);
  EXPECT_EQ(parsed.header.session_id, header.session_id);
  EXPECT_EQ(parsed.header.epoch, header.epoch);
  EXPECT_EQ(parsed.header.layout, header.layout);
  EXPECT_EQ(parsed.header.sequence, header.sequence);
  EXPECT_EQ(parsed.header.tick_id, 0U);
  EXPECT_DOUBLE_EQ(parsed.header.simulation_time_s, 0.0);
  std::array<double, 2> decoded{};
  ASSERT_TRUE(mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded, &error))
      << error.message;
  EXPECT_EQ(std::bit_cast<std::uint64_t>(decoded[0]), std::bit_cast<std::uint64_t>(-0.0));
  EXPECT_DOUBLE_EQ(decoded[1], 2.25);
}

TEST(DataFrameTest, RejectsHeaderSizesForTheWrongKind) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
  header.sequence = 1;
  mujoco_sim::protocol::DataFrameError error;
  std::string encoded;
  ASSERT_TRUE(
      mujoco_sim::protocol::EncodeDataFrame(header, std::span<const double>{}, &encoded, &error));
  ASSERT_EQ(encoded.size(), mujoco_sim::protocol::kCommandDataFrameHeaderSize);

  mujoco_sim::protocol::ParsedDataFrame parsed;
  encoded[12] = static_cast<char>(mujoco_sim::protocol::DataFrameKind::kControllerState);
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(encoded, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kHeaderSizeMismatch);

  header.kind = mujoco_sim::protocol::DataFrameKind::kObservation;
  ASSERT_TRUE(
      mujoco_sim::protocol::EncodeDataFrame(header, std::span<const double>{}, &encoded, &error));
  ASSERT_EQ(encoded.size(), mujoco_sim::protocol::kStateDataFrameHeaderSize);
  encoded[12] = static_cast<char>(mujoco_sim::protocol::DataFrameKind::kControllerCommand);
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(encoded, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kHeaderSizeMismatch);
}

TEST(DataFrameTest, RoundTripsObservationAndNanPayloadBits) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kObservation;
  header.session_id = 42;
  header.epoch = 7;
  header.tick_id = 99;
  header.layout = {11, 12};
  header.sequence = 5;
  header.simulation_time_s = 0.125;
  const double nan = std::bit_cast<double>(std::uint64_t{0x7ff8000000000042ULL});
  const std::array scalars{nan, std::numeric_limits<double>::infinity()};
  mujoco_sim::protocol::DataFrameError error;
  std::string encoded;
  ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error))
      << error.message;
  mujoco_sim::protocol::ParsedDataFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseDataFrame(encoded, &parsed, &error)) << error.message;
  EXPECT_EQ(parsed.header.kind, mujoco_sim::protocol::DataFrameKind::kObservation);
  EXPECT_EQ(parsed.header.sequence, 5U);
  std::array<double, 2> decoded{};
  ASSERT_TRUE(mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded, &error))
      << error.message;
  EXPECT_EQ(std::bit_cast<std::uint64_t>(decoded[0]), std::bit_cast<std::uint64_t>(nan));
  EXPECT_EQ(std::bit_cast<std::uint64_t>(decoded[1]), std::bit_cast<std::uint64_t>(scalars[1]));
}

TEST(DataFrameTest, RejectsInvalidSemanticsAndMalformedSizes) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
  header.session_id = 1;
  header.epoch = 2;
  header.tick_id = 3;
  header.sequence = 1;
  const std::array scalars{1.0};
  mujoco_sim::protocol::DataFrameError error;
  std::string encoded;
  ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error));

  mujoco_sim::protocol::ParsedDataFrame parsed;
  std::string malformed = encoded;
  malformed[0] = 'X';
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kMagicMismatch);

  malformed = encoded;
  malformed[12] = static_cast<char>(9);
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kInvalidKind);

  malformed = encoded;
  malformed[13] = static_cast<char>(1);
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kInvalidFlags);

  malformed = encoded;
  malformed[14] = 0;
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kHeaderSizeMismatch);

  malformed = encoded;
  malformed[16] = static_cast<char>(encoded.size() - 1);
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kFrameSizeMismatch);

  malformed = encoded;
  malformed[20] = 2;
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kScalarCountMismatch);

  header.sequence = 0;
  EXPECT_FALSE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kInvalidArgument);

  header = {};
  header.kind = mujoco_sim::protocol::DataFrameKind::kObservation;
  EXPECT_FALSE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kInvalidArgument);
}

TEST(DataFrameTest, HandlesUnalignedInputAndRejectsVersionAndExactSizes) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
  header.session_id = 9;
  header.epoch = 8;
  header.tick_id = 7;
  header.sequence = 1;
  const std::array scalars{1.0, -2.0};
  mujoco_sim::protocol::DataFrameError error;
  std::string encoded;
  ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded, &error));

  std::vector<std::byte> storage(encoded.size() + 1);
  std::copy(std::as_bytes(std::span(encoded)).begin(), std::as_bytes(std::span(encoded)).end(),
            storage.begin() + 1);
  mujoco_sim::protocol::ParsedDataFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseDataFrame(std::span<const std::byte>(storage).subspan(1),
                                                   &parsed, &error));
  std::array<double, 2> decoded{};
  ASSERT_TRUE(mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded, &error));
  EXPECT_EQ(decoded, scalars);

  std::string malformed = encoded;
  malformed[8] = 1;
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kVersionMismatch);

  malformed = encoded;
  malformed.pop_back();
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kFrameSizeMismatch);

  malformed = encoded;
  malformed.push_back('\0');
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(malformed, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kFrameSizeMismatch);

  std::vector<std::byte> oversized(mujoco_sim::protocol::kMaximumDataFrameBytes + 1U);
  EXPECT_FALSE(mujoco_sim::protocol::ParseDataFrame(oversized, &parsed, &error));
  EXPECT_EQ(error.code, mujoco_sim::protocol::DataFrameErrorCode::kFrameTooLarge);
}

TEST(DataFrameTest, ArbitraryInputCorpusNeverEscapesTheParser) {
  std::mt19937_64 random(0x4d4a53594e4333ULL);
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError error;
  for (std::size_t iteration = 0; iteration < 10000; ++iteration) {
    std::vector<std::byte> input(random() % 1025U);
    for (auto& value : input) {
      value = static_cast<std::byte>(random() & 0xffU);
    }
    static_cast<void>(mujoco_sim::protocol::ParseDataFrame(input, &parsed, &error));
  }
}

}  // namespace
