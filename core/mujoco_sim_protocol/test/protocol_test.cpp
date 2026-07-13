#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v2/sync.pb.h>

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
    bytes.push_back(static_cast<char>(
        (HexDigit(hex[index]) << 4U) | HexDigit(hex[index + 1])));
  }
  return bytes;
}

TEST(ProtocolV2Test, ExposesOnlyTheV2Contract) {
  EXPECT_EQ(mujoco_sim::protocol::kProtocolMajor, 2U);
  EXPECT_EQ(mujoco_sim::protocol::kProtocolMinor, 0U);

  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  EXPECT_NE(pool->FindMessageTypeByName("mujoco_sim.v2.RegisterAdapterRequest"),
            nullptr);
}

TEST(ProtocolTopicsTest, BuildsVersionedRegistryAndRuntimeNames) {
  EXPECT_EQ(mujoco_sim::protocol::RegistryServiceName("default"),
            "mujoco_sim.v2.default.registry");
  EXPECT_EQ(mujoco_sim::protocol::TickTopicName("default", "arm"),
            "mujoco_sim/v2/default/tick/arm");
  EXPECT_EQ(mujoco_sim::protocol::CommitTopicName("default"),
            "mujoco_sim/v2/default/commit");
  EXPECT_EQ(mujoco_sim::protocol::AdapterHeartbeatTopicName("default"),
            "mujoco_sim/v2/default/adapter_heartbeat");
  EXPECT_EQ(mujoco_sim::protocol::StatusTopicName("default"),
            "mujoco_sim/v2/default/status");
  EXPECT_EQ(mujoco_sim::protocol::ObservationTopicName("default", "imu"),
            "mujoco_sim/v2/default/observation/imu");
  EXPECT_EQ(mujoco_sim::protocol::SensorStreamTopicName("default", "rgb"),
            "mujoco_sim/v2/default/sensor/rgb");
}

TEST(ProtocolSerializationTest, RoundTripsUnifiedAdapterContract) {
  mujoco_sim::v2::RegisterAdapterResponse response;
  response.set_code(mujoco_sim::v2::OK);
  response.set_adapter_id("arm");
  response.set_role(mujoco_sim::v2::CONTROLLER);
  response.set_session_id(42);
  response.set_epoch(7);
  response.set_state(mujoco_sim::v2::PROVISIONAL);
  response.mutable_layout()->set_layout_hash(std::string(16, '\x2a'));
  auto* command = response.mutable_layout()->add_commands();
  command->set_name("hinge_motor");
  command->set_type(mujoco_sim::v2::COMMAND_ACTUATOR);
  command->set_dimension(1);
  command->set_has_ctrl_range(true);
  command->set_ctrl_min(-1.0);
  command->set_ctrl_max(1.0);

  mujoco_sim::v2::RegisterAdapterResponse parsed;
  ASSERT_TRUE(parsed.ParseFromString(response.SerializeAsString()));
  EXPECT_EQ(parsed.adapter_id(), "arm");
  EXPECT_EQ(parsed.role(), mujoco_sim::v2::CONTROLLER);
  EXPECT_EQ(parsed.layout().layout_hash().size(), 16U);
  ASSERT_EQ(parsed.layout().commands_size(), 1);
  EXPECT_EQ(parsed.layout().commands(0).name(), "hinge_motor");
}

TEST(ProtocolSerializationTest, RoundTripsUnifiedAdapterStatus) {
  mujoco_sim::v2::AdapterStatus status;
  status.set_adapter_id("arm");
  status.set_role(mujoco_sim::v2::CONTROLLER);
  status.set_session_id(12);
  status.set_state(mujoco_sim::v2::TIMED_OUT);
  status.set_last_heartbeat_tick_id(20);
  status.set_last_commit_tick_id(19);
  status.set_last_delivery_tick_id(20);
  status.set_state_since_tick_id(20);
  status.set_timeout_count(2);
  status.set_required_for_barrier(false);

  mujoco_sim::v2::AdapterStatus parsed;
  ASSERT_TRUE(parsed.ParseFromString(status.SerializeAsString()));
  EXPECT_EQ(parsed.adapter_id(), "arm");
  EXPECT_TRUE(parsed.has_last_heartbeat_tick_id());
  EXPECT_EQ(parsed.last_heartbeat_tick_id(), 20U);
  EXPECT_TRUE(parsed.has_last_commit_tick_id());
  EXPECT_EQ(parsed.last_commit_tick_id(), 19U);
  EXPECT_FALSE(parsed.required_for_barrier());
}

TEST(IdentifierTest, EnforcesTheV2EndpointGrammar) {
  std::string error;
  EXPECT_TRUE(mujoco_sim::protocol::ValidateIdentifier(
      "sim_01-test", mujoco_sim::protocol::IdentifierKind::kServer, &error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(mujoco_sim::protocol::ValidateIdentifier(
      std::string(64, 'a'), mujoco_sim::protocol::IdentifierKind::kAdapter,
      &error));
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
  mujoco_sim::v2::ResourceLayout layout;
  EXPECT_EQ(mujoco_sim::protocol::ComputeLayoutHash(layout),
            BytesFromHex("f307c38ce3de5a6a7ce9cfa2e60c5f7e"));
}

TEST(LayoutHashTest, CoversDescriptorFieldsAndOrder) {
  mujoco_sim::v2::ResourceLayout layout;
  auto* first = layout.add_observations();
  first->set_name("position");
  first->set_type(mujoco_sim::v2::OBSERVATION_SENSOR);
  first->set_dimension(1);
  auto* second = layout.add_observations();
  second->set_name("velocity");
  second->set_type(mujoco_sim::v2::OBSERVATION_SENSOR);
  second->set_dimension(1);
  const std::string original =
      mujoco_sim::protocol::ComputeLayoutHash(layout);
  EXPECT_EQ(original.size(), mujoco_sim::protocol::kLayoutHashSize);

  auto changed = layout;
  changed.mutable_observations(0)->set_unit("rad");
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  changed = layout;
  changed.mutable_observations()->SwapElements(0, 1);
  EXPECT_NE(mujoco_sim::protocol::ComputeLayoutHash(changed), original);

  layout.set_layout_hash(original);
  std::string error;
  EXPECT_TRUE(mujoco_sim::protocol::ValidateLayoutHash(layout, &error));
  layout.set_layout_hash(std::string(16, '\0'));
  EXPECT_FALSE(mujoco_sim::protocol::ValidateLayoutHash(layout, &error));
}

TEST(ProtocolSerializationTest, CarriesObserverLifecycleAndSimulationTime) {
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::CONTROLLER), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::OBSERVER), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::RGB8), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::DEPTH32F), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::SEGMENTATION32U), 3);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::XYZ32F), 4);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::STREAM_IDLE), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::STREAM_RUNNING), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v2::STREAM_ERROR), 3);

  mujoco_sim::v2::RegisterAdapterRequest request;
  request.set_server_name("default");
  request.set_adapter_id("state-observer");
  request.set_role(mujoco_sim::v2::OBSERVER);
  request.set_protocol_major(2);
  request.set_protocol_minor(0);
  request.add_observation_names("imu_gyro");
  request.mutable_delivery()->set_every_n_steps(4);

  const std::string bytes = request.SerializeAsString();
  mujoco_sim::v2::RegisterAdapterRequest parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.role(), mujoco_sim::v2::OBSERVER);
  EXPECT_EQ(parsed.delivery().every_n_steps(), 4U);

  mujoco_sim::v2::ObservationFrame frame;
  frame.mutable_header()->set_tick_id(10);
  frame.mutable_header()->set_simulation_time_s(0.02);
  frame.set_adapter_id("state-observer");
  frame.set_sequence(3);
  auto* block = frame.add_observations();
  block->set_name("imu_gyro");
  block->add_values(1.5);
  mujoco_sim::v2::ObservationFrame parsed_frame;
  ASSERT_TRUE(parsed_frame.ParseFromString(frame.SerializeAsString()));
  EXPECT_TRUE(parsed_frame.header().has_simulation_time_s());
  EXPECT_DOUBLE_EQ(parsed_frame.header().simulation_time_s(), 0.02);
  EXPECT_EQ(parsed_frame.sequence(), 3U);
  ASSERT_EQ(parsed_frame.observations_size(), 1);
  EXPECT_DOUBLE_EQ(parsed_frame.observations(0).values(0), 1.5);
}

TEST(StreamFrameTest, RoundTripsHeaderAndViewsRawPayload) {
  mujoco_sim::v2::StreamFrameHeader header;
  header.mutable_header()->set_server_name("default");
  header.mutable_header()->set_epoch(2);
  header.mutable_header()->set_tick_id(7);
  header.mutable_header()->set_simulation_time_s(0.014);
  header.set_stream_name("front_rgb");
  header.set_stream_sequence(4);
  header.set_capture_id(3);
  header.set_format(mujoco_sim::v2::RGB8);
  header.set_width(2);
  header.set_height(1);
  header.set_row_stride_bytes(6);
  header.set_payload_size(6);
  const std::array<std::byte, 6> payload = {
      std::byte{1}, std::byte{2}, std::byte{3},
      std::byte{4}, std::byte{5}, std::byte{6}};

  std::string bytes;
  std::string error;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, std::span<const std::byte>(payload), &bytes, &error))
      << error;
  mujoco_sim::protocol::ParsedStreamFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error))
      << error;
  EXPECT_EQ(parsed.header.stream_name(), "front_rgb");
  EXPECT_EQ(parsed.header.header().tick_id(), 7U);
  ASSERT_EQ(parsed.payload.size(), payload.size());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(),
                         parsed.payload.begin()));
}

TEST(StreamFrameTest, SupportsPayloadAliasingOutputBuffer) {
  std::string bytes(1024, '\0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<char>(index & 0xffU);
  }
  const std::string expected_payload = bytes;

  mujoco_sim::v2::StreamFrameHeader header;
  header.set_stream_name("aliased");
  header.set_format(mujoco_sim::v2::RGB8);
  header.set_width(1024);
  header.set_height(1);
  header.set_row_stride_bytes(1024);
  header.set_payload_size(bytes.size());
  const auto payload = std::span<const std::byte>(
      reinterpret_cast<const std::byte*>(bytes.data()), bytes.size());
  std::string error;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error))
      << error;

  mujoco_sim::protocol::ParsedStreamFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error))
      << error;
  ASSERT_EQ(parsed.payload.size(), expected_payload.size());
  EXPECT_TRUE(std::equal(
      parsed.payload.begin(), parsed.payload.end(),
      reinterpret_cast<const std::byte*>(expected_payload.data())));
}

TEST(StreamFrameTest, SerializesFixedGoldenBytes) {
  mujoco_sim::v2::StreamFrameHeader header;
  header.mutable_header()->set_server_name("s");
  header.mutable_header()->set_protocol_major(2);
  header.mutable_header()->set_protocol_minor(0);
  header.set_stream_name("x");
  header.set_format(mujoco_sim::v2::RGB8);
  header.set_width(1);
  header.set_height(1);
  header.set_row_stride_bytes(3);
  header.set_payload_size(3);
  const std::array<std::byte, 3> payload = {
      std::byte{1}, std::byte{2}, std::byte{3}};

  std::string bytes;
  std::string error;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error))
      << error;
  const std::string golden = BytesFromHex(
      "4d4a53545245414d01000000140000000300000000000000"
      "0a050a0173300212017828013001380140035803"
      "010203");
  EXPECT_EQ(bytes, golden);

  mujoco_sim::protocol::ParsedStreamFrame parsed;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(golden, &parsed, &error))
      << error;
  EXPECT_EQ(parsed.header.SerializeAsString(), header.SerializeAsString());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(),
                         parsed.payload.begin(), parsed.payload.end()));
}

TEST(StreamFrameTest, RejectsMalformedOrInconsistentFrames) {
  mujoco_sim::v2::StreamFrameHeader header;
  header.set_stream_name("depth");
  header.set_payload_size(2);
  const std::array<std::byte, 1> payload = {std::byte{1}};
  std::string bytes;
  std::string error;
  EXPECT_FALSE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  EXPECT_NE(error.find("payload_size"), std::string::npos);

  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(
      std::string_view("short"),
      static_cast<mujoco_sim::protocol::ParsedStreamFrame*>(nullptr),
      &error));

  header.set_payload_size(1);
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes[0] = 'X';
  mujoco_sim::protocol::ParsedStreamFrame parsed;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("magic"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes[8] = 2;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("major"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes[10] = 1;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("minor"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes[mujoco_sim::protocol::kStreamFramePrefixSize] =
      static_cast<char>(0x0f);
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("header"), std::string::npos);
  EXPECT_EQ(parsed.header.ByteSizeLong(), 0U);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes[12] = static_cast<char>(
      static_cast<unsigned char>(bytes[12]) + 1U);
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("size"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes[mujoco_sim::protocol::kStreamFramePrefixSize +
        header.ByteSizeLong() - 1] = 2;
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("payload_size"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  bytes.pop_back();
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("size"), std::string::npos);

  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, payload, &bytes, &error));
  const std::uint64_t oversized =
      mujoco_sim::protocol::kMaximumStreamPayloadBytes + 1;
  for (std::size_t index = 0; index < sizeof(oversized); ++index) {
    bytes[16 + index] = static_cast<char>((oversized >> (8U * index)) & 0xffU);
  }
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("maximum"), std::string::npos);

  header.Clear();
  header.set_frame_id(std::string(
      mujoco_sim::protocol::kMaximumStreamHeaderBytes + 1U, 'x'));
  EXPECT_FALSE(mujoco_sim::protocol::SerializeStreamFrame(
      header, std::span<const std::byte>{}, &bytes, &error));
  EXPECT_NE(error.find("header exceeds maximum"), std::string::npos);

  header.Clear();
  header.set_payload_size(0);
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, std::span<const std::byte>{}, &bytes, &error));
  const std::uint32_t oversized_header =
      mujoco_sim::protocol::kMaximumStreamHeaderBytes + 1U;
  for (std::size_t index = 0; index < sizeof(oversized_header); ++index) {
    bytes[12 + index] = static_cast<char>(
        (oversized_header >> (8U * index)) & 0xffU);
  }
  EXPECT_FALSE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error));
  EXPECT_NE(error.find("header exceeds maximum"), std::string::npos);
}

}  // namespace
