#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v1/sync.pb.h>

namespace {

struct LegacyMessageFields {
  std::string_view name;
  std::vector<std::string_view> fields;
};

const std::vector<LegacyMessageFields>& V11MessageFields() {
  static const std::vector<LegacyMessageFields> fields = {
      {"Header",
       {"server_name", "session_id", "epoch", "tick_id",
        "timestamp_unix_nanos", "protocol_major", "protocol_minor"}},
      {"ResourceDescriptor",
       {"name", "type", "dimension", "mujoco_id", "mujoco_type",
        "mujoco_datatype", "has_ctrl_range", "ctrl_min", "ctrl_max"}},
      {"ResourceLayout", {"observations", "commands"}},
      {"GetAvailableResourcesRequest",
       {"server_name", "protocol_major", "protocol_minor"}},
      {"GetAvailableResourcesResponse",
       {"code", "message", "epoch", "effective_protocol_major",
        "effective_protocol_minor", "observations", "commands"}},
      {"RegisterControllerRequest",
       {"server_name", "controller_id", "protocol_major", "protocol_minor",
        "observation_names", "command_names", "timeout_ms"}},
      {"RegisterControllerResponse",
       {"code", "message", "controller_id", "session_id", "epoch",
        "effective_protocol_major", "effective_protocol_minor", "layout",
        "state"}},
      {"ActivateControllerRequest",
       {"controller_id", "session_id", "epoch"}},
      {"ActivateControllerResponse", {"code", "message", "state"}},
      {"UnregisterControllerRequest",
       {"controller_id", "session_id", "epoch"}},
      {"UnregisterControllerResponse", {"code", "message", "state"}},
      {"ObservationBlock", {"name", "values"}},
      {"ActuatorCommandBlock", {"name", "values"}},
      {"Tick",
       {"header", "controller_id", "layout", "observations",
        "controller_state"}},
      {"AdapterCommit",
       {"header", "controller_id", "status", "commands", "error_message"}},
      {"Heartbeat", {"header", "controller_id"}},
      {"ControllerStatus",
       {"controller_id", "session_id", "last_tick_id", "state", "message",
        "last_commit_tick_id", "state_since_tick_id", "timeout_count",
        "required_for_barrier"}},
      {"ServerStatus", {"header", "state", "message", "controllers"}},
  };
  return fields;
}

struct LegacyEnumValues {
  std::string_view name;
  std::vector<std::string_view> values;
};

const std::vector<LegacyEnumValues>& V11EnumValues() {
  static const std::vector<LegacyEnumValues> values = {
      {"ErrorCode",
       {"ERROR_CODE_UNSPECIFIED", "OK", "INVALID_ARGUMENT", "NOT_FOUND",
        "ALREADY_EXISTS", "VERSION_MISMATCH", "RESOURCE_CONFLICT",
        "INVALID_SESSION", "INVALID_EPOCH", "INVALID_TICK",
        "INVALID_COMMAND", "TIMEOUT", "FAILED_PRECONDITION",
        "INTERNAL_ERROR"}},
      {"ResourceType",
       {"RESOURCE_TYPE_UNSPECIFIED", "OBSERVATION_SENSOR",
        "COMMAND_ACTUATOR"}},
      {"ControllerState",
       {"CONTROLLER_STATE_UNSPECIFIED", "PROVISIONAL", "ACTIVE", "ERROR",
        "UNREGISTERED", "TIMED_OUT"}},
      {"ServerState",
       {"SERVER_STATE_UNSPECIFIED", "IDLE", "RUNNING", "ERROR_PAUSED",
        "STOPPED"}},
      {"CommitStatus",
       {"COMMIT_STATUS_UNSPECIFIED", "COMMIT_OK", "COMMIT_ERROR"}},
  };
  return values;
}

const std::vector<LegacyMessageFields>& V12MessageFields() {
  static const std::vector<LegacyMessageFields> fields = {
      {"Header",
       {"server_name", "session_id", "epoch", "tick_id",
        "timestamp_unix_nanos", "protocol_major", "protocol_minor",
        "simulation_time_s"}},
      {"ResourceDescriptor",
       {"name", "type", "dimension", "mujoco_id", "mujoco_type",
        "mujoco_datatype", "has_ctrl_range", "ctrl_min", "ctrl_max",
        "mujoco_stage", "interval_period_s", "interval_phase_s", "delay_s",
        "frame_id", "unit", "object_name", "object_type",
        "interpolation"}},
      {"ObservationDelivery", {"every_n_steps", "period_s"}},
      {"RegisterAdapterRequest",
       {"server_name", "adapter_id", "role", "protocol_major",
        "protocol_minor", "observation_names", "command_names", "timeout_ms",
        "delivery"}},
      {"RegisterAdapterResponse",
       {"code", "message", "adapter_id", "role", "session_id", "epoch",
        "effective_protocol_major", "effective_protocol_minor", "layout",
        "state"}},
      {"ActivateAdapterRequest",
       {"adapter_id", "role", "session_id", "epoch"}},
      {"ActivateAdapterResponse", {"code", "message", "state"}},
      {"UnregisterAdapterRequest",
       {"adapter_id", "role", "session_id", "epoch"}},
      {"UnregisterAdapterResponse", {"code", "message", "state"}},
      {"ObservationFrame",
       {"header", "observer_id", "sequence", "observations"}},
      {"AdapterHeartbeat",
       {"header", "adapter_id", "role", "last_observation_sequence"}},
      {"AdapterStatus",
       {"adapter_id", "role", "session_id", "state", "message",
        "state_since_tick_id", "timeout_count", "last_delivery_tick_id",
        "last_observation_sequence", "dropped_observation_frames"}},
      {"CameraIntrinsics", {"fx", "fy", "cx", "cy"}},
      {"SegmentationLabel",
       {"value", "mujoco_object_type", "mujoco_object_id", "name"}},
      {"StreamDescriptor",
       {"name", "topic", "format", "width", "height", "row_stride_bytes",
        "frame_id", "frame_convention", "source_name", "rate_hz",
        "maximum_payload_bytes", "camera_intrinsics",
        "segmentation_labels"}},
      {"GetSensorStreamsRequest",
       {"server_name", "protocol_major", "protocol_minor"}},
      {"GetSensorStreamsResponse",
       {"code", "message", "epoch", "effective_protocol_major",
        "effective_protocol_minor", "streams"}},
      {"StreamFrameHeader",
       {"header", "stream_name", "stream_sequence", "capture_id", "format",
        "width", "height", "row_stride_bytes", "frame_id",
        "frame_convention", "payload_size", "dropped_since_previous"}},
      {"StreamStatus",
       {"name", "state", "message", "subscriber_count", "captures",
        "published", "dropped", "publish_errors", "blocked_time_ms",
        "last_capture_tick_id"}},
      {"ServerStatus",
       {"header", "state", "message", "controllers", "adapters",
        "streams"}},
  };
  return fields;
}

const std::vector<LegacyEnumValues>& V12EnumValues() {
  static const std::vector<LegacyEnumValues> values = {
      {"AdapterRole",
       {"ADAPTER_ROLE_UNSPECIFIED", "CONTROLLER", "OBSERVER"}},
      {"StreamFormat",
       {"STREAM_FORMAT_UNSPECIFIED", "RGB8", "DEPTH32F",
        "SEGMENTATION32U", "XYZ32F"}},
      {"FrameConvention",
       {"FRAME_CONVENTION_UNSPECIFIED", "MUJOCO_NATIVE", "ROS_OPTICAL",
        "ROS_BODY"}},
      {"StreamState",
       {"STREAM_STATE_UNSPECIFIED", "STREAM_IDLE", "STREAM_RUNNING",
        "STREAM_ERROR"}},
  };
  return values;
}

struct GoldenMessage {
  std::string_view name;
  std::string_view hex;
};

constexpr std::array<GoldenMessage, 18> kV11GoldenMessages = {{
    {"Header", "0a0178100118012001280130013801"},
    {"ResourceDescriptor",
     "0a017810011801200128013001380141000000000000f03f"
     "49000000000000f03f"},
    {"ResourceLayout", "0a001200"},
    {"GetAvailableResourcesRequest", "0a017810011801"},
    {"GetAvailableResourcesResponse",
     "080112017818012001280132003a00"},
    {"RegisterControllerRequest",
     "0a0178120178180120012a01783201783801"},
    {"RegisterControllerResponse",
     "08011201781a0178200128013001380142004801"},
    {"ActivateControllerRequest", "0a017810011801"},
    {"ActivateControllerResponse", "08011201781801"},
    {"UnregisterControllerRequest", "0a017810011801"},
    {"UnregisterControllerResponse", "08011201781801"},
    {"ObservationBlock", "0a01781208000000000000f03f"},
    {"ActuatorCommandBlock", "0a01781208000000000000f03f"},
    {"Tick", "0a001201781a0022002801"},
    {"AdapterCommit", "0a00120178180122002a0178"},
    {"Heartbeat", "0a00120178"},
    {"ControllerStatus", "0a01781001180120012a01783001380140014801"},
    {"ServerStatus", "0a0010011a01782200"},
}};

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
    const auto value = static_cast<std::uint8_t>(
        (HexDigit(hex[index]) << 4U) | HexDigit(hex[index + 1]));
    bytes.push_back(static_cast<char>(value));
  }
  return bytes;
}

TEST(ProtocolV11CompatibilityTest, PreservesLegacyDescriptorNumbers) {
  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  std::size_t legacy_field_count = 0;
  for (const auto& message : V11MessageFields()) {
    SCOPED_TRACE(message.name);
    const auto* descriptor = pool->FindMessageTypeByName(
        "mujoco_sim.v1." + std::string(message.name));
    ASSERT_NE(descriptor, nullptr);
    for (std::size_t index = 0; index < message.fields.size(); ++index) {
      SCOPED_TRACE(message.fields[index]);
      const auto* field =
          descriptor->FindFieldByName(std::string(message.fields[index]));
      ASSERT_NE(field, nullptr);
      EXPECT_EQ(field->number(), static_cast<int>(index + 1));
      ++legacy_field_count;
    }
  }
  EXPECT_EQ(legacy_field_count, 85U);

  std::size_t legacy_enum_value_count = 0;
  for (const auto& legacy_enum : V11EnumValues()) {
    SCOPED_TRACE(legacy_enum.name);
    const auto* descriptor = pool->FindEnumTypeByName(
        "mujoco_sim.v1." + std::string(legacy_enum.name));
    ASSERT_NE(descriptor, nullptr);
    for (std::size_t index = 0; index < legacy_enum.values.size(); ++index) {
      SCOPED_TRACE(legacy_enum.values[index]);
      const auto* value =
          descriptor->FindValueByName(std::string(legacy_enum.values[index]));
      ASSERT_NE(value, nullptr);
      EXPECT_EQ(value->number(), static_cast<int>(index));
      ++legacy_enum_value_count;
    }
  }
  EXPECT_EQ(legacy_enum_value_count, 31U);
}

TEST(ProtocolV11CompatibilityTest, ParsesAndReemitsFixedGoldenBytes) {
  ASSERT_EQ(kV11GoldenMessages.size(), V11MessageFields().size());
  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  google::protobuf::DynamicMessageFactory factory;
  for (std::size_t index = 0; index < kV11GoldenMessages.size(); ++index) {
    const auto& golden = kV11GoldenMessages[index];
    SCOPED_TRACE(golden.name);
    EXPECT_EQ(golden.name, V11MessageFields()[index].name);
    const auto* descriptor = pool->FindMessageTypeByName(
        "mujoco_sim.v1." + std::string(golden.name));
    ASSERT_NE(descriptor, nullptr);
    const auto* prototype = factory.GetPrototype(descriptor);
    ASSERT_NE(prototype, nullptr);
    std::unique_ptr<google::protobuf::Message> message(prototype->New());
    const std::string golden_bytes = BytesFromHex(golden.hex);
    ASSERT_TRUE(message->ParseFromString(golden_bytes));
    EXPECT_EQ(message->GetReflection()
                  ->GetUnknownFields(*message)
                  .field_count(),
              0);
    EXPECT_EQ(message->SerializeAsString(), golden_bytes);
  }
}

TEST(ProtocolV12CompatibilityTest, LocksAllAddedFieldAndEnumNumbers) {
  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  std::size_t field_count = 0;
  for (const auto& message : V12MessageFields()) {
    SCOPED_TRACE(message.name);
    const auto* descriptor = pool->FindMessageTypeByName(
        "mujoco_sim.v1." + std::string(message.name));
    ASSERT_NE(descriptor, nullptr);
    ASSERT_EQ(descriptor->field_count(),
              static_cast<int>(message.fields.size()));
    for (std::size_t index = 0; index < message.fields.size(); ++index) {
      SCOPED_TRACE(message.fields[index]);
      const auto* field =
          descriptor->FindFieldByName(std::string(message.fields[index]));
      ASSERT_NE(field, nullptr);
      EXPECT_EQ(field->number(), static_cast<int>(index + 1));
      ++field_count;
    }
  }
  EXPECT_EQ(field_count, 137U);

  std::size_t enum_count = 0;
  for (const auto& expected : V12EnumValues()) {
    SCOPED_TRACE(expected.name);
    const auto* descriptor = pool->FindEnumTypeByName(
        "mujoco_sim.v1." + std::string(expected.name));
    ASSERT_NE(descriptor, nullptr);
    ASSERT_EQ(descriptor->value_count(),
              static_cast<int>(expected.values.size()));
    for (std::size_t index = 0; index < expected.values.size(); ++index) {
      const auto* value =
          descriptor->FindValueByName(std::string(expected.values[index]));
      ASSERT_NE(value, nullptr);
      EXPECT_EQ(value->number(), static_cast<int>(index));
      ++enum_count;
    }
  }
  EXPECT_EQ(enum_count, 16U);
}

TEST(ProtocolTopicsTest, BuildsRegistryAndRuntimeNames) {
  EXPECT_EQ(mujoco_sim::protocol::RegistryServiceName("default"),
            "mujoco_sim.default.registry");
  EXPECT_EQ(mujoco_sim::protocol::TickTopicName("default", "arm"),
            "mujoco_sim/default/tick/arm");
  EXPECT_EQ(mujoco_sim::protocol::CommitTopicName("default"),
            "mujoco_sim/default/commit");
  EXPECT_EQ(mujoco_sim::protocol::HeartbeatTopicName("default"),
            "mujoco_sim/default/heartbeat");
  EXPECT_EQ(mujoco_sim::protocol::StatusTopicName("default"),
            "mujoco_sim/default/status");
  EXPECT_EQ(mujoco_sim::protocol::ObservationTopicName("default", "imu"),
            "mujoco_sim/default/observation/imu");
  EXPECT_EQ(mujoco_sim::protocol::AdapterHeartbeatTopicName("default"),
            "mujoco_sim/default/adapter_heartbeat");
  EXPECT_EQ(mujoco_sim::protocol::SensorStreamTopicName("default", "rgb"),
            "mujoco_sim/default/sensor/rgb");
}

TEST(ProtocolSerializationTest, RoundTripsRegisterResponse) {
  mujoco_sim::v1::RegisterControllerResponse response;
  response.set_code(mujoco_sim::v1::OK);
  response.set_controller_id("arm");
  response.set_session_id(42);
  response.set_epoch(7);
  response.set_effective_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  response.set_effective_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  response.set_state(mujoco_sim::v1::PROVISIONAL);

  auto* command = response.mutable_layout()->add_commands();
  command->set_name("hinge_motor");
  command->set_type(mujoco_sim::v1::COMMAND_ACTUATOR);
  command->set_dimension(1);
  command->set_has_ctrl_range(true);
  command->set_ctrl_min(-1.0);
  command->set_ctrl_max(1.0);

  std::string bytes;
  ASSERT_TRUE(response.SerializeToString(&bytes));

  mujoco_sim::v1::RegisterControllerResponse parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.code(), mujoco_sim::v1::OK);
  EXPECT_EQ(parsed.controller_id(), "arm");
  EXPECT_EQ(parsed.session_id(), 42);
  ASSERT_EQ(parsed.layout().commands_size(), 1);
  EXPECT_EQ(parsed.layout().commands(0).name(), "hinge_motor");
  EXPECT_DOUBLE_EQ(parsed.layout().commands(0).ctrl_max(), 1.0);
}

TEST(ProtocolSerializationTest, CommitCarriesAtomicCommands) {
  mujoco_sim::v1::AdapterCommit commit;
  commit.mutable_header()->set_server_name("default");
  commit.mutable_header()->set_session_id(10);
  commit.mutable_header()->set_epoch(1);
  commit.mutable_header()->set_tick_id(5);
  commit.set_controller_id("arm");
  commit.set_status(mujoco_sim::v1::COMMIT_OK);
  auto* command = commit.add_commands();
  command->set_name("hinge_motor");
  command->add_values(0.25);

  std::string bytes;
  ASSERT_TRUE(commit.SerializeToString(&bytes));
  mujoco_sim::v1::AdapterCommit parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().tick_id(), 5);
  ASSERT_EQ(parsed.commands_size(), 1);
  EXPECT_DOUBLE_EQ(parsed.commands(0).values(0), 0.25);
}

TEST(ProtocolSerializationTest, CarriesDynamicControllerLifecycleStatus) {
  EXPECT_EQ(mujoco_sim::protocol::kProtocolMajor, 1U);
  EXPECT_EQ(mujoco_sim::protocol::kProtocolMinor, 2U);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::PROVISIONAL), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::ACTIVE), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::ERROR), 3);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::UNREGISTERED), 4);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::TIMED_OUT), 5);

  mujoco_sim::v1::ControllerStatus status;
  status.set_controller_id("arm");
  status.set_session_id(12);
  status.set_last_tick_id(20);
  status.set_state(mujoco_sim::v1::TIMED_OUT);
  status.set_message("controller timed out");
  status.set_last_commit_tick_id(19);
  status.set_state_since_tick_id(20);
  status.set_timeout_count(2);
  status.set_required_for_barrier(false);

  const std::string bytes = status.SerializeAsString();
  mujoco_sim::v1::ControllerStatus parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.state(), mujoco_sim::v1::TIMED_OUT);
  EXPECT_TRUE(parsed.has_last_commit_tick_id());
  EXPECT_EQ(parsed.last_commit_tick_id(), 19U);
  EXPECT_EQ(parsed.state_since_tick_id(), 20U);
  EXPECT_EQ(parsed.timeout_count(), 2U);
  EXPECT_FALSE(parsed.required_for_barrier());
}

TEST(ProtocolSerializationTest, CarriesObserverLifecycleAndSimulationTime) {
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::CONTROLLER), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::OBSERVER), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::RGB8), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::DEPTH32F), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::SEGMENTATION32U), 3);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::XYZ32F), 4);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::STREAM_IDLE), 1);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::STREAM_RUNNING), 2);
  EXPECT_EQ(static_cast<int>(mujoco_sim::v1::STREAM_ERROR), 3);

  mujoco_sim::v1::RegisterAdapterRequest request;
  request.set_server_name("default");
  request.set_adapter_id("state-observer");
  request.set_role(mujoco_sim::v1::OBSERVER);
  request.set_protocol_major(1);
  request.set_protocol_minor(2);
  request.add_observation_names("imu_gyro");
  request.mutable_delivery()->set_every_n_steps(4);

  const std::string bytes = request.SerializeAsString();
  mujoco_sim::v1::RegisterAdapterRequest parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.role(), mujoco_sim::v1::OBSERVER);
  EXPECT_EQ(parsed.delivery().every_n_steps(), 4U);

  mujoco_sim::v1::ObservationFrame frame;
  frame.mutable_header()->set_tick_id(10);
  frame.mutable_header()->set_simulation_time_s(0.02);
  frame.set_observer_id("state-observer");
  frame.set_sequence(3);
  auto* block = frame.add_observations();
  block->set_name("imu_gyro");
  block->add_values(1.5);
  mujoco_sim::v1::ObservationFrame parsed_frame;
  ASSERT_TRUE(parsed_frame.ParseFromString(frame.SerializeAsString()));
  EXPECT_TRUE(parsed_frame.header().has_simulation_time_s());
  EXPECT_DOUBLE_EQ(parsed_frame.header().simulation_time_s(), 0.02);
  EXPECT_EQ(parsed_frame.sequence(), 3U);
  ASSERT_EQ(parsed_frame.observations_size(), 1);
  EXPECT_DOUBLE_EQ(parsed_frame.observations(0).values(0), 1.5);
}

TEST(StreamFrameTest, RoundTripsHeaderAndViewsRawPayload) {
  mujoco_sim::v1::StreamFrameHeader header;
  header.mutable_header()->set_server_name("default");
  header.mutable_header()->set_epoch(2);
  header.mutable_header()->set_tick_id(7);
  header.mutable_header()->set_simulation_time_s(0.014);
  header.set_stream_name("front_rgb");
  header.set_stream_sequence(4);
  header.set_capture_id(3);
  header.set_format(mujoco_sim::v1::RGB8);
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

  mujoco_sim::v1::StreamFrameHeader header;
  header.set_stream_name("aliased");
  header.set_format(mujoco_sim::v1::RGB8);
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
  mujoco_sim::v1::StreamFrameHeader header;
  header.mutable_header()->set_server_name("s");
  header.mutable_header()->set_protocol_major(1);
  header.mutable_header()->set_protocol_minor(2);
  header.set_stream_name("x");
  header.set_format(mujoco_sim::v1::RGB8);
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
      "4d4a53545245414d01000000160000000300000000000000"
      "0a070a01733001380212017828013001380140035803"
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
  mujoco_sim::v1::StreamFrameHeader header;
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
