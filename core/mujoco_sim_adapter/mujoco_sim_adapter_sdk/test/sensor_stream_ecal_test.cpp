#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ecal/ecal.h>
#include <ecal/pubsub/publisher.h>
#include <ecal/service/server.h>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v2/sync.pb.h>

namespace {

using namespace std::chrono_literals;

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

class EcalScope {
 public:
  explicit EcalScope(const std::string& name)
      : initialized_(eCAL::Initialize(name)) {}

  ~EcalScope() {
    if (initialized_) eCAL::Finalize();
  }

  bool initialized() const { return initialized_; }

 private:
  bool initialized_ = false;
};

TEST(SensorStreamEcalTest, SameTickStreamErrorWakesWaiterAcrossTransport) {
  EcalScope ecal("mujoco-sim-stream-error-test");
  ASSERT_TRUE(ecal.initialized());

  const std::string server_name =
      "stream-error-" + std::to_string(
                            std::chrono::steady_clock::now()
                                .time_since_epoch()
                                .count());
  constexpr std::uint64_t kEpoch = 77;
  eCAL::CServiceServer registry(
      mujoco_sim::protocol::RegistryServiceName(server_name));
  ASSERT_TRUE(registry.SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kGetSensorStreamsMethod),
      [&server_name](const eCAL::SServiceMethodInformation&,
                     const std::string& request_bytes,
                     std::string& response_bytes) {
        mujoco_sim::v2::GetSensorStreamsRequest request;
        mujoco_sim::v2::GetSensorStreamsResponse response;
        if (!request.ParseFromString(request_bytes) ||
            request.server_name() != server_name) {
          response.set_code(mujoco_sim::v2::INVALID_ARGUMENT);
          response.set_message("invalid test discovery request");
        } else {
          response.set_code(mujoco_sim::v2::OK);
          response.set_epoch(kEpoch);
          auto* descriptor = response.add_streams();
          descriptor->set_name("rgb");
          descriptor->set_topic(mujoco_sim::protocol::SensorStreamTopicName(
              server_name, "rgb"));
          descriptor->set_format(mujoco_sim::v2::RGB8);
          descriptor->set_width(1);
          descriptor->set_height(1);
          descriptor->set_row_stride_bytes(3);
          descriptor->set_frame_id("camera");
          descriptor->set_frame_convention(mujoco_sim::v2::ROS_OPTICAL);
          descriptor->set_source_name("test_camera");
          descriptor->set_rate_hz(30.0);
          descriptor->set_maximum_payload_bytes(3);
        }
        return response.SerializeToString(&response_bytes) ? 0 : 1;
      }));

  eCAL::CPublisher status_publisher(
      mujoco_sim::protocol::StatusTopicName(server_name));
  mujoco_sim::adapter_sdk::SensorStreamOptions options;
  options.server_name = server_name;
  options.stream_names = {"rgb"};
  options.service_timeout_ms = 5000;
  mujoco_sim::adapter_sdk::SensorStreamClient client(std::move(options));
  std::string error;
  ASSERT_TRUE(client.Start(&error)) << error;

  const auto discovery_deadline = std::chrono::steady_clock::now() + 3s;
  while (status_publisher.GetSubscriberCount() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(status_publisher.GetSubscriberCount(), 0U);

  mujoco_sim::adapter_sdk::OwnedStreamSample sample;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForSample("rgb", 5s, &sample, &error);
  });

  mujoco_sim::v2::ServerStatus status;
  auto* header = status.mutable_header();
  header->set_server_name(server_name);
  header->set_epoch(kEpoch);
  header->set_tick_id(123);
  header->set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  header->set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  auto* stream = status.add_streams();
  stream->set_name("rgb");
  stream->set_state(mujoco_sim::v2::STREAM_RUNNING);
  ASSERT_TRUE(status_publisher.Send(status.SerializeAsString()));
  EXPECT_EQ(waiting.wait_for(100ms), std::future_status::timeout);

  stream->set_state(mujoco_sim::v2::STREAM_ERROR);
  stream->set_message("injected renderer failure");
  const auto error_deadline = std::chrono::steady_clock::now() + 3s;
  while (waiting.wait_for(0ms) != std::future_status::ready &&
         std::chrono::steady_clock::now() < error_deadline) {
    ASSERT_TRUE(status_publisher.Send(status.SerializeAsString()));
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_EQ(waiting.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(waiting.get(),
            mujoco_sim::adapter_sdk::SampleWaitResult::kError);
  EXPECT_EQ(error, "injected renderer failure");
  EXPECT_EQ(client.stream_diagnostic("rgb"), "injected renderer failure");
  client.Stop();
}

TEST(SensorStreamEcalTest, DestructionWaitsForRealInflightCallback) {
  EcalScope ecal("mujoco-sim-stream-teardown-test");
  ASSERT_TRUE(ecal.initialized());

  const std::string server_name =
      "stream-teardown-" +
      std::to_string(std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count());
  constexpr std::uint64_t kEpoch = 91;
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName(server_name, "rgb");
  eCAL::CServiceServer registry(
      mujoco_sim::protocol::RegistryServiceName(server_name));
  ASSERT_TRUE(registry.SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kGetSensorStreamsMethod),
      [&server_name, &topic](const eCAL::SServiceMethodInformation&,
                             const std::string& request_bytes,
                             std::string& response_bytes) {
        mujoco_sim::v2::GetSensorStreamsRequest request;
        mujoco_sim::v2::GetSensorStreamsResponse response;
        if (!request.ParseFromString(request_bytes) ||
            request.server_name() != server_name) {
          response.set_code(mujoco_sim::v2::INVALID_ARGUMENT);
        } else {
          response.set_code(mujoco_sim::v2::OK);
          response.set_epoch(kEpoch);
          auto* descriptor = response.add_streams();
          descriptor->set_name("rgb");
          descriptor->set_topic(topic);
          descriptor->set_format(mujoco_sim::v2::RGB8);
          descriptor->set_width(1);
          descriptor->set_height(1);
          descriptor->set_row_stride_bytes(3);
          descriptor->set_frame_id("camera");
          descriptor->set_frame_convention(mujoco_sim::v2::ROS_OPTICAL);
          descriptor->set_source_name("test_camera");
          descriptor->set_rate_hz(30.0);
          descriptor->set_maximum_payload_bytes(3);
        }
        return response.SerializeToString(&response_bytes) ? 0 : 1;
      }));

  eCAL::CPublisher frame_publisher(topic);
  mujoco_sim::adapter_sdk::SensorStreamOptions options;
  options.server_name = server_name;
  options.stream_names = {"rgb"};
  options.service_timeout_ms = 5000;
  auto client =
      std::make_unique<mujoco_sim::adapter_sdk::SensorStreamClient>(
          std::move(options));
  std::string error;
  ASSERT_TRUE(client->Start(&error)) << error;

  std::mutex callback_mutex;
  std::condition_variable callback_condition;
  bool callback_entered = false;
  bool callback_released = false;
  ASSERT_TRUE(client->SetCallback(
      "rgb",
      [&](const mujoco_sim::adapter_sdk::StreamSampleView&) {
        std::unique_lock lock(callback_mutex);
        callback_entered = true;
        callback_condition.notify_all();
        callback_condition.wait(lock, [&] { return callback_released; });
      },
      &error))
      << error;

  const auto discovery_deadline = std::chrono::steady_clock::now() + 3s;
  while (frame_publisher.GetSubscriberCount() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(frame_publisher.GetSubscriberCount(), 0U);

  mujoco_sim::v2::StreamFrameHeader header;
  header.mutable_header()->set_server_name(server_name);
  header.mutable_header()->set_epoch(kEpoch);
  header.mutable_header()->set_tick_id(1);
  header.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  header.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  header.set_stream_name("rgb");
  header.set_stream_sequence(1);
  header.set_capture_id(1);
  header.set_format(mujoco_sim::v2::RGB8);
  header.set_width(1);
  header.set_height(1);
  header.set_row_stride_bytes(3);
  header.set_frame_id("camera");
  header.set_frame_convention(mujoco_sim::v2::ROS_OPTICAL);
  header.set_payload_size(3);
  const std::vector<std::byte> payload = {
      std::byte{1}, std::byte{2}, std::byte{3}};
  std::string frame;
  ASSERT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(
      header, std::span<const std::byte>(payload), &frame, &error))
      << error;

  const auto callback_deadline = std::chrono::steady_clock::now() + 3s;
  std::unique_lock callback_lock(callback_mutex);
  while (!callback_entered &&
         std::chrono::steady_clock::now() < callback_deadline) {
    callback_lock.unlock();
    ASSERT_TRUE(frame_publisher.Send(frame));
    std::this_thread::sleep_for(20ms);
    callback_lock.lock();
  }
  ASSERT_TRUE(callback_entered);
  callback_lock.unlock();

  auto destroying = std::async(
      std::launch::async,
      [owned = std::move(client)]() mutable { owned.reset(); });
  EXPECT_EQ(destroying.wait_for(100ms), std::future_status::timeout);
  {
    std::lock_guard lock(callback_mutex);
    callback_released = true;
  }
  callback_condition.notify_all();
  ASSERT_EQ(destroying.wait_for(3s), std::future_status::ready);
  destroying.get();
}

}  // namespace
