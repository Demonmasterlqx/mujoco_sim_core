#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <future>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>

#include "../src/internal/sensor_stream_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {

class SensorStreamClientTestPeer {
 public:
  static bool ValidateOptions(SensorStreamClient* client, std::string* error) {
    return client->impl_->ValidateOptions(error);
  }

  static bool ValidateDescriptor(SensorStreamClient* client,
                                 const mujoco_sim::v4::StreamDescriptor& descriptor,
                                 std::string* error) {
    return client->impl_->ValidateDescriptor(descriptor, error);
  }

  static void Configure(SensorStreamClient* client,
                        const std::vector<mujoco_sim::v4::StreamDescriptor>& descriptors) {
    client->impl_->stop_requested_ = false;
    {
      std::lock_guard lock(client->impl_->mutex_);
      client->impl_->discovery_.Clear();
      client->impl_->discovery_.set_code(mujoco_sim::v4::OK);
      client->impl_->discovery_.set_epoch(9);
      client->impl_->streams_.clear();
      client->impl_->last_status_tick_id_.reset();
      for (const auto& descriptor : descriptors) {
        *client->impl_->discovery_.add_streams() = descriptor;
        auto state = std::make_shared<SensorStreamClient::Impl::StreamState>();
        state->descriptor = descriptor;
        client->impl_->streams_.emplace(descriptor.name(), std::move(state));
      }
      client->impl_->phase_ = SensorStreamClient::Impl::Phase::kActive;
      client->impl_->monitor_ecal_ = false;
    }
    {
      std::lock_guard lock(client->impl_->callback_state_->mutex);
      client->impl_->callback_state_->client = client->impl_.get();
      client->impl_->callback_state_->enabled = true;
    }
  }

  static void HandleFrame(SensorStreamClient* client, const std::string& stream_name,
                          const mujoco_sim::v4::StreamFrameHeader& header,
                          std::span<const std::byte> payload) {
    mujoco_sim::protocol::ParsedStreamFrame frame{header, payload};
    std::string error;
    if (client->impl_->IsExpectedFrame(stream_name, frame, &error)) {
      client->impl_->HandleStreamFrame(stream_name, frame);
    } else {
      client->impl_->RecordDiagnostic(stream_name, error);
    }
  }

  static void HandleBytes(SensorStreamClient* client, const std::string& stream_name,
                          const std::string& bytes) {
    client->impl_->HandleStreamBytes(stream_name, bytes.data(), bytes.size());
  }

  static void HandleStatus(SensorStreamClient* client, const mujoco_sim::v4::ServerStatus& status) {
    const std::string bytes = status.SerializeAsString();
    client->impl_->HandleStatusBytes(bytes.data(), bytes.size());
  }

  static void HandleStatusBytes(SensorStreamClient* client, const std::string& bytes) {
    client->impl_->HandleStatusBytes(bytes.data(), bytes.size());
  }

  static std::function<bool()> MakePendingCallback(SensorStreamClient* client) {
    const auto state = client->impl_->callback_state_;
    return [state] {
      SensorStreamClient::Impl::CallbackLease callback(state);
      return callback.client() != nullptr;
    };
  }

  static std::function<bool()> MakePendingStatusCallback(
      SensorStreamClient* client, const mujoco_sim::v4::ServerStatus& status) {
    const auto state = client->impl_->callback_state_;
    const std::string bytes = status.SerializeAsString();
    return [state, bytes] {
      SensorStreamClient::Impl::CallbackLease callback(state);
      if (callback.client() == nullptr) {
        return false;
      }
      callback.client()->HandleStatusBytes(bytes.data(), bytes.size());
      return true;
    };
  }

  static void HoldCallback(SensorStreamClient* client, std::promise<void>* entered,
                           std::shared_future<void> release) {
    SensorStreamClient::Impl::CallbackLease callback(client->impl_->callback_state_);
    entered->set_value();
    release.wait();
  }

  static void StopSampleReception(SensorStreamClient* client) {
    client->impl_->StopSampleReception();
  }

  static void StopFromCallback(SensorStreamClient* client) {
    SensorStreamClient::Impl::CallbackLease callback(client->impl_->callback_state_);
    ASSERT_NE(callback.client(), nullptr);
    callback.client()->StopSampleReception();
  }

  static void DestroyFromCallback(std::unique_ptr<SensorStreamClient>* client) {
    const auto state = (*client)->impl_->callback_state_;
    SensorStreamClient::Impl::CallbackLease callback(state);
    ASSERT_NE(callback.client(), nullptr);
    client->reset();
  }

  static bool WaitForSampleReceptionDisabled(SensorStreamClient* client,
                                             std::chrono::milliseconds timeout) {
    const auto state = client->impl_->callback_state_;
    std::unique_lock lock(state->mutex);
    return state->condition.wait_for(lock, timeout,
                                     [&] { return !state->enabled && state->client == nullptr; });
  }

  static bool WaitInProgress(SensorStreamClient* client, const std::string& stream_name) {
    std::lock_guard lock(client->impl_->mutex_);
    const auto found = client->impl_->streams_.find(stream_name);
    return found != client->impl_->streams_.end() && found->second->samples.wait_in_progress();
  }
};

}  // namespace mujoco_sim::adapter_sdk

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::OwnedStreamSample;
using mujoco_sim::adapter_sdk::SampleWaitResult;
using mujoco_sim::adapter_sdk::SensorStreamClient;
using mujoco_sim::adapter_sdk::SensorStreamClientTestPeer;

mujoco_sim::adapter_sdk::SensorStreamOptions MakeOptions(std::size_t queue_capacity = 1) {
  mujoco_sim::adapter_sdk::SensorStreamOptions options;
  options.server_name = "default";
  options.stream_names = {"rgb"};
  options.queue_capacity = queue_capacity;
  return options;
}

mujoco_sim::v4::StreamDescriptor MakeDescriptor(const std::string& name = "rgb") {
  mujoco_sim::v4::StreamDescriptor descriptor;
  descriptor.set_name(name);
  descriptor.set_topic(mujoco_sim::protocol::SensorStreamTopicName("default", name));
  descriptor.set_format(mujoco_sim::v4::RGB8);
  descriptor.set_width(2);
  descriptor.set_height(2);
  descriptor.set_row_stride_bytes(6);
  descriptor.set_frame_id("camera");
  descriptor.set_frame_convention(mujoco_sim::v4::ROS_OPTICAL);
  descriptor.set_source_name("main_camera");
  descriptor.set_rate_hz(30.0);
  descriptor.set_maximum_payload_bytes(12);
  return descriptor;
}

mujoco_sim::v4::StreamFrameHeader MakeHeader(std::uint64_t sequence) {
  mujoco_sim::v4::StreamFrameHeader header;
  header.set_epoch(9);
  header.set_tick_id(50 + sequence);
  header.set_stream_name("rgb");
  header.set_stream_sequence(sequence);
  header.set_capture_id(1000 + sequence);
  header.set_format(mujoco_sim::v4::RGB8);
  header.set_width(2);
  header.set_height(2);
  header.set_row_stride_bytes(6);
  header.set_frame_id("camera");
  header.set_frame_convention(mujoco_sim::v4::ROS_OPTICAL);
  header.set_payload_size(12);
  return header;
}

mujoco_sim::v4::ServerStatus MakeStatus(std::uint64_t tick_id, const std::string& stream_name,
                                        mujoco_sim::v4::StreamState state,
                                        std::string message = {}) {
  mujoco_sim::v4::ServerStatus status;
  status.set_epoch(9);
  status.set_tick_id(tick_id);
  status.set_state(mujoco_sim::v4::RUNNING);
  auto* stream = status.add_streams();
  stream->set_name(stream_name);
  stream->set_state(state);
  stream->set_message(std::move(message));
  return status;
}

std::vector<std::byte> MakePayload(std::uint8_t first = 1) {
  std::vector<std::byte> payload(12);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::byte>(first + i);
  }
  return payload;
}

std::string Serialize(const mujoco_sim::v4::StreamFrameHeader& header,
                      std::span<const std::byte> payload) {
  std::string bytes;
  std::string error;
  EXPECT_TRUE(mujoco_sim::protocol::SerializeStreamFrame(header, payload, &bytes, &error)) << error;
  return bytes;
}

void Deliver(SensorStreamClient* client, std::uint64_t sequence, std::vector<std::byte>* payload) {
  SensorStreamClientTestPeer::HandleFrame(client, "rgb", MakeHeader(sequence), *payload);
}

TEST(SensorStreamClientTest, ValidatesOptionsAndStaticDescriptors) {
  SensorStreamClient client(MakeOptions());
  std::string error;
  EXPECT_TRUE(SensorStreamClientTestPeer::ValidateOptions(&client, &error));
  EXPECT_TRUE(SensorStreamClientTestPeer::ValidateDescriptor(&client, MakeDescriptor(), &error));

  auto invalid = MakeDescriptor();
  invalid.set_row_stride_bytes(5);
  EXPECT_FALSE(SensorStreamClientTestPeer::ValidateDescriptor(&client, invalid, &error));
  EXPECT_EQ(error, "sensor stream row stride is too small: rgb");

  auto duplicate_options = MakeOptions();
  duplicate_options.stream_names.push_back("rgb");
  SensorStreamClient duplicate_client(duplicate_options);
  EXPECT_FALSE(SensorStreamClientTestPeer::ValidateOptions(&duplicate_client, &error));
}

TEST(SensorStreamClientTest, QueueDropsOldestAndIgnoresStaleSequences) {
  SensorStreamClient client(MakeOptions(2));
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  auto payload = MakePayload();
  Deliver(&client, 1, &payload);
  Deliver(&client, 2, &payload);
  Deliver(&client, 1, &payload);
  Deliver(&client, 3, &payload);
  EXPECT_EQ(client.dropped_samples("rgb"), 1U);

  OwnedStreamSample sample;
  std::string error;
  ASSERT_EQ(client.WaitForSample("rgb", 0ms, &sample, &error), SampleWaitResult::kReceived);
  EXPECT_EQ(sample.header.stream_sequence(), 2U);
  ASSERT_EQ(client.WaitForSample("rgb", 0ms, &sample, &error), SampleWaitResult::kReceived);
  EXPECT_EQ(sample.header.stream_sequence(), 3U);
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &sample, &error), SampleWaitResult::kTimeout);
}

TEST(SensorStreamClientTest, MalformedAndDescriptorMismatchedFramesAreIgnored) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  SensorStreamClientTestPeer::HandleBytes(&client, "rgb", "bad");
  EXPECT_EQ(client.stream_diagnostic("rgb"), "stream frame is shorter than its prefix");

  auto payload = MakePayload();
  auto wrong_shape = MakeHeader(1);
  wrong_shape.set_width(3);
  SensorStreamClientTestPeer::HandleBytes(&client, "rgb", Serialize(wrong_shape, payload));
  EXPECT_EQ(client.stream_diagnostic("rgb"), "sensor stream frame shape mismatch");

  OwnedStreamSample output;
  output.header.set_stream_name("unchanged");
  std::string error;
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kTimeout);
  EXPECT_EQ(output.header.stream_name(), "unchanged");

  SensorStreamClientTestPeer::HandleBytes(&client, "rgb", Serialize(MakeHeader(2), payload));
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kReceived);
  EXPECT_EQ(output.header.stream_sequence(), 2U);
}

TEST(SensorStreamClientTest, CallbackViewCopiesHeaderAndPayloadExplicitly) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  OwnedStreamSample copied;
  bool called = false;
  std::string error;
  ASSERT_TRUE(client.SetCallback(
      "rgb",
      [&](const mujoco_sim::adapter_sdk::StreamSampleView& view) {
        called = true;
        EXPECT_EQ(view.header().stream_sequence(), 4U);
        EXPECT_EQ(view.payload().size(), 12U);
        copied = view.Copy();
      },
      &error))
      << error;

  auto payload = MakePayload(10);
  Deliver(&client, 4, &payload);
  ASSERT_TRUE(called);
  payload[0] = std::byte{99};
  EXPECT_EQ(copied.header.stream_sequence(), 4U);
  EXPECT_EQ(std::to_integer<int>(copied.payload[0]), 10);
}

TEST(SensorStreamClientTest, CallbackAndBlockingModesAreMutuallyExclusive) {
  SensorStreamClient callback_client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&callback_client, {MakeDescriptor()});
  std::string error;
  ASSERT_TRUE(callback_client.SetCallback("rgb", [](const auto&) {}, &error));
  OwnedStreamSample sample;
  EXPECT_EQ(callback_client.WaitForSample("rgb", 0ms, &sample, &error), SampleWaitResult::kError);
  EXPECT_EQ(error, "stream is already in callback mode: rgb");

  SensorStreamClient wait_client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&wait_client, {MakeDescriptor()});
  EXPECT_EQ(wait_client.WaitForSample("rgb", 0ms, &sample, &error), SampleWaitResult::kTimeout);
  EXPECT_FALSE(wait_client.SetCallback("rgb", [](const auto&) {}, &error));
  EXPECT_EQ(error, "stream is already in blocking wait mode: rgb");
}

TEST(SensorStreamClientTest, WaitTimesOutAndStopWakesWaiter) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  OwnedStreamSample output;
  std::string error;
  EXPECT_EQ(client.WaitForSample("rgb", 1ms, &output, &error), SampleWaitResult::kTimeout);
  EXPECT_EQ(client.WaitForSample("rgb", -1ms, &output, &error), SampleWaitResult::kError);

  auto waiting = std::async(std::launch::async, [&] {
    std::string wait_error;
    return client.WaitForSample("rgb", std::chrono::milliseconds::max(), &output, &wait_error);
  });
  std::this_thread::sleep_for(20ms);
  client.Stop();
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), SampleWaitResult::kStopped);
}

TEST(SensorStreamClientTest, MatchingStreamErrorWakesWaiterWithReason) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  OwnedStreamSample output;
  output.header.set_stream_name("unchanged");
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForSample("rgb", std::chrono::milliseconds::max(), &output, &error);
  });
  for (int i = 0; i < 100 && !SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"));

  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_ERROR, "renderer failed"));

  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), SampleWaitResult::kError);
  EXPECT_EQ(error, "renderer failed");
  EXPECT_EQ(output.header.stream_name(), "unchanged");
  EXPECT_EQ(client.stream_diagnostic("rgb"), "renderer failed");
}

TEST(SensorStreamClientTest, ServerTerminalStatesWakeInfiniteWait) {
  SensorStreamClient stopped_client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&stopped_client, {MakeDescriptor()});
  OwnedStreamSample stopped_output;
  std::string stopped_error;
  auto stopped_wait = std::async(std::launch::async, [&] {
    return stopped_client.WaitForSample("rgb", std::chrono::milliseconds::max(), &stopped_output,
                                        &stopped_error);
  });
  std::this_thread::sleep_for(20ms);
  auto stopped_status = MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_ERROR, "ignored stream error");
  stopped_status.set_state(mujoco_sim::v4::STOPPED);
  SensorStreamClientTestPeer::HandleStatus(&stopped_client, stopped_status);
  ASSERT_EQ(stopped_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(stopped_wait.get(), SampleWaitResult::kStopped);
  EXPECT_TRUE(stopped_error.empty());

  SensorStreamClient error_client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&error_client, {MakeDescriptor()});
  OwnedStreamSample error_output;
  std::string error_message;
  auto error_wait = std::async(std::launch::async, [&] {
    return error_client.WaitForSample("rgb", std::chrono::milliseconds::max(), &error_output,
                                      &error_message);
  });
  std::this_thread::sleep_for(20ms);
  auto error_status = MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_RUNNING);
  error_status.set_state(mujoco_sim::v4::ERROR_PAUSED);
  error_status.set_message("fatal simulation error");
  SensorStreamClientTestPeer::HandleStatus(&error_client, error_status);
  ASSERT_EQ(error_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(error_wait.get(), SampleWaitResult::kError);
  EXPECT_EQ(error_message, "fatal simulation error");
  EXPECT_EQ(error_client.stream_diagnostic("rgb"), "fatal simulation error");
}

TEST(SensorStreamClientTest, PausedStatusLeavesInfiniteWaitNonterminal) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  OwnedStreamSample output;
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForSample("rgb", std::chrono::milliseconds::max(), &output, &error);
  });
  for (int i = 0; i < 100 && !SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"));
  auto paused = MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_RUNNING);
  paused.set_state(mujoco_sim::v4::PAUSED);
  SensorStreamClientTestPeer::HandleStatus(&client, paused);

  EXPECT_EQ(waiting.wait_for(50ms), std::future_status::timeout);
  client.Stop();
  EXPECT_EQ(waiting.get(), SampleWaitResult::kStopped);
  EXPECT_TRUE(error.empty());
}

TEST(SensorStreamClientTest, SameTickRunningThenErrorWakesWaiter) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  OwnedStreamSample output;
  output.header.set_stream_name("unchanged");
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForSample("rgb", std::chrono::milliseconds::max(), &output, &error);
  });
  for (int i = 0; i < 100 && !SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"));

  SensorStreamClientTestPeer::HandleStatus(&client,
                                           MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_RUNNING));
  EXPECT_EQ(waiting.wait_for(0ms), std::future_status::timeout);
  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_ERROR, "same-tick renderer failure"));

  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), SampleWaitResult::kError);
  EXPECT_EQ(error, "same-tick renderer failure");
  EXPECT_EQ(output.header.stream_name(), "unchanged");
}

TEST(SensorStreamClientTest, CallbackModeExposesServerStreamError) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  std::size_t callback_count = 0;
  std::string error;
  ASSERT_TRUE(client.SetCallback("rgb", [&](const auto&) { ++callback_count; }, &error));

  auto payload = MakePayload();
  Deliver(&client, 1, &payload);
  ASSERT_EQ(callback_count, 1U);
  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_ERROR, "camera disconnected"));
  EXPECT_EQ(client.stream_diagnostic("rgb"), "camera disconnected");

  Deliver(&client, 2, &payload);
  EXPECT_EQ(callback_count, 1U);
}

TEST(SensorStreamClientTest, StrictlyFiltersStatusIdentityAndMalformedBytes) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});

  SensorStreamClientTestPeer::HandleStatusBytes(&client, "not protobuf");
  auto wrong_epoch = MakeStatus(101, "rgb", mujoco_sim::v4::STREAM_ERROR, "wrong epoch");
  wrong_epoch.set_epoch(10);
  SensorStreamClientTestPeer::HandleStatus(&client, wrong_epoch);

  EXPECT_TRUE(client.stream_diagnostic("rgb").empty());
  OwnedStreamSample output;
  std::string error;
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kTimeout);

  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(5, "rgb", mujoco_sim::v4::STREAM_ERROR, "valid error"));
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kError);
  EXPECT_EQ(error, "valid error");
}

TEST(SensorStreamClientTest, IgnoresOutOfOrderAndUnrequestedStreamStatus) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  SensorStreamClientTestPeer::HandleStatus(&client,
                                           MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_RUNNING));
  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(19, "rgb", mujoco_sim::v4::STREAM_ERROR, "stale error"));
  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(21, "depth", mujoco_sim::v4::STREAM_ERROR, "other stream failed"));

  EXPECT_TRUE(client.stream_diagnostic("rgb").empty());
  OwnedStreamSample output;
  std::string error;
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kTimeout);

  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(22, "rgb", mujoco_sim::v4::STREAM_ERROR, "current error"));
  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(21, "rgb", mujoco_sim::v4::STREAM_ERROR, "late stale error"));
  EXPECT_EQ(client.stream_diagnostic("rgb"), "current error");
}

TEST(SensorStreamClientTest, StopSuppressesRacingStatusError) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  client.Stop();
  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_ERROR, "late server error"));

  EXPECT_TRUE(client.stream_diagnostic("rgb").empty());
  OwnedStreamSample output;
  std::string error;
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kStopped);
}

TEST(SensorStreamClientTest, StreamErrorIsIsolatedToMatchingRequestedStream) {
  auto options = MakeOptions();
  options.stream_names.push_back("depth");
  SensorStreamClient client(std::move(options));
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor(), MakeDescriptor("depth")});
  OwnedStreamSample rgb_sample;
  std::string rgb_error;
  auto rgb_wait = std::async(std::launch::async, [&] {
    return client.WaitForSample("rgb", std::chrono::milliseconds::max(), &rgb_sample, &rgb_error);
  });
  for (int i = 0; i < 100 && !SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"));

  SensorStreamClientTestPeer::HandleStatus(
      &client, MakeStatus(20, "depth", mujoco_sim::v4::STREAM_ERROR, "depth renderer failed"));
  auto payload = MakePayload();
  Deliver(&client, 1, &payload);

  ASSERT_EQ(rgb_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(rgb_wait.get(), SampleWaitResult::kReceived);
  EXPECT_TRUE(rgb_error.empty());
  EXPECT_TRUE(client.stream_diagnostic("rgb").empty());
  EXPECT_EQ(client.stream_diagnostic("depth"), "depth renderer failed");

  OwnedStreamSample depth_sample;
  std::string depth_error;
  EXPECT_EQ(client.WaitForSample("depth", 0ms, &depth_sample, &depth_error),
            SampleWaitResult::kError);
  EXPECT_EQ(depth_error, "depth renderer failed");
}

TEST(SensorStreamClientTest, ConcurrentWaitOnOneStreamIsRejected) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  std::promise<void> waiting_promise;
  auto waiting_started = waiting_promise.get_future();
  auto first_wait = std::async(std::launch::async, [&] {
    OwnedStreamSample output;
    std::string error;
    waiting_promise.set_value();
    return client.WaitForSample("rgb", 1s, &output, &error);
  });
  ASSERT_EQ(waiting_started.wait_for(1s), std::future_status::ready);
  for (int i = 0; i < 100 && !SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(SensorStreamClientTestPeer::WaitInProgress(&client, "rgb"));

  OwnedStreamSample output;
  std::string error;
  EXPECT_EQ(client.WaitForSample("rgb", 0ms, &output, &error), SampleWaitResult::kError);
  EXPECT_EQ(error, "another thread is already waiting for stream: rgb");
  auto payload = MakePayload();
  Deliver(&client, 1, &payload);
  EXPECT_EQ(first_wait.get(), SampleWaitResult::kReceived);
}

TEST(SensorStreamClientTest, ExposesDiscoveredDescriptorAndDiagnostics) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  const auto descriptor = client.descriptor("rgb");
  ASSERT_TRUE(descriptor.has_value());
  EXPECT_EQ(descriptor->format(), mujoco_sim::v4::RGB8);
  EXPECT_EQ(client.discovery().epoch(), 9U);
  EXPECT_FALSE(client.descriptor("missing").has_value());
}

TEST(SensorStreamClientTest, CallbackExceptionsDoNotEscapeTransportThread) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  std::string error;
  ASSERT_TRUE(
      client.SetCallback("rgb", [](const auto&) { throw std::runtime_error("failure"); }, &error));
  auto payload = MakePayload();
  EXPECT_NO_THROW(Deliver(&client, 1, &payload));
  EXPECT_EQ(client.stream_diagnostic("rgb"), "stream callback threw: failure");
}

TEST(SensorStreamClientTest, DropsConcurrentFrameInsteadOfBlockingCallbackThread) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  std::promise<void> callback_entered;
  auto entered = callback_entered.get_future();
  std::promise<void> release_callback;
  auto release = release_callback.get_future().share();
  std::atomic<std::size_t> callback_count = 0;
  std::string error;
  ASSERT_TRUE(client.SetCallback(
      "rgb",
      [&](const auto&) {
        if (++callback_count == 1) {
          callback_entered.set_value();
          release.wait();
        }
      },
      &error));
  auto payload = MakePayload();
  auto first = std::async(std::launch::async, [&] { Deliver(&client, 1, &payload); });
  ASSERT_EQ(entered.wait_for(1s), std::future_status::ready);

  auto second = std::async(std::launch::async, [&] { Deliver(&client, 2, &payload); });
  ASSERT_EQ(second.wait_for(1s), std::future_status::ready);
  second.get();
  EXPECT_EQ(callback_count.load(), 1U);
  EXPECT_EQ(client.dropped_samples("rgb"), 1U);

  release_callback.set_value();
  ASSERT_EQ(first.wait_for(1s), std::future_status::ready);
  first.get();
}

TEST(SensorStreamClientTest, TeardownDrainsAndDisablesCallbacks) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  auto delayed = SensorStreamClientTestPeer::MakePendingCallback(&client);
  EXPECT_TRUE(delayed());

  std::promise<void> entered_promise;
  auto entered = entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  auto callback = std::async(std::launch::async, [&] {
    SensorStreamClientTestPeer::HoldCallback(&client, &entered_promise, release);
  });
  ASSERT_EQ(entered.wait_for(1s), std::future_status::ready);
  auto stopping = std::async(std::launch::async,
                             [&] { SensorStreamClientTestPeer::StopSampleReception(&client); });
  EXPECT_TRUE(SensorStreamClientTestPeer::WaitForSampleReceptionDisabled(&client, 1s));
  EXPECT_EQ(stopping.wait_for(0ms), std::future_status::timeout);
  release_promise.set_value();
  callback.get();
  ASSERT_EQ(stopping.wait_for(1s), std::future_status::ready);
  stopping.get();
  EXPECT_FALSE(delayed());
}

TEST(SensorStreamClientTest, TeardownDoesNotWaitForCurrentCallback) {
  SensorStreamClient client(MakeOptions());
  SensorStreamClientTestPeer::Configure(&client, {MakeDescriptor()});
  auto stopping = std::async(std::launch::async,
                             [&] { SensorStreamClientTestPeer::StopFromCallback(&client); });
  ASSERT_EQ(stopping.wait_for(1s), std::future_status::ready);
  stopping.get();
}

TEST(SensorStreamClientTest, ClientCanBeDestroyedFromCurrentCallback) {
  auto client = std::make_unique<SensorStreamClient>(MakeOptions());
  SensorStreamClientTestPeer::Configure(client.get(), {MakeDescriptor()});
  auto destroying = std::async(std::launch::async,
                               [&] { SensorStreamClientTestPeer::DestroyFromCallback(&client); });
  ASSERT_EQ(destroying.wait_for(1s), std::future_status::ready);
  destroying.get();
  EXPECT_EQ(client, nullptr);
}

TEST(SensorStreamClientTest, DelayedCallbackCannotReachDestroyedClient) {
  std::function<bool()> delayed;
  {
    auto client = std::make_unique<SensorStreamClient>(MakeOptions());
    SensorStreamClientTestPeer::Configure(client.get(), {MakeDescriptor()});
    delayed = SensorStreamClientTestPeer::MakePendingStatusCallback(
        client.get(),
        MakeStatus(20, "rgb", mujoco_sim::v4::STREAM_ERROR, "status callback reached client"));
    EXPECT_TRUE(delayed());
    EXPECT_EQ(client->stream_diagnostic("rgb"), "status callback reached client");
  }
  EXPECT_FALSE(delayed());
}

}  // namespace
