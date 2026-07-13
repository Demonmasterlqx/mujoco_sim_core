#include "sensor_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::server {
namespace {

using namespace std::chrono_literals;

class FakePublisherBus {
 public:
  struct Channel {
    std::atomic<std::size_t> subscribers{1};
    std::atomic<bool> fail_send{false};
    std::atomic<bool> disconnect_on_send{false};
    std::atomic<bool> replace_subscriber_on_send{false};
    std::mutex mutex;
    std::condition_variable condition;
    std::vector<std::string> frames;
  };

  explicit FakePublisherBus(std::size_t default_subscribers = 1)
      : default_subscribers_(default_subscribers) {}

  SensorRuntimePublisher MakePublisher(const std::string& topic) {
    auto channel = GetOrCreate(topic);
    SensorRuntimePublisher publisher;
    publisher.lifetime = channel;
    publisher.subscriber_count = [channel] {
      return channel->subscribers.load(std::memory_order_relaxed);
    };
    publisher.send = [channel](const std::string& frame) {
      if (channel->disconnect_on_send.exchange(false,
                                               std::memory_order_relaxed)) {
        channel->subscribers.store(0, std::memory_order_relaxed);
        return false;
      }
      if (channel->replace_subscriber_on_send.exchange(
              false, std::memory_order_relaxed)) {
        return false;
      }
      if (channel->fail_send.load(std::memory_order_relaxed)) return false;
      {
        std::lock_guard lock(channel->mutex);
        channel->frames.push_back(frame);
      }
      channel->condition.notify_all();
      return true;
    };
    return publisher;
  }

  std::shared_ptr<Channel> GetOrCreate(const std::string& topic) {
    std::lock_guard lock(mutex_);
    const auto found = channels_.find(topic);
    if (found != channels_.end()) return found->second;
    auto channel = std::make_shared<Channel>();
    channel->subscribers.store(default_subscribers_,
                               std::memory_order_relaxed);
    channels_.emplace(topic, channel);
    return channel;
  }

  bool WaitForFrames(const std::string& topic, std::size_t count,
                     std::chrono::milliseconds timeout = 3s) {
    auto channel = GetOrCreate(topic);
    std::unique_lock lock(channel->mutex);
    return channel->condition.wait_for(
        lock, timeout, [&] { return channel->frames.size() >= count; });
  }

  std::vector<std::string> Frames(const std::string& topic) {
    auto channel = GetOrCreate(topic);
    std::lock_guard lock(channel->mutex);
    return channel->frames;
  }

 private:
  std::size_t default_subscribers_;
  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<Channel>> channels_;
};

class FirstCaptureGate {
 public:
  void BeforeCapture(const std::string&) {
    if (calls_.fetch_add(1, std::memory_order_relaxed) != 0) return;
    std::unique_lock lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait_for(lock, 5s, [&] { return released_; });
  }

  bool WaitUntilEntered(std::chrono::milliseconds timeout = 3s) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return entered_; });
  }

  void Release() {
    {
      std::lock_guard lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

 private:
  std::atomic<std::size_t> calls_{0};
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false;
  bool released_ = false;
};

class BlockingWaitGate {
 public:
  void OnWaitStarted(const std::string&) {
    {
      std::lock_guard lock(mutex_);
      entered_ = true;
    }
    condition_.notify_all();
  }

  bool WaitUntilEntered(std::chrono::milliseconds timeout = 3s) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return entered_; });
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false;
};

std::unique_ptr<mujoco_sim::core::SimulationCore> LoadCore() {
  auto core = std::make_unique<mujoco_sim::core::SimulationCore>(
      mujoco_sim::core::CoreOptions{"sensor-test"});
  std::string error;
  EXPECT_TRUE(core->LoadModel(MUJOCO_SIM_SENSOR_RUNTIME_TEST_MODEL, &error))
      << error;
  return core;
}

ServerConfig::SensorStream LidarStream(
    std::string name = "lidar", std::string queue_mode = "latest",
    std::uint32_t queue_depth = 1,
    std::string overflow = "drop_oldest", double rate_hz = 100.0) {
  ServerConfig::SensorStream stream;
  stream.name = std::move(name);
  stream.kind = "lidar_pointcloud";
  stream.source = "lidar_site";
  stream.frame_id = "lidar";
  stream.rate_hz = rate_hz;
  stream.horizontal_samples = 72;
  stream.vertical_samples = 4;
  stream.vertical_fov_min_deg = -10.0;
  stream.vertical_fov_max_deg = 10.0;
  stream.min_range_m = 0.05;
  stream.max_range_m = 8.0;
  stream.queue_mode = std::move(queue_mode);
  stream.queue_depth = queue_depth;
  stream.fifo_overflow = std::move(overflow);
  return stream;
}

ServerConfig::SensorStream CameraStream(std::string name, std::string kind) {
  ServerConfig::SensorStream stream;
  stream.name = std::move(name);
  stream.kind = std::move(kind);
  stream.source = "test_camera";
  stream.frame_id = "camera_optical";
  stream.rate_hz = 100.0;
  stream.width = 64;
  stream.height = 48;
  stream.queue_mode = "blocking";
  stream.queue_depth = 1;
  return stream;
}

ServerConfig BaseConfig() {
  ServerConfig config;
  config.server_name = "sensor-test";
  config.model_path = MUJOCO_SIM_SENSOR_RUNTIME_TEST_MODEL;
  config.headless = true;
  config.render_backend = "auto";
  return config;
}

bool Advance(mujoco_sim::core::SimulationCore* core,
             SensorRuntime* runtime) {
  core->OpenIssuedTicks();
  if (!core->CompleteEmptyBarrier().step_completed.has_value()) return false;
  runtime->OnStepCompleted(*core);
  runtime->CompleteBlockingEnqueues();
  return true;
}

mujoco_sim::v2::GetSensorStreamsRequest CatalogRequest() {
  mujoco_sim::v2::GetSensorStreamsRequest request;
  request.set_server_name("sensor-test");
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  return request;
}

std::optional<mujoco_sim::v2::StreamStatus> StreamStatusFor(
    const SensorRuntime& runtime, const std::string& name) {
  mujoco_sim::v2::ServerStatus status;
  runtime.AppendStatus(&status);
  const auto found = std::find_if(
      status.streams().begin(), status.streams().end(),
      [&name](const auto& stream) { return stream.name() == name; });
  if (found == status.streams().end()) return std::nullopt;
  return *found;
}

template <typename Predicate>
bool WaitUntil(Predicate predicate,
               std::chrono::milliseconds timeout = 3s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

std::vector<std::uint64_t> TickIds(const std::vector<std::string>& frames) {
  std::vector<std::uint64_t> result;
  for (const auto& bytes : frames) {
    mujoco_sim::protocol::ParsedStreamFrame parsed;
    std::string error;
    EXPECT_TRUE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error))
        << error;
    result.push_back(parsed.header.header().tick_id());
  }
  return result;
}

std::vector<std::uint64_t> DroppedCounts(
    const std::vector<std::string>& frames) {
  std::vector<std::uint64_t> result;
  for (const auto& bytes : frames) {
    mujoco_sim::protocol::ParsedStreamFrame parsed;
    std::string error;
    EXPECT_TRUE(mujoco_sim::protocol::ParseStreamFrame(bytes, &parsed, &error))
        << error;
    result.push_back(parsed.header.dropped_since_previous());
  }
  return result;
}

TEST(SensorRuntimeTest, PublishesHeadlessCameraAndLidarCatalogFrames) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      CameraStream("rgb", "rgb"),
      CameraStream("depth", "depth"),
      CameraStream("segmentation", "segmentation"),
      CameraStream("camera_cloud", "camera_pointcloud"),
      LidarStream(),
  };

  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  const auto catalog = runtime.GetCatalog(CatalogRequest());
  ASSERT_EQ(catalog.code(), mujoco_sim::v2::OK);
  ASSERT_EQ(catalog.streams_size(), 5);
  for (const auto& descriptor : catalog.streams()) {
    EXPECT_EQ(descriptor.topic(),
              mujoco_sim::protocol::SensorStreamTopicName(
                  "sensor-test", descriptor.name()));
    EXPECT_GT(descriptor.maximum_payload_bytes(), 0U);
  }

  ASSERT_TRUE(Advance(core.get(), &runtime));
  for (const std::string name :
       {"rgb", "depth", "segmentation", "camera_cloud", "lidar"}) {
    ASSERT_TRUE(bus.WaitForFrames(
        mujoco_sim::protocol::SensorStreamTopicName("sensor-test", name), 1))
        << name;
  }

  const auto rgb_bytes = bus.Frames(
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "rgb"));
  mujoco_sim::protocol::ParsedStreamFrame rgb;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(rgb_bytes.front(), &rgb,
                                                     &error))
      << error;
  EXPECT_EQ(rgb.header.format(), mujoco_sim::v2::RGB8);
  EXPECT_EQ(rgb.payload.size(), 64U * 48U * 3U);
  EXPECT_FALSE(std::all_of(rgb.payload.begin(), rgb.payload.end(),
                           [first = rgb.payload.front()](std::byte value) {
                             return value == first;
                           }));

  const auto lidar_bytes = bus.Frames(
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar"));
  mujoco_sim::protocol::ParsedStreamFrame lidar;
  ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(
      lidar_bytes.front(), &lidar, &error))
      << error;
  EXPECT_EQ(lidar.header.format(), mujoco_sim::v2::XYZ32F);
  EXPECT_EQ(lidar.payload.size(), 72U * 4U * 3U * sizeof(float));
  bool has_finite_point = false;
  for (std::size_t offset = 0; offset < lidar.payload.size();
       offset += 3 * sizeof(float)) {
    float x = 0.0F;
    std::memcpy(&x, lidar.payload.data() + offset, sizeof(x));
    has_finite_point = has_finite_point || std::isfinite(x);
  }
  EXPECT_TRUE(has_finite_point);
}

TEST(SensorRuntimeTest, RejectsAnyProtocolVersionOtherThanTwoZero) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  auto request = CatalogRequest();
  request.set_protocol_minor(1);
  const auto response = runtime.GetCatalog(request);
  EXPECT_EQ(response.code(), mujoco_sim::v2::VERSION_MISMATCH);
  EXPECT_NE(response.message().find("2.0"), std::string::npos);
  EXPECT_EQ(response.streams_size(), 0);
}

TEST(SensorRuntimeTest, UsesSimulationTimeForStreamRate) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "blocking", 1, "drop_oldest", 20.0)};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  for (int step = 0; step < 21; ++step) {
    ASSERT_TRUE(Advance(core.get(), &runtime));
  }
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  ASSERT_TRUE(bus.WaitForFrames(topic, 5));
  const auto frames = bus.Frames(topic);
  ASSERT_EQ(frames.size(), 5U);
  for (std::size_t index = 0; index < frames.size(); ++index) {
    mujoco_sim::protocol::ParsedStreamFrame parsed;
    ASSERT_TRUE(mujoco_sim::protocol::ParseStreamFrame(frames[index], &parsed,
                                                       &error));
    ASSERT_TRUE(parsed.header.header().has_simulation_time_s());
    EXPECT_NEAR(parsed.header.header().simulation_time_s(),
                0.01 + 0.05 * static_cast<double>(index), 1e-10);
  }
}

TEST(SensorRuntimeTest, MaximumRateAdvancesScheduleWithoutCatchUpLoop) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream("lidar", "blocking", 1,
                                       "drop_oldest",
                                       kMaximumSensorStreamRateHz)};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  for (int step = 0; step < 3; ++step) {
    ASSERT_TRUE(Advance(core.get(), &runtime));
  }
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  EXPECT_TRUE(bus.WaitForFrames(topic, 3));
}

TEST(SensorRuntimeTest, LatestQueueKeepsNewestCaptureAndCountsDrops) {
  auto core = LoadCore();
  FakePublisherBus bus;
  FirstCaptureGate gate;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&gate](const std::string& source) { gate.BeforeCapture(source); }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(gate.WaitUntilEntered());
  for (int step = 0; step < 5; ++step) {
    ASSERT_TRUE(Advance(core.get(), &runtime));
  }
  gate.Release();

  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  ASSERT_TRUE(bus.WaitForFrames(topic, 2));
  const auto frames = bus.Frames(topic);
  EXPECT_EQ(TickIds(frames),
            (std::vector<std::uint64_t>{1, 6}));
  EXPECT_EQ(DroppedCounts(frames),
            (std::vector<std::uint64_t>{0, 4}));
  ASSERT_TRUE(WaitUntil([&] {
    const auto status = StreamStatusFor(runtime, "lidar");
    return status && status->dropped() == 4;
  }));
}

class SensorFifoRuntimeTest
    : public testing::TestWithParam<std::pair<std::string,
                                              std::vector<std::uint64_t>>> {};

TEST_P(SensorFifoRuntimeTest, AppliesConfiguredOverflowPolicy) {
  auto core = LoadCore();
  FakePublisherBus bus;
  FirstCaptureGate gate;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&gate](const std::string& source) { gate.BeforeCapture(source); }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "fifo", 2, GetParam().first)};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(gate.WaitUntilEntered());
  for (int step = 0; step < 4; ++step) {
    ASSERT_TRUE(Advance(core.get(), &runtime));
  }
  gate.Release();

  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  ASSERT_TRUE(bus.WaitForFrames(topic, 3));
  EXPECT_EQ(TickIds(bus.Frames(topic)), GetParam().second);
  ASSERT_TRUE(WaitUntil([&] {
    const auto status = StreamStatusFor(runtime, "lidar");
    return status && status->dropped() == 2;
  }));
}

INSTANTIATE_TEST_SUITE_P(
    OverflowPolicies, SensorFifoRuntimeTest,
    testing::Values(
        std::make_pair("drop_oldest",
                       std::vector<std::uint64_t>{1, 4, 5}),
        std::make_pair("drop_newest",
                       std::vector<std::uint64_t>{1, 2, 3})));

TEST(SensorRuntimeTest, BlockingQueueWaitsThroughSuccessfulPublish) {
  auto core = LoadCore();
  std::mutex core_mutex;
  FakePublisherBus bus;
  FirstCaptureGate gate;
  BlockingWaitGate wait_gate;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&gate](const std::string& source) { gate.BeforeCapture(source); },
      [&wait_gate](const std::string& source) {
        wait_gate.OnWaitStarted(source);
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "blocking", 1, "drop_oldest")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  {
    std::lock_guard lock(core_mutex);
    core->OpenIssuedTicks();
    ASSERT_TRUE(core->CompleteEmptyBarrier().step_completed.has_value());
    runtime.OnStepCompleted(*core);
  }
  ASSERT_TRUE(gate.WaitUntilEntered());
  auto completion = std::async(std::launch::async, [&] {
    runtime.CompleteBlockingEnqueues();
  });
  ASSERT_TRUE(wait_gate.WaitUntilEntered());
  EXPECT_EQ(completion.wait_for(0ms), std::future_status::timeout);
  auto status_future = std::async(std::launch::async, [&] {
    std::lock_guard lock(core_mutex);
    return core->MakeStatus();
  });
  ASSERT_EQ(status_future.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(status_future.get().state(), mujoco_sim::v2::RUNNING);
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  EXPECT_TRUE(bus.Frames(topic).empty());
  gate.Release();
  ASSERT_EQ(completion.wait_for(3s), std::future_status::ready);
  completion.get();

  EXPECT_EQ(bus.Frames(topic).size(), 1U);
  const auto status = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(status.has_value());
  EXPECT_GT(status->blocked_time_ms(), 0.0);
}

TEST(SensorRuntimeTest, StopRequestReleasesBlockingEnqueueBeforeWorkerJoin) {
  auto core = LoadCore();
  FakePublisherBus bus;
  FirstCaptureGate capture_gate;
  BlockingWaitGate wait_gate;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&capture_gate](const std::string& source) {
        capture_gate.BeforeCapture(source);
      },
      [&wait_gate](const std::string& source) {
        wait_gate.OnWaitStarted(source);
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "blocking", 1, "drop_oldest")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  core->OpenIssuedTicks();
  ASSERT_TRUE(core->CompleteEmptyBarrier().step_completed.has_value());
  runtime.OnStepCompleted(*core);
  ASSERT_TRUE(capture_gate.WaitUntilEntered());
  auto completion = std::async(std::launch::async, [&] {
    runtime.CompleteBlockingEnqueues();
  });
  ASSERT_TRUE(wait_gate.WaitUntilEntered());
  ASSERT_EQ(completion.wait_for(0ms), std::future_status::timeout);

  runtime.RequestStop();
  ASSERT_EQ(completion.wait_for(500ms), std::future_status::ready);
  completion.get();

  capture_gate.Release();
  runtime.Stop();
}

TEST(SensorRuntimeTest, ExternalStopPredicateReleasesBlockingEnqueue) {
  auto core = LoadCore();
  FakePublisherBus bus;
  FirstCaptureGate capture_gate;
  BlockingWaitGate wait_gate;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&capture_gate](const std::string& source) {
        capture_gate.BeforeCapture(source);
      },
      [&wait_gate](const std::string& source) {
        wait_gate.OnWaitStarted(source);
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "blocking", 1, "drop_oldest")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  core->OpenIssuedTicks();
  ASSERT_TRUE(core->CompleteEmptyBarrier().step_completed.has_value());
  runtime.OnStepCompleted(*core);
  ASSERT_TRUE(capture_gate.WaitUntilEntered());
  std::stop_source stop_source;
  auto completion = std::async(std::launch::async, [&] {
    runtime.CompleteBlockingEnqueues(stop_source.get_token());
  });
  ASSERT_TRUE(wait_gate.WaitUntilEntered());
  ASSERT_EQ(completion.wait_for(0ms), std::future_status::timeout);

  stop_source.request_stop();
  ASSERT_EQ(completion.wait_for(500ms), std::future_status::ready);
  completion.get();

  capture_gate.Release();
  runtime.Stop();
}

TEST(SensorRuntimeTest, BlockingCaptureFailureReleasesPendingWaitSafely) {
  auto core = LoadCore();
  FakePublisherBus bus;
  FirstCaptureGate capture_gate;
  BlockingWaitGate wait_gate;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&capture_gate](const std::string& source) {
        capture_gate.BeforeCapture(source);
        throw std::runtime_error("injected blocking capture failure");
      },
      [&wait_gate](const std::string& source) {
        wait_gate.OnWaitStarted(source);
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "blocking", 1, "drop_oldest")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  core->OpenIssuedTicks();
  ASSERT_TRUE(core->CompleteEmptyBarrier().step_completed.has_value());
  runtime.OnStepCompleted(*core);
  ASSERT_TRUE(capture_gate.WaitUntilEntered());
  for (int step = 0; step < 2; ++step) {
    core->OpenIssuedTicks();
    ASSERT_TRUE(core->CompleteEmptyBarrier().step_completed.has_value());
    runtime.OnStepCompleted(*core);
  }
  auto completion = std::async(std::launch::async, [&] {
    runtime.CompleteBlockingEnqueues();
  });
  ASSERT_TRUE(wait_gate.WaitUntilEntered());
  EXPECT_EQ(completion.wait_for(0ms), std::future_status::timeout);
  capture_gate.Release();
  ASSERT_EQ(completion.wait_for(3s), std::future_status::ready);
  completion.get();

  const auto status = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->state(), mujoco_sim::v2::STREAM_ERROR);
  EXPECT_NE(status->message().find("injected blocking capture failure"),
            std::string::npos);
}

TEST(SensorRuntimeTest, PublishRejectionIsRetriedWithoutStoppingStream) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {CameraStream("rgb", "rgb"),
                           CameraStream("depth", "depth")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  const std::string rgb_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "rgb");
  const std::string depth_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "depth");
  bus.GetOrCreate(rgb_topic)->fail_send.store(true,
                                               std::memory_order_relaxed);

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(bus.WaitForFrames(depth_topic, 1));
  auto rgb = StreamStatusFor(runtime, "rgb");
  ASSERT_TRUE(rgb.has_value());
  EXPECT_EQ(rgb->state(), mujoco_sim::v2::STREAM_RUNNING);
  EXPECT_EQ(rgb->publish_errors(), 1U);
  EXPECT_EQ(rgb->published(), 0U);

  bus.GetOrCreate(rgb_topic)->fail_send.store(false,
                                                std::memory_order_relaxed);
  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(bus.WaitForFrames(rgb_topic, 1));
  ASSERT_TRUE(bus.WaitForFrames(depth_topic, 2));

  rgb = StreamStatusFor(runtime, "rgb");
  const auto depth = StreamStatusFor(runtime, "depth");
  ASSERT_TRUE(rgb.has_value());
  ASSERT_TRUE(depth.has_value());
  EXPECT_EQ(rgb->state(), mujoco_sim::v2::STREAM_RUNNING);
  EXPECT_EQ(rgb->publish_errors(), 1U);
  EXPECT_EQ(rgb->published(), 1U);
  EXPECT_EQ(depth->state(), mujoco_sim::v2::STREAM_RUNNING);
  EXPECT_EQ(depth->published(), 2U);
}

TEST(SensorRuntimeTest, SerializationFailureErrorsOnlyThatCameraOutput) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      {}, {},
      [](mujoco_sim::v2::StreamFrameHeader* header) {
        if (header->stream_name() == "rgb") {
          header->set_payload_size(header->payload_size() + 1);
        }
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {CameraStream("rgb", "rgb"),
                           CameraStream("depth", "depth")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  const std::string rgb_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "rgb");
  const std::string depth_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "depth");

  ASSERT_TRUE(Advance(core.get(), &runtime));
  EXPECT_TRUE(bus.Frames(rgb_topic).empty());
  ASSERT_TRUE(bus.WaitForFrames(depth_topic, 1));
  auto rgb = StreamStatusFor(runtime, "rgb");
  auto depth = StreamStatusFor(runtime, "depth");
  ASSERT_TRUE(rgb.has_value());
  ASSERT_TRUE(depth.has_value());
  EXPECT_EQ(rgb->state(), mujoco_sim::v2::STREAM_ERROR);
  EXPECT_EQ(rgb->publish_errors(), 1U);
  EXPECT_NE(rgb->message().find("payload_size"), std::string::npos);
  EXPECT_EQ(depth->state(), mujoco_sim::v2::STREAM_RUNNING);

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(bus.WaitForFrames(depth_topic, 2));
  rgb = StreamStatusFor(runtime, "rgb");
  depth = StreamStatusFor(runtime, "depth");
  ASSERT_TRUE(rgb.has_value());
  ASSERT_TRUE(depth.has_value());
  EXPECT_EQ(rgb->publish_errors(), 1U);
  EXPECT_EQ(depth->published(), 2U);
}

TEST(SensorRuntimeTest, SerializationExceptionErrorsOnlyThatCameraOutput) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntimeTestHooks hooks;
  hooks.make_publisher = [&bus](const std::string& topic) {
    return bus.MakePublisher(topic);
  };
  hooks.before_serialize = [](mujoco_sim::v2::StreamFrameHeader* header) {
    if (header->stream_name() == "rgb") {
      throw std::runtime_error("injected serialization exception");
    }
  };
  SensorRuntime runtime(std::move(hooks));
  ServerConfig config = BaseConfig();
  config.sensor_streams = {CameraStream("rgb", "rgb"),
                           CameraStream("depth", "depth")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  ASSERT_TRUE(Advance(core.get(), &runtime));
  const std::string depth_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "depth");
  ASSERT_TRUE(bus.WaitForFrames(depth_topic, 1));
  const auto rgb = StreamStatusFor(runtime, "rgb");
  const auto depth = StreamStatusFor(runtime, "depth");
  ASSERT_TRUE(rgb.has_value());
  ASSERT_TRUE(depth.has_value());
  EXPECT_EQ(rgb->state(), mujoco_sim::v2::STREAM_ERROR);
  EXPECT_NE(rgb->message().find("injected serialization exception"),
            std::string::npos);
  EXPECT_EQ(depth->state(), mujoco_sim::v2::STREAM_RUNNING);
}

TEST(SensorRuntimeTest, PublisherExceptionErrorsOnlyThatCameraOutput) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntimeTestHooks hooks;
  hooks.make_publisher = [&bus](const std::string& topic) {
    auto publisher = bus.MakePublisher(topic);
    if (topic.ends_with("/rgb")) {
      publisher.send = [](const std::string&) -> bool {
        throw std::runtime_error("injected publisher exception");
      };
    }
    return publisher;
  };
  SensorRuntime runtime(std::move(hooks));
  ServerConfig config = BaseConfig();
  config.sensor_streams = {CameraStream("rgb", "rgb"),
                           CameraStream("depth", "depth")};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  ASSERT_TRUE(Advance(core.get(), &runtime));
  const std::string depth_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "depth");
  ASSERT_TRUE(bus.WaitForFrames(depth_topic, 1));
  const auto rgb = StreamStatusFor(runtime, "rgb");
  const auto depth = StreamStatusFor(runtime, "depth");
  ASSERT_TRUE(rgb.has_value());
  ASSERT_TRUE(depth.has_value());
  EXPECT_EQ(rgb->state(), mujoco_sim::v2::STREAM_ERROR);
  EXPECT_NE(rgb->message().find("injected publisher exception"),
            std::string::npos);
  EXPECT_EQ(depth->state(), mujoco_sim::v2::STREAM_RUNNING);
}

TEST(SensorRuntimeTest, SubscriberReplacementDuringSendIsRetryable) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  const auto channel = bus.GetOrCreate(topic);
  channel->replace_subscriber_on_send.store(true,
                                             std::memory_order_relaxed);

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(WaitUntil([&] {
    const auto status = StreamStatusFor(runtime, "lidar");
    return status && status->publish_errors() == 1;
  }));
  auto status = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->state(), mujoco_sim::v2::STREAM_RUNNING);
  EXPECT_EQ(status->subscriber_count(), 1U);

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(bus.WaitForFrames(topic, 1));
  ASSERT_TRUE(WaitUntil([&] {
    const auto current = StreamStatusFor(runtime, "lidar");
    return current && current->published() == 1;
  }));
}

TEST(SensorRuntimeTest, SubscriberDisconnectDuringSendIsNotAStreamError) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  const auto channel = bus.GetOrCreate(topic);
  channel->disconnect_on_send.store(true, std::memory_order_relaxed);

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(WaitUntil([&] {
    const auto status = StreamStatusFor(runtime, "lidar");
    return status && status->state() == mujoco_sim::v2::STREAM_IDLE &&
           status->captures() == 1 && status->subscriber_count() == 0;
  }));
  const auto disconnected = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(disconnected.has_value());
  EXPECT_EQ(disconnected->publish_errors(), 0U);

  channel->subscribers.store(1, std::memory_order_relaxed);
  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(bus.WaitForFrames(topic, 1));
  ASSERT_TRUE(WaitUntil([&] {
    const auto status = StreamStatusFor(runtime, "lidar");
    return status && status->state() == mujoco_sim::v2::STREAM_RUNNING;
  }));
}

TEST(SensorRuntimeTest, LazyStreamReturnsToIdleAfterLastSubscriberLeaves) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {
      LidarStream("lidar", "latest", 1, "drop_oldest", 1.0)};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  const std::string topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  const auto channel = bus.GetOrCreate(topic);

  ASSERT_TRUE(Advance(core.get(), &runtime));
  ASSERT_TRUE(bus.WaitForFrames(topic, 1));
  ASSERT_TRUE(WaitUntil([&] {
    const auto status = StreamStatusFor(runtime, "lidar");
    return status && status->state() == mujoco_sim::v2::STREAM_RUNNING;
  }));

  channel->subscribers.store(0, std::memory_order_relaxed);
  ASSERT_TRUE(Advance(core.get(), &runtime));
  const auto status = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->state(), mujoco_sim::v2::STREAM_IDLE);
  EXPECT_EQ(status->subscriber_count(), 0U);
  EXPECT_EQ(status->captures(), 1U);
}

TEST(SensorRuntimeTest, CameraCaptureFailureErrorsItsGroupButNotOtherWorkers) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [](const std::string& source) {
        if (source == "test_camera") {
          throw std::runtime_error("injected camera failure");
        }
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {CameraStream("rgb", "rgb"),
                           CameraStream("depth", "depth"), LidarStream()};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;

  ASSERT_TRUE(Advance(core.get(), &runtime));
  const std::string lidar_topic =
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "lidar");
  ASSERT_TRUE(bus.WaitForFrames(lidar_topic, 1));
  ASSERT_TRUE(WaitUntil([&] {
    const auto rgb = StreamStatusFor(runtime, "rgb");
    const auto depth = StreamStatusFor(runtime, "depth");
    return rgb && depth && rgb->state() == mujoco_sim::v2::STREAM_ERROR &&
           depth->state() == mujoco_sim::v2::STREAM_ERROR;
  }));
  const auto lidar = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(lidar.has_value());
  EXPECT_EQ(lidar->state(), mujoco_sim::v2::STREAM_RUNNING);
}

TEST(SensorRuntimeTest, NoSubscribersAvoidCaptureWork) {
  auto core = LoadCore();
  FakePublisherBus bus(0);
  std::atomic<std::size_t> captures{0};
  SensorRuntime runtime({
      [&bus](const std::string& topic) { return bus.MakePublisher(topic); },
      [&captures](const std::string&) {
        captures.fetch_add(1, std::memory_order_relaxed);
      }});
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  ASSERT_TRUE(runtime.Initialize(*core, config, &error)) << error;
  for (int step = 0; step < 20; ++step) {
    ASSERT_TRUE(Advance(core.get(), &runtime));
  }
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(captures.load(std::memory_order_relaxed), 0U);
  const auto status = StreamStatusFor(runtime, "lidar");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->captures(), 0U);
  EXPECT_EQ(status->published(), 0U);
}

TEST(SensorRuntimeTest, InitializationFailureRollsBackStartedWorkers) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  auto invalid = LidarStream("invalid");
  invalid.source = "missing_site";
  config.sensor_streams = {LidarStream("valid"), invalid};
  std::string error;
  ASSERT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("missing_site"), std::string::npos);
  EXPECT_EQ(runtime.GetCatalog(CatalogRequest()).code(),
            mujoco_sim::v2::FAILED_PRECONDITION);
  EXPECT_TRUE(Advance(core.get(), &runtime));
  EXPECT_FALSE(bus.WaitForFrames(
      mujoco_sim::protocol::SensorStreamTopicName("sensor-test", "valid"), 1,
      50ms));
}

TEST(SensorRuntimeTest, InitializationExceptionIsReportedWithoutHanging) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntimeTestHooks hooks;
  hooks.make_publisher = [&bus](const std::string& topic) {
    return bus.MakePublisher(topic);
  };
  hooks.before_initialize = [](const std::string&) {
    throw std::runtime_error("injected initialization exception");
  };
  SensorRuntime runtime(std::move(hooks));
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("injected initialization exception"),
            std::string::npos);
}

TEST(SensorRuntimeTest, PublisherFactoryExceptionIsReported) {
  auto core = LoadCore();
  SensorRuntimeTestHooks hooks;
  hooks.make_publisher = [](const std::string&) -> SensorRuntimePublisher {
    throw std::runtime_error("injected publisher factory exception");
  };
  SensorRuntime runtime(std::move(hooks));
  ServerConfig config = BaseConfig();
  config.sensor_streams = {LidarStream()};
  std::string error;
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("injected publisher factory exception"),
            std::string::npos);
}

TEST(SensorRuntimeTest, RejectsUnsafeDirectConfigurationBeforeAllocation) {
  auto core = LoadCore();
  FakePublisherBus bus;
  SensorRuntime runtime({[&bus](const std::string& topic) {
                           return bus.MakePublisher(topic);
                         },
                         {}});
  ServerConfig config = BaseConfig();
  auto stream = LidarStream();
  stream.queue_mode = "fifo";
  stream.queue_depth = kMaximumSensorStreamQueueDepth + 1;
  config.sensor_streams = {stream};
  std::string error;
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("queue_depth"), std::string::npos);

  config = BaseConfig();
  config.headless = false;
  config.render_backend = "osmesa";
  config.sensor_streams = {CameraStream("rgb", "rgb")};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("graphical viewer"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.rate_hz = 1e-300;
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("rate_hz"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.queue_mode = "unknown";
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("queue_mode"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.kind = "unknown";
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("kind"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.vertical_fov_min_deg =
      std::numeric_limits<double>::quiet_NaN();
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("vertical_fov_min_deg"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.vertical_fov_max_deg = 90.001;
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("vertical_fov_max_deg"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.min_range_m = -0.01;
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("range"), std::string::npos);

  config = BaseConfig();
  stream = LidarStream();
  stream.max_range_m = std::numeric_limits<double>::infinity();
  config.sensor_streams = {stream};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("range"), std::string::npos);

  config = BaseConfig();
  config.sensor_streams = {LidarStream("duplicate"),
                           LidarStream("duplicate")};
  EXPECT_FALSE(runtime.Initialize(*core, config, &error));
  EXPECT_NE(error.find("duplicate"), std::string::npos);
}

TEST(SensorRuntimeTest, RejectsOversizedStreamShapesAtConfigBoundary) {
  const auto expect_invalid = [](ServerConfig config,
                                 const std::string& expected_error) {
    std::string error;
    EXPECT_FALSE(ValidateServerConfig(config, &error));
    EXPECT_NE(error.find(expected_error), std::string::npos) << error;
  };
  constexpr std::uint32_t kBeyondInt =
      static_cast<std::uint32_t>(std::numeric_limits<int>::max()) + 1U;

  ServerConfig config = BaseConfig();
  auto camera = CameraStream("rgb", "rgb");
  camera.width = kBeyondInt;
  config.sensor_streams = {camera};
  expect_invalid(config, "integer limits");

  camera = CameraStream("rgb", "rgb");
  camera.height = kBeyondInt;
  config.sensor_streams = {camera};
  expect_invalid(config, "integer limits");

  auto lidar = LidarStream();
  lidar.horizontal_samples = kBeyondInt;
  config.sensor_streams = {lidar};
  expect_invalid(config, "integer limits");

  lidar = LidarStream();
  lidar.vertical_samples = kBeyondInt;
  config.sensor_streams = {lidar};
  expect_invalid(config, "integer limits");

  for (const auto& [kind, width, height] :
       std::vector<std::tuple<std::string, std::uint32_t, std::uint32_t>>{
           {"rgb", 30'000U, 6'000U},
           {"depth", 30'000U, 5'000U},
           {"segmentation", 30'000U, 5'000U},
           {"camera_pointcloud", 10'000U, 5'000U}}) {
    camera = CameraStream(kind, kind);
    camera.width = width;
    camera.height = height;
    config.sensor_streams = {camera};
    expect_invalid(config, "payload limits");
  }

  lidar = LidarStream();
  lidar.horizontal_samples = 100'000U;
  lidar.vertical_samples = 500U;
  config.sensor_streams = {lidar};
  expect_invalid(config, "payload limits");

  lidar = LidarStream();
  lidar.max_range_m =
      static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
  config.sensor_streams = {lidar};
  expect_invalid(config, "float range");
}

}  // namespace
}  // namespace mujoco_sim::server
