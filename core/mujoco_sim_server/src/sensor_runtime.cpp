#include "sensor_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ecal/pubsub/publisher.h>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/sensor/sensor.hpp>

namespace mujoco_sim::server {
namespace {

using Clock = std::chrono::steady_clock;

mujoco_sim::v4::StreamFormat FormatForKind(const std::string& kind) {
  if (kind == "rgb") return mujoco_sim::v4::RGB8;
  if (kind == "depth") return mujoco_sim::v4::DEPTH32F;
  if (kind == "segmentation") return mujoco_sim::v4::SEGMENTATION32U;
  return mujoco_sim::v4::XYZ32F;
}

mujoco_sim::sensor::BackendPreference BackendFor(const std::string& backend, bool headless) {
  if (backend == "egl") {
    return mujoco_sim::sensor::BackendPreference::kEglOnly;
  }
  if (backend == "osmesa") {
    return mujoco_sim::sensor::BackendPreference::kOsMesaOnly;
  }
  if (!headless) {
    return mujoco_sim::sensor::BackendPreference::kEglOnly;
  }
  return mujoco_sim::sensor::BackendPreference::kAutomatic;
}

enum class QueueMode {
  kLatest,
  kFifo,
  kBlocking
};
enum class Overflow {
  kDropOldest,
  kDropNewest
};

QueueMode ParseQueueMode(const std::string& mode) {
  if (mode == "fifo") return QueueMode::kFifo;
  if (mode == "blocking") return QueueMode::kBlocking;
  return QueueMode::kLatest;
}

struct CameraGroupKey {
  std::string source;
  std::string frame_id;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double rate_hz = 0.0;
  std::string queue_mode;
  std::uint32_t queue_depth = 0;
  std::string overflow;

  auto operator<=>(const CameraGroupKey&) const = default;
};

}  // namespace

struct SensorRuntime::Impl {
  struct StreamInfo {
    mujoco_sim::v4::StreamDescriptor descriptor;
    SensorRuntimePublisher publisher;
    mujoco_sim::v4::StreamState state = mujoco_sim::v4::STREAM_IDLE;
    std::string message;
    std::uint64_t subscribers = 0;
    std::uint64_t captures = 0;
    std::uint64_t published = 0;
    std::uint64_t dropped = 0;
    std::uint64_t publish_errors = 0;
    double blocked_time_ms = 0.0;
    std::optional<std::uint64_t> last_capture_tick;
  };

  struct Job {
    std::unique_ptr<mujoco_sim::core::SimulationSnapshot> snapshot;
    std::uint64_t epoch = 0;
    std::uint64_t tick_id = 0;
    double simulation_time_s = 0.0;
    std::uint64_t capture_id = 0;
    std::uint64_t dropped_since_previous = 0;
  };

  struct Worker {
    std::string server_name;
    std::string source_name;
    QueueMode queue_mode = QueueMode::kLatest;
    Overflow overflow = Overflow::kDropOldest;
    std::size_t queue_depth = 1;
    double rate_hz = 10.0;
    double next_due_time = 0.0;
    bool is_camera = false;
    mujoco_sim::sensor::CameraProfile camera_profile;
    mujoco_sim::sensor::RayLidarProfile lidar_profile;
    mujoco_sim::sensor::OffscreenRendererOptions renderer_options;
    std::vector<StreamInfo> streams;

    mutable std::mutex mutex;
    std::mutex publisher_mutex;
    std::condition_variable condition;
    std::condition_variable ready_condition;
    std::deque<Job> jobs;
    std::optional<Job> pending_blocking_job;
    std::vector<std::unique_ptr<mujoco_sim::core::SimulationSnapshot>> free;
    std::thread thread;
    bool stopping = false;
    bool ready = false;
    std::string initialization_error;
    std::uint64_t sequence = 0;
    std::uint64_t pending_dropped = 0;
    std::uint64_t latest_blocking_capture_id = 0;
    std::uint64_t completed_capture_id = 0;
    std::uint64_t acknowledged_blocking_capture_id = 0;
    std::function<void(const std::string&)> before_capture;
    std::function<void(const std::string&)> on_blocking_wait_started;
    std::function<void(mujoco_sim::v4::StreamFrameHeader*)> before_serialize;
    std::function<void(const std::string&)> before_initialize;

    void UpdateSubscriberCache() {
      for (auto& stream : streams) {
        std::size_t subscribers = 0;
        try {
          std::lock_guard publisher_lock(publisher_mutex);
          subscribers = stream.publisher.subscriber_count();
        } catch (const std::exception& exception) {
          std::lock_guard lock(mutex);
          stream.state = mujoco_sim::v4::STREAM_ERROR;
          stream.message = std::string("subscriber query threw: ") + exception.what();
          ++stream.publish_errors;
          continue;
        } catch (...) {
          std::lock_guard lock(mutex);
          stream.state = mujoco_sim::v4::STREAM_ERROR;
          stream.message = "subscriber query threw";
          ++stream.publish_errors;
          continue;
        }
        std::lock_guard lock(mutex);
        stream.subscribers = subscribers;
        if (stream.state != mujoco_sim::v4::STREAM_ERROR && stream.subscribers == 0) {
          stream.state = mujoco_sim::v4::STREAM_IDLE;
          stream.message.clear();
        }
      }
    }

    bool HasLiveSubscriber() const {
      std::lock_guard lock(mutex);
      const bool live = std::any_of(streams.begin(), streams.end(), [](const auto& stream) {
        return stream.state != mujoco_sim::v4::STREAM_ERROR && stream.subscribers > 0;
      });
      return live;
    }

    bool AllStreamsFailedLocked() const {
      return std::all_of(streams.begin(), streams.end(), [](const auto& info) {
        return info.state == mujoco_sim::v4::STREAM_ERROR;
      });
    }

    void MarkDroppedLocked(std::uint64_t count = 1, std::uint64_t carried = 0) {
      pending_dropped += count + carried;
      for (auto& stream : streams)
        stream.dropped += count;
    }

    bool Enqueue(mujoco_sim::core::SimulationCore& core, std::uint64_t capture_id) {
      const auto core_state = core.state();
      const double simulation_time = core_state.simulation_time_s;
      const double period = 1.0 / rate_hz;
      const bool has_live_subscriber = HasLiveSubscriber();
      if (simulation_time + 1e-12 < next_due_time) return true;
      if (next_due_time <= 0.0) {
        next_due_time = simulation_time + period;
      } else {
        const double due_limit = simulation_time + 1e-12;
        const double intervals = std::floor((due_limit - next_due_time) / period) + 1.0;
        const double advanced = next_due_time + intervals * period;
        next_due_time = std::isfinite(advanced) && advanced > due_limit
                            ? advanced
                            : std::nextafter(due_limit, std::numeric_limits<double>::infinity());
      }
      if (!has_live_subscriber) return true;

      std::unique_lock lock(mutex);
      if (stopping || AllStreamsFailedLocked()) return true;
      bool defer_blocking_enqueue = false;
      if (queue_mode == QueueMode::kBlocking) {
        if (pending_blocking_job.has_value()) {
          for (auto& stream : streams) {
            stream.state = mujoco_sim::v4::STREAM_ERROR;
            stream.message = "blocking sensor enqueue overlapped";
          }
          condition.notify_all();
          return false;
        }
        defer_blocking_enqueue = jobs.size() >= queue_depth || free.size() <= 1;
      } else if (jobs.size() >= queue_depth) {
        if (queue_mode == QueueMode::kLatest || overflow == Overflow::kDropOldest) {
          const std::uint64_t carried = jobs.front().dropped_since_previous;
          free.push_back(std::move(jobs.front().snapshot));
          jobs.pop_front();
          MarkDroppedLocked(1, carried);
        } else {
          MarkDroppedLocked();
          return true;
        }
      }
      if (free.empty()) {
        MarkDroppedLocked();
        return true;
      }
      auto snapshot = std::move(free.back());
      free.pop_back();
      lock.unlock();
      std::string error;
      if (!core.UpdateSnapshot(*snapshot, &error)) {
        lock.lock();
        free.push_back(std::move(snapshot));
        for (auto& stream : streams) {
          stream.state = mujoco_sim::v4::STREAM_ERROR;
          stream.message = "snapshot update failed: " + error;
        }
        lock.unlock();
        condition.notify_all();
        return false;
      }

      Job job;
      job.snapshot = std::move(snapshot);
      job.capture_id = capture_id;
      job.epoch = core_state.epoch;
      job.tick_id = core_state.current_tick_id;
      job.simulation_time_s = simulation_time;
      lock.lock();
      job.dropped_since_previous = pending_dropped;
      pending_dropped = 0;
      if (defer_blocking_enqueue) {
        pending_blocking_job = std::move(job);
      } else {
        jobs.push_back(std::move(job));
      }
      if (queue_mode == QueueMode::kBlocking) {
        latest_blocking_capture_id = capture_id;
      }
      lock.unlock();
      condition.notify_all();
      return true;
    }

    void CompleteBlockingEnqueue(std::stop_token stop_token) {
      if (queue_mode != QueueMode::kBlocking) return;
      const auto blocked_at = Clock::now();
      std::unique_lock lock(mutex);
      const std::uint64_t target_capture_id = latest_blocking_capture_id;
      if (target_capture_id == 0 || acknowledged_blocking_capture_id >= target_capture_id) {
        return;
      }
      if (completed_capture_id >= target_capture_id) {
        acknowledged_blocking_capture_id = target_capture_id;
        return;
      }

      bool wait_reported = false;
      while (!stopping && !AllStreamsFailedLocked() && completed_capture_id < target_capture_id) {
        if (stop_token.stop_requested()) {
          stopping = true;
          condition.notify_all();
          break;
        }
        if (pending_blocking_job.has_value() && jobs.size() < queue_depth && !free.empty()) {
          jobs.push_back(std::move(*pending_blocking_job));
          pending_blocking_job.reset();
          condition.notify_all();
          continue;
        }
        if (!wait_reported && on_blocking_wait_started) {
          wait_reported = true;
          auto callback = on_blocking_wait_started;
          const auto source = source_name;
          lock.unlock();
          std::string callback_error;
          try {
            callback(source);
          } catch (const std::exception& exception) {
            callback_error = std::string("blocking wait callback threw: ") + exception.what();
          } catch (...) {
            callback_error = "blocking wait callback threw";
          }
          lock.lock();
          if (!callback_error.empty()) {
            for (auto& stream : streams) {
              stream.state = mujoco_sim::v4::STREAM_ERROR;
              stream.message = callback_error;
            }
            condition.notify_all();
            break;
          }
          continue;
        }
        condition.wait_for(lock, std::chrono::milliseconds(50), [this, target_capture_id] {
          return stopping || AllStreamsFailedLocked() ||
                 completed_capture_id >= target_capture_id ||
                 (pending_blocking_job.has_value() && jobs.size() < queue_depth && !free.empty());
        });
      }
      const double blocked_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - blocked_at).count();
      for (auto& stream : streams)
        stream.blocked_time_ms += blocked_ms;
      if (pending_blocking_job.has_value() && (stopping || AllStreamsFailedLocked())) {
        free.push_back(std::move(pending_blocking_job->snapshot));
        pending_blocking_job.reset();
      }
      acknowledged_blocking_capture_id = target_capture_id;
    }

    void SetCaptureError(const std::string& error) {
      std::lock_guard lock(mutex);
      for (auto& stream : streams) {
        stream.state = mujoco_sim::v4::STREAM_ERROR;
        stream.message = error;
      }
      while (!jobs.empty()) {
        free.push_back(std::move(jobs.front().snapshot));
        jobs.pop_front();
      }
      if (pending_blocking_job.has_value()) {
        free.push_back(std::move(pending_blocking_job->snapshot));
        pending_blocking_job.reset();
      }
      condition.notify_all();
    }

    void SetStreamError(StreamInfo* stream, const std::string& error) {
      std::lock_guard lock(mutex);
      stream->state = mujoco_sim::v4::STREAM_ERROR;
      stream->message = error;
      ++stream->publish_errors;
      condition.notify_all();
    }

    void HandleUnhandledFailure(const std::string& error) noexcept {
      try {
        std::lock_guard lock(mutex);
        if (!ready) {
          initialization_error = error;
          ready = true;
        } else {
          stopping = true;
          for (auto& stream : streams) {
            stream.state = mujoco_sim::v4::STREAM_ERROR;
            stream.message = error;
          }
          while (!jobs.empty()) {
            free.push_back(std::move(jobs.front().snapshot));
            jobs.pop_front();
          }
          if (pending_blocking_job.has_value()) {
            free.push_back(std::move(pending_blocking_job->snapshot));
            pending_blocking_job.reset();
          }
        }
      } catch (...) {
      }
      ready_condition.notify_all();
      condition.notify_all();
    }

    void RunGuarded() {
      std::unique_ptr<mujoco_sim::sensor::OffscreenRenderer> renderer;
      std::unique_ptr<mujoco_sim::sensor::RayLidar> lidar;
      {
        std::unique_lock lock(mutex);
        auto* initial = free.empty() ? nullptr : free.front().get();
        lock.unlock();
        std::string error;
        if (before_initialize) before_initialize(source_name);
        if (initial == nullptr) {
          error = "sensor worker has no snapshot";
        } else if (is_camera) {
          renderer = mujoco_sim::sensor::OffscreenRenderer::Create(*initial, camera_profile,
                                                                   renderer_options, &error);
        } else {
          lidar = mujoco_sim::sensor::RayLidar::Create(*initial, lidar_profile, &error);
        }
        lock.lock();
        if ((!renderer && is_camera) || (!lidar && !is_camera)) {
          initialization_error = error.empty() ? "sensor initialization failed" : std::move(error);
        }
        ready = true;
        lock.unlock();
        ready_condition.notify_all();
      }
      if (!initialization_error.empty()) return;

      while (true) {
        Job job;
        {
          std::unique_lock lock(mutex);
          condition.wait(lock, [this] { return stopping || !jobs.empty(); });
          if (stopping && jobs.empty()) break;
          job = std::move(jobs.front());
          jobs.pop_front();
          condition.notify_all();
        }

        try {
          mujoco_sim::sensor::CaptureMetadata metadata;
          metadata.epoch = job.epoch;
          metadata.tick_id = job.tick_id;
          metadata.simulation_time_s = job.simulation_time_s;
          metadata.capture_id = job.capture_id;
          metadata.stream_sequence = ++sequence;
          metadata.dropped_since_previous = job.dropped_since_previous;
          std::vector<mujoco_sim::sensor::RawFrame> frames;
          std::string error;
          bool captured = false;
          if (before_capture) before_capture(source_name);
          if (is_camera) {
            captured = renderer->Capture(*job.snapshot, metadata, &frames, &error);
          } else {
            mujoco_sim::sensor::RawFrame frame;
            captured = lidar->Capture(*job.snapshot, metadata, &frame, &error);
            if (captured) frames.push_back(std::move(frame));
          }
          if (!captured) {
            SetCaptureError("capture failed: " + error);
          } else {
            Publish(job.tick_id, frames);
          }
        } catch (const std::exception& exception) {
          SetCaptureError(std::string("sensor job threw: ") + exception.what());
        } catch (...) {
          SetCaptureError("sensor job threw");
        }
        {
          std::lock_guard lock(mutex);
          completed_capture_id = std::max(completed_capture_id, job.capture_id);
          free.push_back(std::move(job.snapshot));
        }
        condition.notify_all();
      }
    }

    void Run() noexcept {
      try {
        RunGuarded();
      } catch (const std::exception& exception) {
        HandleUnhandledFailure(std::string("sensor worker threw: ") + exception.what());
      } catch (...) {
        HandleUnhandledFailure("sensor worker threw");
      }
    }

    void Publish(std::uint64_t tick_id, const std::vector<mujoco_sim::sensor::RawFrame>& frames) {
      for (const auto& frame : frames) {
        const auto found =
            std::find_if(streams.begin(), streams.end(), [&frame](const auto& stream) {
              return stream.descriptor.name() == frame.header.stream_name();
            });
        if (found == streams.end()) continue;
        {
          std::lock_guard lock(mutex);
          ++found->captures;
          found->last_capture_tick = tick_id;
          if (found->state == mujoco_sim::v4::STREAM_ERROR) continue;
        }
        try {
          std::string bytes;
          std::string error;
          auto frame_header = frame.header;
          if (before_serialize) before_serialize(&frame_header);
          if (!mujoco_sim::protocol::SerializeStreamFrame(frame_header, frame.payload, &bytes,
                                                          &error)) {
            SetStreamError(&*found, error);
            continue;
          }
          bool has_subscriber = false;
          {
            std::lock_guard lock(mutex);
            has_subscriber = found->subscribers > 0;
          }
          if (!has_subscriber) continue;
          bool sent = false;
          {
            std::lock_guard publisher_lock(publisher_mutex);
            sent = found->publisher.send(bytes);
          }
          std::lock_guard lock(mutex);
          if (sent) {
            found->state = mujoco_sim::v4::STREAM_RUNNING;
            found->message.clear();
            ++found->published;
          } else {
            found->state = mujoco_sim::v4::STREAM_RUNNING;
            found->message = "eCAL rejected stream frame publish; retrying";
            ++found->publish_errors;
            condition.notify_all();
          }
        } catch (const std::exception& exception) {
          SetStreamError(&*found, std::string("stream publish threw: ") + exception.what());
        } catch (...) {
          SetStreamError(&*found, "stream publish threw");
        }
      }
    }

    void RequestStop() {
      {
        std::lock_guard lock(mutex);
        stopping = true;
      }
      condition.notify_all();
    }

    void Stop() {
      RequestStop();
      if (thread.joinable()) thread.join();
    }

    void AppendStatus(mujoco_sim::v4::ServerStatus* status) const {
      std::lock_guard lock(mutex);
      for (const auto& stream : streams) {
        auto* output = status->add_streams();
        output->set_name(stream.descriptor.name());
        output->set_state(stream.state);
        output->set_message(stream.message);
        output->set_subscriber_count(stream.subscribers);
        output->set_captures(stream.captures);
        output->set_published(stream.published);
        output->set_dropped(stream.dropped);
        output->set_publish_errors(stream.publish_errors);
        output->set_blocked_time_ms(stream.blocked_time_ms);
        if (stream.last_capture_tick) {
          output->set_last_capture_tick_id(*stream.last_capture_tick);
        }
      }
    }
  };

  std::string server_name;
  std::uint64_t epoch = 0;
  std::vector<std::unique_ptr<Worker>> workers;
  std::atomic<std::uint64_t> next_capture_id{1};
  std::chrono::milliseconds subscriber_sample_period{200};
  Clock::time_point next_subscriber_sample{};
  bool initialized = false;
  SensorRuntimeTestHooks hooks;

  explicit Impl(SensorRuntimeTestHooks runtime_hooks) : hooks(std::move(runtime_hooks)) {}

  bool AddWorker(mujoco_sim::core::SimulationCore& core, std::unique_ptr<Worker> worker,
                 std::string* error_message) {
    const std::size_t pool_size =
        worker->queue_depth + (worker->queue_mode == QueueMode::kBlocking ? 2U : 1U);
    auto first = core.CreateSnapshot(error_message);
    if (!first) return false;
    worker->free.push_back(std::move(first));
    for (std::size_t i = 1; i < pool_size; ++i) {
      auto snapshot = core.CloneSnapshot(*worker->free.front(), error_message);
      if (!snapshot) return false;
      worker->free.push_back(std::move(snapshot));
    }
    for (auto& stream : worker->streams) {
      try {
        if (hooks.make_publisher) {
          stream.publisher = hooks.make_publisher(stream.descriptor.topic());
        } else {
          auto publisher = std::make_shared<eCAL::CPublisher>(stream.descriptor.topic());
          stream.publisher.lifetime = publisher;
          stream.publisher.subscriber_count = [publisher] {
            return publisher->GetSubscriberCount();
          };
          stream.publisher.send = [publisher](const std::string& payload) {
            return publisher->Send(payload);
          };
        }
      } catch (const std::exception& exception) {
        *error_message = std::string("sensor publisher creation threw: ") + exception.what();
        return false;
      } catch (...) {
        *error_message = "sensor publisher creation threw";
        return false;
      }
      if (!stream.publisher.subscriber_count || !stream.publisher.send) {
        *error_message = "sensor publisher factory returned invalid callbacks";
        return false;
      }
    }
    worker->before_capture = hooks.before_capture;
    worker->on_blocking_wait_started = hooks.on_blocking_wait_started;
    worker->before_serialize = hooks.before_serialize;
    worker->before_initialize = hooks.before_initialize;
    Worker* raw = worker.get();
    raw->thread = std::thread([raw] { raw->Run(); });
    {
      std::unique_lock lock(raw->mutex);
      raw->ready_condition.wait(lock, [raw] { return raw->ready; });
      if (!raw->initialization_error.empty()) {
        *error_message = raw->initialization_error;
        lock.unlock();
        raw->Stop();
        return false;
      }
    }
    workers.push_back(std::move(worker));
    return true;
  }
};

SensorRuntime::SensorRuntime() : SensorRuntime(SensorRuntimeTestHooks{}) {}
SensorRuntime::SensorRuntime(SensorRuntimeTestHooks hooks)
    : hooks_(std::move(hooks)), impl_(std::make_unique<Impl>(hooks_)) {}
SensorRuntime::~SensorRuntime() {
  Stop();
}

bool SensorRuntime::Initialize(mujoco_sim::core::SimulationCore& core, const ServerConfig& config,
                               std::string* error_message) {
  if (error_message == nullptr || !core.IsModelLoaded()) {
    if (error_message) *error_message = "sensor runtime requires a loaded core";
    return false;
  }
  if (!ValidateServerConfig(config, error_message)) {
    return false;
  }
  Stop();
  impl_ = std::make_unique<Impl>(hooks_);
  impl_->server_name = config.server_name;
  impl_->epoch = core.state().epoch;
  impl_->subscriber_sample_period =
      std::chrono::milliseconds(config.sensor_subscriber_sample_period_ms);
  impl_->next_subscriber_sample = Clock::now();
  if (config.sensor_streams.empty()) {
    impl_->initialized = true;
    return true;
  }
  auto validation_snapshot = core.CreateSnapshot(error_message);
  if (!validation_snapshot) return false;

  std::map<CameraGroupKey, std::vector<ServerConfig::SensorStream>> cameras;
  std::vector<ServerConfig::SensorStream> lidars;
  for (const auto& stream : config.sensor_streams) {
    if (stream.kind == "lidar_pointcloud") {
      lidars.push_back(stream);
    } else {
      cameras[{stream.source, stream.frame_id, stream.width, stream.height, stream.rate_hz,
               stream.queue_mode, stream.queue_depth, stream.fifo_overflow}]
          .push_back(stream);
    }
  }

  for (const auto& [key, outputs] : cameras) {
    auto worker = std::make_unique<Impl::Worker>();
    worker->server_name = config.server_name;
    worker->source_name = key.source;
    worker->is_camera = true;
    worker->queue_mode = ParseQueueMode(key.queue_mode);
    worker->overflow =
        key.overflow == "drop_newest" ? Overflow::kDropNewest : Overflow::kDropOldest;
    worker->queue_depth = key.queue_depth;
    worker->rate_hz = key.rate_hz;
    worker->renderer_options.backend = BackendFor(config.render_backend, config.headless);
    worker->camera_profile.camera_name = key.source;
    worker->camera_profile.width = key.width;
    worker->camera_profile.height = key.height;
    worker->camera_profile.rate_hz = key.rate_hz;
    worker->camera_profile.frame_id = key.frame_id;
    for (const auto& output : outputs) {
      worker->camera_profile.outputs.push_back({FormatForKind(output.kind), output.name});
    }
    std::vector<mujoco_sim::v4::StreamDescriptor> descriptors;
    if (!mujoco_sim::sensor::DescribeCameraStreams(*validation_snapshot, worker->camera_profile,
                                                   &descriptors, error_message)) {
      Stop();
      return false;
    }
    for (auto& descriptor : descriptors) {
      descriptor.set_topic(
          mujoco_sim::protocol::SensorStreamTopicName(config.server_name, descriptor.name()));
      Impl::StreamInfo info;
      info.descriptor = std::move(descriptor);
      worker->streams.push_back(std::move(info));
    }
    if (!impl_->AddWorker(core, std::move(worker), error_message)) {
      Stop();
      return false;
    }
  }

  for (const auto& source : lidars) {
    auto worker = std::make_unique<Impl::Worker>();
    worker->server_name = config.server_name;
    worker->source_name = source.source;
    worker->queue_mode = ParseQueueMode(source.queue_mode);
    worker->overflow =
        source.fifo_overflow == "drop_newest" ? Overflow::kDropNewest : Overflow::kDropOldest;
    worker->queue_depth = source.queue_depth;
    worker->rate_hz = source.rate_hz;
    worker->lidar_profile.site_name = source.source;
    worker->lidar_profile.horizontal_samples = source.horizontal_samples;
    worker->lidar_profile.vertical_fov = mujoco_sim::sensor::VerticalFieldOfView{
        source.vertical_samples, source.vertical_fov_min_deg * std::numbers::pi / 180.0,
        source.vertical_fov_max_deg * std::numbers::pi / 180.0};
    worker->lidar_profile.min_range_m = source.min_range_m;
    worker->lidar_profile.max_range_m = source.max_range_m;
    worker->lidar_profile.rate_hz = source.rate_hz;
    worker->lidar_profile.frame_id = source.frame_id;
    worker->lidar_profile.stream_name = source.name;
    mujoco_sim::v4::StreamDescriptor descriptor;
    if (!mujoco_sim::sensor::DescribeRayLidarStream(*validation_snapshot, worker->lidar_profile,
                                                    &descriptor, error_message)) {
      Stop();
      return false;
    }
    descriptor.set_topic(
        mujoco_sim::protocol::SensorStreamTopicName(config.server_name, descriptor.name()));
    Impl::StreamInfo info;
    info.descriptor = std::move(descriptor);
    worker->streams.push_back(std::move(info));
    if (!impl_->AddWorker(core, std::move(worker), error_message)) {
      Stop();
      return false;
    }
  }
  impl_->initialized = true;
  return true;
}

void SensorRuntime::OnStepCompleted(mujoco_sim::core::SimulationCore& core) {
  if (!impl_->initialized) return;
  const auto now = Clock::now();
  if (now >= impl_->next_subscriber_sample) {
    for (auto& worker : impl_->workers)
      worker->UpdateSubscriberCache();
    impl_->next_subscriber_sample = now + impl_->subscriber_sample_period;
  }
  for (auto& worker : impl_->workers) {
    worker->Enqueue(core, impl_->next_capture_id.fetch_add(1));
  }
}

void SensorRuntime::CompleteBlockingEnqueues(std::stop_token stop_token) {
  if (!impl_->initialized) return;
  for (auto& worker : impl_->workers) {
    worker->CompleteBlockingEnqueue(stop_token);
  }
}

void SensorRuntime::RequestStop() {
  if (!impl_) return;
  for (auto& worker : impl_->workers)
    worker->RequestStop();
}

void SensorRuntime::Stop() {
  if (!impl_) return;
  RequestStop();
  for (auto& worker : impl_->workers)
    worker->Stop();
  impl_->workers.clear();
  impl_->initialized = false;
}

mujoco_sim::v4::GetSensorStreamsResponse SensorRuntime::GetCatalog(
    const mujoco_sim::v4::GetSensorStreamsRequest&) const {
  mujoco_sim::v4::GetSensorStreamsResponse response;
  if (!impl_->initialized) {
    response.set_code(mujoco_sim::v4::FAILED_PRECONDITION);
    response.set_message("sensor runtime is not initialized");
    return response;
  }
  response.set_code(mujoco_sim::v4::OK);
  response.set_epoch(impl_->epoch);
  for (const auto& worker : impl_->workers) {
    std::lock_guard lock(worker->mutex);
    for (const auto& stream : worker->streams) {
      *response.add_streams() = stream.descriptor;
    }
  }
  return response;
}

void SensorRuntime::AppendStatus(mujoco_sim::v4::ServerStatus* status) const {
  if (status == nullptr || !impl_->initialized) return;
  for (const auto& worker : impl_->workers)
    worker->AppendStatus(status);
}

}  // namespace mujoco_sim::server
