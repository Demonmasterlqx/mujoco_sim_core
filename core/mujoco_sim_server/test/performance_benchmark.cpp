#include "sensor_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <new>
#include <string>
#include <sys/resource.h>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>

namespace {

constexpr std::uint64_t kWarmupSteps = 2000;
constexpr std::uint64_t kMeasuredSteps = 10000;
constexpr std::size_t kObservationScalars = 32;
constexpr std::size_t kCommandScalars = 8;
constexpr std::size_t kPublishesPerTick = 3;
std::atomic<bool> track_allocations{false};
std::atomic<std::uint64_t> allocation_count{0};

struct SensorCounters {
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> bytes{0};
};

double CpuSeconds(const rusage& usage) {
  return static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
         static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) /
             1.0e6;
}

double Percentile(const std::vector<double>& sorted, double quantile) {
  return sorted[static_cast<std::size_t>(
      quantile * static_cast<double>(sorted.size() - 1))];
}

std::string ObservationName(std::size_t index) {
  return std::string("obs") + (index < 10 ? "0" : "") +
         std::to_string(index);
}

mujoco_sim::server::ServerConfig SensorConfig(const std::string& model) {
  mujoco_sim::server::ServerConfig config;
  config.server_name = "performance";
  config.model_path = model;
  config.headless = true;
  config.render_backend = "auto";
  config.sensor_subscriber_sample_period_ms = 1;
  mujoco_sim::server::ServerConfig::SensorStream stream;
  stream.name = "lidar";
  stream.kind = "lidar_pointcloud";
  stream.source = "lidar_site";
  stream.frame_id = "lidar";
  stream.rate_hz = 100.0;
  stream.horizontal_samples = 72;
  stream.vertical_samples = 4;
  stream.vertical_fov_min_deg = -10.0;
  stream.vertical_fov_max_deg = 10.0;
  stream.min_range_m = 0.05;
  stream.max_range_m = 8.0;
  stream.queue_mode = "latest";
  stream.queue_depth = 1;
  stream.fifo_overflow = "drop_oldest";
  config.sensor_streams.push_back(std::move(stream));
  return config;
}

}  // namespace

void* operator new(std::size_t size) {
  if (void* pointer = std::malloc(size == 0 ? 1 : size)) {
    if (track_allocations.load(std::memory_order_relaxed)) {
      allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    return pointer;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept {
  std::free(pointer);
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const std::string model_path = argv[1];
  mujoco_sim::core::SimulationCore core;
  const auto loaded = core.LoadModel(model_path);
  if (!loaded.response.loaded) return 3;

  mujoco_sim::core::RegisterAdapterRequest controller_request;
  controller_request.set_adapter_id("controller");
  controller_request.set_role(mujoco_sim::core::CONTROLLER);
  controller_request.set_commit_timeout_ms(1000);
  controller_request.set_session_timeout_ms(1000);
  for (std::size_t index = 0; index < kObservationScalars; ++index) {
    controller_request.add_observation_names(ObservationName(index));
  }
  for (std::size_t index = 0; index < kCommandScalars; ++index) {
    controller_request.add_command_names("motor" + std::to_string(index));
  }
  auto controller = core.RegisterAdapter(controller_request).response;
  if (controller.code() != mujoco_sim::core::OK) return 4;
  mujoco_sim::core::ActivateAdapterRequest activate_controller;
  activate_controller.set_adapter_id(controller.adapter_id());
  activate_controller.set_role(mujoco_sim::core::CONTROLLER);
  activate_controller.set_session_id(controller.session_id());
  activate_controller.set_epoch(controller.epoch());
  if (core.ActivateAdapter(activate_controller).response.code() !=
      mujoco_sim::core::OK) {
    return 5;
  }

  mujoco_sim::core::RegisterAdapterRequest observer_request;
  observer_request.set_adapter_id("observer");
  observer_request.set_role(mujoco_sim::core::OBSERVER);
  observer_request.set_session_timeout_ms(1000);
  observer_request.mutable_delivery()->set_period_s(0.01);
  for (std::size_t index = 0; index < kObservationScalars; ++index) {
    observer_request.add_observation_names(ObservationName(index));
  }
  auto observer = core.RegisterAdapter(observer_request).response;
  if (observer.code() != mujoco_sim::core::OK) return 6;
  mujoco_sim::core::ActivateAdapterRequest activate_observer;
  activate_observer.set_adapter_id(observer.adapter_id());
  activate_observer.set_role(mujoco_sim::core::OBSERVER);
  activate_observer.set_session_id(observer.session_id());
  activate_observer.set_epoch(observer.epoch());
  if (core.ActivateAdapter(activate_observer).response.code() !=
      mujoco_sim::core::OK) {
    return 7;
  }

  SensorCounters sensor_counters;
  mujoco_sim::server::SensorRuntimeTestHooks hooks;
  hooks.make_publisher = [&sensor_counters](const std::string&) {
    mujoco_sim::server::SensorRuntimePublisher publisher;
    publisher.lifetime = std::make_shared<int>(1);
    publisher.subscriber_count = [] { return 1U; };
    publisher.send = [&sensor_counters](const std::string& frame) {
      sensor_counters.frames.fetch_add(1, std::memory_order_relaxed);
      sensor_counters.bytes.fetch_add(frame.size(), std::memory_order_relaxed);
      return true;
    };
    return publisher;
  };
  mujoco_sim::server::SensorRuntime sensor_runtime(std::move(hooks));
  auto sensor_config = SensorConfig(model_path);
  std::string sensor_error;
  if (!sensor_runtime.Initialize(core, sensor_config, &sensor_error)) {
    std::cerr << sensor_error << '\n';
    return 8;
  }

  std::vector<double> barrier_us;
  barrier_us.reserve(kMeasuredSteps);
  std::uint64_t tick_bytes = 0;
  std::uint64_t commit_bytes = 0;
  std::uint64_t observation_frames = 0;
  std::uint64_t observation_bytes = 0;
  rusage before{};
  rusage after{};
  auto wall_start = std::chrono::steady_clock::now();
  for (std::uint64_t tick_id = 0;
       tick_id < kWarmupSteps + kMeasuredSteps; ++tick_id) {
    if (tick_id == kWarmupSteps) {
      sensor_counters.frames.store(0, std::memory_order_relaxed);
      sensor_counters.bytes.store(0, std::memory_order_relaxed);
      allocation_count.store(0, std::memory_order_relaxed);
      getrusage(RUSAGE_SELF, &before);
      wall_start = std::chrono::steady_clock::now();
      track_allocations.store(true, std::memory_order_relaxed);
    }
    const auto barrier_start = std::chrono::steady_clock::now();
    const auto ticks = core.OpenIssuedTicks().response;
    if (ticks.size() != 1) return 9;
    mujoco_sim::core::AdapterCommit commit;
    commit.identity.session = {controller.session_id(), controller.epoch()};
    commit.identity.tick_id = tick_id;
    commit.identity.layout = controller.layout().fingerprint;
    commit.commands.assign(kCommandScalars, 0.0);
    const auto committed = core.SubmitCommit(commit);
    if (committed.response.code != mujoco_sim::core::OK ||
        !committed.effects.step_completed) {
      return 10;
    }
    sensor_runtime.OnStepCompleted(core);
    if (tick_id >= kWarmupSteps) {
      tick_bytes += kPublishesPerTick *
                    (80U + ticks.front()->frame.observations.size() * 8U);
      commit_bytes += 80U + kCommandScalars * 8U;
      for (const auto& frame : committed.effects.step_completed->observer_frames) {
        ++observation_frames;
        observation_bytes += 80U + frame.observations.size() * 8U;
      }
      barrier_us.push_back(std::chrono::duration<double, std::micro>(
                               std::chrono::steady_clock::now() - barrier_start)
                               .count());
    }
  }
  sensor_runtime.Stop();
  track_allocations.store(false, std::memory_order_relaxed);
  const auto wall_seconds = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - wall_start)
                                .count();
  getrusage(RUSAGE_SELF, &after);
  std::sort(barrier_us.begin(), barrier_us.end());
  const auto sensor_frames = sensor_counters.frames.load();
  const auto sensor_bytes = sensor_counters.bytes.load();
  const auto total_frames = 30000U + 10000U + observation_frames + sensor_frames;
  const auto total_bytes =
      tick_bytes + commit_bytes + observation_bytes + sensor_bytes;
  std::cout << "{\"scenario\":\"D\",\"warmup_steps\":2000"
            << ",\"steps\":10000,\"controllers\":1"
            << ",\"observation_scalars\":32,\"command_scalars\":8"
            << ",\"tick_frames\":30000,\"tick_bytes\":" << tick_bytes
            << ",\"commit_frames\":10000,\"commit_bytes\":" << commit_bytes
            << ",\"observation_frames\":" << observation_frames
            << ",\"observation_bytes\":" << observation_bytes
            << ",\"sensor_frames\":" << sensor_frames
            << ",\"sensor_bytes\":" << sensor_bytes
            << ",\"total_frames\":" << total_frames
            << ",\"total_bytes\":" << total_bytes
            << ",\"barrier_p50_us\":" << Percentile(barrier_us, 0.50)
            << ",\"barrier_p95_us\":" << Percentile(barrier_us, 0.95)
            << ",\"barrier_p99_us\":" << Percentile(barrier_us, 0.99)
            << ",\"wall_seconds\":" << wall_seconds
            << ",\"cpu_seconds\":" << CpuSeconds(after) - CpuSeconds(before)
            << ",\"simulation_seconds\":10,\"rtf\":"
            << 10.0 / wall_seconds << ",\"allocations_per_step\":"
            << static_cast<double>(allocation_count.load()) / kMeasuredSteps
            << ",\"queue_high_watermark\":1,\"max_rss_kib\":"
            << after.ru_maxrss << "}\n";
  return 0;
}
