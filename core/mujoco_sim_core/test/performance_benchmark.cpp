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
constexpr std::size_t kPublishesPerTick = 3;
std::atomic<bool> track_allocations{false};
std::atomic<std::uint64_t> allocation_count{0};

struct Scenario {
  char name = 'B';
  std::size_t controllers = 8;
  std::size_t observations = 30;
  std::size_t commands = 1;
  bool observer = false;
};

Scenario ParseScenario(const char* value) {
  if (value == nullptr || value[0] == 'B') return {};
  if (value[0] == 'A') return {'A', 1, 32, 8, false};
  if (value[0] == 'C') return {'C', 1, 32, 8, true};
  return {'\0', 0, 0, 0, false};
}

double CpuSeconds(const rusage& usage) {
  return static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
         static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) /
             1.0e6;
}

double Percentile(const std::vector<double>& sorted, double quantile) {
  const auto index = static_cast<std::size_t>(
      quantile * static_cast<double>(sorted.size() - 1));
  return sorted[index];
}

std::string ObservationName(std::size_t index) {
  return std::string("obs") + (index < 10 ? "0" : "") +
         std::to_string(index);
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
  if (argc < 2 || argc > 3) return 2;
  const Scenario scenario = ParseScenario(argc == 3 ? argv[2] : "B");
  if (scenario.name == '\0') return 2;

  mujoco_sim::core::SimulationCore core;
  const auto loaded = core.LoadModel(argv[1]);
  if (!loaded.response.loaded) {
    std::cerr << loaded.response.message << '\n';
    return 3;
  }

  std::vector<mujoco_sim::core::RegisterAdapterResponse> registrations;
  for (std::size_t controller = 0; controller < scenario.controllers;
       ++controller) {
    mujoco_sim::core::RegisterAdapterRequest request;
    request.set_adapter_id("controller" + std::to_string(controller));
    request.set_role(mujoco_sim::core::CONTROLLER);
    request.set_commit_timeout_ms(1000);
    request.set_session_timeout_ms(1000);
    for (std::size_t observation = 0; observation < scenario.observations;
         ++observation) {
      request.add_observation_names(ObservationName(observation));
    }
    for (std::size_t command = 0; command < scenario.commands; ++command) {
      request.add_command_names(
          "motor" + std::to_string(scenario.controllers == 1 ? command
                                                              : controller));
    }
    auto registration = core.RegisterAdapter(request).response;
    if (registration.code() != mujoco_sim::core::OK) return 4;
    mujoco_sim::core::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::core::CONTROLLER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    if (core.ActivateAdapter(activate).response.code() !=
        mujoco_sim::core::OK) {
      return 5;
    }
    registrations.push_back(std::move(registration));
  }

  if (scenario.observer) {
    mujoco_sim::core::RegisterAdapterRequest request;
    request.set_adapter_id("observer");
    request.set_role(mujoco_sim::core::OBSERVER);
    request.set_session_timeout_ms(1000);
    request.mutable_delivery()->set_period_s(0.01);
    for (std::size_t observation = 0; observation < scenario.observations;
         ++observation) {
      request.add_observation_names(ObservationName(observation));
    }
    auto registration = core.RegisterAdapter(request).response;
    if (registration.code() != mujoco_sim::core::OK) return 10;
    mujoco_sim::core::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::core::OBSERVER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    if (core.ActivateAdapter(activate).response.code() !=
        mujoco_sim::core::OK) {
      return 11;
    }
  }

  std::vector<double> barrier_us;
  barrier_us.reserve(kMeasuredSteps);
  std::uint64_t tick_frames = 0;
  std::uint64_t tick_bytes = 0;
  std::uint64_t commit_frames = 0;
  std::uint64_t commit_bytes = 0;
  std::uint64_t observation_frames = 0;
  std::uint64_t observation_bytes = 0;
  rusage before{};
  rusage after{};
  auto wall_start = std::chrono::steady_clock::now();

  const auto total_steps = kWarmupSteps + kMeasuredSteps;
  for (std::uint64_t tick_id = 0; tick_id < total_steps; ++tick_id) {
    if (tick_id == kWarmupSteps) {
      allocation_count.store(0, std::memory_order_relaxed);
      getrusage(RUSAGE_SELF, &before);
      wall_start = std::chrono::steady_clock::now();
      track_allocations.store(true, std::memory_order_relaxed);
    }
    const auto barrier_start = std::chrono::steady_clock::now();
    const auto ticks = core.OpenIssuedTicks().response;
    if (ticks.size() != scenario.controllers) return 6;
    if (tick_id >= kWarmupSteps) {
      for (const auto& tick : ticks) {
        const auto bytes = 80U + tick->frame.observations.size() * 8U;
        tick_frames += kPublishesPerTick;
        tick_bytes += kPublishesPerTick * bytes;
      }
    }
    for (std::size_t controller = 0; controller < scenario.controllers;
         ++controller) {
      mujoco_sim::core::AdapterCommit commit;
      commit.identity.session = {registrations[controller].session_id(),
                                 registrations[controller].epoch()};
      commit.identity.tick_id = tick_id;
      commit.identity.layout = registrations[controller].layout().fingerprint;
      commit.commands.assign(scenario.commands, 0.0);
      const auto result = core.SubmitCommit(commit);
      if (result.response.code != mujoco_sim::core::OK) return 7;
      if ((controller + 1 == scenario.controllers) !=
          result.effects.step_completed.has_value()) {
        return 8;
      }
      if (tick_id >= kWarmupSteps) {
        ++commit_frames;
        commit_bytes += 80U + commit.commands.size() * 8U;
        if (result.effects.step_completed) {
          for (const auto& frame :
               result.effects.step_completed->observer_frames) {
            ++observation_frames;
            observation_bytes += 80U + frame.observations.size() * 8U;
          }
        }
      }
    }
    if (tick_id >= kWarmupSteps) {
      barrier_us.push_back(std::chrono::duration<double, std::micro>(
                               std::chrono::steady_clock::now() - barrier_start)
                               .count());
    }
  }
  track_allocations.store(false, std::memory_order_relaxed);

  const auto wall_seconds = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - wall_start)
                                .count();
  getrusage(RUSAGE_SELF, &after);
  std::sort(barrier_us.begin(), barrier_us.end());
  const double simulation_seconds =
      core.state().simulation_time_s - kWarmupSteps * 0.001;
  const auto total_frames = tick_frames + commit_frames + observation_frames;
  const auto total_bytes = tick_bytes + commit_bytes + observation_bytes;
  std::cout << "{\"scenario\":\"" << scenario.name
            << "\",\"warmup_steps\":" << kWarmupSteps
            << ",\"steps\":" << kMeasuredSteps
            << ",\"controllers\":" << scenario.controllers
            << ",\"observation_scalars\":" << scenario.observations
            << ",\"command_scalars\":" << scenario.commands
            << ",\"tick_frames\":" << tick_frames
            << ",\"tick_bytes\":" << tick_bytes
            << ",\"commit_frames\":" << commit_frames
            << ",\"commit_bytes\":" << commit_bytes
            << ",\"observation_frames\":" << observation_frames
            << ",\"observation_bytes\":" << observation_bytes
            << ",\"total_frames\":" << total_frames
            << ",\"total_bytes\":" << total_bytes
            << ",\"barrier_p50_us\":" << Percentile(barrier_us, 0.50)
            << ",\"barrier_p95_us\":" << Percentile(barrier_us, 0.95)
            << ",\"barrier_p99_us\":" << Percentile(barrier_us, 0.99)
            << ",\"wall_seconds\":" << wall_seconds
            << ",\"cpu_seconds\":" << CpuSeconds(after) - CpuSeconds(before)
            << ",\"simulation_seconds\":" << simulation_seconds
            << ",\"rtf\":" << simulation_seconds / wall_seconds
            << ",\"allocations_per_step\":"
            << static_cast<double>(allocation_count.load()) / kMeasuredSteps
            << ",\"queue_high_watermark\":0"
            << ",\"max_rss_kib\":" << after.ru_maxrss << "}\n";
  return 0;
}
