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
         static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1.0e6;
}

double Percentile(const std::vector<double>& sorted, double quantile) {
  const auto index = static_cast<std::size_t>(quantile * static_cast<double>(sorted.size() - 1));
  return sorted[index];
}

std::string ObservationName(std::size_t index) {
  return std::string("obs") + (index < 10 ? "0" : "") + std::to_string(index);
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

void* operator new[](std::size_t size) {
  return ::operator new(size);
}
void operator delete(void* pointer) noexcept {
  std::free(pointer);
}
void operator delete[](void* pointer) noexcept {
  std::free(pointer);
}
void operator delete(void* pointer, std::size_t) noexcept {
  std::free(pointer);
}
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
  for (std::size_t controller = 0; controller < scenario.controllers; ++controller) {
    mujoco_sim::core::RegisterAdapterRequest request;
    request.set_adapter_id("controller" + std::to_string(controller));
    request.set_role(mujoco_sim::core::CONTROLLER);
    request.set_session_timeout_ms(1000);
    for (std::size_t observation = 0; observation < scenario.observations; ++observation) {
      request.add_observation_names(ObservationName(observation));
    }
    for (std::size_t command = 0; command < scenario.commands; ++command) {
      request.add_command_names("motor" +
                                std::to_string(scenario.controllers == 1 ? command : controller));
    }
    auto registration = core.RegisterAdapter(request).response;
    if (registration.code() != mujoco_sim::core::OK) return 4;
    mujoco_sim::core::ActivateAdapterRequest activate;
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    if (core.ActivateAdapter(activate).response.code() != mujoco_sim::core::OK) {
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
    for (std::size_t observation = 0; observation < scenario.observations; ++observation) {
      request.add_observation_names(ObservationName(observation));
    }
    auto registration = core.RegisterAdapter(request).response;
    if (registration.code() != mujoco_sim::core::OK) return 10;
    mujoco_sim::core::ActivateAdapterRequest activate;
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    if (core.ActivateAdapter(activate).response.code() != mujoco_sim::core::OK) {
      return 11;
    }
  }

  std::vector<double> step_cycle_us;
  step_cycle_us.reserve(kMeasuredSteps);
  std::uint64_t state_frames = 0;
  std::uint64_t state_bytes = 0;
  std::uint64_t command_frames = 0;
  std::uint64_t command_bytes = 0;
  std::uint64_t observation_frames = 0;
  std::uint64_t observation_bytes = 0;
  rusage before{};
  rusage after{};
  auto wall_start = std::chrono::steady_clock::now();

  const auto total_steps = kWarmupSteps + kMeasuredSteps;
  for (std::uint64_t step_index = 0; step_index < total_steps; ++step_index) {
    if (step_index == kWarmupSteps) {
      allocation_count.store(0, std::memory_order_relaxed);
      getrusage(RUSAGE_SELF, &before);
      wall_start = std::chrono::steady_clock::now();
      track_allocations.store(true, std::memory_order_relaxed);
    }
    const auto step_cycle_start = std::chrono::steady_clock::now();
    for (std::size_t controller = 0; controller < scenario.controllers; ++controller) {
      mujoco_sim::core::ControllerCommand command;
      command.session = {registrations[controller].session_id(), registrations[controller].epoch()};
      command.sequence = step_index + 1;
      command.layout = registrations[controller].layout().fingerprint;
      command.commands.assign(scenario.commands, 0.0);
      const auto result = core.SubmitCommand(command, std::chrono::steady_clock::now());
      if (result.response.code != mujoco_sim::core::OK) return 7;
      if (step_index >= kWarmupSteps) {
        ++command_frames;
        command_bytes += 64U + command.commands.size() * 8U;
      }
    }
    const auto stepped = core.Step();
    if (!stepped.step_completed) return 8;
    if (step_index >= kWarmupSteps) {
      for (const auto& frame : stepped.step_completed->controller_states) {
        ++state_frames;
        state_bytes += 80U + frame.observations.size() * 8U;
      }
      for (const auto& frame : stepped.step_completed->observer_frames) {
        ++observation_frames;
        observation_bytes += 80U + frame.observations.size() * 8U;
      }
    }
    if (step_index >= kWarmupSteps) {
      step_cycle_us.push_back(std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - step_cycle_start)
                                  .count());
    }
  }
  track_allocations.store(false, std::memory_order_relaxed);

  const auto wall_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  getrusage(RUSAGE_SELF, &after);
  std::sort(step_cycle_us.begin(), step_cycle_us.end());
  const double simulation_seconds = core.state().simulation_time_s - kWarmupSteps * 0.001;
  const auto total_frames = state_frames + command_frames + observation_frames;
  const auto total_bytes = state_bytes + command_bytes + observation_bytes;
  std::cout << "{\"scenario\":\"" << scenario.name << "\",\"warmup_steps\":" << kWarmupSteps
            << ",\"steps\":" << kMeasuredSteps << ",\"controllers\":" << scenario.controllers
            << ",\"observation_scalars\":" << scenario.observations
            << ",\"command_scalars\":" << scenario.commands << ",\"state_frames\":" << state_frames
            << ",\"state_bytes\":" << state_bytes << ",\"command_frames\":" << command_frames
            << ",\"command_bytes\":" << command_bytes
            << ",\"observation_frames\":" << observation_frames
            << ",\"observation_bytes\":" << observation_bytes
            << ",\"total_frames\":" << total_frames << ",\"total_bytes\":" << total_bytes
            << ",\"step_cycle_p50_us\":" << Percentile(step_cycle_us, 0.50)
            << ",\"step_cycle_p95_us\":" << Percentile(step_cycle_us, 0.95)
            << ",\"step_cycle_p99_us\":" << Percentile(step_cycle_us, 0.99)
            << ",\"wall_seconds\":" << wall_seconds
            << ",\"cpu_seconds\":" << CpuSeconds(after) - CpuSeconds(before)
            << ",\"simulation_seconds\":" << simulation_seconds
            << ",\"rtf\":" << simulation_seconds / wall_seconds << ",\"allocations_per_step\":"
            << static_cast<double>(allocation_count.load()) / kMeasuredSteps
            << ",\"queue_high_watermark\":0"
            << ",\"max_rss_kib\":" << after.ru_maxrss << "}\n";
  return 0;
}
