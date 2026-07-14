#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <string>
#include <sys/resource.h>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace {

constexpr std::size_t kControllerCount = 8;
constexpr std::size_t kObservationCount = 30;
constexpr std::uint64_t kSteps = 10000;
constexpr std::size_t kPublishesPerTick = 3;

double CpuSeconds(const rusage& usage) {
  return static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
         static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) /
             1.0e6;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  mujoco_sim::core::SimulationCore core;
  const auto loaded = core.LoadModel(argv[1]);
  if (!loaded.response.loaded) {
    std::cerr << loaded.response.message << '\n';
    return 3;
  }

  std::vector<mujoco_sim::v2::RegisterAdapterResponse> registrations;
  for (std::size_t controller = 0; controller < kControllerCount;
       ++controller) {
    mujoco_sim::v2::RegisterAdapterRequest request;
    request.set_server_name("default");
    request.set_adapter_id("controller" + std::to_string(controller));
    request.set_role(mujoco_sim::v2::CONTROLLER);
    request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
    request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
    request.set_timeout_ms(1000);
    for (std::size_t observation = 0; observation < kObservationCount;
         ++observation) {
      const auto suffix = observation < 10 ? "0" + std::to_string(observation)
                                           : std::to_string(observation);
      request.add_observation_names("obs" + suffix);
    }
    request.add_command_names("motor" + std::to_string(controller));
    auto registration = core.RegisterAdapter(request).response;
    if (registration.code() != mujoco_sim::v2::OK) return 4;
    mujoco_sim::v2::ActivateAdapterRequest activate;
    activate.set_adapter_id(registration.adapter_id());
    activate.set_role(mujoco_sim::v2::CONTROLLER);
    activate.set_session_id(registration.session_id());
    activate.set_epoch(registration.epoch());
    if (core.ActivateAdapter(activate).response.code() != mujoco_sim::v2::OK) return 5;
    registrations.push_back(std::move(registration));
  }

  std::vector<double> barrier_us;
  barrier_us.reserve(kSteps);
  std::uint64_t serialization_count = 0;
  std::uint64_t published_bytes = 0;
  std::uint64_t max_tick_bytes = 0;
  std::uint64_t layout_bytes_avoided = 0;
  rusage before{};
  rusage after{};
  getrusage(RUSAGE_SELF, &before);
  const auto wall_start = std::chrono::steady_clock::now();

  for (std::uint64_t tick_id = 0; tick_id < kSteps; ++tick_id) {
    const auto barrier_start = std::chrono::steady_clock::now();
    const auto ticks = core.OpenIssuedTicks().response;
    if (ticks.size() != kControllerCount) return 6;
    for (const auto& tick : ticks) {
      serialization_count += tick->serialization_count;
      layout_bytes_avoided += tick->layout_bytes_avoided;
      max_tick_bytes = std::max<std::uint64_t>(max_tick_bytes,
                                               tick->serialized_bytes.size());
      for (std::size_t publish = 0; publish < kPublishesPerTick; ++publish) {
        published_bytes += tick->serialized_bytes.size();
      }
    }
    for (std::size_t controller = 0; controller < kControllerCount;
         ++controller) {
      mujoco_sim::v2::AdapterCommit commit;
      commit.mutable_header()->set_server_name("default");
      commit.mutable_header()->set_session_id(
          registrations[controller].session_id());
      commit.mutable_header()->set_epoch(registrations[controller].epoch());
      commit.mutable_header()->set_tick_id(tick_id);
      commit.mutable_header()->set_protocol_major(
          mujoco_sim::protocol::kProtocolMajor);
      commit.mutable_header()->set_protocol_minor(
          mujoco_sim::protocol::kProtocolMinor);
      commit.set_adapter_id(registrations[controller].adapter_id());
      commit.set_status(mujoco_sim::v2::COMMIT_OK);
      auto* command = commit.add_commands();
      command->set_name("motor" + std::to_string(controller));
      command->add_values(0.0);
      const auto result = core.SubmitCommit(commit);
      if (result.response.code != mujoco_sim::v2::OK) return 7;
      if ((controller + 1 == kControllerCount) !=
          result.effects.step_completed.has_value()) {
        return 8;
      }
    }
    barrier_us.push_back(std::chrono::duration<double, std::micro>(
                             std::chrono::steady_clock::now() - barrier_start)
                             .count());
  }

  const auto wall_seconds = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - wall_start)
                                .count();
  getrusage(RUSAGE_SELF, &after);
  std::sort(barrier_us.begin(), barrier_us.end());
  const auto p99 = barrier_us[static_cast<std::size_t>(
      0.99 * static_cast<double>(barrier_us.size() - 1))];
  const auto message_count =
      kSteps * kControllerCount * kPublishesPerTick;
  std::cout << "{\"controllers\":" << kControllerCount
            << ",\"observations_per_controller\":" << kObservationCount
            << ",\"steps\":" << kSteps
            << ",\"simulation_seconds\":" << core.state().simulation_time_s
            << ",\"retransmission_ms\":5"
            << ",\"serialization_count\":" << serialization_count
            << ",\"published_messages\":" << message_count
            << ",\"average_tick_bytes\":"
            << static_cast<double>(published_bytes) / message_count
            << ",\"max_tick_bytes\":" << max_tick_bytes
            << ",\"layout_bytes_avoided\":" << layout_bytes_avoided
            << ",\"barrier_p99_us\":" << p99
            << ",\"wall_seconds\":" << wall_seconds
            << ",\"cpu_seconds\":" << CpuSeconds(after) - CpuSeconds(before)
            << ",\"max_rss_kib\":" << after.ru_maxrss << "}\n";
  return serialization_count == kSteps * kControllerCount ? 0 : 9;
}
