#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

namespace {
using namespace std::chrono_literals;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: message_rate_e2e_probe SERVER_NAME\n";
    return 2;
  }
  const std::string server_name = argv[1];
  std::string error;

  mujoco_sim::adapter_sdk::ObserverOptions observer_options;
  observer_options.server_name = server_name;
  observer_options.observer_id = "rate_observer";
  observer_options.observation_names = {"hinge_pos"};
  observer_options.session_timeout_ms = 3000;
  observer_options.service_timeout_ms = 10000;
  observer_options.heartbeat_period_ms = 250;
  observer_options.queue_capacity = 4;
  mujoco_sim::adapter_sdk::ObserverClient observer(
      std::move(observer_options));
  if (!observer.Start(&error) || !observer.Activate(&error)) {
    std::cerr << "observer startup failed: " << error << '\n';
    return 1;
  }

  std::atomic<bool> stop_observer{false};
  std::atomic<std::uint64_t> observations{0};
  std::thread observer_thread([&] {
    while (!stop_observer.load()) {
      mujoco_sim::adapter_sdk::ObservationLease frame;
      std::string wait_error;
      const auto result = observer.WaitForObservation(100ms, &frame,
                                                       &wait_error);
      if (result ==
          mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived) {
        ++observations;
      } else if (result ==
                 mujoco_sim::adapter_sdk::ObservationWaitResult::kError) {
        std::cerr << "observer wait failed: " << wait_error << '\n';
        stop_observer.store(true);
      }
    }
  });

  mujoco_sim::adapter_sdk::AdapterOptions controller_options;
  controller_options.server_name = server_name;
  controller_options.adapter_id = "rate_controller";
  controller_options.observation_names = {"hinge_pos"};
  controller_options.command_names = {"hinge_motor"};
  controller_options.commit_timeout_ms = 100;
  controller_options.session_timeout_ms = 3000;
  controller_options.heartbeat_period_ms = 250;
  controller_options.service_timeout_ms = 10000;
  mujoco_sim::adapter_sdk::SyncAdapterClient controller(
      std::move(controller_options));
  if (!controller.Start(&error) || !controller.Activate(&error)) {
    std::cerr << "controller startup failed: " << error << '\n';
    stop_observer.store(true);
    observer.Stop();
    observer_thread.join();
    observer.Unregister(&error);
    return 1;
  }

  constexpr std::uint64_t kTicks = 10000;
  std::uint64_t commits = 0;
  while (commits < kTicks) {
    mujoco_sim::adapter_sdk::TickLease tick;
    const auto result = controller.WaitForTick(6s, &tick, &error);
    if (result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
      std::cerr << "controller wait failed at " << commits << ": " << error
                << '\n';
      break;
    }
    if (commits == 0) std::this_thread::sleep_for(8ms);
    std::vector<double> commands(controller.layout().command_scalar_count(),
                                 0.0);
    if (!controller.Commit(tick, commands, &error)) {
      std::cerr << "controller Commit failed at " << commits << ": "
                << error << '\n';
      break;
    }
    ++commits;
  }

  bool clean = commits == kTicks;
  if (!controller.Unregister(&error)) {
    std::cerr << "controller unregister failed: " << error << '\n';
    clean = false;
  }
  stop_observer.store(true);
  observer.Stop();
  observer_thread.join();
  if (!observer.Unregister(&error)) {
    std::cerr << "observer unregister failed: " << error << '\n';
    clean = false;
  }
  std::cout << "rate probe commits=" << commits
            << " observations=" << observations.load() << '\n';
  return clean ? 0 : 1;
}
