#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>
#include <utility>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>

namespace {

using namespace std::chrono_literals;

volatile std::sig_atomic_t stop_requested = 0;

void HandleSignal(int) { stop_requested = 1; }

const mujoco_sim::v2::ObservationBlock* FindBlock(
    const mujoco_sim::v2::ObservationFrame& frame,
    const std::string& name) {
  for (const auto& block : frame.observations()) {
    if (block.name() == name) return &block;
  }
  return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: e2e_truth_recorder SERVER_NAME OUTPUT_CSV\n";
    return 2;
  }

  mujoco_sim::adapter_sdk::ObserverOptions options;
  options.server_name = argv[1];
  options.observer_id = "e2e_truth_recorder";
  options.observation_names = {"base_position", "imu_quat"};
  options.timeout_ms = 5000;
  options.service_timeout_ms = 20000;
  options.keepalive_period_ms = 500;
  options.every_n_steps = 1;
  options.queue_capacity = 4096;
  mujoco_sim::adapter_sdk::ObserverClient observer(std::move(options));
  std::string error;
  if (!observer.Start(&error) || !observer.Activate(&error)) {
    std::cerr << "truth observer start failed: " << error << '\n';
    return 1;
  }
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  std::cout << "truth recorder ready" << std::endl;

  const std::filesystem::path output_path = argv[2];
  std::ofstream output(output_path, std::ios::trunc);
  if (!output) {
    std::cerr << "failed to create truth CSV: " << output_path << '\n';
    return 1;
  }
  output.imbue(std::locale::classic());
  output << std::setprecision(17)
         << "timestamp,x,y,z,qx,qy,qz,qw\n";

  std::uint64_t samples = 0;
  while (stop_requested == 0) {
    mujoco_sim::v2::ObservationFrame frame;
    const auto result = observer.WaitForObservation(500ms, &frame, &error);
    if (result ==
        mujoco_sim::adapter_sdk::ObservationWaitResult::kTimeout) {
      continue;
    }
    if (result ==
        mujoco_sim::adapter_sdk::ObservationWaitResult::kStopped) {
      break;
    }
    if (result !=
        mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived) {
      std::cerr << "truth observer stopped with error: " << error << '\n';
      return 1;
    }
    const auto* position = FindBlock(frame, "base_position");
    const auto* quaternion = FindBlock(frame, "imu_quat");
    if (position == nullptr || quaternion == nullptr ||
        position->values_size() != 3 || quaternion->values_size() != 4 ||
        !frame.header().has_simulation_time_s()) {
      std::cerr << "truth observation layout is invalid\n";
      return 1;
    }
    output << frame.header().simulation_time_s() << ','
           << position->values(0) << ',' << position->values(1) << ','
           << position->values(2) << ',' << quaternion->values(1) << ','
           << quaternion->values(2) << ',' << quaternion->values(3) << ','
           << quaternion->values(0) << '\n';
    ++samples;
  }
  std::string unregister_error;
  if (!observer.Unregister(&unregister_error)) {
    std::cerr << "truth observer unregister failed: " << unregister_error
              << '\n';
    return 1;
  }
  output.close();
  if (!output || samples < 20) {
    std::cerr << "truth recorder produced too few samples: " << samples
              << '\n';
    return 1;
  }
  std::cout << "truth recorder summary samples=" << samples << '\n';
  return 0;
}
