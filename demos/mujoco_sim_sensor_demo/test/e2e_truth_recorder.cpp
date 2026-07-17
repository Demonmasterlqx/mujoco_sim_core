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

void HandleSignal(int) {
  stop_requested = 1;
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
  options.session_timeout_ms = 5000;
  options.service_timeout_ms = 20000;
  options.heartbeat_period_ms = 500;
  options.every_n_steps = 1;
  options.period_s.reset();
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

  const auto position = observer.layout().Observation("base_position");
  const auto quaternion = observer.layout().Observation("imu_quat");
  if (!position || !quaternion || position->count != 3 || quaternion->count != 4) {
    std::cerr << "truth observation layout is invalid\n";
    return 1;
  }

  const std::filesystem::path output_path = argv[2];
  std::ofstream output(output_path, std::ios::trunc);
  if (!output) {
    std::cerr << "failed to create truth CSV: " << output_path << '\n';
    return 1;
  }
  output.imbue(std::locale::classic());
  output << std::setprecision(17) << "timestamp,x,y,z,qx,qy,qz,qw\n";

  std::uint64_t samples = 0;
  while (stop_requested == 0) {
    mujoco_sim::adapter_sdk::ObservationLease frame;
    const auto result = observer.WaitForObservation(500ms, &frame, &error);
    if (result == mujoco_sim::adapter_sdk::ObservationWaitResult::kTimeout) {
      continue;
    }
    if (result == mujoco_sim::adapter_sdk::ObservationWaitResult::kStopped) {
      break;
    }
    if (result != mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived) {
      std::cerr << "truth observer stopped with error: " << error << '\n';
      return 1;
    }
    const auto values = frame.observations();
    output << frame.simulation_time_s() << ',' << values[position->offset] << ','
           << values[position->offset + 1] << ',' << values[position->offset + 2] << ','
           << values[quaternion->offset + 1] << ',' << values[quaternion->offset + 2] << ','
           << values[quaternion->offset + 3] << ',' << values[quaternion->offset] << '\n';
    ++samples;
  }
  std::string unregister_error;
  if (!observer.Unregister(&unregister_error)) {
    std::cerr << "truth observer unregister failed: " << unregister_error << '\n';
    return 1;
  }
  output.close();
  if (!output || samples < 20) {
    std::cerr << "truth recorder produced too few samples: " << samples << '\n';
    return 1;
  }
  std::cout << "truth recorder summary samples=" << samples << '\n';
  return 0;
}
