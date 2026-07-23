#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/sensor_demo/interface_data.hpp>

#include "app_config.hpp"
#include "process_signal.hpp"

namespace demo = mujoco_sim::sensor_demo;

int main(int argc, char** argv) {
  demo::AppConfig app_config;
  std::string message;
  const auto parse_result = demo::ParseAppConfig(argc, argv, &app_config, &message);
  if (parse_result == demo::ParseAppConfigResult::kHelp) {
    std::cout << message;
    return 0;
  }
  if (parse_result != demo::ParseAppConfigResult::kSuccess) {
    std::cerr << message << '\n';
    return 1;
  }
  if (!demo::InstallProcessSignalHandlers(&message)) {
    std::cerr << message << '\n';
    return 1;
  }
  const auto& config = app_config.state_observer;
  mujoco_sim::adapter_sdk::ObserverClient client(config.observer);
  if (!client.Start(&message) || !client.Activate(&message)) {
    std::cerr << "state observer start failed: " << message << '\n';
    std::string unregister_error;
    if (!client.Unregister(&unregister_error)) {
      std::cerr << "state observer cleanup after start failure failed: " << unregister_error
                << '\n';
    }
    return 1;
  }
  const demo::StateDecoder decoder(client.layout(), config.interfaces);
  std::cout << "sensor-demo state observer ready adapter=" << config.observer.observer_id
            << std::endl;

  std::uint64_t frames = 0;
  int result = 0;
  while (!demo::ProcessStopRequested() && (config.max_frames == 0 || frames < config.max_frames)) {
    mujoco_sim::adapter_sdk::ObservationLease frame;
    const auto wait_result = client.WaitForObservation(
        std::chrono::milliseconds(config.wait_timeout_ms), &frame, &message);
    if (wait_result == mujoco_sim::adapter_sdk::ObservationWaitResult::kTimeout) {
      continue;
    }
    if (wait_result != mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived) {
      std::cerr << (message.empty() ? "state observation stream stopped" : message) << '\n';
      result = 1;
      break;
    }
    demo::StateSample sample;
    if (!decoder.Decode(frame, &sample, &message)) {
      std::cerr << "state observation validation failed: " << message << '\n';
      result = 1;
      break;
    }
    ++frames;
    if (config.log_every_frames > 0 && frames % config.log_every_frames == 0) {
      std::cout << "sensor-demo state frame=" << frames << " sim_time=" << sample.simulation_time
                << " wheel_velocity=" << sample.wheel_velocities[0]
                << " imu_w=" << sample.imu_quaternion[0] << '\n';
    }
  }

  std::string unregister_error;
  if (!client.Unregister(&unregister_error)) {
    std::cerr << "state observer unregister failed: " << unregister_error << '\n';
    result = 1;
  }
  std::cout << "sensor-demo state observer summary frames=" << frames
            << " dropped=" << client.dropped_frames() << '\n';
  return result;
}
