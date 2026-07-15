#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/sensor_demo/interface_data.hpp>
#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>

#include "app_config.hpp"
#include "process_signal.hpp"

namespace demo = mujoco_sim::sensor_demo;

int main(int argc, char** argv) {
  demo::AppConfig app_config;
  std::string message;
  const auto parse_result = demo::ParseAppConfig(argc, argv, &app_config,
                                                  &message);
  if (parse_result == demo::ParseAppConfigResult::kHelp) {
    std::cout << message;
    return 0;
  }
  if (parse_result != demo::ParseAppConfigResult::kSuccess) {
    std::cerr << message << '\n';
    return 1;
  }
  demo::InstallProcessSignalHandlers();
  const auto& config = app_config.controller;

  mujoco_sim::adapter_sdk::SyncAdapterClient client(config.adapter);
  if (!client.Start(&message)) {
    std::cerr << "controller start failed: " << message << '\n';
    return 1;
  }
  if (!client.Activate(&message)) {
    std::cerr << "controller activation failed: " << message << '\n';
    std::string ignored;
    client.Unregister(&ignored);
    return 1;
  }
  demo::WaypointController controller(config.control);
  const demo::VehicleDecoder decoder(client.layout(), config.interfaces);
  const demo::WheelCommandMapper command_mapper(client.layout(),
                                                 config.interfaces);
  std::cout << "sensor-demo controller ready adapter="
            << config.adapter.adapter_id << std::endl;

  std::uint64_t handled_ticks = 0;
  std::size_t completed_laps = 0;
  bool test_limit_reached = false;
  std::optional<double> first_simulation_time;
  int result = 0;
  while (!demo::ProcessStopRequested()) {
    mujoco_sim::adapter_sdk::TickLease tick;
    const auto wait_result = client.WaitForTick(
        std::chrono::milliseconds(config.wait_timeout_ms), &tick, &message);
    if (wait_result == mujoco_sim::adapter_sdk::TickWaitResult::kTimeout) {
      continue;
    }
    if (wait_result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
      std::cerr << (message.empty() ? "controller Tick stream stopped" : message)
                << '\n';
      result = 1;
      break;
    }

    try {
      demo::VehicleSample sample;
      if (!decoder.Decode(tick, &sample, &message)) {
        throw std::runtime_error(message);
      }
      if (!first_simulation_time.has_value()) {
        first_simulation_time = sample.simulation_time;
      }
      const demo::WaypointControllerOutput output = controller.Update(sample);
      std::vector<double> commands;
      if (!command_mapper.Map(output.wheel_angular_velocities, &commands,
                              &message)) {
        throw std::runtime_error(message);
      }
      if (!client.Commit(tick, commands, &message)) {
        throw std::runtime_error("Commit failed: " + message);
      }
      ++handled_ticks;
      completed_laps = output.completed_laps;
      if (config.log_every_ticks > 0 &&
          handled_ticks % config.log_every_ticks == 0) {
        std::cout << "sensor-demo controller tick=" << handled_ticks
                  << " sim_time=" << sample.simulation_time
                  << " waypoint=" << output.target_waypoint
                  << " laps=" << output.completed_laps << '\n';
      }
      if (output.lap_complete && config.control.stop_after_lap) break;
      if (config.max_simulation_time_s > 0.0 &&
          sample.simulation_time - *first_simulation_time >=
              config.max_simulation_time_s) {
        test_limit_reached = true;
        break;
      }
    } catch (const std::exception& error) {
      std::cerr << "controller processing failed: " << error.what() << '\n';
      result = 1;
      break;
    }
  }

  std::string unregister_error;
  if (!client.Unregister(&unregister_error)) {
    std::cerr << "controller unregister failed: " << unregister_error << '\n';
    if (result == 0 && !demo::ProcessStopRequested()) result = 1;
  }
  if (!client.lifecycle_warning().empty()) {
    std::cerr << "controller lifecycle warning: " << client.lifecycle_warning()
              << '\n';
  }
  std::cout << "sensor-demo controller summary ticks=" << handled_ticks
            << " laps=" << completed_laps
            << " test_limit=" << (test_limit_reached ? 1 : 0) << '\n';
  return result;
}
