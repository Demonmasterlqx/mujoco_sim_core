#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ecal/pubsub/publisher.h>
#include <ecal/service/client.h>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace {

using namespace std::chrono_literals;

float ReadFloat32LittleEndian(const std::byte* source) {
  const std::uint32_t bits =
      std::to_integer<std::uint32_t>(source[0]) |
      (std::to_integer<std::uint32_t>(source[1]) << 8U) |
      (std::to_integer<std::uint32_t>(source[2]) << 16U) |
      (std::to_integer<std::uint32_t>(source[3]) << 24U);
  return std::bit_cast<float>(bits);
}

bool ValidateObservation(
    const mujoco_sim::adapter_sdk::ObservationLease& frame,
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                         std::uint64_t previous_sequence,
                         double previous_time) {
  if (frame.sequence() <= previous_sequence ||
      frame.simulation_time_s() <= previous_time) {
    std::cerr << "observer sequence or simulation time is not monotonic\n";
    return false;
  }
  const auto quaternion = layout.Observation("imu_quat");
  const auto gyro = layout.Observation("imu_gyro");
  if (!quaternion || quaternion->count != 4 || !gyro || gyro->count != 3) {
    std::cerr << "observer layout is missing configured IMU spans\n";
    return false;
  }
  for (double value : frame.observations()) {
    if (!std::isfinite(value)) {
      std::cerr << "observer delivered a non-finite value\n";
      return false;
    }
  }
  return true;
}

bool ValidateRgb(const mujoco_sim::adapter_sdk::OwnedStreamSample& sample) {
  if (sample.header.format() != mujoco_sim::v3::RGB8 ||
      sample.header.width() != 64 || sample.header.height() != 48 ||
      sample.payload.size() != 64U * 48U * 3U) {
    std::cerr << "RGB sample shape or format mismatch\n";
    return false;
  }
  const std::byte first = sample.payload.front();
  if (std::all_of(sample.payload.begin(), sample.payload.end(),
                  [first](std::byte value) { return value == first; })) {
    std::cerr << "RGB sample is blank\n";
    return false;
  }
  return true;
}

bool ValidateLidar(const mujoco_sim::adapter_sdk::OwnedStreamSample& sample) {
  if (sample.header.format() != mujoco_sim::v3::XYZ32F ||
      sample.header.width() != 72 || sample.header.height() != 4 ||
      sample.payload.size() != 72U * 4U * 3U * sizeof(float)) {
    std::cerr << "LiDAR sample shape or format mismatch\n";
    return false;
  }
  constexpr double kMinimumRangeMeters = 0.05;
  constexpr double kMaximumRangeMeters = 8.0;
  constexpr double kRangeToleranceMeters = 1e-4;
  std::size_t valid_points = 0;
  for (std::size_t offset = 0; offset < sample.payload.size();
       offset += 3 * sizeof(float)) {
    const float x = ReadFloat32LittleEndian(sample.payload.data() + offset);
    const float y = ReadFloat32LittleEndian(sample.payload.data() + offset +
                                            sizeof(float));
    const float z = ReadFloat32LittleEndian(sample.payload.data() + offset +
                                            2U * sizeof(float));
    const bool finite_x = std::isfinite(x);
    const bool finite_y = std::isfinite(y);
    const bool finite_z = std::isfinite(z);
    if (!finite_x && !finite_y && !finite_z) {
      continue;
    }
    if (!finite_x || !finite_y || !finite_z) {
      std::cerr << "LiDAR sample contains a partially invalid XYZ return\n";
      return false;
    }
    const double distance = std::sqrt(
        static_cast<double>(x) * x + static_cast<double>(y) * y +
        static_cast<double>(z) * z);
    if (distance < kMinimumRangeMeters - kRangeToleranceMeters ||
        distance > kMaximumRangeMeters + kRangeToleranceMeters) {
      std::cerr << "LiDAR return distance is outside the configured range: "
                << distance << '\n';
      return false;
    }
    ++valid_points;
  }
  const std::size_t point_count =
      sample.payload.size() / (3U * sizeof(float));
  if (valid_points < point_count / 4U) {
    std::cerr << "LiDAR sample has too few valid returns: " << valid_points
              << " of " << point_count << '\n';
    return false;
  }
  return true;
}

bool WaitForStreamSamples(mujoco_sim::adapter_sdk::SensorStreamClient* client,
                          const std::string& name, bool rgb) {
  std::string error;
  std::uint64_t previous_sequence = 0;
  double previous_time = -1.0;
  for (int index = 0; index < 2; ++index) {
    mujoco_sim::adapter_sdk::OwnedStreamSample sample;
    const auto result = client->WaitForSample(name, 5s, &sample, &error);
    if (result != mujoco_sim::adapter_sdk::SampleWaitResult::kReceived) {
      std::cerr << "waiting for " << name << " failed: " << error << '\n';
      return false;
    }
    if (sample.header.stream_sequence() <= previous_sequence ||
        sample.header.simulation_time_s() <= previous_time) {
      std::cerr << name << " sequence or simulation time is not monotonic\n";
      return false;
    }
    if (!(rgb ? ValidateRgb(sample) : ValidateLidar(sample))) return false;
    previous_sequence = sample.header.stream_sequence();
    previous_time = sample.header.simulation_time_s();
  }
  return true;
}

struct PipelineCursor {
  std::uint64_t observation_sequence = 0;
  double observation_time = -1.0;
  std::uint64_t lidar_sequence = 0;
  double lidar_time = -1.0;
};

bool WaitForPipelineTick(
    mujoco_sim::adapter_sdk::ObserverClient* observer,
    mujoco_sim::adapter_sdk::SensorStreamClient* streams,
    std::uint64_t expected_tick_id, bool exact_tick,
    const std::string& stage, PipelineCursor* cursor) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  std::string error;
  bool observed = false;
  while (!observed && std::chrono::steady_clock::now() < deadline) {
    mujoco_sim::adapter_sdk::ObservationLease frame;
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    const auto result = observer->WaitForObservation(
        std::max(1ms, remaining), &frame, &error);
    if (result !=
        mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived) {
      std::cerr << stage << " observation wait failed: " << error << '\n';
      return false;
    }
    if (!ValidateObservation(frame, observer->layout(),
                             cursor->observation_sequence,
                             cursor->observation_time)) {
      return false;
    }
    cursor->observation_sequence = frame.sequence();
    cursor->observation_time = frame.simulation_time_s();
    if (exact_tick && frame.tick_id() > expected_tick_id) {
      std::cerr << stage << " skipped expected observation tick "
                << expected_tick_id << " and reached "
                << frame.tick_id() << '\n';
      return false;
    }
    observed = exact_tick ? frame.tick_id() == expected_tick_id
                          : frame.tick_id() >= expected_tick_id;
  }
  if (!observed) {
    std::cerr << stage << " did not produce a new observation frame\n";
    return false;
  }

  while (std::chrono::steady_clock::now() < deadline) {
    mujoco_sim::adapter_sdk::OwnedStreamSample sample;
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    const auto result = streams->WaitForSample(
        "lidar", std::max(1ms, remaining), &sample, &error);
    if (result != mujoco_sim::adapter_sdk::SampleWaitResult::kReceived) {
      std::cerr << stage << " LiDAR wait failed: " << error << '\n';
      return false;
    }
    if (sample.header.stream_sequence() <= cursor->lidar_sequence ||
        sample.header.simulation_time_s() <= cursor->lidar_time ||
        !ValidateLidar(sample)) {
      std::cerr << stage << " LiDAR sequence or time is not monotonic\n";
      return false;
    }
    cursor->lidar_sequence = sample.header.stream_sequence();
    cursor->lidar_time = sample.header.simulation_time_s();
    const auto tick_id = sample.header.tick_id();
    if (exact_tick && tick_id > expected_tick_id) {
      std::cerr << stage << " skipped expected LiDAR tick "
                << expected_tick_id << " and reached " << tick_id << '\n';
      return false;
    }
    if (exact_tick ? tick_id == expected_tick_id
                   : tick_id >= expected_tick_id) {
      return true;
    }
  }
  std::cerr << stage << " did not produce a new LiDAR frame\n";
  return false;
}

bool WaitForPipelineAfterTick(
    mujoco_sim::adapter_sdk::ObserverClient* observer,
    mujoco_sim::adapter_sdk::SensorStreamClient* streams,
    std::uint64_t minimum_tick_id, const std::string& stage,
    PipelineCursor* cursor) {
  return WaitForPipelineTick(observer, streams, minimum_tick_id, false, stage,
                             cursor);
}

bool WaitForPipelineAtTick(
    mujoco_sim::adapter_sdk::ObserverClient* observer,
    mujoco_sim::adapter_sdk::SensorStreamClient* streams,
    std::uint64_t tick_id, const std::string& stage, PipelineCursor* cursor) {
  return WaitForPipelineTick(observer, streams, tick_id, true, stage, cursor);
}

mujoco_sim::adapter_sdk::AdapterOptions ControllerOptions(
    const std::string& server_name, std::string adapter_id,
    std::uint32_t timeout_ms = 2000,
    std::string observation_name = "hinge_pos",
    std::string command_name = "hinge_motor") {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = server_name;
  options.adapter_id = std::move(adapter_id);
  options.observation_names = {std::move(observation_name)};
  options.command_names = {std::move(command_name)};
  options.commit_timeout_ms = timeout_ms;
  options.session_timeout_ms = std::max<std::uint32_t>(5000, timeout_ms * 2);
  options.service_timeout_ms = 5000;
  return options;
}

bool StartController(mujoco_sim::adapter_sdk::SyncAdapterClient* controller,
                     std::string* error) {
  return controller->Start(error) && controller->Activate(error);
}

bool WaitForControllerTick(
    mujoco_sim::adapter_sdk::SyncAdapterClient* controller,
    mujoco_sim::adapter_sdk::TickLease* tick,
    std::string* error) {
  const auto result = controller->WaitForTick(5s, tick, error);
  if (result == mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
    return true;
  }
  *error = "unexpected controller Tick result=" +
           std::to_string(static_cast<int>(result)) + " error=" + *error;
  return false;
}

bool CommitCommand(mujoco_sim::adapter_sdk::SyncAdapterClient* controller,
                   const mujoco_sim::adapter_sdk::TickLease& tick,
                   const std::string&, double value,
                   std::string* error) {
  const std::array commands{value};
  return controller->Commit(tick, commands, error);
}

bool JoinTargetOnNextTick(
    mujoco_sim::adapter_sdk::SyncAdapterClient* gate,
    mujoco_sim::adapter_sdk::SyncAdapterClient* target,
    mujoco_sim::adapter_sdk::ObserverClient* observer,
    mujoco_sim::adapter_sdk::SensorStreamClient* streams,
    mujoco_sim::adapter_sdk::TickLease* pending_gate_tick,
    mujoco_sim::adapter_sdk::TickLease* target_tick, PipelineCursor* cursor,
    const std::string& stage, std::string* error) {
  const std::uint64_t transition_tick =
      pending_gate_tick->tick_id() + 1;
  if (!CommitCommand(gate, *pending_gate_tick, "gate_motor", 0.0, error) ||
      !WaitForPipelineAtTick(observer, streams, transition_tick,
                             stage + " join transition", cursor) ||
      !WaitForControllerTick(gate, pending_gate_tick, error) ||
      !WaitForControllerTick(target, target_tick, error)) {
    return false;
  }
  if (pending_gate_tick->tick_id() != target_tick->tick_id()) {
    *error = stage + " controllers received different Tick IDs";
    return false;
  }
  return true;
}

bool UnregisterAdapter(
    mujoco_sim::adapter_sdk::SyncAdapterClient* controller,
    const std::string& stage) {
  std::string error;
  if (controller->Unregister(&error)) return true;
  std::cerr << stage << " unregister failed: " << error << '\n';
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: sensor_observer_e2e_probe SERVER_NAME\n";
    return 1;
  }
  const std::string server_name = argv[1];

  mujoco_sim::adapter_sdk::SensorStreamOptions runtime_owner_options;
  runtime_owner_options.server_name = server_name;
  runtime_owner_options.stream_names = {"rgb"};
  runtime_owner_options.service_timeout_ms = 5000;
  runtime_owner_options.queue_capacity = 1;
  auto runtime_owner =
      std::make_unique<mujoco_sim::adapter_sdk::SensorStreamClient>(
          std::move(runtime_owner_options));
  std::string error;
  if (!runtime_owner->Start(&error)) {
    std::cerr << "runtime owner stream client startup failed: " << error
              << '\n';
    return 1;
  }

  mujoco_sim::adapter_sdk::ObserverOptions observer_options;
  observer_options.server_name = server_name;
  observer_options.observer_id = "sdk_sensor_observer";
  observer_options.observation_names = {"imu_quat", "imu_gyro"};
  observer_options.session_timeout_ms = 2000;
  observer_options.service_timeout_ms = 5000;
  observer_options.heartbeat_period_ms = 100;
  observer_options.every_n_steps = 1;
  observer_options.period_s.reset();
  observer_options.queue_capacity = 16;
  mujoco_sim::adapter_sdk::ObserverClient observer(
      std::move(observer_options));

  mujoco_sim::adapter_sdk::SensorStreamOptions stream_options;
  stream_options.server_name = server_name;
  stream_options.stream_names = {"rgb", "lidar"};
  stream_options.service_timeout_ms = 5000;
  stream_options.queue_capacity = 8;
  mujoco_sim::adapter_sdk::SensorStreamClient streams(
      std::move(stream_options));

  if (!observer.Start(&error) || !observer.Activate(&error)) {
    std::cerr << "observer startup failed: " << error << '\n';
    return 1;
  }
  if (!streams.Start(&error)) {
    std::cerr << "stream client startup failed: " << error << '\n';
    observer.Unregister(&error);
    return 1;
  }

  runtime_owner.reset();
  PipelineCursor cursor;
  bool valid = WaitForPipelineAfterTick(&observer, &streams, 1,
                                        "shared runtime owner teardown",
                                        &cursor);
  valid = valid && WaitForStreamSamples(&streams, "rgb", true);

  mujoco_sim::adapter_sdk::SyncAdapterClient gate_controller(
      ControllerOptions(server_name, "sensor_gate_controller", 10000,
                        "gate_pos", "gate_motor"));
  mujoco_sim::adapter_sdk::TickLease gate_tick;
  if (valid &&
      (!StartController(&gate_controller, &error) ||
       !WaitForControllerTick(&gate_controller, &gate_tick, &error))) {
    std::cerr << "gate controller startup failed: " << error << '\n';
    valid = false;
  }

  mujoco_sim::adapter_sdk::SyncAdapterClient commit_controller(
      ControllerOptions(server_name, "sensor_commit_controller", 10000));
  mujoco_sim::adapter_sdk::TickLease commit_tick;
  if (valid &&
      (!StartController(&commit_controller, &error) ||
       !JoinTargetOnNextTick(&gate_controller, &commit_controller, &observer,
                             &streams, &gate_tick, &commit_tick, &cursor,
                             "commit", &error))) {
    std::cerr << "commit controller join failed: " << error << '\n';
    valid = false;
  }
  if (valid &&
      (!CommitCommand(&gate_controller, gate_tick, "gate_motor", 0.0,
                      &error) ||
       !CommitCommand(&commit_controller, commit_tick, "hinge_motor", 0.1,
                      &error) ||
       !WaitForPipelineAtTick(&observer, &streams,
                              commit_tick.tick_id() + 1,
                              "commit completion", &cursor))) {
    std::cerr << "commit completion failed: " << error << '\n';
    valid = false;
  }
  if (valid && !UnregisterAdapter(&commit_controller,
                                     "commit controller")) {
    valid = false;
  }
  if (valid && !WaitForControllerTick(&gate_controller, &gate_tick, &error)) {
    std::cerr << "gate Tick after commit failed: " << error << '\n';
    valid = false;
  }

  mujoco_sim::adapter_sdk::SyncAdapterClient unregister_controller(
      ControllerOptions(server_name, "sensor_unregister_controller", 10000));
  mujoco_sim::adapter_sdk::TickLease unregister_tick;
  if (valid &&
      (!StartController(&unregister_controller, &error) ||
       !JoinTargetOnNextTick(
           &gate_controller, &unregister_controller, &observer, &streams,
           &gate_tick, &unregister_tick, &cursor, "unregister",
           &error))) {
    std::cerr << "unregister controller join failed: " << error << '\n';
    valid = false;
  }
  if (valid &&
      (!CommitCommand(&gate_controller, gate_tick, "gate_motor", 0.0,
                      &error) ||
       !UnregisterAdapter(&unregister_controller, "active controller") ||
       !WaitForPipelineAtTick(&observer, &streams,
                              unregister_tick.tick_id() + 1,
                              "unregister completion", &cursor))) {
    std::cerr << "unregister completion failed: " << error << '\n';
    valid = false;
  }
  if (valid && !WaitForControllerTick(&gate_controller, &gate_tick, &error)) {
    std::cerr << "gate Tick after unregister failed: " << error << '\n';
    valid = false;
  }

  mujoco_sim::adapter_sdk::SyncAdapterClient timeout_controller(
      ControllerOptions(server_name, "sensor_timeout_controller", 250));
  mujoco_sim::adapter_sdk::TickLease timeout_tick;
  if (valid &&
      (!StartController(&timeout_controller, &error) ||
       !JoinTargetOnNextTick(&gate_controller, &timeout_controller, &observer,
                             &streams, &gate_tick, &timeout_tick, &cursor,
                             "timeout", &error))) {
    std::cerr << "timeout controller join failed: " << error << '\n';
    valid = false;
  }
  if (valid &&
      (!CommitCommand(&gate_controller, gate_tick, "gate_motor", 0.0,
                      &error) ||
       !WaitForPipelineAtTick(&observer, &streams,
                              timeout_tick.tick_id() + 1,
                              "timeout completion", &cursor))) {
    std::cerr << "timeout completion failed: " << error << '\n';
    valid = false;
  }
  if (valid &&
      (!UnregisterAdapter(&timeout_controller, "timeout controller") ||
       !WaitForControllerTick(&gate_controller, &gate_tick, &error))) {
    std::cerr << "timeout cleanup failed: " << error << '\n';
    valid = false;
  }

  mujoco_sim::adapter_sdk::SyncAdapterClient faulty_controller(
      ControllerOptions(server_name, "sensor_faulty_controller", 10000));
  mujoco_sim::adapter_sdk::TickLease fault_tick;
  if (valid &&
      (!StartController(&faulty_controller, &error) ||
       !JoinTargetOnNextTick(&gate_controller, &faulty_controller, &observer,
                             &streams, &gate_tick, &fault_tick, &cursor,
                             "local error", &error))) {
    std::cerr << "faulty controller join failed: " << error << '\n';
    valid = false;
  }
  if (valid && !CommitCommand(&gate_controller, gate_tick, "gate_motor", 0.0,
                              &error)) {
    std::cerr << "gate Commit before local error failed: " << error << '\n';
    valid = false;
  }
  if (valid) {
    if (!faulty_controller.FailTick(
            fault_tick, mujoco_sim::adapter_sdk::FaultCode::kInvalidOutput,
            "invalid command output", &error)) {
      std::cerr << "controller fault publish failed: " << error << '\n';
      valid = false;
    }
  }
  if (valid && !WaitForPipelineAtTick(
                   &observer, &streams, fault_tick.tick_id() + 1,
                   "local error completion", &cursor)) {
    valid = false;
  }
  if (valid) {
    const auto error_deadline = std::chrono::steady_clock::now() + 2s;
    while (faulty_controller.lifecycle_warning().empty() &&
           std::chrono::steady_clock::now() < error_deadline) {
      std::this_thread::sleep_for(20ms);
    }
    if (faulty_controller.lifecycle_warning().empty()) {
      std::cerr << "server did not report the local command error\n";
      valid = false;
    }
  }
  if (valid && !UnregisterAdapter(&faulty_controller,
                                     "faulty controller")) {
    valid = false;
  }
  if (!UnregisterAdapter(&gate_controller, "gate controller")) {
    valid = false;
  }

  streams.Stop();
  std::string unregister_error;
  if (!observer.Unregister(&unregister_error)) {
    std::cerr << "observer unregister failed: " << unregister_error << '\n';
    valid = false;
  }
  if (!observer.lifecycle_warning().empty()) {
    std::cerr << "observer lifecycle warning: "
              << observer.lifecycle_warning() << '\n';
    valid = false;
  }
  return valid ? 0 : 1;
}
