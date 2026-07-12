#include <mujoco_sim/sensor_demo/interface_data.hpp>

#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>

#include <google/protobuf/repeated_ptr_field.h>

namespace mujoco_sim::sensor_demo {
namespace {

using ObservationIndex =
    std::unordered_map<std::string, const mujoco_sim::v1::ObservationBlock*>;

bool Fail(const std::string& message, std::string* error_message) {
  if (error_message != nullptr) *error_message = message;
  return false;
}

bool BuildIndex(
    const google::protobuf::RepeatedPtrField<
        mujoco_sim::v1::ObservationBlock>& observations,
    ObservationIndex* index, std::string* error_message) {
  index->clear();
  for (const auto& observation : observations) {
    if (observation.name().empty()) {
      return Fail("observation block has an empty name", error_message);
    }
    if (!index->emplace(observation.name(), &observation).second) {
      return Fail("duplicate observation block: " + observation.name(),
                  error_message);
    }
  }
  return true;
}

template <std::size_t Size>
bool CopyObservation(const ObservationIndex& index, const std::string& name,
                     std::array<double, Size>* output,
                     std::string* error_message) {
  const auto found = index.find(name);
  if (found == index.end()) {
    return Fail("missing observation block: " + name, error_message);
  }
  const auto& values = found->second->values();
  if (values.size() != static_cast<int>(Size)) {
    return Fail("observation dimension mismatch for " + name + ": expected " +
                    std::to_string(Size) + ", got " +
                    std::to_string(values.size()),
                error_message);
  }
  std::array<double, Size> next{};
  for (std::size_t index_value = 0; index_value < Size; ++index_value) {
    const double value = values.Get(static_cast<int>(index_value));
    if (!std::isfinite(value)) {
      return Fail("non-finite observation value in " + name, error_message);
    }
    next[index_value] = value;
  }
  *output = next;
  return true;
}

bool CopyScalar(const ObservationIndex& index, const std::string& name,
                double* output, std::string* error_message) {
  std::array<double, 1> value{};
  if (!CopyObservation(index, name, &value, error_message)) return false;
  *output = value[0];
  return true;
}

bool SimulationTime(const mujoco_sim::v1::Header& header, double* output,
                    std::string* error_message) {
  if (!header.has_simulation_time_s() ||
      !std::isfinite(header.simulation_time_s()) ||
      header.simulation_time_s() < 0.0) {
    return Fail("message header has no valid simulation_time_s",
                error_message);
  }
  *output = header.simulation_time_s();
  return true;
}

template <std::size_t Size>
void Append(const std::array<std::string, Size>& source,
            std::vector<std::string>* target) {
  target->insert(target->end(), source.begin(), source.end());
}

}  // namespace

std::vector<std::string> ControllerObservationNames(
    const ControllerInterfaces& interfaces) {
  std::vector<std::string> result;
  result.reserve(6);
  Append(interfaces.wheel_positions, &result);
  result.push_back(interfaces.imu_quaternion);
  result.push_back(interfaces.imu_angular_velocity);
  return result;
}

std::vector<std::string> ControllerCommandNames(
    const ControllerInterfaces& interfaces) {
  return {interfaces.wheel_velocity_commands.begin(),
          interfaces.wheel_velocity_commands.end()};
}

std::vector<std::string> StateObservationNames(
    const StateObserverInterfaces& interfaces) {
  std::vector<std::string> result;
  result.reserve(11);
  Append(interfaces.wheel_positions, &result);
  Append(interfaces.wheel_velocities, &result);
  result.push_back(interfaces.imu_quaternion);
  result.push_back(interfaces.imu_angular_velocity);
  result.push_back(interfaces.imu_linear_acceleration);
  return result;
}

bool DecodeVehicleSample(const mujoco_sim::v1::Tick& tick,
                         const ControllerInterfaces& interfaces,
                         VehicleSample* sample,
                         std::string* error_message) {
  if (sample == nullptr || error_message == nullptr) {
    return Fail("sample and error_message must not be null", error_message);
  }
  error_message->clear();
  ObservationIndex index;
  if (!BuildIndex(tick.observations(), &index, error_message)) return false;

  VehicleSample next;
  if (!SimulationTime(tick.header(), &next.simulation_time, error_message)) {
    return false;
  }
  for (std::size_t i = 0; i < interfaces.wheel_positions.size(); ++i) {
    if (!CopyScalar(index, interfaces.wheel_positions[i],
                    &next.wheel_positions[i], error_message)) {
      return false;
    }
  }
  if (!CopyObservation(index, interfaces.imu_quaternion,
                       &next.imu_quaternion, error_message) ||
      !CopyObservation(index, interfaces.imu_angular_velocity,
                       &next.imu_gyro, error_message)) {
    return false;
  }
  *sample = next;
  return true;
}

bool DecodeStateSample(const mujoco_sim::v1::ObservationFrame& frame,
                       const StateObserverInterfaces& interfaces,
                       StateSample* sample,
                       std::string* error_message) {
  if (sample == nullptr || error_message == nullptr) {
    return Fail("sample and error_message must not be null", error_message);
  }
  error_message->clear();
  ObservationIndex index;
  if (!BuildIndex(frame.observations(), &index, error_message)) return false;

  StateSample next;
  if (!SimulationTime(frame.header(), &next.simulation_time, error_message)) {
    return false;
  }
  for (std::size_t i = 0; i < interfaces.wheel_positions.size(); ++i) {
    if (!CopyScalar(index, interfaces.wheel_positions[i],
                    &next.wheel_positions[i], error_message) ||
        !CopyScalar(index, interfaces.wheel_velocities[i],
                    &next.wheel_velocities[i], error_message)) {
      return false;
    }
  }
  if (!CopyObservation(index, interfaces.imu_quaternion,
                       &next.imu_quaternion, error_message) ||
      !CopyObservation(index, interfaces.imu_angular_velocity,
                       &next.imu_angular_velocity, error_message) ||
      !CopyObservation(index, interfaces.imu_linear_acceleration,
                       &next.imu_linear_acceleration, error_message)) {
    return false;
  }
  *sample = next;
  return true;
}

bool DecodeImuOrientation(const mujoco_sim::v1::ObservationFrame& frame,
                          const std::string& observation_name,
                          ImuOrientationSample* sample,
                          std::string* error_message) {
  if (sample == nullptr || error_message == nullptr) {
    return Fail("sample and error_message must not be null", error_message);
  }
  error_message->clear();
  ObservationIndex index;
  if (!BuildIndex(frame.observations(), &index, error_message)) return false;
  ImuOrientationSample next;
  if (!SimulationTime(frame.header(), &next.simulation_time, error_message) ||
      !CopyObservation(index, observation_name, &next.quaternion,
                       error_message)) {
    return false;
  }
  *sample = next;
  return true;
}

std::vector<mujoco_sim::adapter_sdk::CommandValue> BuildWheelCommands(
    const ControllerInterfaces& interfaces,
    const std::array<double, 4>& wheel_angular_velocities) {
  std::vector<mujoco_sim::adapter_sdk::CommandValue> result;
  result.reserve(wheel_angular_velocities.size());
  for (std::size_t i = 0; i < wheel_angular_velocities.size(); ++i) {
    result.push_back(
        {interfaces.wheel_velocity_commands[i], wheel_angular_velocities[i]});
  }
  return result;
}

}  // namespace mujoco_sim::sensor_demo
