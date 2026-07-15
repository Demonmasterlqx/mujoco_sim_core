#include <mujoco_sim/sensor_demo/interface_data.hpp>

#include <algorithm>
#include <cmath>
#include <span>

namespace mujoco_sim::sensor_demo {
namespace {

bool Fail(const std::string& message, std::string* error) {
  if (error) *error = message;
  return false;
}

template <std::size_t Size>
bool Copy(std::span<const double> values,
          mujoco_sim::adapter_sdk::ScalarSpan span, const char* name,
          std::array<double, Size>* output, std::string* error) {
  if (span.count != Size || span.offset + Size > values.size()) {
    return Fail(std::string("observation layout mismatch for ") + name,
                error);
  }
  for (std::size_t i = 0; i < Size; ++i) {
    const double value = values[span.offset + i];
    if (!std::isfinite(value)) {
      return Fail(std::string("non-finite observation value in ") + name,
                  error);
    }
    (*output)[i] = value;
  }
  return true;
}

bool ResolveObservation(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
    const std::string& name, std::size_t count,
    mujoco_sim::adapter_sdk::ScalarSpan* output, std::string* error) {
  const auto span = layout.Observation(name);
  if (!span || span->count != count ||
      span->offset + span->count > layout.observation_scalar_count()) {
    if (error->empty()) *error = "observation layout mismatch for " + name;
    return false;
  }
  *output = *span;
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
  Append(interfaces.wheel_positions, &result);
  Append(interfaces.wheel_velocities, &result);
  result.push_back(interfaces.imu_quaternion);
  result.push_back(interfaces.imu_angular_velocity);
  result.push_back(interfaces.imu_linear_acceleration);
  return result;
}

VehicleDecoder::VehicleDecoder(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
    const ControllerInterfaces& interfaces) {
  for (std::size_t i = 0; i < wheel_positions_.size(); ++i) {
    ResolveObservation(layout, interfaces.wheel_positions[i], 1,
                       &wheel_positions_[i], &layout_error_);
  }
  ResolveObservation(layout, interfaces.imu_quaternion, 4, &imu_quaternion_,
                     &layout_error_);
  ResolveObservation(layout, interfaces.imu_angular_velocity, 3, &imu_gyro_,
                     &layout_error_);
}

bool VehicleDecoder::Decode(const mujoco_sim::adapter_sdk::TickLease& tick,
                            VehicleSample* sample, std::string* error) const {
  return Decode(tick.observations(), tick.simulation_time_s(), sample, error);
}

bool VehicleDecoder::Decode(std::span<const double> values,
                            double simulation_time_s, VehicleSample* sample,
                            std::string* error) const {
  if (!sample || !error) return Fail("invalid output arguments", error);
  if (!layout_error_.empty()) return Fail(layout_error_, error);
  VehicleSample next;
  next.simulation_time = simulation_time_s;
  for (std::size_t i = 0; i < wheel_positions_.size(); ++i) {
    std::array<double, 1> value{};
    if (!Copy(values, wheel_positions_[i], "wheel position", &value, error))
      return false;
    next.wheel_positions[i] = value[0];
  }
  if (!Copy(values, imu_quaternion_, "IMU quaternion", &next.imu_quaternion,
            error) ||
      !Copy(values, imu_gyro_, "IMU angular velocity", &next.imu_gyro,
            error))
    return false;
  *sample = next;
  return true;
}

StateDecoder::StateDecoder(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
    const StateObserverInterfaces& interfaces) {
  for (std::size_t i = 0; i < wheel_positions_.size(); ++i) {
    ResolveObservation(layout, interfaces.wheel_positions[i], 1,
                       &wheel_positions_[i], &layout_error_);
    ResolveObservation(layout, interfaces.wheel_velocities[i], 1,
                       &wheel_velocities_[i], &layout_error_);
  }
  ResolveObservation(layout, interfaces.imu_quaternion, 4, &imu_quaternion_,
                     &layout_error_);
  ResolveObservation(layout, interfaces.imu_angular_velocity, 3,
                     &imu_angular_velocity_, &layout_error_);
  ResolveObservation(layout, interfaces.imu_linear_acceleration, 3,
                     &imu_linear_acceleration_, &layout_error_);
}

bool StateDecoder::Decode(
    const mujoco_sim::adapter_sdk::ObservationLease& frame,
    StateSample* sample, std::string* error) const {
  return Decode(frame.observations(), frame.simulation_time_s(), sample,
                error);
}

bool StateDecoder::Decode(std::span<const double> values,
                          double simulation_time_s, StateSample* sample,
                          std::string* error) const {
  if (!sample || !error) return Fail("invalid output arguments", error);
  if (!layout_error_.empty()) return Fail(layout_error_, error);
  StateSample next;
  next.simulation_time = simulation_time_s;
  for (std::size_t i = 0; i < wheel_positions_.size(); ++i) {
    std::array<double, 1> position{}, velocity{};
    if (!Copy(values, wheel_positions_[i], "wheel position", &position,
              error) ||
        !Copy(values, wheel_velocities_[i], "wheel velocity", &velocity,
              error))
      return false;
    next.wheel_positions[i] = position[0];
    next.wheel_velocities[i] = velocity[0];
  }
  if (!Copy(values, imu_quaternion_, "IMU quaternion", &next.imu_quaternion,
            error) ||
      !Copy(values, imu_angular_velocity_, "IMU angular velocity",
            &next.imu_angular_velocity, error) ||
      !Copy(values, imu_linear_acceleration_, "IMU linear acceleration",
            &next.imu_linear_acceleration, error))
    return false;
  *sample = next;
  return true;
}

ImuOrientationDecoder::ImuOrientationDecoder(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
    const std::string& name) {
  ResolveObservation(layout, name, 4, &quaternion_, &layout_error_);
}

bool ImuOrientationDecoder::Decode(
    const mujoco_sim::adapter_sdk::ObservationLease& frame,
    ImuOrientationSample* sample, std::string* error) const {
  return Decode(frame.observations(), frame.simulation_time_s(), sample,
                error);
}

bool ImuOrientationDecoder::Decode(std::span<const double> values,
                                   double simulation_time_s,
                                   ImuOrientationSample* sample,
                                   std::string* error) const {
  if (!sample || !error) return Fail("invalid output arguments", error);
  if (!layout_error_.empty()) return Fail(layout_error_, error);
  ImuOrientationSample next;
  next.simulation_time = simulation_time_s;
  if (!Copy(values, quaternion_, "IMU quaternion", &next.quaternion, error))
    return false;
  *sample = next;
  return true;
}

WheelCommandMapper::WheelCommandMapper(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout,
    const ControllerInterfaces& interfaces)
    : scalar_count_(layout.command_scalar_count()) {
  for (std::size_t i = 0; i < offsets_.size(); ++i) {
    const auto span = layout.Command(interfaces.wheel_velocity_commands[i]);
    if (!span || span->count != 1 || span->offset >= scalar_count_) {
      if (layout_error_.empty()) {
        layout_error_ = "command layout mismatch for " +
                        interfaces.wheel_velocity_commands[i];
      }
      continue;
    }
    offsets_[i] = span->offset;
  }
}

bool WheelCommandMapper::Map(const std::array<double, 4>& velocities,
                             std::vector<double>* commands,
                             std::string* error) const {
  if (!commands || !error) return Fail("invalid output arguments", error);
  if (!layout_error_.empty()) return Fail(layout_error_, error);
  std::vector<double> next(scalar_count_);
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    if (!std::isfinite(velocities[i])) {
      return Fail("non-finite wheel command", error);
    }
    next[offsets_[i]] = velocities[i];
  }
  *commands = std::move(next);
  return true;
}

}  // namespace mujoco_sim::sensor_demo
