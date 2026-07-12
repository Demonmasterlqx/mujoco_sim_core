#include <mujoco_sim/sensor_demo/imu_synchronizer.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace mujoco_sim::sensor_demo {

ImuSynchronizer::ImuSynchronizer(std::size_t capacity) : capacity_(capacity) {
  if (capacity_ == 0) {
    throw std::invalid_argument("IMU synchronizer capacity must be positive");
  }
}

bool ImuSynchronizer::Add(TimedOrientation sample,
                          std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  if (!std::isfinite(sample.simulation_time) || sample.simulation_time < 0.0) {
    *error_message = "IMU sample timestamp must be finite and non-negative";
    return false;
  }
  if (!samples_.empty() &&
      sample.simulation_time <= samples_.back().simulation_time) {
    *error_message = "IMU sample timestamps must increase strictly";
    return false;
  }
  if (!sample.orientation.coeffs().allFinite() ||
      sample.orientation.norm() < 1e-12) {
    *error_message = "IMU orientation must be finite and non-zero";
    return false;
  }
  sample.orientation.normalize();
  samples_.push_back(std::move(sample));
  while (samples_.size() > capacity_) samples_.pop_front();
  return true;
}

std::optional<TimedOrientation> ImuSynchronizer::FindNearest(
    double simulation_time, double maximum_age_s) const {
  if (!std::isfinite(simulation_time) || simulation_time < 0.0 ||
      !std::isfinite(maximum_age_s) || maximum_age_s < 0.0 ||
      samples_.empty()) {
    return std::nullopt;
  }
  const auto after = std::lower_bound(
      samples_.begin(), samples_.end(), simulation_time,
      [](const TimedOrientation& sample, double time) {
        return sample.simulation_time < time;
      });
  auto nearest = after;
  if (after == samples_.end()) {
    nearest = std::prev(samples_.end());
  } else if (after != samples_.begin()) {
    const auto before = std::prev(after);
    if (simulation_time - before->simulation_time <=
        after->simulation_time - simulation_time) {
      nearest = before;
    }
  }
  if (std::abs(nearest->simulation_time - simulation_time) > maximum_age_s) {
    return std::nullopt;
  }
  return *nearest;
}

std::size_t ImuSynchronizer::size() const noexcept { return samples_.size(); }

void ImuSynchronizer::Clear() noexcept { samples_.clear(); }

}  // namespace mujoco_sim::sensor_demo
