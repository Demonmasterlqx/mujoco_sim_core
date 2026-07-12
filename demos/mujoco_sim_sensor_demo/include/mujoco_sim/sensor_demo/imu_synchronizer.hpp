#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>

#include <Eigen/Geometry>

namespace mujoco_sim::sensor_demo {

struct TimedOrientation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double simulation_time = 0.0;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class ImuSynchronizer {
 public:
  explicit ImuSynchronizer(std::size_t capacity);

  // Samples must be strictly time ordered. Invalid input leaves the buffer
  // unchanged.
  bool Add(TimedOrientation sample, std::string* error_message);
  std::optional<TimedOrientation> FindNearest(double simulation_time,
                                               double maximum_age_s) const;

  std::size_t size() const noexcept;
  void Clear() noexcept;

 private:
  std::size_t capacity_ = 0;
  std::deque<TimedOrientation> samples_;
};

}  // namespace mujoco_sim::sensor_demo
