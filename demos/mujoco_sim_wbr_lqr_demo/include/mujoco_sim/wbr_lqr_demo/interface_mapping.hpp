#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <span>
#include <string>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/wbr_lqr_demo/config.hpp>

namespace mujoco_sim::wbr_lqr_demo {

inline constexpr std::size_t kObservationResourceCount = 21;
inline constexpr std::size_t kCommandResourceCount = 6;

class InterfaceError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

std::vector<std::string> WbrObservationNames();
std::vector<std::string> WbrObservationNames(const InterfaceConfig& config);
std::vector<std::string> WbrCommandNames();
std::vector<std::string> WbrCommandNames(const InterfaceConfig& config);
void ValidateWbrLayout(
    const mujoco_sim::adapter_sdk::ControllerLayout& layout);
void ValidateWbrLayout(const InterfaceConfig& config,
                       const mujoco_sim::adapter_sdk::ControllerLayout& layout);

class TickDecoder {
 public:
  explicit TickDecoder(
      const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);
  TickDecoder(InterfaceConfig config,
              const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);

  control::msg_raw_state_t Decode(
      const mujoco_sim::adapter_sdk::TickLease& tick) const;
  control::msg_raw_state_t Decode(std::span<const double> observations,
                                  double simulation_time_s) const;

 private:
  InterfaceConfig config_;
  mujoco_sim::adapter_sdk::ScalarSpan quaternion_{};
  mujoco_sim::adapter_sdk::ScalarSpan acceleration_{};
  mujoco_sim::adapter_sdk::ScalarSpan angular_velocity_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 6> positions_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 6> velocities_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 6> efforts_{};
};

class TorqueCommandMapper {
 public:
  explicit TorqueCommandMapper(
      const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);
  TorqueCommandMapper(InterfaceConfig config,
                      const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);

  std::vector<double> Map(const std::array<float, 6>& raw_torque) const;
  std::size_t last_clamp_count() const { return last_clamp_count_; }

 private:
  struct Range {
    double minimum = 0.0;
    double maximum = 0.0;
  };

  std::array<Range, 6> ranges_{};
  std::array<std::size_t, 6> scalar_offsets_{};
  mutable std::size_t last_clamp_count_ = 0;
};

}  // namespace mujoco_sim::wbr_lqr_demo
