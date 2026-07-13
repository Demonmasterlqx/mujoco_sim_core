#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/v2/sync.pb.h>
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
void ValidateWbrLayout(const mujoco_sim::v2::ResourceLayout& layout);
void ValidateWbrLayout(const InterfaceConfig& config,
                       const mujoco_sim::v2::ResourceLayout& layout);

class TickDecoder {
 public:
  explicit TickDecoder(const mujoco_sim::v2::ResourceLayout& registered_layout);
  TickDecoder(InterfaceConfig config,
              const mujoco_sim::v2::ResourceLayout& registered_layout);

  control::msg_raw_state_t Decode(const mujoco_sim::v2::Tick& tick) const;

 private:
  InterfaceConfig config_;
  std::string registered_layout_hash_;
};

class TorqueCommandMapper {
 public:
  explicit TorqueCommandMapper(
      const mujoco_sim::v2::ResourceLayout& registered_layout);
  TorqueCommandMapper(InterfaceConfig config,
                      const mujoco_sim::v2::ResourceLayout& registered_layout);

  std::vector<mujoco_sim::adapter_sdk::CommandValue> Map(
      const std::array<float, 6>& raw_torque) const;
  std::size_t last_clamp_count() const { return last_clamp_count_; }

 private:
  struct Range {
    double minimum = 0.0;
    double maximum = 0.0;
  };

  std::array<Range, 6> ranges_{};
  std::array<std::size_t, 6> registered_order_{};
  std::array<std::string, 6> command_names_{};
  mutable std::size_t last_clamp_count_ = 0;
};

}  // namespace mujoco_sim::wbr_lqr_demo
