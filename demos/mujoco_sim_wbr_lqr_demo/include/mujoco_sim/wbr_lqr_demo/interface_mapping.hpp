#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/adapter_sdk/controller_client.hpp>
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

/**
 * @brief Validates the upstream WBR resource contract against a dense layout.
 *
 * The layout must contain exactly the required resources and scalar counts.
 * Every scalar span must be in bounds, non-overlapping, and cover the complete
 * dense observation or command vector.
 *
 * 根据 dense layout 校验上游 WBR 资源契约。
 *
 * layout 必须包含且仅包含所需资源及标量数量。每个标量 span 都必须位于边界内、
 * 互不重叠，并完整覆盖 dense observation 或 command 向量。
 *
 * @param layout Frozen Controller layout returned by registration.
 *               注册完成后返回的冻结 Controller layout。
 * @throws InterfaceError if the layout violates the WBR resource contract.
 *                        layout 违反 WBR 资源契约时抛出 InterfaceError。
 */
void ValidateWbrLayout(const mujoco_sim::adapter_sdk::ControllerLayout& layout);

/**
 * @brief Validates a configured WBR interface against a dense layout.
 *
 * 根据 dense layout 校验配置后的 WBR 接口。
 *
 * @param config Resource names expected by the WBR integration.
 *               WBR 集成所期望的资源名称。
 * @param layout Frozen Controller layout returned by registration.
 *               注册完成后返回的冻结 Controller layout。
 * @throws InterfaceError if names, dimensions, ranges, or dense spans are invalid.
 *                        名称、维度、范围或 dense span 无效时抛出 InterfaceError。
 */
void ValidateWbrLayout(const InterfaceConfig& config,
                       const mujoco_sim::adapter_sdk::ControllerLayout& layout);

class StateDecoder {
 public:
  explicit StateDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);
  StateDecoder(InterfaceConfig config,
               const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);

  control::msg_raw_state_t Decode(const mujoco_sim::adapter_sdk::StateLease& state) const;

  /**
   * @brief Decodes one complete dense observation vector.
   *
   * The input length must exactly match the scalar count frozen at
   * registration. Every value and the simulation timestamp must be finite.
   * The decoder borrows the span only for the duration of this call.
   *
   * 解码一帧完整的 dense observation 向量。
   *
   * 输入长度必须与注册时冻结的标量数量完全一致。所有数值和仿真时间戳都必须为
   * 有限值。decoder 仅在本次调用期间借用该 span。
   *
   * @param observations Dense observation scalars in the registered layout.
   *                     按注册 layout 排列的 dense observation 标量。
   * @param simulation_time_s Simulation timestamp in seconds.
   *                          以秒为单位的仿真时间戳。
   * @return Upstream WBR raw-state value owning all decoded data.
   *         持有全部已解码数据的上游 WBR raw-state 值。
   * @throws InterfaceError if the input size or a numeric value is invalid.
   *                        输入尺寸或数值无效时抛出 InterfaceError。
   */
  control::msg_raw_state_t Decode(std::span<const double> observations,
                                  double simulation_time_s) const;

 private:
  InterfaceConfig config_;
  std::size_t observation_scalar_count_ = 0;
  mujoco_sim::adapter_sdk::ScalarSpan quaternion_{};
  mujoco_sim::adapter_sdk::ScalarSpan acceleration_{};
  mujoco_sim::adapter_sdk::ScalarSpan angular_velocity_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 6> positions_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 6> velocities_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 6> efforts_{};
};

class TorqueCommandMapper {
 public:
  explicit TorqueCommandMapper(const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout);
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
