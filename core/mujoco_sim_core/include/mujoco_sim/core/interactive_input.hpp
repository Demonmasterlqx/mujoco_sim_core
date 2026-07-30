#pragma once

#include <array>
#include <span>

namespace mujoco_sim::core {

/**
 * @brief Describes one actuator control write from a local interactive source.
 *
 * The write is applied once at the next simulation Step boundary. It does not
 * acquire Controller ownership and does not remain as a persistent override.
 *
 * 描述来自本地交互源的一次 actuator 控制写入。
 *
 * 该写入在下一个仿真 Step 边界应用一次。它不会取得 Controller 所有权，也不会形成
 * 持续覆盖。
 */
struct InteractiveActuatorWrite {
  /** Zero-based MuJoCo actuator ID. MuJoCo actuator 的零基 ID。 */
  int actuator_id = -1;
  /** Finite control value; no range clamping is performed. 有限控制值；不会执行量程钳制。 */
  double value = 0.0;
};

/**
 * @brief Describes a Cartesian force and torque applied to one dynamic body.
 *
 * Force and torque are expressed in the world frame. The force acts at the
 * body's center of mass and the torque is taken about that center. World,
 * static, invalid, and non-finite writes are ignored.
 *
 * 描述施加到一个动态刚体的笛卡尔力和力矩。
 *
 * 力和力矩均使用世界坐标系。力作用于刚体质心，力矩也以该质心为参考。world body、
 * 静态刚体、非法 ID 和非有限写入会被忽略。
 */
struct InteractiveBodyWrench {
  /** Positive MuJoCo body ID for a body with degrees of freedom.
   * 具有自由度的 MuJoCo 刚体正 ID。 */
  int body_id = -1;
  /** World-frame force at the center of mass, in newtons.
   * 世界坐标系下作用于质心的力，单位为牛顿。 */
  std::array<double, 3> force{};
  /** World-frame torque about the center of mass, in newton-metres.
   * 世界坐标系下相对于质心的力矩，单位为牛米。 */
  std::array<double, 3> torque{};
};

/**
 * @brief Borrows interactive values for one simulation Step boundary.
 *
 * Actuator writes are one-shot and are applied after pending Controller writes;
 * repeated actuator IDs therefore use the last value in this span. When
 * replace_body_wrenches is true, body_wrenches atomically replaces the prior
 * interactive wrench state; an empty span explicitly clears that state.
 * Spans are borrowed only for the duration of SimulationCore::Step().
 *
 * 借用一个仿真 Step 边界所需的交互输入值。
 *
 * actuator 写入是一次性的，并在待处理 Controller 写入之后应用；因此重复 actuator ID
 * 使用 span 中最后一个值。replace_body_wrenches 为 true 时，body_wrenches 会原子替换
 * 之前的交互扰动力状态；空 span 表示显式清除。所有 span 仅在
 * SimulationCore::Step() 调用期间被借用。
 */
struct InteractiveStepInput {
  /** One-shot actuator writes. 一次性 actuator 写入。 */
  std::span<const InteractiveActuatorWrite> actuator_writes;
  /** Complete replacement wrench state when enabled below.
   * 在下方开关启用时使用的完整替换扰动力状态。 */
  std::span<const InteractiveBodyWrench> body_wrenches;
  /** Whether body_wrenches replaces the previous interactive state.
   * body_wrenches 是否替换之前的交互状态。 */
  bool replace_body_wrenches = false;
};

}  // namespace mujoco_sim::core
