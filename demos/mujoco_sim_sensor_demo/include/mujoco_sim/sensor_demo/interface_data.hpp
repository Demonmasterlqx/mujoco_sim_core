#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/sensor_demo/waypoint_controller.hpp>

namespace mujoco_sim::sensor_demo {

struct ControllerInterfaces {
  std::array<std::string, 4> wheel_positions;
  std::string imu_quaternion;
  std::string imu_angular_velocity;
  std::array<std::string, 4> wheel_velocity_commands;
};

struct StateObserverInterfaces {
  std::array<std::string, 4> wheel_positions;
  std::array<std::string, 4> wheel_velocities;
  std::string imu_quaternion;
  std::string imu_angular_velocity;
  std::string imu_linear_acceleration;
};

struct StateSample {
  double simulation_time = 0.0;
  std::array<double, 4> wheel_positions{};
  std::array<double, 4> wheel_velocities{};
  std::array<double, 4> imu_quaternion{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> imu_angular_velocity{};
  std::array<double, 3> imu_linear_acceleration{};
};

struct ImuOrientationSample {
  double simulation_time = 0.0;
  std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
};

std::vector<std::string> ControllerObservationNames(const ControllerInterfaces& interfaces);
std::vector<std::string> ControllerCommandNames(const ControllerInterfaces& interfaces);
std::vector<std::string> StateObservationNames(const StateObserverInterfaces& interfaces);

class VehicleDecoder {
 public:
  VehicleDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                 const ControllerInterfaces& interfaces);
  bool Decode(const mujoco_sim::adapter_sdk::StateLease& tick, VehicleSample* sample,
              std::string* error_message) const;

  /**
   * @brief Decodes one complete Controller observation vector transactionally.
   *
   * The scalar count must exactly match the registered layout, the simulation
   * timestamp must be finite and non-negative, and all decoded scalars must be
   * finite. Output remains unchanged on failure.
   *
   * 以事务方式解码一帧完整的 Controller observation 向量。
   *
   * 标量数量必须与注册 layout 完全一致，仿真时间戳必须为有限非负值，所有解码标量
   * 也必须为有限值。失败时输出保持不变。
   *
   * @param observations Complete dense observation vector. / 完整的稠密 observation 向量。
   * @param simulation_time_s Simulation timestamp in seconds. / 以秒为单位的仿真时间戳。
   * @param sample Receives the decoded sample and must not be null. /
   *               接收解码结果，且不得为空。
   * @param error_message Receives a failure diagnostic and must not be null. /
   *                      接收失败诊断，且不得为空。
   * @return true on success; false when any input or cached layout is invalid. /
   *         成功时返回 true；任一输入或缓存 layout 无效时返回 false。
   *
   * @par Thread safety / 线程安全
   * Concurrent calls are safe when each call uses distinct output objects. /
   * 当每次调用使用不同的输出对象时，可以并发调用。
   */
  bool Decode(std::span<const double> observations, double simulation_time_s, VehicleSample* sample,
              std::string* error_message) const;

 private:
  std::size_t scalar_count_ = 0;
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 4> wheel_positions_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_quaternion_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_gyro_{};
  std::string layout_error_;
};

class StateDecoder {
 public:
  StateDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
               const StateObserverInterfaces& interfaces);
  bool Decode(const mujoco_sim::adapter_sdk::ObservationLease& frame, StateSample* sample,
              std::string* error_message) const;

  /**
   * @brief Decodes one complete Observer vector without partial output.
   *
   * 解码一帧完整 Observer 向量，失败时不产生部分输出。
   *
   * The scalar count must match the registered layout and all timestamps and
   * decoded values must be finite and non-negative where applicable.
   * 标量数量必须与注册 layout 一致；时间戳和解码值必须为有限值，时间戳还必须非负。
   *
   * @param observations Complete dense observation vector. / 完整的稠密 observation 向量。
   * @param simulation_time_s Simulation timestamp in seconds. / 以秒为单位的仿真时间戳。
   * @param sample Receives the decoded sample and must not be null. /
   *               接收解码结果，且不得为空。
   * @param error_message Receives a failure diagnostic and must not be null. /
   *                      接收失败诊断，且不得为空。
   * @return true on success; false when any input or cached layout is invalid. /
   *         成功时返回 true；任一输入或缓存 layout 无效时返回 false。
   *
   * @par Thread safety / 线程安全
   * Concurrent calls are safe when each call uses distinct output objects. /
   * 当每次调用使用不同的输出对象时，可以并发调用。
   */
  bool Decode(std::span<const double> observations, double simulation_time_s, StateSample* sample,
              std::string* error_message) const;

 private:
  std::size_t scalar_count_ = 0;
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 4> wheel_positions_{};
  std::array<mujoco_sim::adapter_sdk::ScalarSpan, 4> wheel_velocities_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_quaternion_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_angular_velocity_{};
  mujoco_sim::adapter_sdk::ScalarSpan imu_linear_acceleration_{};
  std::string layout_error_;
};

class ImuOrientationDecoder {
 public:
  ImuOrientationDecoder(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                        const std::string& observation_name);
  bool Decode(const mujoco_sim::adapter_sdk::ObservationLease& frame, ImuOrientationSample* sample,
              std::string* error_message) const;

  /**
   * @brief Decodes a finite IMU orientation from one complete Observer vector.
   *        从一帧完整 Observer 向量解码有限的 IMU 姿态。
   *
   * Output remains unchanged if the vector size, timestamp, or quaternion is
   * invalid.
   * 向量尺寸、时间戳或四元数无效时，输出保持不变。
   *
   * @param observations Complete dense observation vector. / 完整的稠密 observation 向量。
   * @param simulation_time_s Simulation timestamp in seconds. / 以秒为单位的仿真时间戳。
   * @param sample Receives the decoded orientation and must not be null. /
   *               接收解码后的姿态，且不得为空。
   * @param error_message Receives a failure diagnostic and must not be null. /
   *                      接收失败诊断，且不得为空。
   * @return true on success; false when any input or cached layout is invalid. /
   *         成功时返回 true；任一输入或缓存 layout 无效时返回 false。
   *
   * @par Thread safety / 线程安全
   * Concurrent calls are safe when each call uses distinct output objects. /
   * 当每次调用使用不同的输出对象时，可以并发调用。
   */
  bool Decode(std::span<const double> observations, double simulation_time_s,
              ImuOrientationSample* sample, std::string* error_message) const;

 private:
  std::size_t scalar_count_ = 0;
  mujoco_sim::adapter_sdk::ScalarSpan quaternion_{};
  std::string layout_error_;
};

class WheelCommandMapper {
 public:
  WheelCommandMapper(const mujoco_sim::adapter_sdk::ControllerLayout& layout,
                     const ControllerInterfaces& interfaces);
  bool Map(const std::array<double, 4>& wheel_angular_velocities, std::vector<double>* commands,
           std::string* error_message) const;

 private:
  std::array<std::size_t, 4> offsets_{};
  std::size_t scalar_count_ = 0;
  std::string layout_error_;
};

}  // namespace mujoco_sim::sensor_demo
