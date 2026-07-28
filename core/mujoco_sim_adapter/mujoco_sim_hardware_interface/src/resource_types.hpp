#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "resource.hpp"

namespace mujoco_sim::hardware::internal {

/** @brief Realtime publisher type for scalar debug values.
 *         标量调试值使用的实时 publisher 类型。 */
using ScalarPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64>;
/** @brief Realtime publisher type for grouped debug values.
 *         分组调试值使用的实时 publisher 类型。 */
using ArrayPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
/** @brief Realtime publisher type for one IMU-owned dynamic transform.
 *         单个 IMU 自有动态变换使用的 realtime publisher 类型。 */
using TfPublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;

/** @brief Implements scalar joint state and command mappings.
 *         实现标量 joint 状态与指令映射。 */
class JointResource final : public ResourceInterface {
 public:
  /**
   * @brief Takes ownership of one validated scalar joint mapping.
   *        取得一个已校验标量 joint 映射的所有权。
   * @param name Non-empty ROS joint name. 非空 ROS joint 名称。
   * @param debug Whether scalar realtime debug topics are enabled. 是否启用标量实时调试 Topic。
   * @param states Generated scalar state bindings. 自动生成的标量状态绑定。
   * @param commands Generated scalar command bindings. 自动生成的标量指令绑定。
   */
  JointResource(std::string name, bool debug, std::vector<StateBinding> states,
                std::vector<CommandBinding> commands);
  const std::string& name() const noexcept override;
  bool RegisterInterfaces(InterfaceRegistry& registry, std::string& error) override;
  std::span<const StateBinding> state_bindings() const noexcept override;
  std::span<const CommandBinding> command_bindings() const noexcept override;
  bool Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) override;
  bool StageState(std::span<const double> observations, std::span<double> staged_state,
                  std::string& error) const override;
  bool StageCommand(std::span<double> ros_commands, std::span<double> dense_command,
                    std::string& error) const override;
  bool ConfigurePublishers(const rclcpp::Node::SharedPtr& node, std::string& error) override;
  void ResetPublishers() noexcept override;
  void OnStateCommitted(std::span<const double> state_values,
                        const builtin_interfaces::msg::Time& stamp) noexcept override;
  void OnCommandAccepted(std::span<const double> command_values) noexcept override;

 private:
  /// Owned ROS joint component name. 持有的 ROS joint 组件名。
  std::string name_;
  /// Immutable component-level debug setting. 不可变的组件级调试设置。
  bool debug_ = false;
  /// State bindings rebound to each active session's immutable layout.
  /// 每次激活时重新绑定到该会话不可变布局的状态绑定。
  std::vector<StateBinding> states_;
  /// Command bindings rebound to each active session's immutable layout.
  /// 每次激活时重新绑定到该会话不可变布局的指令绑定。
  std::vector<CommandBinding> commands_;
  /// One optional realtime scalar publisher per state binding. 每个状态绑定对应一个可选实时标量
  /// publisher。
  std::vector<std::unique_ptr<ScalarPublisher>> state_publishers_;
  /// One optional realtime scalar publisher per command binding. 每个指令绑定对应一个可选实时标量
  /// publisher。
  std::vector<std::unique_ptr<ScalarPublisher>> command_publishers_;

  /** @brief Returns this resource's exclusive debug topics. 返回本资源独占调试 Topic。 */
  std::vector<std::string> ExclusivePublisherTopics() const;
};

/** @brief Implements one grouped generic sensor mapping.
 *         实现一个分组通用 sensor 映射。 */
class GenericSensorResource final : public ResourceInterface {
 public:
  /**
   * @brief Takes ownership of one validated grouped sensor mapping.
   *        取得一个已校验分组 sensor 映射的所有权。
   * @param name Non-empty ROS sensor component name. 非空 ROS sensor 组件名。
   * @param debug Whether the grouped realtime debug topic is enabled.
   *              是否启用分组实时调试 Topic。
   * @param states Ordered bindings whose count equals sensor_dim.
   *               数量等于 sensor_dim 的有序状态绑定。
   */
  GenericSensorResource(std::string name, bool debug, std::vector<StateBinding> states);
  const std::string& name() const noexcept override;
  bool RegisterInterfaces(InterfaceRegistry& registry, std::string& error) override;
  std::span<const StateBinding> state_bindings() const noexcept override;
  std::span<const CommandBinding> command_bindings() const noexcept override;
  bool Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) override;
  bool StageState(std::span<const double> observations, std::span<double> staged_state,
                  std::string& error) const override;
  bool StageCommand(std::span<double> ros_commands, std::span<double> dense_command,
                    std::string& error) const override;
  bool ConfigurePublishers(const rclcpp::Node::SharedPtr& node, std::string& error) override;
  void ResetPublishers() noexcept override;
  void OnStateCommitted(std::span<const double> state_values,
                        const builtin_interfaces::msg::Time& stamp) noexcept override;
  void OnCommandAccepted(std::span<const double> command_values) noexcept override;

 private:
  /// Owned ROS sensor component name. 持有的 ROS sensor 组件名。
  std::string name_;
  /// Immutable component-level debug setting. 不可变的组件级调试设置。
  bool debug_ = false;
  /// State bindings in the public sensor_dim_names order. 按公开 sensor_dim_names
  /// 顺序排列的状态绑定。
  std::vector<StateBinding> states_;
  /// Permanently empty command view required by the common resource contract.
  /// 公共资源契约所需的永久空指令视图。
  std::vector<CommandBinding> commands_;
  /// Optional publisher whose preallocated message contains the whole sensor array.
  /// 可选 publisher，其预分配消息包含整组 sensor 数组。
  std::unique_ptr<ArrayPublisher> debug_publisher_;

  /** @brief Returns this resource's exclusive debug topic. 返回本资源独占调试 Topic。 */
  std::vector<std::string> ExclusivePublisherTopics() const;
};

/** @brief Implements grouped six-axis IMU state and optional direct pose mapping.
 *         实现分组六轴 IMU 状态与可选直接位姿映射。 */
class ImuResource final : public ResourceInterface {
 public:
  /**
   * @brief Takes ownership of one six-axis IMU and optional direct-pose mapping.
   *        取得一个六轴 IMU 与可选直接位姿映射的所有权。
   * @param name Non-empty ROS sensor component name. 非空 ROS sensor 组件名。
   * @param debug Whether the grouped realtime debug topic is enabled.
   *              是否启用分组实时调试 Topic。
   * @param parent_frame Valid parent TF frame ID. 合法的父 TF 坐标系 ID。
   * @param child_frame Valid distinct child TF frame ID. 合法且不同的子 TF 坐标系 ID。
   * @param states Ordered acceleration, gyro, and optional pose bindings.
   *               按加速度、陀螺仪及可选位姿排列的状态绑定。
   * @param has_pose true only when quaternion and position are both configured.
   *                 仅当四元数与位置均已配置时为 true。
   */
  ImuResource(std::string name, bool debug, std::string parent_frame, std::string child_frame,
              std::vector<StateBinding> states, bool has_pose);
  const std::string& name() const noexcept override;
  bool RegisterInterfaces(InterfaceRegistry& registry, std::string& error) override;
  std::span<const StateBinding> state_bindings() const noexcept override;
  std::span<const CommandBinding> command_bindings() const noexcept override;
  bool Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) override;
  bool StageState(std::span<const double> observations, std::span<double> staged_state,
                  std::string& error) const override;
  bool StageCommand(std::span<double> ros_commands, std::span<double> dense_command,
                    std::string& error) const override;
  bool ConfigurePublishers(const rclcpp::Node::SharedPtr& node, std::string& error) override;
  void ResetPublishers() noexcept override;
  void OnStateCommitted(std::span<const double> state_values,
                        const builtin_interfaces::msg::Time& stamp) noexcept override;
  void OnCommandAccepted(std::span<const double> command_values) noexcept override;

 private:
  /// Owned ROS IMU component name. 持有的 ROS IMU 组件名。
  std::string name_;
  /// Immutable component-level debug setting. 不可变的组件级调试设置。
  bool debug_ = false;
  /// Owned parent TF frame ID. 持有的父 TF 坐标系 ID。
  std::string parent_frame_;
  /// Owned child TF frame ID. 持有的子 TF 坐标系 ID。
  std::string child_frame_;
  /// Whether states_ also contains normalized orientation and position.
  /// states_ 是否还包含归一化姿态与位置。
  bool has_pose_ = false;
  /// Ordered acceleration, gyro, orientation xyzw, and position xyz bindings.
  /// 按加速度、陀螺仪、姿态 xyzw 与位置 xyz 排列的绑定。
  std::vector<StateBinding> states_;
  /// Permanently empty command view required by the common resource contract.
  /// 公共资源契约所需的永久空指令视图。
  std::vector<CommandBinding> commands_;
  /// Optional publisher whose preallocated message contains the whole IMU array.
  /// 可选 publisher，其预分配消息包含整组 IMU 数组。
  std::unique_ptr<ArrayPublisher> debug_publisher_;
  /// Optional IMU-owned publisher for the configured direct pose. 可选的 IMU 自有直接位姿
  /// publisher。
  std::unique_ptr<TfPublisher> transform_publisher_;

  /** @brief Returns this resource's exclusive debug topic. 返回本资源独占调试 Topic。 */
  std::vector<std::string> ExclusivePublisherTopics() const;
};

/** @brief Non-owning optional reference to immutable resource metadata.
 *         指向不可变资源元数据的非持有可选引用。 */
using ResourceInfoReference = std::reference_wrapper<const adapter_sdk::ResourceInfo>;
/** @brief Finds exact observation metadata. 查找精确的观测元数据。 */
std::optional<ResourceInfoReference> FindObservation(const adapter_sdk::ControllerLayout& layout,
                                                     const std::string& name);
/** @brief Finds exact command metadata. 查找精确的指令元数据。 */
std::optional<ResourceInfoReference> FindCommand(const adapter_sdk::ControllerLayout& layout,
                                                 const std::string& name);
/** @brief Binds state metadata shared by resource implementations.
 *         绑定资源实现共用的状态元数据。 */
bool BindStates(const adapter_sdk::ControllerLayout& layout, std::vector<StateBinding>& states,
                std::string& error);
/** @brief Binds command metadata shared by resource implementations.
 *         绑定资源实现共用的指令元数据。 */
bool BindCommands(const adapter_sdk::ControllerLayout& layout,
                  std::vector<CommandBinding>& commands, std::string& error);
/** @brief Reads finite state scalars shared by resource implementations.
 *         读取资源实现共用的有限状态标量。 */
bool ReadBindings(const std::vector<StateBinding>& states, std::span<const double> observations,
                  std::span<double> staged_state, std::string& error);
/** @brief Validates state spans and scalars without changing the staging array.
 *         在不修改暂存数组的情况下校验状态区间与标量。 */
bool ValidateReadBindings(const std::vector<StateBinding>& states,
                          std::span<const double> observations,
                          std::span<const double> staged_state, std::string& error);
/** @brief Copies previously validated state scalars into their generated ROS order.
 *         将已校验的状态标量复制到自动生成的 ROS 顺序。 */
void CopyReadBindings(const std::vector<StateBinding>& states, std::span<const double> observations,
                      std::span<double> staged_state) noexcept;

}  // namespace mujoco_sim::hardware::internal
