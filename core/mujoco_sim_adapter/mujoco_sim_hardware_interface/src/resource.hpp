#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>
#include <rclcpp/node.hpp>

namespace mujoco_sim::hardware::internal {

/** @brief String parameter map used by resource factories.
 *         资源工厂使用的字符串参数表。 */
using ParameterMap = std::unordered_map<std::string, std::string>;

/**
 * @brief Describes one generated ROS state interface and its dense source.
 *        描述一个自动生成的 ROS 状态接口及其稠密数据来源。
 */
struct StateBinding {
  /// ROS component name that owns the generated interface. 自动生成接口所属的 ROS 组件名。
  std::string component;
  /// Interface suffix relative to the component. 相对于组件的接口后缀。
  std::string interface_name;
  /// Protocol V4 observation resource name. Protocol V4 观测资源名。
  std::string resource_name;
  /// Zero-based scalar index within the observation resource. 观测资源内从零开始的标量索引。
  std::size_t resource_component = 0;
  /// Exact registered dimension required for the resource. 该资源要求的精确注册维度。
  std::size_t resource_dimension = 0;
  /// Offset in the complete generated ROS state array. 完整自动生成 ROS 状态数组中的偏移。
  std::size_t state_index = 0;
  /// Bound offset in the session's dense observation array. 会话稠密观测数组中的已绑定偏移。
  std::size_t dense_offset = 0;

  /** @brief Returns `<component>/<interface>`. 返回 `<component>/<interface>`。 */
  std::string FullName() const;
};

/**
 * @brief Describes one generated ROS command interface and its dense destination.
 *        描述一个自动生成的 ROS 指令接口及其稠密数据目标。
 */
struct CommandBinding {
  /// ROS component name that owns the generated interface. 自动生成接口所属的 ROS 组件名。
  std::string component;
  /// Interface suffix relative to the component. 相对于组件的接口后缀。
  std::string interface_name;
  /// Protocol V4 scalar actuator resource name. Protocol V4 标量执行器资源名。
  std::string resource_name;
  /// Offset in the complete generated ROS command array. 完整自动生成 ROS 指令数组中的偏移。
  std::size_t command_index = 0;
  /// Bound offset in the session's dense command array. 会话稠密指令数组中的已绑定偏移。
  std::size_t dense_offset = 0;
  /// Whether the Server supplied an inclusive command range. Server 是否提供了闭区间指令范围。
  bool has_range = false;
  /// Inclusive lower command bound when has_range is true. has_range 为 true 时的闭区间下限。
  double minimum = 0.0;
  /// Inclusive upper command bound when has_range is true. has_range 为 true 时的闭区间上限。
  double maximum = 0.0;

  /** @brief Returns `<component>/<interface>`. 返回 `<component>/<interface>`。 */
  std::string FullName() const;
};

/**
 * @brief Owns the complete generated ROS/Controller registration manifest.
 *        持有完整的自动生成 ROS/Controller 注册清单。
 */
struct InterfaceManifest {
  /// Generated ROS state descriptions in handle order. 按句柄顺序排列的 ROS 状态描述。
  std::vector<hardware_interface::InterfaceDescription> state_descriptions;
  /// Generated ROS command descriptions in handle order. 按句柄顺序排列的 ROS 指令描述。
  std::vector<hardware_interface::InterfaceDescription> command_descriptions;
  /// Unique observation resources in registration order. 按注册顺序排列的唯一观测资源。
  std::vector<std::string> observation_names;
  /// Unique command resources in registration order. 按注册顺序排列的唯一指令资源。
  std::vector<std::string> command_names;
  /// Full ROS state names in handle order. 按句柄顺序排列的完整 ROS 状态名称。
  std::vector<std::string> state_names;
  /// Full ROS command names in handle order. 按句柄顺序排列的完整 ROS 指令名称。
  std::vector<std::string> command_interface_names;
};

/**
 * @brief Coordinates cross-resource interface, resource, and exclusive-topic registration.
 *        协调跨资源接口、Protocol 资源和独占 Topic 注册。
 *
 * Each ResourceInterface submits its own mutable bindings exactly once during on_init(). The
 * registry validates complete resource coverage, assigns global ROS indices, and prevents
 * cross-resource conflicts without exposing mutable bindings to MujocoSimSystem.
 * 每个 ResourceInterface 在 on_init() 期间恰好一次提交自身可变绑定。registry 校验资源完整
 * 覆盖、分配全局 ROS 索引并阻止跨资源冲突，且不向 MujocoSimSystem 暴露可变绑定。
 */
class InterfaceRegistry {
 public:
  InterfaceRegistry() = default;

  /**
   * @brief Registers one resource's complete local contract transactionally.
   *        以事务方式注册一个资源的完整局部契约。
   * @param states Resource-owned state bindings whose indices are assigned on success.
   *               成功时分配索引的资源自有状态绑定。
   * @param commands Resource-owned command bindings whose indices are assigned on success.
   *                 成功时分配索引的资源自有指令绑定。
   * @param exclusive_topics Resource-owned ROS topics that must be globally unique.
   *                         必须全局唯一的资源自有 ROS Topic。
   * @param error Required diagnostic output. 必需的诊断输出。
   */
  bool RegisterResource(std::span<StateBinding> states, std::span<CommandBinding> commands,
                        std::span<const std::string> exclusive_topics, std::string& error);

  /** @brief Moves out the completed manifest after all resources register.
   *         所有资源注册完成后移出完整清单。 */
  InterfaceManifest ReleaseManifest() noexcept;

 private:
  /// Manifest accumulated in deterministic resource registration order.
  /// 按确定性资源注册顺序累积的接口清单。
  InterfaceManifest manifest_;
  /// Globally claimed Protocol observation resource names. 全局已占用的 Protocol 观测资源名。
  std::unordered_set<std::string> observations_;
  /// Globally claimed Protocol command resource names. 全局已占用的 Protocol 指令资源名。
  std::unordered_set<std::string> commands_;
  /// Globally claimed generated ROS state interface names. 全局已占用的 ROS 状态接口名。
  std::unordered_set<std::string> state_interfaces_;
  /// Globally claimed generated ROS command interface names. 全局已占用的 ROS 指令接口名。
  std::unordered_set<std::string> command_interfaces_;
  /// Globally claimed resource-owned exclusive publisher topics.
  /// 全局已占用的资源自有独占 publisher Topic。
  std::unordered_set<std::string> exclusive_topics_;
};

/**
 * @brief Common factory-built mapping contract for joints and sensors.
 *        joint 与 sensor 共用的工厂构建映射契约。
 *
 * Instances are configured before Controller registration, then bound exactly
 * once to an immutable ControllerLayout for each active session. StageState()
 * and StageCommand() borrow caller-owned preallocated buffers and never retain
 * their spans. Publisher side effects occur only through post-commit hooks.
 *
 * 实例在 Controller 注册前完成配置，随后在每个活动会话中恰好一次绑定到不可变的
 * ControllerLayout。StageState() 与 StageCommand() 借用调用方预分配缓冲区，且不会保存
 * span。publisher 副作用仅通过提交后 hook 发生。
 */
class ResourceInterface {
 public:
  /** @brief Creates an empty, unconfigured resource. 创建空的未配置资源。 */
  ResourceInterface() = default;
  /** @brief Enables polymorphic destruction. 支持多态析构。 */
  virtual ~ResourceInterface() = default;

  ResourceInterface(const ResourceInterface&) = delete;
  ResourceInterface& operator=(const ResourceInterface&) = delete;

  /** @brief Returns this ROS component name. 返回该 ROS 组件名称。 */
  virtual const std::string& name() const noexcept = 0;
  /**
   * @brief Registers this resource's interfaces, Protocol resources, and exclusive ROS topics.
   *        注册本资源的接口、Protocol 资源和独占 ROS Topic。
   */
  virtual bool RegisterInterfaces(InterfaceRegistry& registry, std::string& error) = 0;

  /** @brief Returns read-only generated state mappings. 返回只读的自动生成状态映射。 */
  virtual std::span<const StateBinding> state_bindings() const noexcept = 0;
  /** @brief Returns read-only generated command mappings. 返回只读的自动生成指令映射。 */
  virtual std::span<const CommandBinding> command_bindings() const noexcept = 0;

  /**
   * @brief Binds resource names to immutable dense spans and command ranges.
   *        将资源名绑定到不可变稠密区间与指令范围。
   * @param layout Successful SDK registration layout. SDK 成功注册得到的布局。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true if every expected resource has the exact dimension and valid metadata.
   *         所有资源维度和元数据均精确有效时返回 true。
   */
  virtual bool Bind(const adapter_sdk::ControllerLayout& layout, std::string& error) = 0;

  /**
   * @brief Validates and stages this resource's state values transactionally.
   *        以事务方式校验并暂存本资源的状态值。
   * @param observations Complete dense observation array borrowed for this call.
   *                     本次调用期间借用的完整稠密观测数组。
   * @param staged_state Preallocated complete ROS-state staging array.
   *                     预分配的完整 ROS 状态暂存数组。
   * @param error Required diagnostic output. 必需的诊断输出。
   */
  virtual bool StageState(std::span<const double> observations, std::span<double> staged_state,
                          std::string& error) const = 0;

  /**
   * @brief Validates and maps this resource's commands into a dense array.
   *        校验并将本资源指令映射到稠密数组。
   * @param ros_commands Complete generated ROS command staging values. The resource replaces its
   *                     entries with validated and clamped accepted values.
   *                     完整的 ROS 指令暂存值；资源以已校验、限幅的接受值替换自身条目。
   * @param dense_command Preallocated SDK command array. 预分配的 SDK 指令数组。
   * @param error Required diagnostic output. 必需的诊断输出。
   */
  virtual bool StageCommand(std::span<double> ros_commands, std::span<double> dense_command,
                            std::string& error) const = 0;

  /** @brief Creates every resource-owned ROS publisher on the component node.
   *         在组件节点上创建全部资源自有 ROS publisher。 */
  virtual bool ConfigurePublishers(const rclcpp::Node::SharedPtr& node, std::string& error) = 0;
  /** @brief Releases every resource-owned publisher. 释放全部资源自有 publisher。 */
  virtual void ResetPublishers() noexcept = 0;
  /**
   * @brief Best-effort handles one globally committed State without affecting control I/O.
   *        尽力处理一帧全局已提交 State，且不影响控制 I/O。
   */
  virtual void OnStateCommitted(std::span<const double> state_values,
                                const builtin_interfaces::msg::Time& stamp) noexcept = 0;
  /**
   * @brief Best-effort handles one SDK-accepted Command without affecting control I/O.
   *        尽力处理一帧 SDK 已接受 Command，且不影响控制 I/O。
   */
  virtual void OnCommandAccepted(std::span<const double> command_values) noexcept = 0;
};

/**
 * @brief Builds a joint or an explicitly typed sensor mapping from merged parameters.
 *        根据合并后的参数构建 joint 或显式指定类型的 sensor 映射。
 * @param component Parsed ros2_control component. 已解析的 ros2_control 组件。
 * @param merged_parameters Hardware parameters overlaid by component parameters.
 *                          由组件参数覆盖后的硬件参数。
 * @param error Required diagnostic output. 必需的诊断输出。
 * Sensors require `sensor_type=imu` or `sensor_type=other`; the value is never inferred from
 * the remaining parameters. sensor 必须设置 `sensor_type=imu` 或 `sensor_type=other`，类型不会
 * 根据其他参数推断。
 * @return Configured resource, or nullptr with a diagnostic on error.
 *         成功时返回已配置资源；失败时返回 nullptr 并给出诊断。
 */
std::unique_ptr<ResourceInterface> MakeResource(const hardware_interface::ComponentInfo& component,
                                                const ParameterMap& merged_parameters,
                                                std::string& error);

/**
 * @brief Parses and validates hardware-level Controller options.
 *        解析并校验硬件级 Controller 参数。
 */
struct HardwareOptions {
  /// Protocol V4 Server identifier; defaults to `default`. Protocol V4 Server 标识符，默认为
  /// `default`。
  std::string server_id = "default";
  /// Required Protocol V4 Controller identifier. 必需的 Protocol V4 Controller 标识符。
  std::string adapter_id;
  /// Positive SDK discovery/RPC timeout in milliseconds. SDK 发现与 RPC 的正数超时，单位为毫秒。
  std::uint32_t server_timeout_ms = 5000;
  /// Publishes accepted State simulation time on `/clock`. 将已接受 State 的仿真时间发布到
  /// `/clock`。
  bool publish_sim_time = false;
};

/**
 * @brief Parses the strict public hardware parameter contract.
 *        解析严格的公开硬件参数契约。
 * @param parameters Hardware-level parameters. 硬件级参数。
 * @param options Transactional output, changed only on success.
 *                事务性输出，仅成功时修改。
 * @param error Required diagnostic output. 必需的诊断输出。
 */
bool ParseHardwareOptions(const ParameterMap& parameters, HardwareOptions& options,
                          std::string& error);

/**
 * @brief Validates the complete registered layout against requested resources.
 *        根据请求资源校验完整注册布局。
 */
bool ValidateLayout(const adapter_sdk::ControllerLayout& layout,
                    const std::vector<std::unique_ptr<ResourceInterface>>& resources,
                    std::string& error);

}  // namespace mujoco_sim::hardware::internal
