#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace mujoco_sim::hardware::internal {
class ResourceInterface;
class MujocoSimSystemTestPeer;
}  // namespace mujoco_sim::hardware::internal

namespace mujoco_sim::hardware {

/**
 * @brief Bridges one Protocol V4 Controller session into a ros2_control system.
 *        将一个 Protocol V4 Controller 会话桥接为 ros2_control system。
 *
 * The plugin derives its state and command interfaces from resource mapping
 * parameters in the ros2_control description. It owns a single-use
 * ControllerClient only while ACTIVE. The ros2_control framework must quiesce
 * read/write callbacks before lifecycle transitions, as required for ordinary
 * SystemInterface plugins.
 *
 * 本插件根据 ros2_control 描述中的资源映射参数生成状态与指令接口，仅在 ACTIVE
 * 阶段持有一个单次使用的 ControllerClient。与普通 SystemInterface 插件一致，
 * ros2_control 框架必须在生命周期切换前停止 read/write 回调。
 *
 * The class deliberately owns its lifecycle state and SDK objects directly;
 * it does not use an opaque implementation object. Resource-specific behavior remains in
 * the polymorphic ResourceInterface collection.
 * 本类有意直接持有生命周期状态与 SDK 对象，不使用隐藏实现对象；各类资源的
 * 专用行为仍由多态 ResourceInterface 集合承载。
 *
 * @par Thread safety / 线程安全
 * read() and write() belong to the same ros2_control update loop and must be
 * serialized; concurrent calls are unsupported. Lifecycle callbacks are also
 * serialized and must not overlap read/write callbacks. ACTIVE read and write
 * calls are non-blocking apart from short SDK and ros2_control handle critical
 * sections.
 * read() 与 write() 属于同一个 ros2_control 更新循环，必须串行调用，不支持并发。
 * 生命周期函数同样串行执行，且不得与 read/write 回调重叠。除 SDK 与
 * ros2_control 句柄的短临界区外，ACTIVE 阶段的 read/write 调用均不阻塞。
 */
class MujocoSimSystem final : public hardware_interface::SystemInterface {
 public:
  /** @brief Creates an uninitialized plugin. 创建尚未初始化的插件。 */
  MujocoSimSystem();

  /**
   * @brief Stops local communication and performs best-effort remote cleanup.
   *        停止本地通信并尽力清理远端会话。
   */
  ~MujocoSimSystem() override;

  /**
   * @brief Parses hardware/component parameters and declares generated interfaces.
   *        解析硬件与组件参数，并声明自动生成的接口。
   * @param params Parsed ros2_control hardware information and executor handle.
   *               已解析的 ros2_control 硬件信息与执行器句柄。
   * @return SUCCESS for a complete conflict-free configuration, otherwise ERROR.
   *         配置完整且无冲突时返回 SUCCESS，否则返回 ERROR。
   */
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;

  /** @brief Returns generated state-interface descriptions. 返回生成的状态接口描述。 */
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

  /** @brief Returns generated command-interface descriptions. 返回生成的指令接口描述。 */
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override;

  /**
   * @brief Exports generated state handles and caches their validated order.
   *        导出自动生成的状态句柄，并缓存其已校验顺序。
   *
   * The cached handles remove resource-name lookup from read(). Ownership is
   * shared with ros2_control for the hardware component lifetime.
   * 缓存句柄使 read() 无需按资源名查找；其所有权在硬件组件生命周期内与
   * ros2_control 共享。
   */
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
      override;

  /**
   * @brief Exports generated command handles and caches their validated order.
   *        导出自动生成的指令句柄，并缓存其已校验顺序。
   *
   * The cached handles remove resource-name lookup from write(). Ownership is
   * shared with ros2_control for the hardware component lifetime.
   * 缓存句柄使 write() 无需按资源名查找；其所有权在硬件组件生命周期内与
   * ros2_control 共享。
   */
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
      override;

  /** @brief Creates ROS publishers without claiming remote resources.
   *         创建 ROS publisher，但不占用远端资源。 */
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Registers, binds, and activates a fresh Controller session.
   *         注册、绑定并激活一个全新的 Controller 会话。 */
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Stops local communication, then unregisters the active Controller session.
   *         停止本地通信后注销活动 Controller 会话。 */
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Releases publishers and retries session cleanup. 释放 publisher 并重试会话清理。 */
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Performs bounded shutdown cleanup. 执行有界关闭清理。 */
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Performs bounded error-path cleanup. 执行有界错误路径清理。 */
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Non-blockingly projects the latest complete dense State into ROS interfaces.
   *        非阻塞地将最新完整稠密 State 投影到 ROS 接口。
   * @return OK when no new frame exists or a complete frame was applied; ERROR
   *         for terminal SDK or data errors. 没有新帧或完整帧已应用时返回 OK；
   *         SDK 终止错误或数据错误时返回 ERROR。
   */
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  /**
   * @brief Collects, validates, clamps, and atomically submits one dense Command.
   *        收集、校验、限幅并原子提交一帧稠密 Command。
   * @return OK when the SDK accepts the latest-value command; otherwise ERROR.
   *         SDK 接受 latest-value 指令时返回 OK，否则返回 ERROR。
   */
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  friend class internal::MujocoSimSystemTestPeer;

  /** @brief Parses mappings and builds the immutable local interface contract.
   *         解析映射并构建不可变的本地接口契约。 */
  bool Initialize(const hardware_interface::HardwareInfo& info, std::string& error);
  /** @brief Configures resource-owned ROS outputs and the system-owned `/clock` publisher.
   *         配置资源自有 ROS 输出与 system 自有 `/clock` publisher。 */
  bool ConfigureRosOutputs(std::string& error);
  /** @brief Creates, binds, and activates one fresh SDK session.
   *         创建、绑定并激活一个全新的 SDK 会话。 */
  bool Activate(std::string& error);
  /** @brief Stops I/O and unregisters the current session. 停止 I/O 并注销当前会话。 */
  bool Deactivate(std::string& error);
  /** @brief Releases the inactive session and all publishers. 释放非活动会话与全部 publisher。 */
  bool Cleanup(std::string& error);
  /** @brief Performs non-throwing destructor cleanup. 执行不抛异常的析构清理。 */
  void BestEffortCleanup() noexcept;
  /** @brief Executes one active non-blocking State poll. 执行一次活动状态的非阻塞 State 轮询。 */
  hardware_interface::return_type ReadActive();
  /** @brief Executes one active atomic Command submission. 执行一次活动状态的原子 Command 提交。 */
  hardware_interface::return_type WriteActive();

  /**
   * @brief Validates and caches generated state handles in description order.
   *        按描述顺序校验并缓存自动生成的状态句柄。
   * @param handles Handles returned by the Jazzy unlisted-interface API.
   *                Jazzy unlisted-interface API 返回的句柄。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true if every handle exists and preserves the generated order.
   *         所有句柄均存在且保持生成顺序时返回 true。
   */
  bool CacheStateHandles(
      const std::vector<hardware_interface::StateInterface::ConstSharedPtr>& handles,
      std::string& error);
  /**
   * @brief Validates and caches generated command handles in description order.
   *        按描述顺序校验并缓存自动生成的指令句柄。
   * @param handles Handles returned by the Jazzy unlisted-interface API.
   *                Jazzy unlisted-interface API 返回的句柄。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true if every handle exists and preserves the generated order.
   *         所有句柄均存在且保持生成顺序时返回 true。
   */
  bool CacheCommandHandles(
      const std::vector<hardware_interface::CommandInterface::SharedPtr>& handles,
      std::string& error);
  /**
   * @brief Stores one validated complete state frame into cached ROS handles.
   *        将一帧已校验的完整状态写入缓存的 ROS 句柄。
   * @param values Complete generated-interface values borrowed for this call.
   *               本次调用期间借用的完整自动生成接口数值。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true if all typed handles accepted their scalar values.
   *         所有类型化句柄均接受标量值时返回 true。
   */
  bool StoreStateValues(std::span<const double> values, std::string& error);
  /**
   * @brief Loads one complete ROS command frame without resource-name lookup.
   *        不按资源名查找地读取一帧完整 ROS 指令。
   * @param values Preallocated destination whose size equals the command count.
   *               大小等于指令数量的预分配目标区。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true if every typed handle supplied a value. 所有类型化句柄均提供数值时返回 true。
   */
  bool LoadCommandValues(std::span<double> values, std::string& error) const;
  /**
   * @brief Writes accepted, range-clamped commands back to ROS handles.
   *        将已接受且完成范围限幅的指令写回 ROS 句柄。
   * @param values Complete generated command array borrowed for this call.
   *               本次调用期间借用的完整自动生成指令数组。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true if all typed handles accepted their scalar values.
   *         所有类型化句柄均接受标量值时返回 true。
   */
  bool StoreCommandValues(std::span<const double> values, std::string& error);
  /** @brief Sets configured state handles to NaN and command handles to zero.
   *         将已配置状态句柄设为 NaN，并将指令句柄设为零。 */
  bool InitializeHandles(std::string& error);
  /** @brief Applies registered ranges to zero commands after layout binding.
   *         布局绑定后对零指令应用已注册范围。 */
  bool InitializeBoundCommands(std::string& error);
  /**
   * @brief Validates and decodes one complete SDK State into the staging array.
   *        校验一帧完整 SDK State 并解码到暂存数组。
   * @param state Immutable frame lease valid for this call. 本次调用期间有效的不可变帧租约。
   * @param error Required diagnostic output. 必需的诊断输出。
   * @return true only when time, length, and every mapped scalar are valid.
   *         仅当时间、长度和全部映射标量均有效时返回 true。
   */
  bool StageState(const adapter_sdk::StateLease& state, std::string& error);
  /** @brief Returns the bound dense observation scalar count. 返回已绑定稠密观测标量数量。 */
  std::size_t expected_observation_count() const;
  /**
   * @brief Best-effort reports the first local terminal fault in a session.
   *        尽力上报当前会话中的第一个本地终止故障。
   *
   * The serialized control loop guarantees at most one SDK ReportFault attempt
   * per activation without additional synchronization.
   * 串行控制循环无需额外同步，即可保证每次激活最多尝试一次 SDK ReportFault。
   */
  void ReportFault(adapter_sdk::FaultCode code, const std::string& message);
  /** @brief Performs bounded compensating unregister after activation failure.
   *         激活失败后执行有界补偿注销。 */
  void CleanupCandidate(adapter_sdk::ControllerClient& candidate, std::string& primary_error);
  /** @brief Best-effort publishes validated simulation time on `/clock`.
   *         尽力在 `/clock` 发布已校验仿真时间。 */
  void PublishClock(const builtin_interfaces::msg::Time& stamp);
  /** @brief Destroys the clock publisher and asks every resource to reset its ROS outputs.
   *         销毁 clock publisher，并要求每个资源重置其 ROS 输出。 */
  void ResetRosOutputs() noexcept;

  /// Protocol V4 Server identifier used for the next activation.
  /// 下一次激活使用的 Protocol V4 Server 标识符。
  std::string server_id_ = "default";
  /// Protocol V4 Controller identifier required for every activation.
  /// 每次激活必需的 Protocol V4 Controller 标识符。
  std::string adapter_id_;
  /// Bounded discovery and RPC timeout in milliseconds. 有界发现与 RPC 超时，单位为毫秒。
  std::uint32_t server_timeout_ms_ = 5000;
  /// Enables `/clock` publication for each accepted new State. 为每帧已接受新 State 启用 `/clock`。
  bool publish_sim_time_ = false;
  /// Directly owned resource mappings; spans borrowed by calls are never retained.
  /// 直接持有的资源映射；调用期间借用的 span 从不被保存。
  std::vector<std::unique_ptr<internal::ResourceInterface>> resources_;
  /// Automatically generated state-interface descriptions. 自动生成的状态接口描述。
  std::vector<hardware_interface::InterfaceDescription> state_descriptions_;
  /// Automatically generated command-interface descriptions. 自动生成的指令接口描述。
  std::vector<hardware_interface::InterfaceDescription> command_descriptions_;
  /// Unique requested observation names in registration order. 按注册顺序排列的唯一观测资源名。
  std::vector<std::string> observation_names_;
  /// Unique requested command names in registration order. 按注册顺序排列的唯一指令资源名。
  std::vector<std::string> command_names_;
  /// Generated ROS state full names in cached-handle order. 按缓存句柄顺序排列的 ROS 状态全名。
  std::vector<std::string> state_names_;
  /// Generated ROS command full names in cached-handle order. 按缓存句柄顺序排列的 ROS 指令全名。
  std::vector<std::string> command_names_ros_;
  /// Transactional staging array used only by serialized read() calls.
  /// 仅供串行 read() 调用使用的事务状态暂存数组。
  std::vector<double> state_staging_;
  /// ROS command staging array used only by serialized write() calls.
  /// 仅供串行 write() 调用使用的 ROS 指令暂存数组。
  std::vector<double> ros_commands_;
  /// Dense SDK command array used only by serialized write() calls and allocated at activation.
  /// 仅供串行 write() 调用使用并在激活期间分配的 SDK 稠密指令数组。
  std::vector<double> dense_command_;
  /// Cached typed state handles shared with ros2_control for this component lifetime.
  /// 与 ros2_control 共享、在本组件生命周期内有效的类型化状态句柄缓存。
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles_;
  /// Cached typed command handles shared with ros2_control for this component lifetime.
  /// 与 ros2_control 共享、在本组件生命周期内有效的类型化指令句柄缓存。
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  /// Directly owned single-use SDK client; non-null from successful Start until unregister.
  /// 直接持有的单次使用 SDK client；从 Start 成功到注销期间非空。
  std::unique_ptr<adapter_sdk::ControllerClient> client_;
  /// Optional realtime `/clock` publisher owned while configured. 配置期间持有的可选实时 `/clock`
  /// publisher。
  std::unique_ptr<realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>> clock_publisher_;
  /// Serialized I/O gate set only after SDK activation commits.
  /// 仅在 SDK 激活提交后置位的串行 I/O 门闩。
  bool active_ = false;
  /// Last transactionally committed session-local State sequence. 最近事务提交的会话内 State
  /// 序列号。
  std::uint64_t last_sequence_ = 0;
  /// True while ROS publishers and handle initial values are configured.
  /// ROS publisher 与句柄初值配置完成期间为 true。
  bool configured_ = false;
  /// Per-activation gate for the first local fault report. 每次激活首个本地故障上报门闩。
  bool fault_reported_ = false;
};

}  // namespace mujoco_sim::hardware
