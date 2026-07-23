#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_sim::server {

#ifndef MUJOCO_SIM_HAS_GUI
#define MUJOCO_SIM_HAS_GUI 1
#endif

/** Minimum accepted Sensor Stream rate in hertz. Sensor Stream 接受的最小频率（Hz）。 */
inline constexpr double kMinimumSensorStreamRateHz = 0.000001;

/** Maximum accepted Sensor Stream rate in hertz. Sensor Stream 接受的最大频率（Hz）。 */
inline constexpr double kMaximumSensorStreamRateHz = 1'000'000.0;

/** Maximum bounded Sensor Stream queue depth. Sensor Stream 有界队列的最大深度。 */
inline constexpr std::uint32_t kMaximumSensorStreamQueueDepth = 64;

/**
 * @brief Complete validated configuration for one Server process.
 *
 * Values may originate from defaults, strict YAML, or CLI overrides. Call
 * ValidateServerConfig() before constructing ServerApplication. Relative paths
 * loaded from YAML are resolved by the generated configuration runtime.
 *
 * 一个 Server 进程的完整、可校验配置。
 *
 * 配置值可以来自默认值、严格 YAML 或 CLI 覆盖。构造 ServerApplication 前应调用
 * ValidateServerConfig()。从 YAML 加载的相对路径由生成配置运行时解析。
 *
 * @par Thread safety / 线程安全
 * This aggregate has no internal synchronization. Publish an immutable copy to
 * runtime threads rather than mutating a shared instance.
 * 此聚合类型不提供内部同步。应向运行时线程发布不可变副本，不得并发修改共享实例。
 */
struct ServerConfig {
  /**
   * @brief Complete Controller-specific policy selected by adapter ID.
   *
   * Omitted YAML members use this record's defaults; records do not merge with
   * the Server-wide Controller values.
   *
   * 按 adapter ID 选择的完整 Controller 专用策略。
   *
   * YAML 中省略的成员使用本记录默认值；记录不会与 Server 全局 Controller 配置合并。
   */
  struct ControllerOverride {
    std::string adapter_id;  ///< Target Adapter identifier. 目标 Adapter 标识符。
    std::uint32_t command_timeout_ms = 500;  ///< OFFLINE timeout in ms. 离线超时（毫秒）。
    std::uint32_t command_keepalive_ms = 100;  ///< SDK keepalive in ms. SDK 保活周期（毫秒）。
    std::uint32_t offline_session_expire_ms = 0;  ///< Zero disables expiry. 零表示禁用过期。
    std::uint32_t state_every_n_steps = 1;  ///< Step cadence fallback. 按步数发布的后备周期。
    double state_period_s =
        0.0;  ///< Simulation-time cadence; zero disables. 仿真时间周期；零禁用。
    std::string startup_ctrl_policy =
        "zero";  ///< Pre-command control policy. 首条命令前的控制策略。
    std::string offline_ctrl_policy = "hold_last";  ///< OFFLINE control policy. 离线控制策略。
    std::string error_ctrl_policy = "zero";  ///< ERROR control policy. 错误状态控制策略。
    std::string release_ctrl_policy = "zero";  ///< Unregister control policy. 注销控制策略。
  };

  /**
   * @brief Configuration for one asynchronous Sensor Stream.
   *
   * Dimensions and ranges are interpreted according to kind. Queue behavior is
   * explicit and bounded; blocking is allowed only for the blocking mode.
   *
   * 一个异步 Sensor Stream 的配置。
   *
   * 尺寸和量程按 kind 解释。队列行为显式且有界；仅 blocking 模式允许阻塞。
   */
  struct SensorStream {
    std::string name;           ///< Unique stream identifier. 唯一流标识符。
    std::string kind = "rgb";   ///< Stream payload kind. 流负载类型。
    std::string source;         ///< MuJoCo camera or sensor source. MuJoCo 相机或传感器源。
    std::string frame_id;       ///< Coordinate-frame identifier. 坐标系标识符。
    double rate_hz = 10.0;      ///< Capture cadence in hertz. 采集频率（Hz）。
    std::uint32_t width = 320;  ///< Camera width in pixels. 相机宽度（像素）。
    std::uint32_t height = 240;  ///< Camera height in pixels. 相机高度（像素）。
    std::uint32_t horizontal_samples = 720;  ///< Horizontal LiDAR samples. LiDAR 水平采样数。
    std::uint32_t vertical_samples = 16;     ///< Vertical LiDAR samples. LiDAR 垂直采样数。
    double vertical_fov_min_deg = -15.0;  ///< Minimum vertical angle in degrees. 最小垂直角（度）。
    double vertical_fov_max_deg = 15.0;  ///< Maximum vertical angle in degrees. 最大垂直角（度）。
    double min_range_m = 0.15;  ///< Minimum valid range in metres. 最小有效量程（米）。
    double max_range_m = 15.0;  ///< Maximum valid range in metres. 最大有效量程（米）。
    std::string queue_mode = "latest";  ///< latest, fifo, or blocking. latest、fifo 或 blocking。
    std::uint32_t queue_depth = 1;  ///< Bounded capture queue depth. 有界采集队列深度。
    std::string fifo_overflow = "drop_oldest";  ///< FIFO overflow policy. FIFO 溢出策略。
  };

  std::string server_name = "default";  ///< Protocol namespace. 协议命名空间。
  std::string model_path;               ///< MuJoCo MJCF path. MuJoCo MJCF 路径。
  std::uint64_t max_ticks =
      0;  ///< Successful stop tick; zero is unlimited. 成功停止 tick；零为无限。
  std::string pacing_mode = "realtime";  ///< realtime or unthrottled. realtime 或 unthrottled。
  double realtime_factor = 1.0;  ///< Simulation/wall-time target ratio. 仿真时间与墙钟时间目标比。
  std::uint32_t command_timeout_ms =
      500;  ///< Default Controller timeout in ms. 默认控制器超时（毫秒）。
  std::uint32_t command_keepalive_ms =
      100;  ///< Default SDK keepalive in ms. 默认 SDK 保活周期（毫秒）。
  std::uint32_t offline_session_expire_ms =
      0;  ///< Default OFFLINE expiry; zero disables. 默认离线过期；零禁用。
  std::uint32_t controller_state_every_n_steps =
      1;  ///< Default State step cadence. 默认 State 步数周期。
  double controller_state_period_s = 0.0;  ///< Default simulation-time cadence. 默认仿真时间周期。
  double controller_state_max_rate_hz =
      1000.0;  ///< State rate ceiling in hertz. State 频率上限（Hz）。
  std::string startup_ctrl_policy = "zero";  ///< Default startup control policy. 默认启动控制策略。
  std::string offline_ctrl_policy = "hold_last";  ///< Default OFFLINE policy. 默认离线控制策略。
  std::string error_ctrl_policy = "zero";  ///< Default ERROR policy. 默认错误控制策略。
  std::string release_ctrl_policy = "zero";  ///< Default unregister policy. 默认注销控制策略。
  std::vector<ControllerOverride>
      controller_overrides;  ///< Unique per-Adapter records. 唯一的逐 Adapter 记录。
  std::uint32_t session_timeout_ms =
      1000;  ///< Adapter presence timeout in ms. Adapter 在线超时（毫秒）。
  std::uint32_t observer_heartbeat_period_ms =
      250;  ///< Granted Observer keepalive in ms. Observer 保活周期（毫秒）。
  std::uint32_t status_period_ms =
      100;  ///< Periodic status cadence in ms. 周期状态发布间隔（毫秒）。
  std::uint32_t status_min_interval_ms =
      20;  ///< Status coalescing floor in ms. 状态合并最小间隔（毫秒）。
  double observer_max_rate_hz =
      500.0;  ///< Observer delivery ceiling in hertz. Observer 投递频率上限（Hz）。
  bool allow_every_step_observer =
      false;  ///< Allow every-Step Observer delivery. 是否允许每步投递 Observer。
  std::uint32_t transport_subscriber_sample_period_ms =
      1000;  ///< Transport sampling cadence in ms. 传输采样周期（毫秒）。
  std::uint32_t sensor_subscriber_sample_period_ms =
      200;  ///< Sensor sampling cadence in ms. Sensor 订阅采样周期（毫秒）。
  bool headless = !MUJOCO_SIM_HAS_GUI;  ///< Disable the graphical viewer. 是否禁用图形查看器。
  std::uint32_t viewer_sync_period_ms =
      16;  ///< Graphical capture cadence in ms in [1, UINT32_MAX]; ignored headless.
           ///< 图形采集周期（毫秒），范围 [1, UINT32_MAX]；headless 时忽略。
  std::string initial_keyframe;  ///< Named initial MuJoCo keyframe. MuJoCo 初始关键帧名称。
  std::string log_level = "info";  ///< Minimum logging severity. 最低日志级别。
  std::string log_file;            ///< Optional rotating log path. 可选轮转日志路径。
  std::uint32_t log_file_max_size_mb = 10;  ///< Per-file limit in MiB. 单文件上限（MiB）。
  std::uint32_t log_file_max_files = 5;     ///< Rotating file count. 轮转文件数量。
  std::uint32_t log_summary_period_ms =
      1000;  ///< Summary cadence; zero disables. 汇总周期；零禁用。
  std::uint32_t activation_timeout_ms =
      5000;  ///< Provisional activation timeout in ms. 临时会话激活超时（毫秒）。
  std::string render_backend = "auto";  ///< auto, egl, or osmesa. auto、egl 或 osmesa。
  std::vector<SensorStream> sensor_streams;  ///< Configured asynchronous streams. 已配置的异步流。
};

/**
 * @brief Applies one strict YAML file to an existing configuration.
 * @param path YAML file path. YAML 文件路径。
 * @param config Input defaults and transactional output. 输入默认值及事务性输出。
 * @param error_message Receives a contextual parser diagnostic. 接收带上下文的解析诊断。
 * @return true on success; false with config unchanged on failure.
 *         成功返回 true；失败返回 false 且 config 保持不变。
 * @par Thread safety / 线程安全
 * The pointed objects must not be accessed concurrently. 指向的对象不得被并发访问。
 */
bool LoadConfigFile(const std::string& path, ServerConfig* config, std::string* error_message);

/**
 * @brief Parses CLI arguments, including YAML loading and CLI overrides.
 * @param argc Argument count. 参数数量。
 * @param argv Argument vector borrowed for this call. 本次调用期间借用的参数向量。
 * @param config Input defaults and transactional output. 输入默认值及事务性输出。
 * @param error_message Receives usage text or a contextual diagnostic.
 *                      接收帮助文本或带上下文的诊断。
 * @return true for a valid complete configuration; otherwise false.
 *         配置完整有效时返回 true，否则返回 false。
 * @par Thread safety / 线程安全
 * The pointed objects must not be accessed concurrently. 指向的对象不得被并发访问。
 */
bool ParseCommandLine(int argc, char** argv, ServerConfig* config, std::string* error_message);

/**
 * @brief Validates cross-field and build-feature constraints.
 * @param config Configuration to validate. 待校验配置。
 * @param error_message Receives a contextual validation diagnostic.
 *                      接收带上下文的校验诊断。
 * @return true when valid; otherwise false. 有效时返回 true，否则返回 false。
 */
bool ValidateServerConfig(const ServerConfig& config, std::string* error_message);

/**
 * @brief Returns generated command-line usage text.
 * @return Owned help text for mujoco-sim-server. mujoco-sim-server 的自有帮助文本。
 */
std::string Usage();

}  // namespace mujoco_sim::server
