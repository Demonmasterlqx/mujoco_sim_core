#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>

#include "controller_command_inbox.hpp"
#include "ecal_server_transport.hpp"
#include "server_event_queue.hpp"
#include "viewer_input_state.hpp"

namespace spdlog {
class logger;
}

namespace mujoco_sim::server {

class SensorRuntime;
class SimViewer;
class WarningLimiter;

/**
 * @brief Owns the Server's single-writer simulation thread.
 *
 * After Initialize(), only Run() calls SimulationCore. Transport callbacks
 * submit owned events through MakeHandlers(); Sensor and GUI workers consume
 * owned snapshots. Stop requests close inputs and wake workers without calling
 * Core from the requesting thread.
 *
 * 拥有 Server 的单写者仿真线程。
 *
 * Initialize() 完成后，仅 Run() 调用 SimulationCore。传输回调通过 MakeHandlers()
 * 提交自有事件；Sensor 和 GUI worker 只消费自有快照。停止请求会关闭输入并唤醒 worker，
 * 不会在请求线程调用 Core。
 */
class SimulationLoop {
 public:
  /**
   * @brief Constructs an idle loop with borrowed process services.
   * @param config Immutable runtime configuration. 不可变运行配置。
   * @param logger Shared logger kept alive by the loop. 由循环保持存活的共享 logger。
   * @param warning_limiter Borrowed limiter valid through Join(). 在 Join() 前有效的借用限流器。
   * @param transport Borrowed transport valid through Join(). 在 Join() 前有效的借用传输。
   * @param viewer Optional borrowed viewer valid through Join(). 在 Join() 前有效的可选借用查看器。
   */
  SimulationLoop(ServerConfig config, std::shared_ptr<spdlog::logger> logger,
                 WarningLimiter* warning_limiter, EcalServerTransport* transport,
                 SimViewer* viewer);

  /** Requests stop and joins the simulation thread. 请求停止并 join 仿真线程。 */
  ~SimulationLoop();

  /** Loop ownership cannot be copied. 仿真循环所有权不可复制。 */
  SimulationLoop(const SimulationLoop&) = delete;

  /** Loop ownership cannot be copy-assigned. 仿真循环所有权不可复制赋值。 */
  SimulationLoop& operator=(const SimulationLoop&) = delete;

  /**
   * @brief Loads the model and initializes Sensor and optional viewer snapshots.
   * @param error_message Receives contextual initialization failure. 接收上下文初始化错误。
   * @return true on success; otherwise false before any thread starts. 成功返回
   * true；线程启动前失败返回 false。
   */
  bool Initialize(std::string* error_message);

  /**
   * @brief Creates callback handlers that enqueue owned values.
   * @return Handlers borrowing this loop until transport input is stopped.
   *         在传输输入停止前借用本循环的处理器。
   */
  EcalServerTransport::Handlers MakeHandlers();

  /**
   * @brief Starts the single simulation thread.
   * @param error_message Receives thread-start failure. 接收线程启动错误。
   * @return true if started; otherwise false. 启动成功返回 true，否则返回 false。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Requests cooperative shutdown without joining worker threads.
   * @param reason Owned reason retained for the first request. 首次请求保留的自有原因。
   * @par Thread safety / 线程安全
   * May be called concurrently; it does not join or call Core. 可并发调用；不会 join 或调用 Core。
   */
  void RequestStop(std::string reason);
  /**
   * @brief Requests shutdown with a terminal runtime failure.
   *
   * The first stop reason is preserved, while the final process exit code is
   * forced to 2. This function only signals owned workers and never joins them.
   *
   * 请求因终止性运行时故障而关闭。
   *
   * 保留首次停止原因，并将最终进程退出码强制设为 2。本函数只通知自有 worker，
   * 不会 join 它们。
   *
   * @param reason Owned contextual failure reason. 自有的上下文故障原因。
   * @par Thread safety / 线程安全
   * May be called concurrently with Run() and RequestStop(). It does not call
   * SimulationCore.
   * 可与 Run() 和 RequestStop() 并发调用，且不会调用 SimulationCore。
   */
  void RequestFailure(std::string reason);

  /**
   * @brief Joins the simulation thread and returns its final code.
   * @return 0 for ordinary shutdown, 1 for external eCAL termination, or 2 for failure.
   *         正常关闭返回 0，外部 eCAL 终止返回 1，故障返回 2。
   * @par Blocking / 阻塞
   * Blocks until Finish() completes. 阻塞至 Finish() 完成。
   */
  int Join();

 private:
  /** Thread entry containing all post-initialization Core calls. 包含初始化后全部 Core
   * 调用的线程入口。 */
  void Run(std::stop_token) noexcept;

  /** Dispatches one owned queued event. 分派一个自有队列事件。 */
  void HandleEvent(ServerEvent);

  /** Handles a resource query on the simulation thread. 在仿真线程处理资源查询。 */
  void Handle(GetAvailableResourcesEvent);

  /** Handles one registration transaction. 处理一次注册事务。 */
  void Handle(RegisterAdapterEvent);

  /** Handles one activation transaction. 处理一次激活事务。 */
  void Handle(ActivateAdapterEvent);

  /** Handles one unregister transaction. 处理一次注销事务。 */
  void Handle(UnregisterAdapterEvent);

  /** Handles a Sensor Stream query. 处理 Sensor Stream 查询。 */
  void Handle(GetSensorStreamsEvent);

  /** Applies one Observer heartbeat. 应用一次 Observer 心跳。 */
  void Handle(HeartbeatEvent);

  /** Drains depth-one Controller input slots at a Step boundary. 在 Step 边界清空 depth-one
   * Controller 输入槽。 */
  void DrainControllerInputs(std::chrono::steady_clock::time_point);

  /** Drains non-blocking local GUI input into simulation-thread state.
   * 将非阻塞本地 GUI 输入清空到仿真线程状态。 */
  void DrainViewerInputs();

  /**
   * @brief Applies Reset then the latest Pause/Run action on the Core owner thread.
   *
   * Successful Reset clears viewer data, advances the Sensor generation, and
   * rebuilds runtime anchors. A rejected Pause/Run schedules a non-blocking GUI
   * indicator correction. No transport or worker wait occurs here.
   *
   * 在 Core 所有者线程上依次应用 Reset 与最新 Pause/Run 动作。
   *
   * Reset 成功后清除 viewer 数据、推进 Sensor generation 并重建运行锚点。Pause/Run
   * 被拒绝时排队非阻塞 GUI 指示器修正。本函数不等待传输或 worker。
   *
   * @param now Explicit steady-clock action boundary. 显式 steady clock 动作边界。
   */
  void HandleViewerSimulationActions(std::chrono::steady_clock::time_point now);

  /**
   * @brief Rebuilds pacing, viewer, and summary anchors after simulation-time Reset.
   *
   * The next Step and viewer capture become immediately due, while the next
   * summary starts one complete configured period later. Tick and simulation
   * anchors are copied from post-Reset Core state.
   *
   * 在仿真时间 Reset 后重建 pacing、viewer 与 summary 锚点。
   *
   * 下一 Step 与 viewer capture 立即到期；下一次 summary 则在一个完整配置周期后到期。
   * tick 与 simulation 锚点从 Reset 后 Core 状态复制。
   *
   * @param now Explicit steady-clock Reset boundary. 显式 steady clock Reset 边界。
   */
  void ResetRuntimeAnchorsAfterSimulationReset(std::chrono::steady_clock::time_point now);

  /** Applies Core-owned effects in documented order. 按文档顺序应用 Core 自有效果。 */
  void ApplyEffects(mujoco_sim::core::EngineEffects);

  /** Applies one completed-Step output bundle. 应用一个 completed-Step 输出集合。 */
  void ApplyCompletedStep(mujoco_sim::core::StepCompletedEvent);

  /** Encodes and publishes due Controller State frames. 编码并发布到期的 Controller State 帧。 */
  void PublishControllerStates(const std::vector<mujoco_sim::core::ControllerStateFrame>&);

  /** Encodes and publishes due Observer frames. 编码并发布到期的 Observer 帧。 */
  void PublishObservations(const std::vector<mujoco_sim::core::ObservationFrame>&);

  /** Publishes current status; terminal publication is explicit. 发布当前状态；终止发布需显式指定。
   */
  void PublishStatus(bool, bool = false);

  /** Publishes and resets one steady-clock metric interval. 发布并清零一个 steady-clock 指标区间。
   */
  void PublishSummary(std::chrono::steady_clock::time_point, const mujoco_sim::core::CoreState&);

  /** Runs due capture, sampling, status, and summary timers.
   * 执行到期的采集、采样、状态和汇总定时器。 */
  void ProcessTimers(std::chrono::steady_clock::time_point);
  /**
   * @brief Completes every shutdown stage without leaking an exception.
   *
   * Stages are isolated so a Sensor, Core, transport, or logging failure does
   * not prevent the remaining shutdown work. Any stage failure forces exit
   * code 2.
   *
   * 在不泄漏异常的前提下完成全部关闭阶段。
   *
   * 各阶段彼此隔离，因此 Sensor、Core、传输或日志失败不会阻止后续关闭工作；任一
   * 阶段失败都会将退出码强制设为 2。
   *
   * @param code Exit code selected by the run loop. 仿真循环选定的退出码。
   * @param reason Owned terminal reason. 自有的终止原因。
   *
   * @par Thread safety / 线程安全
   * Called only by the simulation thread after input closure.
   * 仅由仿真线程在输入关闭后调用。
   */
  void Finish(int code, std::string reason) noexcept;

  /**
   * @brief Enqueues one RPC event and waits for its owned promise response.
   * @return Protocol response or a queue-closure error. 协议响应或队列关闭错误。
   * @par Locking / 锁
   * No Server mutex is held while waiting. 等待期间不持有 Server 互斥锁。
   */
  template <class Event, class Request>
  typename Event::response_type SubmitRpc(Request);

  /** Minimal transport metadata retained for an active session. 活跃会话保留的最小传输元数据。 */
  struct SessionInfo {
    /** Adapter identifier used for endpoint lookup. 用于端点查找的 Adapter 标识符。 */
    std::string adapter_id;
    /** Registered Adapter role. 已注册的 Adapter 角色。 */
    mujoco_sim::core::AdapterRole role = mujoco_sim::core::ADAPTER_ROLE_UNSPECIFIED;
  };

  /** Immutable runtime configuration. 不可变运行配置。 */
  ServerConfig config_;
  /** Shared logging lifetime. 共享日志生命周期。 */
  std::shared_ptr<spdlog::logger> logger_;
  /** Borrowed warning limiter valid through Join(). 在 Join() 前有效的借用告警限流器。 */
  WarningLimiter* warning_limiter_ = nullptr;
  /** Borrowed transport valid through Join(). 在 Join() 前有效的借用传输。 */
  EcalServerTransport* transport_ = nullptr;
  /** Optional borrowed viewer valid through Join(). 在 Join() 前有效的可选借用查看器。 */
  SimViewer* viewer_ = nullptr;
  /** Simulation-thread-owned live Core. 仿真线程独占的实时 Core。 */
  mujoco_sim::core::SimulationCore core_;
  /** Simulation-thread GUI input merge and timeout state.
   * 仿真线程的 GUI 输入合并与超时状态。 */
  ViewerInputState viewer_input_state_;
  /** Reusable output for non-blocking viewer consumption.
   * 用于非阻塞 viewer 消费的可复用输出。 */
  SimViewer::ViewerInputBatch viewer_input_batch_;
  /** Sensor runtime stopped during Finish(). 在 Finish() 中停止的 Sensor runtime。 */
  std::unique_ptr<SensorRuntime> sensor_runtime_;
  /** Bounded callback-to-loop event queue. 有界的回调到循环事件队列。 */
  ServerEventQueue queue_;
  /** Bounded per-session Controller slots. 有界的逐会话 Controller 槽。 */
  ControllerCommandInbox command_inbox_;
  /** Sole post-initialization Core thread. 初始化后唯一 Core 线程。 */
  std::jthread thread_;
  /** Stop token observed by blocking Sensor fences. 阻塞 Sensor fence 观察的停止 token。 */
  std::stop_token run_stop_token_;
  /** Simulation-thread-only active session metadata. 仅仿真线程访问的活跃会话元数据。 */
  std::unordered_map<std::uint64_t, SessionInfo> sessions_;
  /** Wall-clock pacing period. 墙钟步进周期。 */
  std::chrono::steady_clock::duration step_period_{};
  /** Next pacing deadline. 下一个步进期限。 */
  std::chrono::steady_clock::time_point next_step_{};
  /** Next periodic status deadline. 下一个周期状态期限。 */
  std::chrono::steady_clock::time_point next_status_{};
  /** Earliest lifecycle-status deadline. 最早生命周期状态期限。 */
  std::chrono::steady_clock::time_point next_status_allowed_{};
  /** Next non-blocking GUI capture deadline. 下一个非阻塞 GUI 采集期限。 */
  std::chrono::steady_clock::time_point next_viewer_capture_{};
  /** Next transport metric sampling deadline. 下一个传输指标采样期限。 */
  std::chrono::steady_clock::time_point next_transport_sample_{};
  /** Next summary deadline, or max when disabled. 下一个汇总期限；禁用时为最大值。 */
  std::chrono::steady_clock::time_point next_summary_{};
  /** Wall anchor for summary deltas. 汇总差值的墙钟锚点。 */
  std::chrono::steady_clock::time_point summary_wall_anchor_{};
  /** Simulation-time anchor in seconds. 仿真时间锚点（秒）。 */
  double summary_simulation_anchor_ = 0.0;
  /** Completed-tick anchor. completed-tick 锚点。 */
  std::uint64_t summary_tick_anchor_ = 0;
  /** Latest sampled subscribed endpoint count. 最近采样的已订阅端点数。 */
  std::size_t transport_endpoint_count_ = 0;
  /** Cross-thread idempotent stop flag. 跨线程幂等停止标志。 */
  std::atomic<bool> stop_requested_{false};
  /** Callback-produced interval malformed count. 回调产生的区间畸形消息数。 */
  std::atomic<std::uint64_t> malformed_pending_{0};
  /** Protects the first stop reason. 保护首次停止原因。 */
  mutable std::mutex lifecycle_mutex_;
  /** Owned first terminal reason. 自有的首次终止原因。 */
  std::string stop_reason_ = "external_stop";
  /** Final process code readable after Join(). Join() 后可读的最终进程码。 */
  std::atomic<int> exit_code_{1};
  /** True after all initialization stages succeed. 全部初始化阶段成功后为 true。 */
  bool initialized_ = false;
  /** Simulation-thread-only lifecycle status coalescing flag.
   * 仅仿真线程访问的生命周期状态合并标志。 */
  bool status_dirty_ = true;
};

}  // namespace mujoco_sim::server
