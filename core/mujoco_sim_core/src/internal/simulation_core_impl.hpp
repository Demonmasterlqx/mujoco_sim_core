#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>

#include "model_runtime.hpp"
#include "resource_catalog.hpp"
#include "snapshot_manager.hpp"

namespace mujoco_sim::core {

/**
 * @brief Private mutable state owned exclusively by SimulationCore.
 *
 * The simulation-loop owner thread is the only permitted runtime caller. This
 * type is source-private and exists in a header only so lifecycle actions can
 * be implemented in a responsibility-focused translation unit.
 *
 * SimulationCore 独占的私有可变状态。
 *
 * 运行期仅允许仿真循环所有者线程调用。该类型仅在源码内可见；放入头文件只是为了让
 * 生命周期动作可以在职责独立的翻译单元中实现。
 *
 * @par Ownership and lifetime / 所有权与生命周期
 * The enclosing SimulationCore owns this object and every record and component
 * reachable from it. Returned frame values own their payload; helper inputs are
 * borrowed only for the duration of each call.
 * 外层 SimulationCore 拥有本对象以及可从本对象访问的全部记录和组件。返回的帧值自有
 * payload；helper 输入仅在各次调用期间被借用。
 *
 * @par Thread safety / 线程安全
 * No member provides internal synchronization. After initialization every
 * method and mutable field is confined to the SimulationCore owner thread.
 * 成员不提供内部同步。初始化完成后，全部方法与可变字段都限制在 SimulationCore
 * 所有者线程使用。
 */
class SimulationCore::Impl {
 public:
  /**
   * @brief Owns one Controller session's lifecycle, command, policy, and cadence state.
   *
   * Reset preserves identity, layout, ownership, and sequence fields while
   * clearing `last_command`; ONLINE transitions to ACTIVE_NO_COMMAND.
   *
   * 持有一个 Controller session 的生命周期、command、策略与节拍状态。
   *
   * Reset 保留身份、layout、所有权和 sequence 字段并清除 `last_command`；ONLINE 会
   * 转换为 ACTIVE_NO_COMMAND。
   */
  struct Controller {
    /** Stable Adapter ID owning this record. 持有本记录的稳定 Adapter ID。 */
    std::string id;
    /** Nonzero session identity within `epoch`. `epoch` 内的非零 session 标识。 */
    std::uint64_t session = 0;
    /** Current Controller lifecycle state. 当前 Controller 生命周期状态。 */
    AdapterState state = PROVISIONAL;
    /** Immutable dense resource layout owned by this session. 本 session 自有的不可变稠密资源布局。
     */
    ResourceLayout layout;
    /** Complete granted runtime policy. 完整的已授予运行策略。 */
    ControllerRuntimePolicy policy;
    /** Steady-clock registration time. steady clock 注册时间。 */
    std::chrono::steady_clock::time_point registered_at{};
    /** Steady-clock activation time. steady clock 激活时间。 */
    std::chrono::steady_clock::time_point activated_at{};
    /** Time of the last accepted Command or Reset liveness anchor.
     * 最后接受 Command 的时间或 Reset 存活性锚点。 */
    std::chrono::steady_clock::time_point last_command_at{};
    /** Original steady-clock entry time for the current OFFLINE episode.
     * 当前 OFFLINE episode 的原始 steady clock 进入时间。 */
    std::chrono::steady_clock::time_point offline_at{};
    /** Completed-step tick anchoring State cadence. 锚定 State 节拍的 completed-step tick。 */
    std::uint64_t activation_tick = 0;
    /** Tick at which the current AdapterState began. 当前 AdapterState 开始时的 tick。 */
    std::uint64_t state_since_tick = 0;
    /** Last emitted Controller State sequence. 最后产生的 Controller State sequence。 */
    std::uint64_t state_sequence = 0;
    /** Last accepted wire Command sequence; Reset never rolls it back.
     * 最后接受的 wire Command sequence；Reset 不会回退该值。 */
    std::uint64_t last_command_sequence = 0;
    /** Cumulative command timeout count. 累计 command 超时次数。 */
    std::uint64_t timeout_count = 0;
    /** Next simulation-time cadence multiplier, starting at one.
     * 下一个仿真时间节拍倍数，初值为一。 */
    std::uint64_t next_period = 1;
    /** Dense latest accepted Command, empty while waiting or after Reset.
     * 最新接受的稠密 Command；等待期间或 Reset 后为空。 */
    std::vector<double> last_command;
    /** Whether `last_command` must be written at the next Step. 下一 Step 是否必须写入
     * `last_command`。 */
    bool command_dirty = false;
    /** Policy to resolve into writes at the next Step boundary. 下一 Step 边界待解析为写入的策略。
     */
    std::optional<ControlPolicy> pending_policy;
    /** Owned lifecycle diagnostic published in status. 在 status 中发布的自有生命周期诊断。 */
    std::string message;
  };

  /**
   * @brief Owns one Observer session's heartbeat and delivery state.
   *
   * Reset preserves identity, heartbeat, lifecycle state, and sequence while
   * rebuilding cadence so the first completed post-Reset Step is due.
   *
   * 持有一个 Observer session 的心跳与投递状态。
   *
   * Reset 保留身份、心跳、生命周期状态和 sequence，并重建节拍，使 Reset 后第一个
   * completed Step 到期投递。
   */
  struct Observer {
    /** Stable Adapter ID owning this record. 持有本记录的稳定 Adapter ID。 */
    std::string id;
    /** Nonzero session identity within `epoch`. `epoch` 内的非零 session 标识。 */
    std::uint64_t session = 0;
    /** Current Observer lifecycle state. 当前 Observer 生命周期状态。 */
    AdapterState state = PROVISIONAL;
    /** Immutable dense observation layout. 不可变稠密 observation 布局。 */
    ResourceLayout layout;
    /** Granted completed-step or simulation-time delivery cadence.
     * 已授予的 completed-step 或仿真时间投递节拍。 */
    ObservationDelivery delivery;
    /** Heartbeat timeout in steady-clock milliseconds. steady clock 心跳超时（毫秒）。 */
    std::uint32_t timeout_ms = 1000;
    /** Steady-clock registration time. steady clock 注册时间。 */
    std::chrono::steady_clock::time_point registered_at{};
    /** Steady-clock time of the last accepted heartbeat. 最后接受心跳的 steady clock 时间。 */
    std::chrono::steady_clock::time_point last_heartbeat_at{};
    /** Completed-step tick anchoring delivery cadence. 锚定投递节拍的 completed-step tick。 */
    std::uint64_t activation_tick = 0;
    /** Tick at which the current AdapterState began. 当前 AdapterState 开始时的 tick。 */
    std::uint64_t state_since_tick = 0;
    /** Last emitted Observation sequence. 最后产生的 Observation sequence。 */
    std::uint64_t sequence = 0;
    /** Cumulative heartbeat timeout count. 累计心跳超时次数。 */
    std::uint64_t timeout_count = 0;
    /** Next simulation-time cadence multiplier. 下一个仿真时间节拍倍数。 */
    std::uint64_t next_period = 1;
    /** Last completed-step tick delivered, cleared by Reset. 最后投递的 completed-step tick；Reset
     * 时清除。 */
    std::optional<std::uint64_t> last_delivery_tick;
    /** Owned lifecycle diagnostic published in status. 在 status 中发布的自有生命周期诊断。 */
    std::string message;
  };

  /**
   * @brief Creates empty Core-owned runtime components from immutable options.
   * @param value Options moved into this implementation. 移入本实现的选项。
   */
  explicit Impl(CoreOptions value);

  /**
   * @brief Builds and validates one immutable Adapter layout.
   * @param request Borrowed registration request. 借用的注册请求。
   * @param code Non-null result code output. 非空结果码输出。
   * @param message Non-null owned diagnostic output. 非空自有诊断输出。
   * @return Owned layout; empty on failure. 自有 layout；失败时为空。
   */
  ResourceLayout MakeLayout(const RegisterAdapterRequest& request, ErrorCode* code,
                            std::string* message) const;

  /**
   * @brief Reads dense observations for one frozen layout from live MuJoCo data.
   * @param layout Borrowed layout belonging to the current model epoch.
   *               属于当前 model epoch 的借用 layout。
   * @return Owned scalar vector in frozen layout order. 按冻结布局顺序排列的自有标量向量。
   */
  std::vector<double> Read(const ResourceLayout& layout) const;

  /**
   * @brief Reports whether a delivery cadence is due at `current_tick`.
   * @param delivery Borrowed granted cadence. 借用的已授予节拍。
   * @param activation_tick Completed-step cadence anchor. completed-step 节拍锚点。
   * @param next_period Next simulation-time multiplier. 下一个仿真时间倍数。
   * @return true when a frame is due. 帧到期时返回 true。
   */
  bool Due(const ObservationDelivery& delivery, std::uint64_t activation_tick,
           std::uint64_t next_period) const;

  /**
   * @brief Clamps a requested Controller cadence to the configured maximum rate.
   * @param delivery Requested cadence copied by value. 按值复制的请求节拍。
   * @return Effective owned cadence. 自有的有效节拍。
   */
  ObservationDelivery EffectiveDelivery(ObservationDelivery delivery) const;

  /**
   * @brief Resolves the complete global or per-Adapter Controller policy.
   * @param adapter_id Borrowed Adapter ID. 借用的 Adapter ID。
   * @return Owned effective policy. 自有的有效策略。
   */
  ControllerRuntimePolicy PolicyFor(std::string_view adapter_id) const;

  /**
   * @brief Queues zero writes for a Controller policy when required.
   * @param controller Borrowed record with a valid current-epoch layout.
   *                   具有有效当前 epoch layout 的借用记录。
   * @param policy Policy to resolve; hold-last produces no writes.
   *               待解析策略；hold-last 不产生写入。
   */
  void QueuePolicy(const Controller& controller, ControlPolicy policy);

  /**
   * @brief Validates command finiteness and registered actuator ranges.
   * @param controller Borrowed target record. 借用的目标记录。
   * @param command Borrowed dense Command. 借用的稠密 Command。
   * @return true only when every scalar is valid. 仅全部标量有效时返回 true。
   */
  bool CommandValuesValid(const Controller& controller, const ControllerCommand& command) const;

  /**
   * @brief Applies pending policy then Controller writes at one Step boundary.
   *
   * Consumes pending policies and dirty command flags and writes only live
   * authoritative MuJoCo data. It does not execute `mj_step()`.
   *
   * 在一个 Step 边界依次应用待处理策略与 Controller 写入。
   *
   * 消费待处理策略与 dirty command 标志，并且只写实时权威 MuJoCo data；不执行
   * `mj_step()`。
   */
  void ApplyControllerWrites();

  /**
   * @brief Builds due Controller State frames after a completed Step.
   * @param time Completed-Step simulation time in seconds. completed Step 仿真时间（秒）。
   * @return Owned due frames with monotonic per-session sequences.
   *         带有逐 session 单调 sequence 的自有到期帧。
   */
  std::vector<ControllerStateFrame> BuildControllerStates(double time);

  /**
   * @brief Builds due Observer frames after a completed Step.
   * @param time Completed-Step simulation time in seconds. completed Step 仿真时间（秒）。
   * @return Owned due frames with monotonic per-session sequences.
   *         带有逐 session 单调 sequence 的自有到期帧。
   */
  std::vector<ObservationFrame> BuildObserverFrames(double time);

  /**
   * @brief Applies Controller steady-clock deadlines and records expired IDs.
   * @param now Explicit steady-clock evaluation time. 显式 steady clock 评估时间。
   * @param effects Non-null owned effect accumulator. 非空自有效果累加器。
   * @param erase Non-null output IDs to unregister after iteration.
   *              非空输出；记录迭代后待注销 ID。
   */
  void CheckControllerDeadlines(std::chrono::steady_clock::time_point now, EngineEffects* effects,
                                std::vector<std::string>* erase);

  /**
   * @brief Applies Observer steady-clock deadlines and records expired IDs.
   * @param now Explicit steady-clock evaluation time. 显式 steady clock 评估时间。
   * @param effects Non-null owned effect accumulator. 非空自有效果累加器。
   * @param erase Non-null output IDs to unregister after iteration.
   *              非空输出；记录迭代后待注销 ID。
   */
  void CheckObserverDeadlines(std::chrono::steady_clock::time_point now, EngineEffects* effects,
                              std::vector<std::string>* erase);

  /**
   * @brief Moves an Observer into OFFLINE after a heartbeat timeout.
   *
   * Updates only the record-local lifecycle state, tick anchor, cumulative
   * timeout count, and owned diagnostic. The caller records transport-neutral
   * effects after this transition.
   *
   * Observer 心跳超时后将其转入 OFFLINE。
   *
   * 仅更新记录内的生命周期状态、tick 锚点、累计超时次数和自有诊断；调用者在转换后记录
   * 与传输无关的 effects。
   *
   * @param observer Mutable record owned by this implementation.
   *                 本实现持有的可变记录。
   */
  void MarkObserverTimedOut(Observer* observer);

  /**
   * @brief Registers or atomically replaces an eligible Controller.
   * @param request Borrowed validated role request. 借用的已校验角色请求。
   * @param layout Owned validated layout moved into the record. 移入记录的自有已校验 layout。
   * @param now Explicit steady-clock registration time. 显式 steady clock 注册时间。
   * @return Owned response and lifecycle effects. 自有响应与生命周期效果。
   */
  EngineResult<RegisterAdapterResponse> RegisterController(
      const RegisterAdapterRequest& request, ResourceLayout layout,
      std::chrono::steady_clock::time_point now);

  /**
   * @brief Registers an Observer with immutable layout and cadence.
   * @param request Borrowed validated role request. 借用的已校验角色请求。
   * @param layout Owned validated layout moved into the record. 移入记录的自有已校验 layout。
   * @param now Explicit steady-clock registration time. 显式 steady clock 注册时间。
   * @return Owned response and lifecycle effects. 自有响应与生命周期效果。
   */
  EngineResult<RegisterAdapterResponse> RegisterObserver(const RegisterAdapterRequest& request,
                                                         ResourceLayout layout,
                                                         std::chrono::steady_clock::time_point now);

  /** Immutable runtime options copied at construction. 构造时复制的不可变运行选项。 */
  CoreOptions options;
  /** Owned authoritative MuJoCo runtime. 自有的权威 MuJoCo 运行时。 */
  std::unique_ptr<internal::ModelRuntime> model;
  /** Owned current-epoch resource catalog. 自有的当前 epoch 资源目录。 */
  std::unique_ptr<internal::ResourceCatalog> catalog;
  /** Owned isolated snapshot factory. 自有的隔离快照工厂。 */
  std::unique_ptr<detail::SnapshotManager> snapshots;
  /** Controller records keyed by stable Adapter ID. 按稳定 Adapter ID 索引的 Controller 记录。 */
  std::unordered_map<std::string, Controller> controllers;
  /** Observer records keyed by stable Adapter ID. 按稳定 Adapter ID 索引的 Observer 记录。 */
  std::unordered_map<std::string, Observer> observers;
  /** Session-to-role/ID index for the current epoch. 当前 epoch 的 session 到角色/ID 索引。 */
  std::unordered_map<std::uint64_t, std::pair<AdapterRole, std::string>> sessions;
  /** Exclusive command-resource owner ID by canonical resource name.
   * 按规范资源名索引的独占 command 资源 owner ID。 */
  std::unordered_map<std::string, std::string> owners;
  /** Validated zero-policy writes pending at the next Step. 下一 Step 待处理的已校验零策略写入。 */
  std::vector<internal::ValidatedActuatorWrite> queued_policy_writes;
  /** Next nonzero session ID. 下一个非零 session ID。 */
  std::uint64_t next_session = 1;
  /** Current model epoch, incremented only by successful model load. 当前 model
   * epoch，仅成功加载时递增。 */
  std::uint64_t epoch = 1;
  /** Monotonic completed-step tick, preserved by Reset. Reset 保留的单调 completed-step tick。 */
  std::uint64_t current_tick = 0;
  /** Authoritative Server lifecycle state. 权威 Server 生命周期状态。 */
  ServerState server_state = IDLE;
  /** Owned global lifecycle diagnostic. 自有全局生命周期诊断。 */
  std::string status_message;
};

}  // namespace mujoco_sim::core
