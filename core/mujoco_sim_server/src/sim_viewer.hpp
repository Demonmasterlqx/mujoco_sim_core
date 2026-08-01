#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco_sim/core/interactive_input.hpp>

struct mjData_;
struct mjModel_;

namespace mujoco_sim::core {
class SimulationSnapshot;
}  // namespace mujoco_sim::core

namespace mujoco_sim::server {

namespace detail {

/**
 * @brief Describes the MuJoCo storage relevant to viewer snapshot compatibility.
 *
 * Pointer values express ownership isolation only and are never dereferenced by
 * the compatibility check. Buffer sizes are measured in bytes.
 *
 * 描述查看器快照兼容性所需的 MuJoCo 存储元数据。
 *
 * 指针值仅用于表达所有权隔离，兼容性检查不会解引用这些指针。缓冲大小以字节为单位。
 */
struct ViewerSnapshotMetadata {
  /** Model storage identity. model 存储标识。 */
  const void* model_identity = nullptr;
  /** Data storage identity. data 存储标识。 */
  const void* data_identity = nullptr;
  /** MuJoCo model compilation signature. MuJoCo model 编译签名。 */
  std::uint64_t model_signature = 0;
  /** MuJoCo data compilation signature. MuJoCo data 编译签名。 */
  std::uint64_t data_signature = 0;
  /** Model main-buffer size in bytes. model 主缓冲大小（字节）。 */
  std::uint64_t model_buffer_bytes = 0;
  /** Data main-buffer size in bytes. data 主缓冲大小（字节）。 */
  std::uint64_t data_buffer_bytes = 0;
};

/**
 * @brief Validates fixed viewer snapshot ownership and MuJoCo layout compatibility.
 *
 * The display model must be isolated from the shared producer model, all data
 * buffers must be distinct, and every compilation signature and corresponding
 * buffer size must agree. This pure check performs no allocation or locking.
 *
 * 校验固定查看器快照的所有权和 MuJoCo 布局兼容性。
 *
 * 显示 model 必须与生产者共享 model 隔离，所有 data 缓冲必须彼此独立，并且各编译签名
 * 与对应缓冲大小必须一致。本纯函数不分配内存，也不获取锁。
 *
 * @param display Stable display snapshot metadata. 稳定显示快照元数据。
 * @param first First producer snapshot metadata. 第一个生产快照元数据。
 * @param second Second producer snapshot metadata. 第二个生产快照元数据。
 * @return true when the three snapshots can be used together; otherwise false.
 *         三个快照可共同使用时返回 true，否则返回 false。
 */
inline bool ViewerSnapshotsCompatible(const ViewerSnapshotMetadata& display,
                                      const ViewerSnapshotMetadata& first,
                                      const ViewerSnapshotMetadata& second) noexcept {
  return display.model_identity != nullptr && display.data_identity != nullptr &&
         first.model_identity != nullptr && first.data_identity != nullptr &&
         second.model_identity != nullptr && second.data_identity != nullptr &&
         display.model_identity != first.model_identity &&
         first.model_identity == second.model_identity &&
         display.data_identity != first.data_identity &&
         display.data_identity != second.data_identity &&
         first.data_identity != second.data_identity &&
         display.model_signature == first.model_signature &&
         first.model_signature == second.model_signature &&
         display.data_signature == display.model_signature &&
         first.data_signature == first.model_signature &&
         second.data_signature == second.model_signature &&
         display.model_buffer_bytes == first.model_buffer_bytes &&
         first.model_buffer_bytes == second.model_buffer_bytes &&
         display.data_buffer_bytes == first.data_buffer_bytes &&
         first.data_buffer_bytes == second.data_buffer_bytes;
}

/**
 * @brief Extracts finite nonzero wrenches for dynamic bodies from one snapshot.
 *
 * The output is replaced transactionally. World and static bodies are skipped.
 * A non-finite wrench makes the function fail without changing output.
 *
 * 从一个快照中提取动态刚体的有限非零扰动力。
 *
 * 输出以事务方式替换。world body 与静态刚体会被跳过；存在非有限扰动力时函数失败且
 * 不改变输出。
 *
 * @param model Borrowed MuJoCo model. 借用的 MuJoCo model。
 * @param data Borrowed compatible MuJoCo data. 借用的兼容 MuJoCo data。
 * @param output Transactional owned output. 事务性的自有输出。
 * @return true on valid input; otherwise false. 输入有效时返回 true，否则返回 false。
 */
bool ExtractViewerBodyWrenches(const mjModel_* model, const mjData_* data,
                               std::vector<mujoco_sim::core::InteractiveBodyWrench>* output);

}  // namespace detail

/**
 * @brief Owns the passive MuJoCo viewer and its asynchronous snapshot path.
 *
 * The simulation thread is the sole snapshot producer. It may acquire and
 * submit one of two reusable production buffers without waiting. A dedicated
 * worker copies the latest submitted buffer into a lifetime-stable display
 * snapshot before synchronizing the passive viewer. Viewer synchronization is
 * serialized with the complete MuJoCo render call at the first-party platform
 * adapter boundary. RenderLoop() remains a main-thread operation.
 *
 * 拥有 MuJoCo 被动查看器及其异步快照通路。
 *
 * 仿真线程是唯一的快照生产者。它可以在不等待的情况下获取并提交两个可复用生产缓冲
 * 之一。专用 worker 将最新提交的缓冲复制到生命周期稳定的显示快照，然后同步被动查看器。
 * viewer sync 与完整 MuJoCo render 调用会在第一方 platform adapter 边界串行化。
 * RenderLoop() 仍必须在主线程运行。
 *
 * @par Thread safety / 线程安全
 * TryAcquireSnapshot() and TrySubmitSnapshot() are single-producer APIs.
 * RequestClose(), RequestScreenshot(), RequestRunningIndicator(),
 * CompleteSimulationResetAction(), and metric collection may run concurrently.
 * TryConsumeInput() is called only by the simulation producer. Destruction must
 * not overlap RenderLoop().
 * TryAcquireSnapshot() 和 TrySubmitSnapshot() 是单生产者 API。RequestClose()、
 * RequestScreenshot()、RequestRunningIndicator()、CompleteSimulationResetAction() 与指标采集
 * 可以并发执行。TryConsumeInput() 仅由仿真生产者调用。析构不得与 RenderLoop() 重叠。
 */
class SimViewer {
 private:
  /** Owns viewer state, fixed snapshots, mailbox, worker, and metrics.
   * 拥有查看器状态、固定快照、邮箱、worker 和指标。 */
  struct Impl;

 public:
  /**
   * @brief Selects which local GUI values may be forwarded to SimulationCore.
   *
   * Options are immutable after construction. They are still carried by the
   * GUI-disabled stub, which never produces input or actions.
   *
   * 选择允许转发给 SimulationCore 的本地 GUI 输入。
   *
   * 选项在构造后不可变。GUI-disabled stub 仍保留这些选项，但永不产生输入或动作。
   */
  struct InputOptions {
    /** Accept Control-panel actuator edits. 接受 Control 面板的 actuator 编辑。 */
    bool control_enabled = false;
    /** Accept mouse perturbation wrenches. 接受鼠标扰动力。 */
    bool mouse_force_enabled = false;
    /** Allow the local GUI to pause and resume physics stepping.
     * 允许本地 GUI 暂停和恢复物理步进。 */
    bool pause_run_enabled = false;
    /** Allow the local GUI to reset authoritative simulation data.
     * 允许本地 GUI 重置权威仿真数据。 */
    bool reset_enabled = false;
  };

  /**
   * @brief Owns local simulation lifecycle actions extracted from viewer sync.
   *
   * Pause/Run is a latest desired state. Reset is a coalescing edge latch and
   * forms a barrier that removes older and same-sync Control/Wrench input.
   *
   * 持有从 viewer sync 提取的本地仿真生命周期动作。
   *
   * Pause/Run 是 latest 期望状态。Reset 是会合并边沿的 latch，并形成屏障，移除更早以及
   * 同次 sync 的 Control/Wrench 输入。
   */
  struct ViewerSimulationActions {
    /** Latest desired running state; no value means no new selection.
     * 最新期望运行状态；无值表示没有新的选择。 */
    std::optional<bool> running;
    /** Latched Reset edge; repeated unconsumed requests coalesce.
     * 锁存的 Reset 边沿；未消费前的重复请求会合并。 */
    bool reset_requested = false;
  };

  /** Complete mouse-wrench state produced by one successful viewer sync.
   * 一次成功 viewer sync 产生的完整鼠标扰动力状态。 */
  struct ViewerWrenchFrame {
    /** Monotonic viewer-worker sequence. viewer worker 单调递增序列。 */
    std::uint64_t sequence = 0;
    /** Steady-clock generation time. steady clock 生成时间。 */
    std::chrono::steady_clock::time_point generated_at{};
    /** Complete dynamic-body wrench state; empty explicitly clears.
     * 完整动态刚体扰动力状态；为空时表示显式清除。 */
    std::vector<mujoco_sim::core::InteractiveBodyWrench> body_wrenches;
  };

  /**
   * @brief Owns one transactionally consumed viewer input batch.
   *
   * Every vector and optional value owns its payload. The SimulationLoop may
   * reuse the object after `TryConsumeInput()` returns.
   *
   * 持有一个以事务方式消费的 viewer 输入批次。
   *
   * 每个 vector 与 optional 值都自有 payload。`TryConsumeInput()` 返回后，
   * SimulationLoop 可以复用该对象。
   */
  struct ViewerInputBatch {
    /** Last pending value for each changed actuator. 每个已修改 actuator 的最后待处理值。 */
    std::vector<mujoco_sim::core::InteractiveActuatorWrite> actuator_writes;
    /** No value means no new frame; an empty frame means clear.
     * 无值表示没有新帧；空帧表示清除。 */
    std::optional<ViewerWrenchFrame> wrench_frame;
    /** Independently merged lifecycle actions. 独立合并的生命周期动作。 */
    ViewerSimulationActions simulation_actions;
  };

  /**
   * @brief Owns Simulation-panel item predicates for first-party tests.
   *
   * Values are copied while holding the MuJoCo viewer mutex. They are MuJoCo
   * item-state predicates, not booleans, and have no production consumer.
   *
   * 持有供第一方测试使用的 Simulation 面板 item predicate。
   *
   * 各值在持有 MuJoCo viewer mutex 时复制。它们是 MuJoCo item-state predicate，
   * 不是布尔值，且没有生产消费者。
   */
  struct SimulationUiStateForTesting {
    /** Pause/Run item predicate. Pause/Run item predicate。 */
    int run = -1;
    /** Reset item predicate. Reset item predicate。 */
    int reset = -1;
    /** Num threads item predicate. Num threads item predicate。 */
    int num_threads = -1;
    /** Reload item predicate. Reload item predicate。 */
    int reload = -1;
    /** Noise scale item predicate. Noise scale item predicate。 */
    int noise_scale = -1;
    /** Noise rate item predicate. Noise rate item predicate。 */
    int noise_rate = -1;
    /** History scrub item predicate. History scrub item predicate。 */
    int history_scrub = -1;
  };

  /**
   * @brief Interval-local viewer capture and synchronization metrics.
   *
   * Durations are reported in microseconds. An average is zero when the
   * corresponding interval contains no duration samples.
   *
   * 查看器采集和同步的区间指标。
   *
   * 耗时单位为微秒。对应区间没有耗时样本时，平均值为零。
   */
  struct IntervalMetrics {
    /** Capture cadence expirations. 快照采集周期到期次数。 */
    std::uint64_t capture_attempts = 0;
    /** Frames accepted by the mailbox. 邮箱接受的帧数。 */
    std::uint64_t capture_submitted = 0;
    /** Captures skipped or abandoned. 跳过或放弃的采集数。 */
    std::uint64_t capture_skipped = 0;
    /** Older pending frames replaced. 被替换的旧待处理帧数。 */
    std::uint64_t pending_replaced = 0;
    /** Successful passive-viewer syncs. 成功的被动查看器同步数。 */
    std::uint64_t sync_completed = 0;
    /** Worker sync failures. worker 同步失败数。 */
    std::uint64_t sync_failures = 0;
    /** Mean acquired-buffer capture time in microseconds. 已获取缓冲的平均采集耗时（微秒）。 */
    double capture_avg_us = 0.0;
    /** Maximum acquired-buffer capture time in microseconds. 已获取缓冲的最大采集耗时（微秒）。 */
    double capture_max_us = 0.0;
    /** Mean worker sync time in microseconds. worker 平均同步耗时（微秒）。 */
    double sync_avg_us = 0.0;
    /** Maximum worker sync time in microseconds. worker 最大同步耗时（微秒）。 */
    double sync_max_us = 0.0;
  };

  /**
   * @brief Move-only lease for one reusable production snapshot.
   *
   * An unsubmitted lease returns its buffer to the viewer without blocking
   * when destroyed. The lease and referenced snapshot must remain on the
   * simulation producer thread, and their owning SimViewer must outlive every
   * valid lease.
   *
   * 一个可复用生产快照的只移动租约。
   *
   * 未提交的租约在析构时会以非阻塞方式将缓冲归还查看器。租约及其引用的快照必须始终
   * 留在仿真生产线程，并且其所属 SimViewer 的生命周期必须长于所有有效租约。
   */
  class SnapshotLease {
   public:
    /** Creates an empty lease. 创建空租约。 */
    SnapshotLease() noexcept = default;

    /** Returns an unsubmitted buffer to its owner. 将未提交缓冲归还其所有者。 */
    ~SnapshotLease();

    /** Snapshot leases cannot be copied. 快照租约不可复制。 */
    SnapshotLease(const SnapshotLease&) = delete;

    /** Snapshot leases cannot be copy-assigned. 快照租约不可复制赋值。 */
    SnapshotLease& operator=(const SnapshotLease&) = delete;

    /** Transfers ownership of the leased buffer. 转移租用缓冲的所有权。 */
    SnapshotLease(SnapshotLease&& other) noexcept;

    /** Returns the current buffer, then takes ownership from other. 归还当前缓冲，然后从 other
     * 接管所有权。 */
    SnapshotLease& operator=(SnapshotLease&& other) noexcept;

    /**
     * @brief Reports whether this lease owns a snapshot.
     * @return true for a valid lease; otherwise false.
     *         有效租约返回 true，否则返回 false。
     */
    explicit operator bool() const noexcept;

    /**
     * @brief Returns the writable snapshot owned by this lease.
     * @return Borrowed snapshot valid until submission, move, or destruction.
     *         在提交、移动或析构前有效的借用快照。
     * @pre The lease is valid. 租约必须有效。
     */
    mujoco_sim::core::SimulationSnapshot& snapshot() const noexcept;

   private:
    friend class SimViewer;

    /** Creates a lease for one owner-managed fixed buffer. 为 owner 管理的一个固定缓冲创建租约。 */
    SnapshotLease(Impl* owner, std::uint32_t buffer_index,
                  std::chrono::steady_clock::time_point capture_started) noexcept;

    /** Records completion and optionally returns the buffer as skipped.
     * 记录采集完成，并按需以跳过状态归还缓冲。 */
    void Reset(bool skipped) noexcept;

    Impl* owner_ = nullptr;
    std::uint32_t buffer_index_ = 0;
    std::chrono::steady_clock::time_point capture_started_{};
  };

  /** Constructs an uninitialized viewer with input forwarding disabled.
   * 构造禁用输入转发的未初始化查看器。 */
  SimViewer();

  /** Constructs an uninitialized viewer with immutable input options.
   * 使用不可变输入选项构造未初始化的查看器。 */
  explicit SimViewer(InputOptions options);

  /** Requests closure and joins an active sync worker. 请求关闭并 join 活跃同步 worker。 */
  ~SimViewer();

  /** Viewer ownership cannot be copied. 查看器所有权不可复制。 */
  SimViewer(const SimViewer&) = delete;

  /** Viewer ownership cannot be copy-assigned. 查看器所有权不可复制赋值。 */
  SimViewer& operator=(const SimViewer&) = delete;

  /**
   * @brief Initializes GLFW and the passive MuJoCo viewer.
   * @param error_message Receives contextual failure diagnostics.
   *                      接收带上下文的失败诊断。
   * @return true on success; otherwise false. 成功返回 true，否则返回 false。
   * @par Thread safety / 线程安全
   * Call once on the main thread before QueueLoad().
   * 在 QueueLoad() 前由主线程调用一次。
   */
  bool Initialize(std::string* error_message);

  /**
   * @brief Installs the stable display snapshot and two production buffers.
   *
   * All three snapshots must have the same MuJoCo compilation signature and
   * compatible data buffers. The display snapshot must own a model copy
   * independent from the model shared by the two production snapshots,
   * preventing passive-viewer edits from racing with producer capture.
   * Ownership is retained until the viewer is destroyed. This function queues
   * the stable snapshot for the render thread but does not wait for model
   * loading.
   *
   * 安装稳定显示快照和两个生产缓冲。
   *
   * 三个快照必须具有相同的 MuJoCo 编译签名和兼容的数据缓冲。显示快照必须拥有独立于
   * 两个生产快照共享 model 的 model 副本，以防止被动查看器修改与生产者采集发生竞争。
   * 其所有权保持到查看器析构。本函数把稳定快照排队给渲染线程，但不等待模型加载完成。
   *
   * @param display_snapshot Lifetime-stable snapshot borrowed by MuJoCo.
   *                         供 MuJoCo 借用的生命周期稳定快照。
   * @param first_production_snapshot First reusable producer buffer.
   *                                  第一个可复用生产缓冲。
   * @param second_production_snapshot Second reusable producer buffer.
   *                                   第二个可复用生产缓冲。
   * @param displayed_filename Filename shown by the viewer. 查看器显示的文件名。
   * @param error_message Receives contextual failure diagnostics.
   *                      接收带上下文的失败诊断。
   * @return true on success; otherwise false. 成功返回 true，否则返回 false。
   */
  bool QueueLoad(std::unique_ptr<mujoco_sim::core::SimulationSnapshot> display_snapshot,
                 std::unique_ptr<mujoco_sim::core::SimulationSnapshot> first_production_snapshot,
                 std::unique_ptr<mujoco_sim::core::SimulationSnapshot> second_production_snapshot,
                 const std::string& displayed_filename, std::string* error_message);

  /**
   * @brief Tries to acquire a writable production snapshot without waiting.
   * @return A valid lease on success, or an empty lease on lock contention,
   *         buffer exhaustion, or closure.
   *         成功时返回有效租约；锁竞争、缓冲耗尽或已关闭时返回空租约。
   * @par Thread safety / 线程安全
   * Call only from the simulation producer thread. This function never blocks.
   * 仅由仿真生产线程调用。本函数绝不阻塞。
   */
  SnapshotLease TryAcquireSnapshot() noexcept;

  /**
   * @brief Tries to publish a completed snapshot to the depth-one mailbox.
   *
   * A successful submission replaces an older pending frame. On failure the
   * lease is invalidated and its buffer is returned without waiting.
   *
   * 尝试把完成的快照发布到深度为一的邮箱。
   *
   * 成功提交会替换较旧的待处理帧。失败时租约失效，缓冲会被非阻塞归还。
   *
   * @param lease Completed producer lease. 已完成的生产租约。
   * @return true if submitted; otherwise false. 提交成功返回 true，否则返回 false。
   * @par Thread safety / 线程安全
   * Call only from the simulation producer thread. This function never blocks.
   * 仅由仿真生产线程调用。本函数绝不阻塞。
   */
  bool TrySubmitSnapshot(SnapshotLease&& lease) noexcept;

  /**
   * @brief Tries to consume merged GUI input without waiting.
   *
   * On success output is replaced and the consumed values are removed. Lock
   * contention, closure, no input, or allocation failure returns false and
   * retains all pending input.
   *
   * 尝试以非阻塞方式消费合并后的 GUI 输入。
   *
   * 成功时替换输出并移除已消费值。锁竞争、关闭、无输入或分配失败会返回 false，并保留
   * 全部待处理输入。
   *
   * @param output Reusable simulation-thread-owned output. 仿真线程所有的可复用输出。
   * @return true when at least one value was consumed; otherwise false.
   *         消费至少一个值时返回 true，否则返回 false。
   */
  bool TryConsumeInput(ViewerInputBatch* output) noexcept;

  /**
   * @brief Runs the GUI, joins the sync worker, and reports sticky failure.
   * @param error_message Receives a sync-worker or render failure diagnostic.
   *                      接收同步 worker 或渲染失败诊断。
   * @return true after ordinary closure; false after worker failure.
   *         正常关闭后返回 true；worker 失败后返回 false。
   * @par Thread safety / 线程安全
   * Call once on the main thread. The function blocks until the window closes.
   * 由主线程调用一次。本函数会阻塞直到窗口关闭。
   */
  bool RenderLoop(std::string* error_message) noexcept;

  /** Queues a non-blocking screenshot request. 非阻塞排队一次截图请求。 */
  void RequestScreenshot() noexcept;

  /**
   * @brief Requests a non-blocking correction of the GUI Run indicator.
   *
   * The latest request is applied by the sync worker under the MuJoCo viewer
   * mutex. Applying a correction does not emit a new lifecycle action.
   *
   * 非阻塞请求修正 GUI Run 指示器。
   *
   * 最新请求由 sync worker 在 MuJoCo viewer mutex 下应用。修正不会产生新的生命周期动作。
   *
   * @param running true for Run; false for Pause. true 表示 Run，false 表示 Pause。
   * @par Thread safety / 线程安全
   * May be called by SimulationLoop while rendering. 可由 SimulationLoop 在渲染期间调用。
   */
  void RequestRunningIndicator(bool running) noexcept;

  /**
   * @brief Completes one consumed Reset action and advances the viewer input generation.
   *
   * The Reset barrier remains active after `TryConsumeInput()` returns the edge.
   * SimulationLoop calls this after accepting or rejecting that action. The
   * method clears queued Control/Wrench data and pending pre-completion
   * snapshots, invalidates an in-flight old-generation sync, and releases the
   * barrier while preserving the latest pending Pause/Run selection.
   *
   * 完成一个已消费的 Reset 动作，并推进 viewer 输入 generation。
   *
   * `TryConsumeInput()` 返回 Reset 边沿后，Reset 屏障仍保持活跃。SimulationLoop 在接受或
   * 拒绝该动作后调用本函数。本函数清除排队的 Control/Wrench 数据及完成前的待处理快照，
   * 使在途旧 generation sync 失效，并在保留最新待处理 Pause/Run 选择的同时释放屏障。
   *
   * @par Thread safety / 线程安全
   * Called only by SimulationLoop. It may briefly wait for first-party mailbox
   * critical sections but never waits for render, sync, or transport work.
   * 仅由 SimulationLoop 调用。可能短暂等待第一方 mailbox 临界区，但不等待 render、sync
   * 或 transport 工作。
   */
  void CompleteSimulationResetAction() noexcept;

  /**
   * @brief Closes submission, wakes the worker, and requests window exit.
   *
   * This function does not join the worker. RenderLoop() or destruction owns
   * the join operation.
   *
   * 关闭提交入口、唤醒 worker 并请求窗口退出。
   *
   * 本函数不 join worker；join 由 RenderLoop() 或析构负责。
   */
  void RequestClose() noexcept;

  /**
   * @return true after viewer close has been requested; otherwise false.
   *         查看器被请求关闭后返回 true，否则返回 false。
   */
  bool close_requested() const noexcept;

  /**
   * @return true after the render thread has loaded a model; otherwise false.
   *         渲染线程完成模型加载后返回 true，否则返回 false。
   * @par Blocking / 阻塞
   * May briefly wait for the MuJoCo viewer mutex. 可能短暂等待 MuJoCo viewer mutex。
   */
  bool model_loaded() const;

  /**
   * @brief Reads and clears interval viewer metrics without waiting.
   * @return Metrics accumulated since the preceding successful reset. If the
   *         worker is updating sync metrics, sync fields are returned as zero
   *         and retained for the next call.
   *         自上次成功清零后累计的指标。若 worker 正在更新同步指标，则同步字段返回零，
   *         并保留到下次调用。
   * @par Thread safety / 线程安全
   * May run on the simulation thread and never acquires a blocking mutex.
   * 可在仿真线程运行，且绝不获取会阻塞的互斥锁。
   */
  IntervalMetrics GetAndResetIntervalMetrics() noexcept;

  /**
   * @brief Requests a controlled failure at the next worker sync for tests.
   *
   * This internal test seam verifies exception containment and sticky failure
   * propagation. It has no effect until a submitted frame reaches the worker.
   *
   * 请求在下一次 worker 同步时产生受控测试失败。
   *
   * 此内部测试接口用于验证异常隔离和粘滞失败传播。提交帧到达 worker 前不会生效。
   */
  void RequestSyncFailureForTesting() noexcept;

  /**
   * @brief Holds the mailbox mutex for a bounded non-production test interval.
   * @param duration Time to retain the mutex. 保持互斥锁的时间。
   * @param lock_acquired Optional flag set after acquisition. 获取后置位的可选标志。
   * @par Testing / 测试
   * This blocking seam exists only for deterministic try-lock tests and must
   * never be called by Server runtime code.
   * 此阻塞接口仅用于确定性的 try-lock 测试，Server 运行时代码不得调用。
   */
  void HoldMailboxForTesting(std::chrono::milliseconds duration,
                             std::atomic<bool>* lock_acquired) noexcept;

  /**
   * @brief Holds the sync-metrics mutex for a bounded test interval.
   * @param duration Time to retain the mutex. 保持互斥锁的时间。
   * @param lock_acquired Optional flag set after acquisition. 获取后置位的可选标志。
   * @par Testing / 测试
   * Verifies that metric collection uses try-lock and retains a coherent sync
   * record on contention.
   * 验证指标采集使用 try-lock，并在竞争时保留完整同步记录。
   */
  void HoldSyncMetricsForTesting(std::chrono::milliseconds duration,
                                 std::atomic<bool>* lock_acquired) noexcept;

  /**
   * @brief Injects a bounded delay before each worker sync for tests.
   * @param duration Nonnegative delay applied without the mailbox mutex.
   *                 不持有邮箱互斥锁时施加的非负延迟。
   */
  void SetSyncDelayForTesting(std::chrono::milliseconds duration) noexcept;

  /** Sets the passive viewer's Control value under its mutex for tests.
   * 在测试中持有 passive viewer mutex 设置 Control 值。 */
  bool SetControlForTesting(int actuator_id, double value) noexcept;

  /** Requests the passive viewer's Clear all action for tests.
   * 在测试中请求 passive viewer 的 Clear all 操作。 */
  bool RequestClearControlsForTesting() noexcept;

  /**
   * @brief Changes the passive viewer Run selection for extraction tests.
   * @param running true selects Run; false selects Pause. true 选择 Run；false 选择 Pause。
   * @return true after UI setup accepted the change; otherwise false.
   *         UI 配置完成且接受修改时返回 true，否则返回 false。
   * @par Thread safety / 线程安全
   * Acquires the MuJoCo viewer mutex and may run beside RenderLoop().
   * 获取 MuJoCo viewer mutex，可与 RenderLoop() 并发运行。
   */
  bool SetRunningForTesting(bool running) noexcept;

  /**
   * @brief Sets the passive viewer Reset edge for extraction tests.
   * @return true after UI setup accepted the edge; otherwise false.
   *         UI 配置完成且接受边沿时返回 true，否则返回 false。
   * @par Thread safety / 线程安全
   * Acquires the MuJoCo viewer mutex and may run beside RenderLoop().
   * 获取 MuJoCo viewer mutex，可与 RenderLoop() 并发运行。
   */
  bool RequestResetForTesting() noexcept;

  /**
   * @brief Copies configured Simulation-panel predicates for tests.
   * @return Owned values, or no value before UI setup. 自有值；UI 配置前返回无值。
   * @par Thread safety / 线程安全
   * Acquires the MuJoCo viewer mutex and may run beside RenderLoop().
   * 获取 MuJoCo viewer mutex，可与 RenderLoop() 并发运行。
   */
  std::optional<SimulationUiStateForTesting> GetSimulationUiStateForTesting() const;

  /** Publishes a synthetic extracted batch through production merge rules.
   * 通过生产合并规则发布一个合成的已提取批次。 */
  void PublishInputForTesting(const ViewerInputBatch& input) noexcept;

  /** Holds the input mailbox mutex for deterministic non-blocking tests.
   * 为确定性非阻塞测试持有输入邮箱互斥锁。 */
  void HoldInputMailboxForTesting(std::chrono::milliseconds duration,
                                  std::atomic<bool>* lock_acquired) noexcept;

  /**
   * @return true while the worker owns a production buffer for test sync.
   *         worker 为测试同步持有生产缓冲时返回 true。
   */
  bool sync_active_for_testing() const noexcept;

  /**
   * @return Pending production time, or NaN when no frame is pending.
   *         待处理生产快照时间；没有待处理帧时返回 NaN。
   * @par Testing / 测试
   * This accessor may block on the mailbox mutex and is never used by runtime
   * code. 本访问器可能等待邮箱互斥锁，运行时代码不得调用。
   */
  double PendingTimeForTesting() const;

  /**
   * @return Stable display time, or NaN before snapshot configuration.
   *         稳定显示快照时间；配置快照前返回 NaN。
   * @par Testing / 测试
   * This accessor may block on the MuJoCo viewer mutex. 本访问器可能等待 MuJoCo viewer mutex。
   */
  double DisplayTimeForTesting() const;

  /**
   * @return true when the display model is isolated from the producer model.
   *         显示 model 与生产 model 隔离时返回 true。
   */
  bool snapshot_models_isolated_for_testing() const noexcept;

 private:
  /** Lifetime-stable implementation owner. 生命周期稳定的实现所有者。 */
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
