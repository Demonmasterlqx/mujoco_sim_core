#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/protocol/stream_frame.hpp>

#include "adapter_transport.hpp"
#include "blocking_mailbox.hpp"
#include "callback_gate.hpp"
#include "lifecycle_operation.hpp"

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Implements one single-use multi-stream Sensor consumer.
 *        实现一个单次使用的多数据流 Sensor 消费端。
 *
 * lifecycle_ serializes startup/teardown admission without retaining its lock
 * during transport or callback work. mutex_ protects phase, per-stream state,
 * endpoint maps, and status sequence. CallbackGate prevents use-after-free and
 * permits callback-initiated destruction. No project lock is held while
 * invoking a user callback, eCAL teardown, or callback drain wait.
 * lifecycle_ 在传输或回调工作期间不持锁地串行化启动/拆除准入。mutex_ 保护阶段、每流
 * 状态、端点映射与状态序列。CallbackGate 防止释放后使用，并允许回调内发起析构。调用
 * 用户回调、执行 eCAL 拆除或等待回调退出时不持有项目锁。
 */
class SensorStreamClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  /** @brief Creates a non-started implementation and owns its options.
   *         创建未启动实现并持有其配置。
   * @param options Immutable discovery/buffering options. 不可变发现/缓冲配置。
   */
  explicit Impl(SensorStreamOptions options);
  /** @brief Stops delivery, drains callbacks, and releases endpoints/runtime.
   *         停止投递、清空回调并释放端点/运行时。
   */
  ~Impl();
  /** @brief Implements discovery, validation, endpoint creation, and local commit.
   *         实现发现、校验、端点创建与本地提交。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return true when all requested streams are active. 所有请求数据流活动时为 true。
   */
  bool Start(std::string* error_message);
  /** @brief Selects callback mode for one stream.
   *         为一个数据流选择回调模式。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @param callback Non-empty callback moved into protected stream state.
   * 移入受保护流状态的非空回调。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return true after callback mode commits. 回调模式提交后为 true。
   */
  bool SetCallback(const std::string& stream_name, StreamCallback callback,
                   std::string* error_message);
  /** @brief Implements transactional single-waiter sample delivery per stream.
   *         实现每数据流事务性的单等待方样本投递。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @param timeout Non-negative wait duration. 非负等待时长。
   * @param sample Required output modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required diagnostic cleared on valid entry. 必需诊断；有效入口先清空。
   * @return Public wait outcome. 公开等待结果。
   */
  SampleWaitResult WaitForSample(const std::string& stream_name, std::chrono::milliseconds timeout,
                                 OwnedStreamSample* sample, std::string* error_message);
  /** @brief Publishes idempotent cancellation and wakes every stream waiter.
   *         发布幂等取消状态并唤醒所有数据流等待方。
   */
  void Stop();
  /** @brief Copies one synchronized descriptor.
   *         复制一个同步后的描述符。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @return Owned descriptor or std::nullopt. 自有描述符或 std::nullopt。
   */
  std::optional<mujoco_sim::v4::StreamDescriptor> descriptor(const std::string& stream_name) const;
  /** @brief Returns synchronized drop accounting.
   *         返回同步后的丢帧计数。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @return Accumulated count, or zero for an unknown stream. 累计数量；未知流为零。
   */
  std::uint64_t dropped_samples(const std::string& stream_name) const;
  /** @brief Returns a synchronized stream diagnostic snapshot.
   *         返回同步后的数据流诊断快照。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @return Owned text, or empty for no diagnostic/unknown stream. 自有文本；无诊断或未知流时为空。
   */
  std::string stream_diagnostic(const std::string& stream_name) const;

 private:
  /// Grants the public wrapper access to immutable discovery state.
  /// 允许公开包装访问不可变发现状态。
  friend class SensorStreamClient;
  /// Grants focused tests access to receive/lifecycle hooks. 允许定向测试访问接收/生命周期钩子。
  friend class SensorStreamClientTestPeer;

  /** @brief Local client startup/delivery phase. 本地客户端启动/投递阶段。 */
  enum class Phase {
    /// No startup has committed. 尚未提交启动。
    kConstructed,
    /// Endpoints are being created. 正在创建端点。
    kStarting,
    /// Discovery and receive endpoints committed. 发现与接收端点已提交。
    kActive,
    /// Cancellation or Server stop permanently ended delivery. 取消或服务器停止已永久结束投递。
    kStopped
  };
  /** @brief Per-stream mutually exclusive delivery mode. 每数据流互斥投递模式。 */
  enum class DeliveryMode {
    /// No public delivery call has selected a mode. 尚无公开投递调用选择模式。
    kUnset,
    /// Frames invoke a user callback. 帧会调用用户回调。
    kCallback,
    /// Frames enter the bounded blocking mailbox. 帧进入有界阻塞邮箱。
    kWait
  };
  /**
   * @brief Owns all mutable delivery and diagnostics for one discovered stream.
   *        持有一个已发现数据流的全部可变投递状态与诊断。
   *
   * Except callback_active, every field is protected by Impl::mutex_. The
   * atomic flag drops concurrent same-stream callback delivery without blocking
   * an eCAL receive thread and is cleared on every admitted callback exit.
   * 除 callback_active 外，所有字段均由 Impl::mutex_ 保护。原子标志会丢弃同一数据流的
   * 并发回调投递而不阻塞 eCAL 接收线程，并在每次已准入回调退出时清除。
   */
  struct StreamState {
    /// Lock-free same-stream user-callback admission flag. 无锁的同一数据流用户回调准入标志。
    std::atomic_flag callback_active = ATOMIC_FLAG_INIT;
    /// Validated immutable discovery descriptor. 已校验的不可变发现描述符。
    mujoco_sim::v4::StreamDescriptor descriptor;
    /// Permanently selected delivery mode, protected by mutex_. 由 mutex_ 保护的永久投递模式。
    DeliveryMode mode = DeliveryMode::kUnset;
    /// Client-owned callback copied out under mutex_ before invocation. 调用前在 mutex_
    /// 下复制出的客户端自有回调。
    StreamCallback callback;
    /// Bounded owning sample mailbox, accessed only under mutex_. 仅在 mutex_
    /// 下访问的有界自有样本邮箱。
    internal::BlockingMailbox<OwnedStreamSample> samples;
    /// Last accepted positive stream sequence, protected by mutex_. 由 mutex_
    /// 保护的最近接收正序列号。
    std::uint64_t last_sequence = 0;
    /// Sequence gaps, Server drops, evictions, and busy-callback drops; protected by mutex_.
    /// 序列缺口、服务器丢帧、淘汰与回调忙碌丢帧总数；由 mutex_ 保护。
    std::uint64_t dropped_samples = 0;
    /// Latest non-terminal diagnostic or terminal text, protected by mutex_.
    /// 最新非终止诊断或终止文本；由 mutex_ 保护。
    std::string diagnostic;
    /// Sticky first terminal stream error, protected by mutex_. 由 mutex_
    /// 保护的首个粘性流终止错误。
    std::string terminal_error;
  };

  /** @brief Validates immutable identifiers, timeout, names, and capacity without I/O.
   *         不执行 I/O 地校验不可变标识符、超时、名称与容量。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true when all options are valid. 所有配置有效时为 true。
   */
  bool ValidateOptions(std::string* error_message) const;
  /** @brief Creates stream/status subscribers and enables callback admission.
   *         创建数据流/status 订阅者并启用回调准入。
   * @param error_message Non-null endpoint diagnostic. 非空端点诊断。
   * @return true when all endpoints are installed. 所有端点安装后为 true。
   */
  bool StartSampleReception(std::string* error_message);
  /** @brief Commits active phase unless cancellation won the race.
   *         提交活动阶段，除非取消在竞争中先发生。
   * @param error_message Non-null startup diagnostic. 非空启动诊断。
   * @return true when local active state commits. 本地活动状态提交后为 true。
   */
  bool FinishStart(std::string* error_message);
  /** @brief Performs bounded discovery and validates global response identity.
   *         执行有界发现并校验全局响应身份。
   * @param error_message Non-null RPC/response diagnostic. 非空 RPC/响应诊断。
   * @return true for a successful response with nonzero epoch. 成功响应且 epoch 非零时为 true。
   */
  bool DiscoverStreams(std::string* error_message);
  /** @brief Selects requested descriptors and creates per-stream state transactionally.
   *         事务性选择请求描述符并创建每流状态。
   * @param error_message Non-null descriptor diagnostic. 非空描述符诊断。
   * @return true when every requested descriptor is unique and valid. 每个请求描述符唯一且有效时为
   * true。
   */
  bool ConfigureDiscoveredStreams(std::string* error_message);
  /** @brief Validates descriptor topic, shape, payload bounds, convention, and rate.
   *         校验描述符主题、形状、payload 边界、坐标约定与频率。
   * @param descriptor Borrowed discovery descriptor. 借用的发现描述符。
   * @param error_message Non-null diagnostic output. 非空诊断输出。
   * @return true when descriptor semantics match protocol limits. 描述符语义符合协议限制时为 true。
   */
  bool ValidateDescriptor(const mujoco_sim::v4::StreamDescriptor& descriptor,
                          std::string* error_message) const;
  /** @brief Validates parsed frame identity, shape, and payload against discovery.
   *         根据发现信息校验解析帧身份、形状与 payload。
   * @param stream_name Endpoint stream identifier. 端点数据流标识符。
   * @param frame Borrowed parsed frame whose payload remains callback-scoped. 借用的解析帧；payload
   * 仍限于回调范围。
   * @param error_message Non-null diagnostic cleared on entry. 非空诊断；入口先清空。
   * @return true when frame matches the selected descriptor and epoch. 帧匹配所选描述符与 epoch
   * 时为 true。
   */
  bool IsExpectedFrame(const std::string& stream_name,
                       const mujoco_sim::protocol::ParsedStreamFrame& frame,
                       std::string* error_message) const;
  /** @brief Tests phase, epoch, and monotonic status tick under mutex_.
   *         在 mutex_ 下检查阶段、epoch 与单调状态 tick。
   * @param status Borrowed parsed Server status. 借用的已解析服务器状态。
   * @return true when status may mutate client state. 状态可修改客户端时为 true。
   * @pre mutex_ is held. 已持有 mutex_。
   */
  bool CanAcceptStatusLocked(const mujoco_sim::v4::ServerStatus& status) const;
  /** @brief Applies STOPPED/ERROR_PAUSED global state under mutex_.
   *         在 mutex_ 下应用 STOPPED/ERROR_PAUSED 全局状态。
   * @param status Accepted Server status. 已接受的服务器状态。
   * @return true when wait predicates changed. 等待谓词发生变化时为 true。
   * @pre mutex_ is held. 已持有 mutex_。
   */
  bool ApplyGlobalStatusLocked(const mujoco_sim::v4::ServerStatus& status);
  /** @brief Applies per-stream terminal errors under mutex_.
   *         在 mutex_ 下应用每流终止错误。
   * @param status Accepted Server status. 已接受的服务器状态。
   * @return true when at least one terminal error was newly recorded. 新记录至少一个终止错误时为
   * true。
   * @pre mutex_ is held. 已持有 mutex_。
   */
  bool ApplyStreamErrorsLocked(const mujoco_sim::v4::ServerStatus& status);
  /** @brief Accepts a newer stream sequence and updates drop accounting.
   *         接受更新的数据流序列并更新丢帧计数。
   * @param stream Non-null selected stream state. 非空已选数据流状态。
   * @param header Validated frame header. 已校验帧 header。
   * @return true for a strictly newer sequence; false without mutation otherwise.
   *         序列严格更新时为 true，否则不修改并返回 false。
   * @pre mutex_ is held. 已持有 mutex_。
   */
  bool AcceptStreamSequenceLocked(StreamState* stream,
                                  const mujoco_sim::v4::StreamFrameHeader& header);
  /** @brief Parses borrowed eCAL stream bytes and records malformed input diagnostics.
   *         解析借用的 eCAL 数据流字节并记录畸形输入诊断。
   * @param stream_name Endpoint stream identifier. 端点数据流标识符。
   * @param data Callback-owned bytes valid only for this call. 仅本次调用有效的回调自有字节。
   * @param size Byte count. 字节数。
   */
  void HandleStreamBytes(const std::string& stream_name, const void* data, std::size_t size);
  /** @brief Parses borrowed ServerStatus and applies accepted terminal state.
   *         解析借用 ServerStatus 并应用已接受的终止状态。
   * @param data Callback-owned bytes valid only for this call. 仅本次调用有效的回调自有字节。
   * @param size Byte count. 字节数。
   */
  void HandleStatusBytes(const void* data, std::size_t size);
  /** @brief Delivers a validated frame in the stream's permanent mode.
   *         按数据流永久模式投递一帧已校验数据。
   *
   * User callbacks run without mutex_. Their view borrows frame payload only
   * for the invocation. 用户回调执行时不持有 mutex_；其视图仅在调用期间借用帧 payload。
   *
   * @param stream_name Selected stream identifier. 已选数据流标识符。
   * @param frame Callback-scoped parsed frame. 回调范围内的解析帧。
   */
  void HandleStreamFrame(const std::string& stream_name,
                         const mujoco_sim::protocol::ParsedStreamFrame& frame);
  /** @brief Stores latest non-terminal diagnostic unless a terminal error is sticky.
   *         在没有粘性终止错误时保存最新非终止诊断。
   * @param stream_name Selected stream identifier. 已选数据流标识符。
   * @param diagnostic Non-empty owned/copyable text. 非空自有/可复制文本。
   */
  void RecordDiagnostic(const std::string& stream_name, const std::string& diagnostic);
  /** @brief Disables reception, removes callbacks, and drains other-thread callbacks.
   *         禁用接收、移除回调并等待其他线程上的回调退出。
   *
   * Teardown exceptions are retained in stream diagnostics. No client mutex is
   * held while invoking eCAL or waiting. 拆除异常会保存在流诊断中；调用 eCAL 或等待时
   * 不持有客户端互斥锁。
   */
  void StopSampleReception() noexcept;

  /// Callback admission gate specialized for this implementation. 针对此实现的回调准入门控。
  using CallbackGate = internal::CallbackGate<Impl>;
  /// Shared callback admission/accounting state. 共享回调准入/计数状态。
  using CallbackState = CallbackGate::State;
  /// One admitted transport callback invocation. 一次已准入传输回调调用。
  using CallbackLease = CallbackGate::Lease;

  /// Immutable construction options. 不可变构造配置。
  SensorStreamOptions options_;
  /// Successful Protocol V4 discovery response; immutable after Start().
  /// 成功的 Protocol V4 发现响应；Start() 后不可变。
  mujoco_sim::v4::GetSensorStreamsResponse discovery_;
  /// Shared bounded discovery transport retained through endpoint teardown.
  /// 保留至端点拆除完成的共享有界发现传输。
  std::shared_ptr<internal::SensorStreamTransport> transport_;
  /// Owned per-stream receive endpoints; mutated only during admitted lifecycle work.
  /// 自有每流接收端点；仅在已准入生命周期工作中修改。
  std::unordered_map<std::string, std::unique_ptr<eCAL::CSubscriber>> subscribers_;
  /// Owned Server status endpoint; mutated only during admitted lifecycle work.
  /// 自有服务器状态端点；仅在已准入生命周期工作中修改。
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  /// Shared per-stream state map, structurally fixed after Start() and field-protected by mutex_.
  /// 共享每流状态映射；Start() 后结构固定，各字段由 mutex_ 保护。
  std::unordered_map<std::string, std::shared_ptr<StreamState>> streams_;
  /// Last accepted ServerStatus tick for monotonic filtering, protected by mutex_.
  /// 用于单调过滤的最近服务器状态 tick；由 mutex_ 保护。
  std::optional<std::uint64_t> last_status_tick_id_;
  /// Admission state outliving registered callback functors. 生命周期长于已注册回调函数的准入状态。
  std::shared_ptr<CallbackState> callback_state_;
  /// Serializes lifecycle calls without retaining a lock during their work.
  /// 在工作期间不持锁地串行化生命周期调用。
  internal::LifecycleOperation lifecycle_;
  /// Protects phase_, stream fields, status tick, and monitor flag; never held for external calls.
  /// 保护 phase_、流字段、状态 tick 与监控标志；调用外部代码时绝不持有。
  mutable std::mutex mutex_;
  /// Wakes all stream waits when data, stop, or terminal predicates change.
  /// 数据、停止或终止谓词变化时唤醒所有数据流等待。
  std::condition_variable condition_;
  /// Lock-free cancellation visible to RPC polling and callbacks. RPC
  /// 轮询与回调可见的无锁取消状态。
  std::atomic<bool> stop_requested_{false};
  /// Process-wide eCAL runtime lease released after endpoints/transport. 在端点/传输后释放的进程级
  /// eCAL 租约。
  std::shared_ptr<void> ecal_runtime_;
  /// Whether waits should treat eCAL runtime shutdown as normal stop; protected by mutex_.
  /// 等待是否应将 eCAL 运行时关闭视为正常停止；由 mutex_ 保护。
  bool monitor_ecal_ = false;
  /// Local lifecycle phase protected by mutex_. 由 mutex_ 保护的本地生命周期阶段。
  Phase phase_ = Phase::kConstructed;
};

}  // namespace mujoco_sim::adapter_sdk
