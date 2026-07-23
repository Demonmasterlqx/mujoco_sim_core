#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Configures Sensor stream discovery, selection, and buffering.
 *        配置 Sensor 数据流发现、选择与缓冲行为。
 */
struct SensorStreamOptions {
  /// Server identifier used to derive endpoints; defaults to `"default"` and
  /// must satisfy Protocol V4 Server identifier rules.
  /// 用于派生端点的服务器标识符；默认为 `"default"`，且须符合 Protocol V4 服务器标识符规则。
  std::string server_name = "default";
  /// Non-empty Protocol V4 stream identifiers to discover and subscribe;
  /// duplicates are invalid. 要发现并订阅的非空 Protocol V4 数据流标识符；不得重复。
  std::vector<std::string> stream_names;
  /// Positive discovery and RPC timeout in milliseconds; defaults to 5000 ms.
  /// 正数的发现与 RPC 超时，单位为毫秒；默认为 5000 ms。
  std::uint32_t service_timeout_ms = 5000;
  /// Positive per-stream blocking-mode queue capacity; defaults to one newest
  /// sample and does not create a callback backlog. 正数的每数据流阻塞模式队列容量；
  /// 默认为仅保留最新一个样本，且不会形成回调积压。
  std::size_t queue_capacity = 1;
};

/**
 * @brief Owns a decoded stream header and its raw payload bytes.
 *        持有解码后的数据流头与原始 payload 字节。
 *
 * The header is a Protocol V4 Protobuf value. Payload shape, format, stride,
 * frame convention, simulation timestamp, tick, and sequence are described by
 * the validated header and discovery descriptor.
 * header 为 Protocol V4 Protobuf 值。payload 的形状、格式、步幅、坐标约定、仿真
 * 时间戳、tick 与序列由已校验的 header 和发现描述符说明。
 */
struct OwnedStreamSample {
  /// Owned metadata copied from the MJSTREAM frame. 从 MJSTREAM 帧复制的自有元数据。
  mujoco_sim::v4::StreamFrameHeader header;
  /// Owned raw payload bytes, safe to retain across calls and threads.
  /// 自有原始 payload 字节，可安全跨调用和线程保留。
  std::vector<std::byte> payload;
};

/**
 * @brief Presents a callback-scoped, non-owning Sensor sample view.
 *        提供仅在回调范围内有效的非自有 Sensor 样本视图。
 *
 * The view itself is created by SensorStreamClient. Its header is owned by the
 * view, but payload() borrows transport memory and expires when the callback
 * returns. Only Copy() produces data that may be retained beyond the callback.
 * 该视图由 SensorStreamClient 创建。header 由视图持有，但 payload() 借用传输内存，
 * 在回调返回时失效。只有 Copy() 得到的数据可保留到回调结束之后。
 */
class StreamSampleView {
 public:
  /** @brief Copying is disabled to emphasize callback-scoped lifetime.
   *         禁止复制，以强调其仅在回调范围内有效的生命周期。
   */
  StreamSampleView(const StreamSampleView&) = delete;
  /** @brief Copy assignment is disabled. 禁止复制赋值。 */
  StreamSampleView& operator=(const StreamSampleView&) = delete;

  /** @brief Borrows the parsed header for this view's lifetime.
   *         在本视图生命周期内借用解析后的 header。
   * @return Reference that expires when the callback returns. 回调返回时失效的引用。
   */
  const mujoco_sim::v4::StreamFrameHeader& header() const { return header_; }
  /** @brief Borrows raw payload bytes for the current callback only.
   *         仅在当前回调内借用原始 payload 字节。
   * @return Span that must not be stored or used after callback return.
   *         不得保存或在回调返回后使用的 span。
   */
  std::span<const std::byte> payload() const { return payload_; }

  /**
   * @brief Copies header and payload into independently owned storage.
   *        将 header 与 payload 复制到独立自有存储。
   * @return Sample safe to retain after callback return. 可在回调返回后安全保留的样本。
   * @par Blocking / 阻塞行为
   * Performs allocation and a payload-sized copy on the callback thread.
   * 在回调线程执行内存分配与 payload 大小的数据复制。
   */
  [[nodiscard]] OwnedStreamSample Copy() const;

 private:
  /// Allows the client to construct a view for one callback invocation.
  /// 允许客户端为一次回调调用构造视图。
  friend class SensorStreamClient;

  /** @brief Creates a callback-scoped view over borrowed payload memory.
   *         在借用 payload 内存上创建回调范围视图。
   * @param header Parsed metadata moved into the view. 移入视图的已解析元数据。
   * @param payload Transport bytes valid through the callback only. 仅在回调期间有效的传输字节。
   */
  StreamSampleView(mujoco_sim::v4::StreamFrameHeader header, std::span<const std::byte> payload)
      : header_(std::move(header)), payload_(payload) {}

  /// Owned header valid for the view lifetime. 在视图生命周期内有效的自有 header。
  mujoco_sim::v4::StreamFrameHeader header_;
  /// Transport-owned bytes expiring at callback return. 回调返回时失效的传输方自有字节。
  std::span<const std::byte> payload_;
};

/** @brief Result of waiting for a Sensor stream sample. 等待 Sensor 数据流样本的结果。 */
enum class SampleWaitResult {
  /// A fully owned sample was written to the output. 已向输出写入完全自有样本。
  kReceived,
  /// The deadline elapsed; output is unchanged and diagnostic is empty.
  /// 截止期限已到；输出不变且诊断为空。
  kTimeout,
  /// Local cancellation, Server stop, or eCAL shutdown ended the wait; output
  /// is unchanged and diagnostic is empty. 本地取消、服务器停止或 eCAL 关闭终止等待；
  /// 输出不变且诊断为空。
  kStopped,
  /// Invalid use or a sticky stream/lifecycle/protocol error ended the wait;
  /// output is unchanged and diagnostic is non-empty. 无效调用或粘性数据流/生命周期/
  /// 协议错误终止等待；输出不变且诊断非空。
  kError,
};

/**
 * @brief Discovers and consumes Protocol V4 Sensor streams without an Adapter session.
 *        在不创建 Adapter 会话的情况下发现并消费 Protocol V4 Sensor 数据流。
 *
 * The client is single-use. Each stream permanently selects callback or
 * blocking mode on first use. Different streams may wait or invoke callbacks
 * concurrently, but each stream permits one waiter and never overlaps its own
 * callbacks. Stop() may concurrently cancel waiters. Lifecycle operations do
 * not hold project locks across transport or callback teardown.
 * 客户端仅可使用一次。每个数据流首次使用时会永久选择回调或阻塞模式。不同数据流可
 * 并发等待或调用回调，但每个数据流只允许一个等待方，且同一数据流的回调绝不重叠。
 * Stop() 可并发取消等待。生命周期操作执行传输或回调拆除时不持有项目锁。
 */
class SensorStreamClient {
 public:
  /**
   * @brief Callback invoked synchronously on an eCAL receive thread.
   *        在 eCAL 接收线程同步调用的回调。
   *
   * The view and its payload expire at callback return. The callback may call
   * Copy(), query thread-safe diagnostics, call Stop(), or destroy its own
   * client, but after self-destruction it must return without accessing the
   * client. Exceptions are caught and recorded as diagnostics.
   * 视图及 payload 在回调返回时失效。回调可调用 Copy()、查询线程安全诊断、调用
   * Stop() 或销毁自身客户端，但自销毁后必须立即返回且不得再访问客户端。异常会被捕获并记录为诊断。
   */
  using StreamCallback = std::function<void(const StreamSampleView&)>;

  /** @brief Creates a non-started client and takes ownership of its options.
   *         创建未启动客户端并取得配置所有权。
   * @param options Discovery and buffering options validated by Start().
   *                由 Start() 校验的发现与缓冲配置。
   */
  explicit SensorStreamClient(SensorStreamOptions options);
  /**
   * @brief Cancels delivery, removes endpoints, and drains callbacks on other threads.
   *        取消投递、移除端点并等待其他线程上的回调退出。
   * @par Blocking / 阻塞行为
   * May wait for lifecycle work and callbacks other than the current callback;
   * callback-initiated destruction never waits for itself.
   * 可能等待生命周期工作及当前回调之外的回调；回调内触发的析构不会等待自身。
   */
  ~SensorStreamClient();

  /** @brief Clients are neither copyable nor movable. 客户端不可复制或移动。 */
  SensorStreamClient(const SensorStreamClient&) = delete;
  /** @brief Clients are neither copyable nor movable. 客户端不可复制或移动。 */
  SensorStreamClient& operator=(const SensorStreamClient&) = delete;

  /**
   * @brief Discovers descriptors, validates them, and starts receive endpoints.
   *        发现并校验描述符，然后启动接收端点。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when all requested streams are configured; otherwise false.
   *         所有请求数据流均配置完成时返回 true，否则返回 false。
   * @pre This single-use client has not previously started or stopped.
   *      此单次使用客户端此前未启动或停止。
   * @par Blocking / 阻塞行为
   * Performs bounded discovery/RPC and endpoint creation. 执行有界发现/RPC 与端点创建。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Selects callback delivery and discards queued blocking-mode samples.
   *        选择回调投递并丢弃已排队的阻塞模式样本。
   *
   * A same-stream frame arriving while the callback is active is dropped and
   * counted instead of blocking the eCAL thread. Replacing a callback is allowed
   * after callback mode is selected; switching from blocking mode is rejected.
   * 同一数据流回调执行期间到达的帧会被丢弃并计数，而不会阻塞 eCAL 线程。选择回调
   * 模式后允许替换回调；不允许从阻塞模式切换到回调模式。
   *
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @param callback Non-empty callable moved into client-owned storage.
   * 移入客户端自有存储的非空回调。
   * @param error_message Required diagnostic output; cleared on valid entry and
   *        empty on success. 必需诊断输出；有效入口会先清空，成功时保持为空。
   * @return true when callback mode is configured; otherwise false.
   *         回调模式配置成功时返回 true，否则返回 false。
   * @pre Start() succeeded and the stream has not selected blocking mode.
   *      Start() 已成功，且该数据流尚未选择阻塞模式。
   */
  bool SetCallback(const std::string& stream_name, StreamCallback callback,
                   std::string* error_message);

  /**
   * @brief Selects blocking delivery and waits for one fully owned sample.
   *        选择阻塞投递并等待一个完全自有样本。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @param timeout Non-negative wall-clock duration; `milliseconds::max()` waits
   *        until sample, cancellation, Server stop, or error. 非负墙钟等待时长；
   *        `milliseconds::max()` 会等待至样本、取消、服务器停止或错误发生。
   * @param sample Required output, modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required diagnostic output; cleared on valid entry.
   *                      必需诊断输出；有效入口会先清空。
   * @return Wait outcome; kReceived, kTimeout, and kStopped leave the diagnostic empty.
   *         等待结果；kReceived、kTimeout 与 kStopped 均保持诊断为空。
   * @pre Start() succeeded and the stream has not selected callback mode.
   *      Start() 已成功，且该数据流尚未选择回调模式。
   * @par Thread safety / 线程安全
   * One waiter per stream; different streams may wait concurrently, and Stop()
   * may cancel all waits. 每个数据流一个等待方；不同数据流可并发等待，Stop() 可取消所有等待。
   */
  SampleWaitResult WaitForSample(const std::string& stream_name, std::chrono::milliseconds timeout,
                                 OwnedStreamSample* sample, std::string* error_message);

  /**
   * @brief Permanently cancels local delivery and wakes all waiters.
   *        永久取消本地投递并唤醒所有等待方。
   *
   * The call is idempotent and non-blocking apart from short critical sections;
   * it does not immediately remove receive callbacks.
   * 本调用幂等，除短临界区外不阻塞；不会立即移除接收回调。
   */
  void Stop();

  /** @brief Copies the validated discovery descriptor for a requested stream.
   *         复制指定数据流的已校验发现描述符。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @return Owned descriptor, or std::nullopt before discovery/for an unknown stream.
   *         自有描述符；尚未发现或数据流未知时返回 std::nullopt。
   * @par Thread safety / 线程安全
   * Safe during reception and callback execution. 可在接收与回调执行期间安全调用。
   */
  std::optional<mujoco_sim::v4::StreamDescriptor> descriptor(const std::string& stream_name) const;
  /** @brief Returns transport gaps, Server drops, local evictions, and busy-callback drops.
   *         返回传输序列缺口、服务器丢帧、本地淘汰与回调忙碌丢帧之和。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @return Accumulated count, or zero for an unknown stream. 累计数量；数据流未知时为零。
   * @par Thread safety / 线程安全
   * Safe during reception and callback execution. 可在接收与回调执行期间安全调用。
   */
  std::uint64_t dropped_samples(const std::string& stream_name) const;
  /** @brief Returns the latest non-terminal diagnostic or sticky terminal error.
   *         返回最新非终止诊断或粘性终止错误。
   * @param stream_name Requested stream identifier. 已请求的数据流标识符。
   * @return Owned diagnostic snapshot; empty for no diagnostic or unknown stream.
   *         自有诊断快照；无诊断或数据流未知时为空。
   * @par Thread safety / 线程安全
   * Safe during reception and callback execution. 可在接收与回调执行期间安全调用。
   */
  std::string stream_diagnostic(const std::string& stream_name) const;

  /** @brief Returns the successful Protocol V4 discovery response.
   *         返回成功的 Protocol V4 发现响应。
   * @return Reference valid until client destruction. 在客户端析构前有效的引用。
   * @pre Start() completed successfully; do not race this access with Start().
   *      Start() 已成功完成；不得与 Start() 并发访问。
   */
  const mujoco_sim::v4::GetSensorStreamsResponse& discovery() const;

 private:
  /// Grants focused tests access to receive and lifecycle hooks.
  /// 允许定向测试访问接收与生命周期钩子。
  friend class SensorStreamClientTestPeer;
  /** @brief Private implementation type; not part of the installed ABI surface.
   *         私有实现类型；不属于已安装 ABI 接口。
   */
  class Impl;
  /// Shared implementation retained by admitted callbacks during teardown.
  /// 拆除期间由已准入回调共同持有的实现。
  std::shared_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::adapter_sdk
