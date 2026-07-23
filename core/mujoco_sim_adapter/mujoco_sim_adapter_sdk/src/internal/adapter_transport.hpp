#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::adapter_sdk::internal {

/**
 * @brief Describes one non-blocking Adapter publication attempt.
 *
 * `sent == false` with an empty error is an expected retryable transport
 * outcome. A non-empty error identifies serialization, endpoint, or eCAL
 * failure and must be surfaced through the owning client's error mechanism.
 *
 * 描述一次非阻塞的 Adapter 发布尝试。
 *
 * `sent == false` 且错误为空表示正常的可重试传输结果。非空错误用于标识序列化、端点
 * 或 eCAL 失败，必须通过所属客户端的错误机制向上报告。
 */
struct PublishResult {
  /// Whether eCAL accepted the payload. eCAL 是否接受该 payload。
  bool sent = false;
  /// Owned serialization, endpoint, or eCAL diagnostic. 自有的序列化、端点或 eCAL 诊断。
  std::string error;
};

/**
 * @brief Abstracts bounded Adapter RPC and non-blocking publication for clients/tests.
 *        为客户端与测试抽象有界 Adapter RPC 与非阻塞发布。
 *
 * Implementations own their eCAL endpoints. RPC methods write their response
 * only when transport invocation succeeds and provide contextual diagnostics
 * on failure. Publication methods distinguish retryable rejection from
 * terminal serialization/endpoint/eCAL failure through PublishResult.
 * 实现持有其 eCAL 端点。RPC 方法仅在传输调用成功时写入响应，失败时提供上下文诊断。
 * 发布方法通过 PublishResult 区分可重试拒绝与终止性的序列化/端点/eCAL 故障。
 */
class AdapterTransport {
 public:
  /** @brief Releases owned transport endpoints. 释放持有的传输端点。 */
  virtual ~AdapterTransport() = default;
  /** @brief Performs bounded Adapter registration RPC.
   *         执行有界 Adapter 注册 RPC。
   * @param request Borrowed registration request. 借用的注册请求。
   * @param response Required output written on transport success. 必需输出，传输成功时写入。
   * @param error_message Required contextual transport diagnostic. 必需的上下文传输诊断。
   * @return true when an RPC response was received; otherwise false.
   *         收到 RPC 响应时返回 true，否则返回 false。
   */
  virtual bool Register(const mujoco_sim::v4::RegisterAdapterRequest& request,
                        mujoco_sim::v4::RegisterAdapterResponse* response,
                        std::string* error_message) = 0;
  /** @brief Performs bounded Adapter activation RPC.
   *         执行有界 Adapter 激活 RPC。
   * @param request Borrowed activation request. 借用的激活请求。
   * @param response Required output written on transport success. 必需输出，传输成功时写入。
   * @param error_message Required contextual transport diagnostic. 必需的上下文传输诊断。
   * @return true when an RPC response was received; otherwise false.
   *         收到 RPC 响应时返回 true，否则返回 false。
   */
  virtual bool Activate(const mujoco_sim::v4::ActivateAdapterRequest& request,
                        mujoco_sim::v4::ActivateAdapterResponse* response,
                        std::string* error_message) = 0;
  /** @brief Performs bounded Adapter cleanup RPC.
   *         执行有界 Adapter 清理 RPC。
   * @param request Borrowed unregister request. 借用的注销请求。
   * @param response Required output written on transport success. 必需输出，传输成功时写入。
   * @param error_message Required contextual transport diagnostic. 必需的上下文传输诊断。
   * @return true when an RPC response was received; otherwise false.
   *         收到 RPC 响应时返回 true，否则返回 false。
   */
  virtual bool Unregister(const mujoco_sim::v4::UnregisterAdapterRequest& request,
                          mujoco_sim::v4::UnregisterAdapterResponse* response,
                          std::string* error_message) = 0;
  /**
   * @brief Attempts to publish one already encoded Controller command.
   *        尝试发布一条已编码的控制器指令。
   * @param encoded_command Owned command bytes borrowed for this call.
   *                        本次调用期间借用的自有指令字节。
   * @return Structured non-blocking publication result. 结构化的非阻塞发布结果。
   */
  virtual PublishResult PublishCommand(std::string_view encoded_command) = 0;

  /**
   * @brief Serializes and attempts to publish one Controller fault.
   *        序列化并尝试发布一条控制器故障。
   * @param fault Owned fault value borrowed for this call. 本次调用期间借用的自有故障值。
   * @return Structured non-blocking publication result. 结构化的非阻塞发布结果。
   */
  virtual PublishResult PublishFault(const mujoco_sim::v4::ControllerFault& fault) = 0;

  /**
   * @brief Publishes one session heartbeat without blocking for server processing.
   *
   * A rejected result without an error is retryable. Callers retain lifecycle
   * ownership and may report a non-terminal diagnostic before the next retry.
   *
   * 发布一条会话心跳，且不等待服务器处理。
   *
   * 未包含错误的拒绝结果表示可重试。调用方仍持有生命周期所有权，并可在下次重试前
   * 记录非终止诊断。
   *
   * @param heartbeat Owned heartbeat value serialized during this call.
   *                  在本次调用中序列化的自有心跳数据。
   * @return Structured non-blocking publication result. 结构化的非阻塞发布结果。
   */
  virtual PublishResult PublishHeartbeat(const mujoco_sim::v4::SessionHeartbeat& heartbeat) = 0;
};

/** @brief Abstracts bounded Sensor stream discovery for clients and tests.
 *         为客户端与测试抽象有界 Sensor 数据流发现。
 */
class SensorStreamTransport {
 public:
  /** @brief Releases owned discovery endpoint. 释放持有的发现端点。 */
  virtual ~SensorStreamTransport() = default;
  /** @brief Performs bounded Sensor stream discovery RPC.
   *         执行有界 Sensor 数据流发现 RPC。
   * @param request Borrowed discovery request. 借用的发现请求。
   * @param response Required output written on transport success. 必需输出，传输成功时写入。
   * @param error_message Required contextual transport diagnostic. 必需的上下文传输诊断。
   * @return true when an RPC response was received; otherwise false.
   *         收到 RPC 响应时返回 true，否则返回 false。
   */
  virtual bool Discover(const mujoco_sim::v4::GetSensorStreamsRequest& request,
                        mujoco_sim::v4::GetSensorStreamsResponse* response,
                        std::string* error_message) = 0;
};

/** @brief Creates the production eCAL Adapter transport.
 *         创建生产用 eCAL Adapter 传输。
 * @param server_name Valid Server identifier used for endpoints. 用于端点的有效服务器标识符。
 * @param service_timeout_ms Positive discovery/RPC timeout in milliseconds.
 *                           正数的发现/RPC 超时，单位为毫秒。
 * @param stop_requested Borrowed atomic cancellation flag that outlives the transport.
 *                       生命周期长于传输的借用原子取消标志。
 * @param publish_commands Whether Command/Fault publishers are required.
 *                         是否需要 Command/Fault 发布端点。
 * @return Owning transport; endpoint construction may throw. 自有传输；端点构造可能抛出异常。
 */
std::shared_ptr<AdapterTransport> MakeEcalAdapterTransport(const std::string& server_name,
                                                           std::uint32_t service_timeout_ms,
                                                           const std::atomic<bool>* stop_requested,
                                                           bool publish_commands);
/** @brief Creates the production eCAL Sensor discovery transport.
 *         创建生产用 eCAL Sensor 发现传输。
 * @param server_name Valid Server identifier used for endpoints. 用于端点的有效服务器标识符。
 * @param service_timeout_ms Positive discovery/RPC timeout in milliseconds.
 *                           正数的发现/RPC 超时，单位为毫秒。
 * @param stop_requested Borrowed atomic cancellation flag that outlives the transport.
 *                       生命周期长于传输的借用原子取消标志。
 * @return Owning transport; endpoint construction may throw. 自有传输；端点构造可能抛出异常。
 */
std::shared_ptr<SensorStreamTransport> MakeEcalSensorStreamTransport(
    const std::string& server_name, std::uint32_t service_timeout_ms,
    const std::atomic<bool>* stop_requested);

}  // namespace mujoco_sim::adapter_sdk::internal
