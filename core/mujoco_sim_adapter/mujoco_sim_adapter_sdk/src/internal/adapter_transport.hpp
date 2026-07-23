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

class AdapterTransport {
 public:
  virtual ~AdapterTransport() = default;
  virtual bool Register(const mujoco_sim::v4::RegisterAdapterRequest& request,
                        mujoco_sim::v4::RegisterAdapterResponse* response,
                        std::string* error_message) = 0;
  virtual bool Activate(const mujoco_sim::v4::ActivateAdapterRequest& request,
                        mujoco_sim::v4::ActivateAdapterResponse* response,
                        std::string* error_message) = 0;
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

class SensorStreamTransport {
 public:
  virtual ~SensorStreamTransport() = default;
  virtual bool Discover(const mujoco_sim::v4::GetSensorStreamsRequest& request,
                        mujoco_sim::v4::GetSensorStreamsResponse* response,
                        std::string* error_message) = 0;
};

std::shared_ptr<AdapterTransport> MakeEcalAdapterTransport(const std::string& server_name,
                                                           std::uint32_t service_timeout_ms,
                                                           const std::atomic<bool>* stop_requested,
                                                           bool publish_commands);
std::shared_ptr<SensorStreamTransport> MakeEcalSensorStreamTransport(
    const std::string& server_name, std::uint32_t service_timeout_ms,
    const std::atomic<bool>* stop_requested);

}  // namespace mujoco_sim::adapter_sdk::internal
