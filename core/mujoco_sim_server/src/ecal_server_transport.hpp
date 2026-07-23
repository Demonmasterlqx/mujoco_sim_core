#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/v4/sync.pb.h>

namespace eCAL {
class CPublisher;
class CServiceServer;
class CSubscriber;
}  // namespace eCAL

namespace spdlog {
class logger;
}  // namespace spdlog

namespace mujoco_sim::server {

class CallbackState;

class EcalServerTransport {
 public:
  struct Handlers {
    std::function<mujoco_sim::v4::GetAvailableResourcesResponse(
        mujoco_sim::v4::GetAvailableResourcesRequest)>
        get_available_resources;
    std::function<mujoco_sim::v4::RegisterAdapterResponse(mujoco_sim::v4::RegisterAdapterRequest)>
        register_adapter;
    std::function<mujoco_sim::v4::ActivateAdapterResponse(mujoco_sim::v4::ActivateAdapterRequest)>
        activate_adapter;
    std::function<mujoco_sim::v4::UnregisterAdapterResponse(
        mujoco_sim::v4::UnregisterAdapterRequest)>
        unregister_adapter;
    std::function<mujoco_sim::v4::GetSensorStreamsResponse(mujoco_sim::v4::GetSensorStreamsRequest)>
        get_sensor_streams;
    std::function<void(mujoco_sim::core::ControllerCommand)> command;
    std::function<void(mujoco_sim::core::AdapterHeartbeat)> heartbeat;
    std::function<void(mujoco_sim::core::ControllerFault)> fault;
    std::function<void(std::string_view, std::size_t)> malformed;
  };

  /**
   * @brief Describes one non-blocking outbound transport attempt.
   *
   * `sent == false` without `serialization_failed` or `transport_error` is an
   * ordinary no-subscriber/retryable result. Serialization and eCAL exceptions
   * retain contextual diagnostics and must be reported by the simulation loop.
   * String and byte fields own their values after the transport call returns.
   *
   * 描述一次非阻塞的出站传输尝试。
   *
   * `sent == false` 且未设置 `serialization_failed` 或 `transport_error` 表示普通的
   * 无订阅者或可重试结果。序列化失败与 eCAL 异常会保留带上下文的诊断，并必须由仿真
   * 循环报告。传输调用返回后，字符串与字节计数字段仍持有自身数据。
   */
  struct PublishResult {
    /// Destination topic, including missing-endpoint failures. 目标 topic，包括端点缺失失败。
    std::string topic;
    /// Whether the transport reached a send operation. 是否到达传输发送操作。
    bool attempted = false;
    /// Whether eCAL accepted the payload. eCAL 是否接受该 payload。
    bool sent = false;
    /// Whether this attempt lazily created the endpoint. 本次尝试是否延迟创建了端点。
    bool publisher_created = false;
    /// Whether Protobuf serialization failed before send. 是否在发送前发生 Protobuf 序列化失败。
    bool serialization_failed = false;
    /// Serialized or encoded payload size in bytes. 已序列化或编码 payload 的字节数。
    std::size_t bytes = 0;
    /// Owned endpoint or eCAL exception diagnostic. 自有的端点或 eCAL 异常诊断。
    std::string transport_error;
  };

  EcalServerTransport(std::string server_name, std::shared_ptr<spdlog::logger> logger);

  /**
   * @brief Stops input, removes transport callbacks, and drains active callback work.
   *        停止输入、移除传输回调，并等待活动回调工作退出。
   *
   * Teardown diagnostics are logged without allowing exceptions to escape.
   * 拆除诊断会写入日志，且异常不会从析构函数逸出。
   */
  ~EcalServerTransport() noexcept;

  EcalServerTransport(const EcalServerTransport&) = delete;
  EcalServerTransport& operator=(const EcalServerTransport&) = delete;

  bool Start(Handlers handlers, std::string* error_message);
  void StopInputs();
  void Reset();

  PublishResult PublishControllerState(std::string_view adapter_id, std::string_view encoded_frame);
  PublishResult PublishObservation(std::string_view adapter_id, std::string_view encoded_frame);
  PublishResult PublishStatus(const mujoco_sim::v4::ServerStatus& status);
  bool PrepareAdapterPublisher(std::string_view adapter_id, mujoco_sim::v4::AdapterRole role,
                               std::string* error_message, bool* created);
  void RemoveAdapterPublisher(std::string_view adapter_id, mujoco_sim::v4::AdapterRole role);
  std::size_t SampleSubscriberCounts();
  bool HasControllerStateSubscriber(std::string_view adapter_id) const;

 private:
  std::string server_name_;
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<CallbackState> callback_state_;
  std::unique_ptr<eCAL::CServiceServer> registry_service_;
  std::unique_ptr<eCAL::CSubscriber> command_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> heartbeat_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> fault_subscriber_;
  std::unique_ptr<eCAL::CPublisher> status_publisher_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>> state_publishers_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>> observation_publishers_;
  std::unordered_map<std::string, std::size_t> state_subscriber_counts_;
  // Failure keys are owned and updated only by the simulation thread.
  // 失败键仅由仿真线程持有并更新。
  std::unordered_set<std::string> reported_transport_failures_;
  std::vector<std::string> methods_;
};

}  // namespace mujoco_sim::server
