#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/v4/sync.pb.h>

#include "adapter_session.hpp"
#include "adapter_transport.hpp"
#include "blocking_mailbox.hpp"
#include "callback_gate.hpp"
#include "lifecycle_operation.hpp"

namespace mujoco_sim::adapter_sdk {

struct ObservationLease::Data {
  mujoco_sim::protocol::DataFrameHeader header;
  std::vector<double> scalars;
};

class ObserverClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  explicit Impl(ObserverOptions options);
  ~Impl();
  bool Start(std::string* error_message);
  bool Activate(std::string* error_message);
  ObservationWaitResult WaitForObservation(std::chrono::milliseconds timeout,
                                           ObservationLease* frame, std::string* error_message);
  void Stop();
  bool Unregister(std::string* error_message);
  std::uint64_t dropped_frames() const;
  std::string lifecycle_warning() const;

 private:
  friend class ObserverClient;
  friend class ObserverClientTestPeer;
  bool ValidateOptions(std::string* error_message) const;
  bool ValidateRegistration(std::string* error_message);
  void ResetSessionData();
  bool StartObservationReception(std::string* error_message);
  bool StartHeartbeat(std::string* error_message);
  void AbortStart();

  /**
   * @brief Rolls back a remotely accepted activation that cannot be committed locally.
   *
   * Local cancellation is published first, then the heartbeat thread and receive
   * callbacks are drained before the remote session is unregistered. A failed
   * compensating unregister preserves the session identity for an explicit retry
   * and appends its diagnostic to error_message.
   *
   * 回滚已被远端接受、但无法在本地提交的激活操作。
   *
   * 本函数先发布本地取消状态，再等待心跳线程退出并清空接收回调，最后注销远端会话。
   * 补偿注销失败时会保留会话标识以供显式重试，并将诊断附加到 error_message。
   *
   * @param error_message Non-null activation diagnostic updated on cleanup failure.
   *                      非空的激活诊断；清理失败时会更新该诊断。
   * @par Thread safety / 线程安全
   * The caller must own lifecycle-operation admission and no project mutex may be held.
   * 调用方必须持有生命周期操作准入权，且不得持有项目互斥锁。
   */
  void AbortActivation(std::string* error_message);
  bool Register(std::string* error_message);
  bool FinishStart(std::string* error_message);
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  bool UnregisterSession(std::string* error_message);
  bool CompleteUnregister(const mujoco_sim::v4::UnregisterAdapterResponse& response,
                          std::string* error_message);
  void MarkStoppedLocked();
  void TransitionToStopped();
  void HandleObservationBytes(const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleObservation(std::shared_ptr<const ObservationLease::Data> frame);
  void SetTerminalError(const std::string& error);

  /**
   * @brief Records a non-terminal lifecycle diagnostic without duplicating it.
   *        记录非终止生命周期诊断，并避免重复记录。
   * @param warning Contextual diagnostic text. 带上下文的诊断文本。
   * @par Thread safety / 线程安全
   * May be called by the heartbeat thread.
   * 可由心跳线程调用。
   */
  void RecordLifecycleWarning(const std::string& warning);
  /**
   * @brief Disables reception, removes eCAL callbacks, and drains active callbacks.
   *        禁用接收、移除 eCAL 回调并等待活动回调退出。
   *
   * eCAL teardown exceptions are retained as non-terminal lifecycle warnings.
   * eCAL 拆除异常会保存为非终止生命周期警告。
   */
  void StopObservationReception() noexcept;

  /**
   * @brief Starts the owned heartbeat thread unless cancellation was requested.
   *        在尚未请求取消时启动持有的心跳线程。
   *
   * @return true if the thread is running; false if startup was cancelled.
   *         线程正在运行时返回 true；启动已取消时返回 false。
   * @par Thread safety / 线程安全
   * Serializes heartbeat-thread ownership internally.
   * 在内部串行化心跳线程所有权。
   */
  bool StartHeartbeatThread();
  void JoinHeartbeatThread();

  /**
   * @brief Runs the heartbeat thread entry and converts exceptions into a terminal client error.
   *        运行心跳线程入口，并将异常转换为客户端终止错误。
   *
   * @par Thread safety / 线程安全
   * This function is called only by the owned heartbeat thread.
   * 本函数仅由持有的心跳线程调用。
   */
  void RunHeartbeatLoop() noexcept;
  void HeartbeatLoop();
  mujoco_sim::v4::SessionHeartbeat BuildHeartbeat() const;

  using CallbackGate = internal::CallbackGate<Impl>;
  using CallbackState = CallbackGate::State;
  using CallbackLease = CallbackGate::Lease;

  ObserverOptions options_;
  mujoco_sim::v4::RegisterAdapterResponse wire_registration_;
  AdapterRegistration registration_;
  mujoco_sim::protocol::LayoutFingerprint layout_fingerprint_{};
  std::shared_ptr<internal::AdapterTransport> transport_;
  std::unique_ptr<eCAL::CSubscriber> observation_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::shared_ptr<CallbackState> callback_state_;
  internal::LifecycleOperation lifecycle_;
  std::mutex heartbeat_thread_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::condition_variable heartbeat_condition_;
  std::thread heartbeat_thread_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  std::chrono::milliseconds activation_delay_{100};
  bool monitor_ecal_ = false;
  internal::AdapterSession session_;
  internal::BlockingMailbox<std::shared_ptr<const ObservationLease::Data>> observations_;
  std::uint64_t last_sequence_ = 0;
  std::uint64_t last_tick_id_ = 0;
  std::uint64_t dropped_frames_ = 0;
  std::string terminal_error_;
  std::string lifecycle_warning_;
};

}  // namespace mujoco_sim::adapter_sdk
