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

class SensorStreamClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  explicit Impl(SensorStreamOptions options);
  ~Impl();
  bool Start(std::string* error_message);
  bool SetCallback(const std::string& stream_name, StreamCallback callback,
                   std::string* error_message);
  SampleWaitResult WaitForSample(const std::string& stream_name, std::chrono::milliseconds timeout,
                                 OwnedStreamSample* sample, std::string* error_message);
  void Stop();
  std::optional<mujoco_sim::v4::StreamDescriptor> descriptor(const std::string& stream_name) const;
  std::uint64_t dropped_samples(const std::string& stream_name) const;
  std::string stream_diagnostic(const std::string& stream_name) const;

 private:
  friend class SensorStreamClient;
  friend class SensorStreamClientTestPeer;
  enum class Phase {
    kConstructed,
    kStarting,
    kActive,
    kStopped
  };
  enum class DeliveryMode {
    kUnset,
    kCallback,
    kWait
  };
  struct StreamState {
    // The shared StreamState owns callback admission for its full lifetime.
    // test_and_set() drops concurrent delivery instead of blocking an eCAL
    // callback thread; the admitted callback clears the flag on every exit.
    // 共享 StreamState 在完整生命周期内持有回调准入状态。test_and_set() 会丢弃
    // 并发投递而不阻塞 eCAL 回调线程；获准执行的回调会在所有退出路径清除该标志。
    std::atomic_flag callback_active = ATOMIC_FLAG_INIT;
    mujoco_sim::v4::StreamDescriptor descriptor;
    DeliveryMode mode = DeliveryMode::kUnset;
    StreamCallback callback;
    internal::BlockingMailbox<OwnedStreamSample> samples;
    std::uint64_t last_sequence = 0;
    std::uint64_t dropped_samples = 0;
    std::string diagnostic;
    std::string terminal_error;
  };
  bool ValidateOptions(std::string* error_message) const;
  bool StartSampleReception(std::string* error_message);
  bool FinishStart(std::string* error_message);
  bool DiscoverStreams(std::string* error_message);
  bool ConfigureDiscoveredStreams(std::string* error_message);
  bool ValidateDescriptor(const mujoco_sim::v4::StreamDescriptor& descriptor,
                          std::string* error_message) const;
  bool IsExpectedFrame(const std::string& stream_name,
                       const mujoco_sim::protocol::ParsedStreamFrame& frame,
                       std::string* error_message) const;
  bool CanAcceptStatusLocked(const mujoco_sim::v4::ServerStatus& status) const;
  bool ApplyGlobalStatusLocked(const mujoco_sim::v4::ServerStatus& status);
  bool ApplyStreamErrorsLocked(const mujoco_sim::v4::ServerStatus& status);
  /**
   * @brief Accepts a newer stream sequence and updates drop accounting.
   *        接受较新的数据流序列号并更新丢帧计数。
   *
   * The caller must hold mutex_. A stale or repeated sequence leaves the
   * stream state unchanged.
   * 调用方必须持有 mutex_。过期或重复的序列号不会改变数据流状态。
   */
  bool AcceptStreamSequenceLocked(StreamState* stream,
                                  const mujoco_sim::v4::StreamFrameHeader& header);
  void HandleStreamBytes(const std::string& stream_name, const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleStreamFrame(const std::string& stream_name,
                         const mujoco_sim::protocol::ParsedStreamFrame& frame);
  void RecordDiagnostic(const std::string& stream_name, const std::string& diagnostic);
  /**
   * @brief Disables sample reception, removes eCAL callbacks, and drains active callbacks.
   *        禁用样本接收、移除 eCAL 回调并等待活动回调退出。
   *
   * eCAL teardown exceptions are retained in stream diagnostics.
   * eCAL 拆除异常会保存在数据流诊断中。
   */
  void StopSampleReception() noexcept;
  using CallbackGate = internal::CallbackGate<Impl>;
  using CallbackState = CallbackGate::State;
  using CallbackLease = CallbackGate::Lease;

  SensorStreamOptions options_;
  mujoco_sim::v4::GetSensorStreamsResponse discovery_;
  std::shared_ptr<internal::SensorStreamTransport> transport_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CSubscriber>> subscribers_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::unordered_map<std::string, std::shared_ptr<StreamState>> streams_;
  std::optional<std::uint64_t> last_status_tick_id_;
  std::shared_ptr<CallbackState> callback_state_;
  internal::LifecycleOperation lifecycle_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  bool monitor_ecal_ = false;
  Phase phase_ = Phase::kConstructed;
};

}  // namespace mujoco_sim::adapter_sdk
