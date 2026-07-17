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
    std::mutex delivery_mutex;
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
  void HandleStreamBytes(const std::string& stream_name, const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleStreamFrame(const std::string& stream_name,
                         const mujoco_sim::protocol::ParsedStreamFrame& frame);
  void RecordDiagnostic(const std::string& stream_name, const std::string& diagnostic);
  void StopSampleReception();
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
  std::mutex lifecycle_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  bool monitor_ecal_ = false;
  Phase phase_ = Phase::kConstructed;
};

}  // namespace mujoco_sim::adapter_sdk
