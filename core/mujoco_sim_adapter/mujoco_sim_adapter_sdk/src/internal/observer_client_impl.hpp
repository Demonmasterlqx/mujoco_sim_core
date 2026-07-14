#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>

#include "adapter_transport.hpp"
#include "blocking_mailbox.hpp"
#include "callback_gate.hpp"
#include "adapter_session.hpp"

namespace mujoco_sim::adapter_sdk {

class ObserverClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  explicit Impl(ObserverOptions options);
  ~Impl();
  bool Start(std::string* error_message);
  bool Activate(std::string* error_message);
  ObservationWaitResult WaitForObservation(
      std::chrono::milliseconds timeout,
      mujoco_sim::v2::ObservationFrame* frame, std::string* error_message);
  void Stop();
  bool Unregister(std::string* error_message);
  std::uint64_t dropped_frames() const;
  std::string lifecycle_warning() const;

 private:
  friend class ObserverClient;
  friend class ObserverClientTestPeer;
  bool ValidateOptions(std::string* error_message) const;
  bool ValidateRegistration(std::string* error_message) const;
  void ResetSessionData();
  bool StartObservationReception(std::string* error_message);
  bool StartHeartbeat(std::string* error_message);
  void AbortStart();
  bool Register(std::string* error_message);
  bool FinishStart(std::string* error_message);
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  bool UnregisterSession(std::string* error_message);
  bool CompleteUnregister(
      const mujoco_sim::v2::UnregisterAdapterResponse& response,
      std::string* error_message);
  void MarkStoppedLocked();
  void TransitionToStopped();
  bool IsExpectedObservation(
      const mujoco_sim::v2::ObservationFrame& frame) const;
  void HandleObservationBytes(const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleObservation(const mujoco_sim::v2::ObservationFrame& frame);
  void SetTerminalError(const std::string& error);
  void StopObservationReception();
  void StartHeartbeatThread();
  void JoinHeartbeatThread();
  void HeartbeatLoop();
  mujoco_sim::v2::AdapterHeartbeat BuildHeartbeat() const;

  using CallbackGate = internal::CallbackGate<Impl>;
  using CallbackState = CallbackGate::State;
  using CallbackLease = CallbackGate::Lease;

  ObserverOptions options_;
  mujoco_sim::v2::RegisterAdapterResponse registration_;
  std::shared_ptr<internal::AdapterTransport> transport_;
  std::unique_ptr<eCAL::CSubscriber> observation_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::shared_ptr<CallbackState> callback_state_;
  std::mutex lifecycle_mutex_;
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
  internal::BlockingMailbox<mujoco_sim::v2::ObservationFrame> observations_;
  std::uint64_t last_sequence_ = 0;
  std::uint64_t last_tick_id_ = 0;
  std::uint64_t dropped_frames_ = 0;
  std::string terminal_error_;
  std::string lifecycle_warning_;
};

}  // namespace mujoco_sim::adapter_sdk
