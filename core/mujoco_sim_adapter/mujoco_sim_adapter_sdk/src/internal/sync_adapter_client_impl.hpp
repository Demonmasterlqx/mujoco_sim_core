#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/v3/sync.pb.h>

#include "adapter_session.hpp"
#include "adapter_transport.hpp"
#include "callback_gate.hpp"

namespace mujoco_sim::adapter_sdk {

struct TickLease::Data {
  mujoco_sim::protocol::DataFrameHeader header;
  std::vector<double> scalars;
};

class SyncAdapterClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  explicit Impl(AdapterOptions options);
  ~Impl();

  bool Start(std::string* error_message);
  bool Activate(std::string* error_message);
  TickWaitResult WaitForTick(std::chrono::milliseconds timeout,
                             TickLease* tick, std::string* error_message);
  bool Commit(const TickLease& tick, std::span<const double> commands,
              std::string* error_message);
  bool FailTick(const TickLease& tick, FaultCode code, std::string_view message,
                std::string* error_message);
  void Stop();
  bool Unregister(std::string* error_message);
  std::string lifecycle_warning() const;
  std::uint64_t layout_hash_mismatches() const;

 private:
  friend class SyncAdapterClientTestPeer;
  friend class SyncAdapterClient;

  bool ValidateOptions(std::string* error_message) const;
  bool StartTickReception(std::string* error_message);
  bool Register(std::string* error_message);
  bool ValidateRegistration(std::string* error_message);
  bool FinishStart(std::string* error_message);
  std::optional<TickWaitResult> TickWaitRejectionLocked(
      std::string* error_message) const;
  bool WaitForQueuedTick(std::unique_lock<std::mutex>& lock,
                         std::chrono::milliseconds timeout);
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  bool ValidateLeaseLocked(const TickLease& tick,
                           std::shared_ptr<const TickLease::Data>* canonical,
                           std::string* error_message) const;
  bool ValidateCommands(std::span<const double> commands,
                        std::string* error_message) const;
  bool UnregisterSession(std::string* error_message);
  bool CompleteUnregister(const mujoco_sim::v3::UnregisterAdapterResponse& response,
                          std::string* error_message);
  void MarkStoppedLocked();
  void TransitionToStopped();
  void AddLifecycleWarning(const std::string& warning);
  void SetTerminalError(const std::string& error);
  void StopTickReception();
  void HandleTickBytes(const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleTick(std::shared_ptr<const TickLease::Data> tick);
  enum class TickAction { kReject, kRetryCommit, kQueue };
  TickAction ClassifyTickLocked(
      const std::shared_ptr<const TickLease::Data>& tick,
      std::shared_ptr<const std::string>* retry_commit);
  bool AcceptKnownTickLocked(
      const std::shared_ptr<const TickLease::Data>& tick,
      const std::shared_ptr<const TickLease::Data>& known);
  std::optional<std::uint64_t> NewestKnownTickLocked(
      const TickLease::Data& tick) const;
  void RecordTickConflictLocked(std::uint64_t tick_id);
  void StartHeartbeatThread();
  void JoinHeartbeatThread();
  void HeartbeatLoop();
  mujoco_sim::v3::SessionHeartbeat BuildHeartbeat() const;

  using CallbackGate = internal::CallbackGate<Impl>;
  using CallbackState = CallbackGate::State;
  using CallbackLease = CallbackGate::Lease;

  AdapterOptions options_;
  mujoco_sim::v3::RegisterAdapterResponse wire_registration_;
  AdapterRegistration registration_;
  mujoco_sim::protocol::LayoutFingerprint layout_fingerprint_{};
  std::shared_ptr<internal::AdapterTransport> transport_;
  std::unique_ptr<eCAL::CSubscriber> tick_subscriber_;
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
  std::chrono::steady_clock::time_point last_commit_publish_{};
  std::chrono::steady_clock::time_point last_heartbeat_publish_{};
  bool monitor_ecal_ = false;
  internal::AdapterSession session_;
  bool wait_in_progress_ = false;
  std::shared_ptr<const TickLease::Data> queued_tick_;
  std::shared_ptr<const TickLease::Data> outstanding_tick_;
  std::shared_ptr<const TickLease::Data> committed_tick_;
  std::shared_ptr<const std::string> cached_commit_;
  std::string terminal_error_;
  std::string lifecycle_warning_;
  std::uint64_t layout_hash_mismatches_ = 0;
};

}  // namespace mujoco_sim::adapter_sdk
