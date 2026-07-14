#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include <ecal/pubsub/subscriber.h>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include "adapter_transport.hpp"
#include "callback_gate.hpp"
#include "adapter_session.hpp"

namespace mujoco_sim::adapter_sdk {

class SyncAdapterClient::Impl : public std::enable_shared_from_this<Impl> {
 public:
  explicit Impl(AdapterOptions options);
  ~Impl();

  bool Start(std::string* error_message);
  bool Activate(std::string* error_message);
  TickWaitResult WaitForTick(std::chrono::milliseconds timeout,
                             mujoco_sim::v2::Tick* tick,
                             std::string* error_message);
  bool Commit(const mujoco_sim::v2::Tick& tick,
              const std::vector<CommandValue>& commands,
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
  bool FinishStart(std::string* error_message);
  std::optional<TickWaitResult> TickWaitRejectionLocked(
      std::string* error_message) const;
  bool WaitForQueuedTick(std::unique_lock<std::mutex>& lock,
                         std::chrono::milliseconds timeout);
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  bool BuildValidatedCommit(const mujoco_sim::v2::Tick& tick,
                            const std::vector<CommandValue>& commands,
                            mujoco_sim::v2::AdapterCommit* commit,
                            std::string* error_message) const;
  bool SelectCanonicalTickLocked(const mujoco_sim::v2::Tick& tick,
                                 mujoco_sim::v2::Tick* canonical_tick,
                                 std::string* error_message) const;
  bool FinalizeCommitLocked(const mujoco_sim::v2::Tick& canonical_tick,
                            mujoco_sim::v2::AdapterCommit* commit,
                            std::string* error_message);
  bool UnregisterSession(std::string* error_message);
  bool CompleteUnregister(
      const mujoco_sim::v2::UnregisterAdapterResponse& response,
      std::string* error_message);
  void MarkStoppedLocked();
  void TransitionToStopped();
  void AddLifecycleWarning(const std::string& warning);
  void SetTerminalError(const std::string& error);
  void StopTickReception();
  bool HasExpectedTickIdentity(const mujoco_sim::v2::Tick& tick) const;
  bool IsExpectedTick(const mujoco_sim::v2::Tick& tick) const;
  void HandleTickBytes(const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleTick(const mujoco_sim::v2::Tick& tick);
  enum class TickAction { kReject, kHeartbeat, kRetryCommit, kQueue };
  TickAction ClassifyTickLocked(
      const mujoco_sim::v2::Tick& tick,
      std::optional<mujoco_sim::v2::AdapterCommit>* retry_commit);
  bool AcceptKnownTickLocked(
      const mujoco_sim::v2::Tick& tick,
      const std::optional<mujoco_sim::v2::Tick>& known);
  std::optional<std::uint64_t> NewestKnownTickIdLocked(
      const mujoco_sim::v2::Tick& tick) const;
  void RecordTickConflictLocked(std::uint64_t tick_id);

  using CallbackGate = internal::CallbackGate<Impl>;
  using CallbackState = CallbackGate::State;
  using CallbackLease = CallbackGate::Lease;

  AdapterOptions options_;
  mujoco_sim::v2::RegisterAdapterResponse registration_;
  std::shared_ptr<internal::AdapterTransport> transport_;
  std::unique_ptr<eCAL::CSubscriber> tick_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::shared_ptr<CallbackState> callback_state_;
  std::mutex lifecycle_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  std::chrono::milliseconds activation_delay_{100};
  bool monitor_ecal_ = false;
  internal::AdapterSession session_;
  bool wait_in_progress_ = false;
  std::optional<mujoco_sim::v2::Tick> queued_tick_;
  std::optional<mujoco_sim::v2::Tick> outstanding_tick_;
  std::optional<mujoco_sim::v2::Tick> committed_tick_;
  std::optional<mujoco_sim::v2::AdapterCommit> cached_commit_;
  std::string terminal_error_;
  std::string lifecycle_warning_;
  std::uint64_t layout_hash_mismatches_ = 0;
};

}  // namespace mujoco_sim::adapter_sdk
