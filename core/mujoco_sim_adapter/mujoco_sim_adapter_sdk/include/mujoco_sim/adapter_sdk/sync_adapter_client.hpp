#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/client.h>
#include <mujoco_sim/v1/sync.pb.h>

namespace mujoco_sim::adapter_sdk {

struct AdapterOptions {
  std::string server_name = "default";
  std::string controller_id;
  std::vector<std::string> observation_names;
  std::vector<std::string> command_names;
  std::uint32_t timeout_ms = 1000;
  std::uint32_t service_timeout_ms = 5000;
};

struct CommandValue {
  std::string name;
  double value = 0.0;
};

enum class TickWaitResult {
  kReceived,
  kTimeout,
  kStopped,
  kError,
};

class SyncAdapterClient {
 public:
  explicit SyncAdapterClient(AdapterOptions options);
  ~SyncAdapterClient();

  SyncAdapterClient(const SyncAdapterClient&) = delete;
  SyncAdapterClient& operator=(const SyncAdapterClient&) = delete;

  bool Start(std::string* error_message);
  bool Activate(std::string* error_message);
  TickWaitResult WaitForTick(std::chrono::milliseconds timeout,
                             mujoco_sim::v1::Tick* tick,
                             std::string* error_message);
  bool Commit(const mujoco_sim::v1::Tick& tick,
              const std::vector<CommandValue>& commands,
              std::string* error_message);
  void Stop();
  bool Unregister(std::string* error_message);
  std::string lifecycle_warning() const;

  const mujoco_sim::v1::RegisterControllerResponse& registration() const {
    return registration_;
  }

 private:
  friend class SyncAdapterClientTestPeer;

  template <typename Request, typename Response>
  bool CallService(const std::string& method, const Request& request,
                   Response* response, std::string* error_message,
                   bool allow_after_stop = false);

  bool Register(std::string* error_message);
  bool FinishStart(const std::function<void()>& unregister,
                   std::string* error_message);
  bool ActivateWith(
      const std::function<bool(
          const mujoco_sim::v1::ActivateControllerRequest&,
          mujoco_sim::v1::ActivateControllerResponse*, std::string*)>& call,
      std::chrono::milliseconds discovery_delay,
      std::string* error_message);
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  bool CommitWith(
      const mujoco_sim::v1::Tick& tick,
      const std::vector<CommandValue>& commands,
      const std::function<bool(const mujoco_sim::v1::AdapterCommit&)>& publish,
      std::string* error_message);
  bool BuildValidatedCommit(
      const mujoco_sim::v1::Tick& tick,
      const std::vector<CommandValue>& commands,
      mujoco_sim::v1::AdapterCommit* commit,
      std::string* error_message) const;
  bool UnregisterWith(
      const std::function<bool(
          const mujoco_sim::v1::UnregisterControllerRequest&,
          mujoco_sim::v1::UnregisterControllerResponse*, std::string*)>& call,
      std::string* error_message);
  bool CompleteUnregister(
      const mujoco_sim::v1::UnregisterControllerResponse& response,
      std::string* error_message);
  void MarkStoppedLocked();
  void TransitionToStopped();
  void AddLifecycleWarning(const std::string& warning);
  void SetTerminalError(const std::string& error);
  void StopTickReception();
  bool IsExpectedTick(const mujoco_sim::v1::Tick& tick) const;
  void HandleTickBytes(const void* data, std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleTick(
      const mujoco_sim::v1::Tick& tick,
      const std::function<void(const mujoco_sim::v1::Heartbeat&)>&
          publish_heartbeat,
      const std::function<void(const mujoco_sim::v1::AdapterCommit&)>&
          publish_commit);
  bool PublishCommit(const mujoco_sim::v1::AdapterCommit& commit);
  void PublishHeartbeat(const mujoco_sim::v1::Heartbeat& heartbeat);
  bool PublishSerialized(const std::function<bool()>& publish);

  struct CallbackState {
    std::mutex mutex;
    std::condition_variable condition;
    SyncAdapterClient* client = nullptr;
    bool enabled = false;
    std::size_t active_callbacks = 0;
  };

  class CallbackLease {
   public:
    explicit CallbackLease(std::shared_ptr<CallbackState> state);
    ~CallbackLease();

    SyncAdapterClient* client() const { return client_; }

   private:
    std::shared_ptr<CallbackState> state_;
    SyncAdapterClient* client_ = nullptr;
  };

  AdapterOptions options_;
  mujoco_sim::v1::RegisterControllerResponse registration_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
  std::unique_ptr<eCAL::CPublisher> commit_publisher_;
  std::unique_ptr<eCAL::CPublisher> heartbeat_publisher_;
  std::unique_ptr<eCAL::CSubscriber> tick_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::shared_ptr<CallbackState> callback_state_;
  std::mutex lifecycle_mutex_;
  std::mutex publisher_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  bool monitor_ecal_ = false;
  bool started_ = false;
  bool activated_ = false;
  bool registered_ = false;
  bool accepting_ticks_ = false;
  bool wait_in_progress_ = false;
  std::optional<mujoco_sim::v1::Tick> queued_tick_;
  std::optional<mujoco_sim::v1::Tick> outstanding_tick_;
  std::optional<mujoco_sim::v1::Tick> committed_tick_;
  std::optional<mujoco_sim::v1::AdapterCommit> cached_commit_;
  std::string terminal_error_;
  std::string lifecycle_warning_;
};

}  // namespace mujoco_sim::adapter_sdk
