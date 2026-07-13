#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/client.h>
#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::adapter_sdk {

struct ObserverOptions {
  std::string server_name = "default";
  std::string observer_id;
  std::vector<std::string> observation_names;
  std::uint32_t timeout_ms = 1000;
  std::uint32_t service_timeout_ms = 5000;
  std::uint32_t keepalive_period_ms = 250;
  std::optional<std::uint32_t> every_n_steps = 1;
  std::optional<double> period_s;
  std::size_t queue_capacity = 1;
};

enum class ObservationWaitResult {
  kReceived,
  kTimeout,
  kStopped,
  kError,
};

class ObserverClient {
 public:
  explicit ObserverClient(ObserverOptions options);
  ~ObserverClient();

  ObserverClient(const ObserverClient&) = delete;
  ObserverClient& operator=(const ObserverClient&) = delete;

  bool Start(std::string* error_message);
  bool Activate(std::string* error_message);
  ObservationWaitResult WaitForObservation(
      std::chrono::milliseconds timeout,
      mujoco_sim::v2::ObservationFrame* frame,
      std::string* error_message);
  void Stop();
  bool Unregister(std::string* error_message);

  std::uint64_t dropped_frames() const;
  std::string lifecycle_warning() const;

  const mujoco_sim::v2::RegisterAdapterResponse& registration() const {
    return registration_;
  }

 private:
  friend class ObserverClientTestPeer;

  template <typename Request, typename Response>
  bool CallService(const std::string& method, const Request& request,
                   Response* response, std::string* error_message,
                   bool allow_after_stop = false);

  bool ValidateOptions(std::string* error_message) const;
  bool Register(std::string* error_message);
  bool FinishStart(const std::function<void()>& unregister,
                   std::string* error_message);
  bool ActivateWith(
      const std::function<bool(
          const mujoco_sim::v2::ActivateAdapterRequest&,
          mujoco_sim::v2::ActivateAdapterResponse*, std::string*)>& call,
      std::chrono::milliseconds discovery_delay,
      std::string* error_message);
  bool WaitBeforeActivation(std::chrono::milliseconds delay);
  bool UnregisterWith(
      const std::function<bool(
          const mujoco_sim::v2::UnregisterAdapterRequest&,
          mujoco_sim::v2::UnregisterAdapterResponse*, std::string*)>& call,
      std::string* error_message);
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
  void PublishHeartbeat(const mujoco_sim::v2::AdapterHeartbeat& heartbeat);

  struct CallbackState {
    std::mutex mutex;
    std::condition_variable condition;
    ObserverClient* client = nullptr;
    bool enabled = false;
    std::size_t active_callbacks = 0;
  };

  class CallbackLease {
   public:
    explicit CallbackLease(std::shared_ptr<CallbackState> state);
    ~CallbackLease();

    ObserverClient* client() const { return client_; }

   private:
    std::shared_ptr<CallbackState> state_;
    ObserverClient* client_ = nullptr;
  };

  ObserverOptions options_;
  mujoco_sim::v2::RegisterAdapterResponse registration_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
  std::unique_ptr<eCAL::CPublisher> heartbeat_publisher_;
  std::unique_ptr<eCAL::CSubscriber> observation_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::shared_ptr<CallbackState> callback_state_;
  std::mutex lifecycle_mutex_;
  std::mutex publisher_mutex_;
  std::mutex heartbeat_thread_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::condition_variable heartbeat_condition_;
  std::thread heartbeat_thread_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  bool monitor_ecal_ = false;
  bool started_ = false;
  bool activated_ = false;
  bool registered_ = false;
  bool accepting_observations_ = false;
  bool wait_in_progress_ = false;
  std::deque<mujoco_sim::v2::ObservationFrame> observations_;
  std::uint64_t last_sequence_ = 0;
  std::uint64_t last_tick_id_ = 0;
  std::uint64_t dropped_frames_ = 0;
  std::string terminal_error_;
  std::string lifecycle_warning_;
};

}  // namespace mujoco_sim::adapter_sdk
