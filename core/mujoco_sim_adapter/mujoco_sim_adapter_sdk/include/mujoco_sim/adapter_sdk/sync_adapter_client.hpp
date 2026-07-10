#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
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

using TickHandler =
    std::function<std::vector<CommandValue>(const mujoco_sim::v1::Tick&)>;

mujoco_sim::v1::AdapterCommit BuildCommit(
    const mujoco_sim::v1::Tick& tick,
    const std::vector<CommandValue>& commands);
mujoco_sim::v1::Heartbeat BuildHeartbeat(const mujoco_sim::v1::Tick& tick);

class SyncAdapterClient {
 public:
  explicit SyncAdapterClient(AdapterOptions options);
  ~SyncAdapterClient();

  SyncAdapterClient(const SyncAdapterClient&) = delete;
  SyncAdapterClient& operator=(const SyncAdapterClient&) = delete;

  bool Start(std::string* error_message);
  int Run(TickHandler handler, std::uint64_t max_ticks);
  void Stop();

  const mujoco_sim::v1::RegisterControllerResponse& registration() const {
    return registration_;
  }

 private:
  friend class SyncAdapterClientTestPeer;

  template <typename Request, typename Response>
  bool CallService(const std::string& method, const Request& request,
                   Response* response, std::string* error_message);

  bool Register(std::string* error_message);
  bool Activate(std::string* error_message);
  bool BeginTickCallback();
  void EndTickCallback();
  void StopTickReception();
  bool IsExpectedTick(const mujoco_sim::v1::Tick& tick) const;
  void HandleTickBytes(const void* data, std::size_t size);
  void HandleTick(
      const mujoco_sim::v1::Tick& tick,
      const std::function<void(const mujoco_sim::v1::Heartbeat&)>&
          publish_heartbeat,
      const std::function<void(const mujoco_sim::v1::AdapterCommit&)>&
          publish_commit);
  void PublishCommit(const mujoco_sim::v1::AdapterCommit& commit);
  void PublishHeartbeat(const mujoco_sim::v1::Heartbeat& heartbeat);

  class TickCallbackGuard {
   public:
    explicit TickCallbackGuard(SyncAdapterClient* client);
    ~TickCallbackGuard();

    bool accepted() const { return accepted_; }

   private:
    SyncAdapterClient* client_;
    bool accepted_;
  };

  AdapterOptions options_;
  mujoco_sim::v1::RegisterControllerResponse registration_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
  std::unique_ptr<eCAL::CPublisher> commit_publisher_;
  std::unique_ptr<eCAL::CPublisher> heartbeat_publisher_;
  std::unique_ptr<eCAL::CSubscriber> tick_subscriber_;
  TickHandler handler_;
  std::mutex tick_mutex_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  bool owns_ecal_ = false;
  bool accepting_ticks_ = false;
  std::size_t active_tick_callbacks_ = 0;
  std::uint64_t handled_ticks_ = 0;
  bool have_last_commit_ = false;
  mujoco_sim::v1::Tick last_tick_;
  mujoco_sim::v1::AdapterCommit last_commit_;
};

}  // namespace mujoco_sim::adapter_sdk
