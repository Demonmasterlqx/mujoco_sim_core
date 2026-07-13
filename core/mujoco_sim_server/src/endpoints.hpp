#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <cstddef>
#include <functional>
#include <vector>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/v2/sync.pb.h>

#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/server.h>

namespace mujoco_sim::server {

class RegistryEndpoint {
 public:
  using ServiceHandler =
      std::function<int(const std::string&, std::string*)>;
  using DataHandler = std::function<void(const void*, std::size_t)>;

  explicit RegistryEndpoint(const std::string& service_name);

  bool RegisterMethod(const std::string& method, ServiceHandler handler);
  bool Parse(const std::string& bytes,
             mujoco_sim::v2::GetAvailableResourcesRequest* message) const;
  bool Parse(const std::string& bytes,
             mujoco_sim::v2::RegisterAdapterRequest* message) const;
  bool Parse(const std::string& bytes,
             mujoco_sim::v2::ActivateAdapterRequest* message) const;
  bool Parse(const std::string& bytes,
             mujoco_sim::v2::UnregisterAdapterRequest* message) const;
  bool Parse(const std::string& bytes,
             mujoco_sim::v2::GetSensorStreamsRequest* message) const;
  bool ParseHeartbeat(const void* data, std::size_t size,
                      mujoco_sim::v2::AdapterHeartbeat* message) const;
  template <typename Message>
  int Serialize(const Message& message, std::string* bytes) const {
    return message.SerializeToString(bytes) ? 0 : 1;
  }
  void SubscribeHeartbeat(const std::string& topic, DataHandler handler);
  void StopCallbacks();
  void ResetService();

 private:
  std::unique_ptr<eCAL::CServiceServer> service_;
  std::unique_ptr<eCAL::CSubscriber> heartbeat_subscriber_;
  std::vector<std::string> methods_;
};

class ControlEndpoint {
 public:
  using DataHandler = std::function<void(const void*, std::size_t)>;
  struct PublishResult {
    std::string topic;
    std::size_t subscribers_before = 0;
    std::size_t subscribers_after = 0;
    bool attempted = false;
    bool sent = false;
    bool publisher_created = false;
    bool serialization_failed = false;
    std::size_t bytes = 0;
    std::string transport_error;
  };

  void SubscribeCommits(const std::string& topic, DataHandler handler);
  bool ParseCommit(const void* data, std::size_t size,
                   mujoco_sim::v2::AdapterCommit* message) const;
  void StopSubscriber();
  void ClearPublishers();
  void ErasePublisher(const std::string& adapter_id);
  PublishResult Publish(const mujoco_sim::core::IssuedTickHandle& issued,
                        const std::string& server_name);

 private:
  std::unique_ptr<eCAL::CSubscriber> commit_subscriber_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>>
      publishers_;
  std::mutex publisher_mutex_;
};

class ObservationEndpoint {
 public:
  using PublishResult = ControlEndpoint::PublishResult;
  void ClearPublishers();
  void ErasePublisher(const std::string& adapter_id);
  PublishResult Publish(const mujoco_sim::v2::ObservationFrame& frame,
                        const std::string& server_name);

 private:
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>>
      publishers_;
  std::mutex publisher_mutex_;
};

class StatusEndpoint {
 public:
  using PublishResult = ControlEndpoint::PublishResult;
  explicit StatusEndpoint(const std::string& topic);
  bool available() const noexcept { return publisher_ != nullptr; }
  PublishResult Publish(const mujoco_sim::v2::ServerStatus& status);
  void Reset();

 private:
  std::unique_ptr<eCAL::CPublisher> publisher_;
  std::mutex publisher_mutex_;
};

}  // namespace mujoco_sim::server
