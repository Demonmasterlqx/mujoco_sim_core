#pragma once

#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/server.h>
#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>

namespace mujoco_sim::server {

class SimViewer;

class EcalServer {
 public:
  explicit EcalServer(ServerConfig config);
  ~EcalServer();

  EcalServer(const EcalServer&) = delete;
  EcalServer& operator=(const EcalServer&) = delete;

  bool Start(std::string* error_message);
  int Run();
  void Stop();

  const ServerConfig& config() const { return config_; }

 private:
  int RunLoop();
  int RunGraphical();
  void RegisterServices();
  void RegisterSubscribersAndPublishers();
  void PublishTicks(const std::vector<mujoco_sim::v1::Tick>& ticks);
  void PublishStatus();
  std::vector<mujoco_sim::v1::Tick> OpenTickLocked();

  int HandleGetAvailableResources(const std::string& request,
                                  std::string* response);
  int HandleRegisterController(const std::string& request,
                               std::string* response);
  int HandleActivateController(const std::string& request,
                               std::string* response);
  int HandleUnregisterController(const std::string& request,
                                 std::string* response);
  void HandleCommitBytes(const void* data, std::size_t size);
  void HandleHeartbeatBytes(const void* data, std::size_t size);

  ServerConfig config_;
  mujoco_sim::core::SimulationCore core_;
  std::unique_ptr<mujoco_sim::core::SimulationSnapshot> viewer_snapshot_;
  std::unique_ptr<SimViewer> viewer_;
  std::unique_ptr<eCAL::CServiceServer> registry_;
  std::unique_ptr<eCAL::CSubscriber> commit_subscriber_;
  std::unique_ptr<eCAL::CSubscriber> heartbeat_subscriber_;
  std::unique_ptr<eCAL::CPublisher> status_publisher_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CPublisher>>
      tick_publishers_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  bool owns_ecal_ = false;
};

}  // namespace mujoco_sim::server
