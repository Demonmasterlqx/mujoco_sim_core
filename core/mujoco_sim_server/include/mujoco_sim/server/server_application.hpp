#pragma once

#include <memory>
#include <string>

#include <mujoco_sim/server/server_config.hpp>

namespace mujoco_sim::server {

/// Owns the server process runtime and its single-writer simulation loop.
///
/// Start() initializes all resources and starts the simulation thread. Run()
/// blocks until that thread exits, or runs the GUI loop on the calling thread.
/// RequestStop() is idempotent and may be called concurrently from any thread.
class ServerApplication {
 public:
  explicit ServerApplication(ServerConfig config);
  ~ServerApplication();

  ServerApplication(const ServerApplication&) = delete;
  ServerApplication& operator=(const ServerApplication&) = delete;

  bool Start(std::string* error_message);
  int Run();
  void RequestStop(std::string reason = "external_stop");

  const ServerConfig& config() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
