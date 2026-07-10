#include <string>

#include <mujoco_sim/server/ecal_server.hpp>
#include <mujoco_sim/server/server_config.hpp>

int main() {
  mujoco_sim::server::ServerConfig config;
  config.headless = true;
  mujoco_sim::server::EcalServer server(config);
  const std::string usage = mujoco_sim::server::Usage();
  return usage.find("--model") == std::string::npos ? 1 : 0;
}
