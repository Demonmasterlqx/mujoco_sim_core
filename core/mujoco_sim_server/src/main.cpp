#include <iostream>
#include <string>

#include <mujoco_sim/server/ecal_server.hpp>
#include <mujoco_sim/server/server_config.hpp>

int main(int argc, char** argv) {
  mujoco_sim::server::ServerConfig config;
  std::string error;
  if (!mujoco_sim::server::ParseCommandLine(argc, argv, &config, &error)) {
    if (error == mujoco_sim::server::Usage()) {
      std::cout << error;
      return 0;
    }
    std::cerr << error << '\n';
    return 1;
  }

  mujoco_sim::server::EcalServer server(config);
  if (!server.Start(&error)) {
    return 1;
  }
  return server.Run();
}
