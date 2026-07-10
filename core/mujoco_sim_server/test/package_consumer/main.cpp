#include <string>

#include <mujoco_sim/server/server_config.hpp>

int main() {
  const std::string usage = mujoco_sim::server::Usage();
  return usage.find("--model") == std::string::npos ? 1 : 0;
}
