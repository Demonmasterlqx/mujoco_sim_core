#include <string>

#include <mujoco_sim/server/server_application.hpp>
#include <mujoco_sim/server/server_config.hpp>

int main() {
  mujoco_sim::server::ServerConfig config;
  config.headless = true;
  mujoco_sim::server::ServerApplication server(config);
  const std::string usage = mujoco_sim::server::Usage();
  const bool headless_screenshot_rejected = !server.RequestScreenshot();
  return usage.find("--model") == std::string::npos || !headless_screenshot_rejected ? 1 : 0;
}
