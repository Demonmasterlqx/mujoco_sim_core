#include <string>

#include <mujoco_sim/server/server_application.hpp>
#include <mujoco_sim/server/server_config.hpp>

int main() {
  mujoco_sim::server::ServerConfig config;
  config.headless = true;
  config.viewer_sync_period_ms = 16;
  config.viewer_control_input_enabled = false;
  config.viewer_mouse_force_enabled = false;
  config.viewer_mouse_force_timeout_ms = 100;
  mujoco_sim::server::ServerApplication server(config);
  const std::string usage = mujoco_sim::server::Usage();
  const bool headless_screenshot_rejected = !server.RequestScreenshot();
  return usage.find("--model") == std::string::npos ||
                 usage.find("--viewer-sync-period-ms") == std::string::npos ||
                 !headless_screenshot_rejected
             ? 1
             : 0;
}
