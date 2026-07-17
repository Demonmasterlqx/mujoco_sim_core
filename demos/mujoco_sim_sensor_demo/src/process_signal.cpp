#include "process_signal.hpp"

#include <csignal>

namespace mujoco_sim::sensor_demo {
namespace {

volatile std::sig_atomic_t stop_requested = 0;

void HandleSignal(int) {
  stop_requested = 1;
}

}  // namespace

void InstallProcessSignalHandlers() {
  stop_requested = 0;
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
}

bool ProcessStopRequested() noexcept {
  return stop_requested != 0;
}

}  // namespace mujoco_sim::sensor_demo
