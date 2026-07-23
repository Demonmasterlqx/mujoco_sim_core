#include "process_signal.hpp"

#include <csignal>

namespace mujoco_sim::sensor_demo {
namespace {

volatile std::sig_atomic_t stop_requested = 0;

void HandleSignal(int) {
  stop_requested = 1;
}

}  // namespace

bool InstallProcessSignalHandlers(std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  stop_requested = 0;
  const auto previous_interrupt = std::signal(SIGINT, HandleSignal);
  if (previous_interrupt == SIG_ERR) {
    *error_message = "failed to install the Sensor Demo SIGINT handler";
    return false;
  }
  if (std::signal(SIGTERM, HandleSignal) == SIG_ERR) {
    if (std::signal(SIGINT, previous_interrupt) == SIG_ERR) {
      *error_message =
          "failed to install the Sensor Demo SIGTERM handler and restore the SIGINT handler";
    } else {
      *error_message = "failed to install the Sensor Demo SIGTERM handler";
    }
    return false;
  }
  return true;
}

bool ProcessStopRequested() noexcept {
  return stop_requested != 0;
}

}  // namespace mujoco_sim::sensor_demo
