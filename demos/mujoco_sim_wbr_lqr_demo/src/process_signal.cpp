#include "process_signal.hpp"

#include <cstdio>

namespace mujoco_sim::wbr_lqr_demo {
namespace {

volatile std::sig_atomic_t stop_requested = 0;

void HandleTerminationSignal(int) {
  stop_requested = 1;
}

}  // namespace

std::unique_ptr<ProcessSignalGuard> ProcessSignalGuard::Install(std::string* error_message) {
  if (error_message == nullptr) return nullptr;
  error_message->clear();
  stop_requested = 0;
  const Handler previous_interrupt = std::signal(SIGINT, HandleTerminationSignal);
  if (previous_interrupt == SIG_ERR) {
    *error_message = "failed to install WBR adapter SIGINT handler";
    return nullptr;
  }
  const Handler previous_terminate = std::signal(SIGTERM, HandleTerminationSignal);
  if (previous_terminate == SIG_ERR) {
    if (std::signal(SIGINT, previous_interrupt) == SIG_ERR) {
      *error_message = "failed to install WBR adapter SIGTERM handler and restore SIGINT handler";
    } else {
      *error_message = "failed to install WBR adapter SIGTERM handler";
    }
    return nullptr;
  }
  return std::unique_ptr<ProcessSignalGuard>(
      new ProcessSignalGuard(previous_interrupt, previous_terminate));
}

ProcessSignalGuard::~ProcessSignalGuard() noexcept {
  if (restored_) return;
  try {
    std::string error;
    if (!Restore(&error) && !error.empty()) {
      std::fputs(error.c_str(), stderr);
      std::fputc('\n', stderr);
    }
  } catch (...) {
  }
}

bool ProcessSignalGuard::StopRequested() noexcept {
  return stop_requested != 0;
}

bool ProcessSignalGuard::Restore(std::string* error_message) noexcept {
  if (error_message == nullptr) return false;
  error_message->clear();
  if (restored_) return true;

  const bool interrupt_restored = std::signal(SIGINT, previous_interrupt_) != SIG_ERR;
  const bool terminate_restored = std::signal(SIGTERM, previous_terminate_) != SIG_ERR;
  restored_ = interrupt_restored && terminate_restored;
  if (!restored_) {
    try {
      if (!interrupt_restored && !terminate_restored) {
        *error_message = "failed to restore WBR adapter SIGINT and SIGTERM handlers";
      } else if (!interrupt_restored) {
        *error_message = "failed to restore WBR adapter SIGINT handler";
      } else {
        *error_message = "failed to restore WBR adapter SIGTERM handler";
      }
    } catch (...) {
      error_message->clear();
    }
  }
  return restored_;
}

}  // namespace mujoco_sim::wbr_lqr_demo
