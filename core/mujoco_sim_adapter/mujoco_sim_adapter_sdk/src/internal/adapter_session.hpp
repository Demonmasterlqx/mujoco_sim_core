#pragma once

#include <cstdint>

namespace mujoco_sim::adapter_sdk::internal {

enum class ClientPhase {
  kConstructed,
  kStarting,
  kProvisional,
  kActivating,
  kActive,
  kStopped,
  kTerminalError,
};

struct SessionIdentity {
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
};

// Callers serialize transitions with their lifecycle/state mutexes. Remote
// session ownership is intentionally independent from the local terminal
// outcome so Unregister remains retryable after Stop or a terminal error.
class AdapterSession {
 public:
  ClientPhase phase() const { return phase_; }
  bool started() const {
    return phase_ == ClientPhase::kProvisional || phase_ == ClientPhase::kActivating ||
           phase_ == ClientPhase::kActive;
  }
  bool active() const { return phase_ == ClientPhase::kActive; }
  bool accepting_data() const { return phase_ == ClientPhase::kActivating || active(); }
  bool has_session() const { return has_session_; }
  const SessionIdentity& identity() const { return identity_; }
  bool stopped() const { return phase_ == ClientPhase::kStopped; }
  bool terminal_error() const { return phase_ == ClientPhase::kTerminalError; }

  bool Registered(SessionIdentity identity) {
    identity_ = identity;
    has_session_ = true;
    if (phase_ == ClientPhase::kStopped || terminal_error()) return false;
    phase_ = ClientPhase::kStarting;
    return true;
  }
  bool StartCompleted() {
    if (phase_ != ClientPhase::kStarting) return false;
    phase_ = ClientPhase::kProvisional;
    return true;
  }
  bool BeginActivation() {
    if (phase_ != ClientPhase::kProvisional || !has_session()) return false;
    phase_ = ClientPhase::kActivating;
    return true;
  }
  bool ActivationRejected() {
    if (phase_ != ClientPhase::kActivating) return false;
    phase_ = ClientPhase::kProvisional;
    return true;
  }
  bool Activated() {
    if (phase_ != ClientPhase::kActivating) return false;
    phase_ = ClientPhase::kActive;
    return true;
  }
  void Stop() {
    if (!terminal_error()) phase_ = ClientPhase::kStopped;
  }
  void Fail() { phase_ = ClientPhase::kTerminalError; }
  void ReleaseSession() {
    has_session_ = false;
    Stop();
  }

 private:
  ClientPhase phase_ = ClientPhase::kConstructed;
  SessionIdentity identity_;
  bool has_session_ = false;
};

}  // namespace mujoco_sim::adapter_sdk::internal
