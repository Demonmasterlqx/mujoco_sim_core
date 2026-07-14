#pragma once

#include <cstdint>
#include <string>

#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::core::internal {

struct ControllerRecord;
struct ObserverRecord;

enum class ControllerPhase {
  kProvisional,
  kActive,
  kTimedOutAwaitingProof,
  kRecoveryPending,
  kError,
};

enum class ObserverPhase {
  kProvisional,
  kActive,
  kTimedOut,
  kError,
};

mujoco_sim::v2::ControllerState WireState(ControllerPhase phase) noexcept;
mujoco_sim::v2::ControllerState WireState(ObserverPhase phase) noexcept;

class ControllerStateMachine {
 public:
  static bool CanActivate(ControllerPhase phase) noexcept;
  static bool AcceptsBarrierCommit(ControllerPhase phase) noexcept;
  static bool AcceptsRecoveryCommit(ControllerPhase phase) noexcept;
  static bool IsTickRecipient(ControllerPhase phase) noexcept;

  static bool Activate(ControllerRecord* controller, std::uint64_t tick_id);
  static bool AcceptRecoveryProof(
      ControllerRecord* controller,
      const mujoco_sim::v2::AdapterCommit& commit);
  static bool ActivateRecovered(ControllerRecord* controller,
                                std::uint64_t tick_id);
  static bool MarkTimedOut(ControllerRecord* controller,
                           std::uint64_t tick_id,
                           const std::string& message);
  static bool MarkError(ControllerRecord* controller, std::uint64_t tick_id,
                        const std::string& message);
};

class ObserverStateMachine {
 public:
  static bool Activate(ObserverRecord* observer, std::uint64_t tick_id);
  static bool MarkTimedOut(ObserverRecord* observer, std::uint64_t tick_id,
                           const std::string& message);
  static bool MarkError(ObserverRecord* observer, std::uint64_t tick_id,
                        const std::string& message);
};

}  // namespace mujoco_sim::core::internal
