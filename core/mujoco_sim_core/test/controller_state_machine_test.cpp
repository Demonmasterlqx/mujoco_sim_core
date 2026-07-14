#include <gtest/gtest.h>

#include <array>

#include "internal/adapter_registry.hpp"
#include "internal/controller_state_machine.hpp"

namespace {

using mujoco_sim::core::internal::ControllerPhase;
using mujoco_sim::core::internal::ControllerRecord;
using mujoco_sim::core::internal::ControllerStateMachine;
using mujoco_sim::core::internal::ObserverPhase;
using mujoco_sim::core::internal::ObserverRecord;
using mujoco_sim::core::internal::ObserverStateMachine;
using mujoco_sim::core::internal::WireState;

struct PhaseExpectation {
  ControllerPhase phase;
  bool can_activate;
  bool accepts_barrier_commit;
  bool accepts_recovery_commit;
  bool is_tick_recipient;
};

enum class ControllerEvent {
  kActivate,
  kRecoveryProof,
  kActivateRecovered,
  kTimeout,
  kError,
};

bool Apply(ControllerEvent event, ControllerRecord* controller) {
  mujoco_sim::v2::AdapterCommit proof;
  proof.mutable_header()->set_tick_id(4);
  switch (event) {
    case ControllerEvent::kActivate:
      return ControllerStateMachine::Activate(controller, 4);
    case ControllerEvent::kRecoveryProof:
      return ControllerStateMachine::AcceptRecoveryProof(controller, proof);
    case ControllerEvent::kActivateRecovered:
      return ControllerStateMachine::ActivateRecovered(controller, 4);
    case ControllerEvent::kTimeout:
      return ControllerStateMachine::MarkTimedOut(controller, 4, "timeout");
    case ControllerEvent::kError:
      return ControllerStateMachine::MarkError(controller, 4, "error");
  }
  return false;
}

enum class ObserverEvent { kActivate, kTimeout, kError };

bool Apply(ObserverEvent event, ObserverRecord* observer) {
  switch (event) {
    case ObserverEvent::kActivate:
      return ObserverStateMachine::Activate(observer, 4);
    case ObserverEvent::kTimeout:
      return ObserverStateMachine::MarkTimedOut(observer, 4, "timeout");
    case ObserverEvent::kError:
      return ObserverStateMachine::MarkError(observer, 4, "error");
  }
  return false;
}

TEST(ControllerStateMachineTest, DefinesCompletePhaseEventMatrix) {
  constexpr std::array expectations{
      PhaseExpectation{ControllerPhase::kProvisional, true, false, false,
                       false},
      PhaseExpectation{ControllerPhase::kActive, false, true, false, true},
      PhaseExpectation{ControllerPhase::kTimedOutAwaitingProof, false, false,
                       true, true},
      PhaseExpectation{ControllerPhase::kRecoveryPending, false, false, true,
                       true},
      PhaseExpectation{ControllerPhase::kError, false, false, false, false},
  };

  for (const auto& expected : expectations) {
    EXPECT_EQ(ControllerStateMachine::CanActivate(expected.phase),
              expected.can_activate);
    EXPECT_EQ(ControllerStateMachine::AcceptsBarrierCommit(expected.phase),
              expected.accepts_barrier_commit);
    EXPECT_EQ(ControllerStateMachine::AcceptsRecoveryCommit(expected.phase),
              expected.accepts_recovery_commit);
    EXPECT_EQ(ControllerStateMachine::IsTickRecipient(expected.phase),
              expected.is_tick_recipient);
  }
}

TEST(ControllerStateMachineTest, EnforcesEveryControllerTransition) {
  constexpr std::array phases{
      ControllerPhase::kProvisional, ControllerPhase::kActive,
      ControllerPhase::kTimedOutAwaitingProof,
      ControllerPhase::kRecoveryPending, ControllerPhase::kError};
  constexpr std::array events{
      ControllerEvent::kActivate, ControllerEvent::kRecoveryProof,
      ControllerEvent::kActivateRecovered, ControllerEvent::kTimeout,
      ControllerEvent::kError};

  for (const auto phase : phases) {
    for (const auto event : events) {
      const bool expected =
          (event == ControllerEvent::kActivate &&
           phase == ControllerPhase::kProvisional) ||
          (event == ControllerEvent::kRecoveryProof &&
           (phase == ControllerPhase::kTimedOutAwaitingProof ||
            phase == ControllerPhase::kRecoveryPending)) ||
          (event == ControllerEvent::kActivateRecovered &&
           phase == ControllerPhase::kRecoveryPending) ||
          (event == ControllerEvent::kTimeout &&
           phase == ControllerPhase::kActive) ||
          (event == ControllerEvent::kError &&
           phase != ControllerPhase::kError);
      ControllerRecord controller;
      controller.phase = phase;
      controller.state_since_tick = 2;
      controller.message = "original";

      EXPECT_EQ(Apply(event, &controller), expected);
      if (!expected) {
        EXPECT_EQ(controller.phase, phase);
        EXPECT_EQ(controller.state_since_tick, 2U);
        EXPECT_EQ(controller.message, "original");
      }
    }
  }
}

TEST(ControllerStateMachineTest, AppliesControllerLifecycleTransitions) {
  ControllerRecord controller;
  EXPECT_TRUE(ControllerStateMachine::Activate(&controller, 3));
  EXPECT_EQ(controller.phase, ControllerPhase::kActive);
  EXPECT_EQ(controller.state_since_tick, 3U);

  EXPECT_TRUE(ControllerStateMachine::MarkTimedOut(
      &controller, 7, "barrier timeout"));
  EXPECT_EQ(controller.phase, ControllerPhase::kTimedOutAwaitingProof);
  EXPECT_EQ(WireState(controller.phase), mujoco_sim::v2::TIMED_OUT);
  EXPECT_EQ(controller.timed_out_since_tick, 7U);
  EXPECT_EQ(controller.timeout_count, 1U);
  EXPECT_EQ(controller.message, "barrier timeout");

  mujoco_sim::v2::AdapterCommit proof;
  proof.mutable_header()->set_tick_id(7);
  EXPECT_TRUE(
      ControllerStateMachine::AcceptRecoveryProof(&controller, proof));
  EXPECT_EQ(controller.phase, ControllerPhase::kRecoveryPending);
  ASSERT_TRUE(controller.recovery_proof.has_value());
  EXPECT_EQ(controller.recovery_proof->header().tick_id(), 7U);
  EXPECT_EQ(WireState(controller.phase), mujoco_sim::v2::TIMED_OUT);

  EXPECT_TRUE(ControllerStateMachine::ActivateRecovered(&controller, 8));
  EXPECT_EQ(controller.phase, ControllerPhase::kActive);
  EXPECT_EQ(controller.state_since_tick, 8U);
  EXPECT_TRUE(controller.message.empty());

  EXPECT_TRUE(
      ControllerStateMachine::MarkError(&controller, 9, "invalid commit"));
  EXPECT_EQ(controller.phase, ControllerPhase::kError);
  EXPECT_EQ(controller.state_since_tick, 9U);
  EXPECT_EQ(controller.message, "invalid commit");
  EXPECT_FALSE(ControllerStateMachine::Activate(&controller, 10));
}

TEST(ControllerStateMachineTest, AppliesObserverLifecycleTransitions) {
  ObserverRecord observer;
  EXPECT_TRUE(ObserverStateMachine::Activate(&observer, 2));
  EXPECT_EQ(observer.phase, ObserverPhase::kActive);
  EXPECT_EQ(observer.activation_tick, 2U);

  EXPECT_TRUE(
      ObserverStateMachine::MarkTimedOut(&observer, 5, "keepalive timeout"));
  EXPECT_EQ(observer.phase, ObserverPhase::kTimedOut);
  EXPECT_EQ(observer.state_since_tick, 5U);
  EXPECT_EQ(observer.timeout_count, 1U);
  EXPECT_EQ(observer.message, "keepalive timeout");

  EXPECT_TRUE(ObserverStateMachine::MarkError(&observer, 6, "server failed"));
  EXPECT_EQ(observer.phase, ObserverPhase::kError);
  EXPECT_EQ(observer.state_since_tick, 6U);
  EXPECT_EQ(observer.message, "server failed");
}

TEST(ControllerStateMachineTest, EnforcesEveryObserverTransition) {
  constexpr std::array phases{
      ObserverPhase::kProvisional, ObserverPhase::kActive,
      ObserverPhase::kTimedOut, ObserverPhase::kError};
  constexpr std::array events{ObserverEvent::kActivate, ObserverEvent::kTimeout,
                              ObserverEvent::kError};
  for (const auto phase : phases) {
    for (const auto event : events) {
      const bool expected =
          (event == ObserverEvent::kActivate &&
           phase == ObserverPhase::kProvisional) ||
          (event == ObserverEvent::kTimeout &&
           phase == ObserverPhase::kActive) ||
          (event == ObserverEvent::kError &&
           phase != ObserverPhase::kError);
      ObserverRecord observer;
      observer.phase = phase;
      observer.state_since_tick = 2;
      observer.message = "original";

      EXPECT_EQ(Apply(event, &observer), expected);
      if (!expected) {
        EXPECT_EQ(observer.phase, phase);
        EXPECT_EQ(observer.state_since_tick, 2U);
        EXPECT_EQ(observer.message, "original");
      }
    }
  }
}

}  // namespace
