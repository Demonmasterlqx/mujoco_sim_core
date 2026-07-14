#pragma once

#include <cstdint>
#include <string>

#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::adapter_sdk::internal {

enum class StatusAction { kIgnore, kContinue, kStopped, kError };

struct StatusDecision {
  StatusAction action = StatusAction::kIgnore;
  std::string message;
};

inline bool HasExpectedStatusIdentity(
    const mujoco_sim::v2::ServerStatus& status,
    const std::string& server_name, std::uint64_t epoch) {
  const auto& header = status.header();
  return header.server_name() == server_name && header.epoch() == epoch &&
         header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
         header.protocol_minor() == mujoco_sim::protocol::kProtocolMinor;
}

inline StatusDecision EvaluateServerState(
    const mujoco_sim::v2::ServerStatus& status) {
  if (status.state() == mujoco_sim::v2::STOPPED) {
    return {StatusAction::kStopped, {}};
  }
  if (status.state() == mujoco_sim::v2::ERROR_PAUSED) {
    return {StatusAction::kError,
            status.message().empty() ? "server entered ERROR_PAUSED"
                                     : status.message()};
  }
  return {StatusAction::kContinue, {}};
}

inline StatusDecision EvaluateAdapterEntry(
    const mujoco_sim::v2::AdapterStatus& adapter,
    std::uint64_t session_id, bool timed_out_is_terminal) {
  if (adapter.session_id() != session_id) {
    return {StatusAction::kError, "server replaced adapter session"};
  }
  const bool terminal = adapter.state() == mujoco_sim::v2::ERROR ||
                        (timed_out_is_terminal &&
                         adapter.state() == mujoco_sim::v2::TIMED_OUT);
  if (!terminal) return {StatusAction::kContinue, {}};

  return {StatusAction::kError,
          adapter.message().empty()
              ? "server marked adapter as " +
                    mujoco_sim::v2::ControllerState_Name(adapter.state())
              : adapter.message()};
}

inline StatusDecision EvaluateAdapterStatus(
    const mujoco_sim::v2::ServerStatus& status,
    const std::string& server_name, std::uint64_t epoch,
    const std::string& adapter_id, std::uint64_t session_id,
    mujoco_sim::v2::AdapterRole role, bool timed_out_is_terminal) {
  if (!HasExpectedStatusIdentity(status, server_name, epoch)) return {};
  const auto server_decision = EvaluateServerState(status);
  if (server_decision.action != StatusAction::kContinue) return server_decision;

  for (const auto& adapter : status.adapters()) {
    if (adapter.adapter_id() != adapter_id || adapter.role() != role) continue;
    return EvaluateAdapterEntry(adapter, session_id, timed_out_is_terminal);
  }
  return {StatusAction::kContinue, {}};
}

}  // namespace mujoco_sim::adapter_sdk::internal
