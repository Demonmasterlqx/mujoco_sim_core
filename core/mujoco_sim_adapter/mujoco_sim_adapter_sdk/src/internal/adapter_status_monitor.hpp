#pragma once

#include <cstdint>
#include <string>

#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::adapter_sdk::internal {

/** @brief Action selected after evaluating a Server status message.
 *         评估服务器状态消息后选择的动作。
 */
enum class StatusAction {
  /// Ignore a message with unrelated identity. 忽略身份无关的消息。
  kIgnore,
  /// Keep the client session running. 保持客户端会话运行。
  kContinue,
  /// End waits normally because the Server stopped. 因服务器停止而正常结束等待。
  kStopped,
  /// End waits with a sticky remote lifecycle error. 以粘性远端生命周期错误结束等待。
  kError
};

/** @brief Carries a status action, diagnostic, and match evidence.
 *         携带状态动作、诊断与匹配证据。
 */
struct StatusDecision {
  /// Selected client action. 选定的客户端动作。
  StatusAction action = StatusAction::kIgnore;
  /// Owned terminal diagnostic for kError; empty for other actions.
  /// kError 的自有终止诊断；其他动作时为空。
  std::string message;
  /// Whether this message contained the exact current session.
  /// 本消息是否包含精确匹配的当前会话。
  bool matched_session = false;
};

/** @brief Checks only the model epoch before inspecting Server status.
 *         在检查服务器状态前仅校验模型 epoch。
 * @param status Borrowed parsed status. 借用的已解析状态。
 * @param epoch Expected nonzero model epoch. 预期的非零模型 epoch。
 * @return true for the expected epoch. epoch 符合预期时返回 true。
 */
inline bool HasExpectedStatusIdentity(const mujoco_sim::v4::ServerStatus& status,
                                      std::uint64_t epoch) {
  return status.epoch() == epoch;
}

/** @brief Maps global Server state to a client action.
 *         将全局服务器状态映射为客户端动作。
 * @param status Borrowed parsed status with an already validated epoch.
 *               epoch 已校验的借用解析状态。
 * @return STOPPED as kStopped, ERROR_PAUSED as kError, otherwise kContinue.
 *         STOPPED 映射为 kStopped，ERROR_PAUSED 映射为 kError，其余为 kContinue。
 */
inline StatusDecision EvaluateServerState(const mujoco_sim::v4::ServerStatus& status) {
  if (status.state() == mujoco_sim::v4::STOPPED) {
    return {StatusAction::kStopped, {}, false};
  }
  if (status.state() == mujoco_sim::v4::ERROR_PAUSED) {
    return {StatusAction::kError,
            status.message().empty() ? "server entered ERROR_PAUSED" : status.message(), false};
  }
  return {StatusAction::kContinue, {}};
}

/** @brief Evaluates one matching Adapter ID/role entry against session identity.
 *         根据会话标识评估一条 ID/角色匹配的 Adapter 状态。
 * @param adapter Borrowed status entry. 借用的状态条目。
 * @param session_id Expected current session ID. 预期的当前会话 ID。
 * @param timed_out_is_terminal Whether OFFLINE is terminal for this client role.
 *                              OFFLINE 对该客户端角色是否为终止状态。
 * @return Continue for live exact session; error for replacement or terminal state.
 *         精确匹配且存活时继续；会话替换或终止状态时返回错误。
 */
inline StatusDecision EvaluateAdapterEntry(const mujoco_sim::v4::AdapterStatus& adapter,
                                           std::uint64_t session_id, bool timed_out_is_terminal) {
  if (adapter.session_id() != session_id) {
    return {StatusAction::kError, "server replaced adapter session", false};
  }
  const bool terminal = adapter.state() == mujoco_sim::v4::ERROR ||
                        adapter.state() == mujoco_sim::v4::UNREGISTERED ||
                        (timed_out_is_terminal && adapter.state() == mujoco_sim::v4::OFFLINE);
  if (!terminal) return {StatusAction::kContinue, {}, true};

  return {StatusAction::kError,
          adapter.message().empty()
              ? "server marked adapter as " + mujoco_sim::v4::AdapterState_Name(adapter.state())
              : adapter.message(),
          true};
}

/**
 * @brief Converges Server status into an identity-aware client lifecycle decision.
 *        将服务器状态收敛为身份感知的客户端生命周期决策。
 * @param status Borrowed parsed status. 借用的已解析状态。
 * @param epoch Expected model epoch. 预期模型 epoch。
 * @param adapter_id Expected Adapter ID. 预期 Adapter ID。
 * @param session_id Expected session ID. 预期会话 ID。
 * @param role Expected Adapter role. 预期 Adapter 角色。
 * @param timed_out_is_terminal Whether OFFLINE is terminal. OFFLINE 是否为终止状态。
 * @param session_was_seen Whether an earlier status contained the exact session.
 *                         先前状态是否曾包含精确会话。
 * @return Decision that ignores wrong epochs/identity before first match, but
 *         treats replacement or disappearance after a match as terminal.
 *         首次匹配前忽略错误 epoch/身份，但将匹配后的替换或消失视为终止状态。
 */
inline StatusDecision EvaluateAdapterStatus(const mujoco_sim::v4::ServerStatus& status,
                                            std::uint64_t epoch, const std::string& adapter_id,
                                            std::uint64_t session_id,
                                            mujoco_sim::v4::AdapterRole role,
                                            bool timed_out_is_terminal, bool session_was_seen) {
  if (!HasExpectedStatusIdentity(status, epoch)) return {};
  const auto server_decision = EvaluateServerState(status);
  if (server_decision.action != StatusAction::kContinue) return server_decision;

  for (const auto& adapter : status.adapters()) {
    if (adapter.adapter_id() != adapter_id || adapter.role() != role) continue;
    return EvaluateAdapterEntry(adapter, session_id, timed_out_is_terminal);
  }
  if (session_was_seen) {
    return {StatusAction::kError, "adapter session disappeared from ServerStatus", false};
  }
  return {StatusAction::kContinue, {}, false};
}

}  // namespace mujoco_sim::adapter_sdk::internal
