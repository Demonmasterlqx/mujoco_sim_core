#pragma once

#include <cstdint>
#include <memory>
#include <string_view>

#include <mujoco_sim/protocol/data_frame.hpp>

#include "ecal_server_transport.hpp"

namespace spdlog {
class logger;
}

namespace mujoco_sim::server {

class WarningLimiter;

/**
 * @brief Identifies one outbound message for contextual failure diagnostics.
 *
 * String views are borrowed only for the duration of a reporting call. The
 * simulation loop supplies the completed-step tick and per-session sequence
 * when the message has those concepts; status messages use sequence zero.
 *
 * 标识一条出站消息，用于生成带上下文的失败诊断。
 *
 * 字符串视图仅在一次报告调用期间借用。消息包含完成步 tick 和会话内序列时，由仿真
 * 循环提供对应值；状态消息的序列使用零。
 */
struct PublishDiagnosticContext {
  /// Borrowed wire-message type name. 借用的线协议消息类型名称。
  std::string_view message_type;
  /// Borrowed Adapter ID; empty for Server-wide messages. 借用的 Adapter 标识符；Server
  /// 级消息为空。
  std::string_view adapter_id;
  /// Completed-step tick, or zero when not applicable. 完成步 tick；不适用时为零。
  std::uint64_t tick_id = 0;
  /// Per-session sequence, or zero when not applicable. 会话内序列；不适用时为零。
  std::uint64_t sequence = 0;
};

/**
 * @brief Logs an explicit serialization or transport exception with rate limiting.
 *
 * Successful sends and ordinary `sent == false` results without an exception
 * are ignored because an absent subscriber is a normal non-blocking state.
 * Rate limiting is shared by message type and operation, so changing Adapter
 * IDs cannot grow limiter state without bound. The function borrows all
 * arguments and does not retain message data.
 *
 * 对明确的序列化失败或传输异常进行限频记录。
 *
 * 成功发送以及未伴随异常的普通 `sent == false` 会被忽略，因为没有订阅者是正常的
 * 非阻塞状态。限频额度按消息类型和操作共享，因此 Adapter 标识符变化不会导致限频
 * 状态无限增长。本函数仅借用所有参数，不保留消息数据。
 *
 * @param logger Logger receiving the structured diagnostic; may be null.
 *               接收结构化诊断的日志器；可以为空。
 * @param warning_limiter Shared limiter used only on explicit failure paths; may be null.
 *                        仅在明确失败路径使用的共享限频器；可以为空。
 * @param context Outbound message identity. 出站消息标识。
 * @param result Structured transport result to classify. 待分类的结构化传输结果。
 */
void ReportPublishResult(const std::shared_ptr<spdlog::logger>& logger,
                         WarningLimiter* warning_limiter, const PublishDiagnosticContext& context,
                         const EcalServerTransport::PublishResult& result);

/**
 * @brief Logs a dense-frame encoding failure with rate limiting.
 *
 * The function borrows all arguments, performs no retry, and does not change
 * simulation state.
 *
 * 对 dense frame 编码失败进行限频记录。
 *
 * 本函数仅借用所有参数，不执行重试，也不改变仿真状态。
 *
 * @param logger Logger receiving the structured diagnostic; may be null.
 *               接收结构化诊断的日志器；可以为空。
 * @param warning_limiter Shared limiter used on the failure path; may be null.
 *                        在失败路径使用的共享限频器；可以为空。
 * @param context Outbound message identity. 出站消息标识。
 * @param error Encoder diagnostic returned by the Protocol package.
 *              Protocol 包的编码器返回的诊断。
 */
void ReportDataFrameEncodingFailure(const std::shared_ptr<spdlog::logger>& logger,
                                    WarningLimiter* warning_limiter,
                                    const PublishDiagnosticContext& context,
                                    const mujoco_sim::protocol::DataFrameError& error);

}  // namespace mujoco_sim::server
