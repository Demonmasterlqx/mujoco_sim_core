#include "publish_diagnostics.hpp"

#include <spdlog/logger.h>

#include <string>

#include "server_logging.hpp"

namespace mujoco_sim::server {
namespace {

void LogFailure(const std::shared_ptr<spdlog::logger>& logger, WarningLimiter* warning_limiter,
                const PublishDiagnosticContext& context, std::string_view operation,
                std::string_view topic, std::size_t bytes, std::string_view error) {
  if (!logger) return;
  const std::string key = std::string("message_publish_failed:") +
                          std::string(context.message_type) + ":" + std::string(operation);
  const WarningDecision decision =
      warning_limiter ? warning_limiter->Admit(key) : WarningDecision{true, 0};
  if (!decision.emit) return;
  logger->warn(
      "event=message_publish_failed message_type={} operation={} adapter_id={} topic={} "
      "tick_id={} sequence={} bytes={} error={} suppressed={}",
      LogValue(context.message_type), LogValue(operation), LogValue(context.adapter_id),
      LogValue(topic), context.tick_id, context.sequence, bytes, LogValue(error),
      decision.suppressed);
}

}  // namespace

void ReportPublishResult(const std::shared_ptr<spdlog::logger>& logger,
                         WarningLimiter* warning_limiter, const PublishDiagnosticContext& context,
                         const EcalServerTransport::PublishResult& result) {
  if (result.serialization_failed) {
    const std::string_view error = result.transport_error.empty()
                                       ? "Protobuf serialization returned failure"
                                       : std::string_view(result.transport_error);
    LogFailure(logger, warning_limiter, context, "serialize", result.topic, result.bytes, error);
    return;
  }
  if (!result.transport_error.empty()) {
    LogFailure(logger, warning_limiter, context, "transport", result.topic, result.bytes,
               result.transport_error);
  }
}

void ReportDataFrameEncodingFailure(const std::shared_ptr<spdlog::logger>& logger,
                                    WarningLimiter* warning_limiter,
                                    const PublishDiagnosticContext& context,
                                    const mujoco_sim::protocol::DataFrameError& error) {
  const std::string_view message = error.message.empty() ? "dense frame encoder returned failure"
                                                         : std::string_view(error.message);
  LogFailure(logger, warning_limiter, context, "encode", {}, 0, message);
}

}  // namespace mujoco_sim::server
