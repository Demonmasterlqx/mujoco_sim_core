#include "publish_diagnostics.hpp"

#include <gtest/gtest.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/ostream_sink.h>

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "server_logging.hpp"

namespace mujoco_sim::server {
namespace {

class PublishDiagnosticsTest : public ::testing::Test {
 protected:
  PublishDiagnosticsTest()
      : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(output_)),
        logger_(std::make_shared<spdlog::logger>("publish_diagnostics_test", sink_)),
        limiter_(std::chrono::hours(1)) {
    logger_->set_pattern("%v");
  }

  std::ostringstream output_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
  std::shared_ptr<spdlog::logger> logger_;
  WarningLimiter limiter_;
};

TEST_F(PublishDiagnosticsTest, IgnoresSuccessAndOrdinarySendRejection) {
  EcalServerTransport::PublishResult result;
  result.attempted = true;
  result.topic = "status/topic";
  result.bytes = 42;

  ReportPublishResult(logger_, &limiter_, {"ServerStatus", {}, 17, 0}, result);
  EXPECT_TRUE(output_.str().empty());

  result.sent = true;
  ReportPublishResult(logger_, &limiter_, {"ServerStatus", {}, 17, 0}, result);
  EXPECT_TRUE(output_.str().empty());
}

TEST_F(PublishDiagnosticsTest, ReportsAndLimitsTransportExceptions) {
  EcalServerTransport::PublishResult result;
  result.attempted = true;
  result.topic = "state/topic";
  result.bytes = 128;
  result.transport_error = "injected send exception";
  const PublishDiagnosticContext context{"ControllerState", "drive", 23, 9};

  ReportPublishResult(logger_, &limiter_, context, result);
  ReportPublishResult(logger_, &limiter_, {"ControllerState", "arm", 24, 10}, result);

  const std::string log = output_.str();
  EXPECT_NE(log.find("event=message_publish_failed"), std::string::npos);
  EXPECT_NE(log.find("message_type=\"ControllerState\""), std::string::npos);
  EXPECT_NE(log.find("operation=\"transport\""), std::string::npos);
  EXPECT_NE(log.find("adapter_id=\"drive\""), std::string::npos);
  EXPECT_NE(log.find("topic=\"state/topic\""), std::string::npos);
  EXPECT_NE(log.find("tick_id=23 sequence=9 bytes=128"), std::string::npos);
  EXPECT_EQ(log.find("event=message_publish_failed", log.find("event=message_publish_failed") + 1),
            std::string::npos);
}

TEST_F(PublishDiagnosticsTest, ReportsSerializationAndEncodingFailures) {
  EcalServerTransport::PublishResult result;
  result.topic = "status/topic";
  result.serialization_failed = true;
  ReportPublishResult(logger_, &limiter_, {"ServerStatus", {}, 31, 0}, result);

  mujoco_sim::protocol::DataFrameError error;
  error.code = mujoco_sim::protocol::DataFrameErrorCode::kFrameTooLarge;
  error.message = "frame exceeds limit";
  ReportDataFrameEncodingFailure(logger_, &limiter_, {"Observation", "camera", 32, 4}, error);

  const std::string log = output_.str();
  EXPECT_NE(log.find("operation=\"serialize\""), std::string::npos);
  EXPECT_NE(log.find("Protobuf serialization returned failure"), std::string::npos);
  EXPECT_NE(log.find("operation=\"encode\""), std::string::npos);
  EXPECT_NE(log.find("frame exceeds limit"), std::string::npos);
}

}  // namespace
}  // namespace mujoco_sim::server
