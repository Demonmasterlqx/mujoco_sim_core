#include <mujoco_sim/server/server_config.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/v3/sync.pb.h>
#if MUJOCO_SIM_HAS_SENSOR_STREAMS
#include <mujoco_sim/sensor/source_profile.hpp>
#endif

#include "server_config_generated.hpp"

namespace mujoco_sim::server {
namespace {

generated::ServerConfig::LogLevel ToGeneratedLogLevel(
    const std::string& level) {
  if (level == "trace") {
    return generated::ServerConfig::LogLevel::kTrace;
  }
  if (level == "debug") {
    return generated::ServerConfig::LogLevel::kDebug;
  }
  if (level == "warn") {
    return generated::ServerConfig::LogLevel::kWarn;
  }
  if (level == "error") {
    return generated::ServerConfig::LogLevel::kError;
  }
  if (level == "critical") {
    return generated::ServerConfig::LogLevel::kCritical;
  }
  if (level == "off") {
    return generated::ServerConfig::LogLevel::kOff;
  }
  return generated::ServerConfig::LogLevel::kInfo;
}

std::string ToPublicLogLevel(generated::ServerConfig::LogLevel level) {
  switch (level) {
    case generated::ServerConfig::LogLevel::kTrace:
      return "trace";
    case generated::ServerConfig::LogLevel::kDebug:
      return "debug";
    case generated::ServerConfig::LogLevel::kInfo:
      return "info";
    case generated::ServerConfig::LogLevel::kWarn:
      return "warn";
    case generated::ServerConfig::LogLevel::kError:
      return "error";
    case generated::ServerConfig::LogLevel::kCritical:
      return "critical";
    case generated::ServerConfig::LogLevel::kOff:
      return "off";
  }
  return "info";
}

generated::ServerConfig::RenderBackend ToGeneratedRenderBackend(
    const std::string& value) {
  if (value == "egl") return generated::ServerConfig::RenderBackend::kEgl;
  if (value == "osmesa") {
    return generated::ServerConfig::RenderBackend::kOsmesa;
  }
  return generated::ServerConfig::RenderBackend::kAuto;
}

std::string ToPublicRenderBackend(
    generated::ServerConfig::RenderBackend value) {
  switch (value) {
    case generated::ServerConfig::RenderBackend::kEgl:
      return "egl";
    case generated::ServerConfig::RenderBackend::kOsmesa:
      return "osmesa";
    case generated::ServerConfig::RenderBackend::kAuto:
      return "auto";
  }
  return "auto";
}

generated::ServerConfig::SensorStreamsItem::Kind ToGeneratedStreamKind(
    const std::string& value) {
  using Kind = generated::ServerConfig::SensorStreamsItem::Kind;
  if (value == "depth") return Kind::kDepth;
  if (value == "segmentation") return Kind::kSegmentation;
  if (value == "camera_pointcloud") return Kind::kCameraPointcloud;
  if (value == "lidar_pointcloud") return Kind::kLidarPointcloud;
  return Kind::kRgb;
}

std::string ToPublicStreamKind(
    generated::ServerConfig::SensorStreamsItem::Kind value) {
  using Kind = generated::ServerConfig::SensorStreamsItem::Kind;
  switch (value) {
    case Kind::kRgb:
      return "rgb";
    case Kind::kDepth:
      return "depth";
    case Kind::kSegmentation:
      return "segmentation";
    case Kind::kCameraPointcloud:
      return "camera_pointcloud";
    case Kind::kLidarPointcloud:
      return "lidar_pointcloud";
  }
  return "rgb";
}

generated::ServerConfig::SensorStreamsItem::QueueMode ToGeneratedQueueMode(
    const std::string& value) {
  using Mode = generated::ServerConfig::SensorStreamsItem::QueueMode;
  if (value == "fifo") return Mode::kFifo;
  if (value == "blocking") return Mode::kBlocking;
  return Mode::kLatest;
}

std::string ToPublicQueueMode(
    generated::ServerConfig::SensorStreamsItem::QueueMode value) {
  using Mode = generated::ServerConfig::SensorStreamsItem::QueueMode;
  switch (value) {
    case Mode::kLatest:
      return "latest";
    case Mode::kFifo:
      return "fifo";
    case Mode::kBlocking:
      return "blocking";
  }
  return "latest";
}

generated::ServerConfig::SensorStreamsItem::FifoOverflow
ToGeneratedFifoOverflow(const std::string& value) {
  using Overflow =
      generated::ServerConfig::SensorStreamsItem::FifoOverflow;
  return value == "drop_newest" ? Overflow::kDropNewest
                                 : Overflow::kDropOldest;
}

std::string ToPublicFifoOverflow(
    generated::ServerConfig::SensorStreamsItem::FifoOverflow value) {
  using Overflow =
      generated::ServerConfig::SensorStreamsItem::FifoOverflow;
  return value == Overflow::kDropNewest ? "drop_newest" : "drop_oldest";
}

mujoco_sim::v3::StreamFormat StreamFormatForKind(const std::string& kind) {
  if (kind == "rgb") return mujoco_sim::v3::RGB8;
  if (kind == "depth") return mujoco_sim::v3::DEPTH32F;
  if (kind == "segmentation") return mujoco_sim::v3::SEGMENTATION32U;
  return mujoco_sim::v3::XYZ32F;
}

generated::ServerConfig ToGenerated(const ServerConfig& config) {
  generated::ServerConfig output;
  output.server_name = config.server_name;
  output.model_path = config.model_path;
  output.initial_keyframe = config.initial_keyframe;
  output.max_ticks = config.max_ticks;
  output.tick_timeout_ms = config.tick_timeout_ms;
  output.session_timeout_ms = config.session_timeout_ms;
  output.controller_heartbeat_period_ms =
      config.controller_heartbeat_period_ms;
  output.observer_heartbeat_period_ms = config.observer_heartbeat_period_ms;
  output.tick_retry_initial_ms = config.tick_retry_initial_ms;
  output.tick_retry_backoff_multiplier = config.tick_retry_backoff_multiplier;
  output.tick_retry_max_delay_ms = config.tick_retry_max_delay_ms;
  output.tick_retry_max_attempts = config.tick_retry_max_attempts;
  output.activation_timeout_ms = config.activation_timeout_ms;
  output.allow_timed_out_adapter_replacement =
      config.allow_timed_out_adapter_replacement;
  output.status_period_ms = config.status_period_ms;
  output.status_min_interval_ms = config.status_min_interval_ms;
  output.observer_max_rate_hz = config.observer_max_rate_hz;
  output.allow_every_step_observer = config.allow_every_step_observer;
  output.transport_subscriber_sample_period_ms =
      config.transport_subscriber_sample_period_ms;
  output.sensor_subscriber_sample_period_ms =
      config.sensor_subscriber_sample_period_ms;
  output.headless = config.headless;
  output.log_level = ToGeneratedLogLevel(config.log_level);
  output.log_file = config.log_file;
  output.log_file_max_size_mb = config.log_file_max_size_mb;
  output.log_file_max_files = config.log_file_max_files;
  output.log_summary_period_ms = config.log_summary_period_ms;
  output.render_backend = ToGeneratedRenderBackend(config.render_backend);
  output.sensor_streams.clear();
  output.sensor_streams.reserve(config.sensor_streams.size());
  for (const auto& stream : config.sensor_streams) {
    generated::ServerConfig::SensorStreamsItem item;
    item.name = stream.name;
    item.kind = ToGeneratedStreamKind(stream.kind);
    item.source = stream.source;
    item.frame_id = stream.frame_id;
    item.rate_hz = stream.rate_hz;
    item.width = stream.width;
    item.height = stream.height;
    item.horizontal_samples = stream.horizontal_samples;
    item.vertical_samples = stream.vertical_samples;
    item.vertical_fov_min_deg = stream.vertical_fov_min_deg;
    item.vertical_fov_max_deg = stream.vertical_fov_max_deg;
    item.min_range_m = stream.min_range_m;
    item.max_range_m = stream.max_range_m;
    item.queue_mode = ToGeneratedQueueMode(stream.queue_mode);
    item.queue_depth = stream.queue_depth;
    item.fifo_overflow = ToGeneratedFifoOverflow(stream.fifo_overflow);
    output.sensor_streams.push_back(std::move(item));
  }
  return output;
}

ServerConfig ToPublic(generated::ServerConfig config) {
  ServerConfig output;
  output.server_name = std::move(config.server_name);
  output.model_path = std::move(config.model_path).string();
  output.initial_keyframe = std::move(config.initial_keyframe);
  output.max_ticks = config.max_ticks;
  output.tick_timeout_ms = config.tick_timeout_ms;
  output.session_timeout_ms = config.session_timeout_ms;
  output.controller_heartbeat_period_ms =
      config.controller_heartbeat_period_ms;
  output.observer_heartbeat_period_ms = config.observer_heartbeat_period_ms;
  output.tick_retry_initial_ms = config.tick_retry_initial_ms;
  output.tick_retry_backoff_multiplier = config.tick_retry_backoff_multiplier;
  output.tick_retry_max_delay_ms = config.tick_retry_max_delay_ms;
  output.tick_retry_max_attempts = config.tick_retry_max_attempts;
  output.activation_timeout_ms = config.activation_timeout_ms;
  output.allow_timed_out_adapter_replacement =
      config.allow_timed_out_adapter_replacement;
  output.status_period_ms = config.status_period_ms;
  output.status_min_interval_ms = config.status_min_interval_ms;
  output.observer_max_rate_hz = config.observer_max_rate_hz;
  output.allow_every_step_observer = config.allow_every_step_observer;
  output.transport_subscriber_sample_period_ms =
      config.transport_subscriber_sample_period_ms;
  output.sensor_subscriber_sample_period_ms =
      config.sensor_subscriber_sample_period_ms;
  output.headless = config.headless;
  output.log_level = ToPublicLogLevel(config.log_level);
  output.log_file = std::move(config.log_file).string();
  output.log_file_max_size_mb = config.log_file_max_size_mb;
  output.log_file_max_files = config.log_file_max_files;
  output.log_summary_period_ms = config.log_summary_period_ms;
  output.render_backend = ToPublicRenderBackend(config.render_backend);
  output.sensor_streams.reserve(config.sensor_streams.size());
  for (auto& stream : config.sensor_streams) {
    ServerConfig::SensorStream item;
    item.name = std::move(stream.name);
    item.kind = ToPublicStreamKind(stream.kind);
    item.source = std::move(stream.source);
    item.frame_id = std::move(stream.frame_id);
    item.rate_hz = stream.rate_hz;
    item.width = stream.width;
    item.height = stream.height;
    item.horizontal_samples = stream.horizontal_samples;
    item.vertical_samples = stream.vertical_samples;
    item.vertical_fov_min_deg = stream.vertical_fov_min_deg;
    item.vertical_fov_max_deg = stream.vertical_fov_max_deg;
    item.min_range_m = stream.min_range_m;
    item.max_range_m = stream.max_range_m;
    item.queue_mode = ToPublicQueueMode(stream.queue_mode);
    item.queue_depth = stream.queue_depth;
    item.fifo_overflow = ToPublicFifoOverflow(stream.fifo_overflow);
    output.sensor_streams.push_back(std::move(item));
  }
  return output;
}

std::vector<const char*> ConstArgv(int argc, char** argv) {
  std::vector<const char*> result;
  result.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    result.push_back(argv[i]);
  }
  return result;
}

bool ValidateSensorStreams(const ServerConfig& config,
                           std::string* error_message) {
#if !MUJOCO_SIM_HAS_SENSOR_STREAMS
  if (!config.sensor_streams.empty()) {
    *error_message = "this build does not include sensor streams";
    return false;
  }
  return true;
#else
  if (config.render_backend != "auto" && config.render_backend != "egl" &&
      config.render_backend != "osmesa") {
    *error_message = "render_backend must be one of: auto, egl, osmesa";
    return false;
  }
  if (!config.headless && config.render_backend == "osmesa") {
    *error_message =
        "render_backend osmesa is incompatible with the graphical viewer";
    return false;
  }
  std::vector<std::string> stream_names;
  const std::vector<std::string> kinds = {
      "rgb", "depth", "segmentation", "camera_pointcloud",
      "lidar_pointcloud"};
  const std::vector<std::string> queue_modes = {"latest", "fifo",
                                                 "blocking"};
  for (const auto& stream : config.sensor_streams) {
    if (!mujoco_sim::protocol::ValidateIdentifier(
            stream.name, mujoco_sim::protocol::IdentifierKind::kStream,
            error_message)) {
      return false;
    }
    if (stream.source.empty() || stream.frame_id.empty()) {
      *error_message =
          "sensor stream name, source, and frame_id must not be empty";
      return false;
    }
    if (std::find(stream_names.begin(), stream_names.end(), stream.name) !=
        stream_names.end()) {
      *error_message = "duplicate sensor stream name: " + stream.name;
      return false;
    }
    stream_names.push_back(stream.name);
    if (std::find(kinds.begin(), kinds.end(), stream.kind) == kinds.end()) {
      *error_message = "invalid sensor stream kind: " + stream.name;
      return false;
    }
    if (!std::isfinite(stream.rate_hz) ||
        stream.rate_hz < kMinimumSensorStreamRateHz ||
        stream.rate_hz > kMaximumSensorStreamRateHz) {
      *error_message = "sensor stream rate_hz is out of range: " +
                       stream.name;
      return false;
    }
    if (stream.width == 0 || stream.height == 0 ||
        stream.horizontal_samples == 0 || stream.vertical_samples == 0 ||
        stream.queue_depth == 0) {
      *error_message = "sensor stream dimensions must be positive: " +
                       stream.name;
      return false;
    }
    constexpr auto kRendererIntegerLimit =
        static_cast<std::uint32_t>(std::numeric_limits<int>::max());
    if (stream.width > kRendererIntegerLimit ||
        stream.height > kRendererIntegerLimit ||
        stream.horizontal_samples > kRendererIntegerLimit ||
        stream.vertical_samples > kRendererIntegerLimit) {
      *error_message =
          "sensor stream dimensions exceed renderer integer limits: " +
          stream.name;
      return false;
    }
    const bool is_lidar = stream.kind == "lidar_pointcloud";
    mujoco_sim::sensor::StreamShape shape;
    std::string shape_error;
    if (!mujoco_sim::sensor::StreamShapeFor(
            StreamFormatForKind(stream.kind),
            is_lidar ? stream.horizontal_samples : stream.width,
            is_lidar ? stream.vertical_samples : stream.height, &shape,
            &shape_error)) {
      *error_message = "sensor stream shape is invalid: " + stream.name +
                       ": " + shape_error;
      return false;
    }
    if (stream.queue_depth > kMaximumSensorStreamQueueDepth) {
      *error_message = "sensor stream queue_depth exceeds maximum of " +
                       std::to_string(kMaximumSensorStreamQueueDepth) + ": " +
                       stream.name;
      return false;
    }
    if (std::find(queue_modes.begin(), queue_modes.end(), stream.queue_mode) ==
        queue_modes.end()) {
      *error_message = "invalid sensor stream queue_mode: " + stream.name;
      return false;
    }
    if (stream.fifo_overflow != "drop_oldest" &&
        stream.fifo_overflow != "drop_newest") {
      *error_message = "invalid sensor stream fifo_overflow: " + stream.name;
      return false;
    }
    if (!std::isfinite(stream.vertical_fov_min_deg) ||
        !std::isfinite(stream.vertical_fov_max_deg) ||
        stream.vertical_fov_min_deg < -90.0 ||
        stream.vertical_fov_max_deg > 90.0 ||
        stream.vertical_fov_min_deg > stream.vertical_fov_max_deg) {
      *error_message =
          "sensor stream vertical_fov_min_deg/vertical_fov_max_deg must be "
          "finite, within [-90, 90], and ordered min <= max: " +
          stream.name;
      return false;
    }
    if (!std::isfinite(stream.min_range_m) ||
        !std::isfinite(stream.max_range_m) || stream.min_range_m < 0.0 ||
        stream.max_range_m < 0.000001 ||
        stream.min_range_m >= stream.max_range_m) {
      *error_message =
          "sensor stream range must be finite with 0 <= min_range_m < "
          "max_range_m: " +
          stream.name;
      return false;
    }
    if (is_lidar &&
        stream.max_range_m >
            static_cast<double>(std::numeric_limits<float>::max())) {
      *error_message =
          "sensor stream max_range_m exceeds lidar float range: " +
          stream.name;
      return false;
    }
    if (stream.queue_mode == "latest" && stream.queue_depth != 1) {
      *error_message =
          "latest sensor stream queue_depth must be 1: " + stream.name;
      return false;
    }
  }
  return true;
#endif
}

bool ValidateLogging(const ServerConfig& config, std::string* error_message) {
  static const std::vector<std::string> levels = {
      "trace", "debug", "info", "warn", "error", "critical", "off"};
  if (std::find(levels.begin(), levels.end(), config.log_level) ==
      levels.end()) {
    *error_message =
        "log_level must be one of: trace, debug, info, warn, error, "
        "critical, off";
    return false;
  }
  if (config.log_file_max_size_mb == 0) {
    *error_message = "log_file_max_size_mb must be >= 1";
    return false;
  }
  if (config.log_file_max_files == 0) {
    *error_message = "log_file_max_files must be in range [1, 200000]";
    return false;
  }
  if (config.log_file_max_files > 200000) {
    *error_message = "log_file_max_files must be in range [1, 200000]";
    return false;
  }
  if (config.activation_timeout_ms == 0) {
    *error_message = "activation_timeout_ms must be >= 1";
    return false;
  }
  if (config.session_timeout_ms == 0 ||
      config.controller_heartbeat_period_ms == 0 ||
      config.observer_heartbeat_period_ms == 0 ||
      config.controller_heartbeat_period_ms > config.session_timeout_ms / 3 ||
      config.observer_heartbeat_period_ms > config.session_timeout_ms / 3) {
    *error_message =
        "heartbeat periods must be positive and no greater than one third "
        "of session_timeout_ms";
    return false;
  }
  if (config.tick_retry_initial_ms == 0 ||
      config.tick_retry_max_delay_ms < config.tick_retry_initial_ms ||
      !std::isfinite(config.tick_retry_backoff_multiplier) ||
      config.tick_retry_backoff_multiplier < 1.0 ||
      config.tick_retry_max_delay_ms > config.tick_timeout_ms) {
    *error_message = "Tick retry configuration is invalid";
    return false;
  }
  if (config.status_period_ms == 0 || config.status_min_interval_ms == 0 ||
      config.status_min_interval_ms > config.status_period_ms) {
    *error_message =
        "status_min_interval_ms must be in [1, status_period_ms]";
    return false;
  }
  if (!std::isfinite(config.observer_max_rate_hz) ||
      config.observer_max_rate_hz <= 0.0 ||
      config.transport_subscriber_sample_period_ms == 0 ||
      config.sensor_subscriber_sample_period_ms == 0) {
    *error_message = "message-rate configuration is invalid";
    return false;
  }
  return ValidateSensorStreams(config, error_message);
}

bool ValidateFinal(const ServerConfig& config, std::string* error_message) {
  if (!ValidateLogging(config, error_message)) {
    return false;
  }
#if !MUJOCO_SIM_HAS_GUI
  if (!config.headless) {
    *error_message = "this build does not include GUI support; use --headless";
    return false;
  }
#endif
  if (config.model_path.empty()) {
    *error_message = "model path is required";
    return false;
  }
  if (config.server_name.empty()) {
    *error_message = "server name is required";
    return false;
  }
  if (!mujoco_sim::protocol::ValidateIdentifier(
          config.server_name,
          mujoco_sim::protocol::IdentifierKind::kServer, error_message)) {
    return false;
  }
  return true;
}

}  // namespace

bool ValidateServerConfig(const ServerConfig& config,
                          std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  return ValidateFinal(config, error_message);
}

bool LoadConfigFile(const std::string& path, ServerConfig* config,
                    std::string* error_message) {
  if (config == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "config and error_message must not be null";
    }
    return false;
  }
  if (!ValidateLogging(*config, error_message)) {
    return false;
  }
  auto parsed = ToGenerated(*config);
  if (!generated::Load(std::filesystem::path(path), &parsed, error_message)) {
    return false;
  }
  ServerConfig output = ToPublic(std::move(parsed));
  if (!ValidateLogging(output, error_message)) {
    return false;
  }
  *config = std::move(output);
  return true;
}

bool ParseCommandLine(int argc, char** argv, ServerConfig* config,
                      std::string* error_message) {
  if (config == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "config and error_message must not be null";
    }
    return false;
  }
  if (!ValidateLogging(*config, error_message)) {
    return false;
  }
  auto parsed = ToGenerated(*config);
  const auto arguments = ConstArgv(argc, argv);
  if (!generated::Parse(argc, arguments.data(), &parsed, error_message)) {
    return false;
  }
  ServerConfig output = ToPublic(std::move(parsed));
  if (!ValidateServerConfig(output, error_message)) {
    return false;
  }
  *config = std::move(output);
  return true;
}

std::string Usage() { return generated::Usage(); }

}  // namespace mujoco_sim::server
