#include "server_logging.hpp"

#include <cctype>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <syncstream>
#include <utility>
#include <vector>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace mujoco_sim::server {
namespace {

constexpr std::string_view kConsolePattern =
    "%Y-%m-%dT%H:%M:%S.%e%z level=%^%l%$ thread=%t component=server %v";
constexpr std::string_view kFilePattern =
    "%Y-%m-%dT%H:%M:%S.%e%z level=%l thread=%t component=server %v";
constexpr std::size_t kBytesPerMiB = 1024U * 1024U;

spdlog::level::level_enum ParseLevel(const std::string& level) {
  if (level == "trace") {
    return spdlog::level::trace;
  }
  if (level == "debug") {
    return spdlog::level::debug;
  }
  if (level == "info") {
    return spdlog::level::info;
  }
  if (level == "warn") {
    return spdlog::level::warn;
  }
  if (level == "error") {
    return spdlog::level::err;
  }
  if (level == "critical") {
    return spdlog::level::critical;
  }
  return spdlog::level::off;
}

bool IsValidLevel(const std::string& level) {
  return level == "trace" || level == "debug" || level == "info" || level == "warn" ||
         level == "error" || level == "critical" || level == "off";
}

}  // namespace

ServerLogger::~ServerLogger() {
  try {
    Flush();
  } catch (const std::exception& error) {
    try {
      std::osyncstream(std::cerr) << "level=error component=server event=log_flush_error error="
                                  << LogValue(error.what()) << '\n';
    } catch (...) {
    }
  } catch (...) {
    try {
      std::osyncstream(std::cerr)
          << "level=error component=server event=log_flush_error error=\"unknown exception\"\n";
    } catch (...) {
    }
  }
}

bool ServerLogger::Initialize(const ServerConfig& config, std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  try {
    Flush();
  } catch (const std::exception& error) {
    *error_message = "failed to flush previous server logger: " + std::string(error.what());
    return false;
  } catch (...) {
    *error_message = "failed to flush previous server logger: unknown exception";
    return false;
  }
  logger_.reset();
  healthy_.reset();
  if (!IsValidLevel(config.log_level)) {
    *error_message =
        "log_level must be one of: trace, debug, info, warn, error, "
        "critical, off";
    return false;
  }
  if (config.log_file_max_size_mb == 0) {
    *error_message = "log_file_max_size_mb must be >= 1";
    return false;
  }
  if (config.log_file_max_files == 0 || config.log_file_max_files > 200000) {
    *error_message = "log_file_max_files must be in range [1, 200000]";
    return false;
  }

  try {
    std::vector<spdlog::sink_ptr> sinks;
    auto console = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    console->set_pattern(std::string(kConsolePattern));
    sinks.push_back(std::move(console));

    if (!config.log_file.empty()) {
      const std::filesystem::path path(config.log_file);
      if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
      }
      if (config.log_file_max_size_mb > std::numeric_limits<std::size_t>::max() / kBytesPerMiB) {
        *error_message = "log file size is too large";
        return false;
      }
      auto file = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          path.string(), static_cast<std::size_t>(config.log_file_max_size_mb) * kBytesPerMiB,
          static_cast<std::size_t>(config.log_file_max_files - 1));
      file->set_pattern(std::string(kFilePattern));
      sinks.push_back(std::move(file));
    }

    auto logger = std::make_shared<spdlog::logger>("mujoco-sim-server", sinks.begin(), sinks.end());
    auto healthy = std::make_shared<std::atomic<bool>>(true);
    logger->set_error_handler([healthy](const std::string& message) noexcept {
      healthy->store(false);
      try {
        std::osyncstream(std::cerr)
            << "level=error component=server event=log_sink_error error=" << LogValue(message)
            << '\n';
      } catch (...) {
      }
    });
    logger->set_level(ParseLevel(config.log_level));
    logger->flush_on(spdlog::level::info);
    logger_ = std::move(logger);
    healthy_ = std::move(healthy);
    return true;
  } catch (const std::exception& error) {
    *error_message = "failed to initialize server logging: " + std::string(error.what());
    return false;
  }
}

void ServerLogger::Flush() {
  if (logger_) {
    logger_->flush();
  }
}

bool ServerLogger::healthy() const {
  return healthy_ == nullptr || healthy_->load();
}

WarningLimiter::WarningLimiter(std::chrono::steady_clock::duration period) : period_(period) {}

WarningDecision WarningLimiter::Admit(std::string_view key) {
  return AdmitAt(key, std::chrono::steady_clock::now());
}

WarningDecision WarningLimiter::AdmitAt(std::string_view key,
                                        std::chrono::steady_clock::time_point now) {
  std::lock_guard lock(mutex_);
  auto& entry = entries_[std::string(key)];
  if (entry.last == std::chrono::steady_clock::time_point{} || now - entry.last >= period_) {
    WarningDecision decision{true, entry.suppressed};
    entry.last = now;
    entry.suppressed = 0;
    return decision;
  }
  ++entry.suppressed;
  return WarningDecision{};
}

std::string LogValue(std::string_view value, std::size_t max_length) {
  std::ostringstream output;
  output << '"';
  const std::string_view displayed = value.substr(0, max_length);
  for (const unsigned char character : displayed) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (std::iscntrl(character) != 0) {
          output << "\\x" << std::hex << std::setw(2) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  if (displayed.size() < value.size()) {
    output << "...";
  }
  output << '"';
  return output.str();
}

}  // namespace mujoco_sim::server
