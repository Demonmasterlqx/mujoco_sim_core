#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

#include <spdlog/logger.h>

#include <mujoco_sim/server/server_config.hpp>

namespace mujoco_sim::server {

class ServerLogger {
 public:
  ServerLogger() = default;
  ~ServerLogger();

  ServerLogger(const ServerLogger&) = delete;
  ServerLogger& operator=(const ServerLogger&) = delete;

  bool Initialize(const ServerConfig& config, std::string* error_message);
  void Flush();
  bool healthy() const;

  std::shared_ptr<spdlog::logger> get() const { return logger_; }

 private:
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<std::atomic<bool>> healthy_;
};

struct WarningDecision {
  bool emit = false;
  std::uint64_t suppressed = 0;
};

class WarningLimiter {
 public:
  explicit WarningLimiter(std::chrono::steady_clock::duration period);

  WarningDecision Admit(std::string_view key);
  WarningDecision AdmitAt(std::string_view key,
                          std::chrono::steady_clock::time_point now);

 private:
  struct Entry {
    std::chrono::steady_clock::time_point last{};
    std::uint64_t suppressed = 0;
  };

  std::chrono::steady_clock::duration period_;
  std::mutex mutex_;
  std::unordered_map<std::string, Entry> entries_;
};

std::string LogValue(std::string_view value, std::size_t max_length = 256);

}  // namespace mujoco_sim::server
