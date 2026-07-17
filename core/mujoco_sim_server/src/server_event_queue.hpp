#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "server_event.hpp"

namespace mujoco_sim::server {

class ServerEventQueue {
 public:
  static constexpr std::size_t kMaxEvents = 1024;
  static constexpr std::size_t kMaxPayloadBytes = 64U * 1024U * 1024U;
  static constexpr std::size_t kMaxBatchSize = 64;

  bool TryPush(ServerEvent event);
  std::vector<ServerEvent> WaitUntil(std::chrono::steady_clock::time_point deadline);
  void Close(const char* reason = "server is stopping");
  void Notify();

  bool closed() const;
  std::size_t size() const;
  std::size_t payload_bytes() const;
  std::size_t high_watermark() const;

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<ServerEvent> events_;
  std::size_t payload_bytes_ = 0;
  std::size_t high_watermark_ = 0;
  bool notification_pending_ = false;
  bool closed_ = false;
};

}  // namespace mujoco_sim::server
