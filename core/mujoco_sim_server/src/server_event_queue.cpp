#include "server_event_queue.hpp"

#include <algorithm>
#include <utility>

namespace mujoco_sim::server {

bool ServerEventQueue::TryPush(ServerEvent event) {
  const std::size_t bytes = EventPayloadBytes(event);
  {
    std::lock_guard lock(mutex_);
    if (closed_ || events_.size() >= kMaxEvents ||
        bytes > kMaxPayloadBytes - std::min(payload_bytes_, kMaxPayloadBytes)) {
      return false;
    }
    payload_bytes_ += bytes;
    events_.push_back(std::move(event));
    high_watermark_ = std::max(high_watermark_, events_.size());
  }
  condition_.notify_one();
  return true;
}

std::vector<ServerEvent> ServerEventQueue::WaitUntil(
    std::chrono::steady_clock::time_point deadline) {
  std::unique_lock lock(mutex_);
  condition_.wait_until(lock, deadline,
                        [this] { return closed_ || notification_pending_ || !events_.empty(); });
  notification_pending_ = false;
  const std::size_t count = std::min(events_.size(), kMaxBatchSize);
  std::vector<ServerEvent> batch;
  batch.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    payload_bytes_ -= EventPayloadBytes(events_.front());
    batch.push_back(std::move(events_.front()));
    events_.pop_front();
  }
  return batch;
}

void ServerEventQueue::Close(const char* reason) {
  std::deque<ServerEvent> pending;
  {
    std::lock_guard lock(mutex_);
    if (closed_) return;
    closed_ = true;
    payload_bytes_ = 0;
    pending.swap(events_);
  }
  for (auto& event : pending)
    CancelRpcEvent(&event, reason);
  condition_.notify_all();
}

bool ServerEventQueue::closed() const {
  std::lock_guard lock(mutex_);
  return closed_;
}

void ServerEventQueue::Notify() {
  {
    std::lock_guard lock(mutex_);
    if (closed_) return;
    notification_pending_ = true;
  }
  condition_.notify_one();
}

std::size_t ServerEventQueue::size() const {
  std::lock_guard lock(mutex_);
  return events_.size();
}

std::size_t ServerEventQueue::payload_bytes() const {
  std::lock_guard lock(mutex_);
  return payload_bytes_;
}

std::size_t ServerEventQueue::high_watermark() const {
  std::lock_guard lock(mutex_);
  return high_watermark_;
}

}  // namespace mujoco_sim::server
