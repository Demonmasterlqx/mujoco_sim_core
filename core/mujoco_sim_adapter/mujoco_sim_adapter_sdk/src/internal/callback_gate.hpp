#pragma once

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace mujoco_sim::adapter_sdk::internal {

template <typename Target>
class CallbackGate {
 public:
  struct State {
    std::mutex mutex;
    std::condition_variable condition;
    Target* client = nullptr;
    bool enabled = false;
    std::size_t active_callbacks = 0;
    std::unordered_map<std::thread::id, std::size_t> callbacks_by_thread;
  };

  class Lease {
   public:
    explicit Lease(std::shared_ptr<State> state) : state_(std::move(state)) {
      std::lock_guard lock(state_->mutex);
      if (!state_->enabled || state_->client == nullptr) return;
      ++state_->active_callbacks;
      ++state_->callbacks_by_thread[std::this_thread::get_id()];
      client_ = state_->client;
    }

    ~Lease() {
      if (client_ == nullptr) return;
      {
        std::lock_guard lock(state_->mutex);
        --state_->active_callbacks;
        const auto found =
            state_->callbacks_by_thread.find(std::this_thread::get_id());
        if (found != state_->callbacks_by_thread.end() &&
            --found->second == 0) {
          state_->callbacks_by_thread.erase(found);
        }
      }
      state_->condition.notify_all();
    }

    Lease(const Lease&) = delete;
    Lease& operator=(const Lease&) = delete;
    Target* client() const { return client_; }

   private:
    std::shared_ptr<State> state_;
    Target* client_ = nullptr;
  };

  static void Disable(const std::shared_ptr<State>& state) {
    {
      std::lock_guard lock(state->mutex);
      state->enabled = false;
      state->client = nullptr;
    }
    state->condition.notify_all();
  }

  static void WaitForOtherCallbacks(const std::shared_ptr<State>& state) {
    std::unique_lock lock(state->mutex);
    const auto current =
        state->callbacks_by_thread.find(std::this_thread::get_id());
    const std::size_t callbacks_on_this_thread =
        current == state->callbacks_by_thread.end() ? 0 : current->second;
    state->condition.wait(lock, [&] {
      return state->active_callbacks <= callbacks_on_this_thread;
    });
  }
};

}  // namespace mujoco_sim::adapter_sdk::internal
