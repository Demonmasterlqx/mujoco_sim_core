#pragma once

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace mujoco_sim::adapter_sdk::internal {

/**
 * @brief Gates callback entry and drains callbacks without waiting for the current callback.
 *
 * A shared State outlives transport callback registrations. Lease admission and
 * Disable() are serialized by State::mutex. WaitForOtherCallbacks() excludes
 * leases owned by its calling thread so teardown invoked from a callback cannot
 * deadlock on itself. State and the Target object must outlive every admitted lease.
 *
 * 门控回调进入并等待回调退出，同时不会等待当前线程自己的回调。
 *
 * 共享 State 的生命周期长于传输回调注册。租约准入与 Disable() 由 State::mutex
 * 串行化。WaitForOtherCallbacks() 会排除调用线程自身持有的租约，避免从回调内触发
 * 拆除时等待自己而死锁。State 与 Target 对象必须长于所有已准入租约。
 *
 * @tparam Target Callback owner type exposed by an admitted lease.
 *                已准入租约所暴露的回调所有者类型。
 */
template <typename Target>
class CallbackGate {
 public:
  /**
   * @brief Shared callback admission and accounting state.
   *        共享的回调准入与计数状态。
   *
   * All fields are protected by mutex. callbacks_by_thread records nested leases
   * so callback-initiated teardown can distinguish its own leases.
   * 所有字段均由 mutex 保护；callbacks_by_thread 记录嵌套租约，使回调内拆除能够识别
   * 当前线程自身的租约。
   */
  struct State {
    std::mutex mutex;
    std::condition_variable condition;
    Target* client = nullptr;
    bool enabled = false;
    std::size_t active_callbacks = 0;
    std::unordered_map<std::thread::id, std::size_t> callbacks_by_thread;
  };

  /**
   * @brief Owns one admitted callback invocation.
   *        持有一次已准入的回调调用。
   *
   * Construction is non-blocking except for the short State mutex critical
   * section. Destruction releases accounting and wakes teardown waiters.
   * 构造过程除短暂持有 State 互斥锁外不会阻塞；析构会释放计数并唤醒拆除等待方。
   */
  class Lease {
   public:
    /**
     * @brief Attempts callback admission against shared state.
     *
     * A disabled state produces an empty lease. Thread accounting is committed
     * before the total count so an allocation failure cannot leave a phantom
     * active callback.
     *
     * 尝试通过共享状态准入回调。
     *
     * 状态已禁用时会生成空租约。线程计数会先于总计数提交，因此分配失败不会遗留
     * 不存在的活动回调计数。
     *
     * @param state Non-null shared state that outlives this lease.
     *              非空共享状态，其生命周期必须长于本租约。
     */
    explicit Lease(std::shared_ptr<State> state) : state_(std::move(state)) {
      std::lock_guard lock(state_->mutex);
      if (!state_->enabled || state_->client == nullptr) return;
      auto [thread_entry, inserted] =
          state_->callbacks_by_thread.try_emplace(std::this_thread::get_id(), 0);
      (void)inserted;
      ++thread_entry->second;
      ++state_->active_callbacks;
      client_ = state_->client;
    }

    /** @brief Releases callback admission and wakes drain waiters.
     *         释放回调准入并唤醒清空等待方。 */
    ~Lease() {
      if (client_ == nullptr) return;
      {
        std::lock_guard lock(state_->mutex);
        --state_->active_callbacks;
        const auto found = state_->callbacks_by_thread.find(std::this_thread::get_id());
        if (found != state_->callbacks_by_thread.end() && --found->second == 0) {
          state_->callbacks_by_thread.erase(found);
        }
      }
      state_->condition.notify_all();
    }

    /** @brief Callback admission has unique ownership and cannot be copied.
     *         回调准入权为独占所有，不能复制。 */
    Lease(const Lease&) = delete;
    /** @brief Callback admission has unique ownership and cannot be copy-assigned.
     *         回调准入权为独占所有，不能复制赋值。 */
    Lease& operator=(const Lease&) = delete;

    /**
     * @brief Returns the admitted callback target, or nullptr when disabled.
     *        返回已准入的回调目标；状态已禁用时返回 nullptr。
     * @return Borrowed target valid only for this lease's lifetime.
     *         仅在本租约生命周期内有效的借用目标。
     */
    Target* client() const noexcept { return client_; }

   private:
    std::shared_ptr<State> state_;
    Target* client_ = nullptr;
  };

  /**
   * @brief Atomically rejects future callbacks and clears the borrowed target.
   *        原子拒绝后续回调并清除借用目标。
   * @param state Non-null shared admission state. 非空的共享准入状态。
   * @par Thread safety / 线程安全
   * May run concurrently with lease construction and destruction.
   * 可与租约构造和析构并发执行。
   */
  static void Disable(const std::shared_ptr<State>& state) {
    {
      std::lock_guard lock(state->mutex);
      state->enabled = false;
      state->client = nullptr;
    }
    state->condition.notify_all();
  }

  /**
   * @brief Blocks until callbacks on every other thread have released their leases.
   *
   * Call Disable() before this function when teardown must prevent new admission.
   * Leases held by the calling thread are excluded to permit callback-initiated
   * teardown. The function does not invoke external code while holding State::mutex.
   *
   * 阻塞等待其他所有线程上的回调释放租约。
   *
   * 拆除需要阻止新准入时，应先调用 Disable()。调用线程自身持有的租约会被排除，
   * 从而允许在回调内发起拆除。本函数持有 State::mutex 时不会调用外部代码。
   *
   * @param state Non-null shared admission state. 非空的共享准入状态。
   * @par Thread safety / 线程安全
   * May run concurrently with lease destruction.
   * 可与租约析构并发执行。
   *
   * @par Blocking / 阻塞行为
   * Blocks until all other-thread callback leases are released.
   * 阻塞直到其他线程的所有回调租约均被释放。
   */
  static void WaitForOtherCallbacks(const std::shared_ptr<State>& state) {
    std::unique_lock lock(state->mutex);
    const auto current = state->callbacks_by_thread.find(std::this_thread::get_id());
    const std::size_t callbacks_on_this_thread =
        current == state->callbacks_by_thread.end() ? 0 : current->second;
    state->condition.wait(lock,
                          [&] { return state->active_callbacks <= callbacks_on_this_thread; });
  }
};

}  // namespace mujoco_sim::adapter_sdk::internal
