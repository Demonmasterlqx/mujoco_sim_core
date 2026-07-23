#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <utility>

namespace mujoco_sim::adapter_sdk::internal {

/** @brief Internal outcome of a bounded mailbox wait. 有界邮箱等待的内部结果。 */
enum class MailboxWaitResult {
  /// One value was transferred to output. 一个值已转移到输出。
  kReceived,
  /// Deadline elapsed without changing output. 截止期限已到且输出未改变。
  kTimeout,
  /// Owner stop predicate became true. 所有者停止谓词变为 true。
  kStopped,
  /// Owner supplied a terminal diagnostic. 所有者提供了终止诊断。
  kError,
  /// Another waiter already owns this mailbox. 另一个等待方已占用该邮箱。
  kConcurrentWait,
};

/** @brief Maps the internal mailbox outcome to a public result enum.
 *         将内部邮箱结果映射为公开结果枚举。
 * @tparam PublicResult Enum providing kReceived/kTimeout/kStopped/kError.
 *                      提供 kReceived/kTimeout/kStopped/kError 的枚举。
 * @param result Internal wait result. 内部等待结果。
 * @return Corresponding public result; concurrent wait maps to kError.
 *         对应公开结果；并发等待映射为 kError。
 */
template <typename PublicResult>
PublicResult ToPublicWaitResult(MailboxWaitResult result) {
  switch (result) {
    case MailboxWaitResult::kReceived:
      return PublicResult::kReceived;
    case MailboxWaitResult::kTimeout:
      return PublicResult::kTimeout;
    case MailboxWaitResult::kStopped:
      return PublicResult::kStopped;
    case MailboxWaitResult::kError:
    case MailboxWaitResult::kConcurrentWait:
      return PublicResult::kError;
  }
  return PublicResult::kError;
}

/**
 * @brief Stores a bounded FIFO whose wait shares the owner's lifecycle mutex.
 *        存储有界 FIFO，其等待与所有者生命周期共享互斥锁。
 *
 * The owner supplies the lock and condition variable so queue data, stop state,
 * and terminal errors are observed atomically. Every member requires the same
 * owning mutex, except that Wait() receives an already-owned unique_lock and
 * temporarily releases it during condition waits. At most one waiter is allowed.
 * 所有者提供锁与条件变量，使队列数据、停止状态和终止错误能被原子观察。所有成员都要求
 * 持有同一个所属互斥锁；Wait() 接收已持有的 unique_lock，并在条件等待期间临时释放它。
 * 最多允许一个等待方。
 *
 * @tparam T Owning value type transferred into and out of the queue.
 *           转入转出队列的自有值类型。
 */
template <typename T>
class BlockingMailbox {
 public:
  /** @brief Pushes one value and evicts the oldest value at capacity.
   *         压入一个值，并在达到容量时淘汰最旧值。
   * @param value Owning value moved into the queue. 移入队列的自有值。
   * @param capacity Positive maximum number of queued values. 正数的最大排队数量。
   * @return Number evicted by this push, either zero or one. 本次压入淘汰的数量，为零或一。
   * @pre The owner mutex is held and capacity is positive. 已持有所属互斥锁且 capacity 为正。
   */
  std::size_t Push(T value, std::size_t capacity) {
    std::size_t dropped = 0;
    if (items_.size() == capacity) {
      items_.pop_front();
      dropped = 1;
    }
    items_.push_back(std::move(value));
    return dropped;
  }

  /**
   * @brief Waits for data, terminal error, owner stop, or deadline.
   *        等待数据、终止错误、所有者停止或截止期限。
   * @tparam StopPredicate Callable returning owner stop state under the owner lock.
   *                       在所属锁下返回停止状态的可调用对象。
   * @tparam ErrorProvider Callable returning an owned/copyable terminal diagnostic.
   *                       返回自有/可复制终止诊断的可调用对象。
   * @param lock Already-owned lock for the owner mutex. 已持有的所属互斥锁。
   * @param condition Condition notified for pushes and lifecycle changes.
   * 数据压入与生命周期变化时通知的条件变量。
   * @param timeout Non-negative duration; max waits without an application deadline.
   *                非负等待时长；max 表示无应用层截止期限。
   * @param stopped Stop predicate evaluated while lock is held. 持锁求值的停止谓词。
   * @param error Terminal error provider evaluated while lock is held. 持锁求值的终止错误提供方。
   * @param output Required output modified only for kReceived. 必需输出，仅在 kReceived 时修改。
   * @param error_message Required output modified only for kError. 必需输出，仅在 kError 时修改。
   * @return Wait outcome, including kConcurrentWait for a second waiter.
   *         等待结果；第二个等待方会得到 kConcurrentWait。
   * @pre lock owns the mutex used by all other mailbox access. lock 持有所有邮箱访问共用的互斥锁。
   */
  template <typename StopPredicate, typename ErrorProvider>
  MailboxWaitResult Wait(std::unique_lock<std::mutex>& lock, std::condition_variable& condition,
                         std::chrono::milliseconds timeout, StopPredicate stopped,
                         ErrorProvider error, T* output, std::string* error_message) {
    if (wait_in_progress_) return MailboxWaitResult::kConcurrentWait;
    wait_in_progress_ = true;
    const auto finish = [this] {
      wait_in_progress_ = false;
    };
    const auto ready = [&] {
      return !items_.empty() || !error().empty() || stopped();
    };
    bool signaled = ready();
    if (!signaled && timeout == std::chrono::milliseconds::max()) {
      while (!(signaled = ready())) {
        condition.wait_for(lock, std::chrono::milliseconds(50));
      }
    } else if (!signaled) {
      signaled = condition.wait_for(lock, timeout, ready);
    }
    if (!signaled) {
      finish();
      return MailboxWaitResult::kTimeout;
    }
    const auto terminal_error = error();
    if (!terminal_error.empty()) {
      *error_message = terminal_error;
      finish();
      return MailboxWaitResult::kError;
    }
    if (stopped()) {
      finish();
      return MailboxWaitResult::kStopped;
    }
    *output = std::move(items_.front());
    items_.pop_front();
    finish();
    return MailboxWaitResult::kReceived;
  }

  /** @brief Removes all queued values. 移除所有排队值。
   * @pre The owner mutex is held. 已持有所属互斥锁。
   */
  void Clear() { items_.clear(); }
  /** @brief Returns queued value count. 返回排队值数量。
   * @pre The owner mutex is held. 已持有所属互斥锁。
   */
  std::size_t size() const { return items_.size(); }
  /** @brief Reports whether one waiter owns admission. 返回是否已有等待方获得准入。
   * @pre The owner mutex is held. 已持有所属互斥锁。
   */
  bool wait_in_progress() const { return wait_in_progress_; }

 private:
  /// FIFO storage protected by the owner-supplied mutex. 由所有者提供的互斥锁保护的 FIFO 存储。
  std::deque<T> items_;
  /// Single-waiter admission flag protected by the owner mutex.
  /// 由所属互斥锁保护的单等待方准入标志。
  bool wait_in_progress_ = false;
};

}  // namespace mujoco_sim::adapter_sdk::internal
