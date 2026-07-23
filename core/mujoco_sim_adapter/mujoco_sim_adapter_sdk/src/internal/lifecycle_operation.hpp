#pragma once

#include <condition_variable>
#include <mutex>
#include <utility>

namespace mujoco_sim::adapter_sdk::internal {

/**
 * @brief Serializes lifecycle operations without holding a mutex during them.
 *
 * Acquire() uses a short critical section to reserve the lifecycle operation.
 * The returned lease contains no locked mutex, so callers may perform RPC,
 * transport, callback teardown, or thread-join work while owning the lease.
 * Threads requesting a later lifecycle operation wait with the mutex released.
 * Cancellation paths that must interrupt an operation must set their atomic
 * stop state before calling Acquire().
 *
 * 在执行期间不持有互斥锁地串行化生命周期操作。
 *
 * Acquire() 仅通过短临界区预留生命周期操作。返回的租约不包含已锁定的互斥锁，因此
 * 调用方持有租约时可以执行 RPC、传输、回调清理或线程等待。后续生命周期操作的线程
 * 会在释放互斥锁的条件等待中阻塞。需要中断当前操作的取消路径必须先设置原子停止状态，
 * 再调用 Acquire()。
 *
 * @par Thread safety / 线程安全
 * Acquire() may be called concurrently. A lease must be destroyed before its
 * LifecycleOperation owner. 同一对象可并发调用 Acquire()；租约必须先于其所有者析构。
 */
class LifecycleOperation {
 public:
  /**
   * @brief Owns admission to one lifecycle operation.
   *
   * The lease is movable and releases admission when destroyed or overwritten.
   * 持有一个生命周期操作的准入权；租约可移动，并在析构或被覆盖时释放准入权。
   */
  class Lease {
   public:
    /** @brief Creates an empty lease. 创建一个空租约。 */
    Lease() = default;
    /** @brief Copying is forbidden because admission has one owner.
     * 准入权只有一个所有者，禁止复制。 */
    Lease(const Lease&) = delete;
    /** @brief Copy assignment is forbidden. 禁止复制赋值。 */
    Lease& operator=(const Lease&) = delete;

    /**
     * @brief Transfers lifecycle-operation ownership.
     * @param other Lease whose ownership is transferred.
     *              所有权将被转移的租约。
     */
    Lease(Lease&& other) noexcept : owner_(std::exchange(other.owner_, nullptr)) {}

    /**
     * @brief Replaces this lease by taking ownership from another lease.
     * @param other Lease whose ownership is transferred.
     *              所有权将被转移的租约。
     * @return This lease. 当前租约。
     */
    Lease& operator=(Lease&& other) noexcept {
      if (this != &other) {
        Reset();
        owner_ = std::exchange(other.owner_, nullptr);
      }
      return *this;
    }

    /** @brief Releases lifecycle-operation admission. 释放生命周期操作准入权。 */
    ~Lease() { Reset(); }

   private:
    /// Allows the gate to construct an admitted lease. 允许门控构造已准入租约。
    friend class LifecycleOperation;
    /** @brief Creates an owning lease for an already-reserved operation.
     *         为已预留操作创建持有租约。
     * @param owner Borrowed gate that outlives this lease. 生命周期长于本租约的借用门控。
     */
    explicit Lease(LifecycleOperation* owner) : owner_(owner) {}

    /** @brief Releases owned admission if present. 释放当前持有的准入权（如有）。 */
    void Reset() noexcept {
      if (owner_ == nullptr) return;
      owner_->Release();
      owner_ = nullptr;
    }

    /// Borrowed gate owning admission state; null in empty/moved-from leases.
    /// 持有准入状态的借用门控；空租约或被移走租约中为空。
    LifecycleOperation* owner_ = nullptr;
  };

  /** @brief Creates an idle lifecycle-operation gate. 创建空闲的生命周期操作门控。 */
  LifecycleOperation() = default;
  /** @brief Copying is forbidden because waiters refer to this gate. 等待方引用当前门控，禁止复制。
   */
  LifecycleOperation(const LifecycleOperation&) = delete;
  /** @brief Copy assignment is forbidden. 禁止复制赋值。 */
  LifecycleOperation& operator=(const LifecycleOperation&) = delete;

  /**
   * @brief Waits for and reserves the next lifecycle operation.
   *
   * The internal mutex is released before this function returns. Destroying the
   * returned lease admits one waiting caller.
   *
   * 等待并预留下一个生命周期操作。
   *
   * 本函数返回前会释放内部互斥锁；销毁返回的租约后将允许一个等待方继续执行。
   *
   * @return An owning operation lease. 持有准入权的操作租约。
   */
  [[nodiscard]] Lease Acquire() {
    std::unique_lock lock(mutex_);
    condition_.wait(lock, [this] { return !active_; });
    active_ = true;
    return Lease(this);
  }

 private:
  /** @brief Releases the active operation and wakes one waiter. 释放活动操作并唤醒一个等待方。 */
  void Release() noexcept {
    {
      std::lock_guard lock(mutex_);
      active_ = false;
    }
    condition_.notify_one();
  }

  /// Protects active_ only; never held while lifecycle work executes.
  /// 仅保护 active_；执行生命周期工作时绝不持有。
  std::mutex mutex_;
  /// Waits for the predicate `!active_` under mutex_. 在 mutex_ 下等待谓词 `!active_`。
  std::condition_variable condition_;
  /// Whether one lease owns lifecycle admission, protected by mutex_.
  /// 是否已有租约持有生命周期准入权；由 mutex_ 保护。
  bool active_ = false;
};

}  // namespace mujoco_sim::adapter_sdk::internal
