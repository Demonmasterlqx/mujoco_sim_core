#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco_sim/core/domain.hpp>

namespace mujoco_sim::server {

/**
 * @brief Stores the latest command and fault for each registered Controller.
 *
 * Callback threads publish bounded per-session values while the simulation
 * thread drains them. Registration reservations coordinate session-slot
 * changes with shutdown without keeping the Inbox mutex locked while Core or
 * transport code runs.
 *
 * 保存每个已注册控制器的最新指令和故障。
 *
 * 回调线程写入按会话限界的数据，仿真线程负责取走数据。注册预留用于协调会话槽位变更与
 * 关闭流程，同时确保调用 Core 或传输代码时不持有 Inbox 互斥锁。
 *
 * @par Thread safety / 线程安全
 * All public functions may be called concurrently. Destruction requires all
 * callers and reservations to have completed.
 *
 * 所有公开函数均可并发调用。析构前必须确保所有调用和注册预留均已结束。
 */
class ControllerCommandInbox {
 public:
  /**
   * @brief Owns one in-progress registration until it is committed or abandoned.
   *
   * The reservation does not keep the Inbox mutex locked. Its lifetime instead
   * prevents Close() from clearing session slots until the registration reaches
   * a commit or rollback boundary.
   *
   * 持有一个进行中的注册，直至提交或放弃。
   *
   * 预留对象不会持续持有 Inbox 互斥锁；其生命周期会阻止 Close() 在注册到达提交或
   * 回滚边界前清除会话槽位。
   */
  class RegistrationReservation {
   public:
    /** @brief Creates an empty reservation. 创建一个空预留对象。 */
    RegistrationReservation() = default;

    /**
     * @brief Transfers ownership of an in-progress registration.
     * @param other Reservation whose ownership is transferred.
     *              所有权将被转移的预留对象。
     */
    RegistrationReservation(RegistrationReservation&& other) noexcept;

    /**
     * @brief Replaces this reservation by taking ownership from another one.
     * @param other Reservation whose ownership is transferred.
     *              所有权将被转移的预留对象。
     * @return This reservation. 当前预留对象。
     */
    RegistrationReservation& operator=(RegistrationReservation&& other) noexcept;
    RegistrationReservation(const RegistrationReservation&) = delete;
    RegistrationReservation& operator=(const RegistrationReservation&) = delete;

    /**
     * @brief Abandons an uncommitted registration and releases its shutdown gate.
     *
     * 放弃尚未提交的注册，并释放其关闭门控。
     */
    ~RegistrationReservation();

    /**
     * @brief Reports whether this object owns an in-progress registration.
     * @return true when Commit() may be attempted; otherwise false.
     *         如果可以尝试 Commit() 则返回 true，否则返回 false。
     *
     * 返回本对象是否持有一个进行中的注册。
     */
    explicit operator bool() const noexcept { return owner_ != nullptr; }

    /**
     * @brief Atomically installs the registered session and removes replaced slots.
     *
     * The function releases the shutdown gate whether the commit succeeds or
     * fails. It does not call Core, transport code, or user callbacks.
     *
     * 原子安装已注册会话，并删除被替换的槽位。
     *
     * 无论提交成功还是失败，本函数都会释放关闭门控；本函数不会调用 Core、传输代码或
     * 用户回调。
     *
     * @param session_id New nonzero session identifier. 新的非零会话标识符。
     * @param epoch Model epoch associated with the session. 会话对应的模型代次。
     * @param removed_sessions Session slots replaced by the registration.
     *                         本次注册所替换的会话槽位。
     * @return true if the slot changes were committed; otherwise false.
     *         如果槽位变更已提交则返回 true，否则返回 false。
     */
    bool Commit(std::uint64_t session_id, std::uint64_t epoch,
                std::span<const std::uint64_t> removed_sessions = {});

   private:
    friend class ControllerCommandInbox;
    /** @brief Creates an owning reservation for the Inbox. 为 Inbox 创建持有型预留。 */
    RegistrationReservation(ControllerCommandInbox* owner, bool controller_slot);
    /** @brief Releases an uncommitted reservation without throwing. 无异常地释放未提交预留。 */
    void Reset() noexcept;

    ControllerCommandInbox* owner_ = nullptr;
    bool controller_slot_ = false;
  };

  /**
   * @brief Begins a registration transaction unless shutdown has started.
   *
   * The returned reservation keeps Close() waiting but does not keep the Inbox
   * mutex locked. At most one Controller-slot reservation may exist at a time.
   *
   * 在关闭开始前启动一个注册事务。
   *
   * 返回的预留对象会使 Close() 等待，但不会持续持有 Inbox 互斥锁。同一时刻最多存在
   * 一个控制器槽位预留。
   *
   * @param controller_slot Whether the registration requires a Controller slot.
   *                        本次注册是否需要控制器槽位。
   * @return An owning reservation, or an empty reservation when registration is
   *         unavailable. 返回持有预留；如果无法注册则返回空预留。
   */
  RegistrationReservation PrepareRegistration(bool controller_slot);

  /**
   * @brief Adds a session slot for test and internal setup paths.
   * @param session_id Nonzero session identifier. 非零会话标识符。
   * @param epoch Model epoch associated with the session. 会话对应的模型代次。
   * @return true if a new slot was inserted; otherwise false.
   *         如果插入新槽位则返回 true，否则返回 false。
   */
  bool Register(std::uint64_t session_id, std::uint64_t epoch = 1);

  /**
   * @brief Removes a session slot and its pending values.
   * @param session_id Session identifier to remove. 要删除的会话标识符。
   *
   * 删除会话槽位及其待处理数据。
   */
  void Remove(std::uint64_t session_id);

  /**
   * @brief Stores a command when it is newer than the pending command.
   * @param command Owned command value to store. 要保存的自有指令值。
   * @return true only when the simulation loop must be notified.
   *         仅当需要唤醒仿真循环时返回 true。
   *
   * 当指令比待处理指令更新时保存该指令。
   */
  bool Publish(mujoco_sim::core::ControllerCommand command);

  /**
   * @brief Stores the latest valid fault for a registered session.
   * @param fault Owned fault value to store. 要保存的自有故障值。
   * @return true only when the simulation loop must be notified.
   *         仅当需要唤醒仿真循环时返回 true。
   *
   * 保存已注册会话的最新有效故障。
   */
  bool PublishFault(mujoco_sim::core::ControllerFault fault);

  /**
   * @brief Moves all pending commands out of the Inbox.
   * @return At most one command per registered session. 每个已注册会话至多一条指令。
   *
   * 从 Inbox 中移出全部待处理指令。
   */
  std::vector<mujoco_sim::core::ControllerCommand> Drain();

  /**
   * @brief Moves all pending faults out of the Inbox.
   * @return At most one fault per registered session. 每个已注册会话至多一个故障。
   *
   * 从 Inbox 中移出全部待处理故障。
   */
  std::vector<mujoco_sim::core::ControllerFault> DrainFaults();

  /**
   * @brief Immediately rejects new input and registration without waiting.
   *
   * Existing registration reservations may still commit their prepared slot.
   * Close() later waits for those reservations and clears every slot. This
   * function is idempotent and does not invoke external code.
   *
   * 立即拒绝新的输入和注册，且不等待。
   *
   * 已存在的注册预留仍可提交其预备槽位。之后由 Close() 等待这些预留完成并清除全部
   * 槽位。本函数可重复调用，且不会调用外部代码。
   *
   * @par Blocking / 阻塞行为
   * This function takes only the short Inbox mutex and never waits for a
   * registration. 本函数仅短暂持有 Inbox 互斥锁，不会等待注册完成。
   */
  void RequestClose();

  /**
   * @brief Rejects new input, waits for registrations, and clears all slots.
   *
   * The wait releases the Inbox mutex. Calls are idempotent, and no slot may be
   * registered after this function begins.
   *
   * 拒绝新输入、等待在途注册完成并清除全部槽位。
   *
   * 等待期间会释放 Inbox 互斥锁。本函数可重复调用；本函数开始后不得再注册新槽位。
   */
  void Close();

 private:
  struct Slot {
    std::uint64_t epoch = 0;
    bool pending = false;
    mujoco_sim::core::ControllerCommand command;
    std::optional<mujoco_sim::core::ControllerFault> fault;
  };
  static constexpr std::uint64_t kReservedSessionId = 0;

  std::mutex mutex_;
  std::condition_variable registration_condition_;
  std::unordered_map<std::uint64_t, Slot> slots_;
  std::size_t active_registrations_ = 0;
  bool closed_ = false;
};

}  // namespace mujoco_sim::server
