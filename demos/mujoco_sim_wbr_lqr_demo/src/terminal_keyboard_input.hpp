#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include <termios.h>

#include <control/msgs.hpp>

namespace mujoco_sim::wbr_lqr_demo {

/**
 * @brief Reports one non-blocking terminal-byte read outcome.
 *        表示一次非阻塞终端字节读取的结果。
 */
enum class KeyboardReadResult {
  kKey,
  kWouldBlock,
  kDisconnected,
  kError,
};

/**
 * @brief Abstracts the pollable byte source used by the terminal reader.
 *
 * Production reads standard input. Tests may provide a bounded pipe-backed
 * source. Implementations own their descriptor unless documented otherwise and
 * must keep it valid until destruction.
 *
 * 抽象终端 reader 使用的可 poll 字节源。
 *
 * 生产环境读取标准输入；测试可以提供基于 pipe 的有界输入源。除非另有说明，实现拥有其
 * 描述符，并须保证描述符在析构前有效。
 */
class KeyboardByteSource {
 public:
  virtual ~KeyboardByteSource() = default;

  /**
   * @brief Returns the descriptor monitored by the reader thread.
   *        返回 reader 线程监视的描述符。
   */
  virtual int PollFileDescriptor() const noexcept = 0;

  /**
   * @brief Reads at most one terminal byte without blocking.
   *
   * `key` is modified only for `kKey`. `error_message` receives context for
   * `kError` and must not be null. The call runs only on the reader thread.
   *
   * 非阻塞地读取至多一个终端字节。
   *
   * 仅当返回 `kKey` 时修改 `key`。`kError` 时由非空的 `error_message` 接收上下文。
   * 本函数仅在 reader 线程调用。
   */
  virtual KeyboardReadResult Read(char* key, std::string* error_message) = 0;

  /**
   * @brief Returns a stable diagnostic name for the source.
   *        返回输入源用于诊断的稳定名称。
   */
  virtual std::string_view Name() const noexcept = 0;
};

/**
 * @brief Stores bounded timed keyboard input shared by two threads.
 *
 * Every W/A/S/D byte starts or refreshes that direction for a fixed number of
 * Controller updates. Each `Consume()` decrements active direction counters.
 * Leg targets use last-key-wins one-shot semantics; Space uses parity. The
 * mailbox has fixed capacity and invokes no external callback while locked.
 *
 * 保存两个线程共享的有界定时键盘输入。
 *
 * 每个 W/A/S/D 字节会启动或刷新该方向，使其在固定数量的 Controller update 内有效。
 * 每次 `Consume()` 都递减有效方向的计数。腿长目标采用最后按键生效的单次语义，Space
 * 使用奇偶语义。mailbox 容量固定，持锁期间不调用外部回调。
 */
class TerminalInputMailbox {
 public:
  /**
   * @brief Creates a mailbox with a nonzero direction lifetime.
   *        创建具有非零方向有效期的 mailbox。
   *
   * @param direction_hold_ticks Controller updates for which each W/A/S/D
   *        character remains active. A zero value is promoted to one update.
   *        每个 W/A/S/D 字符保持生效的 Controller update 数量；零值会提升为一次 update。
   */
  explicit TerminalInputMailbox(std::uint32_t direction_hold_ticks = 200);

  /**
   * @brief Applies one terminal character to the pending frame.
   *        将一个终端字符应用到待处理帧。
   *
   * Both upper- and lower-case W/A/S/D/Q/E/F/X are accepted. Space and Escape
   * retain their usual byte values. Unknown bytes have no effect.
   * 同时接受大小写 W/A/S/D/Q/E/F/X；Space 和 Escape 使用其通常字节值，未知字节无效果。
   */
  void HandleKey(char key);

  /**
   * @brief Consumes one Controller-frame snapshot and advances direction timers.
   *        消费一个 Controller 帧快照并推进方向计时器。
   *
   * @return An owned snapshot containing active timed directions and pending
   *         one-shot actions. One-shot actions are cleared before return.
   *         返回包含有效定时方向和待处理单次动作的自有快照；返回前会清除单次动作。
   */
  control::input_snapshot_t Consume();

  /**
   * @brief Clears all pending keyboard actions.
   *        清除全部待处理键盘动作。
   */
  void Clear();

  /**
   * @brief Reports whether Escape requested process shutdown.
   *        返回 Escape 是否已请求进程停止。
   */
  bool ExitRequested() const;

 private:
  /**
   * @brief Stable indices for the fixed-size direction lifetime array.
   *        固定大小方向有效期数组使用的稳定索引。
   */
  enum class Direction : std::size_t {
    kW,
    kS,
    kA,
    kD,
    kCount,
  };

  /**
   * @brief Clears all direction counters while the caller holds `mutex_`.
   *        在调用方持有 `mutex_` 时清除全部方向计数。
   */
  void ClearDirectionsLocked() noexcept;

  mutable std::mutex mutex_;
  const std::uint32_t direction_hold_ticks_;
  std::array<std::uint32_t, static_cast<std::size_t>(Direction::kCount)> direction_ticks_{};
  control::input_snapshot_t pending_one_shots_{};
  bool exit_requested_ = false;
};

/**
 * @brief Owns the raw-terminal reader thread and bounded input mailbox.
 *
 * Production validates standard input before remote Controller registration,
 * switches it to non-canonical/no-echo mode when `Start()` succeeds, and
 * restores the original settings after `Join()`. Stop wakes `poll()` through an
 * eventfd. The reader never invokes the Adapter SDK or Controller.
 *
 * 拥有 raw terminal reader 线程及有界输入 mailbox。
 *
 * 生产环境在远端 Controller 注册前校验标准输入，在 `Start()` 成功时切换到非规范、无回显
 * 模式，并在 `Join()` 后恢复原设置。停止时通过 eventfd 唤醒 `poll()`。reader 不调用
 * Adapter SDK 或 Controller。
 */
class TerminalKeyboardInput {
 public:
  /**
   * @brief Validates standard input and captures its terminal settings.
   *
   * This call requires an interactive TTY but does not change terminal mode or
   * start a thread.
   *
   * 校验标准输入并保存其终端设置。
   *
   * 本调用要求交互式 TTY，但不会修改终端模式或启动线程。
   *
   * @param direction_hold_ticks Controller updates for which each direction remains active.
   *                             每个方向保持生效的 Controller update 数量。
   * @param error_message Receives a contextual error and must not be null.
   *                      接收上下文错误，且不得为空。
   * @return Owning terminal input on success; otherwise null.
   *         成功时返回拥有所有权的终端输入，否则返回空。
   */
  static std::unique_ptr<TerminalKeyboardInput> OpenStandardInput(
      std::uint32_t direction_hold_ticks, std::string* error_message);

  /**
   * @brief Creates an input instance from an owned test byte source.
   *
   * The returned object does not modify process terminal settings.
   *
   * 从拥有所有权的测试字节源创建输入实例。
   *
   * 返回对象不会修改进程终端设置。
   *
   * @param source Owned pollable test source.
   *               拥有所有权的可 poll 测试输入源。
   * @param direction_hold_ticks Controller updates for which each direction remains active.
   *                             每个方向保持生效的 Controller update 数量。
   * @param error_message Receives a contextual error and must not be null.
   *                      接收上下文错误，且不得为空。
   * @return Owning terminal input on success; otherwise null.
   *         成功时返回拥有所有权的终端输入，否则返回空。
   */
  static std::unique_ptr<TerminalKeyboardInput> CreateForTesting(
      std::unique_ptr<KeyboardByteSource> source, std::uint32_t direction_hold_ticks,
      std::string* error_message);

  /**
   * @brief Applies one synthetic key through the bounded mailbox test seam.
   *        通过有界 mailbox 测试接缝应用一个合成按键。
   *
   * @param key Character value copied synchronously into the mailbox.
   *            同步复制到 mailbox 的字符值。
   */
  void InjectKeyForTesting(char key);

  ~TerminalKeyboardInput() noexcept;

  TerminalKeyboardInput(const TerminalKeyboardInput&) = delete;
  TerminalKeyboardInput& operator=(const TerminalKeyboardInput&) = delete;

  /**
   * @brief Enables immediate terminal input and starts the reader thread.
   *        启用即时终端输入并启动 reader 线程。
   *
   * @param error_message Receives a contextual failure and must not be null.
   *                      接收上下文失败，且不得为空。
   * @return true on success; otherwise false.
   *         成功返回 true，否则返回 false。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Requests reader shutdown and wakes its poll operation.
   *        请求 reader 停止并唤醒其 poll 操作。
   */
  void RequestStop() noexcept;

  /**
   * @brief Joins the reader and restores terminal settings without throwing.
   *        等待 reader 结束并恢复终端设置，且不抛出异常。
   */
  void Join() noexcept;

  /**
   * @brief Consumes one input snapshot from the bounded mailbox.
   *        从有界 mailbox 消费一个输入快照。
   */
  control::input_snapshot_t Consume();

  /**
   * @brief Clears all pending keyboard input.
   *        清除全部待处理键盘输入。
   */
  void Clear();

  /**
   * @brief Reports whether Escape requested shutdown.
   *        返回 Escape 是否已请求停止。
   */
  bool ExitRequested() const;

  /**
   * @brief Transactionally takes one terminal reader or restoration error.
   *
   * The output is unchanged when no error is pending. Errors are consumed at
   * most once.
   *
   * 事务性取得一个终端 reader 或恢复错误。
   *
   * 没有待处理错误时输出保持不变；错误最多消费一次。
   */
  bool TakeError(std::string* error_message);

  /**
   * @brief Returns the stable diagnostic source name.
   *        返回稳定的诊断输入源名称。
   */
  std::string_view source_name() const noexcept;

 private:
  TerminalKeyboardInput(std::unique_ptr<KeyboardByteSource> source, int stop_fd,
                        std::uint32_t direction_hold_ticks, const termios* original_terminal);

  bool EnableImmediateMode(std::string* error_message);
  void RestoreTerminal() noexcept;
  void Run() noexcept;
  bool DrainKeys();
  void RecordError(std::string_view message) noexcept;

  std::unique_ptr<KeyboardByteSource> source_;
  int stop_fd_ = -1;
  TerminalInputMailbox mailbox_;
  std::atomic<bool> stop_requested_{false};
  std::jthread reader_;
  bool manages_terminal_ = false;
  bool terminal_active_ = false;
  termios original_terminal_{};
  mutable std::mutex error_mutex_;
  std::string terminal_error_;
  std::atomic<bool> reader_failed_{false};
};

}  // namespace mujoco_sim::wbr_lqr_demo
