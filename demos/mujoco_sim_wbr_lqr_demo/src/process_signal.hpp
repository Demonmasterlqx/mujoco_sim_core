#pragma once

#include <csignal>
#include <memory>
#include <string>

namespace mujoco_sim::wbr_lqr_demo {

/**
 * @brief Installs and restores cooperative SIGINT/SIGTERM process handlers.
 *
 * Install this guard once on the application thread before starting worker
 * threads. The signal handler only stores a `std::sig_atomic_t` flag. Call
 * `Restore()` during ordered shutdown to obtain restoration diagnostics; the
 * destructor performs a noexcept best-effort fallback.
 *
 * 安装并恢复用于协作停止的 SIGINT/SIGTERM 进程处理器。
 *
 * 应在启动工作线程前由应用线程安装一次。本信号处理器只写入
 * `std::sig_atomic_t` 标志。顺序关闭时调用 `Restore()` 可取得恢复诊断；析构函数会执行
 * noexcept 的尽力兜底恢复。
 */
class ProcessSignalGuard {
 public:
  /**
   * @brief Installs both handlers transactionally.
   *        事务性安装两个信号处理器。
   *
   * @param error_message Receives a contextual failure and must not be null.
   *                      接收上下文错误，且不得为空。
   * @return Owning guard on success; otherwise null.
   *         成功时返回拥有所有权的 guard，否则返回空。
   */
  static std::unique_ptr<ProcessSignalGuard> Install(std::string* error_message);

  ~ProcessSignalGuard() noexcept;

  ProcessSignalGuard(const ProcessSignalGuard&) = delete;
  ProcessSignalGuard& operator=(const ProcessSignalGuard&) = delete;

  /**
   * @brief Reports whether SIGINT or SIGTERM requested shutdown.
   *        返回 SIGINT 或 SIGTERM 是否已请求停止。
   *
   * @return true after a handled signal; otherwise false.
   *         收到已处理信号后返回 true，否则返回 false。
   */
  static bool StopRequested() noexcept;

  /**
   * @brief Restores the handlers that were active before installation.
   *        恢复安装前生效的信号处理器。
   *
   * Repeated calls succeed without changing process state. The method must run
   * on the application thread after worker shutdown.
   * 重复调用会直接成功且不改变进程状态。本函数须在工作线程停止后由应用线程调用。
   *
   * @param error_message Receives a contextual failure and must not be null.
   *                      接收上下文错误，且不得为空。
   * @return true when both handlers are restored; otherwise false.
   *         两个处理器均恢复成功时返回 true，否则返回 false。
   */
  bool Restore(std::string* error_message) noexcept;

 private:
  using Handler = void (*)(int);

  ProcessSignalGuard(Handler previous_interrupt, Handler previous_terminate)
      : previous_interrupt_(previous_interrupt), previous_terminate_(previous_terminate) {}

  Handler previous_interrupt_ = SIG_DFL;
  Handler previous_terminate_ = SIG_DFL;
  bool restored_ = false;
};

}  // namespace mujoco_sim::wbr_lqr_demo
