#pragma once

#include <string>

namespace mujoco_sim::sensor_demo {

/**
 * @brief Installs SIGINT and SIGTERM handlers for cooperative process shutdown.
 *        安装 SIGINT 和 SIGTERM 处理器，以便进程协作式停止。
 *
 * If SIGTERM installation fails after SIGINT succeeds, this function attempts
 * to restore the previous SIGINT handler before returning.
 * 如果 SIGINT 安装成功后 SIGTERM 安装失败，本函数会在返回前尝试恢复原 SIGINT 处理器。
 *
 * @param error_message Receives a contextual failure diagnostic.
 *                      It must not be null.
 *                      接收带上下文的失败诊断，且不得为空。
 * @return true when both handlers are installed; otherwise false.
 *         两个处理器均安装成功时返回 true，否则返回 false。
 *
 * @par Thread safety / 线程安全
 * Call once from the process main thread before starting worker threads.
 * 应在启动工作线程之前，由进程主线程调用一次。
 */
bool InstallProcessSignalHandlers(std::string* error_message);

/**
 * @brief Reports whether an installed process handler observed a stop signal.
 *        返回已安装的进程处理器是否收到停止信号。
 *
 * The function performs only an atomic read and is safe to poll from the main
 * loop. It never clears the request.
 * 本函数仅执行原子读取，可由主循环安全轮询；它不会清除停止请求。
 *
 * @return true after SIGINT or SIGTERM is observed; otherwise false.
 *         收到 SIGINT 或 SIGTERM 后返回 true，否则返回 false。
 *
 * @par Thread safety / 线程安全
 * Safe to poll from the process main thread while the signal handler may run.
 * 当信号处理器可能执行时，可由进程主线程安全轮询。
 */
bool ProcessStopRequested() noexcept;

}  // namespace mujoco_sim::sensor_demo
