#pragma once

#include <memory>
#include <string>

#include <mujoco_sim/server/server_config.hpp>

namespace mujoco_sim::server {

/// Owns the server process runtime and its single-writer simulation loop.
///
/// Start() initializes all resources and starts the simulation thread. Run()
/// blocks until that thread exits, or runs the GUI loop on the calling thread.
/// RequestStop() is idempotent and may be called concurrently from any thread.
class ServerApplication {
 public:
  explicit ServerApplication(ServerConfig config);
  ~ServerApplication();

  ServerApplication(const ServerApplication&) = delete;
  ServerApplication& operator=(const ServerApplication&) = delete;

  bool Start(std::string* error_message);
  int Run();

  /**
   * @brief Requests cooperative, non-blocking Server shutdown.
   *
   * The first request records the terminal reason, rejects new input, wakes
   * blocking waits, and asks Sensor and GUI workers to stop. Repeated calls are
   * idempotent. Exceptions from internal shutdown notification are contained
   * and logged.
   *
   * 请求 Server 以协作、非阻塞方式关闭。
   *
   * 首次请求会记录终止原因、拒绝新输入、唤醒阻塞等待，并请求 Sensor 与 GUI worker
   * 停止。重复调用不改变结果。内部停止通知产生的异常会被隔离并记录。
   *
   * @param reason Owned diagnostic reason for the first request.
   *               首次请求所使用的自有诊断原因。
   *
   * @par Thread safety / 线程安全
   * May be called concurrently with Run() and other RequestStop() calls, but
   * not with destruction.
   * 可与 Run() 及其他 RequestStop() 调用并发，但不得与析构并发。
   */
  void RequestStop(std::string reason = "external_stop") noexcept;

  /**
   * @brief Queues one screenshot request for the graphical viewer.
   *
   * The request is non-blocking and may be submitted concurrently with Run()
   * after Start() succeeds. The MuJoCo viewer selects the output location; on
   * the supported Linux baseline its default is `screenshot.png` in the
   * process working directory. The caller must keep this application alive
   * until the request has been processed.
   *
   * 为图形查看器排队一次截图请求。
   *
   * 本请求是非阻塞的；Start() 成功后，可在 Run() 执行期间并发调用。输出位置由
   * MuJoCo 查看器选择；在受支持的 Linux 基线上，默认写入进程工作目录中的
   * `screenshot.png`。调用方必须保持本应用对象存活，直至请求处理完成。
   *
   * @return true if a running GUI viewer accepted the request; otherwise false
   *         before successful startup, after Run() finishes, or in headless mode.
   *         正在运行的 GUI 查看器接受请求时返回 true；启动成功前、Run() 结束后或
   *         headless 模式下返回 false。
   *
   * @par Thread safety / 线程安全
   * This function may run concurrently with Run() and RequestStop(), but not
   * with destruction.
   * 本函数可以与 Run() 和 RequestStop() 并发执行，但不得与析构并发。
   */
  bool RequestScreenshot();

  const ServerConfig& config() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::server
