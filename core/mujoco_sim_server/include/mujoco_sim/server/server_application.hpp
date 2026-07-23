#pragma once

#include <memory>
#include <string>

#include <mujoco_sim/server/server_config.hpp>

namespace mujoco_sim::server {

namespace detail {
/** Internal process-runtime implementation hidden behind ServerApplication.
 * 隐藏在 ServerApplication 后的内部进程运行时实现。 */
class ServerApplicationImpl;
}  // namespace detail

/**
 * @brief Owns the Server process runtime and single-writer simulation loop.
 *
 * Start() initializes transport, Core, Sensors, and the optional GUI before it
 * starts the simulation thread. Run() waits for headless completion or runs
 * the GUI render loop on the calling thread. The application owns all runtime
 * resources until destruction.
 *
 * 拥有 Server 进程运行时和单写者仿真循环。
 *
 * Start() 会先初始化传输、Core、Sensor 和可选 GUI，再启动仿真线程。Run() 在 headless
 * 模式等待运行结束，或在调用线程上运行 GUI 渲染循环。应用持有全部运行时资源直至析构。
 *
 * @par Thread safety / 线程安全
 * Start() and Run() are lifecycle operations called by the process main
 * thread. RequestStop() and RequestScreenshot() may run concurrently as
 * documented. Destruction must not overlap any member call.
 * Start() 和 Run() 是由进程主线程调用的生命周期操作。RequestStop() 和
 * RequestScreenshot() 可按各自文档并发调用。析构不得与任何成员调用重叠。
 */
class ServerApplication {
 public:
  /**
   * @brief Creates a stopped application with owned configuration.
   * @param config Validated during Start(); moved into the application.
   *               在 Start() 中校验并移动到应用内的配置。
   *
   * 使用自有配置创建尚未启动的应用。
   */
  explicit ServerApplication(ServerConfig config);

  /** Stops and joins owned runtime work before releasing resources.
   * 停止并 join 所有自有运行任务，然后释放资源。 */
  ~ServerApplication();

  /** Application ownership cannot be copied. 应用所有权不可复制。 */
  ServerApplication(const ServerApplication&) = delete;

  /** Application ownership cannot be copy-assigned. 应用所有权不可复制赋值。 */
  ServerApplication& operator=(const ServerApplication&) = delete;

  /**
   * @brief Initializes resources and starts the simulation loop once.
   *
   * 初始化资源并且只启动一次仿真循环。
   *
   * @param error_message Receives a contextual initialization diagnostic.
   *                      接收带上下文的初始化诊断。
   * @return true on success; false before any usable Run() on failure.
   *         成功返回 true；失败返回 false，此时不可正常调用 Run()。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Runs until shutdown and returns the process exit code.
   *
   * 运行至关闭并返回进程退出码。
   *
   * @return 0 for requested or bounded normal shutdown, 1 when not started,
   *         or 2 for a runtime, viewer-worker, transport, or logging failure.
   *         请求关闭或有界正常关闭返回 0；尚未启动返回 1；运行时、查看器 worker、
   *         传输或日志失败返回 2。
   * @par Blocking / 阻塞
   * Blocks until all simulation work stops. In GUI mode it must run on the
   * main thread required by GLFW.
   * 阻塞至全部仿真工作停止。GUI 模式下必须在 GLFW 要求的主线程运行。
   */
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

  /**
   * @return Borrowed immutable configuration valid for this object's lifetime.
   *         在本对象生命周期内有效的只读借用配置。
   * @par Thread safety / 线程安全
   * Safe to read while the application is alive because configuration is
   * immutable after construction.
   * 应用存活期间可安全读取，因为配置在构造后保持不变。
   */
  const ServerConfig& config() const noexcept;

 private:
  /** Lifetime-stable process-runtime implementation. 生命周期稳定的进程运行时实现。 */
  std::unique_ptr<detail::ServerApplicationImpl> impl_;
};

}  // namespace mujoco_sim::server
