#pragma once

#include <memory>
#include <string>

#include <mujoco_sim/server/server_application.hpp>

namespace mujoco_sim::server {

/**
 * @brief Owns the production application implementation for composition tests.
 *
 * This private, non-installed harness exercises the same Start() and Run()
 * implementation as ServerApplication while exposing controlled viewer-worker
 * failure injection. It must only be used by first-party tests.
 *
 * 为组合测试持有生产应用实现。
 *
 * 此私有且不安装的测试夹具使用与 ServerApplication 相同的 Start() 和 Run() 实现，
 * 同时提供受控的查看器 worker 故障注入。它只能用于第一方测试。
 */
class ServerApplicationTestHarness {
 public:
  /** Creates a stopped test application with owned configuration.
   * 使用自有配置创建尚未启动的测试应用。 */
  explicit ServerApplicationTestHarness(ServerConfig config);

  /** Stops, joins, and releases the owned runtime. 停止、join 并释放自有运行时。 */
  ~ServerApplicationTestHarness();

  /** Harness ownership cannot be copied. 测试夹具所有权不可复制。 */
  ServerApplicationTestHarness(const ServerApplicationTestHarness&) = delete;

  /** Harness ownership cannot be copy-assigned. 测试夹具所有权不可复制赋值。 */
  ServerApplicationTestHarness& operator=(const ServerApplicationTestHarness&) = delete;

  /**
   * @brief Initializes resources and starts the simulation loop.
   * @param error_message Receives contextual diagnostics. 接收带上下文的诊断。
   * @return true on success; otherwise false. 成功返回 true，否则返回 false。
   */
  bool Start(std::string* error_message);

  /**
   * @brief Runs the production composition to completion.
   * @return The same process exit code as ServerApplication::Run().
   *         与 ServerApplication::Run() 相同的进程退出码。
   */
  int Run();

  /**
   * @brief Requests a controlled failure at the next viewer-worker sync.
   * @return true when an active GUI viewer accepted the request.
   *         活跃 GUI 查看器接受请求时返回 true。
   *
   * 请求在下一次查看器 worker 同步时产生受控故障。
   */
  bool RequestViewerSyncFailure() noexcept;

  /**
   * @brief Changes the production viewer Run selection for lifecycle tests.
   * @param running true selects Run; false selects Pause. true 选择 Run；false 选择 Pause。
   * @return true when an active initialized GUI accepted the change.
   *         活跃且已初始化 GUI 接受修改时返回 true。
   * @par Thread safety / 线程安全
   * May run beside `Run()`; it never calls SimulationCore. 可与 `Run()` 并发；不调用
   * SimulationCore。
   */
  bool SetViewerRunning(bool running) noexcept;

  /**
   * @brief Injects one production-viewer Reset edge for lifecycle tests.
   * @return true when an active initialized GUI accepted the edge.
   *         活跃且已初始化 GUI 接受边沿时返回 true。
   * @par Thread safety / 线程安全
   * May run beside `Run()`; it never calls SimulationCore. 可与 `Run()` 并发；不调用
   * SimulationCore。
   */
  bool RequestViewerReset() noexcept;

 private:
  /** Exact production runtime owned by this private harness.
   * 此私有测试夹具持有的实际生产运行时。 */
  std::unique_ptr<detail::ServerApplicationImpl> impl_;
};

}  // namespace mujoco_sim::server
