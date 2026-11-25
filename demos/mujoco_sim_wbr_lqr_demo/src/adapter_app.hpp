#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace mujoco_sim::wbr_lqr_demo {

class TerminalKeyboardInput;

/**
 * @brief Creates the process-private keyboard source selected by cin mode.
 *
 * Production supplies `TerminalKeyboardInput::OpenStandardInput`; integration
 * tests may supply an owned fake byte source. The requested direction hold
 * duration is expressed in Controller updates. `RunAdapter` owns the result.
 *
 * 创建 cin 模式选用的进程私有键盘输入源。
 *
 * 生产环境传入 `TerminalKeyboardInput::OpenStandardInput`；集成测试可以传入拥有 fake
 * 字节源的工厂。请求的方向保持时长以 Controller update 表示，返回对象由 `RunAdapter`
 * 持有。
 */
using KeyboardInputFactory = std::unique_ptr<TerminalKeyboardInput> (*)(
    std::uint32_t direction_hold_ticks, std::string* error_message);

/**
 * @brief Process-private synchronization hook used only by integration tests.
 *
 * The hook runs on the application thread without project locks. It returns a
 * contextual error instead of throwing across the adapter boundary.
 *
 * 仅供集成测试使用的进程私有同步 hook。
 *
 * hook 在应用线程且不持有项目锁时运行；它通过上下文错误返回失败，不跨 adapter
 * 边界抛出异常。
 */
using AdapterLifecycleHook = bool (*)(std::string* error_message);

/**
 * @brief Optional integration-test synchronization points in the adapter lifecycle.
 *
 * Production leaves both hooks null. `after_keyboard_start` runs after the
 * reader starts and before the first State wait. `before_keyboard_stop` runs
 * after the control loop and before reader stop/join.
 *
 * adapter 生命周期中可选的集成测试同步点。
 *
 * 生产环境将两个 hook 均置空。`after_keyboard_start` 在 reader 启动后、首次等待 State
 * 前运行；`before_keyboard_stop` 在控制循环结束后、reader stop/join 前运行。
 */
struct AdapterTestHooks {
  AdapterLifecycleHook after_keyboard_start = nullptr;
  AdapterLifecycleHook before_keyboard_stop = nullptr;
};

/**
 * @brief Runs the WBR Controller adapter lifecycle with an injected keyboard factory.
 *
 * The function parses configuration, validates and opens the terminal before remote
 * registration, processes Controller State on the application thread, then
 * stops the reader and unregisters in order. The factory is called only in
 * cin mode. It must return owned storage or a contextual error.
 *
 * 使用注入的键盘工厂运行 WBR Controller adapter 生命周期。
 *
 * 本函数解析配置，在远端注册前校验并打开终端，在应用线程处理 Controller State，
 * 随后依次停止 reader 并注销。工厂仅在 cin 模式调用，且须返回自有对象或上下文错误。
 *
 * @param argc Command-line argument count.
 *             命令行参数数量。
 * @param argv Command-line argument vector borrowed for this call.
 *             本次调用期间借用的命令行参数数组。
 * @param keyboard_factory Process-private keyboard factory; required in cin mode.
 *                         进程私有键盘工厂；cin 模式下必须提供。
 * @param test_hooks Optional process-private integration-test hooks. Production
 *                   callers must leave this null.
 *                   可选的进程私有集成测试 hook；生产调用方必须置空。
 * @return Zero after normal shutdown; nonzero on initialization, reader,
 *         Controller, or unregister failure.
 *         正常关闭返回零；初始化、reader、Controller 或注销失败时返回非零。
 *
 * @par Thread safety / 线程安全
 * Call once on the application thread. The function blocks until the configured
 * tick budget or a stop request ends the adapter lifecycle.
 * 仅可在应用线程调用一次。本函数会阻塞，直至配置的 tick 预算或停止请求结束 adapter 生命周期。
 */
int RunAdapter(int argc, char** argv, KeyboardInputFactory keyboard_factory,
               const AdapterTestHooks* test_hooks = nullptr);

}  // namespace mujoco_sim::wbr_lqr_demo
