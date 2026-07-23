#pragma once

#include <exception>
#include <utility>

namespace mujoco_sim::adapter_sdk::internal {

/**
 * @brief Prevents exceptions from crossing callback or thread-entry boundaries.
 *        防止异常跨越回调或线程入口边界。
 * @tparam Callback Callable containing boundary-owned work. 包含边界内工作的可调用对象。
 * @tparam OnError Callable accepting a borrowed diagnostic C string. 接受借用诊断 C
 * 字符串的可调用对象。
 * @param callback Work invoked exactly once. 恰好调用一次的工作。
 * @param on_error Error sink invoked for a thrown exception; its own exceptions are swallowed.
 *                 callback 抛出异常时调用的错误接收方；其自身异常会被吞掉。
 * @post No exception escapes. 不会有异常向外传播。
 */
template <typename Callback, typename OnError>
void InvokeCallbackBoundary(Callback&& callback, OnError&& on_error) noexcept {
  try {
    std::forward<Callback>(callback)();
  } catch (const std::exception& error) {
    try {
      std::forward<OnError>(on_error)(error.what());
    } catch (...) {
    }
  } catch (...) {
    try {
      std::forward<OnError>(on_error)("unknown exception");
    } catch (...) {
    }
  }
}

}  // namespace mujoco_sim::adapter_sdk::internal
