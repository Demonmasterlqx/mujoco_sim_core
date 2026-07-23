#pragma once

#include <memory>
#include <string>

namespace mujoco_sim::adapter_sdk::internal {

/**
 * @brief Acquires a process-wide lease for the eCAL runtime.
 *
 * Concurrent callers share one initialized runtime. Initialization and final
 * lease release are serialized through transition states, but the eCAL API is
 * invoked without holding the bookkeeping mutex. The returned shared pointer
 * releases one lease when its last copy is destroyed.
 *
 * 获取进程级 eCAL 运行时租约。
 *
 * 并发调用方共享同一个已初始化运行时。初始化与最后一个租约的释放通过过渡状态串行化，
 * 但调用 eCAL API 时不持有记账互斥锁。返回的共享指针在最后一个副本销毁时释放一个租约。
 *
 * @param unit_name Unit name passed to eCAL when this call owns initialization.
 *                  当本次调用负责初始化时传给 eCAL 的单元名称。
 * @param error_message Receives a contextual failure message; must not be null.
 *                      接收带上下文的失败信息；不得为空。
 * @return An owning lease, or an empty pointer on failure.
 *         成功时返回持有租约，失败时返回空指针。
 */
std::shared_ptr<void> AcquireEcalRuntime(const std::string& unit_name, std::string* error_message);

}  // namespace mujoco_sim::adapter_sdk::internal
