#pragma once

#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "status_tool.hpp"

namespace eCAL {
class CServiceClient;
}

namespace mujoco_sim::server::status_tool::internal {

/**
 * @brief Owns the result or contextual error from one resource query.
 *        持有一次资源查询的结果或上下文错误。
 */
struct ResourceQueryResult {
  /// Validated response from the matching Server process. 来自匹配 Server 进程的已验证响应。
  std::optional<mujoco_sim::v4::GetAvailableResourcesResponse> response;
  /// Diagnostic used when no validated response is available. 无有效响应时使用的诊断。
  std::string error;
};

/**
 * @brief Queries resources from the registry process that published a status.
 *
 * The call is bounded by the eCAL timeout and returns owned data. It validates
 * process identity, transport state, Protobuf bytes, response code, and epoch.
 *
 * 查询发布状态的同一注册服务进程所提供的资源。
 *
 * 调用受 eCAL 超时限制并返回自有数据；它会验证进程标识、传输状态、Protobuf 字节、
 * 响应码和模型代次。
 *
 * @param client Borrowed registry client used only for this call. 仅用于本次调用的注册客户端。
 * @param publisher Status publisher identity to match. 要匹配的状态发布者标识。
 * @param epoch Required model epoch. 要求的模型代次。
 * @return Validated response or a contextual diagnostic. 已验证响应或上下文诊断。
 */
ResourceQueryResult QueryResources(eCAL::CServiceClient* client, const PublisherIdentity& publisher,
                                   std::uint64_t epoch);

/**
 * @brief Gates status callbacks and supports bounded-lifetime teardown.
 *
 * StopAccepting() prevents later admission. WaitForIdle() releases the mutex
 * while waiting for every admitted callback to leave.
 *
 * 门控状态回调并支持生命周期受控的拆除。
 *
 * StopAccepting() 阻止后续准入；WaitForIdle() 在等待全部已准入回调退出时释放互斥锁。
 */
class StatusCallbackGate {
 public:
  /** @brief Attempts to admit one callback. 尝试准入一个回调。
   * @return true when Leave() must later be called. 返回 true 时之后必须调用 Leave()。 */
  bool Enter();
  /** @brief Releases one admitted callback without throwing. 无异常地释放一个已准入回调。 */
  void Leave() noexcept;
  /** @brief Rejects all later callbacks. 拒绝所有后续回调。 */
  void StopAccepting();
  /** @brief Blocks until every admitted callback has left. 阻塞直到全部已准入回调退出。 */
  void WaitForIdle();

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t active_ = 0;
  bool accepting_ = true;
};

/**
 * @brief RAII lease for one status callback admission.
 *        一次状态回调准入的 RAII 租约。
 */
class StatusCallbackLease {
 public:
  /**
   * @brief Attempts admission through a shared gate.
   * @param gate Shared gate that outlives this lease. 生命周期长于本租约的共享门控。
   */
  explicit StatusCallbackLease(std::shared_ptr<StatusCallbackGate> gate);
  /** @brief Releases admission when it was granted. 在已获准入时释放准入。 */
  ~StatusCallbackLease() noexcept;

  /** @brief Copying is forbidden because one admission has one owner.
   *         一次准入权只有一个所有者，禁止复制。 */
  StatusCallbackLease(const StatusCallbackLease&) = delete;
  /** @brief Copy assignment is forbidden because it could release another admission.
   *         复制赋值可能释放另一项准入权，因此禁止。 */
  StatusCallbackLease& operator=(const StatusCallbackLease&) = delete;

  /** @brief Reports whether admission was granted. 返回是否已获准入。 */
  explicit operator bool() const noexcept { return entered_; }

 private:
  std::shared_ptr<StatusCallbackGate> gate_;
  bool entered_ = false;
};

}  // namespace mujoco_sim::server::status_tool::internal
