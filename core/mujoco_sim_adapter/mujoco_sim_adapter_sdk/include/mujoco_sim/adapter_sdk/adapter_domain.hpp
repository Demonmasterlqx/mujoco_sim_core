#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace mujoco_sim::adapter_sdk {

/**
 * @brief Identifies a contiguous resource range in a dense scalar array.
 *        标识稠密标量数组中的一段连续资源区间。
 *
 * Offsets and counts are expressed in `double` scalars, not bytes. A span is
 * valid only with the ControllerLayout from which it was obtained.
 * offset 与 count 的单位为 `double` 标量而非字节。该区间仅对其来源
 * ControllerLayout 有效。
 */
struct ScalarSpan {
  /// Zero-based scalar offset. 从零开始的标量偏移。
  std::size_t offset = 0;
  /// Number of contiguous scalars. 连续标量数量。
  std::size_t count = 0;
};

/**
 * @brief Describes one observation or command resource in a dense layout.
 *        描述稠密布局中的一个观测或指令资源。
 */
struct ResourceInfo {
  /// Protocol resource name, unique within its observation or command list.
  /// 协议资源名称，在所属观测或指令列表中唯一。
  std::string name;
  /// Contiguous scalar range occupied by this resource. 该资源占用的连续标量区间。
  ScalarSpan scalars;
  /// Whether command_min and command_max constrain command scalars.
  /// command_min 与 command_max 是否约束指令标量。
  bool has_command_range = false;
  /// Inclusive lower command bound when has_command_range is true.
  /// has_command_range 为 true 时的指令闭区间下界。
  double command_min = 0.0;
  /// Inclusive upper command bound when has_command_range is true.
  /// has_command_range 为 true 时的指令闭区间上界。
  double command_max = 0.0;
};

/**
 * @brief Owns the immutable dense resource layout granted at registration.
 *        持有注册时授予的不可变稠密资源布局。
 *
 * Resource vectors and lookup indexes are fixed by construction. Const access
 * may be shared across threads after construction. Returned references remain
 * valid until this layout is destroyed.
 * 资源向量和查询索引在构造时固定。构造完成后的 const 访问可跨线程共享。返回引用在
 * 本布局析构前保持有效。
 */
class ControllerLayout {
 public:
  /** @brief Creates an empty layout. 创建空布局。 */
  ControllerLayout() = default;

  /**
   * @brief Creates an indexed immutable layout.
   *        创建带索引的不可变布局。
   * @param observations Ordered observation resources. 有序观测资源。
   * @param commands Ordered command resources. 有序指令资源。
   * @param observation_scalar_count Exact total observation scalar count.
   *                                 精确的观测标量总数。
   * @param command_scalar_count Exact total command scalar count.
   *                             精确的指令标量总数。
   * @pre Resource names are unique within each vector and spans are within the
   *      corresponding total count. 每个向量内的资源名称唯一，且区间不超过对应总数。
   */
  ControllerLayout(std::vector<ResourceInfo> observations, std::vector<ResourceInfo> commands,
                   std::size_t observation_scalar_count, std::size_t command_scalar_count);

  /** @brief Finds an observation scalar range by exact resource name.
   * 按精确资源名查找观测标量区间。
   * @param name Resource name borrowed for this call. 本次调用期间借用的资源名。
   * @return The range, or std::nullopt when absent. 找到时返回区间，否则返回 std::nullopt。
   */
  std::optional<ScalarSpan> Observation(std::string_view name) const;

  /** @brief Finds a command scalar range by exact resource name. 按精确资源名查找指令标量区间。
   * @param name Resource name borrowed for this call. 本次调用期间借用的资源名。
   * @return The range, or std::nullopt when absent. 找到时返回区间，否则返回 std::nullopt。
   */
  std::optional<ScalarSpan> Command(std::string_view name) const;

  /** @brief Returns the exact observation scalar array length. 返回精确观测标量数组长度。
   * @return Scalar count. 标量数量。
   */
  std::size_t observation_scalar_count() const noexcept;

  /** @brief Returns the exact command scalar array length. 返回精确指令标量数组长度。
   * @return Scalar count. 标量数量。
   */
  std::size_t command_scalar_count() const noexcept;

  /** @brief Returns ordered observation metadata. 返回有序观测元数据。
   * @return Reference valid for this layout's lifetime. 在本布局生命周期内有效的引用。
   */
  const std::vector<ResourceInfo>& observations() const noexcept;

  /** @brief Returns ordered command metadata. 返回有序指令元数据。
   * @return Reference valid for this layout's lifetime. 在本布局生命周期内有效的引用。
   */
  const std::vector<ResourceInfo>& commands() const noexcept;

 private:
  /// Ordered observation resource metadata. 有序观测资源元数据。
  std::vector<ResourceInfo> observations_;
  /// Ordered command resource metadata. 有序指令资源元数据。
  std::vector<ResourceInfo> commands_;
  /// Observation name-to-span index, immutable after construction. 构造后不可变的观测名到区间索引。
  std::unordered_map<std::string, ScalarSpan> observation_index_;
  /// Command name-to-span index, immutable after construction. 构造后不可变的指令名到区间索引。
  std::unordered_map<std::string, ScalarSpan> command_index_;
  /// Exact total observation scalar count. 精确观测标量总数。
  std::size_t observation_scalar_count_ = 0;
  /// Exact total command scalar count. 精确指令标量总数。
  std::size_t command_scalar_count_ = 0;
};

/**
 * @brief Owns the immutable public projection of a successful Adapter registration.
 *        持有成功 Adapter 注册结果的不可变公开投影。
 */
struct AdapterRegistration {
  /// Adapter identifier accepted by the selected Server. 所选 Server 接受的 Adapter 标识符。
  std::string adapter_id;
  /// Nonzero identity of this registration session. 本次注册会话的非零标识。
  std::uint64_t session_id = 0;
  /// Nonzero model generation owning the layout. 持有该布局的非零模型代次。
  std::uint64_t epoch = 0;
  /// Immutable dense resource layout for this session. 本会话的不可变稠密资源布局。
  ControllerLayout layout;
};

/**
 * @brief Public Controller fault categories mapped to Protocol V4.
 *        映射到 Protocol V4 的公开控制器故障类别。
 */
enum class FaultCode {
  /// Controller algorithm or execution failure. 控制器算法或执行失败。
  kControllerError,
  /// Controller produced semantically invalid output. 控制器产生语义无效输出。
  kInvalidOutput,
  /// SDK integration or internal Controller failure. SDK 集成或控制器内部失败。
  kInternalControllerError,
};

}  // namespace mujoco_sim::adapter_sdk
