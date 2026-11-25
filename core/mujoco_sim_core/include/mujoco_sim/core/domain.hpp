#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mujoco_sim::core {

enum ErrorCode {
  ERROR_CODE_UNSPECIFIED = 0,
  OK = 1,
  INVALID_ARGUMENT = 2,
  NOT_FOUND = 3,
  ALREADY_EXISTS = 4,
  VERSION_MISMATCH = 5,
  RESOURCE_CONFLICT = 6,
  INVALID_SESSION = 7,
  INVALID_EPOCH = 8,
  INVALID_TICK = 9,
  INVALID_COMMAND = 10,
  TIMEOUT = 11,
  FAILED_PRECONDITION = 12,
  INTERNAL_ERROR = 13,
};
enum ResourceType {
  RESOURCE_TYPE_UNSPECIFIED = 0,
  OBSERVATION_SENSOR = 1,
  COMMAND_ACTUATOR = 2
};
enum AdapterRole {
  ADAPTER_ROLE_UNSPECIFIED = 0,
  CONTROLLER = 1,
  OBSERVER = 2
};
enum AdapterState {
  ADAPTER_STATE_UNSPECIFIED = 0,
  PROVISIONAL = 1,
  ACTIVE_NO_COMMAND = 2,
  ONLINE = 3,
  OFFLINE = 4,
  ERROR = 5,
  UNREGISTERED = 6
};
/**
 * @brief Describes the authoritative Server simulation lifecycle state.
 *
 * PAUSED is a normal local manual pause that keeps communication and liveness
 * processing active while completed-step tick and simulation time remain
 * unchanged. ERROR_PAUSED is terminal for ordinary client operation and cannot
 * be resumed through manual pause control. Existing numeric values remain
 * fixed and PAUSED is appended as value 5.
 *
 * 描述权威 Server 的仿真生命周期状态。
 *
 * PAUSED 是正常的本地手动暂停，通信和存活性处理仍然继续，completed-step tick 与
 * simulation time 保持不变。ERROR_PAUSED 是普通客户端操作的终止错误状态，不能通过
 * 手动暂停控制恢复。既有数值保持固定，PAUSED 以数值 5 追加。
 */
enum ServerState {
  SERVER_STATE_UNSPECIFIED = 0,
  IDLE = 1,
  RUNNING = 2,
  ERROR_PAUSED = 3,
  STOPPED = 4,
  PAUSED = 5
};
enum class ControlPolicy {
  kZero,
  kHoldLast
};

struct SessionIdentity {
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
};
struct LayoutFingerprint {
  std::uint64_t low = 0;
  std::uint64_t high = 0;
  friend bool operator==(const LayoutFingerprint&, const LayoutFingerprint&) = default;
};

struct ResourceDescriptor {
  std::string name_;
  ResourceType type_ = RESOURCE_TYPE_UNSPECIFIED;
  std::uint32_t dimension_ = 0;
  std::int32_t mujoco_id_ = -1, mujoco_type_ = 0, mujoco_datatype_ = 0;
  bool has_ctrl_range_ = false;
  double ctrl_min_ = 0.0, ctrl_max_ = 0.0;
  std::int32_t mujoco_stage_ = 0;
  double interval_period_s_ = 0.0, interval_phase_s_ = 0.0, delay_s_ = 0.0;
  std::string frame_id_, unit_, object_name_, interpolation_;
  std::int32_t object_type_ = 0;
  const std::string& name() const noexcept { return name_; }
  void set_name(std::string v) { name_ = std::move(v); }
  ResourceType type() const noexcept { return type_; }
  void set_type(ResourceType v) noexcept { type_ = v; }
  std::uint32_t dimension() const noexcept { return dimension_; }
  void set_dimension(std::uint32_t v) noexcept { dimension_ = v; }
  std::int32_t mujoco_id() const noexcept { return mujoco_id_; }
  void set_mujoco_id(std::int32_t v) noexcept { mujoco_id_ = v; }
  std::int32_t mujoco_type() const noexcept { return mujoco_type_; }
  void set_mujoco_type(std::int32_t v) noexcept { mujoco_type_ = v; }
  std::int32_t mujoco_datatype() const noexcept { return mujoco_datatype_; }
  void set_mujoco_datatype(std::int32_t v) noexcept { mujoco_datatype_ = v; }
  bool has_ctrl_range() const noexcept { return has_ctrl_range_; }
  void set_has_ctrl_range(bool v) noexcept { has_ctrl_range_ = v; }
  double ctrl_min() const noexcept { return ctrl_min_; }
  void set_ctrl_min(double v) noexcept { ctrl_min_ = v; }
  double ctrl_max() const noexcept { return ctrl_max_; }
  void set_ctrl_max(double v) noexcept { ctrl_max_ = v; }
  std::int32_t mujoco_stage() const noexcept { return mujoco_stage_; }
  void set_mujoco_stage(std::int32_t v) noexcept { mujoco_stage_ = v; }
  double interval_period_s() const noexcept { return interval_period_s_; }
  void set_interval_period_s(double v) noexcept { interval_period_s_ = v; }
  double interval_phase_s() const noexcept { return interval_phase_s_; }
  void set_interval_phase_s(double v) noexcept { interval_phase_s_ = v; }
  double delay_s() const noexcept { return delay_s_; }
  void set_delay_s(double v) noexcept { delay_s_ = v; }
  const std::string& frame_id() const noexcept { return frame_id_; }
  void set_frame_id(std::string v) { frame_id_ = std::move(v); }
  const std::string& unit() const noexcept { return unit_; }
  void set_unit(std::string v) { unit_ = std::move(v); }
  const std::string& object_name() const noexcept { return object_name_; }
  void set_object_name(std::string v) { object_name_ = std::move(v); }
  std::int32_t object_type() const noexcept { return object_type_; }
  void set_object_type(std::int32_t v) noexcept { object_type_ = v; }
  const std::string& interpolation() const noexcept { return interpolation_; }
  void set_interpolation(std::string v) { interpolation_ = std::move(v); }
};

struct ResourceSpan {
  std::uint32_t offset = 0, dimension = 0;
};
struct ResourceLayout {
  std::vector<ResourceDescriptor> observations_, commands_;
  std::vector<ResourceSpan> observation_spans, command_spans;
  LayoutFingerprint fingerprint;
  std::uint32_t observation_scalar_count = 0, command_scalar_count = 0;
  const auto& observations() const noexcept { return observations_; }
  const auto& commands() const noexcept { return commands_; }
  int observations_size() const noexcept { return static_cast<int>(observations_.size()); }
  int commands_size() const noexcept { return static_cast<int>(commands_.size()); }
};

enum class DeliveryKind {
  kUnset,
  kEveryNSteps,
  kPeriod
};
struct ObservationDelivery {
  DeliveryKind kind = DeliveryKind::kUnset;
  std::uint32_t every_n_steps_value = 0;
  double period_s_value = 0.0;
  bool has_every_n_steps() const noexcept { return kind == DeliveryKind::kEveryNSteps; }
  bool has_period_s() const noexcept { return kind == DeliveryKind::kPeriod; }
  std::uint32_t every_n_steps() const noexcept { return every_n_steps_value; }
  double period_s() const noexcept { return period_s_value; }
  void set_every_n_steps(std::uint32_t v) noexcept {
    kind = DeliveryKind::kEveryNSteps;
    every_n_steps_value = v;
  }
  void set_period_s(double v) noexcept {
    kind = DeliveryKind::kPeriod;
    period_s_value = v;
  }
};
struct ControllerRuntimePolicy {
  std::uint32_t command_timeout_ms = 500, command_keepalive_ms = 100;
  ObservationDelivery state_delivery = [] {
    ObservationDelivery d;
    d.set_every_n_steps(1);
    return d;
  }();
  ControlPolicy startup_policy = ControlPolicy::kZero;
  ControlPolicy offline_policy = ControlPolicy::kHoldLast;
  ControlPolicy error_policy = ControlPolicy::kZero;
  ControlPolicy release_policy = ControlPolicy::kZero;
  std::uint32_t offline_session_expire_ms = 0;
};

struct GetAvailableResourcesRequest {};
struct GetAvailableResourcesResponse {
  ErrorCode code_ = OK;
  std::string message_;
  std::uint64_t epoch_ = 0;
  std::vector<ResourceDescriptor> observations_, commands_;
  ErrorCode code() const noexcept { return code_; }
  void set_code(ErrorCode v) { code_ = v; }
  const std::string& message() const noexcept { return message_; }
  void set_message(std::string v) { message_ = std::move(v); }
  std::uint64_t epoch() const noexcept { return epoch_; }
  void set_epoch(std::uint64_t v) { epoch_ = v; }
  ResourceDescriptor* add_observations() {
    observations_.emplace_back();
    return &observations_.back();
  }
  ResourceDescriptor* add_commands() {
    commands_.emplace_back();
    return &commands_.back();
  }
  const auto& observations() const noexcept { return observations_; }
  const auto& commands() const noexcept { return commands_; }
};
struct RegisterAdapterRequest {
  std::string adapter_id_;
  AdapterRole role_ = ADAPTER_ROLE_UNSPECIFIED;
  std::vector<std::string> observation_names_, command_names_;
  std::uint32_t session_timeout_ms_ = 0;
  ObservationDelivery delivery_;
  const std::string& adapter_id() const noexcept { return adapter_id_; }
  void set_adapter_id(std::string v) { adapter_id_ = std::move(v); }
  AdapterRole role() const noexcept { return role_; }
  void set_role(AdapterRole v) { role_ = v; }
  const auto& observation_names() const noexcept { return observation_names_; }
  void add_observation_names(std::string v) { observation_names_.push_back(std::move(v)); }
  const auto& command_names() const noexcept { return command_names_; }
  void add_command_names(std::string v) { command_names_.push_back(std::move(v)); }
  int observation_names_size() const { return static_cast<int>(observation_names_.size()); }
  int command_names_size() const { return static_cast<int>(command_names_.size()); }
  std::uint32_t session_timeout_ms() const { return session_timeout_ms_; }
  void set_session_timeout_ms(std::uint32_t v) { session_timeout_ms_ = v; }
  const ObservationDelivery& delivery() const { return delivery_; }
  ObservationDelivery* mutable_delivery() { return &delivery_; }
};
struct RegisterAdapterResponse {
  ErrorCode code_ = OK;
  std::string message_, adapter_id_;
  AdapterRole role_ = ADAPTER_ROLE_UNSPECIFIED;
  SessionIdentity session;
  ResourceLayout layout_;
  AdapterState state_ = ADAPTER_STATE_UNSPECIFIED;
  ControllerRuntimePolicy controller_policy;
  ErrorCode code() const { return code_; }
  void set_code(ErrorCode v) { code_ = v; }
  const std::string& message() const { return message_; }
  void set_message(std::string v) { message_ = std::move(v); }
  const std::string& adapter_id() const { return adapter_id_; }
  void set_adapter_id(std::string v) { adapter_id_ = std::move(v); }
  AdapterRole role() const { return role_; }
  void set_role(AdapterRole v) { role_ = v; }
  std::uint64_t session_id() const { return session.session_id; }
  void set_session_id(std::uint64_t v) { session.session_id = v; }
  std::uint64_t epoch() const { return session.epoch; }
  void set_epoch(std::uint64_t v) { session.epoch = v; }
  const ResourceLayout& layout() const { return layout_; }
  ResourceLayout* mutable_layout() { return &layout_; }
  AdapterState state() const { return state_; }
  void set_state(AdapterState v) { state_ = v; }
};
struct AdapterSessionRequest {
  std::uint64_t session_id_ = 0, epoch_ = 0;
  std::uint64_t session_id() const { return session_id_; }
  void set_session_id(std::uint64_t v) { session_id_ = v; }
  std::uint64_t epoch() const { return epoch_; }
  void set_epoch(std::uint64_t v) { epoch_ = v; }
};
using ActivateAdapterRequest = AdapterSessionRequest;
using UnregisterAdapterRequest = AdapterSessionRequest;
struct AdapterSessionResponse {
  ErrorCode code_ = OK;
  std::string message_;
  AdapterState state_ = ADAPTER_STATE_UNSPECIFIED;
  ErrorCode code() const { return code_; }
  void set_code(ErrorCode v) { code_ = v; }
  const std::string& message() const { return message_; }
  void set_message(std::string v) { message_ = std::move(v); }
  AdapterState state() const { return state_; }
  void set_state(AdapterState v) { state_ = v; }
};
using ActivateAdapterResponse = AdapterSessionResponse;
using UnregisterAdapterResponse = AdapterSessionResponse;

struct FrameIdentity {
  SessionIdentity session;
  std::uint64_t tick_id = 0;
  LayoutFingerprint layout;
  double simulation_time_s = 0.0;
};
struct ControllerCommand {
  SessionIdentity session;
  LayoutFingerprint layout;
  std::uint64_t sequence = 0;
  std::vector<double> commands;
};
enum class FaultCode : std::uint8_t {
  kUnspecified = 0,
  kControllerError = 1,
  kInvalidOutput = 2,
  kInternalControllerError = 3,
};
struct ControllerFault {
  SessionIdentity session;
  FaultCode code = FaultCode::kUnspecified;
  std::string message;
};
struct AdapterHeartbeat {
  SessionIdentity session;
  std::uint64_t last_seen = 0;
};
struct ControllerStateFrame {
  FrameIdentity identity;
  std::uint64_t sequence = 0;
  std::vector<double> observations;
};
struct ObservationFrame {
  FrameIdentity identity;
  std::uint64_t sequence = 0;
  std::vector<double> observations;
};
struct AdapterStatus {
  std::string adapter_id;
  AdapterRole role = ADAPTER_ROLE_UNSPECIFIED;
  std::uint64_t session_id = 0;
  AdapterState state = ADAPTER_STATE_UNSPECIFIED;
  std::string message;
  std::uint64_t state_since_tick_id = 0, timeout_count = 0, last_command_sequence = 0,
                last_observation_sequence = 0;
  std::optional<std::uint64_t> last_delivery_tick_id;
};
struct ServerStatus {
  std::uint64_t epoch = 0, tick_id = 0;
  double simulation_time_s = 0;
  ServerState state = SERVER_STATE_UNSPECIFIED;
  std::string message;
  std::vector<AdapterStatus> adapters;
};

}  // namespace mujoco_sim::core
