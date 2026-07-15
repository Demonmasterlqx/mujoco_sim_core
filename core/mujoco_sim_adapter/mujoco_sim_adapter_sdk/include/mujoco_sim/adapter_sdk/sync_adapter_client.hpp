#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>

namespace mujoco_sim::adapter_sdk {

/// Controller registration, resource, and transport options.
struct AdapterOptions {
  /// Server namespace used to derive service and topic names.
  std::string server_name = "default";
  /// Non-empty Controller identifier, unique within the selected Server.
  std::string adapter_id;
  /// Observation resources requested in registration order.
  std::vector<std::string> observation_names;
  /// Command resources requested in registration order.
  std::vector<std::string> command_names;
  /// Barrier Commit timeout in milliseconds; must be positive.
  std::uint32_t commit_timeout_ms = 1000;
  /// Session presence timeout in milliseconds; must be positive.
  std::uint32_t session_timeout_ms = 1000;
  /// Suppressed keepalive period; must not exceed session_timeout_ms / 3.
  std::uint32_t heartbeat_period_ms = 250;
  /// Discovery and RPC timeout in milliseconds; must be positive.
  std::uint32_t service_timeout_ms = 5000;
};

class TickLease {
 public:
  struct Data;
  TickLease();
  ~TickLease();
  TickLease(const TickLease&) = delete;
  TickLease& operator=(const TickLease&) = delete;
  TickLease(TickLease&&) noexcept;
  TickLease& operator=(TickLease&&) noexcept;

  std::uint64_t tick_id() const noexcept;
  double simulation_time_s() const noexcept;
  bool recovery_proof() const noexcept;
  std::span<const double> observations() const noexcept;
  explicit operator bool() const noexcept;

 private:
  std::shared_ptr<const Data> data_;
  const void* owner_ = nullptr;
  friend class SyncAdapterClient;
};

/// Result of a blocking or polling Tick wait.
enum class TickWaitResult {
  /// A Tick was written to the output argument.
  kReceived,
  /// The deadline elapsed; output arguments were not changed.
  kTimeout,
  /// Local cancellation or successful remote cleanup ended the wait.
  kStopped,
  /// A terminal lifecycle or protocol error ended the wait.
  kError,
};

/// Single-session synchronous Controller client.
///
/// Start(), Activate(), Commit(), and Unregister() are serialized internally.
/// Stop() may be called concurrently to cancel local work. Only one thread may
/// wait for a Tick at a time. The object cannot be restarted after Stop().
class SyncAdapterClient {
 public:
  /// Creates a non-started, single-use Controller client.
  explicit SyncAdapterClient(AdapterOptions options);
  /// Stops local work, drains callbacks, and attempts remote cleanup once.
  ~SyncAdapterClient();

  /// Clients are neither copyable nor movable.
  SyncAdapterClient(const SyncAdapterClient&) = delete;
  /// Clients are neither copyable nor movable.
  SyncAdapterClient& operator=(const SyncAdapterClient&) = delete;

  /// Registers the Controller and creates receive endpoints.
  bool Start(std::string* error_message);
  /// Activates the provisional session. Start() must have succeeded.
  bool Activate(std::string* error_message);
  /// Waits for one Tick. A zero timeout polls; max() waits indefinitely.
  /// Output is changed only when kReceived is returned.
  TickWaitResult WaitForTick(std::chrono::milliseconds timeout,
                             TickLease* tick,
                             std::string* error_message);
  /// Validates and publishes commands for the current outstanding Tick.
  /// A failed local validation keeps the Tick available for correction.
  bool Commit(const TickLease& tick, std::span<const double> commands,
              std::string* error_message);
  bool FailTick(const TickLease& tick, FaultCode code,
                std::string_view message, std::string* error_message);
  /// Cancels local waits and delivery without contacting the Server.
  void Stop();
  /// Removes the remote session; failures preserve it for a later retry.
  bool Unregister(std::string* error_message);
  /// Returns the accumulated asynchronous lifecycle diagnostic.
  std::string lifecycle_warning() const;
  /// Returns the count of rejected Tick layout hash mismatches.
  std::uint64_t layout_hash_mismatches() const;

  const AdapterRegistration& registration() const;
  const ControllerLayout& layout() const;

 private:
  friend class SyncAdapterClientTestPeer;
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::adapter_sdk
