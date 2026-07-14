#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::adapter_sdk {

/// Observer registration, sampling, queue, and transport options.
struct ObserverOptions {
  /// Server namespace used to derive service and topic names.
  std::string server_name = "default";
  /// Non-empty Observer identifier, unique within the selected Server.
  std::string observer_id;
  /// Observation resources requested in registration order.
  std::vector<std::string> observation_names;
  /// Server-side Observer timeout in milliseconds; must be positive.
  std::uint32_t timeout_ms = 1000;
  /// Discovery and RPC timeout in milliseconds; must be positive.
  std::uint32_t service_timeout_ms = 5000;
  /// Heartbeat period in milliseconds; must be positive and below timeout_ms.
  std::uint32_t keepalive_period_ms = 250;
  /// Optional step-based delivery cadence; when set, must be positive.
  std::optional<std::uint32_t> every_n_steps = 1;
  /// Optional simulation-time delivery cadence in seconds; must be positive.
  std::optional<double> period_s;
  /// Bounded frame queue capacity; must be positive.
  std::size_t queue_capacity = 1;
};

/// Result of a blocking or polling observation wait.
enum class ObservationWaitResult {
  /// An ObservationFrame was written to the output argument.
  kReceived,
  /// The deadline elapsed; output arguments were not changed.
  kTimeout,
  /// Local cancellation or successful remote cleanup ended the wait.
  kStopped,
  /// A terminal lifecycle or protocol error ended the wait.
  kError,
};

/// Single-session read-only Observer client.
///
/// Lifecycle calls are serialized internally. Stop() may cancel waits from
/// another thread, while Unregister() performs separately retryable cleanup.
class ObserverClient {
 public:
  /// Creates a non-started, single-use Observer client.
  explicit ObserverClient(ObserverOptions options);
  /// Stops local work, drains callbacks, and attempts remote cleanup once.
  ~ObserverClient();

  /// Clients are neither copyable nor movable.
  ObserverClient(const ObserverClient&) = delete;
  /// Clients are neither copyable nor movable.
  ObserverClient& operator=(const ObserverClient&) = delete;

  /// Registers the Observer and starts receive/heartbeat infrastructure.
  bool Start(std::string* error_message);
  /// Activates the provisional session. Start() must have succeeded.
  bool Activate(std::string* error_message);
  /// Waits for one frame. Only one waiter is allowed per client.
  /// Output is changed only when kReceived is returned.
  ObservationWaitResult WaitForObservation(
      std::chrono::milliseconds timeout,
      mujoco_sim::v2::ObservationFrame* frame,
      std::string* error_message);
  /// Cancels local waits and heartbeat without contacting the Server.
  void Stop();
  /// Removes the remote session; failures preserve it for a later retry.
  bool Unregister(std::string* error_message);

  /// Returns sequence gaps plus frames evicted by the bounded mailbox.
  std::uint64_t dropped_frames() const;
  /// Returns the accumulated asynchronous lifecycle diagnostic.
  std::string lifecycle_warning() const;

  /// Returns the last successful registration response.
  /// Call only after Start() has completed and do not race this access with it.
  const mujoco_sim::v2::RegisterAdapterResponse& registration() const;

 private:
  friend class ObserverClientTestPeer;
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::adapter_sdk
