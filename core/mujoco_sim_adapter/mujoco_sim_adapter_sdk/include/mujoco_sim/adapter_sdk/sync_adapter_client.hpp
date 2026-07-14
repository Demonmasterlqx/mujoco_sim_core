#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <mujoco_sim/v2/sync.pb.h>

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
  /// Server-side Controller timeout in milliseconds; must be positive.
  std::uint32_t timeout_ms = 1000;
  /// Discovery and RPC timeout in milliseconds; must be positive.
  std::uint32_t service_timeout_ms = 5000;
};

/// One scalar actuator command supplied to Commit().
struct CommandValue {
  /// Registered scalar command resource name.
  std::string name;
  /// Command value in the resource unit declared by the Server.
  double value = 0.0;
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
                             mujoco_sim::v2::Tick* tick,
                             std::string* error_message);
  /// Validates and publishes commands for the current outstanding Tick.
  /// A failed local validation keeps the Tick available for correction.
  bool Commit(const mujoco_sim::v2::Tick& tick,
              const std::vector<CommandValue>& commands,
              std::string* error_message);
  /// Cancels local waits and delivery without contacting the Server.
  void Stop();
  /// Removes the remote session; failures preserve it for a later retry.
  bool Unregister(std::string* error_message);
  /// Returns the accumulated asynchronous lifecycle diagnostic.
  std::string lifecycle_warning() const;
  /// Returns the count of rejected Tick layout hash mismatches.
  std::uint64_t layout_hash_mismatches() const;

  /// Returns the last successful registration response.
  /// Call only after Start() has completed and do not race this access with it.
  const mujoco_sim::v2::RegisterAdapterResponse& registration() const;

 private:
  friend class SyncAdapterClientTestPeer;
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::adapter_sdk
