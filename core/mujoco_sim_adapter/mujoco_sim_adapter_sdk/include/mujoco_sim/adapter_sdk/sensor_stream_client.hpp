#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::adapter_sdk {

/// Sensor stream discovery, selection, queue, and transport options.
struct SensorStreamOptions {
  /// Server namespace used to derive service and topic names.
  std::string server_name = "default";
  /// Non-empty stream names to discover and subscribe; duplicates are invalid.
  std::vector<std::string> stream_names;
  /// Discovery and RPC timeout in milliseconds; must be positive.
  std::uint32_t service_timeout_ms = 5000;
  /// Per-stream bounded sample queue capacity; must be positive.
  std::size_t queue_capacity = 1;
};

/// Owning stream header and payload suitable for cross-callback retention.
struct OwnedStreamSample {
  /// Parsed metadata copied from the wire frame.
  mujoco_sim::v2::StreamFrameHeader header;
  /// Owned raw payload bytes described by header.
  std::vector<std::byte> payload;
};

/// Non-owning sample view passed to StreamCallback.
///
/// The payload is borrowed from the transport and is valid only while the
/// callback is running. Use Copy() to retain a sample.
class StreamSampleView {
 public:
  /// Views cannot be copied because their payload lifetime is callback-bound.
  StreamSampleView(const StreamSampleView&) = delete;
  /// Views cannot be assigned because their payload lifetime is callback-bound.
  StreamSampleView& operator=(const StreamSampleView&) = delete;

  /// Returns the owned header valid for the lifetime of this view.
  const mujoco_sim::v2::StreamFrameHeader& header() const { return header_; }
  /// Returns borrowed payload bytes valid only during the callback.
  std::span<const std::byte> payload() const { return payload_; }

  /// Copies the header and payload into a sample safe to retain.
  [[nodiscard]] OwnedStreamSample Copy() const;

 private:
  friend class SensorStreamClient;

  StreamSampleView(mujoco_sim::v2::StreamFrameHeader header,
                   std::span<const std::byte> payload)
      : header_(std::move(header)), payload_(payload) {}

  mujoco_sim::v2::StreamFrameHeader header_;
  std::span<const std::byte> payload_;
};

/// Result of a blocking or polling stream sample wait.
enum class SampleWaitResult {
  /// A sample was written to the output argument.
  kReceived,
  /// The deadline elapsed; output arguments were not changed.
  kTimeout,
  /// Local cancellation ended the wait.
  kStopped,
  /// A terminal stream, lifecycle, or protocol error ended the wait.
  kError,
};

/// Discovers and consumes sensor streams without an Adapter session.
///
/// Each stream permanently selects callback or blocking mode. Different streams
/// may be waited concurrently, but a stream permits only one waiter.
class SensorStreamClient {
 public:
  /// Callback invoked synchronously on a transport receive thread.
  using StreamCallback = std::function<void(const StreamSampleView&)>;

  /// Creates a non-started, single-use sensor stream client.
  explicit SensorStreamClient(SensorStreamOptions options);
  /// Stops delivery and drains callbacks running on other threads.
  ~SensorStreamClient();

  /// Clients are neither copyable nor movable.
  SensorStreamClient(const SensorStreamClient&) = delete;
  /// Clients are neither copyable nor movable.
  SensorStreamClient& operator=(const SensorStreamClient&) = delete;

  /// Discovers descriptors, validates them, and creates receive endpoints.
  bool Start(std::string* error_message);
  /// Selects callback delivery for a stream and discards queued old samples.
  /// Callback exceptions are caught. The client may be destroyed by its own
  /// callback; it must then return without accessing the destroyed object.
  bool SetCallback(const std::string& stream_name, StreamCallback callback,
                   std::string* error_message);
  /// Selects blocking delivery and waits for one sample.
  /// Output is changed only when kReceived is returned.
  SampleWaitResult WaitForSample(const std::string& stream_name,
                                 std::chrono::milliseconds timeout,
                                 OwnedStreamSample* sample,
                                 std::string* error_message);
  /// Cancels local delivery and wakes all waiters. The client is not restartable.
  void Stop();

  /// Returns the validated descriptor for a requested stream.
  std::optional<mujoco_sim::v2::StreamDescriptor> descriptor(
      const std::string& stream_name) const;
  /// Returns transport gaps, Server drops, and local queue evictions.
  std::uint64_t dropped_samples(const std::string& stream_name) const;
  /// Returns the latest non-terminal diagnostic or sticky terminal error.
  std::string stream_diagnostic(const std::string& stream_name) const;

  /// Returns the successful stream discovery response.
  /// Call only after Start() has completed and do not race this access with it.
  const mujoco_sim::v2::GetSensorStreamsResponse& discovery() const;

 private:
  friend class SensorStreamClientTestPeer;
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::adapter_sdk
