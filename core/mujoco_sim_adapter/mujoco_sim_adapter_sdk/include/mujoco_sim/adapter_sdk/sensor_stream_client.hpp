#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ecal/pubsub/subscriber.h>
#include <ecal/service/client.h>
#include <mujoco_sim/protocol/stream_frame.hpp>
#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::adapter_sdk {

struct SensorStreamOptions {
  std::string server_name = "default";
  std::vector<std::string> stream_names;
  std::uint32_t service_timeout_ms = 5000;
  std::size_t queue_capacity = 1;
};

struct OwnedStreamSample {
  mujoco_sim::v2::StreamFrameHeader header;
  std::vector<std::byte> payload;
};

// The payload is borrowed from the transport and is valid only while the
// StreamCallback is running. Use Copy() to retain a sample.
class StreamSampleView {
 public:
  StreamSampleView(const StreamSampleView&) = delete;
  StreamSampleView& operator=(const StreamSampleView&) = delete;

  const mujoco_sim::v2::StreamFrameHeader& header() const { return header_; }
  std::span<const std::byte> payload() const { return payload_; }

  [[nodiscard]] OwnedStreamSample Copy() const;

 private:
  friend class SensorStreamClient;

  StreamSampleView(mujoco_sim::v2::StreamFrameHeader header,
                   std::span<const std::byte> payload)
      : header_(std::move(header)), payload_(payload) {}

  mujoco_sim::v2::StreamFrameHeader header_;
  std::span<const std::byte> payload_;
};

enum class SampleWaitResult {
  kReceived,
  kTimeout,
  kStopped,
  kError,
};

class SensorStreamClient {
 public:
  using StreamCallback = std::function<void(const StreamSampleView&)>;

  explicit SensorStreamClient(SensorStreamOptions options);
  ~SensorStreamClient();

  SensorStreamClient(const SensorStreamClient&) = delete;
  SensorStreamClient& operator=(const SensorStreamClient&) = delete;

  bool Start(std::string* error_message);
  bool SetCallback(const std::string& stream_name, StreamCallback callback,
                   std::string* error_message);
  SampleWaitResult WaitForSample(const std::string& stream_name,
                                 std::chrono::milliseconds timeout,
                                 OwnedStreamSample* sample,
                                 std::string* error_message);
  void Stop();

  std::optional<mujoco_sim::v2::StreamDescriptor> descriptor(
      const std::string& stream_name) const;
  std::uint64_t dropped_samples(const std::string& stream_name) const;
  std::string stream_diagnostic(const std::string& stream_name) const;

  const mujoco_sim::v2::GetSensorStreamsResponse& discovery() const {
    return discovery_;
  }

 private:
  friend class SensorStreamClientTestPeer;

  enum class DeliveryMode {
    kUnset,
    kCallback,
    kWait,
  };

  struct StreamState {
    std::mutex delivery_mutex;
    mujoco_sim::v2::StreamDescriptor descriptor;
    DeliveryMode mode = DeliveryMode::kUnset;
    StreamCallback callback;
    std::deque<OwnedStreamSample> samples;
    std::uint64_t last_sequence = 0;
    std::uint64_t dropped_samples = 0;
    bool wait_in_progress = false;
    std::string diagnostic;
    std::string terminal_error;
  };

  template <typename Request, typename Response>
  bool CallService(const std::string& method, const Request& request,
                   Response* response, std::string* error_message);

  bool ValidateOptions(std::string* error_message) const;
  bool DiscoverStreams(std::string* error_message);
  bool ConfigureDiscoveredStreams(std::string* error_message);
  bool ValidateDescriptor(const mujoco_sim::v2::StreamDescriptor& descriptor,
                          std::string* error_message) const;
  bool IsExpectedFrame(
      const std::string& stream_name,
      const mujoco_sim::protocol::ParsedStreamFrame& frame,
      std::string* error_message) const;
  void HandleStreamBytes(const std::string& stream_name, const void* data,
                         std::size_t size);
  void HandleStatusBytes(const void* data, std::size_t size);
  void HandleStreamFrame(
      const std::string& stream_name,
      const mujoco_sim::protocol::ParsedStreamFrame& frame);
  void RecordDiagnostic(const std::string& stream_name,
                        const std::string& diagnostic);
  void StopSampleReception();

  struct CallbackState {
    std::mutex mutex;
    std::condition_variable condition;
    SensorStreamClient* client = nullptr;
    bool enabled = false;
    std::size_t active_callbacks = 0;
    std::unordered_map<std::thread::id, std::size_t> callbacks_by_thread;
  };

  class CallbackLease {
   public:
    explicit CallbackLease(std::shared_ptr<CallbackState> state);
    ~CallbackLease();

    SensorStreamClient* client() const { return client_; }

   private:
    std::shared_ptr<CallbackState> state_;
    SensorStreamClient* client_ = nullptr;
  };

  SensorStreamOptions options_;
  mujoco_sim::v2::GetSensorStreamsResponse discovery_;
  std::unique_ptr<eCAL::CServiceClient> registry_client_;
  std::unordered_map<std::string, std::unique_ptr<eCAL::CSubscriber>>
      subscribers_;
  std::unique_ptr<eCAL::CSubscriber> status_subscriber_;
  std::unordered_map<std::string, std::shared_ptr<StreamState>> streams_;
  std::optional<std::uint64_t> last_status_tick_id_;
  std::shared_ptr<CallbackState> callback_state_;
  std::mutex lifecycle_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<void> ecal_runtime_;
  bool monitor_ecal_ = false;
  bool started_ = false;
  bool accepting_samples_ = false;
};

}  // namespace mujoco_sim::adapter_sdk
