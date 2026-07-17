#include "status_tool.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ecal/ecal.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/client.h>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/wrappers.pb.h>
#include <mujoco_sim/protocol/topics.hpp>

#include "status_config_generated.hpp"

namespace mujoco_sim::server::status_tool {
namespace {

constexpr std::size_t kMaximumStatusPayloadSize = 16U * 1024U * 1024U;
constexpr auto kResourceRetryDelay = std::chrono::milliseconds(500);
constexpr auto kInterruptPollInterval = std::chrono::milliseconds(50);
constexpr int kResourceCallTimeoutMs = 100;

volatile std::sig_atomic_t interrupt_requested = 0;

void HandleTerminationSignal(int) {
  interrupt_requested = 1;
}

class SignalHandlerGuard {
 public:
  SignalHandlerGuard() {
    interrupt_requested = 0;
    previous_interrupt_ = std::signal(SIGINT, HandleTerminationSignal);
    previous_terminate_ = std::signal(SIGTERM, HandleTerminationSignal);
  }

  ~SignalHandlerGuard() {
    if (previous_interrupt_ != SIG_ERR) {
      std::signal(SIGINT, previous_interrupt_);
    }
    if (previous_terminate_ != SIG_ERR) {
      std::signal(SIGTERM, previous_terminate_);
    }
  }

 private:
  using Handler = void (*)(int);
  Handler previous_interrupt_ = SIG_DFL;
  Handler previous_terminate_ = SIG_DFL;
};

std::string ServerStateName(mujoco_sim::v4::ServerState state) {
  const std::string name = mujoco_sim::v4::ServerState_Name(state);
  return name.empty() ? "UNKNOWN(" + std::to_string(state) + ")" : name;
}

std::string AdapterStateName(mujoco_sim::v4::AdapterState state) {
  const std::string name = mujoco_sim::v4::AdapterState_Name(state);
  return name.empty() ? "UNKNOWN(" + std::to_string(state) + ")" : name;
}

std::string DisplayMessage(const std::string& message) {
  return message.empty() ? "-" : message;
}

mujoco_sim::v4::ServerStatus NormalizeStatus(const mujoco_sim::v4::ServerStatus& input) {
  auto output = input;
  std::sort(output.mutable_adapters()->begin(), output.mutable_adapters()->end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.role() != rhs.role()) return lhs.role() < rhs.role();
              return lhs.adapter_id() < rhs.adapter_id();
            });
  std::sort(output.mutable_streams()->begin(), output.mutable_streams()->end(),
            [](const auto& lhs, const auto& rhs) { return lhs.name() < rhs.name(); });
  return output;
}

mujoco_sim::v4::GetAvailableResourcesResponse NormalizeResources(
    const mujoco_sim::v4::GetAvailableResourcesResponse& input) {
  auto output = input;
  const auto by_name = [](const auto& lhs, const auto& rhs) {
    return lhs.name() < rhs.name();
  };
  std::sort(output.mutable_observations()->begin(), output.mutable_observations()->end(), by_name);
  std::sort(output.mutable_commands()->begin(), output.mutable_commands()->end(), by_name);
  return output;
}

template <typename Message>
bool MessageToJson(const Message& message, std::string* output, std::string* error_message) {
  google::protobuf::util::JsonPrintOptions options;
  options.always_print_primitive_fields = true;
  options.preserve_proto_field_names = true;
  const auto status = google::protobuf::util::MessageToJsonString(message, output, options);
  if (!status.ok()) {
    *error_message = status.ToString();
    return false;
  }
  return true;
}

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

bool SameProcess(const PublisherIdentity& publisher, const eCAL::SServiceId& service) {
  return !publisher.host_name.empty() && publisher.process_id != 0 &&
         publisher.process_id == service.service_id.process_id &&
         publisher.host_name == service.service_id.host_name;
}

struct ResourceQueryResult {
  std::optional<mujoco_sim::v4::GetAvailableResourcesResponse> response;
  std::string error;
};

ResourceQueryResult QueryResources(eCAL::CServiceClient* client, const std::string&,
                                   const PublisherIdentity& publisher, std::uint64_t epoch) {
  if (client == nullptr || !client->IsConnected()) {
    return {std::nullopt, "registry service is unavailable"};
  }
  mujoco_sim::v4::GetAvailableResourcesRequest request;
  std::string bytes;
  if (!request.SerializeToString(&bytes)) {
    return {std::nullopt, "failed to serialize resource request"};
  }

  eCAL::ServiceResponseVecT responses;
  client->CallWithResponse(mujoco_sim::protocol::kGetAvailableResourcesMethod, bytes, responses,
                           kResourceCallTimeoutMs);
  std::vector<mujoco_sim::v4::GetAvailableResourcesResponse> valid;
  std::string matching_error;
  for (const auto& response : responses) {
    if (!SameProcess(publisher, response.server_id)) continue;
    if (response.call_state != eCAL::eCallState::executed) {
      matching_error =
          response.error_msg.empty() ? "resource service call did not execute" : response.error_msg;
      continue;
    }
    if (response.ret_state != 0) {
      matching_error =
          "resource service returned transport error " + std::to_string(response.ret_state);
      continue;
    }
    mujoco_sim::v4::GetAvailableResourcesResponse parsed;
    if (!parsed.ParseFromString(response.response)) {
      matching_error = "failed to parse resource response";
      continue;
    }
    if (parsed.code() != mujoco_sim::v4::OK) {
      matching_error = parsed.message().empty() ? mujoco_sim::v4::ErrorCode_Name(parsed.code())
                                                : parsed.message();
      continue;
    }
    if (parsed.epoch() != epoch) {
      matching_error = "resource response epoch does not match status";
      continue;
    }
    valid.push_back(std::move(parsed));
  }
  if (valid.size() == 1) return {std::move(valid.front()), {}};
  if (valid.size() > 1) {
    return {std::nullopt, "multiple registry services match the status publisher"};
  }
  return {std::nullopt, matching_error.empty()
                            ? "registry response did not match the status publisher"
                            : std::move(matching_error)};
}

class StatusMonitor {
 public:
  explicit StatusMonitor(StatusOptions options)
      : options_(std::move(options)), state_(std::make_shared<SharedState>(options_)) {}
  ~StatusMonitor() { Stop(); }

  bool Start(std::string* error_message) {
    try {
      eCAL::ServiceMethodInformationSetT methods;
      methods.insert(MethodInfo(mujoco_sim::protocol::kGetAvailableResourcesMethod));
      service_client_ = std::make_unique<eCAL::CServiceClient>(
          mujoco_sim::protocol::RegistryServiceName(options_.server_name), methods);
      subscriber_ = std::make_unique<eCAL::CSubscriber>(
          mujoco_sim::protocol::StatusTopicName(options_.server_name));
      const std::weak_ptr<SharedState> weak_state = state_;
      subscriber_->SetReceiveCallback([weak_state](const eCAL::STopicId& publisher,
                                                   const eCAL::SDataTypeInformation&,
                                                   const eCAL::SReceiveCallbackData& data) {
        const auto state = weak_state.lock();
        if (!state) return;
        mujoco_sim::v4::ServerStatus status;
        if (!ParseStatusPayload(data.buffer, data.buffer_size, state->options.server_name,
                                &status)) {
          return;
        }
        const PublisherIdentity identity{publisher.topic_id.host_name,
                                         publisher.topic_id.process_id,
                                         publisher.topic_id.entity_id};
        std::lock_guard lock(state->mutex);
        const StatusUpdate update = state->tracker.Accept(identity, data.send_clock, status,
                                                          std::chrono::steady_clock::now());
        if (update.publisher_conflict && !state->publisher_conflict_active) {
          state->publisher_conflict_pending = true;
          state->publisher_conflict_active = true;
        }
        if (update.accepted) {
          if (update.publisher_changed) {
            state->publisher_conflict_active = false;
          }
          const bool first = state->tracker.latest()->sequence == 1;
          if (first || update.publisher_changed || update.epoch_changed) {
            ++state->resource_generation;
            state->resources.reset();
            state->resource_error = "resource query pending";
            state->resource_attempts = 0;
            state->resource_query_requested = true;
          }
        }
        state->condition.notify_all();
      });
      resource_thread_ = std::thread([this]() { ResourceLoop(); });
      return true;
    } catch (const std::exception& error) {
      *error_message = "failed to create eCAL status endpoints: " + std::string(error.what());
      Stop();
      return false;
    }
  }

  int Run(std::ostream* output, std::ostream* error_output) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(options_.timeout_ms);
    {
      std::unique_lock lock(state_->mutex);
      while (!state_->stop && !state_->tracker.latest() && interrupt_requested == 0 &&
             std::chrono::steady_clock::now() < deadline) {
        state_->condition.wait_until(
            lock, std::min(deadline, std::chrono::steady_clock::now() + kInterruptPollInterval));
      }
      if (interrupt_requested != 0) return 0;
      if (!state_->tracker.latest()) {
        *error_output << "timed out waiting for status from server '" << options_.server_name
                      << "'\n";
        return 2;
      }
      while (!state_->stop && !state_->resources && state_->resource_attempts < 2 &&
             interrupt_requested == 0 && std::chrono::steady_clock::now() < deadline) {
        state_->condition.wait_until(
            lock, std::min(deadline, std::chrono::steady_clock::now() + kInterruptPollInterval));
      }
      if (interrupt_requested != 0) return 0;
    }

    StatusReport report;
    report.server_name = state_->options.server_name;
    std::uint64_t last_sequence;
    {
      std::lock_guard lock(state_->mutex);
      report = MakeReportLocked();
      last_sequence = state_->tracker.latest()->sequence;
    }
    std::string format_error;
    if (!WriteReport(report, options_.json, output, &format_error)) {
      *error_output << "failed to format status: " << format_error << '\n';
      return 1;
    }
    if (!options_.watch) return 0;

    bool stale_reported = false;
    auto next_output =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(options_.interval_ms);
    while (eCAL::Ok() && interrupt_requested == 0) {
      bool have_report = false;
      bool conflict = false;
      bool became_stale = false;
      bool recovered = false;
      {
        std::unique_lock lock(state_->mutex);
        auto wake_at =
            std::min(next_output, std::chrono::steady_clock::now() + kInterruptPollInterval);
        if (!stale_reported && state_->tracker.latest()) {
          wake_at = std::min(wake_at, state_->tracker.latest()->received_at +
                                          std::chrono::milliseconds(options_.timeout_ms));
        }
        state_->condition.wait_until(lock, wake_at);
        if (state_->stop) break;
        const auto now = std::chrono::steady_clock::now();
        const bool is_stale = state_->tracker.stale(now);
        if (is_stale && !stale_reported) {
          stale_reported = true;
          became_stale = true;
        } else if (!is_stale && stale_reported) {
          stale_reported = false;
          recovered = true;
          next_output = now;
        }
        conflict = std::exchange(state_->publisher_conflict_pending, false);
        if (!is_stale && now >= next_output &&
            state_->tracker.latest()->sequence != last_sequence) {
          report = MakeReportLocked();
          last_sequence = state_->tracker.latest()->sequence;
          have_report = true;
        }
        if (now >= next_output) {
          next_output = now + std::chrono::milliseconds(options_.interval_ms);
        }
      }
      if (conflict) {
        *error_output << "multiple publishers found for server '" << options_.server_name
                      << "'; keeping the current publisher\n";
      }
      if (became_stale) {
        *error_output << "status stream is stale for server '" << options_.server_name << "'\n";
      }
      if (recovered) {
        *error_output << "status stream recovered for server '" << options_.server_name << "'\n";
      }
      error_output->flush();
      if (have_report && !WriteReport(report, options_.json, output, &format_error)) {
        *error_output << "failed to format status: " << format_error << '\n';
        return 1;
      }
    }
    return 0;
  }

  void Stop() {
    if (!state_) return;
    {
      std::lock_guard lock(state_->mutex);
      state_->stop = true;
    }
    state_->condition.notify_all();
    if (subscriber_) {
      subscriber_->RemoveReceiveCallback();
      subscriber_.reset();
    }
    if (resource_thread_.joinable()) resource_thread_.join();
    service_client_.reset();
  }

 private:
  struct SharedState {
    explicit SharedState(const StatusOptions& input)
        : options(input), tracker(input.server_name, std::chrono::milliseconds(input.timeout_ms)) {}
    StatusOptions options;
    StatusTracker tracker;
    std::mutex mutex;
    std::condition_variable condition;
    std::optional<mujoco_sim::v4::GetAvailableResourcesResponse> resources;
    std::string resource_error = "resource query pending";
    std::uint64_t resource_generation = 0;
    std::uint32_t resource_attempts = 0;
    bool resource_query_requested = false;
    bool publisher_conflict_pending = false;
    bool publisher_conflict_active = false;
    bool stop = false;
  };

  StatusReport MakeReportLocked() const {
    StatusReport report;
    report.server_name = state_->options.server_name;
    report.status = state_->tracker.latest()->status;
    report.resources = state_->resources;
    if (!report.resources) report.resource_error = state_->resource_error;
    return report;
  }

  void ResourceLoop() {
    auto next_attempt = std::chrono::steady_clock::time_point::max();
    while (true) {
      PublisherIdentity publisher;
      std::uint64_t epoch;
      std::uint64_t generation;
      {
        std::unique_lock lock(state_->mutex);
        const auto ready = [this]() {
          return state_->stop || state_->resource_query_requested;
        };
        if (next_attempt == std::chrono::steady_clock::time_point::max()) {
          state_->condition.wait(lock, ready);
        } else {
          state_->condition.wait_until(lock, next_attempt, ready);
        }
        if (state_->stop) return;
        if (!state_->tracker.latest()) {
          next_attempt = std::chrono::steady_clock::time_point::max();
          continue;
        }
        publisher = state_->tracker.latest()->publisher;
        epoch = state_->tracker.latest()->status.epoch();
        generation = state_->resource_generation;
        state_->resource_query_requested = false;
      }
      auto query = QueryResources(service_client_.get(), options_.server_name, publisher, epoch);
      {
        std::lock_guard lock(state_->mutex);
        if (state_->stop) return;
        const bool current = state_->tracker.latest() &&
                             state_->resource_generation == generation &&
                             state_->tracker.latest()->publisher == publisher &&
                             state_->tracker.latest()->status.epoch() == epoch;
        if (!current) {
          next_attempt = std::chrono::steady_clock::now();
          continue;
        }
        state_->resources = std::move(query.response);
        state_->resource_error = std::move(query.error);
        ++state_->resource_attempts;
        next_attempt = state_->resources ? std::chrono::steady_clock::time_point::max()
                                         : std::chrono::steady_clock::now() + kResourceRetryDelay;
      }
      state_->condition.notify_all();
    }
  }

  StatusOptions options_;
  std::shared_ptr<SharedState> state_;
  std::unique_ptr<eCAL::CServiceClient> service_client_;
  std::unique_ptr<eCAL::CSubscriber> subscriber_;
  std::thread resource_thread_;
};

}  // namespace

CommandLineResult ParseCommandLine(int argc, char** argv, StatusOptions* options,
                                   std::string* message) {
  if (options == nullptr || message == nullptr) {
    if (message != nullptr) {
      *message = "options and message must not be null";
    }
    return CommandLineResult::kError;
  }
  message->clear();
  generated::StatusConfig parsed;
  parsed.server_name = options->server_name;
  parsed.timeout_ms = options->timeout_ms;
  parsed.interval_ms = options->interval_ms;
  parsed.watch = options->watch;
  parsed.json = options->json;
  std::vector<const char*> arguments;
  arguments.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    arguments.push_back(argv[i]);
  }
  if (!generated::Parse(argc, arguments.data(), &parsed, message)) {
    return *message == generated::Usage() ? CommandLineResult::kHelp : CommandLineResult::kError;
  }
  if (parsed.server_name.empty()) {
    *message = "server name is required";
    return CommandLineResult::kError;
  }
  if (parsed.timeout_ms == 0) {
    *message = "--timeout-ms must be greater than zero";
    return CommandLineResult::kError;
  }
  if (parsed.interval_ms == 0) {
    *message = "--interval-ms must be greater than zero";
    return CommandLineResult::kError;
  }
  options->server_name = std::move(parsed.server_name);
  options->timeout_ms = parsed.timeout_ms;
  options->interval_ms = parsed.interval_ms;
  options->watch = parsed.watch;
  options->json = parsed.json;
  return CommandLineResult::kSuccess;
}

std::string Usage() {
  return generated::Usage();
}

StatusTracker::StatusTracker(std::string server_name, std::chrono::milliseconds stale_after)
    : server_name_(std::move(server_name)), stale_after_(stale_after) {}

StatusUpdate StatusTracker::Accept(const PublisherIdentity& publisher, std::int64_t send_clock,
                                   const mujoco_sim::v4::ServerStatus& status,
                                   std::chrono::steady_clock::time_point received_at) {
  StatusUpdate update;
  if (latest_ && latest_->publisher != publisher) {
    if (!stale(received_at)) {
      update.publisher_conflict = true;
      return update;
    }
    update.publisher_changed = true;
  } else if (latest_ && send_clock > 0 && latest_->send_clock > 0 &&
             send_clock <= latest_->send_clock) {
    return update;
  }
  if (latest_ && latest_->status.epoch() != status.epoch()) {
    update.epoch_changed = true;
  }
  latest_ = ReceivedStatus{status, publisher, send_clock, received_at, next_sequence_++};
  update.accepted = true;
  return update;
}

bool StatusTracker::stale(std::chrono::steady_clock::time_point now) const {
  return latest_ && now - latest_->received_at >= stale_after_;
}

bool ParseStatusPayload(const void* data, std::size_t size, const std::string&,
                        mujoco_sim::v4::ServerStatus* status) {
  if (data == nullptr || status == nullptr || size == 0 || size > kMaximumStatusPayloadSize ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  mujoco_sim::v4::ServerStatus parsed;
  if (!parsed.ParseFromArray(data, static_cast<int>(size))) {
    return false;
  }
  *status = std::move(parsed);
  return true;
}

std::string FormatHuman(const StatusReport& report) {
  const auto status = NormalizeStatus(report.status);
  std::ostringstream output;
  output << "Server: " << report.server_name << '\n'
         << "State: " << ServerStateName(status.state()) << '\n'
         << "Epoch: " << status.epoch() << '\n'
         << "Tick: " << status.tick_id() << '\n'
         << "Protocol: " << mujoco_sim::protocol::kProtocolMajor << '.'
         << mujoco_sim::protocol::kProtocolMinor << '\n'
         << "Simulation time: " << status.simulation_time_s() << " s\n"
         << "Message: " << DisplayMessage(status.message()) << '\n';
  if (!report.resources) {
    output << "Resources: unavailable (" << DisplayMessage(report.resource_error) << ")\n";
  } else {
    const auto resources = NormalizeResources(*report.resources);
    output << "Observations (" << resources.observations_size() << "): ";
    for (int i = 0; i < resources.observations_size(); ++i) {
      if (i) output << ", ";
      output << resources.observations(i).name() << '[' << resources.observations(i).dimension()
             << ']';
    }
    output << "\nCommands (" << resources.commands_size() << "): ";
    for (int i = 0; i < resources.commands_size(); ++i) {
      if (i) output << ", ";
      output << resources.commands(i).name() << '[' << resources.commands(i).dimension() << ']';
    }
    output << '\n';
  }
  output << "Adapters (" << status.adapters_size() << "):\n"
         << "ID\tRole\tState\tSession\tLast delivery\tCommand sequence"
            "\tState sequence\tState since\tTimeouts\tMessage\n";
  for (const auto& adapter : status.adapters()) {
    output << adapter.adapter_id() << '\t' << mujoco_sim::v4::AdapterRole_Name(adapter.role())
           << '\t' << AdapterStateName(adapter.state()) << '\t' << adapter.session_id() << '\t'
           << (adapter.has_last_delivery_tick_id() ? std::to_string(adapter.last_delivery_tick_id())
                                                   : "-")
           << '\t'
           << (adapter.role() == mujoco_sim::v4::CONTROLLER
                   ? std::to_string(adapter.last_command_sequence())
                   : "-")
           << '\t'
           << (adapter.role() == mujoco_sim::v4::OBSERVER
                   ? std::to_string(adapter.last_observation_sequence())
                   : "-")
           << '\t' << adapter.state_since_tick_id() << '\t' << adapter.timeout_count() << '\t'
           << DisplayMessage(adapter.message()) << '\n';
  }
  output << "Sensor streams (" << status.streams_size() << "):\n"
         << "Name\tState\tSubscribers\tCaptures\tPublished\tDropped"
            "\tErrors\tBlocked ms\tMessage\n";
  for (const auto& stream : status.streams()) {
    output << stream.name() << '\t' << mujoco_sim::v4::StreamState_Name(stream.state()) << '\t'
           << stream.subscriber_count() << '\t' << stream.captures() << '\t' << stream.published()
           << '\t' << stream.dropped() << '\t' << stream.publish_errors() << '\t'
           << stream.blocked_time_ms() << '\t' << DisplayMessage(stream.message()) << '\n';
  }
  return output.str();
}

bool FormatJson(const StatusReport& report, std::string* output, std::string* error_message) {
  google::protobuf::StringValue server_name;
  server_name.set_value(report.server_name);
  std::string server_name_json;
  if (!MessageToJson(server_name, &server_name_json, error_message)) {
    return false;
  }
  std::string status_json;
  const auto status = NormalizeStatus(report.status);
  if (!MessageToJson(status, &status_json, error_message)) return false;
  std::string resources_json;
  if (report.resources) {
    const auto resources = NormalizeResources(*report.resources);
    if (!MessageToJson(resources, &resources_json, error_message)) return false;
  } else {
    resources_json = "null";
  }
  std::string error_json;
  if (!report.resource_error.empty()) {
    google::protobuf::StringValue error;
    error.set_value(report.resource_error);
    if (!MessageToJson(error, &error_json, error_message)) return false;
  } else {
    error_json = "null";
  }
  *output = "{\"server_name\":" + server_name_json + ",\"status\":" + status_json +
            ",\"resources\":" + resources_json + ",\"resource_error\":" + error_json + "}";
  return true;
}

bool WriteReport(const StatusReport& report, bool json, std::ostream* output,
                 std::string* error_message) {
  if (json) {
    std::string encoded;
    if (!FormatJson(report, &encoded, error_message)) return false;
    *output << encoded << '\n';
  } else {
    *output << FormatHuman(report) << '\n';
  }
  output->flush();
  return output->good();
}

int RunStatusTool(const StatusOptions& options, std::ostream* output, std::ostream* error_output) {
  bool owns_ecal = false;
  if (!eCAL::IsInitialized()) {
    owns_ecal = eCAL::Initialize("mujoco-sim-status-" + options.server_name);
    if (!owns_ecal) {
      *error_output << "failed to initialize eCAL\n";
      return 1;
    }
  }
  int result = 1;
  {
    SignalHandlerGuard signal_handlers;
    {
      StatusMonitor monitor(options);
      std::string error;
      if (monitor.Start(&error)) {
        result = monitor.Run(output, error_output);
      } else {
        *error_output << error << '\n';
      }
      monitor.Stop();
    }
  }
  if (owns_ecal) eCAL::Finalize();
  return result;
}

}  // namespace mujoco_sim::server::status_tool
