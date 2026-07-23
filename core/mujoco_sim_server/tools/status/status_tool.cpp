#include "status_tool.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <google/protobuf/util/json_util.h>
#include <google/protobuf/wrappers.pb.h>
#include <mujoco_sim/protocol/topics.hpp>

#include "status_config_generated.hpp"

namespace mujoco_sim::server::status_tool {
namespace {

constexpr std::size_t kMaximumStatusPayloadSize = 16U * 1024U * 1024U;

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
  if (output->good()) return true;
  *error_message = "output stream rejected the status report";
  return false;
}

}  // namespace mujoco_sim::server::status_tool
