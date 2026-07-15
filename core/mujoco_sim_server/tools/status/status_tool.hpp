#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>

#include <mujoco_sim/v3/sync.pb.h>

namespace mujoco_sim::server::status_tool {

struct StatusOptions {
  std::string server_name = "default";
  std::uint32_t timeout_ms = 3000;
  std::uint32_t interval_ms = 1000;
  bool watch = false;
  bool json = false;
};

enum class CommandLineResult { kSuccess, kHelp, kError };

CommandLineResult ParseCommandLine(int argc, char** argv,
                                   StatusOptions* options,
                                   std::string* message);
std::string Usage();

struct PublisherIdentity {
  std::string host_name;
  std::int32_t process_id = 0;
  std::uint64_t entity_id = 0;

  bool operator==(const PublisherIdentity&) const = default;
};

struct ReceivedStatus {
  mujoco_sim::v3::ServerStatus status;
  PublisherIdentity publisher;
  std::int64_t send_clock = 0;
  std::chrono::steady_clock::time_point received_at;
  std::uint64_t sequence = 0;
};

struct StatusUpdate {
  bool accepted = false;
  bool publisher_changed = false;
  bool epoch_changed = false;
  bool publisher_conflict = false;
};

class StatusTracker {
 public:
  StatusTracker(std::string server_name,
                std::chrono::milliseconds stale_after);
  StatusUpdate Accept(const PublisherIdentity& publisher,
                      std::int64_t send_clock,
                      const mujoco_sim::v3::ServerStatus& status,
                      std::chrono::steady_clock::time_point received_at);
  bool stale(std::chrono::steady_clock::time_point now) const;
  const std::optional<ReceivedStatus>& latest() const { return latest_; }

 private:
  std::string server_name_;
  std::chrono::milliseconds stale_after_;
  std::optional<ReceivedStatus> latest_;
  std::uint64_t next_sequence_ = 1;
};

bool ParseStatusPayload(const void* data, std::size_t size,
                        const std::string& server_name,
                        mujoco_sim::v3::ServerStatus* status);

struct StatusReport {
  std::string server_name;
  mujoco_sim::v3::ServerStatus status;
  std::optional<mujoco_sim::v3::GetAvailableResourcesResponse> resources;
  std::string resource_error;
};

std::string FormatHuman(const StatusReport& report);
bool FormatJson(const StatusReport& report, std::string* output,
                std::string* error_message);
bool WriteReport(const StatusReport& report, bool json, std::ostream* output,
                 std::string* error_message);
int RunStatusTool(const StatusOptions& options, std::ostream* output,
                  std::ostream* error_output);

}  // namespace mujoco_sim::server::status_tool
