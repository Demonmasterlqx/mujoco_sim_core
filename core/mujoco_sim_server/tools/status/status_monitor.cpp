#include "status_tool.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <ecal/ecal.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/client.h>
#include <mujoco_sim/protocol/topics.hpp>

#include "status_monitor_support.hpp"

namespace mujoco_sim::server::status_tool {
namespace {

constexpr auto kResourceRetryDelay = std::chrono::milliseconds(500);
constexpr auto kInterruptPollInterval = std::chrono::milliseconds(50);

using internal::QueryResources;
using internal::StatusCallbackGate;
using internal::StatusCallbackLease;

volatile std::sig_atomic_t interrupt_requested = 0;

void HandleTerminationSignal(int) {
  interrupt_requested = 1;
}

class SignalHandlerGuard {
 public:
  SignalHandlerGuard() {
    interrupt_requested = 0;
    previous_interrupt_ = std::signal(SIGINT, HandleTerminationSignal);
    if (previous_interrupt_ == SIG_ERR) {
      throw std::runtime_error("failed to install SIGINT handler for the status tool");
    }
    previous_terminate_ = std::signal(SIGTERM, HandleTerminationSignal);
    if (previous_terminate_ == SIG_ERR) {
      const bool restored = std::signal(SIGINT, previous_interrupt_) != SIG_ERR;
      throw std::runtime_error(
          restored ? "failed to install SIGTERM handler for the status tool"
                   : "failed to install SIGTERM handler and restore the SIGINT handler for the "
                     "status tool");
    }
  }

  ~SignalHandlerGuard() noexcept {
    if (previous_interrupt_ != SIG_ERR && std::signal(SIGINT, previous_interrupt_) == SIG_ERR) {
      std::fputs("status tool failed to restore the SIGINT handler\n", stderr);
    }
    if (previous_terminate_ != SIG_ERR && std::signal(SIGTERM, previous_terminate_) == SIG_ERR) {
      std::fputs("status tool failed to restore the SIGTERM handler\n", stderr);
    }
  }

 private:
  using Handler = void (*)(int);
  Handler previous_interrupt_ = SIG_DFL;
  Handler previous_terminate_ = SIG_DFL;
};

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

class StatusMonitor {
 public:
  explicit StatusMonitor(StatusOptions options)
      : options_(std::move(options)),
        state_(std::make_shared<SharedState>(options_)),
        callback_gate_(std::make_shared<StatusCallbackGate>()) {}
  ~StatusMonitor() noexcept { Stop(nullptr); }

  bool Start(std::string* error_message) {
    if (error_message == nullptr) return false;
    try {
      eCAL::ServiceMethodInformationSetT methods;
      methods.insert(MethodInfo(mujoco_sim::protocol::kGetAvailableResourcesMethod));
      service_client_ = std::make_unique<eCAL::CServiceClient>(
          mujoco_sim::protocol::RegistryServiceName(options_.server_name), methods);
      subscriber_ = std::make_unique<eCAL::CSubscriber>(
          mujoco_sim::protocol::StatusTopicName(options_.server_name));
      const std::weak_ptr<SharedState> weak_state = state_;
      const auto callback_gate = callback_gate_;
      subscriber_->SetReceiveCallback(
          [weak_state, callback_gate](const eCAL::STopicId& publisher,
                                      const eCAL::SDataTypeInformation&,
                                      const eCAL::SReceiveCallbackData& data) noexcept {
            const auto state = weak_state.lock();
            if (!state) return;
            try {
              StatusCallbackLease lease(callback_gate);
              if (!lease) return;
              HandleStatusUpdate(state, publisher, data);
            } catch (const std::exception& error) {
              RecordTerminalError(state, "status callback failed", error.what());
            } catch (...) {
              RecordTerminalError(state, "status callback failed", "unknown exception");
            }
          });
      resource_thread_ = std::thread([this]() noexcept { RunResourceLoop(); });
      return true;
    } catch (const std::exception& error) {
      *error_message = "failed to create eCAL status endpoints: " + std::string(error.what());
      AppendStopError(error_message);
    } catch (...) {
      *error_message = "failed to create eCAL status endpoints: unknown exception";
      AppendStopError(error_message);
    }
    return false;
  }

  int Run(std::ostream* output, std::ostream* error_output) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(options_.timeout_ms);
    bool timed_out = false;
    std::string monitor_error;
    {
      std::unique_lock lock(state_->mutex);
      while (!state_->stop && !state_->tracker.latest() && interrupt_requested == 0 &&
             std::chrono::steady_clock::now() < deadline) {
        state_->condition.wait_until(
            lock, std::min(deadline, std::chrono::steady_clock::now() + kInterruptPollInterval));
      }
      if (interrupt_requested != 0) return 0;
      timed_out = !state_->tracker.latest();
      if (!timed_out) {
        while (!state_->stop && !state_->resources && state_->resource_attempts < 2 &&
               interrupt_requested == 0 && std::chrono::steady_clock::now() < deadline) {
          state_->condition.wait_until(
              lock, std::min(deadline, std::chrono::steady_clock::now() + kInterruptPollInterval));
        }
      }
      if (interrupt_requested != 0) return 0;
      monitor_error = state_->terminal_error;
    }
    if (!monitor_error.empty()) {
      *error_output << monitor_error << '\n';
      return 1;
    }
    if (timed_out) {
      *error_output << "timed out waiting for status from server '" << options_.server_name
                    << "'\n";
      return 2;
    }

    StatusReport report;
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
        if (state_->stop) {
          monitor_error = state_->terminal_error;
          break;
        }
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
    if (!monitor_error.empty()) {
      *error_output << monitor_error << '\n';
      return 1;
    }
    return 0;
  }

  bool Stop(std::string* error_message) noexcept {
    if (!state_) return true;
    bool stopped_cleanly = true;
    RunStopStage("failed to stop accepting status callbacks", error_message, &stopped_cleanly,
                 [this] { callback_gate_->StopAccepting(); });
    RunStopStage("failed to request status monitor stop", error_message, &stopped_cleanly, [this] {
      std::lock_guard lock(state_->mutex);
      state_->stop = true;
    });
    state_->condition.notify_all();
    if (subscriber_) {
      RunStopStage("failed to remove status callback", error_message, &stopped_cleanly,
                   [this] { subscriber_->RemoveReceiveCallback(); });
    }
    RunStopStage("failed to drain status callbacks", error_message, &stopped_cleanly,
                 [this] { callback_gate_->WaitForIdle(); });
    RunStopStage("failed to destroy status subscriber", error_message, &stopped_cleanly,
                 [this] { subscriber_.reset(); });
    if (resource_thread_.joinable()) {
      RunStopStage("failed to join resource query thread", error_message, &stopped_cleanly,
                   [this] { resource_thread_.join(); });
    }
    RunStopStage("failed to destroy registry service client", error_message, &stopped_cleanly,
                 [this] { service_client_.reset(); });
    return stopped_cleanly;
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
    std::string terminal_error;
  };

  static void HandleStatusUpdate(const std::shared_ptr<SharedState>& state,
                                 const eCAL::STopicId& publisher,
                                 const eCAL::SReceiveCallbackData& data) {
    mujoco_sim::v4::ServerStatus status;
    if (!ParseStatusPayload(data.buffer, data.buffer_size, state->options.server_name, &status)) {
      return;
    }
    const PublisherIdentity identity{publisher.topic_id.host_name, publisher.topic_id.process_id,
                                     publisher.topic_id.entity_id};
    std::lock_guard lock(state->mutex);
    if (state->stop) return;
    const StatusUpdate update =
        state->tracker.Accept(identity, data.send_clock, status, std::chrono::steady_clock::now());
    if (update.publisher_conflict && !state->publisher_conflict_active) {
      state->publisher_conflict_pending = true;
      state->publisher_conflict_active = true;
    }
    if (update.accepted) HandleAcceptedStatus(state.get(), update);
    state->condition.notify_all();
  }

  static void HandleAcceptedStatus(SharedState* state, const StatusUpdate& update) {
    if (update.publisher_changed) state->publisher_conflict_active = false;
    const bool first = state->tracker.latest()->sequence == 1;
    if (!first && !update.publisher_changed && !update.epoch_changed) return;
    ++state->resource_generation;
    state->resources.reset();
    state->resource_error = "resource query pending";
    state->resource_attempts = 0;
    state->resource_query_requested = true;
  }

  static void RecordTerminalError(const std::shared_ptr<SharedState>& state, const char* context,
                                  const char* message) noexcept {
    try {
      std::lock_guard lock(state->mutex);
      if (state->terminal_error.empty()) {
        state->terminal_error = std::string(context) + ": " + message;
      }
      state->stop = true;
    } catch (...) {
      try {
        std::lock_guard lock(state->mutex);
        state->stop = true;
      } catch (...) {
      }
    }
    state->condition.notify_all();
  }

  static void RecordStopError(std::string* error_message, const char* context,
                              const char* message) noexcept {
    if (!error_message) return;
    try {
      const std::string detail = std::string(context) + ": " + message;
      if (error_message->empty()) {
        *error_message = detail;
      } else {
        *error_message += "; " + detail;
      }
    } catch (...) {
      try {
        if (error_message->empty()) *error_message = "status monitor cleanup failed";
      } catch (...) {
      }
    }
  }

  template <typename Action>
  void RunStopStage(const char* context, std::string* error_message, bool* stopped_cleanly,
                    Action&& action) noexcept {
    try {
      std::forward<Action>(action)();
    } catch (const std::exception& error) {
      RecordTerminalError(state_, context, error.what());
      RecordStopError(error_message, context, error.what());
      *stopped_cleanly = false;
    } catch (...) {
      RecordTerminalError(state_, context, "unknown exception");
      RecordStopError(error_message, context, "unknown exception");
      *stopped_cleanly = false;
    }
  }

  void AppendStopError(std::string* error_message) noexcept {
    std::string stop_error;
    if (Stop(&stop_error) || stop_error.empty()) return;
    try {
      *error_message += "; " + stop_error;
    } catch (...) {
    }
  }

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
        const auto ready = [this] {
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
      auto query = QueryResources(service_client_.get(), publisher, epoch);
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

  void RunResourceLoop() noexcept {
    try {
      ResourceLoop();
    } catch (const std::exception& error) {
      RecordTerminalError(state_, "resource query thread failed", error.what());
    } catch (...) {
      RecordTerminalError(state_, "resource query thread failed", "unknown exception");
    }
  }

  StatusOptions options_;
  std::shared_ptr<SharedState> state_;
  std::shared_ptr<StatusCallbackGate> callback_gate_;
  std::unique_ptr<eCAL::CServiceClient> service_client_;
  std::unique_ptr<eCAL::CSubscriber> subscriber_;
  std::thread resource_thread_;
};

}  // namespace

int RunStatusTool(const StatusOptions& options, std::ostream* output, std::ostream* error_output) {
  bool owns_ecal = false;
  int result = 1;
  try {
    if (!eCAL::IsInitialized()) {
      owns_ecal = eCAL::Initialize("mujoco-sim-status-" + options.server_name);
      if (!owns_ecal) {
        *error_output << "failed to initialize eCAL\n";
        return 1;
      }
    }
    {
      SignalHandlerGuard signal_handlers;
      StatusMonitor monitor(options);
      std::string error;
      if (monitor.Start(&error)) {
        result = monitor.Run(output, error_output);
      } else {
        *error_output << error << '\n';
      }
      if (!monitor.Stop(&error)) {
        *error_output << error << '\n';
        result = 1;
      }
    }
  } catch (const std::exception& error) {
    *error_output << "status tool failed: " << error.what() << '\n';
    result = 1;
  } catch (...) {
    *error_output << "status tool failed: unknown exception\n";
    result = 1;
  }
  if (owns_ecal) {
    try {
      if (!eCAL::Finalize()) {
        *error_output << "failed to finalize eCAL for the status tool\n";
        result = 1;
      }
    } catch (const std::exception& error) {
      *error_output << "eCAL finalization for the status tool threw: " << error.what() << '\n';
      result = 1;
    } catch (...) {
      *error_output << "eCAL finalization for the status tool threw: unknown exception\n";
      result = 1;
    }
  }
  return result;
}

}  // namespace mujoco_sim::server::status_tool
