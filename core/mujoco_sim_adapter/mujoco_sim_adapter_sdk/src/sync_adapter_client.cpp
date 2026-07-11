#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::adapter_sdk {
namespace {

using mujoco_sim::v1::COMMIT_OK;
using mujoco_sim::v1::OK;

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

bool SameTickStream(const mujoco_sim::v1::Tick& lhs,
                    const mujoco_sim::v1::Tick& rhs) {
  return lhs.controller_id() == rhs.controller_id() &&
         lhs.header().server_name() == rhs.header().server_name() &&
         lhs.header().session_id() == rhs.header().session_id() &&
         lhs.header().epoch() == rhs.header().epoch() &&
         lhs.header().protocol_major() == rhs.header().protocol_major() &&
         lhs.header().protocol_minor() == rhs.header().protocol_minor();
}

bool SameTickIdentity(const mujoco_sim::v1::Tick& lhs,
                      const mujoco_sim::v1::Tick& rhs) {
  return SameTickStream(lhs, rhs) &&
         lhs.header().tick_id() == rhs.header().tick_id();
}

bool SameTickPayload(const mujoco_sim::v1::Tick& lhs,
                     const mujoco_sim::v1::Tick& rhs) {
  auto normalized_lhs = lhs;
  auto normalized_rhs = rhs;
  normalized_lhs.mutable_header()->set_timestamp_unix_nanos(0);
  normalized_rhs.mutable_header()->set_timestamp_unix_nanos(0);
  return normalized_lhs.SerializeAsString() ==
         normalized_rhs.SerializeAsString();
}

mujoco_sim::v1::Heartbeat BuildHeartbeat(
    const mujoco_sim::v1::Tick& tick) {
  mujoco_sim::v1::Heartbeat heartbeat;
  *heartbeat.mutable_header() = tick.header();
  heartbeat.set_controller_id(tick.controller_id());
  return heartbeat;
}

}  // namespace

SyncAdapterClient::SyncAdapterClient(AdapterOptions options)
    : options_(std::move(options)),
      callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

SyncAdapterClient::~SyncAdapterClient() {
  Stop();
  StopTickReception();
  std::string ignored;
  Unregister(&ignored);
  {
    std::lock_guard publish_lock(publisher_mutex_);
    commit_publisher_.reset();
    heartbeat_publisher_.reset();
  }
  registry_client_.reset();
  if (owns_ecal_) {
    eCAL::Finalize();
  }
}

bool SyncAdapterClient::Start(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (started_) {
      *error_message = "adapter client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error_message = "adapter client has been stopped";
    return false;
  }
  if (options_.controller_id.empty()) {
    *error_message = "controller_id is required";
    return false;
  }
  if (options_.command_names.empty()) {
    *error_message = "at least one command actuator is required";
    return false;
  }
  if (!eCAL::IsInitialized()) {
    owns_ecal_ = eCAL::Initialize("mujoco-sim-adapter-" +
                                  options_.controller_id);
    if (!owns_ecal_) {
      *error_message = "failed to initialize eCAL";
      return false;
    }
  }

  eCAL::ServiceMethodInformationSetT methods;
  methods.insert(MethodInfo(mujoco_sim::protocol::kGetAvailableResourcesMethod));
  methods.insert(MethodInfo(mujoco_sim::protocol::kRegisterControllerMethod));
  methods.insert(MethodInfo(mujoco_sim::protocol::kActivateControllerMethod));
  methods.insert(MethodInfo(mujoco_sim::protocol::kUnregisterControllerMethod));
  registry_client_ = std::make_unique<eCAL::CServiceClient>(
      mujoco_sim::protocol::RegistryServiceName(options_.server_name), methods);
  if (!Register(error_message)) {
    return false;
  }

  commit_publisher_ = std::make_unique<eCAL::CPublisher>(
      mujoco_sim::protocol::CommitTopicName(options_.server_name));
  heartbeat_publisher_ = std::make_unique<eCAL::CPublisher>(
      mujoco_sim::protocol::HeartbeatTopicName(options_.server_name));
  {
    std::lock_guard callback_lock(callback_state_->mutex);
    callback_state_->client = this;
    callback_state_->enabled = true;
  }
  const auto callback_state = callback_state_;
  tick_subscriber_ = std::make_unique<eCAL::CSubscriber>(
      mujoco_sim::protocol::TickTopicName(options_.server_name,
                                          options_.controller_id));
  tick_subscriber_->SetReceiveCallback(
      [callback_state](const eCAL::STopicId&,
                       const eCAL::SDataTypeInformation&,
                       const eCAL::SReceiveCallbackData& data) {
        CallbackLease callback(callback_state);
        if (callback.client() != nullptr) {
          callback.client()->HandleTickBytes(data.buffer, data.buffer_size);
        }
      });
  status_subscriber_ = std::make_unique<eCAL::CSubscriber>(
      mujoco_sim::protocol::StatusTopicName(options_.server_name));
  status_subscriber_->SetReceiveCallback(
      [callback_state](const eCAL::STopicId&,
                       const eCAL::SDataTypeInformation&,
                       const eCAL::SReceiveCallbackData& data) {
        CallbackLease callback(callback_state);
        if (callback.client() != nullptr) {
          callback.client()->HandleStatusBytes(data.buffer, data.buffer_size);
        }
      });

  return FinishStart(
      [this] {
        std::string ignored;
        UnregisterWith(
            [this](
                const mujoco_sim::v1::UnregisterControllerRequest& request,
                mujoco_sim::v1::UnregisterControllerResponse* response,
                std::string* unregister_error) {
              return CallService(
                  mujoco_sim::protocol::kUnregisterControllerMethod, request,
                  response, unregister_error, true);
            },
            &ignored);
      },
      error_message);
}

bool SyncAdapterClient::FinishStart(
    const std::function<void()>& unregister,
    std::string* error_message) {
  bool cancelled = false;
  {
    std::lock_guard lock(mutex_);
    cancelled = stop_requested_;
    if (!cancelled) {
      monitor_ecal_ = true;
      started_ = true;
    }
  }
  if (cancelled) {
    StopTickReception();
    unregister();
    *error_message = "adapter start was cancelled";
    return false;
  }
  return true;
}

bool SyncAdapterClient::Register(std::string* error_message) {
  mujoco_sim::v1::RegisterControllerRequest request;
  request.set_server_name(options_.server_name);
  request.set_controller_id(options_.controller_id);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.set_timeout_ms(options_.timeout_ms);
  for (const auto& name : options_.observation_names) {
    request.add_observation_names(name);
  }
  for (const auto& name : options_.command_names) {
    request.add_command_names(name);
  }

  if (!CallService(mujoco_sim::protocol::kRegisterControllerMethod, request,
                   &registration_, error_message)) {
    return false;
  }
  if (registration_.code() != OK) {
    *error_message = registration_.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    queued_tick_.reset();
    outstanding_tick_.reset();
    committed_tick_.reset();
    cached_commit_.reset();
    terminal_error_.clear();
    lifecycle_warning_.clear();
    registered_ = true;
  }
  return true;
}

bool SyncAdapterClient::Activate(std::string* error_message) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  return ActivateWith(
      [this](const mujoco_sim::v1::ActivateControllerRequest& request,
             mujoco_sim::v1::ActivateControllerResponse* response,
             std::string* error) {
        return CallService(mujoco_sim::protocol::kActivateControllerMethod,
                           request, response, error);
      },
      std::chrono::milliseconds(100), error_message);
}

bool SyncAdapterClient::ActivateWith(
    const std::function<bool(
        const mujoco_sim::v1::ActivateControllerRequest&,
        mujoco_sim::v1::ActivateControllerResponse*, std::string*)>& call,
    std::chrono::milliseconds discovery_delay,
    std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (!started_ || !registered_) {
      *error_message = "adapter client must be started before activation";
      return false;
    }
    if (activated_) {
      *error_message = "adapter client has already been activated";
      return false;
    }
    if (stop_requested_) {
      *error_message = "adapter client has been stopped";
      return false;
    }
  }
  if (!WaitBeforeActivation(discovery_delay)) {
    *error_message = "adapter activation was cancelled";
    return false;
  }

  mujoco_sim::v1::ActivateControllerRequest request;
  request.set_controller_id(options_.controller_id);
  request.set_session_id(registration_.session_id());
  request.set_epoch(registration_.epoch());

  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      *error_message = "adapter activation was cancelled";
      return false;
    }
    accepting_ticks_ = true;
  }
  mujoco_sim::v1::ActivateControllerResponse response;
  const bool called = call(request, &response, error_message);
  if (!called || response.code() != OK) {
    if (called) {
      *error_message = response.message();
    }
    std::lock_guard lock(mutex_);
    accepting_ticks_ = false;
    queued_tick_.reset();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      accepting_ticks_ = false;
      queued_tick_.reset();
      *error_message = "adapter activation was cancelled";
      return false;
    }
    activated_ = true;
  }
  condition_.notify_all();
  return true;
}

template <typename Request, typename Response>
bool SyncAdapterClient::CallService(const std::string& method,
                                    const Request& request, Response* response,
                                    std::string* error_message,
                                    bool allow_after_stop) {
  if (response == nullptr || error_message == nullptr) {
    return false;
  }
  if (!registry_client_) {
    *error_message = "registry client is not initialized";
    return false;
  }
  std::string bytes;
  if (!request.SerializeToString(&bytes)) {
    *error_message = "failed to serialize service request";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(options_.service_timeout_ms);
  while ((allow_after_stop || !stop_requested_) &&
         std::chrono::steady_clock::now() < deadline) {
    if (!registry_client_->IsConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    eCAL::ServiceResponseVecT responses;
    registry_client_->CallWithResponse(method, bytes, responses, 100);
    for (const auto& service_response : responses) {
      if (service_response.call_state == eCAL::eCallState::executed) {
        if (!response->ParseFromString(service_response.response)) {
          *error_message = "failed to parse service response";
          return false;
        }
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  *error_message = stop_requested_ && !allow_after_stop
                       ? "service call cancelled: " + method
                       : "service call timed out: " + method;
  return false;
}

TickWaitResult SyncAdapterClient::WaitForTick(
    std::chrono::milliseconds timeout, mujoco_sim::v1::Tick* tick,
    std::string* error_message) {
  if (tick == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "tick and error_message must not be null";
    }
    return TickWaitResult::kError;
  }
  error_message->clear();
  if (timeout.count() < 0) {
    *error_message = "tick wait timeout must not be negative";
    return TickWaitResult::kError;
  }

  std::unique_lock lock(mutex_);
  if (!started_ || !activated_) {
    *error_message = "adapter client must be activated before waiting";
    return TickWaitResult::kError;
  }
  if (wait_in_progress_) {
    *error_message = "another thread is already waiting for a Tick";
    return TickWaitResult::kError;
  }
  if (outstanding_tick_.has_value()) {
    *error_message = "the previously received Tick has not been committed";
    return TickWaitResult::kError;
  }
  wait_in_progress_ = true;
  const auto finish = [this] { wait_in_progress_ = false; };
  const auto ready = [this] {
    return queued_tick_.has_value() || !terminal_error_.empty() ||
           stop_requested_.load() || (monitor_ecal_ && !eCAL::Ok());
  };

  bool signaled = ready();
  if (!signaled && timeout == std::chrono::milliseconds::max()) {
    while (!(signaled = ready())) {
      condition_.wait_for(lock, std::chrono::milliseconds(50));
    }
  } else if (!signaled) {
    signaled = condition_.wait_for(lock, timeout, ready);
  }
  if (!signaled) {
    finish();
    return TickWaitResult::kTimeout;
  }
  if (!terminal_error_.empty()) {
    *error_message = terminal_error_;
    finish();
    return TickWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    finish();
    return TickWaitResult::kStopped;
  }

  outstanding_tick_ = std::move(queued_tick_);
  queued_tick_.reset();
  *tick = *outstanding_tick_;
  finish();
  return TickWaitResult::kReceived;
}

bool SyncAdapterClient::BuildValidatedCommit(
    const mujoco_sim::v1::Tick& tick,
    const std::vector<CommandValue>& commands,
    mujoco_sim::v1::AdapterCommit* commit,
    std::string* error_message) const {
  if (commit == nullptr || error_message == nullptr) {
    return false;
  }
  std::unordered_map<std::string, double> values;
  values.reserve(commands.size());
  for (const auto& command : commands) {
    if (command.name.empty()) {
      *error_message = "command name must not be empty";
      return false;
    }
    if (!std::isfinite(command.value)) {
      *error_message = "command value must be finite: " + command.name;
      return false;
    }
    if (!values.emplace(command.name, command.value).second) {
      *error_message = "duplicate actuator command: " + command.name;
      return false;
    }
  }

  mujoco_sim::v1::AdapterCommit output;
  *output.mutable_header() = tick.header();
  output.set_controller_id(tick.controller_id());
  output.set_status(COMMIT_OK);
  for (const auto& descriptor : registration_.layout().commands()) {
    if (descriptor.dimension() != 1) {
      *error_message = "unsupported non-scalar actuator command: " +
                       descriptor.name();
      return false;
    }
    const auto value = values.find(descriptor.name());
    if (value == values.end()) {
      *error_message = "missing actuator command: " + descriptor.name();
      return false;
    }
    if (descriptor.has_ctrl_range() &&
        (value->second < descriptor.ctrl_min() ||
         value->second > descriptor.ctrl_max())) {
      *error_message = "actuator command is outside ctrlrange: " +
                       descriptor.name();
      return false;
    }
    auto* block = output.add_commands();
    block->set_name(descriptor.name());
    block->add_values(value->second);
  }
  if (values.size() !=
      static_cast<std::size_t>(registration_.layout().commands_size())) {
    for (const auto& [name, _] : values) {
      const auto known = std::find_if(
          registration_.layout().commands().begin(),
          registration_.layout().commands().end(),
          [&name](const auto& descriptor) {
            return descriptor.name() == name;
          });
      if (known == registration_.layout().commands().end()) {
        *error_message = "unknown actuator command: " + name;
        return false;
      }
    }
    *error_message = "command set does not match the registered layout";
    return false;
  }
  *commit = std::move(output);
  return true;
}

bool SyncAdapterClient::Commit(
    const mujoco_sim::v1::Tick& tick,
    const std::vector<CommandValue>& commands,
    std::string* error_message) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  return CommitWith(
      tick, commands,
      [this](const mujoco_sim::v1::AdapterCommit& commit) {
        return PublishCommit(commit);
      },
      error_message);
}

bool SyncAdapterClient::CommitWith(
    const mujoco_sim::v1::Tick& tick,
    const std::vector<CommandValue>& commands,
    const std::function<bool(const mujoco_sim::v1::AdapterCommit&)>& publish,
    std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();

  mujoco_sim::v1::Tick canonical_tick;
  {
    std::lock_guard lock(mutex_);
    if (!started_ || !activated_ || stop_requested_) {
      *error_message = "adapter client is not active";
      return false;
    }
    if (outstanding_tick_.has_value() &&
        SameTickIdentity(tick, *outstanding_tick_) &&
        SameTickPayload(tick, *outstanding_tick_)) {
      canonical_tick = *outstanding_tick_;
    } else if (committed_tick_.has_value() &&
               SameTickIdentity(tick, *committed_tick_) &&
               SameTickPayload(tick, *committed_tick_)) {
      canonical_tick = *committed_tick_;
    } else {
      *error_message = "Tick is not the current outstanding or committed Tick";
      return false;
    }
  }

  mujoco_sim::v1::AdapterCommit commit;
  if (!BuildValidatedCommit(canonical_tick, commands, &commit,
                            error_message)) {
    return false;
  }

  {
    std::lock_guard lock(mutex_);
    if (stop_requested_ || !activated_) {
      *error_message = "adapter client stopped before Commit";
      return false;
    }
    if (outstanding_tick_.has_value() &&
        SameTickIdentity(canonical_tick, *outstanding_tick_) &&
        SameTickPayload(canonical_tick, *outstanding_tick_)) {
      committed_tick_ = *outstanding_tick_;
      cached_commit_ = commit;
      outstanding_tick_.reset();
    } else if (committed_tick_.has_value() && cached_commit_.has_value() &&
               SameTickIdentity(canonical_tick, *committed_tick_) &&
               SameTickPayload(canonical_tick, *committed_tick_)) {
      if (commit.SerializeAsString() != cached_commit_->SerializeAsString()) {
        *error_message = "a different Commit was already sent for this Tick";
        return false;
      }
      commit = *cached_commit_;
    } else {
      *error_message = "Tick is no longer available for Commit";
      return false;
    }
  }

  const bool published = publish(commit);
  (void)published;
  condition_.notify_all();
  return true;
}

bool SyncAdapterClient::WaitBeforeActivation(std::chrono::milliseconds delay) {
  std::unique_lock lock(mutex_);
  condition_.wait_for(lock, delay,
                      [this] { return stop_requested_.load(); });
  return !stop_requested_;
}

void SyncAdapterClient::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    accepting_ticks_ = false;
  }
  condition_.notify_all();
}

bool SyncAdapterClient::Unregister(std::string* error_message) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  return UnregisterWith(
      [this](const mujoco_sim::v1::UnregisterControllerRequest& request,
             mujoco_sim::v1::UnregisterControllerResponse* response,
             std::string* error) {
        return CallService(mujoco_sim::protocol::kUnregisterControllerMethod,
                           request, response, error, true);
      },
      error_message);
}

bool SyncAdapterClient::UnregisterWith(
    const std::function<bool(
        const mujoco_sim::v1::UnregisterControllerRequest&,
        mujoco_sim::v1::UnregisterControllerResponse*, std::string*)>& call,
    std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  mujoco_sim::v1::UnregisterControllerRequest request;
  {
    std::lock_guard lock(mutex_);
    if (!registered_) {
      return true;
    }
    request.set_controller_id(options_.controller_id);
    request.set_session_id(registration_.session_id());
    request.set_epoch(registration_.epoch());
  }

  mujoco_sim::v1::UnregisterControllerResponse response;
  if (!call(request, &response, error_message)) {
    return false;
  }
  return CompleteUnregister(response, error_message);
}

bool SyncAdapterClient::CompleteUnregister(
    const mujoco_sim::v1::UnregisterControllerResponse& response,
    std::string* error_message) {
  if (response.code() != OK && response.code() != mujoco_sim::v1::NOT_FOUND &&
      response.code() != mujoco_sim::v1::INVALID_SESSION) {
    *error_message = response.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    registered_ = false;
    activated_ = false;
    accepting_ticks_ = false;
    queued_tick_.reset();
    outstanding_tick_.reset();
  }
  Stop();
  return true;
}

void SyncAdapterClient::AddLifecycleWarning(const std::string& warning) {
  if (warning.empty()) {
    return;
  }
  std::lock_guard lock(mutex_);
  if (lifecycle_warning_.empty()) {
    lifecycle_warning_ = warning;
  } else if (lifecycle_warning_.find(warning) == std::string::npos) {
    lifecycle_warning_ += "; " + warning;
  }
}

void SyncAdapterClient::SetTerminalError(const std::string& error) {
  {
    std::lock_guard lock(mutex_);
    if (terminal_error_.empty()) {
      terminal_error_ = error;
    }
    if (lifecycle_warning_.empty()) {
      lifecycle_warning_ = error;
    } else if (lifecycle_warning_.find(error) == std::string::npos) {
      lifecycle_warning_ += "; " + error;
    }
    accepting_ticks_ = false;
  }
  stop_requested_ = true;
  condition_.notify_all();
}

std::string SyncAdapterClient::lifecycle_warning() const {
  std::lock_guard lock(mutex_);
  return lifecycle_warning_;
}

SyncAdapterClient::CallbackLease::CallbackLease(
    std::shared_ptr<CallbackState> state)
    : state_(std::move(state)) {
  std::lock_guard lock(state_->mutex);
  if (state_->enabled && state_->client != nullptr) {
    ++state_->active_callbacks;
    client_ = state_->client;
  }
}

SyncAdapterClient::CallbackLease::~CallbackLease() {
  if (client_ == nullptr) {
    return;
  }
  {
    std::lock_guard lock(state_->mutex);
    --state_->active_callbacks;
  }
  state_->condition.notify_all();
}

void SyncAdapterClient::StopTickReception() {
  {
    std::lock_guard callback_lock(callback_state_->mutex);
    callback_state_->enabled = false;
    callback_state_->client = nullptr;
  }
  {
    std::lock_guard lock(mutex_);
    accepting_ticks_ = false;
  }
  if (tick_subscriber_) {
    tick_subscriber_->RemoveReceiveCallback();
  }
  if (status_subscriber_) {
    status_subscriber_->RemoveReceiveCallback();
  }
  tick_subscriber_.reset();
  status_subscriber_.reset();
  std::unique_lock callback_lock(callback_state_->mutex);
  callback_state_->condition.wait(
      callback_lock,
      [this] { return callback_state_->active_callbacks == 0; });
}

bool SyncAdapterClient::IsExpectedTick(
    const mujoco_sim::v1::Tick& tick) const {
  const auto& header = tick.header();
  return tick.controller_id() == options_.controller_id &&
         (tick.controller_state() == mujoco_sim::v1::ACTIVE ||
          tick.controller_state() == mujoco_sim::v1::TIMED_OUT) &&
         header.server_name() == options_.server_name &&
         header.session_id() == registration_.session_id() &&
         header.epoch() == registration_.epoch() &&
         header.protocol_major() == registration_.effective_protocol_major() &&
         header.protocol_minor() == registration_.effective_protocol_minor() &&
         tick.layout().SerializeAsString() ==
             registration_.layout().SerializeAsString();
}

void SyncAdapterClient::HandleStatusBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v1::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size)) ||
      status.header().server_name() != options_.server_name ||
      status.header().epoch() != registration_.epoch() ||
      status.header().protocol_major() !=
          registration_.effective_protocol_major()) {
    return;
  }
  for (const auto& controller : status.controllers()) {
    if (controller.controller_id() != options_.controller_id) {
      continue;
    }
    if (controller.session_id() != registration_.session_id()) {
      SetTerminalError("server replaced adapter session");
      return;
    }
    if (controller.state() == mujoco_sim::v1::ERROR) {
      SetTerminalError(controller.message().empty()
                           ? "server marked adapter as ERROR"
                           : controller.message());
      return;
    }
  }
}

void SyncAdapterClient::HandleTickBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v1::Tick tick;
  if (!tick.ParseFromArray(data, static_cast<int>(size)) ||
      !IsExpectedTick(tick)) {
    return;
  }
  HandleTick(
      tick,
      [this](const mujoco_sim::v1::Heartbeat& heartbeat) {
        PublishHeartbeat(heartbeat);
      },
      [this](const mujoco_sim::v1::AdapterCommit& commit) {
        (void)PublishCommit(commit);
      });
}

void SyncAdapterClient::HandleTick(
    const mujoco_sim::v1::Tick& tick,
    const std::function<void(const mujoco_sim::v1::Heartbeat&)>&
        publish_heartbeat,
    const std::function<void(const mujoco_sim::v1::AdapterCommit&)>&
        publish_commit) {
  std::optional<mujoco_sim::v1::AdapterCommit> retry_commit;
  bool accepted = false;
  bool queued = false;
  {
    std::lock_guard lock(mutex_);
    if (!accepting_ticks_ || stop_requested_) {
      return;
    }

    const auto record_conflict = [&] {
      const std::string warning =
          "conflicting payload for Tick " +
          std::to_string(tick.header().tick_id());
      if (lifecycle_warning_.empty()) {
        lifecycle_warning_ = warning;
      } else if (lifecycle_warning_.find(warning) == std::string::npos) {
        lifecycle_warning_ += "; " + warning;
      }
    };

    const auto inspect_known = [&](const auto& known) {
      return known.has_value() && SameTickIdentity(tick, *known);
    };
    if (inspect_known(committed_tick_)) {
      if (!SameTickPayload(tick, *committed_tick_)) {
        record_conflict();
        return;
      }
      retry_commit = cached_commit_;
      accepted = true;
    } else if (inspect_known(outstanding_tick_)) {
      if (!SameTickPayload(tick, *outstanding_tick_)) {
        record_conflict();
        return;
      }
      accepted = true;
    } else if (inspect_known(queued_tick_)) {
      if (!SameTickPayload(tick, *queued_tick_)) {
        record_conflict();
        return;
      }
      accepted = true;
    } else {
      std::uint64_t newest_tick_id = 0;
      bool have_known_tick = false;
      const auto include = [&](const auto& known) {
        if (known.has_value() && SameTickStream(tick, *known)) {
          newest_tick_id = std::max(newest_tick_id,
                                    known->header().tick_id());
          have_known_tick = true;
        }
      };
      include(committed_tick_);
      include(outstanding_tick_);
      include(queued_tick_);
      if (have_known_tick && tick.header().tick_id() < newest_tick_id) {
        return;
      }
      queued_tick_ = tick;
      accepted = true;
      queued = true;
    }
  }

  if (!accepted) {
    return;
  }
  publish_heartbeat(BuildHeartbeat(tick));
  if (retry_commit.has_value()) {
    publish_commit(*retry_commit);
  }
  if (queued) {
    condition_.notify_all();
  }
}

bool SyncAdapterClient::PublishCommit(
    const mujoco_sim::v1::AdapterCommit& commit) {
  std::string bytes;
  if (!commit.SerializeToString(&bytes)) {
    return false;
  }
  return PublishSerialized([this, &bytes] {
    return commit_publisher_ != nullptr && commit_publisher_->Send(bytes);
  });
}

void SyncAdapterClient::PublishHeartbeat(
    const mujoco_sim::v1::Heartbeat& heartbeat) {
  std::string bytes;
  if (heartbeat.SerializeToString(&bytes)) {
    (void)PublishSerialized([this, &bytes] {
      return heartbeat_publisher_ != nullptr &&
             heartbeat_publisher_->Send(bytes);
    });
  }
}

bool SyncAdapterClient::PublishSerialized(
    const std::function<bool()>& publish) {
  std::lock_guard publish_lock(publisher_mutex_);
  return publish();
}

}  // namespace mujoco_sim::adapter_sdk
