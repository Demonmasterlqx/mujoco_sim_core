#include <mujoco_sim/adapter_sdk/observer_client.hpp>

#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>

#include "ecal_runtime.hpp"

namespace mujoco_sim::adapter_sdk {
namespace {

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

std::uint64_t NowUnixNanos() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

}  // namespace

ObserverClient::ObserverClient(ObserverOptions options)
    : options_(std::move(options)),
      callback_state_(std::make_shared<CallbackState>()) {
  callback_state_->client = this;
}

ObserverClient::~ObserverClient() {
  Stop();
  StopObservationReception();
  std::string ignored;
  Unregister(&ignored);
  {
    std::lock_guard publish_lock(publisher_mutex_);
    heartbeat_publisher_.reset();
  }
  registry_client_.reset();
  ecal_runtime_.reset();
}

bool ObserverClient::ValidateOptions(std::string* error_message) const {
  if (options_.server_name.empty()) {
    *error_message = "server_name is required";
    return false;
  }
  if (options_.observer_id.empty()) {
    *error_message = "observer_id is required";
    return false;
  }
  if (options_.observation_names.empty()) {
    *error_message = "at least one observation sensor is required";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto& name : options_.observation_names) {
    if (name.empty()) {
      *error_message = "observation name must not be empty";
      return false;
    }
    if (!names.insert(name).second) {
      *error_message = "duplicate observation name: " + name;
      return false;
    }
  }
  if (options_.service_timeout_ms == 0) {
    *error_message = "service_timeout_ms must be greater than zero";
    return false;
  }
  if (options_.keepalive_period_ms == 0) {
    *error_message = "keepalive_period_ms must be greater than zero";
    return false;
  }
  if (options_.timeout_ms != 0 &&
      options_.keepalive_period_ms >= options_.timeout_ms) {
    *error_message = "keepalive_period_ms must be less than timeout_ms";
    return false;
  }
  if (options_.queue_capacity == 0) {
    *error_message = "queue_capacity must be greater than zero";
    return false;
  }
  if (options_.every_n_steps.has_value() == options_.period_s.has_value()) {
    *error_message =
        "exactly one of every_n_steps or period_s must be configured";
    return false;
  }
  if (options_.every_n_steps.has_value() && *options_.every_n_steps == 0) {
    *error_message = "every_n_steps must be greater than zero";
    return false;
  }
  if (options_.period_s.has_value() &&
      (!std::isfinite(*options_.period_s) || *options_.period_s <= 0.0)) {
    *error_message = "period_s must be finite and greater than zero";
    return false;
  }
  return true;
}

bool ObserverClient::Start(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (started_) {
      *error_message = "observer client has already been started";
      return false;
    }
  }
  if (stop_requested_) {
    *error_message = "observer client has been stopped";
    return false;
  }
  if (!ValidateOptions(error_message)) {
    return false;
  }

  ecal_runtime_ = internal::AcquireEcalRuntime(
      "mujoco-sim-observer-" + options_.observer_id, error_message);
  if (!ecal_runtime_) {
    return false;
  }

  eCAL::ServiceMethodInformationSetT methods;
  methods.insert(MethodInfo(mujoco_sim::protocol::kRegisterAdapterMethod));
  methods.insert(MethodInfo(mujoco_sim::protocol::kActivateAdapterMethod));
  methods.insert(MethodInfo(mujoco_sim::protocol::kUnregisterAdapterMethod));
  registry_client_ = std::make_unique<eCAL::CServiceClient>(
      mujoco_sim::protocol::RegistryServiceName(options_.server_name),
      methods);
  if (!Register(error_message)) {
    const std::string registration_error = *error_message;
    std::string ignored;
    UnregisterWith(
        [this](const mujoco_sim::v1::UnregisterAdapterRequest& request,
               mujoco_sim::v1::UnregisterAdapterResponse* response,
               std::string* unregister_error) {
          return CallService(mujoco_sim::protocol::kUnregisterAdapterMethod,
                             request, response, unregister_error, true);
        },
        &ignored);
    registry_client_.reset();
    ecal_runtime_.reset();
    *error_message = registration_error;
    return false;
  }

  const auto unregister = [this] {
    std::string ignored;
    UnregisterWith(
        [this](const mujoco_sim::v1::UnregisterAdapterRequest& request,
               mujoco_sim::v1::UnregisterAdapterResponse* response,
               std::string* unregister_error) {
          return CallService(mujoco_sim::protocol::kUnregisterAdapterMethod,
                             request, response, unregister_error, true);
        },
        &ignored);
  };

  try {
    heartbeat_publisher_ = std::make_unique<eCAL::CPublisher>(
        mujoco_sim::protocol::AdapterHeartbeatTopicName(
            options_.server_name));
    {
      std::lock_guard callback_lock(callback_state_->mutex);
      callback_state_->client = this;
      callback_state_->enabled = true;
    }
    const auto callback_state = callback_state_;
    observation_subscriber_ = std::make_unique<eCAL::CSubscriber>(
        mujoco_sim::protocol::ObservationTopicName(options_.server_name,
                                                    options_.observer_id));
    observation_subscriber_->SetReceiveCallback(
        [callback_state](const eCAL::STopicId&,
                         const eCAL::SDataTypeInformation&,
                         const eCAL::SReceiveCallbackData& data) {
          CallbackLease callback(callback_state);
          if (callback.client() != nullptr) {
            callback.client()->HandleObservationBytes(data.buffer,
                                                       data.buffer_size);
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
            callback.client()->HandleStatusBytes(data.buffer,
                                                 data.buffer_size);
          }
        });
  } catch (const std::exception& error) {
    Stop();
    StopObservationReception();
    unregister();
    {
      std::lock_guard publish_lock(publisher_mutex_);
      heartbeat_publisher_.reset();
    }
    registry_client_.reset();
    ecal_runtime_.reset();
    *error_message = std::string("failed to create observer endpoints: ") +
                     error.what();
    return false;
  }

  if (!FinishStart(unregister, error_message)) {
    {
      std::lock_guard publish_lock(publisher_mutex_);
      heartbeat_publisher_.reset();
    }
    registry_client_.reset();
    ecal_runtime_.reset();
    return false;
  }

  try {
    StartHeartbeatThread();
  } catch (const std::exception& error) {
    Stop();
    StopObservationReception();
    unregister();
    {
      std::lock_guard publish_lock(publisher_mutex_);
      heartbeat_publisher_.reset();
    }
    registry_client_.reset();
    ecal_runtime_.reset();
    *error_message = std::string("failed to start observer keepalive: ") +
                     error.what();
    return false;
  }
  return true;
}

bool ObserverClient::FinishStart(const std::function<void()>& unregister,
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
  if (!cancelled) {
    return true;
  }
  StopObservationReception();
  unregister();
  *error_message = "observer start was cancelled";
  return false;
}

bool ObserverClient::Register(std::string* error_message) {
  mujoco_sim::v1::RegisterAdapterRequest request;
  request.set_server_name(options_.server_name);
  request.set_adapter_id(options_.observer_id);
  request.set_role(mujoco_sim::v1::OBSERVER);
  request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  request.set_timeout_ms(options_.timeout_ms);
  for (const auto& name : options_.observation_names) {
    request.add_observation_names(name);
  }
  if (options_.every_n_steps.has_value()) {
    request.mutable_delivery()->set_every_n_steps(*options_.every_n_steps);
  } else {
    request.mutable_delivery()->set_period_s(*options_.period_s);
  }

  if (!CallService(mujoco_sim::protocol::kRegisterAdapterMethod, request,
                   &registration_, error_message)) {
    return false;
  }
  if (registration_.code() != mujoco_sim::v1::OK) {
    *error_message = registration_.message();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    registered_ = true;
  }
  if (registration_.adapter_id() != options_.observer_id ||
      registration_.role() != mujoco_sim::v1::OBSERVER ||
      registration_.session_id() == 0 ||
      registration_.epoch() == 0 ||
      registration_.effective_protocol_major() !=
          mujoco_sim::protocol::kProtocolMajor ||
      registration_.effective_protocol_minor() !=
          mujoco_sim::protocol::kProtocolMinor ||
      registration_.layout().commands_size() != 0 ||
      registration_.state() != mujoco_sim::v1::PROVISIONAL) {
    *error_message = "invalid observer registration response";
    return false;
  }
  if (registration_.layout().observations_size() !=
      static_cast<int>(options_.observation_names.size())) {
    *error_message = "observer registration layout does not match request";
    return false;
  }
  for (int i = 0; i < registration_.layout().observations_size(); ++i) {
    const auto& descriptor = registration_.layout().observations(i);
    if (descriptor.name() != options_.observation_names[i] ||
        descriptor.type() != mujoco_sim::v1::OBSERVATION_SENSOR ||
        descriptor.dimension() == 0) {
      *error_message = "observer registration layout does not match request";
      return false;
    }
  }

  {
    std::lock_guard lock(mutex_);
    observations_.clear();
    last_sequence_ = 0;
    last_tick_id_ = 0;
    dropped_frames_ = 0;
    terminal_error_.clear();
    lifecycle_warning_.clear();
  }
  return true;
}

template <typename Request, typename Response>
bool ObserverClient::CallService(const std::string& method,
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
      if (service_response.call_state != eCAL::eCallState::executed) {
        continue;
      }
      if (!response->ParseFromString(service_response.response)) {
        *error_message = "failed to parse service response";
        return false;
      }
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  *error_message = stop_requested_ && !allow_after_stop
                       ? "service call cancelled: " + method
                       : "service call timed out: " + method;
  return false;
}

bool ObserverClient::Activate(std::string* error_message) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  return ActivateWith(
      [this](const mujoco_sim::v1::ActivateAdapterRequest& request,
             mujoco_sim::v1::ActivateAdapterResponse* response,
             std::string* error) {
        return CallService(mujoco_sim::protocol::kActivateAdapterMethod,
                           request, response, error);
      },
      std::chrono::milliseconds(100), error_message);
}

bool ObserverClient::ActivateWith(
    const std::function<bool(
        const mujoco_sim::v1::ActivateAdapterRequest&,
        mujoco_sim::v1::ActivateAdapterResponse*, std::string*)>& call,
    std::chrono::milliseconds discovery_delay, std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  {
    std::lock_guard lock(mutex_);
    if (!started_ || !registered_) {
      *error_message = "observer client must be started before activation";
      return false;
    }
    if (activated_) {
      *error_message = "observer client has already been activated";
      return false;
    }
    if (stop_requested_) {
      *error_message = "observer client has been stopped";
      return false;
    }
  }
  if (!WaitBeforeActivation(discovery_delay)) {
    *error_message = "observer activation was cancelled";
    return false;
  }

  mujoco_sim::v1::ActivateAdapterRequest request;
  request.set_adapter_id(options_.observer_id);
  request.set_role(mujoco_sim::v1::OBSERVER);
  request.set_session_id(registration_.session_id());
  request.set_epoch(registration_.epoch());

  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      *error_message = "observer activation was cancelled";
      return false;
    }
    accepting_observations_ = true;
  }
  mujoco_sim::v1::ActivateAdapterResponse response;
  const bool called = call(request, &response, error_message);
  if (!called || response.code() != mujoco_sim::v1::OK ||
      response.state() != mujoco_sim::v1::ACTIVE) {
    if (called) {
      *error_message = response.message().empty()
                           ? "observer activation was rejected"
                           : response.message();
    }
    std::lock_guard lock(mutex_);
    accepting_observations_ = false;
    observations_.clear();
    return false;
  }
  {
    std::lock_guard lock(mutex_);
    if (stop_requested_) {
      accepting_observations_ = false;
      observations_.clear();
      *error_message = "observer activation was cancelled";
      return false;
    }
    activated_ = true;
  }
  heartbeat_condition_.notify_all();
  condition_.notify_all();
  return true;
}

bool ObserverClient::WaitBeforeActivation(std::chrono::milliseconds delay) {
  std::unique_lock lock(mutex_);
  heartbeat_condition_.wait_for(
      lock, delay, [this] { return stop_requested_.load(); });
  return !stop_requested_;
}

ObservationWaitResult ObserverClient::WaitForObservation(
    std::chrono::milliseconds timeout,
    mujoco_sim::v1::ObservationFrame* frame, std::string* error_message) {
  if (frame == nullptr || error_message == nullptr) {
    if (error_message != nullptr) {
      *error_message = "frame and error_message must not be null";
    }
    return ObservationWaitResult::kError;
  }
  error_message->clear();
  if (timeout.count() < 0) {
    *error_message = "observation wait timeout must not be negative";
    return ObservationWaitResult::kError;
  }

  std::unique_lock lock(mutex_);
  if (!terminal_error_.empty()) {
    *error_message = terminal_error_;
    return ObservationWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    return ObservationWaitResult::kStopped;
  }
  if (!started_ || !activated_) {
    *error_message = "observer client must be activated before waiting";
    return ObservationWaitResult::kError;
  }
  if (wait_in_progress_) {
    *error_message = "another thread is already waiting for an observation";
    return ObservationWaitResult::kError;
  }
  wait_in_progress_ = true;
  const auto finish = [this] { wait_in_progress_ = false; };
  const auto ready = [this] {
    return !observations_.empty() || !terminal_error_.empty() ||
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
    return ObservationWaitResult::kTimeout;
  }
  if (!terminal_error_.empty()) {
    *error_message = terminal_error_;
    finish();
    return ObservationWaitResult::kError;
  }
  if (stop_requested_ || (monitor_ecal_ && !eCAL::Ok())) {
    finish();
    return ObservationWaitResult::kStopped;
  }

  *frame = std::move(observations_.front());
  observations_.pop_front();
  finish();
  return ObservationWaitResult::kReceived;
}

void ObserverClient::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    accepting_observations_ = false;
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  JoinHeartbeatThread();
}

bool ObserverClient::Unregister(std::string* error_message) {
  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  return UnregisterWith(
      [this](const mujoco_sim::v1::UnregisterAdapterRequest& request,
             mujoco_sim::v1::UnregisterAdapterResponse* response,
             std::string* error) {
        return CallService(mujoco_sim::protocol::kUnregisterAdapterMethod,
                           request, response, error, true);
      },
      error_message);
}

bool ObserverClient::UnregisterWith(
    const std::function<bool(
        const mujoco_sim::v1::UnregisterAdapterRequest&,
        mujoco_sim::v1::UnregisterAdapterResponse*, std::string*)>& call,
    std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  mujoco_sim::v1::UnregisterAdapterRequest request;
  {
    std::lock_guard lock(mutex_);
    if (!registered_) {
      return true;
    }
    request.set_adapter_id(options_.observer_id);
    request.set_role(mujoco_sim::v1::OBSERVER);
    request.set_session_id(registration_.session_id());
    request.set_epoch(registration_.epoch());
  }

  mujoco_sim::v1::UnregisterAdapterResponse response;
  if (!call(request, &response, error_message)) {
    return false;
  }
  return CompleteUnregister(response, error_message);
}

bool ObserverClient::CompleteUnregister(
    const mujoco_sim::v1::UnregisterAdapterResponse& response,
    std::string* error_message) {
  if (response.code() != mujoco_sim::v1::OK &&
      response.code() != mujoco_sim::v1::NOT_FOUND &&
      response.code() != mujoco_sim::v1::INVALID_SESSION) {
    *error_message = response.message();
    return false;
  }
  TransitionToStopped();
  return true;
}

void ObserverClient::MarkStoppedLocked() {
  stop_requested_ = true;
  registered_ = false;
  activated_ = false;
  accepting_observations_ = false;
  observations_.clear();
}

void ObserverClient::TransitionToStopped() {
  {
    std::lock_guard lock(mutex_);
    MarkStoppedLocked();
  }
  condition_.notify_all();
  heartbeat_condition_.notify_all();
  JoinHeartbeatThread();
}

bool ObserverClient::IsExpectedObservation(
    const mujoco_sim::v1::ObservationFrame& frame) const {
  const auto& header = frame.header();
  if (frame.observer_id() != options_.observer_id || frame.sequence() == 0 ||
      header.server_name() != options_.server_name ||
      header.session_id() != registration_.session_id() ||
      header.epoch() != registration_.epoch() ||
      header.protocol_major() != registration_.effective_protocol_major() ||
      header.protocol_minor() != registration_.effective_protocol_minor() ||
      frame.observations_size() !=
          registration_.layout().observations_size()) {
    return false;
  }
  for (int i = 0; i < frame.observations_size(); ++i) {
    const auto& block = frame.observations(i);
    const auto& descriptor = registration_.layout().observations(i);
    if (block.name() != descriptor.name() ||
        block.values_size() != static_cast<int>(descriptor.dimension())) {
      return false;
    }
  }
  return true;
}

void ObserverClient::HandleObservationBytes(const void* data,
                                            std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v1::ObservationFrame frame;
  if (!frame.ParseFromArray(data, static_cast<int>(size)) ||
      !IsExpectedObservation(frame)) {
    return;
  }
  HandleObservation(frame);
}

void ObserverClient::HandleObservation(
    const mujoco_sim::v1::ObservationFrame& frame) {
  bool queued = false;
  {
    std::lock_guard lock(mutex_);
    if (!accepting_observations_ || stop_requested_ ||
        frame.sequence() <= last_sequence_) {
      return;
    }
    if (last_sequence_ > 0 && frame.sequence() > last_sequence_ + 1) {
      dropped_frames_ += frame.sequence() - last_sequence_ - 1;
    }
    last_sequence_ = frame.sequence();
    last_tick_id_ = frame.header().tick_id();
    if (observations_.size() == options_.queue_capacity) {
      observations_.pop_front();
      ++dropped_frames_;
    }
    observations_.push_back(frame);
    queued = true;
  }
  if (queued) {
    condition_.notify_all();
  }
}

void ObserverClient::HandleStatusBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v1::ServerStatus status;
  if (!status.ParseFromArray(data, static_cast<int>(size)) ||
      status.header().server_name() != options_.server_name ||
      status.header().epoch() != registration_.epoch() ||
      status.header().protocol_major() !=
          registration_.effective_protocol_major() ||
      status.header().protocol_minor() !=
          registration_.effective_protocol_minor()) {
    return;
  }
  if (status.state() == mujoco_sim::v1::STOPPED) {
    TransitionToStopped();
    return;
  }
  if (status.state() == mujoco_sim::v1::ERROR_PAUSED) {
    SetTerminalError(status.message().empty()
                         ? "server entered ERROR_PAUSED"
                         : status.message());
    return;
  }
  for (const auto& adapter : status.adapters()) {
    if (adapter.adapter_id() != options_.observer_id ||
        adapter.role() != mujoco_sim::v1::OBSERVER) {
      continue;
    }
    if (adapter.session_id() != registration_.session_id()) {
      SetTerminalError("server replaced observer session");
      return;
    }
    if (adapter.state() == mujoco_sim::v1::ERROR ||
        adapter.state() == mujoco_sim::v1::TIMED_OUT) {
      SetTerminalError(adapter.message().empty()
                           ? "server marked observer as " +
                                 mujoco_sim::v1::ControllerState_Name(
                                     adapter.state())
                           : adapter.message());
      return;
    }
  }
}

void ObserverClient::SetTerminalError(const std::string& error) {
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
    accepting_observations_ = false;
  }
  stop_requested_ = true;
  condition_.notify_all();
  heartbeat_condition_.notify_all();
}

std::uint64_t ObserverClient::dropped_frames() const {
  std::lock_guard lock(mutex_);
  return dropped_frames_;
}

std::string ObserverClient::lifecycle_warning() const {
  std::lock_guard lock(mutex_);
  return lifecycle_warning_;
}

ObserverClient::CallbackLease::CallbackLease(
    std::shared_ptr<CallbackState> state)
    : state_(std::move(state)) {
  std::lock_guard lock(state_->mutex);
  if (state_->enabled && state_->client != nullptr) {
    ++state_->active_callbacks;
    client_ = state_->client;
  }
}

ObserverClient::CallbackLease::~CallbackLease() {
  if (client_ == nullptr) {
    return;
  }
  {
    std::lock_guard lock(state_->mutex);
    --state_->active_callbacks;
  }
  state_->condition.notify_all();
}

void ObserverClient::StopObservationReception() {
  {
    std::lock_guard callback_lock(callback_state_->mutex);
    callback_state_->enabled = false;
    callback_state_->client = nullptr;
  }
  callback_state_->condition.notify_all();
  {
    std::lock_guard lock(mutex_);
    accepting_observations_ = false;
  }
  if (observation_subscriber_) {
    observation_subscriber_->RemoveReceiveCallback();
  }
  if (status_subscriber_) {
    status_subscriber_->RemoveReceiveCallback();
  }
  std::unique_lock callback_lock(callback_state_->mutex);
  callback_state_->condition.wait(
      callback_lock,
      [this] { return callback_state_->active_callbacks == 0; });
  callback_lock.unlock();
  observation_subscriber_.reset();
  status_subscriber_.reset();
}

void ObserverClient::StartHeartbeatThread() {
  std::lock_guard thread_lock(heartbeat_thread_mutex_);
  if (!stop_requested_ && !heartbeat_thread_.joinable()) {
    heartbeat_thread_ = std::thread([this] { HeartbeatLoop(); });
  }
}

void ObserverClient::JoinHeartbeatThread() {
  std::lock_guard thread_lock(heartbeat_thread_mutex_);
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
}

void ObserverClient::HeartbeatLoop() {
  std::unique_lock lock(mutex_);
  while (!stop_requested_) {
    heartbeat_condition_.wait(lock, [this] {
      return stop_requested_.load() || (started_ && activated_ && registered_);
    });
    if (stop_requested_) {
      break;
    }
    lock.unlock();
    PublishHeartbeat(BuildHeartbeat());
    lock.lock();
    heartbeat_condition_.wait_for(
        lock, std::chrono::milliseconds(options_.keepalive_period_ms),
        [this] {
          return stop_requested_.load() || !activated_ || !registered_;
        });
  }
}

mujoco_sim::v1::AdapterHeartbeat ObserverClient::BuildHeartbeat() const {
  mujoco_sim::v1::AdapterHeartbeat heartbeat;
  std::lock_guard lock(mutex_);
  auto* header = heartbeat.mutable_header();
  header->set_server_name(options_.server_name);
  header->set_session_id(registration_.session_id());
  header->set_epoch(registration_.epoch());
  header->set_tick_id(last_tick_id_);
  header->set_timestamp_unix_nanos(NowUnixNanos());
  header->set_protocol_major(registration_.effective_protocol_major());
  header->set_protocol_minor(registration_.effective_protocol_minor());
  heartbeat.set_adapter_id(options_.observer_id);
  heartbeat.set_role(mujoco_sim::v1::OBSERVER);
  heartbeat.set_last_observation_sequence(last_sequence_);
  return heartbeat;
}

void ObserverClient::PublishHeartbeat(
    const mujoco_sim::v1::AdapterHeartbeat& heartbeat) {
  std::string bytes;
  if (!heartbeat.SerializeToString(&bytes)) {
    return;
  }
  std::lock_guard publish_lock(publisher_mutex_);
  if (heartbeat_publisher_) {
    (void)heartbeat_publisher_->Send(bytes);
  }
}

}  // namespace mujoco_sim::adapter_sdk
