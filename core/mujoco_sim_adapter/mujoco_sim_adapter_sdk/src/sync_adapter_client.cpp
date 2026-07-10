#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::adapter_sdk {
namespace {

using mujoco_sim::v1::COMMIT_ERROR;
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

}  // namespace

mujoco_sim::v1::AdapterCommit BuildCommit(
    const mujoco_sim::v1::Tick& tick,
    const std::vector<CommandValue>& commands) {
  mujoco_sim::v1::AdapterCommit commit;
  *commit.mutable_header() = tick.header();
  commit.set_controller_id(tick.controller_id());
  commit.set_status(COMMIT_OK);
  for (const auto& command : commands) {
    auto* block = commit.add_commands();
    block->set_name(command.name);
    block->add_values(command.value);
  }
  return commit;
}

mujoco_sim::v1::Heartbeat BuildHeartbeat(
    const mujoco_sim::v1::Tick& tick) {
  mujoco_sim::v1::Heartbeat heartbeat;
  *heartbeat.mutable_header() = tick.header();
  heartbeat.set_controller_id(tick.controller_id());
  return heartbeat;
}

SyncAdapterClient::SyncAdapterClient(AdapterOptions options)
    : options_(std::move(options)) {}

SyncAdapterClient::~SyncAdapterClient() {
  Stop();
  StopTickReception();
  commit_publisher_.reset();
  heartbeat_publisher_.reset();
  registry_client_.reset();
  if (owns_ecal_) {
    eCAL::Finalize();
  }
}

bool SyncAdapterClient::Start(std::string* error_message) {
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
  tick_subscriber_ = std::make_unique<eCAL::CSubscriber>(
      mujoco_sim::protocol::TickTopicName(options_.server_name,
                                          options_.controller_id));
  tick_subscriber_->SetReceiveCallback(
      [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
             const eCAL::SReceiveCallbackData& data) {
        HandleTickBytes(data.buffer, data.buffer_size);
      });

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
    handled_ticks_ = 0;
    have_last_commit_ = false;
    last_tick_.Clear();
    last_commit_.Clear();
  }
  return true;
}

bool SyncAdapterClient::Activate(std::string* error_message) {
  mujoco_sim::v1::ActivateControllerRequest request;
  request.set_controller_id(options_.controller_id);
  request.set_session_id(registration_.session_id());
  request.set_epoch(registration_.epoch());

  mujoco_sim::v1::ActivateControllerResponse response;
  if (!CallService(mujoco_sim::protocol::kActivateControllerMethod, request,
                   &response, error_message)) {
    return false;
  }
  if (response.code() != OK) {
    *error_message = response.message();
    return false;
  }
  return true;
}

template <typename Request, typename Response>
bool SyncAdapterClient::CallService(const std::string& method,
                                    const Request& request, Response* response,
                                    std::string* error_message) {
  std::string bytes;
  if (!request.SerializeToString(&bytes)) {
    *error_message = "failed to serialize service request";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(options_.service_timeout_ms);
  while (!stop_requested_ && std::chrono::steady_clock::now() < deadline) {
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
  *error_message = "service call timed out: " + method;
  return false;
}

int SyncAdapterClient::Run(TickHandler handler, std::uint64_t max_ticks) {
  {
    std::lock_guard lock(mutex_);
    handler_ = std::move(handler);
    accepting_ticks_ = !stop_requested_;
  }
  if (stop_requested_) {
    StopTickReception();
    return 0;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::string error;
  if (!Activate(&error)) {
    StopTickReception();
    return 1;
  }
  int result = 1;
  while (!stop_requested_ && eCAL::Ok()) {
    {
      std::unique_lock lock(mutex_);
      if (max_ticks > 0 && handled_ticks_ >= max_ticks) {
        result = 0;
        break;
      }
      condition_.wait_for(lock, std::chrono::milliseconds(10));
    }
  }
  if (stop_requested_) {
    result = 0;
  }
  StopTickReception();
  return result;
}

void SyncAdapterClient::Stop() {
  stop_requested_ = true;
  {
    std::lock_guard lock(mutex_);
    accepting_ticks_ = false;
  }
  condition_.notify_all();
}

bool SyncAdapterClient::BeginTickCallback() {
  std::lock_guard lock(mutex_);
  ++active_tick_callbacks_;
  return accepting_ticks_;
}

void SyncAdapterClient::EndTickCallback() {
  {
    std::lock_guard lock(mutex_);
    --active_tick_callbacks_;
  }
  condition_.notify_all();
}

void SyncAdapterClient::StopTickReception() {
  {
    std::lock_guard lock(mutex_);
    accepting_ticks_ = false;
  }
  tick_subscriber_.reset();
  std::unique_lock lock(mutex_);
  condition_.wait(lock,
                  [this] { return active_tick_callbacks_ == 0; });
  handler_ = {};
}

bool SyncAdapterClient::IsExpectedTick(
    const mujoco_sim::v1::Tick& tick) const {
  const auto& header = tick.header();
  return tick.controller_id() == options_.controller_id &&
         tick.controller_state() == mujoco_sim::v1::ACTIVE &&
         header.server_name() == options_.server_name &&
         header.session_id() == registration_.session_id() &&
         header.epoch() == registration_.epoch() &&
         header.protocol_major() == registration_.effective_protocol_major() &&
         header.protocol_minor() == registration_.effective_protocol_minor() &&
         tick.layout().SerializeAsString() ==
             registration_.layout().SerializeAsString();
}

SyncAdapterClient::TickCallbackGuard::TickCallbackGuard(
    SyncAdapterClient* client)
    : client_(client), accepted_(client_->BeginTickCallback()) {}

SyncAdapterClient::TickCallbackGuard::~TickCallbackGuard() {
  client_->EndTickCallback();
}

void SyncAdapterClient::HandleTickBytes(const void* data, std::size_t size) {
  TickCallbackGuard callback(this);
  if (!callback.accepted() || data == nullptr || size == 0 ||
      size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }
  mujoco_sim::v1::Tick tick;
  if (!tick.ParseFromArray(data, static_cast<int>(size))) {
    return;
  }
  if (!IsExpectedTick(tick)) {
    return;
  }

  HandleTick(
      tick,
      [this](const mujoco_sim::v1::Heartbeat& heartbeat) {
        PublishHeartbeat(heartbeat);
      },
      [this](const mujoco_sim::v1::AdapterCommit& commit) {
        PublishCommit(commit);
      });
}

void SyncAdapterClient::HandleTick(
    const mujoco_sim::v1::Tick& tick,
    const std::function<void(const mujoco_sim::v1::Heartbeat&)>&
        publish_heartbeat,
    const std::function<void(const mujoco_sim::v1::AdapterCommit&)>&
        publish_commit) {
  std::lock_guard tick_lock(tick_mutex_);

  TickHandler handler;
  mujoco_sim::v1::AdapterCommit cached_commit;
  bool duplicate = false;
  {
    std::lock_guard lock(mutex_);
    if (!accepting_ticks_) {
      return;
    }
    if (have_last_commit_ && SameTickIdentity(tick, last_tick_)) {
      if (!SameTickPayload(tick, last_tick_)) {
        return;
      }
      cached_commit = last_commit_;
      duplicate = true;
    }
    if (have_last_commit_ && SameTickStream(tick, last_tick_) &&
        tick.header().tick_id() < last_tick_.header().tick_id()) {
      return;
    }
    if (!duplicate) {
      handler = handler_;
    }
  }
  if (duplicate) {
    publish_heartbeat(BuildHeartbeat(tick));
    publish_commit(cached_commit);
    return;
  }
  if (!handler) {
    return;
  }

  publish_heartbeat(BuildHeartbeat(tick));
  auto commands = handler(tick);
  auto commit = BuildCommit(tick, commands);
  if (commands.empty()) {
    commit.set_status(COMMIT_ERROR);
    commit.set_error_message("adapter handler returned no commands");
  }

  {
    std::lock_guard lock(mutex_);
    last_tick_ = tick;
    last_commit_ = commit;
    have_last_commit_ = true;
    handled_ticks_++;
  }
  publish_commit(commit);
  condition_.notify_all();
}

void SyncAdapterClient::PublishCommit(
    const mujoco_sim::v1::AdapterCommit& commit) {
  if (!commit_publisher_) {
    return;
  }
  std::string bytes;
  if (commit.SerializeToString(&bytes)) {
    commit_publisher_->Send(bytes);
  }
}

void SyncAdapterClient::PublishHeartbeat(
    const mujoco_sim::v1::Heartbeat& heartbeat) {
  if (!heartbeat_publisher_) {
    return;
  }
  std::string bytes;
  if (heartbeat.SerializeToString(&bytes)) {
    heartbeat_publisher_->Send(bytes);
  }
}

}  // namespace mujoco_sim::adapter_sdk
