#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <chrono>
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
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::string error;
  if (!Activate(&error)) {
    return 1;
  }
  while (!stop_requested_ && eCAL::Ok()) {
    {
      std::unique_lock lock(mutex_);
      if (max_ticks > 0 && handled_ticks_ >= max_ticks) {
        return 0;
      }
      condition_.wait_for(lock, std::chrono::milliseconds(10));
    }
  }
  return stop_requested_ ? 0 : 1;
}

void SyncAdapterClient::Stop() {
  stop_requested_ = true;
  condition_.notify_all();
}

void SyncAdapterClient::HandleTickBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  mujoco_sim::v1::Tick tick;
  if (!tick.ParseFromArray(data, static_cast<int>(size))) {
    return;
  }
  if (tick.controller_id() != options_.controller_id) {
    return;
  }

  TickHandler handler;
  {
    std::lock_guard lock(mutex_);
    if (have_last_commit_ && tick.header().tick_id() == last_tick_id_) {
      PublishHeartbeat(BuildHeartbeat(tick));
      PublishCommit(last_commit_);
      return;
    }
    if (have_last_commit_ && tick.header().tick_id() < last_tick_id_) {
      return;
    }
    handler = handler_;
  }
  if (!handler) {
    return;
  }

  PublishHeartbeat(BuildHeartbeat(tick));
  auto commands = handler(tick);
  auto commit = BuildCommit(tick, commands);
  if (commands.empty()) {
    commit.set_status(COMMIT_ERROR);
    commit.set_error_message("adapter handler returned no commands");
  }

  {
    std::lock_guard lock(mutex_);
    last_tick_id_ = tick.header().tick_id();
    last_commit_ = commit;
    have_last_commit_ = true;
    handled_ticks_++;
  }
  PublishCommit(commit);
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
