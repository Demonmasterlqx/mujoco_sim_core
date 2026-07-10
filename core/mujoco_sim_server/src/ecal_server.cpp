#include <mujoco_sim/server/ecal_server.hpp>

#include "sim_viewer.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ecal/ecal.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v1/sync.pb.h>

namespace mujoco_sim::server {
namespace {

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

template <typename Response>
int SerializeResponse(const Response& message, std::string* response) {
  if (!message.SerializeToString(response)) {
    return 1;
  }
  return 0;
}

template <typename Response>
int ParseError(mujoco_sim::v1::ErrorCode code, const std::string& message,
               std::string* response) {
  Response out;
  out.set_code(code);
  out.set_message(message);
  return SerializeResponse(out, response);
}

}  // namespace

EcalServer::EcalServer(ServerConfig config)
    : config_(std::move(config)),
      core_(mujoco_sim::core::CoreOptions{config_.server_name,
                                          config_.tick_timeout_ms}) {}

EcalServer::~EcalServer() {
  Stop();
  if (owns_ecal_) {
    eCAL::Finalize();
  }
}

bool EcalServer::Start(std::string* error_message) {
  if (!eCAL::IsInitialized()) {
    owns_ecal_ = eCAL::Initialize("mujoco-sim-server");
    if (!owns_ecal_) {
      *error_message = "failed to initialize eCAL";
      return false;
    }
  }

  if (!core_.LoadModel(
          config_.model_path,
          mujoco_sim::core::ModelLoadOptions{config_.initial_keyframe},
          error_message)) {
    return false;
  }

  if (!config_.headless) {
    viewer_snapshot_ = core_.CreateSnapshot(error_message);
    if (viewer_snapshot_ == nullptr) {
      return false;
    }
    viewer_ = std::make_unique<SimViewer>();
    if (!viewer_->Initialize(error_message)) {
      viewer_.reset();
      viewer_snapshot_.reset();
      return false;
    }
  }

  RegisterServices();
  RegisterSubscribersAndPublishers();
  return true;
}

void EcalServer::RegisterServices() {
  registry_ = std::make_unique<eCAL::CServiceServer>(
      mujoco_sim::protocol::RegistryServiceName(config_.server_name));

  registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kGetAvailableResourcesMethod),
      [this](const eCAL::SServiceMethodInformation&, const std::string& request,
             std::string& response) {
        return HandleGetAvailableResources(request, &response);
      });
  registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kRegisterControllerMethod),
      [this](const eCAL::SServiceMethodInformation&, const std::string& request,
             std::string& response) {
        return HandleRegisterController(request, &response);
      });
  registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kActivateControllerMethod),
      [this](const eCAL::SServiceMethodInformation&, const std::string& request,
             std::string& response) {
        return HandleActivateController(request, &response);
      });
  registry_->SetMethodCallback(
      MethodInfo(mujoco_sim::protocol::kUnregisterControllerMethod),
      [this](const eCAL::SServiceMethodInformation&, const std::string& request,
             std::string& response) {
        return HandleUnregisterController(request, &response);
      });
}

void EcalServer::RegisterSubscribersAndPublishers() {
  commit_subscriber_ = std::make_unique<eCAL::CSubscriber>(
      mujoco_sim::protocol::CommitTopicName(config_.server_name));
  commit_subscriber_->SetReceiveCallback(
      [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
             const eCAL::SReceiveCallbackData& data) {
        HandleCommitBytes(data.buffer, data.buffer_size);
      });

  heartbeat_subscriber_ = std::make_unique<eCAL::CSubscriber>(
      mujoco_sim::protocol::HeartbeatTopicName(config_.server_name));
  heartbeat_subscriber_->SetReceiveCallback(
      [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
             const eCAL::SReceiveCallbackData& data) {
        HandleHeartbeatBytes(data.buffer, data.buffer_size);
      });

  status_publisher_ = std::make_unique<eCAL::CPublisher>(
      mujoco_sim::protocol::StatusTopicName(config_.server_name));
}

int EcalServer::Run() {
  if (viewer_ != nullptr) {
    return RunGraphical();
  }
  return RunLoop();
}

int EcalServer::RunGraphical() {
  int result = 1;
  std::thread server_thread([this, &result]() {
    std::string error;
    if (!viewer_->QueueLoad(viewer_snapshot_.get(), config_.model_path,
                            &error)) {
      if (viewer_->close_requested()) {
        result = 0;
      }
      if (!error.empty()) {
        std::cerr << error << '\n';
      }
      viewer_->RequestClose();
      return;
    }
    result = RunLoop();
    viewer_->RequestClose();
  });

  viewer_->RenderLoop();
  Stop();
  server_thread.join();
  return result;
}

int EcalServer::RunLoop() {
  const auto startup_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(config_.startup_timeout_ms);
  auto next_status = std::chrono::steady_clock::now();
  auto next_viewer_sync = std::chrono::steady_clock::now();

  while (!stop_requested_ && eCAL::Ok()) {
    std::vector<mujoco_sim::v1::Tick> ticks;
    bool should_stop_success = false;
    bool should_stop_error = false;
    std::string viewer_error;

    {
      std::unique_lock lock(mutex_);
      if (core_.server_state() == mujoco_sim::v1::ERROR_PAUSED) {
        should_stop_error = true;
      } else if (config_.max_ticks > 0 &&
                 core_.current_tick_id() >= config_.max_ticks) {
        should_stop_success = true;
      } else if (core_.active_controller_count() == 0) {
        if (config_.max_ticks > 0 &&
            std::chrono::steady_clock::now() > startup_deadline) {
          should_stop_error = true;
        }
      } else if (!core_.tick_open()) {
        ticks = OpenTickLocked();
      } else {
        ticks = OpenTickLocked();
        core_.CheckTickTimeout(std::chrono::steady_clock::now());
      }

      if (viewer_ != nullptr &&
          std::chrono::steady_clock::now() >= next_viewer_sync) {
        if (viewer_->Synchronize(&core_, viewer_snapshot_.get(),
                                 &viewer_error)) {
          next_viewer_sync = std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(16);
        } else if (!viewer_->close_requested()) {
          should_stop_error = true;
        }
      }
    }

    if (!ticks.empty()) {
      PublishTicks(ticks);
    }
    if (!viewer_error.empty()) {
      std::cerr << viewer_error << '\n';
    }

    if (std::chrono::steady_clock::now() >= next_status) {
      PublishStatus();
      next_status = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(config_.status_period_ms);
    }

    if (should_stop_success) {
      PublishStatus();
      return 0;
    }
    if (should_stop_error) {
      PublishStatus();
      return 2;
    }

    std::unique_lock lock(mutex_);
    condition_.wait_for(lock, std::chrono::milliseconds(5));
  }
  return stop_requested_ ? 0 : 1;
}

void EcalServer::Stop() {
  {
    std::lock_guard lock(mutex_);
    stop_requested_.store(true);
  }
  condition_.notify_all();
}

std::vector<mujoco_sim::v1::Tick> EcalServer::OpenTickLocked() {
  return core_.OpenTick();
}

void EcalServer::PublishTicks(
    const std::vector<mujoco_sim::v1::Tick>& ticks) {
  for (const auto& tick : ticks) {
    auto& publisher = tick_publishers_[tick.controller_id()];
    if (!publisher) {
      publisher = std::make_unique<eCAL::CPublisher>(
          mujoco_sim::protocol::TickTopicName(config_.server_name,
                                              tick.controller_id()));
    }
    std::string bytes;
    if (tick.SerializeToString(&bytes)) {
      publisher->Send(bytes);
    }
  }
}

void EcalServer::PublishStatus() {
  if (!status_publisher_) {
    return;
  }
  mujoco_sim::v1::ServerStatus status;
  {
    std::lock_guard lock(mutex_);
    status = core_.MakeStatus();
  }
  std::string bytes;
  if (status.SerializeToString(&bytes)) {
    status_publisher_->Send(bytes);
  }
}

int EcalServer::HandleGetAvailableResources(const std::string& request,
                                            std::string* response) {
  mujoco_sim::v1::GetAvailableResourcesRequest parsed;
  if (!parsed.ParseFromString(request)) {
    return ParseError<mujoco_sim::v1::GetAvailableResourcesResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
  }
  std::lock_guard lock(mutex_);
  return SerializeResponse(core_.GetAvailableResources(parsed), response);
}

int EcalServer::HandleRegisterController(const std::string& request,
                                         std::string* response) {
  mujoco_sim::v1::RegisterControllerRequest parsed;
  if (!parsed.ParseFromString(request)) {
    return ParseError<mujoco_sim::v1::RegisterControllerResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
  }
  std::lock_guard lock(mutex_);
  const int result = SerializeResponse(core_.RegisterController(parsed), response);
  condition_.notify_all();
  return result;
}

int EcalServer::HandleActivateController(const std::string& request,
                                         std::string* response) {
  mujoco_sim::v1::ActivateControllerRequest parsed;
  if (!parsed.ParseFromString(request)) {
    return ParseError<mujoco_sim::v1::ActivateControllerResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
  }
  std::lock_guard lock(mutex_);
  const int result = SerializeResponse(core_.ActivateController(parsed), response);
  condition_.notify_all();
  return result;
}

int EcalServer::HandleUnregisterController(const std::string& request,
                                           std::string* response) {
  mujoco_sim::v1::UnregisterControllerRequest parsed;
  if (!parsed.ParseFromString(request)) {
    return ParseError<mujoco_sim::v1::UnregisterControllerResponse>(
        mujoco_sim::v1::INVALID_ARGUMENT, "invalid request", response);
  }
  std::lock_guard lock(mutex_);
  const int result = SerializeResponse(core_.UnregisterController(parsed), response);
  condition_.notify_all();
  return result;
}

void EcalServer::HandleCommitBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  mujoco_sim::v1::AdapterCommit commit;
  if (!commit.ParseFromArray(data, static_cast<int>(size))) {
    return;
  }
  {
    std::lock_guard lock(mutex_);
    core_.SubmitCommit(commit);
  }
  condition_.notify_all();
}

void EcalServer::HandleHeartbeatBytes(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  mujoco_sim::v1::Heartbeat heartbeat;
  if (!heartbeat.ParseFromArray(data, static_cast<int>(size))) {
    return;
  }
  {
    std::lock_guard lock(mutex_);
    core_.AcceptHeartbeat(heartbeat);
  }
  condition_.notify_all();
}

}  // namespace mujoco_sim::server
