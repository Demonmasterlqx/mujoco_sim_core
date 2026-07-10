#include <mujoco_sim/core/simulation_core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v1::ACTIVE;
using mujoco_sim::v1::ALREADY_EXISTS;
using mujoco_sim::v1::COMMAND_ACTUATOR;
using mujoco_sim::v1::ERROR;
using mujoco_sim::v1::ERROR_PAUSED;
using mujoco_sim::v1::FAILED_PRECONDITION;
using mujoco_sim::v1::IDLE;
using mujoco_sim::v1::INVALID_ARGUMENT;
using mujoco_sim::v1::INVALID_COMMAND;
using mujoco_sim::v1::INVALID_EPOCH;
using mujoco_sim::v1::INVALID_SESSION;
using mujoco_sim::v1::INVALID_TICK;
using mujoco_sim::v1::NOT_FOUND;
using mujoco_sim::v1::OBSERVATION_SENSOR;
using mujoco_sim::v1::OK;
using mujoco_sim::v1::PROVISIONAL;
using mujoco_sim::v1::RESOURCE_CONFLICT;
using mujoco_sim::v1::RUNNING;
using mujoco_sim::v1::TIMEOUT;
using mujoco_sim::v1::UNREGISTERED;
using mujoco_sim::v1::VERSION_MISMATCH;

std::uint64_t NowUnixNanos() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool HasDuplicates(const google::protobuf::RepeatedPtrField<std::string>& names,
                   std::string* duplicate) {
  std::set<std::string> seen;
  for (const auto& name : names) {
    if (!seen.insert(name).second) {
      if (duplicate != nullptr) {
        *duplicate = name;
      }
      return true;
    }
  }
  return false;
}

bool SameCommandPayload(
    const google::protobuf::RepeatedPtrField<
        mujoco_sim::v1::ActuatorCommandBlock>& lhs,
    const google::protobuf::RepeatedPtrField<
        mujoco_sim::v1::ActuatorCommandBlock>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }

  std::map<std::string, std::vector<double>> lhs_by_name;
  for (const auto& command : lhs) {
    lhs_by_name.emplace(command.name(),
                        std::vector<double>(command.values().begin(),
                                            command.values().end()));
  }

  std::map<std::string, std::vector<double>> rhs_by_name;
  for (const auto& command : rhs) {
    rhs_by_name.emplace(command.name(),
                        std::vector<double>(command.values().begin(),
                                            command.values().end()));
  }

  return lhs_by_name == rhs_by_name;
}

bool SameCommitPayload(const mujoco_sim::v1::AdapterCommit& lhs,
                       const mujoco_sim::v1::AdapterCommit& rhs) {
  return lhs.status() == rhs.status() &&
         lhs.error_message() == rhs.error_message() &&
         SameCommandPayload(lhs.commands(), rhs.commands());
}

void CopyDescriptor(const mujoco_sim::v1::ResourceDescriptor& from,
                    mujoco_sim::v1::ResourceDescriptor* to) {
  *to = from;
}

template <typename Response>
Response ErrorResponse(mujoco_sim::v1::ErrorCode code,
                       const std::string& message) {
  Response response;
  response.set_code(code);
  response.set_message(message);
  return response;
}

}  // namespace

struct SimulationCore::Controller {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t timeout_ms = 1000;
  mujoco_sim::v1::ControllerState state = PROVISIONAL;
  mujoco_sim::v1::ResourceLayout layout;
  std::uint64_t last_heartbeat_tick = 0;
  std::string message;
};

struct SimulationSnapshot::Impl {
  ~Impl() {
    if (data != nullptr) {
      mj_deleteData(data);
    }
    if (model != nullptr) {
      mj_deleteModel(model);
    }
  }

  mjModel* model = nullptr;
  mjData* data = nullptr;
  std::weak_ptr<void> owner;
  std::uint64_t model_generation = 0;
};

SimulationSnapshot::SimulationSnapshot() : impl_(std::make_unique<Impl>()) {}
SimulationSnapshot::~SimulationSnapshot() = default;
SimulationSnapshot::SimulationSnapshot(SimulationSnapshot&&) noexcept = default;
SimulationSnapshot& SimulationSnapshot::operator=(
    SimulationSnapshot&&) noexcept = default;

mjModel_* SimulationSnapshot::model() const { return impl_->model; }
mjData_* SimulationSnapshot::data() const { return impl_->data; }

void SimulationCore::MjModelDeleter::operator()(mjModel_* model) const {
  if (model != nullptr) {
    mj_deleteModel(model);
  }
}

void SimulationCore::MjDataDeleter::operator()(mjData_* data) const {
  if (data != nullptr) {
    mj_deleteData(data);
  }
}

SimulationCore::SimulationCore(CoreOptions options)
    : options_(std::move(options)) {}

SimulationCore::~SimulationCore() = default;
SimulationCore::SimulationCore(SimulationCore&&) noexcept = default;
SimulationCore& SimulationCore::operator=(SimulationCore&&) noexcept = default;

bool SimulationCore::LoadModel(const std::string& model_path,
                               std::string* error_message) {
  return LoadModel(model_path, ModelLoadOptions{}, error_message);
}

bool SimulationCore::LoadModel(const std::string& model_path,
                               const ModelLoadOptions& options,
                               std::string* error_message) {
  char load_error[1024] = {};
  std::unique_ptr<mjModel_, MjModelDeleter> model(
      mj_loadXML(model_path.c_str(), nullptr, load_error, sizeof(load_error)));
  if (!model) {
    if (error_message != nullptr) {
      *error_message = load_error[0] == '\0' ? "failed to load MuJoCo model"
                                             : load_error;
    }
    return false;
  }

  std::unique_ptr<mjData_, MjDataDeleter> data(mj_makeData(model.get()));
  if (!data) {
    if (error_message != nullptr) {
      *error_message = "failed to allocate MuJoCo data";
    }
    return false;
  }

  if (options.initial_keyframe.empty()) {
    mj_resetData(model.get(), data.get());
  } else {
    const int keyframe_id =
        mj_name2id(model.get(), mjOBJ_KEY, options.initial_keyframe.c_str());
    if (keyframe_id < 0) {
      if (error_message != nullptr) {
        *error_message = "initial keyframe '" + options.initial_keyframe +
                         "' was not found in MuJoCo model";
      }
      return false;
    }
    mj_resetDataKeyframe(model.get(), data.get(), keyframe_id);
  }
  if (model->nu > 0) {
    mju_zero(data->ctrl, model->nu);
  }
  mj_forward(model.get(), data.get());

  model_ = std::move(model);
  data_ = std::move(data);
  ++model_generation_;

  observations_.clear();
  commands_.clear();
  observations_by_name_.clear();
  commands_by_name_.clear();
  controllers_.clear();
  actuator_owner_.clear();
  pending_commits_.clear();
  epoch_++;
  current_tick_id_ = 0;
  simulation_time_ = data_->time;
  server_state_ = IDLE;
  tick_open_ = false;
  status_message_.clear();
  BuildCatalogs();
  return true;
}

bool SimulationCore::IsModelLoaded() const { return model_ && data_; }

std::unique_ptr<SimulationSnapshot> SimulationCore::CreateSnapshot(
    std::string* error_message) const {
  if (!IsModelLoaded()) {
    if (error_message != nullptr) {
      *error_message = "cannot create snapshot before loading a model";
    }
    return nullptr;
  }

  auto snapshot = std::unique_ptr<SimulationSnapshot>(
      new SimulationSnapshot());
  snapshot->impl_->owner = snapshot_identity_;
  snapshot->impl_->model_generation = model_generation_;
  snapshot->impl_->model = mj_copyModel(nullptr, model_.get());
  if (snapshot->impl_->model == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo model for visualization";
    }
    return nullptr;
  }
  snapshot->impl_->data =
      mj_copyData(nullptr, snapshot->impl_->model, data_.get());
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo data for visualization";
    }
    return nullptr;
  }
  return snapshot;
}

bool SimulationCore::UpdateSnapshot(SimulationSnapshot* snapshot,
                                    std::string* error_message) const {
  if (!IsModelLoaded() || snapshot == nullptr || snapshot->impl_ == nullptr ||
      snapshot->impl_->model == nullptr || snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "invalid visualization snapshot";
    }
    return false;
  }
  if (snapshot->impl_->owner.lock() != snapshot_identity_ ||
      snapshot->impl_->model_generation != model_generation_) {
    if (error_message != nullptr) {
      *error_message = "visualization snapshot does not belong to this model";
    }
    return false;
  }
  if (snapshot->impl_->model->nbuffer != model_->nbuffer) {
    if (error_message != nullptr) {
      *error_message = "visualization snapshot model is incompatible";
    }
    return false;
  }

  mj_copyData(snapshot->impl_->data, snapshot->impl_->model, data_.get());
  return true;
}

const std::vector<mujoco_sim::v1::ResourceDescriptor>&
SimulationCore::observations() const {
  return observations_;
}

const std::vector<mujoco_sim::v1::ResourceDescriptor>&
SimulationCore::commands() const {
  return commands_;
}

void SimulationCore::BuildCatalogs() {
  for (int sensor_id = 0; sensor_id < model_->nsensor; ++sensor_id) {
    const char* name = mj_id2name(model_.get(), mjOBJ_SENSOR, sensor_id);
    if (name == nullptr || name[0] == '\0') {
      continue;
    }

    mujoco_sim::v1::ResourceDescriptor descriptor;
    descriptor.set_name(name);
    descriptor.set_type(OBSERVATION_SENSOR);
    descriptor.set_dimension(
        static_cast<std::uint32_t>(model_->sensor_dim[sensor_id]));
    descriptor.set_mujoco_id(sensor_id);
    descriptor.set_mujoco_type(model_->sensor_type[sensor_id]);
    descriptor.set_mujoco_datatype(model_->sensor_datatype[sensor_id]);
    observations_.push_back(descriptor);
    observations_by_name_.emplace(descriptor.name(), descriptor);
  }

  for (int actuator_id = 0; actuator_id < model_->nu; ++actuator_id) {
    const char* name = mj_id2name(model_.get(), mjOBJ_ACTUATOR, actuator_id);
    if (name == nullptr || name[0] == '\0') {
      continue;
    }

    mujoco_sim::v1::ResourceDescriptor descriptor;
    descriptor.set_name(name);
    descriptor.set_type(COMMAND_ACTUATOR);
    descriptor.set_dimension(1);
    descriptor.set_mujoco_id(actuator_id);
    descriptor.set_mujoco_type(mjOBJ_ACTUATOR);
    descriptor.set_mujoco_datatype(0);
    const bool limited = model_->actuator_ctrllimited[actuator_id] != 0;
    descriptor.set_has_ctrl_range(limited);
    if (limited) {
      descriptor.set_ctrl_min(model_->actuator_ctrlrange[2 * actuator_id]);
      descriptor.set_ctrl_max(model_->actuator_ctrlrange[2 * actuator_id + 1]);
    }
    commands_.push_back(descriptor);
    commands_by_name_.emplace(descriptor.name(), descriptor);
  }
}

mujoco_sim::v1::GetAvailableResourcesResponse
SimulationCore::GetAvailableResources(
    const mujoco_sim::v1::GetAvailableResourcesRequest& request) const {
  mujoco_sim::v1::GetAvailableResourcesResponse response;
  if (!IsModelLoaded()) {
    response.set_code(FAILED_PRECONDITION);
    response.set_message("model is not loaded");
    return response;
  }
  if (!request.server_name().empty() &&
      request.server_name() != options_.server_name) {
    response.set_code(NOT_FOUND);
    response.set_message("server name mismatch");
    return response;
  }
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor) {
    response.set_code(VERSION_MISMATCH);
    response.set_message("protocol major mismatch");
    return response;
  }

  response.set_code(OK);
  response.set_epoch(epoch_);
  response.set_effective_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  response.set_effective_protocol_minor(
      std::min(request.protocol_minor(), mujoco_sim::protocol::kProtocolMinor));
  for (const auto& observation : observations_) {
    CopyDescriptor(observation, response.add_observations());
  }
  for (const auto& command : commands_) {
    CopyDescriptor(command, response.add_commands());
  }
  return response;
}

mujoco_sim::v1::RegisterControllerResponse
SimulationCore::RegisterController(
    const mujoco_sim::v1::RegisterControllerRequest& request) {
  if (!IsModelLoaded()) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        FAILED_PRECONDITION, "model is not loaded");
  }
  if (server_state_ == ERROR_PAUSED) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        FAILED_PRECONDITION, "server is error-paused");
  }
  if (tick_open_ || current_tick_id_ != 0) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        FAILED_PRECONDITION, "runtime controller registration is not supported");
  }
  if (request.server_name() != options_.server_name) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        NOT_FOUND, "server name mismatch");
  }
  if (request.controller_id().empty()) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        INVALID_ARGUMENT, "controller_id is required");
  }
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        VERSION_MISMATCH, "protocol major mismatch");
  }
  if (request.command_names().empty()) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        INVALID_ARGUMENT, "at least one command actuator is required");
  }
  if (controllers_.contains(request.controller_id())) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        ALREADY_EXISTS, "controller_id is already registered");
  }

  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate) ||
      HasDuplicates(request.command_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }

  Controller controller;
  controller.id = request.controller_id();
  controller.session_id = next_session_id_++;
  controller.epoch = epoch_;
  controller.timeout_ms = request.timeout_ms() == 0 ? options_.default_timeout_ms
                                                    : request.timeout_ms();
  controller.state = PROVISIONAL;

  for (const auto& name : request.observation_names()) {
    const auto it = observations_by_name_.find(name);
    if (it == observations_by_name_.end()) {
      return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
          NOT_FOUND, "unknown observation sensor: " + name);
    }
    CopyDescriptor(it->second, controller.layout.add_observations());
  }

  for (const auto& name : request.command_names()) {
    const auto command = commands_by_name_.find(name);
    if (command == commands_by_name_.end()) {
      return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
          NOT_FOUND, "unknown command actuator: " + name);
    }
    const auto owner = actuator_owner_.find(name);
    if (owner != actuator_owner_.end()) {
      return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
          RESOURCE_CONFLICT,
          "actuator already owned by controller: " + owner->second);
    }
    CopyDescriptor(command->second, controller.layout.add_commands());
  }

  for (const auto& command : controller.layout.commands()) {
    actuator_owner_.emplace(command.name(), controller.id);
  }

  mujoco_sim::v1::RegisterControllerResponse response;
  response.set_code(OK);
  response.set_controller_id(controller.id);
  response.set_session_id(controller.session_id);
  response.set_epoch(controller.epoch);
  response.set_effective_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  response.set_effective_protocol_minor(
      std::min(request.protocol_minor(), mujoco_sim::protocol::kProtocolMinor));
  response.set_state(PROVISIONAL);
  *response.mutable_layout() = controller.layout;

  controllers_.emplace(controller.id, std::move(controller));
  return response;
}

mujoco_sim::v1::ActivateControllerResponse
SimulationCore::ActivateController(
    const mujoco_sim::v1::ActivateControllerRequest& request) {
  const auto it = controllers_.find(request.controller_id());
  if (it == controllers_.end()) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        NOT_FOUND, "controller is not registered");
  }
  Controller& controller = it->second;
  if (request.session_id() != controller.session_id) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (controller.state != PROVISIONAL) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        FAILED_PRECONDITION, "controller is not provisional");
  }
  if (tick_open_ || current_tick_id_ != 0) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        FAILED_PRECONDITION, "runtime controller activation is not supported");
  }

  controller.state = ACTIVE;
  mujoco_sim::v1::ActivateControllerResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

mujoco_sim::v1::UnregisterControllerResponse
SimulationCore::UnregisterController(
    const mujoco_sim::v1::UnregisterControllerRequest& request) {
  const auto it = controllers_.find(request.controller_id());
  if (it == controllers_.end()) {
    return ErrorResponse<mujoco_sim::v1::UnregisterControllerResponse>(
        NOT_FOUND, "controller is not registered");
  }
  if (tick_open_) {
    return ErrorResponse<mujoco_sim::v1::UnregisterControllerResponse>(
        FAILED_PRECONDITION, "cannot unregister while a tick is open");
  }
  Controller& controller = it->second;
  if (request.session_id() != controller.session_id) {
    return ErrorResponse<mujoco_sim::v1::UnregisterControllerResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v1::UnregisterControllerResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  for (const auto& command : controller.layout.commands()) {
    actuator_owner_.erase(command.name());
  }
  pending_commits_.erase(controller.id);
  controllers_.erase(it);

  mujoco_sim::v1::UnregisterControllerResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return response;
}

std::vector<mujoco_sim::v1::Tick> SimulationCore::OpenTick() {
  std::vector<mujoco_sim::v1::Tick> ticks;
  if (!IsModelLoaded() || server_state_ == ERROR_PAUSED) {
    return ticks;
  }
  for (const auto& [_, controller] : controllers_) {
    if (controller.state == ACTIVE) {
      ticks.push_back(BuildTickFor(controller));
    }
  }
  if (ticks.empty()) {
    server_state_ = IDLE;
    tick_open_ = false;
    return ticks;
  }
  if (!tick_open_) {
    tick_started_at_ = std::chrono::steady_clock::now();
    pending_commits_.clear();
    tick_open_ = true;
  }
  server_state_ = RUNNING;
  return ticks;
}

mujoco_sim::v1::Tick SimulationCore::BuildTickFor(
    const Controller& controller) const {
  mujoco_sim::v1::Tick tick;
  *tick.mutable_header() = MakeHeader(controller.session_id);
  tick.set_controller_id(controller.id);
  tick.set_controller_state(controller.state);
  *tick.mutable_layout() = controller.layout;

  for (const auto& observation : controller.layout.observations()) {
    auto* block = tick.add_observations();
    block->set_name(observation.name());
    const int sensor_id = observation.mujoco_id();
    const int address = model_->sensor_adr[sensor_id];
    const int dimension = model_->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      block->add_values(static_cast<double>(data_->sensordata[address + i]));
    }
  }

  return tick;
}

mujoco_sim::v1::Header SimulationCore::MakeHeader(
    std::uint64_t session_id) const {
  mujoco_sim::v1::Header header;
  header.set_server_name(options_.server_name);
  header.set_session_id(session_id);
  header.set_epoch(epoch_);
  header.set_tick_id(current_tick_id_);
  header.set_timestamp_unix_nanos(NowUnixNanos());
  header.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  header.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  return header;
}

bool SimulationCore::ValidateCommonHeader(
    const mujoco_sim::v1::Header& header, const Controller& controller,
    std::string* message) const {
  if (header.server_name() != options_.server_name) {
    *message = "server name mismatch";
    return false;
  }
  if (header.session_id() != controller.session_id) {
    *message = "session mismatch";
    return false;
  }
  if (header.epoch() != epoch_) {
    *message = "epoch mismatch";
    return false;
  }
  if (header.tick_id() != current_tick_id_) {
    *message = "tick mismatch";
    return false;
  }
  return true;
}

CommitResult SimulationCore::SubmitCommit(
    const mujoco_sim::v1::AdapterCommit& commit) {
  CommitResult result;
  const auto controller_it = controllers_.find(commit.controller_id());
  if (controller_it != controllers_.end()) {
    const Controller& controller = controller_it->second;
    if (commit.header().server_name() == options_.server_name &&
        commit.header().session_id() == controller.session_id &&
        commit.header().epoch() == epoch_ &&
        commit.header().tick_id() < current_tick_id_) {
      result.duplicate = true;
      result.message = "stale duplicate commit ignored";
      return result;
    }
  }
  if (server_state_ == ERROR_PAUSED) {
    result.code = FAILED_PRECONDITION;
    result.message = "server is error-paused";
    return result;
  }
  if (controller_it != controllers_.end() &&
      controller_it->second.state == ACTIVE) {
    std::string message;
    if (!ValidateCommonHeader(commit.header(), controller_it->second,
                              &message)) {
      if (message == "session mismatch") {
        result.code = INVALID_SESSION;
      } else if (message == "epoch mismatch") {
        result.code = INVALID_EPOCH;
      } else if (message == "tick mismatch") {
        result.code = INVALID_TICK;
      } else {
        result.code = INVALID_ARGUMENT;
      }
      EnterErrorPaused(message);
      result.message = message;
      return result;
    }
  }
  if (!tick_open_) {
    result.code = FAILED_PRECONDITION;
    result.message = "no tick is open";
    return result;
  }
  if (controller_it == controllers_.end() ||
      controller_it->second.state != ACTIVE) {
    EnterErrorPaused("commit from unknown or inactive controller");
    result.code = NOT_FOUND;
    result.message = status_message_;
    return result;
  }
  const Controller& controller = controller_it->second;

  std::string message;
  if (!ValidateCommonHeader(commit.header(), controller, &message)) {
    if (message == "session mismatch") {
      result.code = INVALID_SESSION;
    } else if (message == "epoch mismatch") {
      result.code = INVALID_EPOCH;
    } else if (message == "tick mismatch") {
      result.code = INVALID_TICK;
    } else {
      result.code = INVALID_ARGUMENT;
    }
    EnterErrorPaused(message);
    result.message = message;
    return result;
  }

  const auto pending_it = pending_commits_.find(controller.id);
  if (pending_it != pending_commits_.end()) {
    if (!SameCommitPayload(commit, pending_it->second)) {
      EnterErrorPaused("duplicate commit payload mismatch");
      result.code = INVALID_COMMAND;
      result.message = status_message_;
      return result;
    }
    result.duplicate = true;
    result.message = "duplicate commit ignored";
    return result;
  }

  if (commit.status() != mujoco_sim::v1::COMMIT_OK) {
    EnterErrorPaused("controller reported commit error: " +
                     commit.error_message());
    result.code = INVALID_COMMAND;
    result.message = status_message_;
    return result;
  }

  std::set<std::string> expected;
  for (const auto& command : controller.layout.commands()) {
    expected.insert(command.name());
  }
  std::set<std::string> seen;
  for (const auto& command : commit.commands()) {
    if (!expected.contains(command.name())) {
      EnterErrorPaused("controller submitted unowned actuator: " +
                       command.name());
      result.code = INVALID_COMMAND;
      result.message = status_message_;
      return result;
    }
    if (!seen.insert(command.name()).second) {
      EnterErrorPaused("duplicate actuator command: " + command.name());
      result.code = INVALID_COMMAND;
      result.message = status_message_;
      return result;
    }
    if (command.values_size() != 1) {
      EnterErrorPaused("actuator command must contain exactly one value: " +
                       command.name());
      result.code = INVALID_COMMAND;
      result.message = status_message_;
      return result;
    }
    const double value = command.values(0);
    if (!std::isfinite(value)) {
      EnterErrorPaused("actuator command is not finite: " + command.name());
      result.code = INVALID_COMMAND;
      result.message = status_message_;
      return result;
    }
    const auto descriptor = commands_by_name_.at(command.name());
    if (descriptor.has_ctrl_range() &&
        (value < descriptor.ctrl_min() || value > descriptor.ctrl_max())) {
      std::ostringstream out;
      out << "actuator command is outside ctrl range: " << command.name();
      EnterErrorPaused(out.str());
      result.code = INVALID_COMMAND;
      result.message = status_message_;
      return result;
    }
  }
  if (seen != expected) {
    EnterErrorPaused("commit does not include all claimed actuators");
    result.code = INVALID_COMMAND;
    result.message = status_message_;
    return result;
  }

  pending_commits_.emplace(controller.id, commit);

  if (pending_commits_.size() != active_controller_count()) {
    result.message = "commit accepted";
    return result;
  }

  for (const auto& [_, pending] : pending_commits_) {
    for (const auto& command : pending.commands()) {
      const auto& descriptor = commands_by_name_.at(command.name());
      data_->ctrl[descriptor.mujoco_id()] = command.values(0);
    }
  }
  mj_step(model_.get(), data_.get());
  simulation_time_ = data_->time;
  current_tick_id_++;
  pending_commits_.clear();
  tick_open_ = false;
  result.stepped = true;
  result.message = "barrier complete";
  return result;
}

void SimulationCore::AcceptHeartbeat(
    const mujoco_sim::v1::Heartbeat& heartbeat) {
  const auto controller_it = controllers_.find(heartbeat.controller_id());
  if (controller_it == controllers_.end()) {
    return;
  }
  Controller& controller = controller_it->second;
  std::string ignored;
  if (ValidateCommonHeader(heartbeat.header(), controller, &ignored)) {
    controller.last_heartbeat_tick = current_tick_id_;
  }
}

bool SimulationCore::CheckTickTimeout(
    std::chrono::steady_clock::time_point now) {
  if (!tick_open_ || server_state_ == ERROR_PAUSED) {
    return false;
  }
  for (const auto& [controller_id, controller] : controllers_) {
    if (controller.state != ACTIVE || pending_commits_.contains(controller_id)) {
      continue;
    }
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - tick_started_at_)
            .count();
    if (elapsed_ms > controller.timeout_ms) {
      EnterErrorPaused("controller timed out: " + controller_id);
      return true;
    }
  }
  return false;
}

mujoco_sim::v1::ServerStatus SimulationCore::MakeStatus() const {
  mujoco_sim::v1::ServerStatus status;
  *status.mutable_header() = MakeHeader(0);
  status.set_state(server_state_);
  status.set_message(status_message_);
  for (const auto& [_, controller] : controllers_) {
    auto* controller_status = status.add_controllers();
    controller_status->set_controller_id(controller.id);
    controller_status->set_session_id(controller.session_id);
    controller_status->set_last_tick_id(controller.last_heartbeat_tick);
    controller_status->set_state(controller.state);
    controller_status->set_message(controller.message);
  }
  return status;
}

void SimulationCore::EnterErrorPaused(const std::string& message) {
  server_state_ = ERROR_PAUSED;
  status_message_ = message;
  tick_open_ = false;
  pending_commits_.clear();
  for (auto& [_, controller] : controllers_) {
    if (controller.state == ACTIVE) {
      controller.state = ERROR;
      controller.message = message;
    }
  }
}

std::uint64_t SimulationCore::epoch() const { return epoch_; }

std::uint64_t SimulationCore::current_tick_id() const {
  return current_tick_id_;
}

double SimulationCore::simulation_time() const { return simulation_time_; }

mujoco_sim::v1::ServerState SimulationCore::server_state() const {
  return server_state_;
}

bool SimulationCore::tick_open() const { return tick_open_; }

std::size_t SimulationCore::active_controller_count() const {
  std::size_t count = 0;
  for (const auto& [_, controller] : controllers_) {
    if (controller.state == ACTIVE) {
      ++count;
    }
  }
  return count;
}

}  // namespace mujoco_sim::core
