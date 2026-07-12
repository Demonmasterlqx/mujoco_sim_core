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
using mujoco_sim::v1::OBSERVER;
using mujoco_sim::v1::OK;
using mujoco_sim::v1::PROVISIONAL;
using mujoco_sim::v1::RESOURCE_CONFLICT;
using mujoco_sim::v1::RUNNING;
using mujoco_sim::v1::TIMEOUT;
using mujoco_sim::v1::TIMED_OUT;
using mujoco_sim::v1::UNREGISTERED;
using mujoco_sim::v1::VERSION_MISMATCH;

constexpr std::uint32_t kRoleAwareApiMinimumMinor = 2;

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

enum class SensorQuantity { kPosition, kVelocity, kForce };

std::string JointUnit(const mjModel* model, int joint_id,
                      SensorQuantity quantity) {
  if (joint_id < 0 || joint_id >= model->njnt) {
    return {};
  }
  switch (model->jnt_type[joint_id]) {
    case mjJNT_HINGE:
      switch (quantity) {
        case SensorQuantity::kPosition:
          return "rad";
        case SensorQuantity::kVelocity:
          return "rad/s";
        case SensorQuantity::kForce:
          return "N*m";
      }
      break;
    case mjJNT_SLIDE:
      switch (quantity) {
        case SensorQuantity::kPosition:
          return "m";
        case SensorQuantity::kVelocity:
          return "m/s";
        case SensorQuantity::kForce:
          return "N";
      }
      break;
    default:
      break;
  }
  return {};
}

std::string TendonUnit(const mjModel* model, int tendon_id,
                       SensorQuantity quantity) {
  if (tendon_id < 0 || tendon_id >= model->ntendon) {
    return {};
  }

  const int first_wrap = model->tendon_adr[tendon_id];
  const int wrap_count = model->tendon_num[tendon_id];
  std::string unit;
  for (int offset = 0; offset < wrap_count; ++offset) {
    const int wrap_id = first_wrap + offset;
    if (model->wrap_type[wrap_id] != mjWRAP_JOINT) {
      switch (quantity) {
        case SensorQuantity::kPosition:
          return "m";
        case SensorQuantity::kVelocity:
          return "m/s";
        case SensorQuantity::kForce:
          return "N";
      }
    }

    const std::string joint_unit =
        JointUnit(model, model->wrap_objid[wrap_id], quantity);
    if (joint_unit.empty() || (!unit.empty() && unit != joint_unit)) {
      return {};
    }
    unit = joint_unit;
  }
  return unit;
}

std::string ActuatorUnit(const mjModel* model, int actuator_id,
                         SensorQuantity quantity) {
  if (actuator_id < 0 || actuator_id >= model->nu) {
    return {};
  }
  switch (model->actuator_trntype[actuator_id]) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
      return JointUnit(model, model->actuator_trnid[2 * actuator_id],
                       quantity);
    case mjTRN_SLIDERCRANK:
      switch (quantity) {
        case SensorQuantity::kPosition:
          return "m";
        case SensorQuantity::kVelocity:
          return "m/s";
        case SensorQuantity::kForce:
          return "N";
      }
      break;
    case mjTRN_TENDON:
      return TendonUnit(model, model->actuator_trnid[2 * actuator_id],
                        quantity);
    default:
      // Site, body and plugin transmissions can mix linear and angular axes.
      break;
  }
  return {};
}

std::string SensorUnit(const mjModel* model, int sensor_id) {
  const int sensor_type = model->sensor_type[sensor_id];
  const int object_id = model->sensor_objid[sensor_id];
  switch (sensor_type) {
    case mjSENS_JOINTPOS:
    case mjSENS_JOINTLIMITPOS:
      return JointUnit(model, object_id, SensorQuantity::kPosition);
    case mjSENS_JOINTVEL:
    case mjSENS_JOINTLIMITVEL:
      return JointUnit(model, object_id, SensorQuantity::kVelocity);
    case mjSENS_JOINTACTFRC:
    case mjSENS_JOINTLIMITFRC:
      return JointUnit(model, object_id, SensorQuantity::kForce);
    case mjSENS_TENDONPOS:
    case mjSENS_TENDONLIMITPOS:
      return TendonUnit(model, object_id, SensorQuantity::kPosition);
    case mjSENS_TENDONVEL:
    case mjSENS_TENDONLIMITVEL:
      return TendonUnit(model, object_id, SensorQuantity::kVelocity);
    case mjSENS_TENDONACTFRC:
    case mjSENS_TENDONLIMITFRC:
      return TendonUnit(model, object_id, SensorQuantity::kForce);
    case mjSENS_ACTUATORPOS:
      return ActuatorUnit(model, object_id, SensorQuantity::kPosition);
    case mjSENS_ACTUATORVEL:
      return ActuatorUnit(model, object_id, SensorQuantity::kVelocity);
    case mjSENS_ACTUATORFRC:
      return ActuatorUnit(model, object_id, SensorQuantity::kForce);
    case mjSENS_BALLQUAT:
    case mjSENS_FRAMEQUAT:
    case mjSENS_FRAMEXAXIS:
    case mjSENS_FRAMEYAXIS:
    case mjSENS_FRAMEZAXIS:
    case mjSENS_GEOMNORMAL:
    case mjSENS_INSIDESITE:
      return "1";
    case mjSENS_BALLANGVEL:
    case mjSENS_GYRO:
    case mjSENS_FRAMEANGVEL:
      return "rad/s";
    case mjSENS_ACCELEROMETER:
    case mjSENS_FRAMELINACC:
      return "m/s^2";
    case mjSENS_FRAMEANGACC:
      return "rad/s^2";
    case mjSENS_VELOCIMETER:
    case mjSENS_FRAMELINVEL:
    case mjSENS_SUBTREELINVEL:
      return "m/s";
    case mjSENS_FORCE:
    case mjSENS_TOUCH:
      return "N";
    case mjSENS_TORQUE:
      return "N*m";
    case mjSENS_RANGEFINDER:
    case mjSENS_FRAMEPOS:
    case mjSENS_SUBTREECOM:
    case mjSENS_GEOMDIST:
      return "m";
    case mjSENS_GEOMFROMTO:
      return "m";
    case mjSENS_CAMPROJECTION:
      return "px";
    case mjSENS_MAGNETOMETER:
      return "T";
    case mjSENS_CLOCK:
      return "s";
    case mjSENS_E_POTENTIAL:
    case mjSENS_E_KINETIC:
      return "J";
    default:
      return {};
  }
}

std::string SensorInterpolation(const mjModel* model, int sensor_id) {
  if (model->sensor_history[2 * sensor_id] <= 0) {
    return {};
  }
  switch (model->sensor_history[2 * sensor_id + 1]) {
    case 1:
      return "linear";
    case 2:
      return "cubic";
    default:
      return "zoh";
  }
}

bool IsFrameSensor(int sensor_type) {
  switch (sensor_type) {
    case mjSENS_FRAMEPOS:
    case mjSENS_FRAMEQUAT:
    case mjSENS_FRAMEXAXIS:
    case mjSENS_FRAMEYAXIS:
    case mjSENS_FRAMEZAXIS:
    case mjSENS_FRAMELINVEL:
    case mjSENS_FRAMEANGVEL:
    case mjSENS_FRAMELINACC:
    case mjSENS_FRAMEANGACC:
      return true;
    default:
      return false;
  }
}

std::string ObjectName(const mjModel* model, int object_type, int object_id) {
  if (object_type == mjOBJ_UNKNOWN || object_id < 0) {
    return {};
  }
  const char* name =
      mj_id2name(model, static_cast<mjtObj>(object_type), object_id);
  return name == nullptr ? "" : name;
}

std::string SensorFrameId(const mjModel* model, int sensor_id) {
  const int sensor_type = model->sensor_type[sensor_id];
  if (IsFrameSensor(sensor_type)) {
    const int reference_id = model->sensor_refid[sensor_id];
    if (reference_id < 0) {
      return "world";
    }
    return ObjectName(model, model->sensor_reftype[sensor_id], reference_id);
  }

  switch (sensor_type) {
    case mjSENS_ACCELEROMETER:
    case mjSENS_VELOCIMETER:
    case mjSENS_GYRO:
    case mjSENS_FORCE:
    case mjSENS_TORQUE:
    case mjSENS_MAGNETOMETER:
      return ObjectName(model, model->sensor_objtype[sensor_id],
                        model->sensor_objid[sensor_id]);
    case mjSENS_GEOMNORMAL:
    case mjSENS_GEOMFROMTO:
    case mjSENS_SUBTREECOM:
    case mjSENS_SUBTREELINVEL:
    case mjSENS_SUBTREEANGMOM:
      return "world";
    case mjSENS_CAMPROJECTION:
      return ObjectName(model, model->sensor_reftype[sensor_id],
                        model->sensor_refid[sensor_id]);
    default:
      return {};
  }
}

}  // namespace

struct SimulationCore::Controller {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t timeout_ms = 1000;
  std::uint32_t protocol_minor = mujoco_sim::protocol::kProtocolMinor;
  mujoco_sim::v1::ControllerState state = PROVISIONAL;
  mujoco_sim::v1::ResourceLayout layout;
  std::chrono::steady_clock::time_point registered_at =
      std::chrono::steady_clock::now();
  std::uint64_t last_heartbeat_tick = 0;
  std::uint64_t last_commit_tick = 0;
  std::uint64_t last_tick_sent = 0;
  std::uint64_t state_since_tick = 0;
  std::uint64_t timed_out_since_tick = 0;
  std::uint64_t last_recovery_proof_tick = 0;
  std::uint64_t timeout_count = 0;
  bool has_commit = false;
  bool has_tick_sent = false;
  bool has_recovery_proof = false;
  bool recovery_pending = false;
  mujoco_sim::v1::AdapterCommit last_recovery_commit;
  mujoco_sim::v1::AdapterCommit last_commit;
  std::string message;
};

struct SimulationCore::Observer {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t timeout_ms = 1000;
  std::uint32_t protocol_minor = mujoco_sim::protocol::kProtocolMinor;
  mujoco_sim::v1::ControllerState state = PROVISIONAL;
  mujoco_sim::v1::ResourceLayout layout;
  mujoco_sim::v1::ObservationDelivery delivery;
  std::chrono::steady_clock::time_point registered_at =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_heartbeat_at = registered_at;
  std::uint64_t state_since_tick = 0;
  std::uint64_t activation_tick = 0;
  std::uint64_t timeout_count = 0;
  std::uint64_t sequence = 0;
  std::uint64_t last_delivery_tick = 0;
  std::uint64_t delivery_phase_tick = 0;
  std::uint64_t next_delivery_period = 1;
  bool has_delivery = false;
  std::string message;
};

struct SimulationSnapshot::Impl {
  ~Impl() {
    if (data != nullptr) {
      mj_deleteData(data);
    }
  }

  std::shared_ptr<mjModel> model;
  mjData* data = nullptr;
  std::weak_ptr<void> owner;
  std::uint64_t model_generation = 0;
};

SimulationSnapshot::SimulationSnapshot() : impl_(std::make_unique<Impl>()) {}
SimulationSnapshot::~SimulationSnapshot() = default;
SimulationSnapshot::SimulationSnapshot(SimulationSnapshot&&) noexcept = default;
SimulationSnapshot& SimulationSnapshot::operator=(
    SimulationSnapshot&&) noexcept = default;

const mjModel_* SimulationSnapshot::model() const {
  return impl_->model.get();
}
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
  snapshot_identity_ = std::make_shared<char>();
  ++model_generation_;

  observations_.clear();
  commands_.clear();
  observations_by_name_.clear();
  commands_by_name_.clear();
  controllers_.clear();
  observers_.clear();
  actuator_owner_.clear();
  pending_commits_.clear();
  open_ticks_.clear();
  tick_required_controllers_.clear();
  tick_recipient_controllers_.clear();
  pending_observation_frames_.clear();
  epoch_++;
  current_tick_id_ = 0;
  simulation_time_ = data_->time;
  server_state_ = RUNNING;
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
  snapshot->impl_->model = std::shared_ptr<mjModel>(
      mj_copyModel(nullptr, model_.get()),
      [](mjModel* model) { mj_deleteModel(model); });
  if (snapshot->impl_->model == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo model for visualization";
    }
    return nullptr;
  }
  snapshot->impl_->data =
      mj_copyData(nullptr, snapshot->impl_->model.get(), data_.get());
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo data for visualization";
    }
    return nullptr;
  }
  return snapshot;
}

std::unique_ptr<SimulationSnapshot> SimulationCore::CloneSnapshot(
    const SimulationSnapshot& source, std::string* error_message) const {
  if (!IsModelLoaded() || source.impl_ == nullptr ||
      source.impl_->model == nullptr || source.impl_->data == nullptr ||
      source.impl_->owner.lock() != snapshot_identity_ ||
      source.impl_->model_generation != model_generation_) {
    if (error_message != nullptr) {
      *error_message = "cannot clone an invalid or stale simulation snapshot";
    }
    return nullptr;
  }
  auto snapshot =
      std::unique_ptr<SimulationSnapshot>(new SimulationSnapshot());
  snapshot->impl_->owner = snapshot_identity_;
  snapshot->impl_->model_generation = model_generation_;
  snapshot->impl_->model = source.impl_->model;
  snapshot->impl_->data =
      mj_copyData(nullptr, snapshot->impl_->model.get(), source.impl_->data);
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to allocate cloned MuJoCo snapshot data";
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

  mj_copyData(snapshot->impl_->data, snapshot->impl_->model.get(), data_.get());
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
    descriptor.set_mujoco_stage(model_->sensor_needstage[sensor_id]);
    descriptor.set_interval_period_s(
        model_->sensor_interval[2 * sensor_id]);
    descriptor.set_interval_phase_s(
        model_->sensor_interval[2 * sensor_id + 1]);
    descriptor.set_delay_s(model_->sensor_delay[sensor_id]);
    descriptor.set_unit(SensorUnit(model_.get(), sensor_id));
    descriptor.set_object_type(model_->sensor_objtype[sensor_id]);
    descriptor.set_interpolation(SensorInterpolation(model_.get(), sensor_id));
    descriptor.set_frame_id(SensorFrameId(model_.get(), sensor_id));
    const int object_type = model_->sensor_objtype[sensor_id];
    const int object_id = model_->sensor_objid[sensor_id];
    if (object_type != mjOBJ_UNKNOWN && object_id >= 0) {
      const char* object_name = mj_id2name(
          model_.get(), static_cast<mjtObj>(object_type), object_id);
      if (object_name != nullptr) {
        descriptor.set_object_name(object_name);
      }
    }
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
  if (server_state_ == ERROR_PAUSED || server_state_ == mujoco_sim::v1::STOPPED) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        FAILED_PRECONDITION, "server is not accepting registrations");
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
  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate) ||
      HasDuplicates(request.command_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }

  if (observers_.contains(request.controller_id())) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        ALREADY_EXISTS, "adapter_id is already registered as an observer");
  }

  const auto existing = controllers_.find(request.controller_id());
  const bool replacing =
      existing != controllers_.end() && existing->second.state == TIMED_OUT &&
      options_.allow_timed_out_adapter_replacement;
  if (existing != controllers_.end() && !replacing) {
    return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
        ALREADY_EXISTS,
        existing->second.state == TIMED_OUT
            ? "timed-out controller replacement is disabled"
            : "controller_id is already registered");
  }

  Controller controller;
  controller.id = request.controller_id();
  controller.session_id = next_session_id_++;
  controller.epoch = epoch_;
  controller.timeout_ms = request.timeout_ms() == 0 ? options_.default_timeout_ms
                                                    : request.timeout_ms();
  controller.protocol_minor =
      std::min(request.protocol_minor(), mujoco_sim::protocol::kProtocolMinor);
  controller.state = PROVISIONAL;
  controller.registered_at = std::chrono::steady_clock::now();
  controller.state_since_tick = current_tick_id_;

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
    if (owner != actuator_owner_.end() &&
        (!replacing || owner->second != request.controller_id())) {
      return ErrorResponse<mujoco_sim::v1::RegisterControllerResponse>(
          RESOURCE_CONFLICT,
          "actuator already owned by controller: " + owner->second);
    }
    CopyDescriptor(command->second, controller.layout.add_commands());
  }

  if (replacing) {
    ZeroControllerCommands(existing->second);
    ReleaseControllerResources(existing->second);
    pending_commits_.erase(existing->first);
    open_ticks_.erase(existing->first);
    tick_required_controllers_.erase(existing->first);
    tick_recipient_controllers_.erase(existing->first);
    controllers_.erase(existing);
  }

  for (const auto& command : controller.layout.commands()) {
    actuator_owner_[command.name()] = controller.id;
  }

  mujoco_sim::v1::RegisterControllerResponse response;
  response.set_code(OK);
  response.set_controller_id(controller.id);
  response.set_session_id(controller.session_id);
  response.set_epoch(controller.epoch);
  response.set_effective_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  response.set_effective_protocol_minor(
      controller.protocol_minor);
  response.set_state(PROVISIONAL);
  if (replacing) {
    response.set_message("replaced timed-out controller session");
  }
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
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v1::STOPPED) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  if (controller.state != PROVISIONAL) {
    return ErrorResponse<mujoco_sim::v1::ActivateControllerResponse>(
        FAILED_PRECONDITION, "controller is not provisional");
  }
  SetControllerState(&controller, ACTIVE, {});
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
  Controller& controller = it->second;
  if (request.session_id() != controller.session_id) {
    return ErrorResponse<mujoco_sim::v1::UnregisterControllerResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v1::UnregisterControllerResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  ZeroControllerCommands(controller);
  ReleaseControllerResources(controller);
  pending_commits_.erase(controller.id);
  open_ticks_.erase(controller.id);
  tick_required_controllers_.erase(controller.id);
  tick_recipient_controllers_.erase(controller.id);
  controllers_.erase(it);
  StepIfBarrierComplete();

  mujoco_sim::v1::UnregisterControllerResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return response;
}

mujoco_sim::v1::RegisterAdapterResponse SimulationCore::RegisterAdapter(
    const mujoco_sim::v1::RegisterAdapterRequest& request) {
  if (request.role() != mujoco_sim::v1::CONTROLLER &&
      request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        VERSION_MISMATCH, "protocol major mismatch");
  }
  if (request.protocol_minor() < kRoleAwareApiMinimumMinor) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        VERSION_MISMATCH, "role-aware adapter API requires protocol minor 2");
  }
  if (request.role() == mujoco_sim::v1::CONTROLLER) {
    mujoco_sim::v1::RegisterControllerRequest controller_request;
    controller_request.set_server_name(request.server_name());
    controller_request.set_controller_id(request.adapter_id());
    controller_request.set_protocol_major(request.protocol_major());
    controller_request.set_protocol_minor(request.protocol_minor());
    controller_request.mutable_observation_names()->CopyFrom(
        request.observation_names());
    controller_request.mutable_command_names()->CopyFrom(
        request.command_names());
    controller_request.set_timeout_ms(request.timeout_ms());
    const auto controller_response = RegisterController(controller_request);
    mujoco_sim::v1::RegisterAdapterResponse response;
    response.set_code(controller_response.code());
    response.set_message(controller_response.message());
    response.set_adapter_id(controller_response.controller_id());
    response.set_role(mujoco_sim::v1::CONTROLLER);
    response.set_session_id(controller_response.session_id());
    response.set_epoch(controller_response.epoch());
    response.set_effective_protocol_major(
        controller_response.effective_protocol_major());
    response.set_effective_protocol_minor(
        controller_response.effective_protocol_minor());
    *response.mutable_layout() = controller_response.layout();
    response.set_state(controller_response.state());
    return response;
  }
  if (!IsModelLoaded()) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "model is not loaded");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v1::STOPPED) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting registrations");
  }
  if (request.server_name() != options_.server_name) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        NOT_FOUND, "server name mismatch");
  }
  if (request.adapter_id().empty()) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter_id is required");
  }
  if (request.observation_names().empty()) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "at least one observation sensor is required");
  }
  if (!request.command_names().empty()) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "observer adapters cannot claim actuators");
  }
  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }
  if (request.delivery().has_every_n_steps() &&
      request.delivery().every_n_steps() == 0) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "every_n_steps must be >= 1");
  }
  if (request.delivery().has_period_s() &&
      (!std::isfinite(request.delivery().period_s()) ||
       request.delivery().period_s() <= 0.0)) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "period_s must be finite and > 0");
  }
  if (controllers_.contains(request.adapter_id())) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        ALREADY_EXISTS, "adapter_id is already registered as a controller");
  }

  const auto existing = observers_.find(request.adapter_id());
  const bool replacing =
      existing != observers_.end() && existing->second.state == TIMED_OUT &&
      options_.allow_timed_out_adapter_replacement;
  if (existing != observers_.end() && !replacing) {
    return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
        ALREADY_EXISTS,
        existing->second.state == TIMED_OUT
            ? "timed-out observer replacement is disabled"
            : "adapter_id is already registered");
  }

  Observer observer;
  observer.id = request.adapter_id();
  observer.epoch = epoch_;
  observer.timeout_ms = request.timeout_ms() == 0
                            ? options_.default_timeout_ms
                            : request.timeout_ms();
  observer.protocol_minor =
      std::min(request.protocol_minor(), mujoco_sim::protocol::kProtocolMinor);
  observer.registered_at = std::chrono::steady_clock::now();
  observer.last_heartbeat_at = observer.registered_at;
  observer.state_since_tick = current_tick_id_;
  observer.delivery = request.delivery();
  if (observer.delivery.schedule_case() ==
      mujoco_sim::v1::ObservationDelivery::SCHEDULE_NOT_SET) {
    observer.delivery.set_every_n_steps(1);
  }
  for (const auto& name : request.observation_names()) {
    const auto found = observations_by_name_.find(name);
    if (found == observations_by_name_.end()) {
      return ErrorResponse<mujoco_sim::v1::RegisterAdapterResponse>(
          NOT_FOUND, "unknown observation sensor: " + name);
    }
    CopyDescriptor(found->second, observer.layout.add_observations());
  }
  observer.session_id = next_session_id_++;
  if (replacing) {
    observers_.erase(existing);
  }

  mujoco_sim::v1::RegisterAdapterResponse response;
  response.set_code(OK);
  response.set_adapter_id(observer.id);
  response.set_role(OBSERVER);
  response.set_session_id(observer.session_id);
  response.set_epoch(observer.epoch);
  response.set_effective_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  response.set_effective_protocol_minor(observer.protocol_minor);
  response.set_state(PROVISIONAL);
  *response.mutable_layout() = observer.layout;
  if (replacing) {
    response.set_message("replaced timed-out observer session");
  }
  observers_.emplace(observer.id, std::move(observer));
  return response;
}

mujoco_sim::v1::ActivateAdapterResponse SimulationCore::ActivateAdapter(
    const mujoco_sim::v1::ActivateAdapterRequest& request) {
  if (request.role() == mujoco_sim::v1::CONTROLLER) {
    const auto found = controllers_.find(request.adapter_id());
    if (found != controllers_.end() &&
        found->second.protocol_minor < kRoleAwareApiMinimumMinor) {
      return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
          VERSION_MISMATCH,
          "role-aware adapter API requires protocol minor 2");
    }
    mujoco_sim::v1::ActivateControllerRequest controller_request;
    controller_request.set_controller_id(request.adapter_id());
    controller_request.set_session_id(request.session_id());
    controller_request.set_epoch(request.epoch());
    const auto controller_response = ActivateController(controller_request);
    mujoco_sim::v1::ActivateAdapterResponse response;
    response.set_code(controller_response.code());
    response.set_message(controller_response.message());
    response.set_state(controller_response.state());
    return response;
  }
  if (request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  const auto found = observers_.find(request.adapter_id());
  if (found == observers_.end()) {
    return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
        NOT_FOUND, "observer is not registered");
  }
  Observer& observer = found->second;
  if (request.session_id() != observer.session_id) {
    return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (observer.state != PROVISIONAL) {
    return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "observer is not provisional");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v1::STOPPED) {
    return ErrorResponse<mujoco_sim::v1::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  observer.state = ACTIVE;
  observer.state_since_tick = current_tick_id_;
  observer.activation_tick = current_tick_id_;
  observer.last_heartbeat_at = std::chrono::steady_clock::now();
  observer.message.clear();
  mujoco_sim::v1::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

mujoco_sim::v1::UnregisterAdapterResponse SimulationCore::UnregisterAdapter(
    const mujoco_sim::v1::UnregisterAdapterRequest& request) {
  if (request.role() == mujoco_sim::v1::CONTROLLER) {
    const auto found = controllers_.find(request.adapter_id());
    if (found != controllers_.end() &&
        found->second.protocol_minor < kRoleAwareApiMinimumMinor) {
      return ErrorResponse<mujoco_sim::v1::UnregisterAdapterResponse>(
          VERSION_MISMATCH,
          "role-aware adapter API requires protocol minor 2");
    }
    mujoco_sim::v1::UnregisterControllerRequest controller_request;
    controller_request.set_controller_id(request.adapter_id());
    controller_request.set_session_id(request.session_id());
    controller_request.set_epoch(request.epoch());
    const auto controller_response = UnregisterController(controller_request);
    mujoco_sim::v1::UnregisterAdapterResponse response;
    response.set_code(controller_response.code());
    response.set_message(controller_response.message());
    response.set_state(controller_response.state());
    return response;
  }
  if (request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::v1::UnregisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  const auto found = observers_.find(request.adapter_id());
  if (found == observers_.end()) {
    return ErrorResponse<mujoco_sim::v1::UnregisterAdapterResponse>(
        NOT_FOUND, "observer is not registered");
  }
  if (request.session_id() != found->second.session_id) {
    return ErrorResponse<mujoco_sim::v1::UnregisterAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v1::UnregisterAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  observers_.erase(found);
  mujoco_sim::v1::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return response;
}

std::vector<mujoco_sim::v1::Tick> SimulationCore::OpenTick() {
  std::vector<mujoco_sim::v1::Tick> ticks;
  if (!IsModelLoaded() || server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v1::STOPPED) {
    return ticks;
  }
  if (!tick_open_) {
    pending_commits_.clear();
    open_ticks_.clear();
    tick_required_controllers_.clear();
    tick_recipient_controllers_.clear();
    for (auto& [controller_id, controller] : controllers_) {
      if (controller.recovery_pending) {
        controller.recovery_pending = false;
        SetControllerState(&controller, ACTIVE, {});
      }
      if (controller.state == ACTIVE) {
        tick_required_controllers_.emplace(controller_id, true);
        tick_recipient_controllers_.emplace(controller_id, true);
      } else if (controller.state == TIMED_OUT) {
        tick_recipient_controllers_.emplace(controller_id, true);
      }
    }
    for (const auto& [controller_id, _] : tick_recipient_controllers_) {
      auto& controller = controllers_.at(controller_id);
      controller.last_tick_sent = current_tick_id_;
      controller.has_tick_sent = true;
      open_ticks_.emplace(controller_id, BuildTickFor(controller));
    }
    tick_started_at_ = std::chrono::steady_clock::now();
    tick_open_ = true;
  }
  server_state_ = RUNNING;
  for (const auto& [controller_id, _] : tick_recipient_controllers_) {
    const auto it = controllers_.find(controller_id);
    if (it == controllers_.end() ||
        (it->second.state != ACTIVE && it->second.state != TIMED_OUT)) {
      continue;
    }
    const auto tick = open_ticks_.find(controller_id);
    if (tick != open_ticks_.end()) {
      ticks.push_back(tick->second);
    }
  }
  return ticks;
}

bool SimulationCore::IsTickIssued(const mujoco_sim::v1::Tick& tick) const {
  const auto issued = open_ticks_.find(tick.controller_id());
  return tick_open_ && issued != open_ticks_.end() &&
         issued->second.SerializeAsString() == tick.SerializeAsString();
}

mujoco_sim::v1::Tick SimulationCore::BuildTickFor(
    const Controller& controller) const {
  mujoco_sim::v1::Tick tick;
  *tick.mutable_header() = MakeHeader(controller.session_id);
  tick.set_controller_id(controller.id);
  const auto wire_state =
      controller.protocol_minor == 0 && controller.state == TIMED_OUT
          ? ACTIVE
          : controller.state;
  tick.set_controller_state(wire_state);
  *tick.mutable_layout() = controller.layout;
  tick.mutable_header()->set_protocol_minor(controller.protocol_minor);

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

std::vector<mujoco_sim::v1::ObservationFrame>
SimulationCore::CollectObservationFrames() {
  std::vector<mujoco_sim::v1::ObservationFrame> frames;
  frames.swap(pending_observation_frames_);
  return frames;
}

void SimulationCore::QueueObservationFramesForCompletedStep() {
  for (auto& [_, observer] : observers_) {
    if (observer.state != ACTIVE ||
        current_tick_id_ <= observer.activation_tick) {
      continue;
    }

    bool due = !observer.has_delivery;
    if (!due && observer.delivery.has_every_n_steps()) {
      due = current_tick_id_ - observer.last_delivery_tick >=
            observer.delivery.every_n_steps();
    } else if (!due && observer.delivery.has_period_s()) {
      const long double elapsed_steps = static_cast<long double>(
          current_tick_id_ - observer.delivery_phase_tick);
      const long double elapsed =
          elapsed_steps * static_cast<long double>(timestep());
      const long double deadline =
          static_cast<long double>(observer.next_delivery_period) *
          static_cast<long double>(observer.delivery.period_s());
      const long double tolerance =
          static_cast<long double>(std::numeric_limits<double>::epsilon()) *
          8.0L * std::max({1.0L, std::abs(elapsed), std::abs(deadline)});
      due = elapsed + tolerance >= deadline;
    }
    if (!due) {
      continue;
    }

    pending_observation_frames_.push_back(BuildObservationFrame(&observer));
    observer.last_delivery_tick = current_tick_id_;
    if (observer.delivery.has_period_s()) {
      const double period = observer.delivery.period_s();
      if (!observer.has_delivery) {
        observer.delivery_phase_tick = current_tick_id_;
        observer.next_delivery_period = 1;
      } else if (period <= timestep()) {
        observer.next_delivery_period =
            current_tick_id_ - observer.delivery_phase_tick + 1;
      } else {
        ++observer.next_delivery_period;
      }
    }
    observer.has_delivery = true;
  }
}

mujoco_sim::v1::ObservationFrame SimulationCore::BuildObservationFrame(
    Observer* observer) const {
  mujoco_sim::v1::ObservationFrame frame;
  *frame.mutable_header() = MakeHeader(observer->session_id);
  frame.mutable_header()->set_protocol_minor(observer->protocol_minor);
  frame.set_observer_id(observer->id);
  frame.set_sequence(++observer->sequence);
  for (const auto& observation : observer->layout.observations()) {
    auto* block = frame.add_observations();
    block->set_name(observation.name());
    const int sensor_id = observation.mujoco_id();
    const int address = model_->sensor_adr[sensor_id];
    const int dimension = model_->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      block->add_values(static_cast<double>(data_->sensordata[address + i]));
    }
  }
  return frame;
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
  header.set_simulation_time_s(simulation_time_);
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

bool SimulationCore::StepIfBarrierComplete() {
  if (!tick_open_ ||
      pending_commits_.size() != tick_required_controllers_.size()) {
    return false;
  }
  for (const auto& [controller_id, _] : tick_required_controllers_) {
    if (!pending_commits_.contains(controller_id)) {
      return false;
    }
  }
  for (const auto& [_, pending] : pending_commits_) {
    for (const auto& command : pending.commands()) {
      const auto& descriptor = commands_by_name_.at(command.name());
      data_->ctrl[descriptor.mujoco_id()] = command.values(0);
    }
  }
  mj_step(model_.get(), data_.get());
  simulation_time_ = data_->time;
  ++current_tick_id_;
  QueueObservationFramesForCompletedStep();
  pending_commits_.clear();
  open_ticks_.clear();
  tick_required_controllers_.clear();
  tick_recipient_controllers_.clear();
  tick_open_ = false;
  return true;
}

CommitResult SimulationCore::SubmitCommit(
    const mujoco_sim::v1::AdapterCommit& commit) {
  CommitResult result;
  if (server_state_ == ERROR_PAUSED) {
    result.code = FAILED_PRECONDITION;
    result.message = "server is error-paused";
    return result;
  }
  const auto controller_it = controllers_.find(commit.controller_id());
  if (controller_it == controllers_.end()) {
    result.code = NOT_FOUND;
    result.message = "controller is not registered";
    return result;
  }
  Controller& controller = controller_it->second;
  result.controller_state = controller.state;

  std::string message;
  if (commit.header().server_name() != options_.server_name) {
    result.code = INVALID_ARGUMENT;
    result.message = "server name mismatch";
    return result;
  }
  if (commit.header().session_id() != controller.session_id) {
    result.code = INVALID_SESSION;
    result.message = "session mismatch";
    return result;
  }
  if (commit.header().epoch() != epoch_) {
    result.code = INVALID_EPOCH;
    result.message = "epoch mismatch";
    return result;
  }
  if (commit.header().protocol_major() !=
      mujoco_sim::protocol::kProtocolMajor) {
    result.code = VERSION_MISMATCH;
    result.message = "protocol major mismatch";
    return result;
  }
  if (commit.header().protocol_minor() != controller.protocol_minor) {
    result.code = VERSION_MISMATCH;
    result.message = "protocol minor mismatch";
    return result;
  }

  if (controller.state == TIMED_OUT) {
    const std::uint64_t tick_id = commit.header().tick_id();
    if (controller.has_recovery_proof &&
        tick_id == controller.last_recovery_proof_tick) {
      if (SameCommitPayload(commit, controller.last_recovery_commit)) {
        result.duplicate = true;
        result.message = "duplicate recovery commit ignored";
        return result;
      }
      result.stepped = EnterControllerError(
          &controller, "duplicate recovery commit payload mismatch");
      result.code = INVALID_COMMAND;
      result.message = controller.message;
      result.controller_state_changed = true;
      result.controller_state = ERROR;
      return result;
    }
    if (tick_id < controller.timed_out_since_tick ||
        tick_id > controller.last_tick_sent ||
        (controller.has_recovery_proof &&
         tick_id <= controller.last_recovery_proof_tick)) {
      result.code = INVALID_TICK;
      result.message = "commit does not match a new Tick issued for recovery";
      return result;
    }
    if (commit.status() != mujoco_sim::v1::COMMIT_OK) {
      result.code = INVALID_COMMAND;
      result.message = "timed-out controller did not report a valid commit";
      return result;
    }
    if (!ValidateCommands(controller, commit, &message)) {
      result.code = INVALID_COMMAND;
      result.message = message;
      return result;
    }
    controller.last_commit_tick = tick_id;
    controller.has_commit = true;
    controller.last_recovery_proof_tick = tick_id;
    controller.has_recovery_proof = true;
    controller.recovery_pending = true;
    controller.last_recovery_commit = commit;
    controller.last_commit = commit;
    controller.message = "valid recovery commit received; activation pending";
    result.recovery_pending = true;
    result.message = controller.message;
    return result;
  }

  if (controller.state != ACTIVE) {
    result.code = FAILED_PRECONDITION;
    result.message = "controller is not active";
    return result;
  }
  if (commit.header().tick_id() < current_tick_id_) {
    if (!controller.has_commit ||
        commit.header().tick_id() != controller.last_commit_tick) {
      result.code = INVALID_TICK;
      result.message = "stale commit cannot be verified";
      return result;
    }
    if (!SameCommitPayload(commit, controller.last_commit)) {
      result.stepped = EnterControllerError(
          &controller, "stale duplicate commit payload mismatch");
      result.code = INVALID_COMMAND;
      result.message = controller.message;
      result.controller_state_changed = true;
      result.controller_state = ERROR;
      return result;
    }
    result.duplicate = true;
    result.message = "stale duplicate commit ignored";
    return result;
  }
  if (commit.header().tick_id() > current_tick_id_) {
    result.code = INVALID_TICK;
    result.message = "tick mismatch";
    return result;
  }
  if (!tick_open_) {
    result.code = FAILED_PRECONDITION;
    result.message = "no tick is open";
    return result;
  }
  if (!tick_required_controllers_.contains(controller.id)) {
    result.code = FAILED_PRECONDITION;
    result.message = "controller joins the barrier on the next Tick";
    return result;
  }
  if (!ValidateCommonHeader(commit.header(), controller, &message)) {
    result.code = message == "tick mismatch" ? INVALID_TICK : INVALID_ARGUMENT;
    result.message = message;
    return result;
  }

  const auto pending_it = pending_commits_.find(controller.id);
  if (pending_it != pending_commits_.end()) {
    if (!SameCommitPayload(commit, pending_it->second)) {
      result.stepped = EnterControllerError(
          &controller, "duplicate commit payload mismatch");
      result.code = INVALID_COMMAND;
      result.message = controller.message;
      result.controller_state_changed = true;
      result.controller_state = ERROR;
      return result;
    }
    result.duplicate = true;
    result.message = "duplicate commit ignored";
    return result;
  }

  if (commit.status() != mujoco_sim::v1::COMMIT_OK) {
    result.stepped = EnterControllerError(
        &controller,
        "controller reported commit error: " + commit.error_message());
    result.code = INVALID_COMMAND;
    result.message = controller.message;
    result.controller_state_changed = true;
    result.controller_state = ERROR;
    return result;
  }

  if (!ValidateCommands(controller, commit, &message)) {
    result.stepped = EnterControllerError(&controller, message);
    result.code = INVALID_COMMAND;
    result.message = controller.message;
    result.controller_state_changed = true;
    result.controller_state = ERROR;
    return result;
  }

  pending_commits_.emplace(controller.id, commit);
  controller.last_commit_tick = current_tick_id_;
  controller.has_commit = true;
  controller.last_commit = commit;
  if (pending_commits_.size() != tick_required_controllers_.size()) {
    result.message = "commit accepted";
    return result;
  }
  result.stepped = StepIfBarrierComplete();
  result.message = result.stepped ? "barrier complete" : "commit accepted";
  return result;
}

void SimulationCore::AcceptHeartbeat(
    const mujoco_sim::v1::Heartbeat& heartbeat) {
  const auto controller_it = controllers_.find(heartbeat.controller_id());
  if (controller_it == controllers_.end()) {
    return;
  }
  Controller& controller = controller_it->second;
  const auto& header = heartbeat.header();
  if (header.server_name() == options_.server_name &&
      header.session_id() == controller.session_id &&
      header.epoch() == epoch_ &&
      header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
      header.protocol_minor() == controller.protocol_minor &&
      header.tick_id() <= controller.last_tick_sent) {
    controller.last_heartbeat_tick =
        std::max(controller.last_heartbeat_tick, header.tick_id());
  }
}

void SimulationCore::AcceptAdapterHeartbeat(
    const mujoco_sim::v1::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  if (heartbeat.role() == mujoco_sim::v1::CONTROLLER) {
    if (heartbeat.header().protocol_minor() < kRoleAwareApiMinimumMinor) {
      return;
    }
    mujoco_sim::v1::Heartbeat controller_heartbeat;
    *controller_heartbeat.mutable_header() = heartbeat.header();
    controller_heartbeat.set_controller_id(heartbeat.adapter_id());
    AcceptHeartbeat(controller_heartbeat);
    return;
  }
  if (heartbeat.role() != OBSERVER) {
    return;
  }
  const auto found = observers_.find(heartbeat.adapter_id());
  if (found == observers_.end()) {
    return;
  }
  Observer& observer = found->second;
  const auto& header = heartbeat.header();
  if (observer.state == ACTIVE &&
      header.server_name() == options_.server_name &&
      header.session_id() == observer.session_id && header.epoch() == epoch_ &&
      header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
      header.protocol_minor() == observer.protocol_minor &&
      heartbeat.last_observation_sequence() <= observer.sequence) {
    observer.last_heartbeat_at = now;
  }
}

DeadlineResult SimulationCore::CheckControllerDeadlines(
    std::chrono::steady_clock::time_point now) {
  DeadlineResult result;
  for (auto it = controllers_.begin(); it != controllers_.end();) {
    Controller& controller = it->second;
    if (controller.state == PROVISIONAL &&
        now - controller.registered_at >
            std::chrono::milliseconds(options_.activation_timeout_ms)) {
      result.transitions.push_back(
          ControllerTransition{controller.id, controller.session_id,
                               PROVISIONAL, UNREGISTERED,
                               "controller activation timed out",
                               current_tick_id_});
      ReleaseControllerResources(controller);
      it = controllers_.erase(it);
      continue;
    }
    ++it;
  }

  for (auto it = observers_.begin(); it != observers_.end();) {
    Observer& observer = it->second;
    if (observer.state == PROVISIONAL &&
        now - observer.registered_at >
            std::chrono::milliseconds(options_.activation_timeout_ms)) {
      result.adapter_transitions.push_back(DeadlineResult::AdapterTransition{
          observer.id, OBSERVER, observer.session_id, PROVISIONAL,
          UNREGISTERED, "observer activation timed out", current_tick_id_});
      it = observers_.erase(it);
      continue;
    }
    if (observer.state == ACTIVE &&
        now - observer.last_heartbeat_at >
            std::chrono::milliseconds(observer.timeout_ms)) {
      const auto old_state = observer.state;
      ++observer.timeout_count;
      observer.state = TIMED_OUT;
      observer.state_since_tick = current_tick_id_;
      observer.message = "observer keepalive timed out";
      result.adapter_transitions.push_back(DeadlineResult::AdapterTransition{
          observer.id, OBSERVER, observer.session_id, old_state, TIMED_OUT,
          observer.message, current_tick_id_});
    }
    ++it;
  }

  if (tick_open_ && server_state_ != ERROR_PAUSED) {
    for (auto& [controller_id, controller] : controllers_) {
      if (!tick_required_controllers_.contains(controller_id) ||
          pending_commits_.contains(controller_id)) {
        continue;
      }
      const auto elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now - tick_started_at_)
              .count();
      if (elapsed_ms <= controller.timeout_ms) {
        continue;
      }
      const auto old_state = controller.state;
      ++controller.timeout_count;
      controller.timed_out_since_tick = current_tick_id_;
      controller.has_recovery_proof = false;
      controller.recovery_pending = false;
      controller.last_recovery_commit.Clear();
      SetControllerState(&controller, TIMED_OUT,
                         "controller timed out waiting for Commit");
      ZeroControllerCommands(controller);
      pending_commits_.erase(controller_id);
      tick_required_controllers_.erase(controller_id);
      result.transitions.push_back(
          ControllerTransition{controller.id, controller.session_id, old_state,
                               TIMED_OUT, controller.message,
                               current_tick_id_});
    }
    result.stepped = StepIfBarrierComplete();
  }
  return result;
}

bool SimulationCore::CheckTickTimeout(
    std::chrono::steady_clock::time_point now) {
  const auto result = CheckControllerDeadlines(now);
  return std::any_of(result.transitions.begin(), result.transitions.end(),
                     [](const auto& transition) {
                       return transition.new_state == TIMED_OUT;
                     });
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
    if (controller.has_commit) {
      controller_status->set_last_commit_tick_id(controller.last_commit_tick);
    }
    controller_status->set_state_since_tick_id(controller.state_since_tick);
    controller_status->set_timeout_count(controller.timeout_count);
    controller_status->set_required_for_barrier(
        tick_required_controllers_.contains(controller.id));

    auto* adapter_status = status.add_adapters();
    adapter_status->set_adapter_id(controller.id);
    adapter_status->set_role(mujoco_sim::v1::CONTROLLER);
    adapter_status->set_session_id(controller.session_id);
    adapter_status->set_state(controller.state);
    adapter_status->set_message(controller.message);
    adapter_status->set_state_since_tick_id(controller.state_since_tick);
    adapter_status->set_timeout_count(controller.timeout_count);
    if (controller.has_tick_sent) {
      adapter_status->set_last_delivery_tick_id(controller.last_tick_sent);
    }
  }
  for (const auto& [_, observer] : observers_) {
    auto* adapter_status = status.add_adapters();
    adapter_status->set_adapter_id(observer.id);
    adapter_status->set_role(OBSERVER);
    adapter_status->set_session_id(observer.session_id);
    adapter_status->set_state(observer.state);
    adapter_status->set_message(observer.message);
    adapter_status->set_state_since_tick_id(observer.state_since_tick);
    adapter_status->set_timeout_count(observer.timeout_count);
    if (observer.has_delivery) {
      adapter_status->set_last_delivery_tick_id(observer.last_delivery_tick);
    }
    adapter_status->set_last_observation_sequence(observer.sequence);
  }
  return status;
}

bool SimulationCore::ValidateCommands(
    const Controller& controller,
    const mujoco_sim::v1::AdapterCommit& commit,
    std::string* message) const {
  std::set<std::string> expected;
  for (const auto& command : controller.layout.commands()) {
    expected.insert(command.name());
  }
  std::set<std::string> seen;
  for (const auto& command : commit.commands()) {
    if (!expected.contains(command.name())) {
      *message = "controller submitted unowned actuator: " + command.name();
      return false;
    }
    if (!seen.insert(command.name()).second) {
      *message = "duplicate actuator command: " + command.name();
      return false;
    }
    if (command.values_size() != 1) {
      *message = "actuator command must contain exactly one value: " +
                 command.name();
      return false;
    }
    const double value = command.values(0);
    if (!std::isfinite(value)) {
      *message = "actuator command is not finite: " + command.name();
      return false;
    }
    const auto descriptor = commands_by_name_.at(command.name());
    if (descriptor.has_ctrl_range() &&
        (value < descriptor.ctrl_min() || value > descriptor.ctrl_max())) {
      *message = "actuator command is outside ctrl range: " + command.name();
      return false;
    }
  }
  if (seen != expected) {
    *message = "commit does not include all claimed actuators";
    return false;
  }
  return true;
}

void SimulationCore::ZeroControllerCommands(const Controller& controller) {
  if (data_ == nullptr) {
    return;
  }
  for (const auto& command : controller.layout.commands()) {
    data_->ctrl[command.mujoco_id()] = 0.0;
  }
}

void SimulationCore::SetControllerState(
    Controller* controller, mujoco_sim::v1::ControllerState state,
    const std::string& message) {
  controller->state = state;
  controller->state_since_tick = current_tick_id_;
  controller->message = message;
}

bool SimulationCore::EnterControllerError(Controller* controller,
                                          const std::string& message) {
  SetControllerState(controller, ERROR, message);
  controller->recovery_pending = false;
  ZeroControllerCommands(*controller);
  pending_commits_.erase(controller->id);
  open_ticks_.erase(controller->id);
  tick_required_controllers_.erase(controller->id);
  tick_recipient_controllers_.erase(controller->id);
  return StepIfBarrierComplete();
}

void SimulationCore::ReleaseControllerResources(
    const Controller& controller) {
  for (const auto& command : controller.layout.commands()) {
    const auto owner = actuator_owner_.find(command.name());
    if (owner != actuator_owner_.end() && owner->second == controller.id) {
      actuator_owner_.erase(owner);
    }
  }
}

void SimulationCore::EnterErrorPaused(const std::string& message) {
  server_state_ = ERROR_PAUSED;
  status_message_ = message;
  tick_open_ = false;
  pending_commits_.clear();
  open_ticks_.clear();
  tick_required_controllers_.clear();
  tick_recipient_controllers_.clear();
  for (auto& [_, controller] : controllers_) {
    if (controller.state == ACTIVE) {
      SetControllerState(&controller, ERROR, message);
    }
  }
  for (auto& [_, observer] : observers_) {
    if (observer.state == ACTIVE) {
      observer.state = ERROR;
      observer.state_since_tick = current_tick_id_;
      observer.message = message;
    }
  }
}

void SimulationCore::EnterFatalError(const std::string& message) {
  EnterErrorPaused(message);
}

void SimulationCore::MarkStopped(const std::string& message) {
  server_state_ = mujoco_sim::v1::STOPPED;
  status_message_ = message;
  tick_open_ = false;
  pending_commits_.clear();
  open_ticks_.clear();
  tick_required_controllers_.clear();
  tick_recipient_controllers_.clear();
}

std::uint64_t SimulationCore::epoch() const { return epoch_; }

std::uint64_t SimulationCore::current_tick_id() const {
  return current_tick_id_;
}

double SimulationCore::simulation_time() const { return simulation_time_; }

double SimulationCore::timestep() const {
  return model_ == nullptr ? 0.0 : model_->opt.timestep;
}

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

std::size_t SimulationCore::controller_count(
    mujoco_sim::v1::ControllerState state) const {
  return static_cast<std::size_t>(std::count_if(
      controllers_.begin(), controllers_.end(),
      [state](const auto& entry) { return entry.second.state == state; }));
}

std::size_t SimulationCore::observer_count(
    mujoco_sim::v1::ControllerState state) const {
  return static_cast<std::size_t>(std::count_if(
      observers_.begin(), observers_.end(),
      [state](const auto& entry) { return entry.second.state == state; }));
}

}  // namespace mujoco_sim::core
