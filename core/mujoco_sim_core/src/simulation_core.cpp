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
#include <mujoco_sim/protocol/identifier.hpp>
#include <mujoco_sim/protocol/layout_hash.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v2::ACTIVE;
using mujoco_sim::v2::ALREADY_EXISTS;
using mujoco_sim::v2::COMMAND_ACTUATOR;
using mujoco_sim::v2::ERROR;
using mujoco_sim::v2::ERROR_PAUSED;
using mujoco_sim::v2::FAILED_PRECONDITION;
using mujoco_sim::v2::IDLE;
using mujoco_sim::v2::INVALID_ARGUMENT;
using mujoco_sim::v2::INVALID_COMMAND;
using mujoco_sim::v2::INVALID_EPOCH;
using mujoco_sim::v2::INVALID_SESSION;
using mujoco_sim::v2::INVALID_TICK;
using mujoco_sim::v2::NOT_FOUND;
using mujoco_sim::v2::OBSERVATION_SENSOR;
using mujoco_sim::v2::OBSERVER;
using mujoco_sim::v2::OK;
using mujoco_sim::v2::PROVISIONAL;
using mujoco_sim::v2::RESOURCE_CONFLICT;
using mujoco_sim::v2::RUNNING;
using mujoco_sim::v2::TIMEOUT;
using mujoco_sim::v2::TIMED_OUT;
using mujoco_sim::v2::UNREGISTERED;
using mujoco_sim::v2::VERSION_MISMATCH;

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
        mujoco_sim::v2::ActuatorCommandBlock>& lhs,
    const google::protobuf::RepeatedPtrField<
        mujoco_sim::v2::ActuatorCommandBlock>& rhs) {
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

bool SameCommitPayload(const mujoco_sim::v2::AdapterCommit& lhs,
                       const mujoco_sim::v2::AdapterCommit& rhs) {
  return lhs.status() == rhs.status() &&
         lhs.error_message() == rhs.error_message() &&
         SameCommandPayload(lhs.commands(), rhs.commands());
}

void CopyDescriptor(const mujoco_sim::v2::ResourceDescriptor& from,
                    mujoco_sim::v2::ResourceDescriptor* to) {
  *to = from;
}

template <typename Response>
Response ErrorResponse(mujoco_sim::v2::ErrorCode code,
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

namespace detail {

struct ModelDeleter {
  void operator()(mjModel_* model) const {
    if (model != nullptr) mj_deleteModel(model);
  }
};

struct DataDeleter {
  void operator()(mjData_* data) const {
    if (data != nullptr) mj_deleteData(data);
  }
};

struct Controller {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t timeout_ms = 1000;
  std::uint32_t protocol_minor = mujoco_sim::protocol::kProtocolMinor;
  mujoco_sim::v2::ControllerState state = PROVISIONAL;
  mujoco_sim::v2::ResourceLayout layout;
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
  mujoco_sim::v2::AdapterCommit last_recovery_commit;
  mujoco_sim::v2::AdapterCommit last_commit;
  std::string message;
};

struct Observer {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t timeout_ms = 1000;
  std::uint32_t protocol_minor = mujoco_sim::protocol::kProtocolMinor;
  mujoco_sim::v2::ControllerState state = PROVISIONAL;
  mujoco_sim::v2::ResourceLayout layout;
  mujoco_sim::v2::ObservationDelivery delivery;
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

struct ValidatedActuatorWrite {
  int actuator_id = -1;
  mjtNum value = 0.0;
};

class ResourceCatalog;

class AdapterRegistry {
 public:
  std::uint64_t AllocateSession() noexcept { return next_session_id++; }
  void ClearAdapters() {
    controllers.clear();
    observers.clear();
    actuator_owner.clear();
  }
  std::unordered_map<std::string, Controller> controllers;
  std::unordered_map<std::string, Observer> observers;
  std::unordered_map<std::string, std::string> actuator_owner;
  std::uint64_t next_session_id = 1;
};

class BarrierCoordinator {
 public:
  void Reset() {
    pending_commits.clear();
    required_controllers.clear();
    recipient_controllers.clear();
    pending_zero_actuators.clear();
  }
  bool Ready() const {
    if (pending_commits.size() != required_controllers.size()) return false;
    return std::all_of(required_controllers.begin(), required_controllers.end(),
                       [this](const auto& item) {
                         return pending_commits.contains(item.first);
                       });
  }
  std::vector<ValidatedActuatorWrite> BuildWritePlan(
      const ResourceCatalog& catalog) const;
  std::unordered_map<std::string, mujoco_sim::v2::AdapterCommit>
      pending_commits;
  std::unordered_map<std::string, bool> required_controllers;
  std::unordered_map<std::string, bool> recipient_controllers;
  std::set<int> pending_zero_actuators;
};

class TickStore {
 public:
  IssuedTickHandle Insert(const std::string& adapter_id,
                          mujoco_sim::v2::Tick message,
                          std::size_t layout_bytes_avoided,
                          std::uint64_t session_id, std::uint64_t epoch,
                          std::uint64_t tick_id) {
    auto issued_tick = std::make_shared<IssuedTick>();
    issued_tick->message = std::move(message);
    if (!issued_tick->message.SerializeToString(
            &issued_tick->serialized_bytes)) {
      return {};
    }
    issued_tick->serialization_count = 1;
    issued_tick->layout_bytes_avoided = layout_bytes_avoided;
    issued_tick->identity = {adapter_id, session_id, epoch, tick_id,
                             next_issue_generation++};
    issued.emplace(adapter_id, issued_tick);
    return issued_tick;
  }

  IssuedTickHandle Find(const std::string& adapter_id) const {
    const auto found = issued.find(adapter_id);
    return found == issued.end() ? IssuedTickHandle{} : found->second;
  }

  bool IsCurrent(const IssuedTickIdentity& identity) const {
    const auto found = issued.find(identity.adapter_id);
    return found != issued.end() &&
           found->second->current.load(std::memory_order_acquire) &&
           found->second->identity.session_id == identity.session_id &&
           found->second->identity.epoch == identity.epoch &&
           found->second->identity.tick_id == identity.tick_id &&
           found->second->identity.issue_generation ==
               identity.issue_generation;
  }

  void Invalidate(const std::string& adapter_id) {
    const auto found = issued.find(adapter_id);
    if (found == issued.end()) return;
    found->second->current.store(false, std::memory_order_release);
    issued.erase(found);
  }

  void InvalidateAll() {
    for (const auto& [_, tick] : issued) {
      tick->current.store(false, std::memory_order_release);
    }
    issued.clear();
  }

  std::unordered_map<std::string, IssuedTickHandle> issued;
  std::uint64_t next_issue_generation = 1;
};

class ModelState {
 public:
  struct PreparedModel {
    std::unique_ptr<mjModel_, ModelDeleter> model;
    std::unique_ptr<mjData_, DataDeleter> data;
  };

  static std::optional<PreparedModel> Prepare(
      const std::string& model_path, const ModelLoadOptions& options,
      std::string* error_message) {
    char load_error[1024] = {};
    PreparedModel prepared;
    prepared.model.reset(mj_loadXML(model_path.c_str(), nullptr, load_error,
                                    sizeof(load_error)));
    if (!prepared.model) {
      if (error_message != nullptr) {
        *error_message = load_error[0] == '\0' ? "failed to load MuJoCo model"
                                               : load_error;
      }
      return std::nullopt;
    }
    prepared.data.reset(mj_makeData(prepared.model.get()));
    if (!prepared.data) {
      if (error_message != nullptr) {
        *error_message = "failed to allocate MuJoCo data";
      }
      return std::nullopt;
    }
    if (options.initial_keyframe.empty()) {
      mj_resetData(prepared.model.get(), prepared.data.get());
    } else {
      const int keyframe_id = mj_name2id(prepared.model.get(), mjOBJ_KEY,
                                        options.initial_keyframe.c_str());
      if (keyframe_id < 0) {
        if (error_message != nullptr) {
          *error_message = "initial keyframe '" + options.initial_keyframe +
                           "' was not found in MuJoCo model";
        }
        return std::nullopt;
      }
      mj_resetDataKeyframe(prepared.model.get(), prepared.data.get(),
                           keyframe_id);
    }
    if (prepared.model->nu > 0) {
      mju_zero(prepared.data->ctrl, prepared.model->nu);
    }
    mj_forward(prepared.model.get(), prepared.data.get());
    return prepared;
  }

  void Commit(PreparedModel prepared) {
    model = std::move(prepared.model);
    data = std::move(prepared.data);
    simulation_time = data->time;
    ++generation;
  }

  bool loaded() const noexcept { return model != nullptr && data != nullptr; }
  void ApplyActuatorWrites(
      const std::vector<ValidatedActuatorWrite>& writes) {
    if (data == nullptr) return;
    for (const auto& write : writes) {
      data->ctrl[write.actuator_id] = write.value;
    }
  }

  void ZeroActuators(const std::vector<int>& actuator_ids) {
    if (data == nullptr) return;
    for (const int actuator_id : actuator_ids) {
      data->ctrl[actuator_id] = 0.0;
    }
  }

  void ZeroAllControls() {
    if (loaded() && model->nu > 0) mju_zero(data->ctrl, model->nu);
  }

  void Step() {
    mj_step(model.get(), data.get());
    simulation_time = data->time;
  }

  std::unique_ptr<mjModel_, ModelDeleter> model;
  std::unique_ptr<mjData_, DataDeleter> data;
  double simulation_time = 0.0;
  std::uint64_t generation = 0;
};

class ResourceCatalog {
 public:
  void Build(const mjModel* model);
  std::vector<mujoco_sim::v2::ResourceDescriptor> observations;
  std::vector<mujoco_sim::v2::ResourceDescriptor> commands;
  std::unordered_map<std::string, mujoco_sim::v2::ResourceDescriptor>
      observations_by_name;
  std::unordered_map<std::string, mujoco_sim::v2::ResourceDescriptor>
      commands_by_name;
};

std::vector<ValidatedActuatorWrite> BarrierCoordinator::BuildWritePlan(
    const ResourceCatalog& catalog) const {
  std::vector<ValidatedActuatorWrite> writes;
  for (const auto& [_, pending] : pending_commits) {
    for (const auto& command : pending.commands()) {
      const auto& descriptor = catalog.commands_by_name.at(command.name());
      writes.push_back({descriptor.mujoco_id(), command.values(0)});
    }
  }
  for (const int actuator_id : pending_zero_actuators) {
    writes.push_back({actuator_id, 0.0});
  }
  return writes;
}

class SnapshotManager {
 public:
  void Reset() {
    identity = std::make_shared<char>();
    ++model_generation;
  }
  std::unique_ptr<SimulationSnapshot> Create(
      const ModelState& model_state, std::string* error_message) const;
  std::unique_ptr<SimulationSnapshot> Clone(
      const ModelState& model_state, const SimulationSnapshot& source,
      std::string* error_message) const;
  bool Update(const ModelState& model_state, SimulationSnapshot* snapshot,
              std::string* error_message) const;
  std::shared_ptr<void> identity = std::make_shared<char>();
  std::uint64_t model_generation = 0;
};

class ObservationScheduler {
 public:
  bool Due(const Observer& observer, std::uint64_t tick_id,
           double timestep) const {
    if (!observer.has_delivery) return true;
    if (observer.delivery.has_every_n_steps()) {
      return tick_id - observer.last_delivery_tick >=
             observer.delivery.every_n_steps();
    }
    if (!observer.delivery.has_period_s()) return false;
    const long double elapsed_steps =
        static_cast<long double>(tick_id - observer.delivery_phase_tick);
    const long double elapsed =
        elapsed_steps * static_cast<long double>(timestep);
    const long double deadline =
        static_cast<long double>(observer.next_delivery_period) *
        static_cast<long double>(observer.delivery.period_s());
    const long double tolerance =
        static_cast<long double>(std::numeric_limits<double>::epsilon()) *
        8.0L * std::max({1.0L, std::abs(elapsed), std::abs(deadline)});
    return elapsed + tolerance >= deadline;
  }

  void MarkDelivered(Observer* observer, std::uint64_t tick_id,
                     double timestep) const {
    observer->last_delivery_tick = tick_id;
    if (observer->delivery.has_period_s()) {
      const double period = observer->delivery.period_s();
      if (!observer->has_delivery) {
        observer->delivery_phase_tick = tick_id;
        observer->next_delivery_period = 1;
      } else if (period <= timestep) {
        observer->next_delivery_period =
            tick_id - observer->delivery_phase_tick + 1;
      } else {
        ++observer->next_delivery_period;
      }
    }
    observer->has_delivery = true;
  }
};

class StatusBuilder {
 public:
  mujoco_sim::v2::ServerStatus Build(
      const mujoco_sim::v2::Header& header,
      mujoco_sim::v2::ServerState server_state, const std::string& message,
      const AdapterRegistry& registry,
      const BarrierCoordinator& barrier) const {
    mujoco_sim::v2::ServerStatus status;
    *status.mutable_header() = header;
    status.set_state(server_state);
    status.set_message(message);
    for (const auto& [_, controller] : registry.controllers) {
      auto* adapter = status.add_adapters();
      adapter->set_adapter_id(controller.id);
      adapter->set_role(mujoco_sim::v2::CONTROLLER);
      adapter->set_session_id(controller.session_id);
      adapter->set_state(controller.state);
      adapter->set_message(controller.message);
      adapter->set_state_since_tick_id(controller.state_since_tick);
      adapter->set_timeout_count(controller.timeout_count);
      adapter->set_last_heartbeat_tick_id(controller.last_heartbeat_tick);
      if (controller.has_tick_sent) {
        adapter->set_last_delivery_tick_id(controller.last_tick_sent);
      }
      if (controller.has_commit) {
        adapter->set_last_commit_tick_id(controller.last_commit_tick);
      }
      adapter->set_required_for_barrier(
          barrier.required_controllers.contains(controller.id));
    }
    for (const auto& [_, observer] : registry.observers) {
      auto* adapter = status.add_adapters();
      adapter->set_adapter_id(observer.id);
      adapter->set_role(OBSERVER);
      adapter->set_session_id(observer.session_id);
      adapter->set_state(observer.state);
      adapter->set_message(observer.message);
      adapter->set_state_since_tick_id(observer.state_since_tick);
      adapter->set_timeout_count(observer.timeout_count);
      if (observer.has_delivery) {
        adapter->set_last_delivery_tick_id(observer.last_delivery_tick);
      }
      adapter->set_last_observation_sequence(observer.sequence);
    }
    return status;
  }
};

}  // namespace detail

using detail::AdapterRegistry;
using detail::BarrierCoordinator;
using detail::Controller;
using detail::DataDeleter;
using detail::ModelDeleter;
using detail::ModelState;
using detail::ObservationScheduler;
using detail::Observer;
using detail::ResourceCatalog;
using detail::SnapshotManager;
using detail::StatusBuilder;
using detail::TickStore;

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

std::unique_ptr<SimulationSnapshot> detail::SnapshotManager::Create(
    const ModelState& model_state, std::string* error_message) const {
  if (!model_state.loaded()) {
    if (error_message != nullptr) {
      *error_message = "cannot create snapshot before loading a model";
    }
    return nullptr;
  }
  auto snapshot =
      std::unique_ptr<SimulationSnapshot>(new SimulationSnapshot());
  snapshot->impl_->owner = identity;
  snapshot->impl_->model_generation = model_generation;
  snapshot->impl_->model = std::shared_ptr<mjModel>(
      mj_copyModel(nullptr, model_state.model.get()),
      [](mjModel* model) { mj_deleteModel(model); });
  if (snapshot->impl_->model == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo model for visualization";
    }
    return nullptr;
  }
  snapshot->impl_->data = mj_copyData(nullptr, snapshot->impl_->model.get(),
                                      model_state.data.get());
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to copy MuJoCo data for visualization";
    }
    return nullptr;
  }
  return snapshot;
}

std::unique_ptr<SimulationSnapshot> detail::SnapshotManager::Clone(
    const ModelState& model_state, const SimulationSnapshot& source,
    std::string* error_message) const {
  if (!model_state.loaded() || source.impl_ == nullptr ||
      source.impl_->model == nullptr || source.impl_->data == nullptr ||
      source.impl_->owner.lock() != identity ||
      source.impl_->model_generation != model_generation) {
    if (error_message != nullptr) {
      *error_message = "cannot clone an invalid or stale simulation snapshot";
    }
    return nullptr;
  }
  auto snapshot =
      std::unique_ptr<SimulationSnapshot>(new SimulationSnapshot());
  snapshot->impl_->owner = identity;
  snapshot->impl_->model_generation = model_generation;
  snapshot->impl_->model = source.impl_->model;
  snapshot->impl_->data = mj_copyData(nullptr, snapshot->impl_->model.get(),
                                      source.impl_->data);
  if (snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to allocate cloned MuJoCo snapshot data";
    }
    return nullptr;
  }
  return snapshot;
}

bool detail::SnapshotManager::Update(const ModelState& model_state,
                                     SimulationSnapshot* snapshot,
                                     std::string* error_message) const {
  if (!model_state.loaded() || snapshot == nullptr ||
      snapshot->impl_ == nullptr || snapshot->impl_->model == nullptr ||
      snapshot->impl_->data == nullptr) {
    if (error_message != nullptr) *error_message = "invalid visualization snapshot";
    return false;
  }
  if (snapshot->impl_->owner.lock() != identity ||
      snapshot->impl_->model_generation != model_generation) {
    if (error_message != nullptr) {
      *error_message = "visualization snapshot does not belong to this model";
    }
    return false;
  }
  if (snapshot->impl_->model->nbuffer != model_state.model->nbuffer) {
    if (error_message != nullptr) {
      *error_message = "visualization snapshot model is incompatible";
    }
    return false;
  }
  mj_copyData(snapshot->impl_->data, snapshot->impl_->model.get(),
              model_state.data.get());
  return true;
}

SimulationSnapshot::SimulationSnapshot() : impl_(std::make_unique<Impl>()) {}
SimulationSnapshot::~SimulationSnapshot() = default;
SimulationSnapshot::SimulationSnapshot(SimulationSnapshot&&) noexcept = default;
SimulationSnapshot& SimulationSnapshot::operator=(
    SimulationSnapshot&&) noexcept = default;

const mjModel_* SimulationSnapshot::model() const {
  return impl_->model.get();
}
mjData_* SimulationSnapshot::data() const { return impl_->data; }

SimulationCore::SimulationCore(CoreOptions options)
    : options_(std::move(options)),
      model_state_(std::make_unique<ModelState>()),
      resource_catalog_(std::make_unique<ResourceCatalog>()),
      snapshot_manager_(std::make_unique<SnapshotManager>()),
      observation_scheduler_(std::make_unique<ObservationScheduler>()),
      status_builder_(std::make_unique<StatusBuilder>()),
      adapter_registry_(std::make_unique<AdapterRegistry>()),
      barrier_coordinator_(std::make_unique<BarrierCoordinator>()),
      tick_store_(std::make_unique<TickStore>()) {}

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
  EnsureInternalComponents();
  auto prepared = ModelState::Prepare(model_path, options, error_message);
  if (!prepared) return false;
  model_state_->Commit(std::move(*prepared));
  snapshot_manager_->Reset();

  resource_catalog_->observations.clear();
  resource_catalog_->commands.clear();
  resource_catalog_->observations_by_name.clear();
  resource_catalog_->commands_by_name.clear();
  adapter_registry_->ClearAdapters();
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  epoch_++;
  current_tick_id_ = 0;
  server_state_ = RUNNING;
  tick_open_ = false;
  status_message_.clear();
  BuildCatalogs();
  return true;
}

void SimulationCore::EnsureInternalComponents() {
  if (!adapter_registry_) {
    adapter_registry_ = std::make_unique<AdapterRegistry>();
  }
  if (!barrier_coordinator_) {
    barrier_coordinator_ = std::make_unique<BarrierCoordinator>();
  }
  if (!tick_store_) {
    tick_store_ = std::make_unique<TickStore>();
  }
  if (!model_state_) {
    model_state_ = std::make_unique<ModelState>();
  }
  if (!resource_catalog_) {
    resource_catalog_ = std::make_unique<ResourceCatalog>();
  }
  if (!snapshot_manager_) {
    snapshot_manager_ = std::make_unique<SnapshotManager>();
  }
  if (!observation_scheduler_) {
    observation_scheduler_ = std::make_unique<ObservationScheduler>();
  }
  if (!status_builder_) {
    status_builder_ = std::make_unique<StatusBuilder>();
  }
}

bool SimulationCore::IsModelLoaded() const { return model_state_->loaded(); }

std::unique_ptr<SimulationSnapshot> SimulationCore::CreateSnapshot(
    std::string* error_message) const {
  return snapshot_manager_->Create(*model_state_, error_message);
}

std::unique_ptr<SimulationSnapshot> SimulationCore::CloneSnapshot(
    const SimulationSnapshot& source, std::string* error_message) const {
  return snapshot_manager_->Clone(*model_state_, source, error_message);
}

bool SimulationCore::UpdateSnapshot(SimulationSnapshot* snapshot,
                                    std::string* error_message) const {
  return snapshot_manager_->Update(*model_state_, snapshot, error_message);
}

const std::vector<mujoco_sim::v2::ResourceDescriptor>&
SimulationCore::observations() const {
  return resource_catalog_->observations;
}

const std::vector<mujoco_sim::v2::ResourceDescriptor>&
SimulationCore::commands() const {
  return resource_catalog_->commands;
}

void SimulationCore::BuildCatalogs() {
  resource_catalog_->Build(model_state_->model.get());
}

void detail::ResourceCatalog::Build(const mjModel* model) {
  observations.clear();
  commands.clear();
  observations_by_name.clear();
  commands_by_name.clear();
  for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
    const char* name = mj_id2name(model, mjOBJ_SENSOR, sensor_id);
    if (name == nullptr || name[0] == '\0') {
      continue;
    }

    mujoco_sim::v2::ResourceDescriptor descriptor;
    descriptor.set_name(name);
    descriptor.set_type(OBSERVATION_SENSOR);
    descriptor.set_dimension(
        static_cast<std::uint32_t>(model->sensor_dim[sensor_id]));
    descriptor.set_mujoco_id(sensor_id);
    descriptor.set_mujoco_type(model->sensor_type[sensor_id]);
    descriptor.set_mujoco_datatype(model->sensor_datatype[sensor_id]);
    descriptor.set_mujoco_stage(model->sensor_needstage[sensor_id]);
    descriptor.set_interval_period_s(
        model->sensor_interval[2 * sensor_id]);
    descriptor.set_interval_phase_s(
        model->sensor_interval[2 * sensor_id + 1]);
    descriptor.set_delay_s(model->sensor_delay[sensor_id]);
    descriptor.set_unit(SensorUnit(model, sensor_id));
    descriptor.set_object_type(model->sensor_objtype[sensor_id]);
    descriptor.set_interpolation(SensorInterpolation(model, sensor_id));
    descriptor.set_frame_id(SensorFrameId(model, sensor_id));
    const int object_type = model->sensor_objtype[sensor_id];
    const int object_id = model->sensor_objid[sensor_id];
    if (object_type != mjOBJ_UNKNOWN && object_id >= 0) {
      const char* object_name =
          mj_id2name(model, static_cast<mjtObj>(object_type), object_id);
      if (object_name != nullptr) {
        descriptor.set_object_name(object_name);
      }
    }
    observations.push_back(descriptor);
    observations_by_name.emplace(descriptor.name(), descriptor);
  }

  for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
    const char* name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
    if (name == nullptr || name[0] == '\0') {
      continue;
    }

    mujoco_sim::v2::ResourceDescriptor descriptor;
    descriptor.set_name(name);
    descriptor.set_type(COMMAND_ACTUATOR);
    descriptor.set_dimension(1);
    descriptor.set_mujoco_id(actuator_id);
    descriptor.set_mujoco_type(mjOBJ_ACTUATOR);
    descriptor.set_mujoco_datatype(0);
    const bool limited = model->actuator_ctrllimited[actuator_id] != 0;
    descriptor.set_has_ctrl_range(limited);
    if (limited) {
      descriptor.set_ctrl_min(model->actuator_ctrlrange[2 * actuator_id]);
      descriptor.set_ctrl_max(model->actuator_ctrlrange[2 * actuator_id + 1]);
    }
    commands.push_back(descriptor);
    commands_by_name.emplace(descriptor.name(), descriptor);
  }
}

mujoco_sim::v2::GetAvailableResourcesResponse
SimulationCore::GetAvailableResources(
    const mujoco_sim::v2::GetAvailableResourcesRequest& request) const {
  mujoco_sim::v2::GetAvailableResourcesResponse response;
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
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor ||
      request.protocol_minor() != mujoco_sim::protocol::kProtocolMinor) {
    response.set_code(VERSION_MISMATCH);
    response.set_message("protocol version mismatch; expected 2.0");
    return response;
  }

  response.set_code(OK);
  response.set_epoch(epoch_);
  for (const auto& observation : resource_catalog_->observations) {
    CopyDescriptor(observation, response.add_observations());
  }
  for (const auto& command : resource_catalog_->commands) {
    CopyDescriptor(command, response.add_commands());
  }
  return response;
}

mujoco_sim::v2::RegisterAdapterResponse
SimulationCore::RegisterControlAdapter(
    const mujoco_sim::v2::RegisterAdapterRequest& request) {
  if (!IsModelLoaded()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "model is not loaded");
  }
  if (server_state_ == ERROR_PAUSED || server_state_ == mujoco_sim::v2::STOPPED) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting registrations");
  }
  if (request.server_name() != options_.server_name) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        NOT_FOUND, "server name mismatch");
  }
  if (request.adapter_id().empty()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter_id is required");
  }
  std::string identifier_error;
  if (!mujoco_sim::protocol::ValidateIdentifier(
          request.adapter_id(),
          mujoco_sim::protocol::IdentifierKind::kAdapter,
          &identifier_error)) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, identifier_error);
  }
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor ||
      request.protocol_minor() != mujoco_sim::protocol::kProtocolMinor) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        VERSION_MISMATCH, "protocol version mismatch; expected 2.0");
  }
  if (request.command_names().empty()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "at least one command actuator is required");
  }
  if (request.delivery().schedule_case() !=
      mujoco_sim::v2::ObservationDelivery::SCHEDULE_NOT_SET) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "controller adapters cannot set delivery");
  }
  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate) ||
      HasDuplicates(request.command_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }

  if (adapter_registry_->observers.contains(request.adapter_id())) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        ALREADY_EXISTS, "adapter_id is already registered as an observer");
  }

  const auto existing = adapter_registry_->controllers.find(request.adapter_id());
  const bool replacing =
      existing != adapter_registry_->controllers.end() && existing->second.state == TIMED_OUT &&
      options_.allow_timed_out_adapter_replacement;
  if (existing != adapter_registry_->controllers.end() && !replacing) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        ALREADY_EXISTS,
        existing->second.state == TIMED_OUT
            ? "timed-out controller replacement is disabled"
            : "adapter_id is already registered");
  }

  Controller controller;
  controller.id = request.adapter_id();
  controller.session_id = adapter_registry_->AllocateSession();
  controller.epoch = epoch_;
  controller.timeout_ms = request.timeout_ms() == 0 ? options_.default_timeout_ms
                                                    : request.timeout_ms();
  controller.protocol_minor = mujoco_sim::protocol::kProtocolMinor;
  controller.state = PROVISIONAL;
  controller.registered_at = std::chrono::steady_clock::now();
  controller.state_since_tick = current_tick_id_;

  for (const auto& name : request.observation_names()) {
    const auto it = resource_catalog_->observations_by_name.find(name);
    if (it == resource_catalog_->observations_by_name.end()) {
      return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
          NOT_FOUND, "unknown observation sensor: " + name);
    }
    CopyDescriptor(it->second, controller.layout.add_observations());
  }

  for (const auto& name : request.command_names()) {
    const auto command = resource_catalog_->commands_by_name.find(name);
    if (command == resource_catalog_->commands_by_name.end()) {
      return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
          NOT_FOUND, "unknown command actuator: " + name);
    }
    const auto owner = adapter_registry_->actuator_owner.find(name);
    if (owner != adapter_registry_->actuator_owner.end() &&
        (!replacing || owner->second != request.adapter_id())) {
      return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
          RESOURCE_CONFLICT,
          "actuator already owned by controller: " + owner->second);
    }
    CopyDescriptor(command->second, controller.layout.add_commands());
  }

  if (replacing) {
    ZeroControllerCommands(existing->second);
    ReleaseControllerResources(existing->second);
    barrier_coordinator_->pending_commits.erase(existing->first);
    InvalidateIssuedTick(existing->first);
    barrier_coordinator_->required_controllers.erase(existing->first);
    barrier_coordinator_->recipient_controllers.erase(existing->first);
    adapter_registry_->controllers.erase(existing);
  }

  for (const auto& command : controller.layout.commands()) {
    adapter_registry_->actuator_owner[command.name()] = controller.id;
  }
  controller.layout.set_layout_hash(
      mujoco_sim::protocol::ComputeLayoutHash(controller.layout));

  mujoco_sim::v2::RegisterAdapterResponse response;
  response.set_code(OK);
  response.set_adapter_id(controller.id);
  response.set_role(mujoco_sim::v2::CONTROLLER);
  response.set_session_id(controller.session_id);
  response.set_epoch(controller.epoch);
  response.set_state(PROVISIONAL);
  if (replacing) {
    response.set_message("replaced timed-out controller session");
  }
  *response.mutable_layout() = controller.layout;

  adapter_registry_->controllers.emplace(controller.id, std::move(controller));
  return response;
}

mujoco_sim::v2::ActivateAdapterResponse
SimulationCore::ActivateControlAdapter(
    const mujoco_sim::v2::ActivateAdapterRequest& request) {
  const auto it = adapter_registry_->controllers.find(request.adapter_id());
  if (it == adapter_registry_->controllers.end()) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        NOT_FOUND, "controller is not registered");
  }
  Controller& controller = it->second;
  if (request.session_id() != controller.session_id) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  if (controller.state != PROVISIONAL) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "controller is not provisional");
  }
  SetControllerState(&controller, ACTIVE, {});
  mujoco_sim::v2::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

UnregisterAdapterResult
SimulationCore::UnregisterControlAdapter(
    const mujoco_sim::v2::UnregisterAdapterRequest& request) {
  const auto it = adapter_registry_->controllers.find(request.adapter_id());
  if (it == adapter_registry_->controllers.end()) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                NOT_FOUND, "controller is not registered"),
            std::nullopt};
  }
  Controller& controller = it->second;
  if (request.session_id() != controller.session_id) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_SESSION, "session mismatch"),
            std::nullopt};
  }
  if (request.epoch() != epoch_) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_EPOCH, "epoch mismatch"),
            std::nullopt};
  }
  ZeroControllerCommands(controller);
  ReleaseControllerResources(controller);
  barrier_coordinator_->pending_commits.erase(controller.id);
  InvalidateIssuedTick(controller.id);
  barrier_coordinator_->required_controllers.erase(controller.id);
  barrier_coordinator_->recipient_controllers.erase(controller.id);
  adapter_registry_->controllers.erase(it);
  auto step_completed = CompleteBarrier(StepCompletionSource::kUnregister);

  mujoco_sim::v2::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return {std::move(response), std::move(step_completed)};
}

mujoco_sim::v2::RegisterAdapterResponse SimulationCore::RegisterAdapter(
    const mujoco_sim::v2::RegisterAdapterRequest& request) {
  if (request.role() != mujoco_sim::v2::CONTROLLER &&
      request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  if (request.protocol_major() != mujoco_sim::protocol::kProtocolMajor ||
      request.protocol_minor() != mujoco_sim::protocol::kProtocolMinor) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        VERSION_MISMATCH, "protocol version mismatch; expected 2.0");
  }
  if (request.role() == mujoco_sim::v2::CONTROLLER) {
    return RegisterControlAdapter(request);
  }
  if (!IsModelLoaded()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "model is not loaded");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting registrations");
  }
  if (request.server_name() != options_.server_name) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        NOT_FOUND, "server name mismatch");
  }
  if (request.adapter_id().empty()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "adapter_id is required");
  }
  std::string identifier_error;
  if (!mujoco_sim::protocol::ValidateIdentifier(
          request.adapter_id(),
          mujoco_sim::protocol::IdentifierKind::kAdapter,
          &identifier_error)) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, identifier_error);
  }
  if (request.observation_names().empty()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "at least one observation sensor is required");
  }
  if (!request.command_names().empty()) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "observer adapters cannot claim actuators");
  }
  std::string duplicate;
  if (HasDuplicates(request.observation_names(), &duplicate)) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "duplicate resource name: " + duplicate);
  }
  if (request.delivery().has_every_n_steps() &&
      request.delivery().every_n_steps() == 0) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "every_n_steps must be >= 1");
  }
  if (request.delivery().has_period_s() &&
      (!std::isfinite(request.delivery().period_s()) ||
       request.delivery().period_s() <= 0.0)) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        INVALID_ARGUMENT, "period_s must be finite and > 0");
  }
  if (adapter_registry_->controllers.contains(request.adapter_id())) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
        ALREADY_EXISTS, "adapter_id is already registered as a controller");
  }

  const auto existing = adapter_registry_->observers.find(request.adapter_id());
  const bool replacing =
      existing != adapter_registry_->observers.end() && existing->second.state == TIMED_OUT &&
      options_.allow_timed_out_adapter_replacement;
  if (existing != adapter_registry_->observers.end() && !replacing) {
    return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
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
  observer.protocol_minor = mujoco_sim::protocol::kProtocolMinor;
  observer.registered_at = std::chrono::steady_clock::now();
  observer.last_heartbeat_at = observer.registered_at;
  observer.state_since_tick = current_tick_id_;
  observer.delivery = request.delivery();
  if (observer.delivery.schedule_case() ==
      mujoco_sim::v2::ObservationDelivery::SCHEDULE_NOT_SET) {
    observer.delivery.set_every_n_steps(1);
  }
  for (const auto& name : request.observation_names()) {
    const auto found = resource_catalog_->observations_by_name.find(name);
    if (found == resource_catalog_->observations_by_name.end()) {
      return ErrorResponse<mujoco_sim::v2::RegisterAdapterResponse>(
          NOT_FOUND, "unknown observation sensor: " + name);
    }
    CopyDescriptor(found->second, observer.layout.add_observations());
  }
  observer.session_id = adapter_registry_->AllocateSession();
  observer.layout.set_layout_hash(
      mujoco_sim::protocol::ComputeLayoutHash(observer.layout));
  if (replacing) {
    adapter_registry_->observers.erase(existing);
  }

  mujoco_sim::v2::RegisterAdapterResponse response;
  response.set_code(OK);
  response.set_adapter_id(observer.id);
  response.set_role(OBSERVER);
  response.set_session_id(observer.session_id);
  response.set_epoch(observer.epoch);
  response.set_state(PROVISIONAL);
  *response.mutable_layout() = observer.layout;
  if (replacing) {
    response.set_message("replaced timed-out observer session");
  }
  adapter_registry_->observers.emplace(observer.id, std::move(observer));
  return response;
}

mujoco_sim::v2::ActivateAdapterResponse SimulationCore::ActivateAdapter(
    const mujoco_sim::v2::ActivateAdapterRequest& request) {
  if (request.role() == mujoco_sim::v2::CONTROLLER) {
    return ActivateControlAdapter(request);
  }
  if (request.role() != OBSERVER) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_ARGUMENT, "adapter role must be CONTROLLER or OBSERVER");
  }
  const auto found = adapter_registry_->observers.find(request.adapter_id());
  if (found == adapter_registry_->observers.end()) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        NOT_FOUND, "observer is not registered");
  }
  Observer& observer = found->second;
  if (request.session_id() != observer.session_id) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_SESSION, "session mismatch");
  }
  if (request.epoch() != epoch_) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        INVALID_EPOCH, "epoch mismatch");
  }
  if (observer.state != PROVISIONAL) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "observer is not provisional");
  }
  if (server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
    return ErrorResponse<mujoco_sim::v2::ActivateAdapterResponse>(
        FAILED_PRECONDITION, "server is not accepting activations");
  }
  observer.state = ACTIVE;
  observer.state_since_tick = current_tick_id_;
  observer.activation_tick = current_tick_id_;
  observer.last_heartbeat_at = std::chrono::steady_clock::now();
  observer.message.clear();
  mujoco_sim::v2::ActivateAdapterResponse response;
  response.set_code(OK);
  response.set_state(ACTIVE);
  return response;
}

UnregisterAdapterResult SimulationCore::UnregisterAdapter(
    const mujoco_sim::v2::UnregisterAdapterRequest& request) {
  if (request.role() == mujoco_sim::v2::CONTROLLER) {
    return UnregisterControlAdapter(request);
  }
  if (request.role() != OBSERVER) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_ARGUMENT,
                "adapter role must be CONTROLLER or OBSERVER"),
            std::nullopt};
  }
  const auto found = adapter_registry_->observers.find(request.adapter_id());
  if (found == adapter_registry_->observers.end()) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                NOT_FOUND, "observer is not registered"),
            std::nullopt};
  }
  if (request.session_id() != found->second.session_id) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_SESSION, "session mismatch"),
            std::nullopt};
  }
  if (request.epoch() != epoch_) {
    return {ErrorResponse<mujoco_sim::v2::UnregisterAdapterResponse>(
                INVALID_EPOCH, "epoch mismatch"),
            std::nullopt};
  }
  adapter_registry_->observers.erase(found);
  mujoco_sim::v2::UnregisterAdapterResponse response;
  response.set_code(OK);
  response.set_state(UNREGISTERED);
  return {std::move(response), std::nullopt};
}

std::vector<IssuedTickHandle> SimulationCore::OpenIssuedTicks() {
  std::vector<IssuedTickHandle> ticks;
  if (!IsModelLoaded() || server_state_ == ERROR_PAUSED ||
      server_state_ == mujoco_sim::v2::STOPPED) {
    return ticks;
  }
  if (!tick_open_) {
    InvalidateAllIssuedTicks();
    barrier_coordinator_->Reset();
    for (auto& [adapter_id, controller] : adapter_registry_->controllers) {
      if (controller.recovery_pending) {
        controller.recovery_pending = false;
        SetControllerState(&controller, ACTIVE, {});
      }
      if (controller.state == ACTIVE) {
        barrier_coordinator_->required_controllers.emplace(adapter_id, true);
        barrier_coordinator_->recipient_controllers.emplace(adapter_id, true);
      } else if (controller.state == TIMED_OUT) {
        barrier_coordinator_->recipient_controllers.emplace(adapter_id, true);
      }
    }
    for (const auto& [adapter_id, _] : barrier_coordinator_->recipient_controllers) {
      auto& controller = adapter_registry_->controllers.at(adapter_id);
      controller.last_tick_sent = current_tick_id_;
      controller.has_tick_sent = true;
      const auto issued = tick_store_->Insert(
          adapter_id, BuildTickFor(controller),
          controller.layout.ByteSizeLong(), controller.session_id, epoch_,
          current_tick_id_);
      if (!issued) {
        EnterErrorPaused("failed to serialize issued Tick");
        return {};
      }
    }
    tick_started_at_ = std::chrono::steady_clock::now();
    tick_open_ = true;
  }
  server_state_ = RUNNING;
  for (const auto& [adapter_id, _] : barrier_coordinator_->recipient_controllers) {
    const auto it = adapter_registry_->controllers.find(adapter_id);
    if (it == adapter_registry_->controllers.end() ||
        (it->second.state != ACTIVE && it->second.state != TIMED_OUT)) {
      continue;
    }
    const auto tick = tick_store_->Find(adapter_id);
    if (tick) ticks.push_back(tick);
  }
  return ticks;
}

bool SimulationCore::IsIssuedTickCurrent(
    const IssuedTickIdentity& identity) const {
  return tick_open_ && tick_store_->IsCurrent(identity);
}

void SimulationCore::InvalidateIssuedTick(const std::string& adapter_id) {
  tick_store_->Invalidate(adapter_id);
}

void SimulationCore::InvalidateAllIssuedTicks() {
  tick_store_->InvalidateAll();
}

mujoco_sim::v2::Tick SimulationCore::BuildTickFor(
    const Controller& controller) const {
  mujoco_sim::v2::Tick tick;
  *tick.mutable_header() = MakeHeader(controller.session_id);
  tick.set_adapter_id(controller.id);
  tick.set_controller_state(controller.state);
  tick.set_layout_hash(controller.layout.layout_hash());

  for (const auto& observation : controller.layout.observations()) {
    auto* block = tick.add_observations();
    block->set_name(observation.name());
    const int sensor_id = observation.mujoco_id();
    const int address = model_state_->model->sensor_adr[sensor_id];
    const int dimension = model_state_->model->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      block->add_values(static_cast<double>(model_state_->data->sensordata[address + i]));
    }
  }

  return tick;
}

std::vector<mujoco_sim::v2::ObservationFrame>
SimulationCore::BuildObservationFramesForCompletedStep() {
  std::vector<mujoco_sim::v2::ObservationFrame> frames;
  for (auto& [_, observer] : adapter_registry_->observers) {
    if (observer.state != ACTIVE ||
        current_tick_id_ <= observer.activation_tick) {
      continue;
    }

    if (!observation_scheduler_->Due(observer, current_tick_id_, timestep())) {
      continue;
    }

    frames.push_back(BuildObservationFrame(&observer));
    observation_scheduler_->MarkDelivered(&observer, current_tick_id_,
                                           timestep());
  }
  return frames;
}

mujoco_sim::v2::ObservationFrame SimulationCore::BuildObservationFrame(
    Observer* observer) const {
  mujoco_sim::v2::ObservationFrame frame;
  *frame.mutable_header() = MakeHeader(observer->session_id);
  frame.mutable_header()->set_protocol_minor(observer->protocol_minor);
  frame.set_adapter_id(observer->id);
  frame.set_sequence(++observer->sequence);
  for (const auto& observation : observer->layout.observations()) {
    auto* block = frame.add_observations();
    block->set_name(observation.name());
    const int sensor_id = observation.mujoco_id();
    const int address = model_state_->model->sensor_adr[sensor_id];
    const int dimension = model_state_->model->sensor_dim[sensor_id];
    for (int i = 0; i < dimension; ++i) {
      block->add_values(static_cast<double>(model_state_->data->sensordata[address + i]));
    }
  }
  return frame;
}

mujoco_sim::v2::Header SimulationCore::MakeHeader(
    std::uint64_t session_id) const {
  mujoco_sim::v2::Header header;
  header.set_server_name(options_.server_name);
  header.set_session_id(session_id);
  header.set_epoch(epoch_);
  header.set_tick_id(current_tick_id_);
  header.set_timestamp_unix_nanos(NowUnixNanos());
  header.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  header.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  header.set_simulation_time_s(model_state_->simulation_time);
  return header;
}

bool SimulationCore::ValidateCommonHeader(
    const mujoco_sim::v2::Header& header, const Controller& controller,
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

std::optional<StepCompletedEvent> SimulationCore::CompleteBarrier(
    StepCompletionSource source) {
  if (!tick_open_ || !barrier_coordinator_->Ready()) {
    return std::nullopt;
  }
  model_state_->ApplyActuatorWrites(
      barrier_coordinator_->BuildWritePlan(*resource_catalog_));
  model_state_->Step();
  ++current_tick_id_;
  StepCompletedEvent event;
  event.completed_tick_id = current_tick_id_ - 1;
  event.next_tick_id = current_tick_id_;
  event.simulation_time_s = model_state_->simulation_time;
  event.source = source;
  event.observer_frames = BuildObservationFramesForCompletedStep();
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  tick_open_ = false;
  return event;
}

EmptyBarrierResult SimulationCore::CompleteEmptyBarrier() {
  return {CompleteBarrier(StepCompletionSource::kEmptyBarrier)};
}

CommitResult SimulationCore::SubmitCommit(
    const mujoco_sim::v2::AdapterCommit& commit) {
  CommitResult result;
  if (server_state_ == ERROR_PAUSED) {
    result.code = FAILED_PRECONDITION;
    result.message = "server is error-paused";
    return result;
  }
  const auto controller_it = adapter_registry_->controllers.find(commit.adapter_id());
  if (controller_it == adapter_registry_->controllers.end()) {
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
      result.step_completed = EnterControllerError(
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
    if (commit.status() != mujoco_sim::v2::COMMIT_OK) {
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
      result.step_completed = EnterControllerError(
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
  if (!barrier_coordinator_->required_controllers.contains(controller.id)) {
    result.code = FAILED_PRECONDITION;
    result.message = "controller joins the barrier on the next Tick";
    return result;
  }
  if (!ValidateCommonHeader(commit.header(), controller, &message)) {
    result.code = message == "tick mismatch" ? INVALID_TICK : INVALID_ARGUMENT;
    result.message = message;
    return result;
  }

  const auto pending_it = barrier_coordinator_->pending_commits.find(controller.id);
  if (pending_it != barrier_coordinator_->pending_commits.end()) {
    if (!SameCommitPayload(commit, pending_it->second)) {
      result.step_completed = EnterControllerError(
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

  if (commit.status() != mujoco_sim::v2::COMMIT_OK) {
    result.step_completed = EnterControllerError(
        &controller,
        "controller reported commit error: " + commit.error_message());
    result.code = INVALID_COMMAND;
    result.message = controller.message;
    result.controller_state_changed = true;
    result.controller_state = ERROR;
    return result;
  }

  if (!ValidateCommands(controller, commit, &message)) {
    result.step_completed = EnterControllerError(&controller, message);
    result.code = INVALID_COMMAND;
    result.message = controller.message;
    result.controller_state_changed = true;
    result.controller_state = ERROR;
    return result;
  }

  barrier_coordinator_->pending_commits.emplace(controller.id, commit);
  controller.last_commit_tick = current_tick_id_;
  controller.has_commit = true;
  controller.last_commit = commit;
  if (barrier_coordinator_->pending_commits.size() != barrier_coordinator_->required_controllers.size()) {
    result.message = "commit accepted";
    return result;
  }
  result.step_completed = CompleteBarrier(StepCompletionSource::kCommit);
  result.message = result.step_completed ? "barrier complete"
                                         : "commit accepted";
  return result;
}

void SimulationCore::AcceptHeartbeat(
    const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  if (heartbeat.role() == mujoco_sim::v2::CONTROLLER) {
    const auto found = adapter_registry_->controllers.find(heartbeat.adapter_id());
    if (found == adapter_registry_->controllers.end()) {
      return;
    }
    Controller& controller = found->second;
    const auto& header = heartbeat.header();
    if (header.server_name() == options_.server_name &&
        header.session_id() == controller.session_id &&
        header.epoch() == epoch_ &&
        header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
        header.protocol_minor() == mujoco_sim::protocol::kProtocolMinor &&
        header.tick_id() <= controller.last_tick_sent) {
      controller.last_heartbeat_tick =
          std::max(controller.last_heartbeat_tick, header.tick_id());
    }
    return;
  }
  if (heartbeat.role() != OBSERVER) {
    return;
  }
  const auto found = adapter_registry_->observers.find(heartbeat.adapter_id());
  if (found == adapter_registry_->observers.end()) {
    return;
  }
  Observer& observer = found->second;
  const auto& header = heartbeat.header();
  if (observer.state == ACTIVE &&
      header.server_name() == options_.server_name &&
      header.session_id() == observer.session_id && header.epoch() == epoch_ &&
      header.protocol_major() == mujoco_sim::protocol::kProtocolMajor &&
      header.protocol_minor() == mujoco_sim::protocol::kProtocolMinor &&
      heartbeat.last_observation_sequence() <= observer.sequence) {
    observer.last_heartbeat_at = now;
  }
}

DeadlineResult SimulationCore::CheckDeadlines(
    std::chrono::steady_clock::time_point now) {
  DeadlineResult result;
  for (auto it = adapter_registry_->controllers.begin(); it != adapter_registry_->controllers.end();) {
    Controller& controller = it->second;
    if (controller.state == PROVISIONAL &&
        now - controller.registered_at >
            std::chrono::milliseconds(options_.activation_timeout_ms)) {
      result.transitions.push_back(
          AdapterTransition{controller.id, mujoco_sim::v2::CONTROLLER,
                               controller.session_id,
                               PROVISIONAL, UNREGISTERED,
                               "controller activation timed out",
                               current_tick_id_});
      ReleaseControllerResources(controller);
      it = adapter_registry_->controllers.erase(it);
      continue;
    }
    ++it;
  }

  for (auto it = adapter_registry_->observers.begin(); it != adapter_registry_->observers.end();) {
    Observer& observer = it->second;
    if (observer.state == PROVISIONAL &&
        now - observer.registered_at >
            std::chrono::milliseconds(options_.activation_timeout_ms)) {
      result.transitions.push_back(AdapterTransition{
          observer.id, OBSERVER, observer.session_id, PROVISIONAL,
          UNREGISTERED, "observer activation timed out", current_tick_id_});
      it = adapter_registry_->observers.erase(it);
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
      result.transitions.push_back(AdapterTransition{
          observer.id, OBSERVER, observer.session_id, old_state, TIMED_OUT,
          observer.message, current_tick_id_});
    }
    ++it;
  }

  if (tick_open_ && server_state_ != ERROR_PAUSED) {
    for (auto& [adapter_id, controller] : adapter_registry_->controllers) {
      if (!barrier_coordinator_->required_controllers.contains(adapter_id) ||
          barrier_coordinator_->pending_commits.contains(adapter_id)) {
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
      barrier_coordinator_->pending_commits.erase(adapter_id);
      barrier_coordinator_->required_controllers.erase(adapter_id);
      result.transitions.push_back(
          AdapterTransition{controller.id, mujoco_sim::v2::CONTROLLER,
                               controller.session_id, old_state,
                               TIMED_OUT, controller.message,
                               current_tick_id_});
    }
    result.step_completed = CompleteBarrier(StepCompletionSource::kDeadline);
  }
  return result;
}

mujoco_sim::v2::ServerStatus SimulationCore::MakeStatus() const {
  return status_builder_->Build(MakeHeader(0), server_state_, status_message_,
                                *adapter_registry_, *barrier_coordinator_);
}

bool SimulationCore::ValidateCommands(
    const Controller& controller,
    const mujoco_sim::v2::AdapterCommit& commit,
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
    const auto descriptor = resource_catalog_->commands_by_name.at(command.name());
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
  std::vector<int> actuator_ids;
  actuator_ids.reserve(controller.layout.commands_size());
  for (const auto& command : controller.layout.commands()) {
    actuator_ids.push_back(command.mujoco_id());
  }
  if (tick_open_) {
    barrier_coordinator_->pending_zero_actuators.insert(actuator_ids.begin(),
                                                        actuator_ids.end());
  } else {
    model_state_->ZeroActuators(actuator_ids);
  }
}

void SimulationCore::SetControllerState(
    Controller* controller, mujoco_sim::v2::ControllerState state,
    const std::string& message) {
  controller->state = state;
  controller->state_since_tick = current_tick_id_;
  controller->message = message;
}

std::optional<StepCompletedEvent> SimulationCore::EnterControllerError(
    Controller* controller, const std::string& message) {
  SetControllerState(controller, ERROR, message);
  controller->recovery_pending = false;
  ZeroControllerCommands(*controller);
  barrier_coordinator_->pending_commits.erase(controller->id);
  InvalidateIssuedTick(controller->id);
  barrier_coordinator_->required_controllers.erase(controller->id);
  barrier_coordinator_->recipient_controllers.erase(controller->id);
  return CompleteBarrier(StepCompletionSource::kLocalError);
}

void SimulationCore::ReleaseControllerResources(
    const Controller& controller) {
  for (const auto& command : controller.layout.commands()) {
    const auto owner = adapter_registry_->actuator_owner.find(command.name());
    if (owner != adapter_registry_->actuator_owner.end() && owner->second == controller.id) {
      adapter_registry_->actuator_owner.erase(owner);
    }
  }
}

void SimulationCore::EnterErrorPaused(const std::string& message) {
  server_state_ = ERROR_PAUSED;
  status_message_ = message;
  tick_open_ = false;
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
  for (auto& [_, controller] : adapter_registry_->controllers) {
    if (controller.state == ACTIVE) {
      SetControllerState(&controller, ERROR, message);
    }
  }
  for (auto& [_, observer] : adapter_registry_->observers) {
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
  server_state_ = mujoco_sim::v2::STOPPED;
  status_message_ = message;
  tick_open_ = false;
  InvalidateAllIssuedTicks();
  barrier_coordinator_->Reset();
}

std::uint64_t SimulationCore::epoch() const { return epoch_; }

std::uint64_t SimulationCore::current_tick_id() const {
  return current_tick_id_;
}

double SimulationCore::simulation_time() const { return model_state_->simulation_time; }

double SimulationCore::timestep() const {
  return model_state_->model == nullptr ? 0.0 : model_state_->model->opt.timestep;
}

mujoco_sim::v2::ServerState SimulationCore::server_state() const {
  return server_state_;
}

bool SimulationCore::tick_open() const { return tick_open_; }

std::size_t SimulationCore::active_controller_count() const {
  std::size_t count = 0;
  for (const auto& [_, controller] : adapter_registry_->controllers) {
    if (controller.state == ACTIVE) {
      ++count;
    }
  }
  return count;
}

std::size_t SimulationCore::controller_count(
    mujoco_sim::v2::ControllerState state) const {
  return static_cast<std::size_t>(std::count_if(
      adapter_registry_->controllers.begin(), adapter_registry_->controllers.end(),
      [state](const auto& entry) { return entry.second.state == state; }));
}

std::size_t SimulationCore::observer_count(
    mujoco_sim::v2::ControllerState state) const {
  return static_cast<std::size_t>(std::count_if(
      adapter_registry_->observers.begin(), adapter_registry_->observers.end(),
      [state](const auto& entry) { return entry.second.state == state; }));
}

}  // namespace mujoco_sim::core
