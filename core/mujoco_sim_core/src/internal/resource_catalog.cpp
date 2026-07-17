#include "resource_catalog.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

namespace mujoco_sim::core::internal {
namespace {

using mujoco_sim::core::COMMAND_ACTUATOR;
using mujoco_sim::core::OBSERVATION_SENSOR;

enum class SensorQuantity {
  kPosition,
  kVelocity,
  kForce
};

std::string JointUnit(const mjModel* model, int joint_id, SensorQuantity quantity) {
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

std::string TendonUnit(const mjModel* model, int tendon_id, SensorQuantity quantity) {
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

    const std::string joint_unit = JointUnit(model, model->wrap_objid[wrap_id], quantity);
    if (joint_unit.empty() || (!unit.empty() && unit != joint_unit)) {
      return {};
    }
    unit = joint_unit;
  }
  return unit;
}

std::string ActuatorUnit(const mjModel* model, int actuator_id, SensorQuantity quantity) {
  if (actuator_id < 0 || actuator_id >= model->nu) {
    return {};
  }
  switch (model->actuator_trntype[actuator_id]) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
      return JointUnit(model, model->actuator_trnid[2 * actuator_id], quantity);
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
      return TendonUnit(model, model->actuator_trnid[2 * actuator_id], quantity);
    default:
      // Site, body and plugin transmissions can mix linear and angular axes.
      break;
  }
  return {};
}

std::optional<std::string> JointSensorUnit(const mjModel* model, int sensor_type, int object_id) {
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
    default:
      return std::nullopt;
  }
}

std::optional<std::string> TendonSensorUnit(const mjModel* model, int sensor_type, int object_id) {
  switch (sensor_type) {
    case mjSENS_TENDONPOS:
    case mjSENS_TENDONLIMITPOS:
      return TendonUnit(model, object_id, SensorQuantity::kPosition);
    case mjSENS_TENDONVEL:
    case mjSENS_TENDONLIMITVEL:
      return TendonUnit(model, object_id, SensorQuantity::kVelocity);
    case mjSENS_TENDONACTFRC:
    case mjSENS_TENDONLIMITFRC:
      return TendonUnit(model, object_id, SensorQuantity::kForce);
    default:
      return std::nullopt;
  }
}

std::optional<std::string> ActuatorSensorUnit(const mjModel* model, int sensor_type,
                                              int object_id) {
  switch (sensor_type) {
    case mjSENS_ACTUATORPOS:
      return ActuatorUnit(model, object_id, SensorQuantity::kPosition);
    case mjSENS_ACTUATORVEL:
      return ActuatorUnit(model, object_id, SensorQuantity::kVelocity);
    case mjSENS_ACTUATORFRC:
      return ActuatorUnit(model, object_id, SensorQuantity::kForce);
    default:
      return std::nullopt;
  }
}

struct FixedUnitEntry {
  int sensor_type;
  std::string_view unit;
};

constexpr std::array kFixedSensorUnits{
    FixedUnitEntry{mjSENS_BALLQUAT, "1"},
    FixedUnitEntry{mjSENS_FRAMEQUAT, "1"},
    FixedUnitEntry{mjSENS_FRAMEXAXIS, "1"},
    FixedUnitEntry{mjSENS_FRAMEYAXIS, "1"},
    FixedUnitEntry{mjSENS_FRAMEZAXIS, "1"},
    FixedUnitEntry{mjSENS_GEOMNORMAL, "1"},
    FixedUnitEntry{mjSENS_INSIDESITE, "1"},
    FixedUnitEntry{mjSENS_BALLANGVEL, "rad/s"},
    FixedUnitEntry{mjSENS_GYRO, "rad/s"},
    FixedUnitEntry{mjSENS_FRAMEANGVEL, "rad/s"},
    FixedUnitEntry{mjSENS_ACCELEROMETER, "m/s^2"},
    FixedUnitEntry{mjSENS_FRAMELINACC, "m/s^2"},
    FixedUnitEntry{mjSENS_FRAMEANGACC, "rad/s^2"},
    FixedUnitEntry{mjSENS_VELOCIMETER, "m/s"},
    FixedUnitEntry{mjSENS_FRAMELINVEL, "m/s"},
    FixedUnitEntry{mjSENS_SUBTREELINVEL, "m/s"},
    FixedUnitEntry{mjSENS_FORCE, "N"},
    FixedUnitEntry{mjSENS_TOUCH, "N"},
    FixedUnitEntry{mjSENS_TORQUE, "N*m"},
    FixedUnitEntry{mjSENS_RANGEFINDER, "m"},
    FixedUnitEntry{mjSENS_FRAMEPOS, "m"},
    FixedUnitEntry{mjSENS_SUBTREECOM, "m"},
    FixedUnitEntry{mjSENS_GEOMDIST, "m"},
    FixedUnitEntry{mjSENS_GEOMFROMTO, "m"},
    FixedUnitEntry{mjSENS_CAMPROJECTION, "px"},
    FixedUnitEntry{mjSENS_MAGNETOMETER, "T"},
    FixedUnitEntry{mjSENS_CLOCK, "s"},
    FixedUnitEntry{mjSENS_E_POTENTIAL, "J"},
    FixedUnitEntry{mjSENS_E_KINETIC, "J"},
};

std::string FixedSensorUnit(int sensor_type) {
  const auto found = std::find_if(
      kFixedSensorUnits.begin(), kFixedSensorUnits.end(),
      [sensor_type](const FixedUnitEntry& entry) { return entry.sensor_type == sensor_type; });
  return found == kFixedSensorUnits.end() ? "" : std::string(found->unit);
}

std::string SensorUnit(const mjModel* model, int sensor_id) {
  const int sensor_type = model->sensor_type[sensor_id];
  const int object_id = model->sensor_objid[sensor_id];
  if (auto unit = JointSensorUnit(model, sensor_type, object_id)) {
    return *unit;
  }
  if (auto unit = TendonSensorUnit(model, sensor_type, object_id)) {
    return *unit;
  }
  if (auto unit = ActuatorSensorUnit(model, sensor_type, object_id)) {
    return *unit;
  }
  return FixedSensorUnit(sensor_type);
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
  const char* name = mj_id2name(model, static_cast<mjtObj>(object_type), object_id);
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
      return ObjectName(model, model->sensor_objtype[sensor_id], model->sensor_objid[sensor_id]);
    case mjSENS_GEOMNORMAL:
    case mjSENS_GEOMFROMTO:
    case mjSENS_SUBTREECOM:
    case mjSENS_SUBTREELINVEL:
    case mjSENS_SUBTREEANGMOM:
      return "world";
    case mjSENS_CAMPROJECTION:
      return ObjectName(model, model->sensor_reftype[sensor_id], model->sensor_refid[sensor_id]);
    default:
      return {};
  }
}

}  // namespace

const mujoco_sim::core::ResourceDescriptor* ResourceCatalog::FindObservation(
    const std::string& name) const {
  const auto found = observations_by_name_.find(name);
  return found == observations_by_name_.end() ? nullptr : &found->second;
}

const mujoco_sim::core::ResourceDescriptor* ResourceCatalog::FindCommand(
    const std::string& name) const {
  const auto found = commands_by_name_.find(name);
  return found == commands_by_name_.end() ? nullptr : &found->second;
}

ResourceCatalog ResourceCatalog::Build(const mjModel& model) {
  ResourceCatalog catalog;
  for (int sensor_id = 0; sensor_id < model.nsensor; ++sensor_id) {
    const char* name = mj_id2name(&model, mjOBJ_SENSOR, sensor_id);
    if (name == nullptr || name[0] == '\0') {
      continue;
    }

    mujoco_sim::core::ResourceDescriptor descriptor;
    descriptor.set_name(name);
    descriptor.set_type(OBSERVATION_SENSOR);
    descriptor.set_dimension(static_cast<std::uint32_t>(model.sensor_dim[sensor_id]));
    descriptor.set_mujoco_id(sensor_id);
    descriptor.set_mujoco_type(model.sensor_type[sensor_id]);
    descriptor.set_mujoco_datatype(model.sensor_datatype[sensor_id]);
    descriptor.set_mujoco_stage(model.sensor_needstage[sensor_id]);
    descriptor.set_interval_period_s(model.sensor_interval[2 * sensor_id]);
    descriptor.set_interval_phase_s(model.sensor_interval[2 * sensor_id + 1]);
    descriptor.set_delay_s(model.sensor_delay[sensor_id]);
    descriptor.set_unit(SensorUnit(&model, sensor_id));
    descriptor.set_object_type(model.sensor_objtype[sensor_id]);
    descriptor.set_interpolation(SensorInterpolation(&model, sensor_id));
    descriptor.set_frame_id(SensorFrameId(&model, sensor_id));
    const int object_type = model.sensor_objtype[sensor_id];
    const int object_id = model.sensor_objid[sensor_id];
    if (object_type != mjOBJ_UNKNOWN && object_id >= 0) {
      const char* object_name = mj_id2name(&model, static_cast<mjtObj>(object_type), object_id);
      if (object_name != nullptr) {
        descriptor.set_object_name(object_name);
      }
    }
    catalog.observations_.push_back(descriptor);
    catalog.observations_by_name_.emplace(descriptor.name(), descriptor);
  }

  for (int actuator_id = 0; actuator_id < model.nu; ++actuator_id) {
    const char* name = mj_id2name(&model, mjOBJ_ACTUATOR, actuator_id);
    if (name == nullptr || name[0] == '\0') {
      continue;
    }

    mujoco_sim::core::ResourceDescriptor descriptor;
    descriptor.set_name(name);
    descriptor.set_type(COMMAND_ACTUATOR);
    descriptor.set_dimension(1);
    descriptor.set_mujoco_id(actuator_id);
    descriptor.set_mujoco_type(mjOBJ_ACTUATOR);
    descriptor.set_mujoco_datatype(0);
    const bool limited = model.actuator_ctrllimited[actuator_id] != 0;
    descriptor.set_has_ctrl_range(limited);
    if (limited) {
      descriptor.set_ctrl_min(model.actuator_ctrlrange[2 * actuator_id]);
      descriptor.set_ctrl_max(model.actuator_ctrlrange[2 * actuator_id + 1]);
    }
    catalog.commands_.push_back(descriptor);
    catalog.commands_by_name_.emplace(descriptor.name(), descriptor);
  }
  return catalog;
}

}  // namespace mujoco_sim::core::internal
