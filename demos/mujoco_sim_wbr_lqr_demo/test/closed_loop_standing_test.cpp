#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

#include <mujoco_sim/wbr_lqr_demo/controller_pipeline.hpp>

namespace demo = mujoco_sim::wbr_lqr_demo;

namespace {

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

int SensorAddress(const mjModel* model, const std::string& name) {
  const int sensor = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
  if (sensor < 0) throw std::runtime_error("missing MuJoCo sensor: " + name);
  return model->sensor_adr[sensor];
}

std::array<float, 4> ReadQuaternion(const mjModel* model, const mjData* data) {
  const int address = SensorAddress(model, "imu_quat");
  return {static_cast<float>(data->sensordata[address]),
          static_cast<float>(data->sensordata[address + 1]),
          static_cast<float>(data->sensordata[address + 2]),
          static_cast<float>(data->sensordata[address + 3])};
}

std::array<float, 3> ReadVectorSensor(const mjModel* model, const mjData* data, const char* name) {
  const int address = SensorAddress(model, name);
  return {static_cast<float>(data->sensordata[address]),
          static_cast<float>(data->sensordata[address + 1]),
          static_cast<float>(data->sensordata[address + 2])};
}

float ReadScalarSensor(const mjModel* model, const mjData* data, const std::string& name) {
  return static_cast<float>(data->sensordata[SensorAddress(model, name)]);
}

control::msg_raw_state_t ReadRawState(const mjModel* model, const mjData* data) {
  control::msg_raw_state_t raw{};
  raw.time = data->time;
  const auto quaternion = ReadQuaternion(model, data);
  const auto acceleration = ReadVectorSensor(model, data, "imu_accel");
  const auto angular_velocity = ReadVectorSensor(model, data, "imu_gyro");
  std::copy(quaternion.begin(), quaternion.end(), raw.quat_gt);
  std::copy(acceleration.begin(), acceleration.end(), raw.accel);
  std::copy(angular_velocity.begin(), angular_velocity.end(), raw.gyro);

  constexpr std::array<const char*, 6> kMotors = {"ljoint1", "ljoint4", "lwheel",
                                                  "rjoint1", "rjoint4", "rwheel"};
  for (std::size_t i = 0; i < kMotors.size(); ++i) {
    const std::string prefix = kMotors[i];
    raw.motors[i].q = ReadScalarSensor(model, data, prefix + "_pos");
    raw.motors[i].dq = ReadScalarSensor(model, data, prefix + "_vel");
    raw.motors[i].tau = ReadScalarSensor(model, data, prefix + "_tor");
  }
  return raw;
}

control::input_snapshot_t StandingInput(std::uint64_t tick) {
  control::input_snapshot_t input{};
  input.space = tick == 1;
  input.e = tick == 2000;
  return input;
}

}  // namespace

TEST(ClosedLoopStandingTest, RaisesAndBalancesFromHomeKeyframe) {
  const auto scene = std::filesystem::path(WBR_LQR_MJCF_DIR) / "scene.xml";
  char error[1024] = {};
  ModelPtr model(mj_loadXML(scene.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  ASSERT_NE(model, nullptr) << error;
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);

  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  demo::ControllerPipeline pipeline;
  demo::PipelineOutput output{};
  std::uint64_t normal_ticks = 0;
  double minimum_balanced_height = 1.0;
  double maximum_balanced_pitch = 0.0;
  constexpr std::uint64_t kSimulationTicks = 6000;
  for (std::uint64_t tick = 0; tick < kSimulationTicks; ++tick) {
    mj_step(model.get(), data.get());
    output = pipeline.Step(ReadRawState(model.get(), data.get()), StandingInput(tick));
    for (std::size_t actuator = 0; actuator < output.raw_torque.size(); ++actuator) {
      const double lower = model->actuator_ctrlrange[2 * actuator];
      const double upper = model->actuator_ctrlrange[2 * actuator + 1];
      data->ctrl[actuator] =
          std::clamp(static_cast<double>(output.raw_torque[actuator]), lower, upper);
    }
    if (tick >= 5000 && output.fsm_state == control::chassis_state::normal) {
      ++normal_ticks;
      minimum_balanced_height = std::min(minimum_balanced_height, data->qpos[2]);
      maximum_balanced_pitch =
          std::max(maximum_balanced_pitch, std::fabs(static_cast<double>(output.imu.pitch)));
    }
  }

  EXPECT_EQ(output.fsm_state, control::chassis_state::normal);
  EXPECT_GT(normal_ticks, 900U);
  EXPECT_GT(minimum_balanced_height, 0.14);
  EXPECT_LT(maximum_balanced_pitch, 0.25);
  EXPECT_LT(std::fabs(data->qpos[0]), 0.50);
  EXPECT_NEAR(output.command.len, pipeline.config().chassis.lmid, 1e-6F);
}
