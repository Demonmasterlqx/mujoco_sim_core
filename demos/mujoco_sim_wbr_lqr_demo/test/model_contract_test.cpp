#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

namespace demo = mujoco_sim::wbr_lqr_demo;

namespace {

class ModelHandle {
 public:
  explicit ModelHandle(const std::string& path) {
    char error[1024] = {};
    model_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    error_ = error;
  }
  ~ModelHandle() { mj_deleteModel(model_); }

  mjModel* get() const { return model_; }
  const std::string& error() const { return error_; }

 private:
  mjModel* model_ = nullptr;
  std::string error_;
};

int ExpectedSensorDimension(std::string_view name) {
  if (name == "imu_quat") return 4;
  if (name == "imu_accel" || name == "imu_gyro") return 3;
  return 1;
}

}  // namespace

TEST(ModelContractTest, LoadsSceneWithFixedStepAndAllMeshes) {
  const std::filesystem::path mjcf_dir = WBR_LQR_MJCF_DIR;
  const ModelHandle model((mjcf_dir / "scene.xml").string());
  ASSERT_NE(model.get(), nullptr) << model.error();
  EXPECT_DOUBLE_EQ(model.get()->opt.timestep, 0.001);
  EXPECT_EQ(model.get()->nmesh, 15);

  std::size_t stl_count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(mjcf_dir / "meshes")) {
    if (entry.is_regular_file() && entry.path().extension() == ".STL") {
      ++stl_count;
    }
  }
  EXPECT_EQ(stl_count, 15U);
}

TEST(ModelContractTest, ExposesTwentyOneNamedStateResources) {
  const std::filesystem::path mjcf_dir = WBR_LQR_MJCF_DIR;
  const ModelHandle model((mjcf_dir / "scene.xml").string());
  ASSERT_NE(model.get(), nullptr) << model.error();
  ASSERT_EQ(model.get()->nsensor, 21);

  int total_dimension = 0;
  for (const auto& name : demo::WbrObservationNames()) {
    const int sensor_id = mj_name2id(model.get(), mjOBJ_SENSOR, name.c_str());
    ASSERT_GE(sensor_id, 0) << name;
    EXPECT_EQ(model.get()->sensor_dim[sensor_id], ExpectedSensorDimension(name)) << name;
    total_dimension += model.get()->sensor_dim[sensor_id];
  }
  EXPECT_EQ(total_dimension, 28);
}

TEST(ModelContractTest, PreservesSixMotorOrderAndControlRanges) {
  const std::filesystem::path mjcf_dir = WBR_LQR_MJCF_DIR;
  const ModelHandle model((mjcf_dir / "scene.xml").string());
  ASSERT_NE(model.get(), nullptr) << model.error();
  ASSERT_EQ(model.get()->nu, 6);

  const auto names = demo::WbrCommandNames();
  ASSERT_EQ(names.size(), 6U);
  for (std::size_t i = 0; i < names.size(); ++i) {
    const int actuator_id = mj_name2id(model.get(), mjOBJ_ACTUATOR, names[i].c_str());
    ASSERT_EQ(actuator_id, static_cast<int>(i)) << names[i];
    EXPECT_TRUE(model.get()->actuator_ctrllimited[actuator_id]);
    const double expected_limit = (i == 2 || i == 5) ? 5.2 : 20.0;
    EXPECT_DOUBLE_EQ(model.get()->actuator_ctrlrange[2 * actuator_id], -expected_limit);
    EXPECT_DOUBLE_EQ(model.get()->actuator_ctrlrange[2 * actuator_id + 1], expected_limit);
  }
}

TEST(ModelContractTest, CoreLoadsHomeKeyframeAndBuildsCatalogs) {
  const std::filesystem::path scene = std::filesystem::path(WBR_LQR_MJCF_DIR) / "scene.xml";
  mujoco_sim::core::SimulationCore core;
  mujoco_sim::core::ModelLoadOptions options;
  options.initial_keyframe = "home";
  std::string error;
  const auto model_load = core.LoadModel(scene.string(), options);
  ASSERT_TRUE(model_load.response.loaded) << model_load.response.message;
  const auto resources = core.resources();
  EXPECT_EQ(resources.observations.size(), 21U);
  EXPECT_EQ(resources.commands.size(), 6U);
  EXPECT_DOUBLE_EQ(core.state().simulation_time_s, 0.0);

  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  EXPECT_DOUBLE_EQ(snapshot->data()->qpos[7], -0.75);
  EXPECT_DOUBLE_EQ(snapshot->data()->qpos[14], 0.75);
  for (int i = 0; i < snapshot->model()->nu; ++i) {
    EXPECT_DOUBLE_EQ(snapshot->data()->ctrl[i], 0.0);
  }
}
