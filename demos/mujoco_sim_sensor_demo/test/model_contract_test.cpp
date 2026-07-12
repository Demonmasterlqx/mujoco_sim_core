#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/server/server_config.hpp>

namespace {

class ModelHandle {
 public:
  explicit ModelHandle(const std::filesystem::path& path) {
    char error[1024] = {};
    model_ = mj_loadXML(path.string().c_str(), nullptr, error, sizeof(error));
    error_ = error;
  }
  ~ModelHandle() { mj_deleteModel(model_); }
  mjModel* get() const { return model_; }
  const std::string& error() const { return error_; }

 private:
  mjModel* model_ = nullptr;
  std::string error_;
};

class TemporaryModelFile {
 public:
  explicit TemporaryModelFile(const std::string& contents)
      : path_(std::filesystem::temp_directory_path() /
              ("mujoco_sim_closed_roof_" +
               std::to_string(std::chrono::steady_clock::now()
                                  .time_since_epoch()
                                  .count()) +
               ".xml")) {
    std::ofstream output(path_);
    output << contents;
  }

  ~TemporaryModelFile() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

struct ArenaBounds {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double floor_top_z = 0.0;
};

using FootprintPoint = std::array<double, 2>;

std::vector<FootprintPoint> ClipFootprint(
    const std::vector<FootprintPoint>& polygon, int axis, double boundary,
    bool keep_greater) {
  std::vector<FootprintPoint> clipped;
  if (polygon.empty()) return clipped;
  const auto inside = [=](const FootprintPoint& point) {
    return keep_greater ? point[axis] >= boundary : point[axis] <= boundary;
  };
  const auto intersection = [=](const FootprintPoint& first,
                                const FootprintPoint& second) {
    const double fraction =
        (boundary - first[axis]) / (second[axis] - first[axis]);
    FootprintPoint point = first;
    point[axis] = boundary;
    point[1 - axis] +=
        fraction * (second[1 - axis] - first[1 - axis]);
    return point;
  };

  FootprintPoint previous = polygon.back();
  bool previous_inside = inside(previous);
  for (const FootprintPoint& current : polygon) {
    const bool current_inside = inside(current);
    if (current_inside != previous_inside) {
      clipped.push_back(intersection(previous, current));
    }
    if (current_inside) clipped.push_back(current);
    previous = current;
    previous_inside = current_inside;
  }
  return clipped;
}

double ArenaOverlapArea(const std::array<FootprintPoint, 4>& footprint,
                        const ArenaBounds& arena) {
  std::vector<FootprintPoint> polygon(footprint.begin(), footprint.end());
  polygon = ClipFootprint(polygon, 0, arena.min_x, true);
  polygon = ClipFootprint(polygon, 0, arena.max_x, false);
  polygon = ClipFootprint(polygon, 1, arena.min_y, true);
  polygon = ClipFootprint(polygon, 1, arena.max_y, false);
  if (polygon.size() < 3) return 0.0;

  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto& current = polygon[index];
    const auto& next = polygon[(index + 1) % polygon.size()];
    twice_area += current[0] * next[1] - current[1] * next[0];
  }
  return 0.5 * std::abs(twice_area);
}

bool IsLargeOverheadCover(const mjModel* model, const mjData* data,
                          int geom_id, const ArenaBounds& arena) {
  const int body_id = model->geom_bodyid[geom_id];
  if (model->body_weldid[body_id] != 0) return false;

  const int geom_type = model->geom_type[geom_id];
  if (geom_type != mjGEOM_BOX && geom_type != mjGEOM_PLANE) return false;

  const mjtNum* rotation = data->geom_xmat + 9 * geom_id;
  constexpr double kHorizontalNormalThreshold = 0.99;
  const double center_z = data->geom_xpos[3 * geom_id + 2];
  constexpr double kFloorSeparationTolerance = 1e-6;
  if (geom_type == mjGEOM_PLANE) {
    return std::abs(rotation[8]) >= kHorizontalNormalThreshold &&
           center_z > arena.floor_top_z + kFloorSeparationTolerance;
  }

  int vertical_axis = 0;
  for (int axis = 1; axis < 3; ++axis) {
    if (std::abs(rotation[6 + axis]) >
        std::abs(rotation[6 + vertical_axis])) {
      vertical_axis = axis;
    }
  }
  const double vertical_normal = std::abs(rotation[6 + vertical_axis]);
  if (vertical_normal < kHorizontalNormalThreshold) return false;

  std::array<int, 2> footprint_axes{};
  int footprint_axis_count = 0;
  double vertical_half_extent = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double half_size = model->geom_size[3 * geom_id + axis];
    vertical_half_extent += std::abs(rotation[6 + axis]) * half_size;
    if (axis != vertical_axis) {
      footprint_axes[footprint_axis_count++] = axis;
    }
  }
  if (center_z - vertical_half_extent <=
      arena.floor_top_z + kFloorSeparationTolerance) {
    return false;
  }

  const int first_axis = footprint_axes[0];
  const int second_axis = footprint_axes[1];
  const double first_half_size =
      model->geom_size[3 * geom_id + first_axis];
  const double second_half_size =
      model->geom_size[3 * geom_id + second_axis];
  const double center_x = data->geom_xpos[3 * geom_id];
  const double center_y = data->geom_xpos[3 * geom_id + 1];
  const double first_x = rotation[first_axis] * first_half_size;
  const double first_y = rotation[3 + first_axis] * first_half_size;
  const double second_x = rotation[second_axis] * second_half_size;
  const double second_y = rotation[3 + second_axis] * second_half_size;
  const std::array<FootprintPoint, 4> footprint = {{
      {center_x - first_x - second_x, center_y - first_y - second_y},
      {center_x + first_x - second_x, center_y + first_y - second_y},
      {center_x + first_x + second_x, center_y + first_y + second_y},
      {center_x - first_x + second_x, center_y - first_y + second_y},
  }};
  const double arena_area =
      (arena.max_x - arena.min_x) * (arena.max_y - arena.min_y);
  const double projected_footprint_area =
      4.0 * first_half_size * second_half_size * vertical_normal;
  constexpr double kMajorArenaFraction = 0.5;
  return projected_footprint_area >= kMajorArenaFraction * arena_area &&
         ArenaOverlapArea(footprint, arena) >=
             kMajorArenaFraction * arena_area;
}

}  // namespace

TEST(ModelContractTest, RoofDetectorRejectsRenamedArenaCoverAtWallTop) {
  TemporaryModelFile file(R"(
<mujoco model="closed_arena">
  <worldbody>
    <geom name="west" type="box" pos="-6.1 0 1.5" size="0.1 4 1.5"/>
    <geom name="east" type="box" pos="6.1 0 1.5" size="0.1 4 1.5"/>
    <geom name="south" type="box" pos="0 -4.1 1.5" size="6 0.1 1.5"/>
    <geom name="north" type="box" pos="0 4.1 1.5" size="6 0.1 1.5"/>
    <geom name="weather_shield" type="box" pos="0 0 3" size="6 4 0.1"/>
  </worldbody>
</mujoco>
)");
  ModelHandle model(file.path());
  ASSERT_NE(model.get(), nullptr) << model.error();
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  mj_forward(model.get(), data.get());
  const int roof =
      mj_name2id(model.get(), mjOBJ_GEOM, "weather_shield");
  ASSERT_GE(roof, 0);
  const ArenaBounds arena{-6.0, 6.0, -4.0, 4.0, 0.0};
  EXPECT_TRUE(IsLargeOverheadCover(model.get(), data.get(), roof, arena));
}

TEST(ModelContractTest, RoofDetectorRejectsCoverBelowWallTop) {
  TemporaryModelFile file(R"(
<mujoco model="low_closed_arena">
  <worldbody>
    <geom name="west" type="box" pos="-6.1 0 1.5" size="0.1 4 1.5"/>
    <geom name="east" type="box" pos="6.1 0 1.5" size="0.1 4 1.5"/>
    <geom name="south" type="box" pos="0 -4.1 1.5" size="6 0.1 1.5"/>
    <geom name="north" type="box" pos="0 4.1 1.5" size="6 0.1 1.5"/>
    <geom name="low_shield" type="box" pos="0 0 2" size="6 4 0.1"/>
  </worldbody>
</mujoco>
)");
  ModelHandle model(file.path());
  ASSERT_NE(model.get(), nullptr) << model.error();
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  mj_forward(model.get(), data.get());
  const int roof = mj_name2id(model.get(), mjOBJ_GEOM, "low_shield");
  ASSERT_GE(roof, 0);
  const ArenaBounds arena{-6.0, 6.0, -4.0, 4.0, 0.0};
  EXPECT_TRUE(IsLargeOverheadCover(model.get(), data.get(), roof, arena));
}

TEST(ModelContractTest, RoofDetectorDoesNotTreatRotatedBeamAsArenaCover) {
  TemporaryModelFile file(R"(
<mujoco model="open_arena_with_beam">
  <worldbody>
    <geom name="diagonal_beam" type="box" pos="0 0 2" size="12 1 0.1"
          euler="0 0 45"/>
  </worldbody>
</mujoco>
)");
  ModelHandle model(file.path());
  ASSERT_NE(model.get(), nullptr) << model.error();
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  mj_forward(model.get(), data.get());
  const int beam = mj_name2id(model.get(), mjOBJ_GEOM, "diagonal_beam");
  ASSERT_GE(beam, 0);
  const ArenaBounds arena{-6.0, 6.0, -4.0, 4.0, 0.0};
  EXPECT_FALSE(IsLargeOverheadCover(model.get(), data.get(), beam, arena));
}

TEST(ModelContractTest, RoomVehicleCameraAndLidarSourcesAreNamedAndStable) {
  const std::filesystem::path scene =
      std::filesystem::path(SENSOR_DEMO_MJCF_DIR) / "scene.xml";
  ModelHandle model(scene);
  ASSERT_NE(model.get(), nullptr) << model.error();
  EXPECT_DOUBLE_EQ(model.get()->opt.timestep, 0.01);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM, "floor"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM, "wall_west"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM, "wall_east"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM, "wall_south"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM, "wall_north"), 0);
  const int west = mj_name2id(model.get(), mjOBJ_GEOM, "wall_west");
  const int east = mj_name2id(model.get(), mjOBJ_GEOM, "wall_east");
  const int south = mj_name2id(model.get(), mjOBJ_GEOM, "wall_south");
  const int north = mj_name2id(model.get(), mjOBJ_GEOM, "wall_north");
  const int floor = mj_name2id(model.get(), mjOBJ_GEOM, "floor");
  ASSERT_GE(west, 0);
  ASSERT_GE(east, 0);
  ASSERT_GE(south, 0);
  ASSERT_GE(north, 0);
  ASSERT_GE(floor, 0);
  EXPECT_DOUBLE_EQ(model.get()->geom_pos[3 * west] +
                       model.get()->geom_size[3 * west],
                   -6.0);
  EXPECT_DOUBLE_EQ(model.get()->geom_pos[3 * east] -
                       model.get()->geom_size[3 * east],
                   6.0);
  EXPECT_DOUBLE_EQ(model.get()->geom_pos[3 * south + 1] +
                       model.get()->geom_size[3 * south + 1],
                   -4.0);
  EXPECT_DOUBLE_EQ(model.get()->geom_pos[3 * north + 1] -
                       model.get()->geom_size[3 * north + 1],
                   4.0);
  EXPECT_DOUBLE_EQ(model.get()->geom_pos[3 * floor + 2] +
                       model.get()->geom_size[3 * floor + 2],
                   0.0);

  const std::array<int, 4> walls = {west, east, south, north};
  const double wall_top_z = model.get()->geom_pos[3 * west + 2] +
                            model.get()->geom_size[3 * west + 2];
  for (const int wall : walls) {
    EXPECT_DOUBLE_EQ(model.get()->geom_pos[3 * wall + 2] +
                         model.get()->geom_size[3 * wall + 2],
                     wall_top_z);
  }

  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  mj_forward(model.get(), data.get());
  ArenaBounds arena;
  arena.min_x = model.get()->geom_pos[3 * west] +
                model.get()->geom_size[3 * west];
  arena.max_x = model.get()->geom_pos[3 * east] -
                model.get()->geom_size[3 * east];
  arena.min_y = model.get()->geom_pos[3 * south + 1] +
                model.get()->geom_size[3 * south + 1];
  arena.max_y = model.get()->geom_pos[3 * north + 1] -
                model.get()->geom_size[3 * north + 1];
  arena.floor_top_z = model.get()->geom_pos[3 * floor + 2] +
                      model.get()->geom_size[3 * floor + 2];
  for (int geom_id = 0; geom_id < model.get()->ngeom; ++geom_id) {
    const char* geom_name =
        mj_id2name(model.get(), mjOBJ_GEOM, geom_id);
    EXPECT_FALSE(
        IsLargeOverheadCover(model.get(), data.get(), geom_id, arena))
        << "static geom " << (geom_name == nullptr ? "<unnamed>" : geom_name)
        << " closes the open-top arena";
  }

  for (int index = 1; index <= 5; ++index) {
    EXPECT_GE(mj_name2id(model.get(), mjOBJ_BODY,
                         ("column_" + std::to_string(index)).c_str()),
              0);
    for (int side = 0; side < 6; ++side) {
      EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM,
                           ("column_" + std::to_string(index) + "_side_" +
                            std::to_string(side))
                               .c_str()),
                0);
    }
  }
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_BODY, "base_link"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_CAMERA, "front_camera"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_SITE, "lidar_site"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_SITE, "imu_site"), 0);
}

TEST(ModelContractTest, ExposesExpectedSmallObservationsAndFourWheelCommands) {
  const std::filesystem::path scene =
      std::filesystem::path(SENSOR_DEMO_MJCF_DIR) / "scene.xml";
  ModelHandle model(scene);
  ASSERT_NE(model.get(), nullptr) << model.error();
  ASSERT_EQ(model.get()->nsensor, 12);
  ASSERT_EQ(model.get()->nu, 4);

  const std::unordered_map<std::string, int> expected_dimensions = {
      {"front_left_wheel_pos", 1},  {"front_left_wheel_vel", 1},
      {"front_right_wheel_pos", 1}, {"front_right_wheel_vel", 1},
      {"rear_left_wheel_pos", 1},   {"rear_left_wheel_vel", 1},
      {"rear_right_wheel_pos", 1},  {"rear_right_wheel_vel", 1},
      {"imu_accel", 3},             {"imu_gyro", 3},
      {"imu_quat", 4},            {"base_position", 3}};
  for (const auto& [name, dimension] : expected_dimensions) {
    const int sensor = mj_name2id(model.get(), mjOBJ_SENSOR, name.c_str());
    ASSERT_GE(sensor, 0) << name;
    EXPECT_EQ(model.get()->sensor_dim[sensor], dimension) << name;
  }

  const std::array<std::string, 4> commands = {
      "front_left_wheel_actuator", "front_right_wheel_actuator",
      "rear_left_wheel_actuator", "rear_right_wheel_actuator"};
  for (std::size_t index = 0; index < commands.size(); ++index) {
    const int actuator =
        mj_name2id(model.get(), mjOBJ_ACTUATOR, commands[index].c_str());
    ASSERT_EQ(actuator, static_cast<int>(index));
    EXPECT_TRUE(model.get()->actuator_ctrllimited[actuator]);
    EXPECT_DOUBLE_EQ(model.get()->actuator_ctrlrange[2 * actuator], -12.0);
    EXPECT_DOUBLE_EQ(model.get()->actuator_ctrlrange[2 * actuator + 1], 12.0);
  }
}

TEST(ModelContractTest, CoreCatalogMatchesModelAndServerStreamCatalog) {
  const std::filesystem::path scene =
      std::filesystem::path(SENSOR_DEMO_MJCF_DIR) / "scene.xml";
  mujoco_sim::core::SimulationCore core;
  std::string error;
  ASSERT_TRUE(core.LoadModel(scene.string(), &error)) << error;
  EXPECT_EQ(core.observations().size(), 12U);
  EXPECT_EQ(core.commands().size(), 4U);
  EXPECT_DOUBLE_EQ(core.timestep(), 0.01);
  auto snapshot = core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;

  mujoco_sim::server::ServerConfig server_config;
  ASSERT_TRUE(mujoco_sim::server::LoadConfigFile(
      SENSOR_DEMO_CONFIG_DIR "/sensor_demo_server.yaml", &server_config,
      &error))
      << error;
  EXPECT_EQ(server_config.sensor_streams.size(), 5U);
  EXPECT_TRUE(server_config.headless);
  EXPECT_EQ(server_config.render_backend, "auto");
  EXPECT_EQ(server_config.max_ticks, 0U);
  EXPECT_EQ(std::filesystem::weakly_canonical(server_config.model_path),
            std::filesystem::weakly_canonical(scene));
  EXPECT_EQ(server_config.sensor_streams.back().vertical_samples, 16U);
  EXPECT_EQ(server_config.sensor_streams.back().kind, "lidar_pointcloud");
}
