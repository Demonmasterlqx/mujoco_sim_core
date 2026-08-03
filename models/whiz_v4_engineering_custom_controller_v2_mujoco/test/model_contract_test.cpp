#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

namespace {

using ModelHandle = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataHandle = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr std::array<std::string_view, 7> kJointNames = {"j0", "j1", "j2", "j3", "j4", "j5", "j6"};
constexpr std::array<std::array<double, 2>, 7> kJointRanges = {{{-0.3, 3.14159},
                                                                {-0.1, 1.7},
                                                                {-0.1, 3.14159},
                                                                {-1.4, 3.14159},
                                                                {-0.5, 1.7},
                                                                {-1.05, 1.9},
                                                                {-3.14159, 3.14159}}};
constexpr std::array<double, 7> kTorqueControllerKp = {1.0, 2.0, 2.0, 0.5, 0.5, 0.3, 0.1};
constexpr std::array<double, 7> kTorqueControllerKd = {0.05, 0.1, 0.1, 0.03, 0.03, 0.02, 0.01};

ModelHandle LoadModel() {
  char error[1024] = {};
  const char* override_path = std::getenv("WHIZ_MODEL_PATH_OVERRIDE");
  const char* path = override_path == nullptr ? WHIZ_MODEL_PATH : override_path;
  auto* model = mj_loadXML(path, nullptr, error, sizeof(error));
  EXPECT_NE(model, nullptr) << error;
  return ModelHandle(model, mj_deleteModel);
}

bool HasWarning(const mjData& data) {
  for (int warning = 0; warning < mjNWARNING; ++warning) {
    if (data.warning[warning].number != 0) return true;
  }
  return false;
}

void ApplyTorqueController(const mjModel& model, mjData* data,
                           const std::array<double, 7>& targets) {
  mj_forward(&model, data);
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    const int joint = mj_name2id(&model, mjOBJ_JOINT, kJointNames[index].data());
    const std::string actuator_name = std::string(kJointNames[index]) + "_actuator";
    const int actuator = mj_name2id(&model, mjOBJ_ACTUATOR, actuator_name.c_str());
    ASSERT_GE(joint, 0);
    ASSERT_GE(actuator, 0);
    const int qpos = model.jnt_qposadr[joint];
    const int dof = model.jnt_dofadr[joint];
    const double effort = data->qfrc_bias[dof] +
                          kTorqueControllerKp[index] * (targets[index] - data->qpos[qpos]) -
                          kTorqueControllerKd[index] * data->qvel[dof];
    data->ctrl[actuator] = std::clamp(effort, -10.0, 10.0);
  }
}

TEST(ModelContractTest, LoadsBoundedTorqueMotorContract) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);

  EXPECT_EQ(model->nq, 7);
  EXPECT_EQ(model->nv, 7);
  EXPECT_EQ(model->nu, 7);
  EXPECT_EQ(model->nsensor, 21);
  EXPECT_EQ(model->opt.integrator, mjINT_IMPLICITFAST);
  EXPECT_DOUBLE_EQ(model->opt.timestep, 0.002);

  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, kJointNames[index].data());
    const std::string actuator_name = std::string(kJointNames[index]) + "_actuator";
    const int actuator = mj_name2id(model.get(), mjOBJ_ACTUATOR, actuator_name.c_str());
    ASSERT_GE(joint, 0) << kJointNames[index];
    ASSERT_GE(actuator, 0) << actuator_name;
    EXPECT_TRUE(model->jnt_limited[joint]);
    EXPECT_DOUBLE_EQ(model->jnt_range[2 * joint], kJointRanges[index][0]);
    EXPECT_DOUBLE_EQ(model->jnt_range[2 * joint + 1], kJointRanges[index][1]);
    EXPECT_GT(model->dof_damping[model->jnt_dofadr[joint]], 0.0);
    EXPECT_GT(model->dof_armature[model->jnt_dofadr[joint]], 0.0);
    EXPECT_TRUE(model->actuator_ctrllimited[actuator]);
    EXPECT_TRUE(model->actuator_forcelimited[actuator]);
    EXPECT_DOUBLE_EQ(model->actuator_ctrlrange[2 * actuator], -10.0);
    EXPECT_DOUBLE_EQ(model->actuator_ctrlrange[2 * actuator + 1], 10.0);
    EXPECT_DOUBLE_EQ(model->actuator_forcerange[2 * actuator], -10.0);
    EXPECT_DOUBLE_EQ(model->actuator_forcerange[2 * actuator + 1], 10.0);
    EXPECT_EQ(model->actuator_gaintype[actuator], mjGAIN_FIXED);
    EXPECT_EQ(model->actuator_biastype[actuator], mjBIAS_NONE);
    EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN * actuator], 1.0);
    EXPECT_DOUBLE_EQ(model->actuator_gear[6 * actuator], 1.0);
  }
}

TEST(ModelContractTest, UsesVisualMeshesAndPrimitiveMovingCollisions) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);

  constexpr std::array<std::string_view, 8> kLinkNames = {
      "base_link", "j0_link", "j1_link", "j2_link", "j3_link", "j4_link", "j5_link", "j6_link"};
  constexpr std::array<int, 8> kCollisionGeomCounts = {7, 1, 3, 4, 3, 4, 4, 2};
  EXPECT_EQ(model->nexclude, 0);

  for (std::size_t link_index = 0; link_index < kLinkNames.size(); ++link_index) {
    const std::string_view link = kLinkNames[link_index];
    const std::string visual_name = std::string(link) + "_visual";
    const std::string collision_name = std::string(link) + "_collision";
    const int body = mj_name2id(model.get(), mjOBJ_BODY, link.data());
    const int visual = mj_name2id(model.get(), mjOBJ_GEOM, visual_name.c_str());
    const int collision = mj_name2id(model.get(), mjOBJ_GEOM, collision_name.c_str());
    ASSERT_GE(body, 0) << link;
    ASSERT_GE(visual, 0) << visual_name;
    ASSERT_GE(collision, 0) << collision_name;
    EXPECT_EQ(model->geom_type[visual], mjGEOM_MESH);
    EXPECT_EQ(model->geom_contype[visual], 0);
    EXPECT_EQ(model->geom_conaffinity[visual], 0);

    int collision_count = 0;
    for (int geom = 0; geom < model->ngeom; ++geom) {
      if (model->geom_bodyid[geom] != body || model->geom_contype[geom] == 0) continue;
      ++collision_count;
      EXPECT_NE(model->geom_conaffinity[geom], 0);
      EXPECT_NE(model->geom_type[geom], mjGEOM_MESH);
      EXPECT_TRUE(
          model->geom_type[geom] == mjGEOM_BOX || model->geom_type[geom] == mjGEOM_CAPSULE ||
          model->geom_type[geom] == mjGEOM_CYLINDER || model->geom_type[geom] == mjGEOM_SPHERE);
    }
    EXPECT_EQ(collision_count, kCollisionGeomCounts[link_index]) << link;
  }
}

TEST(ModelContractTest, KeepsJ6InertiaConsistent) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  const int body = mj_name2id(model.get(), mjOBJ_BODY, "j6_link");
  ASSERT_GE(body, 0);

  EXPECT_DOUBLE_EQ(model->body_mass[body], 0.00666677140172814);
  EXPECT_NEAR(model->body_ipos[3 * body], 0.0, 1.0e-12);
  EXPECT_NEAR(model->body_ipos[3 * body + 1], 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(model->body_ipos[3 * body + 2], 0.00852217820789);
  EXPECT_LT(model->body_inertia[3 * body], 5.0e-6);
  EXPECT_GT(model->body_inertia[3 * body + 2], 2.0e-6);
}

TEST(ModelContractTest, HomeAndSingleJointSweepsAreContactFree) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  DataHandle data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);

  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());
  EXPECT_EQ(data->ncon, 0);

  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, kJointNames[index].data());
    ASSERT_GE(joint, 0);
    for (int sample = 0; sample <= 100; ++sample) {
      mj_resetDataKeyframe(model.get(), data.get(), home);
      const double alpha = static_cast<double>(sample) / 100.0;
      data->qpos[model->jnt_qposadr[joint]] =
          (1.0 - alpha) * kJointRanges[index][0] + alpha * kJointRanges[index][1];
      mj_forward(model.get(), data.get());
      EXPECT_EQ(data->ncon, 0) << kJointNames[index] << " sample " << sample;
    }
  }
}

TEST(ModelContractTest, DistalJ1BaseCollisionIsNotHiddenByAMountExclude) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  DataHandle data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int j0 = mj_name2id(model.get(), mjOBJ_JOINT, "j0");
  const int j1 = mj_name2id(model.get(), mjOBJ_JOINT, "j1");
  const int base = mj_name2id(model.get(), mjOBJ_BODY, "base_link");
  const int j1_body = mj_name2id(model.get(), mjOBJ_BODY, "j1_link");
  const int distal = mj_name2id(model.get(), mjOBJ_GEOM, "j1_link_collision_distal");
  ASSERT_GE(j0, 0);
  ASSERT_GE(j1, 0);
  ASSERT_GE(base, 0);
  ASSERT_GE(j1_body, 0);
  ASSERT_GE(distal, 0);

  // Deliberately leave the commandable range to aim the distal link at the base. This synthetic
  // pose verifies collision filtering independently from the safe joint limits.
  data->qpos[model->jnt_qposadr[j0]] = -2.51327412287;
  data->qpos[model->jnt_qposadr[j1]] = -1.63;
  mj_forward(model.get(), data.get());

  EXPECT_TRUE(std::any_of(data->contact, data->contact + data->ncon, [&](const mjContact& contact) {
    const int first_body = model->geom_bodyid[contact.geom[0]];
    const int second_body = model->geom_bodyid[contact.geom[1]];
    const bool is_base_j1 = (first_body == base && second_body == j1_body) ||
                            (first_body == j1_body && second_body == base);
    return is_base_j1 && (contact.geom[0] == distal || contact.geom[1] == distal) &&
           contact.dist < 0.0;
  }));
}

TEST(ModelContractTest, CombinedFoldStillProducesSelfCollision) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  DataHandle data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  constexpr std::array<double, 7> kFoldedPose = {2.58767540777, 0.849301044336, -0.0785269864705,
                                                 2.44934566452, 0.812521244008, 0.0136813457085,
                                                 1.93224900419};
  std::copy(kFoldedPose.begin(), kFoldedPose.end(), data->qpos);
  mj_forward(model.get(), data.get());

  ASSERT_GT(data->ncon, 0);
  EXPECT_TRUE(std::any_of(data->contact, data->contact + data->ncon,
                          [](const mjContact& contact) { return contact.dist < 0.0; }));
}

TEST(ModelContractTest, ShallowContactRecoversWithoutNumericalInstability) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  DataHandle data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  const int j4 = mj_name2id(model.get(), mjOBJ_JOINT, "j4");
  ASSERT_GE(home, 0);
  ASSERT_GE(j4, 0);

  mj_resetDataKeyframe(model.get(), data.get(), home);
  data->qpos[model->jnt_qposadr[j4]] = 1.8;
  mju_zero(data->ctrl, model->nu);
  mj_forward(model.get(), data.get());
  ASSERT_GT(data->ncon, 0);
  ASSERT_TRUE(std::any_of(data->contact, data->contact + data->ncon,
                          [](const mjContact& contact) { return contact.dist < 0.0; }));

  double maximum_speed = 0.0;
  double minimum_contact_distance = 0.0;
  constexpr std::array<double, 7> kRecoveryTargets = {0.0, 0.0, 0.0, 0.0, 1.65, 0.0, 0.0};
  constexpr int kSteps = 1000;
  for (int step = 0; step < kSteps; ++step) {
    ApplyTorqueController(*model, data.get(), kRecoveryTargets);
    mj_step(model.get(), data.get());
    EXPECT_FALSE(HasWarning(*data)) << "step " << step;
    for (int dof = 0; dof < model->nv; ++dof) {
      ASSERT_TRUE(std::isfinite(data->qpos[dof]));
      ASSERT_TRUE(std::isfinite(data->qvel[dof]));
      maximum_speed = std::max(maximum_speed, std::abs(data->qvel[dof]));
    }
    for (int contact = 0; contact < data->ncon; ++contact) {
      minimum_contact_distance = std::min(minimum_contact_distance, data->contact[contact].dist);
    }
  }

  EXPECT_GT(minimum_contact_distance, -0.003);
  EXPECT_LT(maximum_speed, 2.5);
  EXPECT_EQ(data->ncon, 0);
  EXPECT_LE(data->qpos[model->jnt_qposadr[j4]], kJointRanges[4][1]);
}

TEST(ModelContractTest, ZeroTorqueFreeFallRemainsNumericallyStable) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  DataHandle data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mju_zero(data->ctrl, model->nu);

  double maximum_position_change = 0.0;
  double maximum_speed = 0.0;
  double maximum_acceleration = 0.0;
  double minimum_contact_distance = 0.0;
  std::array<double, 7> previous_velocity = {};
  constexpr int kSteps = 5000;
  for (int step = 0; step < kSteps; ++step) {
    mj_step(model.get(), data.get());
    EXPECT_FALSE(HasWarning(*data)) << "step " << step;
    for (int dof = 0; dof < model->nv; ++dof) {
      ASSERT_TRUE(std::isfinite(data->qpos[dof]));
      ASSERT_TRUE(std::isfinite(data->qvel[dof]));
      maximum_position_change = std::max(maximum_position_change, std::abs(data->qpos[dof]));
      maximum_speed = std::max(maximum_speed, std::abs(data->qvel[dof]));
      maximum_acceleration =
          std::max(maximum_acceleration,
                   std::abs(data->qvel[dof] - previous_velocity[dof]) / model->opt.timestep);
      previous_velocity[dof] = data->qvel[dof];
    }
    for (int contact = 0; contact < data->ncon; ++contact) {
      minimum_contact_distance = std::min(minimum_contact_distance, data->contact[contact].dist);
    }
  }

  EXPECT_GT(maximum_position_change, 0.01);
  EXPECT_LT(maximum_speed, 10.0);
  EXPECT_LT(maximum_acceleration, 1000.0);
  EXPECT_GT(minimum_contact_distance, -0.003);
  EXPECT_LT(std::abs(*std::max_element(
                previous_velocity.begin(), previous_velocity.end(),
                [](double lhs, double rhs) { return std::abs(lhs) < std::abs(rhs); })),
            0.05);
}

TEST(ModelContractTest, ExternalTorqueControllerHoldsHomeForTenSimulatedSeconds) {
  const auto model = LoadModel();
  ASSERT_NE(model, nullptr);
  DataHandle data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mju_zero(data->ctrl, model->nu);

  double maximum_position_error = 0.0;
  double maximum_speed = 0.0;
  constexpr std::array<double, 7> kHomeTargets = {};
  constexpr int kSteps = 5000;
  for (int step = 0; step < kSteps; ++step) {
    ApplyTorqueController(*model, data.get(), kHomeTargets);
    mj_step(model.get(), data.get());
    EXPECT_EQ(data->ncon, 0) << "step " << step;
    EXPECT_FALSE(HasWarning(*data)) << "step " << step;
    for (int dof = 0; dof < model->nv; ++dof) {
      ASSERT_TRUE(std::isfinite(data->qpos[dof]));
      ASSERT_TRUE(std::isfinite(data->qvel[dof]));
      maximum_position_error = std::max(maximum_position_error, std::abs(data->qpos[dof]));
      maximum_speed = std::max(maximum_speed, std::abs(data->qvel[dof]));
    }
  }

  EXPECT_NEAR(data->time, 10.0, 1.0e-12);
  EXPECT_LT(maximum_position_error, 0.015);
  EXPECT_LT(maximum_speed, 0.35);
}

}  // namespace
