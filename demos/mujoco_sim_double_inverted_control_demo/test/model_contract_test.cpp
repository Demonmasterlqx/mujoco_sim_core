#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <string_view>

#include <mujoco/mujoco.h>

namespace {

struct RolloutStats {
  double closest_fold_distance = std::numbers::pi;
  double minimum_contact_distance = 0.0;
  bool saw_intended_contact = false;
};

class DoubleInvertedModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto scene = std::filesystem::path(DOUBLE_INVERTED_MJCF_DIR) / "scene.xml";
    char error[1024] = {};
    model_ = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << error;
    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);

    first_collision_ = mj_name2id(model_, mjOBJ_GEOM, "first_link_collision");
    second_collision_ = mj_name2id(model_, mjOBJ_GEOM, "second_link_collision");
    tip_ = mj_name2id(model_, mjOBJ_GEOM, "pendulum_tip");
    ASSERT_GE(first_collision_, 0);
    ASSERT_GE(second_collision_, 0);
    ASSERT_GE(tip_, 0);
  }

  void TearDown() override {
    mj_deleteData(data_);
    mj_deleteModel(model_);
  }

  bool IsIntendedContact(const mjContact& contact) const {
    const bool first_hits_second =
        (contact.geom[0] == first_collision_ && contact.geom[1] == second_collision_) ||
        (contact.geom[1] == first_collision_ && contact.geom[0] == second_collision_);
    const bool first_hits_tip = (contact.geom[0] == first_collision_ && contact.geom[1] == tip_) ||
                                (contact.geom[1] == first_collision_ && contact.geom[0] == tip_);
    return first_hits_second || first_hits_tip;
  }

  RolloutStats RunFromRandomUnstable(double second_joint_torque, int steps) {
    const int keyframe = mj_name2id(model_, mjOBJ_KEY, "random_unstable");
    EXPECT_GE(keyframe, 0);
    mj_resetDataKeyframe(model_, data_, keyframe);
    data_->ctrl[1] = second_joint_torque;

    RolloutStats stats;
    for (int step = 0; step < steps; ++step) {
      mj_step(model_, data_);
      const double wrapped_q2 = std::remainder(data_->qpos[1], 2.0 * std::numbers::pi);
      stats.closest_fold_distance =
          std::min(stats.closest_fold_distance, std::numbers::pi - std::abs(wrapped_q2));
      for (int contact_index = 0; contact_index < data_->ncon; ++contact_index) {
        const auto& contact = data_->contact[contact_index];
        stats.minimum_contact_distance =
            std::min(stats.minimum_contact_distance, static_cast<double>(contact.dist));
        stats.saw_intended_contact = stats.saw_intended_contact || IsIntendedContact(contact);
      }
      EXPECT_TRUE(std::isfinite(data_->qpos[0]));
      EXPECT_TRUE(std::isfinite(data_->qpos[1]));
      EXPECT_TRUE(std::isfinite(data_->qvel[0]));
      EXPECT_TRUE(std::isfinite(data_->qvel[1]));
    }
    return stats;
  }

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  int first_collision_ = -1;
  int second_collision_ = -1;
  int tip_ = -1;
};

TEST_F(DoubleInvertedModelTest, PreservesResourcesMassAndContactFreeKeyframes) {
  EXPECT_EQ(model_->nq, 2);
  EXPECT_EQ(model_->nv, 2);
  EXPECT_EQ(model_->nu, 2);
  EXPECT_EQ(model_->nsensor, 6);
  EXPECT_EQ(model_->nkey, 3);

  const int first_body = mj_name2id(model_, mjOBJ_BODY, "first_link");
  const int second_body = mj_name2id(model_, mjOBJ_BODY, "second_link");
  ASSERT_GE(first_body, 0);
  ASSERT_GE(second_body, 0);
  EXPECT_DOUBLE_EQ(model_->body_mass[first_body], 1.70);
  EXPECT_DOUBLE_EQ(model_->body_mass[second_body], 1.28);

  constexpr std::array<std::string_view, 3> keyframes{"upright", "hanging_down", "random_unstable"};
  for (const std::string_view keyframe_name : keyframes) {
    const int keyframe = mj_name2id(model_, mjOBJ_KEY, keyframe_name.data());
    ASSERT_GE(keyframe, 0) << keyframe_name;
    mj_resetDataKeyframe(model_, data_, keyframe);
    mj_forward(model_, data_);
    EXPECT_EQ(data_->ncon, 0) << keyframe_name;
  }
}

TEST_F(DoubleInvertedModelTest, FullyFoldedLinksGenerateIntendedContact) {
  mj_resetData(model_, data_);
  data_->qpos[1] = std::numbers::pi;
  mj_forward(model_, data_);

  ASSERT_GT(data_->ncon, 0);
  bool saw_intended_contact = false;
  for (int contact_index = 0; contact_index < data_->ncon; ++contact_index) {
    saw_intended_contact = saw_intended_contact || IsIntendedContact(data_->contact[contact_index]);
  }
  EXPECT_TRUE(saw_intended_contact);
}

TEST_F(DoubleInvertedModelTest, SelfCollisionPreventsDynamicFoldThrough) {
  constexpr double kMinimumFoldDistance = 0.2;
  constexpr double kMaximumPenetration = 0.002;
  constexpr int kFreeRolloutSteps = 30000;
  constexpr int kFullTorqueRolloutSteps = 10000;

  const auto free_rollout = RunFromRandomUnstable(0.0, kFreeRolloutSteps);
  EXPECT_TRUE(free_rollout.saw_intended_contact);
  EXPECT_GT(free_rollout.closest_fold_distance, kMinimumFoldDistance);

  for (const double torque : {-8.0, 8.0}) {
    const auto forced_rollout = RunFromRandomUnstable(torque, kFullTorqueRolloutSteps);
    EXPECT_TRUE(forced_rollout.saw_intended_contact) << torque;
    EXPECT_GT(forced_rollout.closest_fold_distance, kMinimumFoldDistance) << torque;
    EXPECT_GE(forced_rollout.minimum_contact_distance, -kMaximumPenetration) << torque;
  }
}

}  // namespace
