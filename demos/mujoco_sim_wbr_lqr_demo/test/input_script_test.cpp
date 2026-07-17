#include <gtest/gtest.h>

#include <mujoco_sim/wbr_lqr_demo/input_script.hpp>

namespace demo = mujoco_sim::wbr_lqr_demo;

TEST(InputScriptTest, UsesHalfOpenIntervals) {
  control::input_snapshot_t forward{};
  forward.w = true;
  control::input_snapshot_t toggle{};
  toggle.space = true;

  const demo::InputScript script({{10, 20, forward}, {20, 21, toggle}});
  EXPECT_FALSE(script.InputAt(9).w);
  EXPECT_TRUE(script.InputAt(10).w);
  EXPECT_TRUE(script.InputAt(19).w);
  EXPECT_FALSE(script.InputAt(20).w);
  EXPECT_TRUE(script.InputAt(20).space);
  EXPECT_FALSE(script.InputAt(21).space);
}

TEST(InputScriptTest, SortsIntervalsAndReturnsIdleInGaps) {
  control::input_snapshot_t left{};
  left.a = true;
  control::input_snapshot_t right{};
  right.d = true;

  const demo::InputScript script({{30, 40, right}, {5, 10, left}});
  EXPECT_TRUE(script.InputAt(5).a);
  EXPECT_FALSE(script.InputAt(15).a);
  EXPECT_TRUE(script.InputAt(30).d);
}

TEST(InputScriptTest, RejectsEmptyAndOverlappingIntervals) {
  EXPECT_THROW((demo::InputScript({{4, 4, {}}})), std::invalid_argument);
  EXPECT_THROW((demo::InputScript({{4, 8, {}}, {7, 9, {}}})), std::invalid_argument);
  EXPECT_NO_THROW((demo::InputScript({{4, 8, {}}, {8, 9, {}}})));
}
