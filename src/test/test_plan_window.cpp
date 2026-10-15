// map2d::terminalGoalWindow / useTerminalGoal: the planner window for the terminal-goal A* over a large
// (composite) occ_2d grid, and when the planner uses it (ROS-free).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "map2d/map2d.hpp"

using map2d::GoalWindow;
using map2d::terminalGoalWindow;
using map2d::useTerminalGoal;

TEST(PlanWindow, CoversTheWholeGrid) {
  // a 30 m composite around the origin, robot and goal well inside it: the window is the grid, so a
  // detour anywhere in remembered space is inside the search
  const GoalWindow w = terminalGoalWindow(0, 0, 5, 5, -15, -15, 15, 15, 3.0, 15, 15);
  EXPECT_DOUBLE_EQ(w.wdx, 30.0);
  EXPECT_DOUBLE_EQ(w.wdy, 30.0);
  EXPECT_DOUBLE_EQ(w.center_x, 0.0);
  EXPECT_DOUBLE_EQ(w.center_y, 0.0);
}

TEST(PlanWindow, RobotNearTheEdgeAddsTheMargin) {
  const GoalWindow w = terminalGoalWindow(14, 0, 0, 0, -15, -15, 15, 15, 3.0, 15, 15);
  EXPECT_DOUBLE_EQ(w.wdx, 32.0);  // [-15, 17]
  EXPECT_DOUBLE_EQ(w.center_x, 1.0);
  EXPECT_DOUBLE_EQ(w.wdy, 30.0);
  EXPECT_DOUBLE_EQ(w.center_y, 0.0);
}

TEST(PlanWindow, SmallGridUsesTheMinWindow) {
  const GoalWindow w = terminalGoalWindow(0, 0, 1, 0, -2, -2, 2, 2, 0.5, 15, 12);
  EXPECT_DOUBLE_EQ(w.wdx, 15.0);
  EXPECT_DOUBLE_EQ(w.wdy, 12.0);
  EXPECT_DOUBLE_EQ(w.center_x, 0.0);
  EXPECT_DOUBLE_EQ(w.center_y, 0.0);
}

TEST(PlanWindow, NegativeDirections) {
  const GoalWindow w = terminalGoalWindow(-12, -6, -38, -28, -40, -30, -10, -5, 3.0, 15, 15);
  EXPECT_DOUBLE_EQ(w.wdx, 32.0);  // [-41, -9]
  EXPECT_DOUBLE_EQ(w.center_x, -25.0);
  EXPECT_DOUBLE_EQ(w.wdy, 28.0);  // [-31, -3]
  EXPECT_DOUBLE_EQ(w.center_y, -17.0);
}

TEST(PlanWindow, GoalInsideTheGridAndUnderTheCapUsesTheTerminalGoal) {
  const GoalWindow w = terminalGoalWindow(0, 0, 5, 5, -15, -15, 15, 15, 3.0, 15, 15);
  EXPECT_TRUE(useTerminalGoal(5, 5, -15, -15, 15, 15, w, 46.0));
  EXPECT_TRUE(useTerminalGoal(5, 5, -15, -15, 15, 15, w, 0.0));  // 0 = no cap
  EXPECT_TRUE(useTerminalGoal(-15, 0, -15, -15, 15, 15, w, 46.0));
}

TEST(PlanWindow, GoalOutsideTheGridUsesTheSubgoal) {
  const GoalWindow w = terminalGoalWindow(0, 0, 16, 0, -15, -15, 15, 15, 3.0, 15, 15);
  EXPECT_FALSE(useTerminalGoal(16, 0, -15, -15, 15, 15, w, 46.0));
  EXPECT_FALSE(useTerminalGoal(15, 0, -15, -15, 15, 15, w, 46.0));  // half-open extent
  EXPECT_FALSE(useTerminalGoal(0, -15.01, -15, -15, 15, 15, w, 46.0));
}

TEST(PlanWindow, WindowOverTheCapUsesTheSubgoal) {
  // a 60 m map (another mapper, or a larger composite cap): beyond max_window_m the planner keeps
  // the straight-line subgoal
  const GoalWindow w = terminalGoalWindow(0, 0, 5, 5, -30, -30, 30, 30, 3.0, 15, 15);
  EXPECT_FALSE(useTerminalGoal(5, 5, -30, -30, 30, 30, w, 46.0));
  EXPECT_TRUE(useTerminalGoal(5, 5, -30, -30, 30, 30, w, 0.0));
}

TEST(PlanWindow, NonFiniteGoalUsesTheSubgoal) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const GoalWindow w = terminalGoalWindow(0, 0, 0, 0, -15, -15, 15, 15, 3.0, 15, 15);
  EXPECT_FALSE(useTerminalGoal(nan, 0, -15, -15, 15, 15, w, 46.0));
  EXPECT_FALSE(useTerminalGoal(0, std::numeric_limits<double>::infinity(), -15, -15, 15, 15, w, 46.0));
}
