// Planner "New Plan" tests: 2D global path, goal handling and PARTIAL.
//
// Source of truth: goal_selector_refactor_spec.md, "## New Plan" and "New Plan: detailed design" N2
// (incl. Answer 5), section 11 items 13 and 15, section 12.3; goal_selector_test_ideas.md, section
// "E. Planner changes".
//
// All tests drive the real planner through HGPManager::solveHGP (see planner_test_fixture.hpp) on a
// hand-built tri-state 2D grid (free 0, unknown -1, occupied 100). Status mapping used here, the same
// one mighty.cpp:735-743 applies:
//   solveHGP() == false                      -> FAILED
//   solveHGP() == true,  reachedGoal() true  -> SUCCESS
//   solveHGP() == true,  reachedGoal() false -> PARTIAL (A* fell back to the best node)
//
//   Test                                          Spec / notes item
//   GlobalPathCrossesUnknownWhenNoFreeRoute       N2 "A* ... unknown cells cost w_unknown"; E "global paths enter unknown"
//   UnknownCrossingIsPricedByWUnknown             N2 w_unknown = extra cost per unknown step
//   FreeDetourBeatsPricedUnknown                  New Plan "extra high cost for traversing unknown"
//   GoalInUnknownIsPlannable                      N2 Answer 5; 11.13; E "goal in unknown is plannable"
//   GoalOnOccupiedCellFails                       N2, 11.15; E "goal on occupied -> FAILED, never freed"
//   GoalInOccupiedBandFails                       N2, 11.15; E "goal in occupied band -> FAILED"
//   GoalJustOutsideBandIsPlannable                N2 (only band cells are blocked)
//   GoalEnclosedByWallsIsPartial                  E "PARTIAL only when goal cannot be reached" (closed room)
//   GoalBehindUnknownIsNotPartial                 E "PARTIAL ... not when reachable through unknown"
//   OpenGoalIsSuccessNotPartial                   status baseline
//   GlobalPathReachesGoalEvenIfPrefixIsCut        N2/N3 "global path reaches goal, executed path is a prefix"

#include <gtest/gtest.h>

#include <cmath>

#include "planner_test_fixture.hpp"

using plannertest::Knobs;
using plannertest::PlannerWorld;

namespace {

/** True if any point of `path` lies in the cell box [x0,x1] x [y0,y1]. */
bool PathTouchesBox(const PlannerWorld& w, const vec_Vecf<3>& path, int x0, int x1, int y0, int y1) {
  for (const auto& p : path) {
    const Veci<3> c = w.cell(p);
    if (c(0) >= x0 && c(0) <= x1 && c(1) >= y0 && c(1) <= y1) return true;
  }
  return false;
}

}  // namespace

// Scenario: a full-height unknown band separates start and goal; no free route exists.
// Expected: the planner still plans THROUGH the band and reaches the goal (SUCCESS).
// Why: N2 - unknown is traversable on the global map at w_unknown cost, never blocked.
TEST(PlannerUnknownPaths, GlobalPathCrossesUnknownWhenNoFreeRoute) {
  PlannerWorld w;
  w.fillColumns(45, 54, -1);
  w.build(Knobs{});
  auto r = w.solve(10, 50, 90, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  EXPECT_TRUE(PathTouchesBox(w, r.raw_path, 45, 54, 0, 99)) << "global path must go through the unknown band";
  const Veci<3> last = w.cell(r.raw_path.back());
  EXPECT_EQ(last(0), 90);
  EXPECT_EQ(last(1), 50);
}

// Scenario: same forced 10-cell unknown crossing planned with w_unknown = 0, 1 and 2.
// Expected: the extra cost over the free baseline scales with w_unknown (delta(2) ~= 2 * delta(1)),
//   and delta(1) > 0.
// Why: N2 - "Unknown cells cost w_unknown per step" (extra cost, multiples of the geometric step).
// ASSUMPTION: only the ratio of extra costs is checked, not absolute units (final_g unit is the
//   planner's, cell- or metre-based).
TEST(PlannerUnknownPaths, UnknownCrossingIsPricedByWUnknown) {
  double g[3];
  const double weights[3] = {0.0, 1.0, 2.0};
  for (int i = 0; i < 3; ++i) {
    PlannerWorld w;
    w.fillColumns(45, 54, -1);
    Knobs k;
    k.w_unknown = weights[i];
    w.build(k);
    auto r = w.solve(10, 50, 90, 50);
    ASSERT_TRUE(r.ok && r.reached_goal);
    g[i] = r.final_g;
  }
  const double d1 = g[1] - g[0];
  const double d2 = g[2] - g[0];
  EXPECT_GT(d1, 0.0) << "unknown must cost extra";
  EXPECT_NEAR(d2 / d1, 2.0, 0.1) << "extra cost must scale linearly with w_unknown";
}

// Scenario: an unknown block sits on the straight line, but a modest free detour exists.
// Expected: with w_unknown = 2 the path avoids the unknown block entirely.
// Why: New Plan - "extra high cost for traversing unknown"; unknown is the last resort.
TEST(PlannerUnknownPaths, FreeDetourBeatsPricedUnknown) {
  PlannerWorld w;
  for (int y = 40; y <= 60; ++y)
    for (int x = 40; x <= 60; ++x) w.at(x, y) = -1;
  w.build(Knobs{});
  auto r = w.solve(10, 50, 90, 50);
  ASSERT_TRUE(r.ok && r.reached_goal);
  EXPECT_FALSE(PathTouchesBox(w, r.raw_path, 40, 60, 40, 60));
}

// Scenario: the goal cell lies inside a large unknown region (beyond the mapper's coverage).
// Expected: plannable; SUCCESS with the global path ending exactly on the goal cell; the planner does
//   not move or free the goal.
// Why: N2 Answer 5 / 11.13 - "As long as the planner can plan a path to a goal in unknown space, no
//   need to deliberately free the goal cells."
TEST(PlannerUnknownPaths, GoalInUnknownIsPlannable) {
  PlannerWorld w;
  w.fillColumns(70, 99, -1);
  w.build(Knobs{});
  auto r = w.solve(10, 50, 85, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  const Veci<3> last = w.cell(r.raw_path.back());
  EXPECT_EQ(last(0), 85);
  EXPECT_EQ(last(1), 50);
  ASSERT_TRUE(w.mgr.map_util_for_planning_->has2DMap());
  EXPECT_TRUE(w.mgr.map_util_for_planning_->is2DUnknown(85, 50)) << "goal cell must stay unknown (not freed)";
}

// Scenario: the goal is on an occupied cell (a wall pixel).
// Expected: FAILED (solveHGP false), and the occupied cells are still occupied afterwards.
// Why: N2 / 11.15 - a goal on an occupied cell fails; the planner never frees or moves the goal.
TEST(PlannerUnknownPaths, GoalOnOccupiedCellFails) {
  PlannerWorld w;
  for (int y = 40; y <= 60; ++y)
    for (int x = 80; x <= 82; ++x) w.at(x, y) = 100;
  w.build(Knobs{});
  auto r = w.solve(10, 50, 81, 50);
  EXPECT_FALSE(r.ok) << "goal on an occupied cell must give FAILED";
  EXPECT_TRUE(w.mgr.map_util_for_planning_->is2DOccupied(81, 50)) << "goal cell must not be freed";
  EXPECT_TRUE(w.mgr.map_util_for_planning_->is2DOccupied(80, 50)) << "goal surroundings must not be freed";
}

// Scenario: occupied-band inflation is on (0.3 m); the goal is a band cell next to a wall, not the
//   wall itself.
// Expected: FAILED, and the band cell stays blocked.
// Why: N2 / 11.15 - "A goal in an occupied or occupied-band cell fails (FAILED)".
TEST(PlannerUnknownPaths, GoalInOccupiedBandFails) {
  PlannerWorld w;
  w.fillColumns(80, 82, 100);
  Knobs k;
  k.inflation_2d_m = 0.3;  // band = 3 cells around the wall: x = 77..79 and 83..85
  w.build(k);
  ASSERT_TRUE(w.mu->is2DInflatedOnly(78, 50)) << "test setup: (78,50) must be an inflation-only cell";
  auto r = w.solve(10, 50, 78, 50);
  EXPECT_FALSE(r.ok) << "goal in the occupied band must give FAILED";
  EXPECT_TRUE(w.mgr.map_util_for_planning_->is2DOccupied(78, 50)) << "band cell must not be freed";
}

// Scenario: same wall and band, goal well outside the band (8 cells from the wall; the band radius
//   0.3 m quantises to 3-4 cells).
// Expected: plannable (SUCCESS).
// Why: N2 - only occupied and band cells block; the band edge itself is free.
TEST(PlannerUnknownPaths, GoalJustOutsideBandIsPlannable) {
  PlannerWorld w;
  w.fillColumns(80, 82, 100);
  Knobs k;
  k.inflation_2d_m = 0.3;
  w.build(k);
  ASSERT_FALSE(w.mu->is2DOccupied(72, 50)) << "test setup: (72,50) must be outside the band";
  auto r = w.solve(10, 50, 72, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
}

// Scenario: goal inside a closed room (one-cell-thick occupied ring), start outside.
// Expected: PARTIAL - solveHGP true (a fallback path to the best node), reachedGoal() false, and the
//   path ends closer to the goal than the start did (best node), not at the goal.
// Why: E - "PARTIAL: only when the goal cannot be reached through free or unknown space (e.g. inside a
//   closed room); A* falls back to the best node".
TEST(PlannerUnknownPaths, GoalEnclosedByWallsIsPartial) {
  PlannerWorld w;
  for (int i = 60; i <= 80; ++i) {  // ring around the interior 61..79 x 61..79
    w.at(i, 60) = 100;
    w.at(i, 80) = 100;
    w.at(60, i) = 100;
    w.at(80, i) = 100;
  }
  w.build(Knobs{});
  auto r = w.solve(10, 10, 70, 70);
  ASSERT_TRUE(r.ok) << "a fallback path must still be returned (PARTIAL, not FAILED)";
  EXPECT_FALSE(r.reached_goal);
  ASSERT_GE(r.raw_path.size(), 2u);
  const Vec3f goal = w.world(70, 70);
  const double d_start = (w.world(10, 10) - goal).head<2>().norm();
  const double d_end = (r.raw_path.back() - goal).head<2>().norm();
  EXPECT_LT(d_end, d_start) << "fallback path must end at the node closest to the goal";
  const Veci<3> e = w.cell(r.raw_path.back());
  EXPECT_FALSE(e(0) > 60 && e(0) < 80 && e(1) > 60 && e(1) < 80) << "path cannot be inside the closed room";
}

// Scenario: the goal is behind an unknown band (no free route, but unknown is passable).
// Expected: SUCCESS (reached goal), NOT PARTIAL.
// Why: E - PARTIAL no longer happens when the goal is reachable through unknown space.
TEST(PlannerUnknownPaths, GoalBehindUnknownIsNotPartial) {
  PlannerWorld w;
  w.fillColumns(45, 54, -1);
  w.build(Knobs{});
  auto r = w.solve(10, 50, 90, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal) << "reachable through unknown must be SUCCESS, not PARTIAL";
}

// Scenario: empty free map. Baseline: SUCCESS and exact goal.
// Why: status baseline for the tests above.
TEST(PlannerUnknownPaths, OpenGoalIsSuccessNotPartial) {
  PlannerWorld w;
  w.build(Knobs{});
  auto r = w.solve(10, 50, 90, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
}

// Scenario: an unknown band lies across the path. The executable prefix (r.path) is cut before it, but
//   the global path (r.raw_path) still ends at the goal.
// Expected: raw_path.back() == goal cell, r.path ends strictly before the band.
// Why: N2 + N3 - "The planner can globally plan all the way to a goal through unknown space.
//   However, the executed plan is a prefix of the global path that stops before unknown."
TEST(PlannerUnknownPaths, GlobalPathReachesGoalEvenIfPrefixIsCut) {
  PlannerWorld w;
  w.fillColumns(50, 59, -1);
  w.build(Knobs{});
  auto r = w.solve(10, 50, 90, 50);
  ASSERT_TRUE(r.ok && r.reached_goal);
  EXPECT_EQ(w.cell(r.raw_path.back())(0), 90);
  ASSERT_FALSE(r.path.empty());
  EXPECT_LT(w.cell(r.path.back())(0), 50) << "prefix must stop before the unknown band";
}
