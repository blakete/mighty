// Persistent-map detection source (exploration.persistent_map.*) of the ROS-free GoalSelector core.
//
// The persistent grid is the mapper's accumulated grid: it grows and its origin moves. With the
// option on, frontier detection runs in onPersistentGrid() (whole grid, or a square window around
// the robot) and onOccGrid() only stores the live grid; the VisitedMap is not used.
//
// World: the same 41 x 7 corridor as the other goal_selector tests (see goal_selector_test_utils.hpp):
// frontiers LEFT (2.25, 1.75) and RIGHT (18.25, 1.75), robot at (7.25, 1.75).

#include <gtest/gtest.h>

#include <cmath>

#include "goal_selector_test_utils.hpp"

using namespace gs_test;

namespace {

SelectorParams PersistentParams(bool whole_map, double radius_m) {
  SelectorParams p = BaseParams();
  p.expl_persistent_map_enabled = true;
  p.expl_persistent_detect_whole_map = whole_map;
  p.expl_persistent_detect_radius_m = radius_m;
  return p;
}

}  // namespace

// Scenario: option on. The live occ_2d grid is stored but does not detect and does not touch the
// VisitedMap; the persistent grid then does detect, and the visited-map publish/broadcast stay off.
TEST(GoalSelectorPersistentMap, OccGridOnlyStored_PersistentGridDetects) {
  Harness h(PersistentParams(true, 30.0));
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.state();

  Output o = h.feed();
  EXPECT_TRUE(h.sel.hasOccGrid());
  EXPECT_EQ(h.sel.frontierManager().size(), 0u) << "occ_2d must not drive detection";
  EXPECT_FALSE(o.publish_markers);
  EXPECT_EQ(h.sel.visitedMap().getStateWorld(5.25, 1.75), VisitedMap::kUnknown) << "no absorb";

  o = h.last_out = h.sel.onPersistentGrid(h.now, OpenBoth());
  EXPECT_TRUE(o.publish_markers);
  EXPECT_FALSE(o.publish_visited_map);
  EXPECT_FALSE(o.broadcast_visited_map);
  ASSERT_EQ(h.sel.frontierManager().size(), 2u);
  EXPECT_NE(h.recordNear(kLeftX, kCorridorY), nullptr);
  EXPECT_NE(h.recordNear(kRightX, kCorridorY), nullptr);
  EXPECT_EQ(h.sel.visitedMap().getStateWorld(5.25, 1.75), VisitedMap::kUnknown) << "still unused";
}

// Scenario: radius mode, 6 m around the robot at x = 7.25 -> window x in [1.25, 13.25]: only the LEFT
// frontier (5 m away) is inside. Whole-map mode on the same grid sees both.
TEST(GoalSelectorPersistentMap, RadiusLimitsDetection_WholeMapDoesNot) {
  {
    Harness h(PersistentParams(false, 6.0));
    h.setRobot(kStartX, kStartY);
    h.state();
    h.sel.onPersistentGrid(h.now, OpenBoth());
    ASSERT_EQ(h.sel.frontierManager().size(), 1u);
    EXPECT_NE(h.recordNear(kLeftX, kCorridorY), nullptr);
  }
  {
    Harness h(PersistentParams(true, 6.0));  // radius ignored in whole-map mode
    h.setRobot(kStartX, kStartY);
    h.state();
    h.sel.onPersistentGrid(h.now, OpenBoth());
    EXPECT_EQ(h.sel.frontierManager().size(), 2u);
  }
}

// Scenario: the accumulated grid has a different origin (it grew). Frontier centroids are in world
// coordinates, so they move with the origin.
TEST(GoalSelectorPersistentMap, ShiftedOriginGrid_CentroidsInWorldFrame) {
  Harness h(PersistentParams(false, 30.0));
  const double dx = -5.0, dy = -3.0;
  GridInput g = OpenBoth();
  g.origin_x = dx;
  g.origin_y = dy;
  h.setRobot(kStartX + dx, kStartY + dy);
  h.state();
  h.sel.onPersistentGrid(h.now, g);
  ASSERT_EQ(h.sel.frontierManager().size(), 2u);
  const FrontierRecord* l = h.recordNear(kLeftX + dx, kCorridorY + dy);
  const FrontierRecord* r = h.recordNear(kRightX + dx, kCorridorY + dy);
  ASSERT_NE(l, nullptr);
  ASSERT_NE(r, nullptr);
  EXPECT_NEAR(l->centroid_xy.x(), kLeftX + dx, 0.3);
  EXPECT_NEAR(r->centroid_xy.x(), kRightX + dx, 0.3);
}

// Scenario: the window does not overlap the grid (robot far outside it) -> no detection, a warning.
TEST(GoalSelectorPersistentMap, RobotOutsideGrid_NoDetectionWarns) {
  Harness h(PersistentParams(false, 5.0));
  h.setRobot(100.0, 100.0);
  h.state();
  Output o = h.sel.onPersistentGrid(h.now, OpenBoth());
  EXPECT_EQ(h.sel.frontierManager().size(), 0u);
  ASSERT_FALSE(o.logs.empty());
  EXPECT_EQ(o.logs.front().level, goal_selector::LogMessage::Level::kWarn);
}

// Scenario: option off (default) -> onPersistentGrid is a no-op, and onOccGrid still detects as before.
TEST(GoalSelectorPersistentMap, OptionOff_PersistentGridIgnored) {
  Harness h(BaseParams());
  h.setRobot(kStartX, kStartY);
  h.state();
  Output o = h.sel.onPersistentGrid(h.now, OpenBoth());
  EXPECT_FALSE(o.publish_markers);
  EXPECT_EQ(h.sel.frontierManager().size(), 0u);
  h.setMap(OpenBoth());
  h.feed();
  EXPECT_EQ(h.sel.frontierManager().size(), 2u);
}
