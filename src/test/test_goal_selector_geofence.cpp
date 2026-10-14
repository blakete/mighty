// Geofence no-go filter (exploration.geofence.*): frontiers whose centroid lies within margin_m of a
// no-go cell of the geofence grid are INVALIDATED before ranking, so the selector never commits a
// goal the planner (whose grid holds the same fence) cannot reach.
//
// World: the OpenBoth corridor of goal_selector_test_utils.hpp (LEFT frontier (2.25, 1.75), RIGHT
// frontier (18.25, 1.75), robot at (7.25, 1.75) -> LEFT is nearer and wins without a fence). Fence
// grids are built here at 0.1 m like the geofence's own (100 = no-go, -1 = no opinion).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "goal_selector_test_utils.hpp"

using namespace gs_test;
using goal_selector::Transform2D;
using goal_selector::nogoNear;

namespace {

/** @brief Fence grid at 0.1 m: `w` x `h` cells from (ox, oy), all -1, no-go (100) in the given
 *  world-frame boxes {x0, x1, y0, y1} (cells whose centre lies inside). */
GridInput FenceGrid(double ox, double oy, int w, int h,
                    const std::vector<std::array<double, 4>>& nogo_boxes) {
  GridInput g;
  g.width = w;
  g.height = h;
  g.resolution = 0.1;
  g.origin_x = ox;
  g.origin_y = oy;
  g.data.assign(static_cast<size_t>(w) * h, -1);
  for (int cy = 0; cy < h; ++cy) {
    for (int cx = 0; cx < w; ++cx) {
      const double x = ox + (cx + 0.5) * 0.1;
      const double y = oy + (cy + 0.5) * 0.1;
      for (const auto& b : nogo_boxes)
        if (x >= b[0] && x <= b[1] && y >= b[2] && y <= b[3])
          g.data[static_cast<size_t>(cx) + static_cast<size_t>(w) * cy] = 100;
    }
  }
  return g;
}

/** @brief Fence grid over the whole corridor (x 0..20.5, y 0..3.5). */
GridInput CorridorFence(const std::vector<std::array<double, 4>>& nogo_boxes) {
  return FenceGrid(0.0, 0.0, 205, 35, nogo_boxes);
}

/** @brief OpenBoth corridor, robot at the default start (Harness is not movable: set up in place). */
void InitCorridor(Harness& h) {
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
}

}  // namespace

TEST(GoalSelectorGeofence, NoGoFrontierIsNeverCommitted) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.sel.onFenceGrid(h.now, CorridorFence({{0.0, 3.0, 0.0, 3.5}}), Transform2D{});  // LEFT in no-go
  h.cycle();

  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY)) << "only RIGHT is allowed";
  for (const auto& g : h.goals) EXPECT_FALSE(GoalNear(g, kLeftX, kCorridorY, 1.0));
  const auto* left = h.recordNear(kLeftX, kCorridorY);
  ASSERT_NE(left, nullptr);
  EXPECT_EQ(left->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kInfo, "geofence no-go zone"));
}

TEST(GoalSelectorGeofence, MarginIsRobotRadius) {
  // LEFT centroid x = 2.25 (fence cell 22). Margin 0.3 m = 3 cells.
  {
    Harness h(BaseParams());
    InitCorridor(h);  // no-go from x = 2.45 (cell 24): 2 cells away -> invalidated
    h.sel.onFenceGrid(h.now, CorridorFence({{2.45, 3.45, 0.0, 3.5}}), Transform2D{});
    h.cycle();
    EXPECT_EQ(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
    EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
  }
  {
    Harness h(BaseParams());
    InitCorridor(h);  // no-go from x = 2.75 (cell 27): 5 cells away -> kept
    h.sel.onFenceGrid(h.now, CorridorFence({{2.75, 3.75, 0.0, 3.5}}), Transform2D{});
    h.cycle();
    EXPECT_NE(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
}

TEST(GoalSelectorGeofence, FailsOpenWithoutAFreshGrid) {
  const auto fence = CorridorFence({{0.0, 3.0, 0.0, 3.5}});  // would invalidate LEFT
  {
    Harness h(BaseParams());
    InitCorridor(h);  // no fence grid at all
    h.cycle();
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
  {
    Harness h(BaseParams());
    InitCorridor(h);  // grid older than max_age (1 s)
    h.sel.onFenceGrid(h.now - 2.0, fence, Transform2D{});
    h.cycle();
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
    EXPECT_NE(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
  }
  {
    SelectorParams p = BaseParams();  // filter disabled
    p.expl_geofence_enabled = false;
    Harness h(p);
    InitCorridor(h);
    h.sel.onFenceGrid(h.now, fence, Transform2D{});
    h.cycle();
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
}

TEST(GoalSelectorGeofence, CentroidOutsideTheWindowHasNoOpinion) {
  // The geofence window only covers x 12..20.5 and is all no-go: RIGHT (inside) is invalidated,
  // LEFT (outside, far from the window) is not.
  Harness h(BaseParams());
  InitCorridor(h);
  h.sel.onFenceGrid(h.now, FenceGrid(12.0, 0.0, 85, 35, {{12.0, 20.5, 0.0, 3.5}}), Transform2D{});
  h.cycle();
  EXPECT_EQ(h.recordNear(kRightX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_NE(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
}

TEST(GoalSelectorGeofence, UsesTheMapToFenceTransform) {
  // map -> fence: rotate 90 deg, then shift (10, 0):  (x, y) -> (10 - y, x).
  //   LEFT  (2.25, 1.75)  -> ( 8.25,  2.25)   no-go box here     -> invalidated
  //   RIGHT (18.25, 1.75) -> ( 8.25, 18.25)   free               -> kept
  // No-go boxes also sit where LEFT/RIGHT would be WITHOUT the transform; they must not matter.
  const Transform2D map_to_fence{10.0, 0.0, M_PI / 2.0};
  const auto fence = FenceGrid(0.0, 0.0, 200, 200,
                               {{7.75, 8.75, 1.75, 2.75},     // transformed LEFT
                                {1.75, 2.75, 1.25, 2.25},     // untransformed LEFT
                                {17.75, 18.75, 1.25, 2.25}}); // untransformed RIGHT
  Harness h(BaseParams());
  InitCorridor(h);
  h.sel.onFenceGrid(h.now, fence, map_to_fence);
  h.cycle();
  EXPECT_EQ(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_NE(h.recordNear(kRightX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
}

TEST(GoalSelectorGeofence, CurrentPursuitTurningNoGoIsDroppedAndReplaced) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.cycle();  // no fence yet: LEFT committed
  ASSERT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
  const auto left_id = h.sel.currentExploreId();

  h.advance(1.0);
  h.sel.onFenceGrid(h.now, CorridorFence({{0.0, 3.0, 0.0, 3.5}}), Transform2D{});
  h.cycle();
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals.back(), kRightX, kCorridorY)) << "re-selected the other frontier";
}

TEST(GoalSelectorGeofence, NogoNearCellRules) {
  // 3 x 1 grid at 1 m from (0, 0): cell 0 unknown, cell 1 no-go, cell 2 "occupied" at 49 (< 50).
  GridInput g;
  g.width = 3;
  g.height = 1;
  g.resolution = 1.0;
  g.data = {-1, 100, 49};
  // floor convention at cell edges, margin 0 = own cell only
  EXPECT_FALSE(nogoNear(g, 0.999, 0.5, 0.0));
  EXPECT_TRUE(nogoNear(g, 1.0, 0.5, 0.0));
  EXPECT_TRUE(nogoNear(g, 1.999, 0.5, 0.0));
  EXPECT_FALSE(nogoNear(g, 2.0, 0.5, 0.0)) << "49 is below the occupied threshold";
  // unknown and out-of-window cells give no opinion
  EXPECT_FALSE(nogoNear(g, 0.5, 0.5, 0.0));
  EXPECT_FALSE(nogoNear(g, -5.0, 0.5, 0.0));
  EXPECT_FALSE(nogoNear(g, 0.5, 5.0, 0.0));
  // a margin reaches neighbouring in-window cells, even from outside the window
  EXPECT_TRUE(nogoNear(g, 0.5, 0.5, 1.0));
  EXPECT_TRUE(nogoNear(g, 1.5, -0.5, 1.0));
  // empty / malformed grids
  EXPECT_FALSE(nogoNear(GridInput{}, 0.0, 0.0, 1.0));
}
