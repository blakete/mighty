// Geofence keep-out in the planner's 2D map (MapUtil::applyKeepOut2D, keepout_2d.* on mighty_node).
//
// The planner plans on the raw occ_2d, which never carries the geofence, so the fence reaches A* only
// through the keep-out: outside keep_in / inside a keep_out is a real obstacle, the band (closer than
// band_m to a fence edge) is OCCUPIED but inflated-only, so A* never plans into it or ends there, yet a
// start inside it may leave it. Driven through HGPManager::solveHGP (planner_test_fixture.hpp).
//
// World: 100 x 100 cells at 0.1 m, keep_in = the whole window, keep_out = the block x 45..55,
// y < 70 (cell-corner units), band 0.3 m (3 cells) unless stated.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "planner_test_fixture.hpp"

using goal_selector::FencePoint;
using goal_selector::GoalFence;
using plannertest::Knobs;
using plannertest::PlannerWorld;

namespace {

std::vector<FencePoint> Box(double x0, double x1, double y0, double y1) {
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

/** Window-wide keep_in plus the block keep_out, band band_m. */
void AddFence(PlannerWorld& w, double band_m, bool block = true) {
  w.keepout = true;
  w.fence_cells.keep_in = Box(0.0, w.n, 0.0, w.n);
  if (block) w.fence_cells.keep_out = {Box(45.0, 55.0, -5.0, 70.0)};
  w.keepout_band_m = band_m;
}

/** Every point along the global path (sampled at res / 4) is at least band_m - res from the fence
 *  and inside it (one cell of slack: A* runs between cell centres). */
void ExpectPathRespectsFence(const PlannerWorld& w, const vec_Vecf<3>& path) {
  GoalFence f = w.fence;
  f.offset_m = std::max(0.0, w.keepout_band_m - w.res);
  for (size_t i = 1; i < path.size(); ++i) {
    const Vec3f a = path[i - 1], b = path[i];
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / (0.25 * w.res))));
    for (int s = 0; s <= n; ++s) {
      const Vec3f p = a + (b - a) * (static_cast<double>(s) / n);
      ASSERT_TRUE(goal_selector::goalFenceAllows(f, p(0), p(1)))
          << "path point (" << p(0) << ", " << p(1) << ") is outside the fence or in its band";
    }
  }
}

bool PathTouchesBox(const PlannerWorld& w, const vec_Vecf<3>& path, int x0, int x1, int y0, int y1) {
  for (size_t i = 1; i < path.size(); ++i) {
    const Vec3f a = path[i - 1], b = path[i];
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / (0.25 * w.res))));
    for (int s = 0; s <= n; ++s) {
      const Veci<3> c = w.cell(a + (b - a) * (static_cast<double>(s) / n));
      if (c(0) >= x0 && c(0) <= x1 && c(1) >= y0 && c(1) <= y1) return true;
    }
  }
  return false;
}

}  // namespace

TEST(PlannerKeepOut, WithoutKeepOutThePathCrossesTheBlock) {
  PlannerWorld w;
  w.build(Knobs{});
  auto r = w.solve(20, 50, 80, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  EXPECT_TRUE(PathTouchesBox(w, r.raw_path, 45, 54, 0, 69)) << "control: no keep-out, straight through";
}

TEST(PlannerKeepOut, KeepOutBlockForcesADetour) {
  PlannerWorld w;
  AddFence(w, 0.3);
  w.build(Knobs{});
  auto r = w.solve(20, 50, 80, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  EXPECT_FALSE(PathTouchesBox(w, r.raw_path, 45, 54, 0, 69));
  ExpectPathRespectsFence(w, r.raw_path);
  ExpectPathRespectsFence(w, r.path);
}

TEST(PlannerKeepOut, MapCellsAreClassified) {
  PlannerWorld w;
  AddFence(w, 0.3);
  w.build(Knobs{});
  const auto& mu = *w.mu;
  // outside the fence: a real obstacle
  EXPECT_TRUE(mu.is2DOccupied(50, 30));
  EXPECT_FALSE(mu.is2DInflatedOnly(50, 30));
  EXPECT_FALSE(mu.is2DKeepOutBand(50, 30));
  // band: centre 0.15 m from the block edge
  EXPECT_TRUE(mu.is2DOccupied(56, 30));
  EXPECT_TRUE(mu.is2DInflatedOnly(56, 30));
  EXPECT_TRUE(mu.is2DKeepOutBand(56, 30));
  // core: centre 0.45 m from the edge
  EXPECT_FALSE(mu.is2DOccupied(59, 30));
  // window border is the keep_in edge: band too
  EXPECT_TRUE(mu.is2DKeepOutBand(0, 50));
}

TEST(PlannerKeepOut, GoalOutsideTheFenceOrInItsBandFails) {
  {
    PlannerWorld w;
    AddFence(w, 0.3);
    w.build(Knobs{});
    EXPECT_FALSE(w.solve(20, 50, 50, 30).ok) << "goal inside the keep_out";
  }
  {
    PlannerWorld w;
    AddFence(w, 0.3);
    w.build(Knobs{});
    EXPECT_FALSE(w.solve(20, 50, 56, 30).ok) << "goal in the band";
  }
  {
    PlannerWorld w;
    AddFence(w, 0.3);
    w.build(Knobs{});
    auto r = w.solve(20, 50, 59, 30);
    ASSERT_TRUE(r.ok) << "goal just inside the core";
    EXPECT_TRUE(r.reached_goal);
    ExpectPathRespectsFence(w, r.raw_path);
  }
}

TEST(PlannerKeepOut, StartInsideTheBandLeavesIt) {
  // band 0.5 m: the start (centre 0.15 m from the block) is 3.5 cells deeper in the band than
  // solveHGP's 0.2 m start freeing reaches, so only the keep-out escape radius gets it out.
  PlannerWorld w;
  AddFence(w, 0.5);
  w.build(Knobs{});
  ASSERT_TRUE(w.mu->is2DKeepOutBand(59, 30));
  auto r = w.solve(56, 30, 85, 30);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  const Veci<3> last = w.cell(r.raw_path.back());
  EXPECT_EQ(last(0), 85);
}

TEST(PlannerKeepOut, StartOutsideTheFenceDoesNotReachTheGoal) {
  PlannerWorld w;
  AddFence(w, 0.3);
  w.build(Knobs{});
  auto r = w.solve(50, 30, 80, 30);  // 5 cells deep inside the keep_out
  EXPECT_FALSE(r.ok && r.reached_goal);
}

TEST(PlannerKeepOut, UnknownInsideTheFenceStaysPlannable) {
  PlannerWorld w;
  w.fillColumns(45, 54, -1);
  AddFence(w, 0.3, /*block=*/false);
  w.build(Knobs{});
  auto r = w.solve(20, 50, 80, 50);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  EXPECT_TRUE(PathTouchesBox(w, r.raw_path, 45, 54, 0, 99)) << "still plans through unknown";
}
