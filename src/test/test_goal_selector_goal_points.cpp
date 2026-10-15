// Frontier goals in already-mapped space (PR #8 follow-up), on the ROS-free GoalSelector core with the
// hardware parameters (config/hw_goal_selector.yaml + inflation_2d_m 0.1 / unknown_inflation_2d_m 0.5,
// persistent-map detection over the whole grid).
//
// Two defects sent the robot to goals with no unknown nearby:
//   1. the unknown band went straight through walls, so a mapped room whose outside is unknown grew a
//      frontier along every wall thinner than 0.4 m (unknown_inflation_2d_m - inflation_2d_m);
//   2. the goal was the raw cluster centroid, which for a ring / L / long frontier lies deep inside known
//      space.
// Small unknown holes still produce a frontier on purpose (spec N1 Q2), guarded below.
//
// World: 20 x 12 m at 0.1 m, origin (0, 0), y = 0 is the first row.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "goal_selector/goal_selector.hpp"

using namespace goal_selector;

namespace {

constexpr double kRes = 0.1;
constexpr int kW = 200;
constexpr int kH = 120;

struct World {
  std::vector<int8_t> d = std::vector<int8_t>(kW * kH, -1);  // all unknown
  // Set every cell whose centre lies in [x0, x1) x [y0, y1) to v.
  World& rect(double x0, double y0, double x1, double y1, int8_t v) {
    for (int iy = 0; iy < kH; ++iy)
      for (int ix = 0; ix < kW; ++ix) {
        const double x = (ix + 0.5) * kRes, y = (iy + 0.5) * kRes;
        if (x >= x0 && x < x1 && y >= y0 && y < y1) d[ix + kW * iy] = v;
      }
    return *this;
  }
  GridInput grid() const {
    GridInput g;
    g.width = kW;
    g.height = kH;
    g.resolution = kRes;
    g.data = d;
    return g;
  }
  // Distance from (x, y) to the nearest real unknown cell centre.
  double nearestUnknown(double x, double y) const {
    double best = std::numeric_limits<double>::infinity();
    for (int iy = 0; iy < kH; ++iy)
      for (int ix = 0; ix < kW; ++ix)
        if (d[ix + kW * iy] < 0) best = std::min(best, std::hypot((ix + 0.5) * kRes - x, (iy + 0.5) * kRes - y));
    return best;
  }
};

SelectorParams HwParams(double max_extent_m = 3.0) {
  SelectorParams p;
  p.robot_id = "JR02";
  p.expl_enabled = true;
  p.expl_persistent_map_enabled = true;
  p.expl_persistent_detect_whole_map = true;
  p.expl_cluster_min_cells = 6;
  p.expl_border_margin_cells = 1;
  p.expl_obstacle_clearance_cells = 0;
  p.expl_robot_snap_radius_m = 0.5;
  p.expl_max_cluster_extent_m = max_extent_m;
  p.expl_w_dist = 60.0;
  p.expl_dist_ref_m = 60.0;
  p.expl_merge_radius_m = 0.75;
  p.expl_visit_radius_m = 0.3;
  p.expl_visit_dwell_sec = 3.0;
  p.expl_verify_radius_cells = 2;
  p.expl_pursuit_timeout_factor = 3.0;
  p.expl_invalidation_keep_out_radius_m = 0.75;
  p.expl_invalidation_cooldown_sec = 60.0;
  p.expl_geofence_enabled = false;
  p.expl_preempt_enabled = true;
  p.expl_return_home_enabled = false;
  p.inflation_2d_m = 0.1;
  p.unknown_inflation_2d_m = 0.5;
  return p;
}

// Feed one map (live occ_2d and persistent grid) and run a select tick.
Output Feed(GoalSelector& s, const World& w, double now) {
  s.onOccGrid(now, w.grid());
  s.onPersistentGrid(now, w.grid());
  return s.onSelectTick(now);
}

std::vector<const FrontierRecord*> Pending(const GoalSelector& s) {
  std::vector<const FrontierRecord*> v;
  for (const auto& r : s.frontierManager().records())
    if (r.state == FrontierState::ACTIVE || r.state == FrontierState::DORMANT) v.push_back(&r);
  return v;
}

bool IsCellOf(const FrontierRecord& r, const Eigen::Vector2d& p) {
  return std::any_of(r.cells.begin(), r.cells.end(), [&](const auto& c) { return (c - p).norm() < 1e-9; });
}

// A frontier cell sits just outside the unknown band: within unknown_inflation_2d_m plus about one cell
// (diagonal) of real unknown.
constexpr double kNearUnknown = 0.5 + 1.5 * kRes;

}  // namespace

// Scenario: a fully mapped closed room (free 1..19 x 1..11 m), walls 0.1 / 0.2 / 0.3 m thick, unknown
// outside. Before the fix the band crossed the wall and the whole wall line became one ring frontier
// whose centroid (the room centre) was the goal. Expected: no frontier, no goal.
TEST(GoalSelectorGoalPoints, ClosedRoomWithThinWallsHasNoFrontier) {
  for (const double t : {0.1, 0.2, 0.3}) {
    GoalSelector s(HwParams());
    s.onState(0.0, Pose{3.0, 6.0, 0.0, 0.0});
    World w;
    w.rect(1.0 - t, 1.0 - t, 19.0 + t, 11.0 + t, 100).rect(1.0, 1.0, 19.0, 11.0, 0);
    const Output o = Feed(s, w, 1.0);
    EXPECT_TRUE(Pending(s).empty()) << "wall " << t << " m";
    EXPECT_FALSE(o.term_goal.has_value()) << "wall " << t << " m";
  }
}

// Scenario: known free x < 12, y < 8 with walls on the south and west sides, unknown to the east and
// north: one L-shaped frontier (no split: extent < 12 m but splitting off). Before the fix the goal was
// its centroid, 1.9 m inside known space. Expected: the goal is one of the frontier's cells, next to the
// unknown, and is what gets published.
TEST(GoalSelectorGoalPoints, LShapedFrontierGoalIsAFrontierCell) {
  GoalSelector s(HwParams(0.0));
  s.onState(0.0, Pose{3.0, 6.0, 0.0, 0.0});
  World w;
  w.rect(0.0, 0.0, 12.0, 8.0, 100).rect(0.6, 0.6, 12.0, 8.0, 0);
  const Output o = Feed(s, w, 1.0);
  const auto pending = Pending(s);
  ASSERT_EQ(pending.size(), 1u);
  const FrontierRecord& r = *pending[0];
  EXPECT_GT(w.nearestUnknown(r.centroid_xy.x(), r.centroid_xy.y()), 1.0) << "the centroid is deep in known space";
  EXPECT_TRUE(IsCellOf(r, r.goal_xy));
  EXPECT_LE(w.nearestUnknown(r.goal_xy.x(), r.goal_xy.y()), kNearUnknown);
  ASSERT_TRUE(o.term_goal.has_value());
  EXPECT_DOUBLE_EQ(o.term_goal->position.x(), r.goal_xy.x());
  EXPECT_DOUBLE_EQ(o.term_goal->position.y(), r.goal_xy.y());
}

// Scenario: a room 18 m wide whose north side is open to unknown (one 18 m frontier). With the hardware
// split length (3 m) it becomes several world-tile frontiers. Expected: at least 6 frontiers, each goal
// one of its own cells next to the unknown, and no frontier longer than 3 m plus a merged sliver.
TEST(GoalSelectorGoalPoints, LongFrontierIsSplitAndEveryGoalBordersUnknown) {
  GoalSelector s(HwParams(3.0));
  s.onState(0.0, Pose{3.0, 3.0, 0.0, 0.0});
  World w;
  w.rect(0.5, 0.5, 19.5, 8.0, 100).rect(1.0, 1.0, 19.0, 8.0, 0);
  Feed(s, w, 1.0);
  const auto pending = Pending(s);
  EXPECT_GE(pending.size(), 6u);
  for (const auto* r : pending) {
    EXPECT_TRUE(IsCellOf(*r, r->goal_xy));
    EXPECT_LE(w.nearestUnknown(r->goal_xy.x(), r->goal_xy.y()), kNearUnknown);
    EXPECT_LE(r->aabb_max.x() - r->aabb_min.x(), 3.0 + 0.6);
  }
}

// Scenario: the L-shaped frontier above, then the whole area becomes known (free inside the walls).
// Expected: the record is retired and no goal is sent into the now-mapped space.
TEST(GoalSelectorGoalPoints, FrontierIsRetiredOnceItsAreaIsMapped) {
  GoalSelector s(HwParams(0.0));
  s.onState(0.0, Pose{3.0, 6.0, 0.0, 0.0});
  World before;
  before.rect(0.0, 0.0, 12.0, 8.0, 100).rect(0.6, 0.6, 12.0, 8.0, 0);
  Feed(s, before, 1.0);
  ASSERT_EQ(Pending(s).size(), 1u);
  World after;
  after.rect(0.0, 0.0, 20.0, 12.0, 100).rect(0.6, 0.6, 19.4, 11.4, 0);
  const Output o = Feed(s, after, 3.0);
  EXPECT_TRUE(Pending(s).empty());
  EXPECT_FALSE(o.term_goal.has_value());
}

// Scenario: fully mapped open area with one 2x2-cell unknown hole (the mapper keeps such holes unknown on
// purpose). Expected, unchanged by this fix (spec N1 Q2, "do not ignore small patches for now"): the hole
// still yields a frontier, with its goal next to the hole.
TEST(GoalSelectorGoalPoints, SmallUnknownHoleStillYieldsAFrontier) {
  GoalSelector s(HwParams());
  s.onState(0.0, Pose{3.0, 6.0, 0.0, 0.0});
  World w;
  w.rect(0.0, 0.0, 20.0, 12.0, 0).rect(14.9, 5.9, 15.1, 6.1, -1);
  Feed(s, w, 1.0);
  const auto pending = Pending(s);
  ASSERT_EQ(pending.size(), 1u);
  EXPECT_LE(w.nearestUnknown(pending[0]->goal_xy.x(), pending[0]->goal_xy.y()), kNearUnknown);
}
