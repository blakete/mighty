// Planner "New Plan" tests: executable prefix trim in HGPManager::solveHGP (hgp_manager.cpp ~320-395).
//
// Source of truth: goal_selector_refactor_spec.md "New Plan: detailed design" N3 incl. the
// "Resolved (2026-10-09)" block, section 11 item 14; goal_selector_test_ideas.md section E ("Prefix
// trim"). The trim is exercised end to end through the public HGPManager::solveHGP on a hand-built 2D
// grid: a straight start->goal line along row y = 50 (cells), start at cell x = 10, goal at x = 90, a
// full-height band of unknown columns forces the path through unknown. Waypoints are every 10 cells
// (max_dist_vertexes 0.95 m with 0.1 m cells); res = 0.1 m, back-off U = unknown_inflation_2d_m.
//
// Expected prefix end for a run starting at cell column F (first unknown): the trim's "edge of
// observed space" is the last free sample (column F-1); the prefix ends U of path length before the
// run. The note says "U of path length before the first run", so the end may lie anywhere from
// (F-1)*res - U to F*res - U (half-cell tolerance), see ExpectEndX().
//
//   Test                                          Spec / notes item
//   NoUnknownKeepsFullPath                        N3: no unknown -> nothing trimmed
//   UnknownRunCutsAndBacksOff                     N3 resolved: cut at run, back off U
//   BackOffWalksAcrossSeveralWaypoints            N3: back-off is arc length along the polyline
//   ZeroInflationMeansNoBackOff                   N3/param doc: unknown_inflation_2d_m 0 = no back-off
//   RunOfExactlyMinLengthCuts                     N3: run >= trim_min_unknown_run_cells
//   ShortUnknownRunIsDrivenThrough                N3: shorter runs (deliberate holes) are driven through
//   MinRunParameterIsHonoured                     trim_min_unknown_run_cells is the knob
//   PrefixNeverEndsBehindStart                    E: "never behind the start"
//   SubCellRemainderCollapsesToStart              E: "remainder shorter than one cell collapses to the start"
//   RemainderOfAtLeastOneCellIsKept               E: complement (remainder >= 1 cell is a real path)
//   UnknownPatchStraddlingWaypointStillCuts       E: patch across a waypoint (n before + m after) must cut
//   PrefixIsPolylineOfGlobalPath                  N3: prefix is a prefix of the global path
//
// NOT TESTED (needs production refactor, see report): "an occupied (non-inflation-only) waypoint stops
// the trim immediately". A* never routes through an occupied cell, so no waypoint on an occupied cell
// can be produced through solveHGP's public API; the branch is only reachable if the trim is extracted
// into a function that takes (path, map).

#include <gtest/gtest.h>

#include <cmath>

#include "planner_test_fixture.hpp"

using plannertest::Knobs;
using plannertest::PlannerWorld;

namespace {

constexpr int kRow = 50;
constexpr int kStartX = 10;
constexpr int kGoalX = 90;
constexpr double kRes = 0.1;

/** World x of the cell centre of column `cx` (all test cells share y = kRow). */
double CellX(const PlannerWorld& w, int cx) { return w.world(cx, kRow)(0); }

/** Prefix length along the polyline [m]. */
double PathLength(const vec_Vecf<3>& p) {
  double L = 0.0;
  for (size_t i = 1; i < p.size(); ++i) L += (p[i] - p[i - 1]).head<2>().norm();
  return L;
}

/** Assert the prefix ends U before an unknown run whose first unknown column is `first_unknown`
 *  (accepting slack, see file header). */
void ExpectEndX(const PlannerWorld& w, const vec_Vecf<3>& path, int first_unknown, double U) {
  ASSERT_FALSE(path.empty());
  const double end = path.back()(0);
  const double lo = CellX(w, first_unknown - 1) - U - 0.5 * kRes;
  const double hi = CellX(w, first_unknown) - U + 0.5 * kRes;
  EXPECT_GE(end, lo) << "prefix ends too early";
  EXPECT_LE(end, hi) << "prefix ends too late";
  EXPECT_NEAR(path.back()(1), w.world(0, kRow)(1), 0.5 * kRes) << "prefix must stay on the row";
}

/** Build a world with unknown columns [x0, x1] and solve start->goal. */
PlannerWorld::Result SolveWithBand(PlannerWorld& w, int x0, int x1, const Knobs& k) {
  w.fillColumns(x0, x1, -1);
  w.build(k);
  auto r = w.solve(kStartX, kRow, kGoalX, kRow);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  return r;
}

}  // namespace

// Scenario: fully observed free map. Expected: nothing is trimmed, the prefix is the whole global
// path ending on the goal. Why: N3 - the trim only acts at unknown/occupied.
TEST(PlannerPrefixTrim, NoUnknownKeepsFullPath) {
  PlannerWorld w;
  w.build(Knobs{});
  auto r = w.solve(kStartX, kRow, kGoalX, kRow);
  ASSERT_TRUE(r.ok && r.reached_goal);
  EXPECT_NEAR(r.path.back()(0), CellX(w, kGoalX), 0.5 * kRes);
}

// Scenario: 10-cell unknown band starting at column 50, U = 0.5 m. Expected: the prefix ends 0.5 m of
// path before the run, about column 44/45. Why: N3 resolved - "The prefix ends unknown_inflation_2d_m
// back along the path from the start of that run".
TEST(PlannerPrefixTrim, UnknownRunCutsAndBacksOff) {
  PlannerWorld w;
  auto r = SolveWithBand(w, 50, 59, Knobs{});
  ExpectEndX(w, r.path, 50, 0.5);
  EXPECT_NEAR(r.path.front()(0), CellX(w, kStartX), 1e-3) << "prefix starts at the start";
}

// Scenario: U = 1.5 m, longer than the 1.0 m waypoint spacing. Expected: the end is 1.5 m back
// (column ~34/35), i.e. the back-off walks over waypoint 40. Why: N3 - back-off is arc length.
TEST(PlannerPrefixTrim, BackOffWalksAcrossSeveralWaypoints) {
  PlannerWorld w;
  Knobs k;
  k.unknown_inflation_2d_m = 1.5;
  auto r = SolveWithBand(w, 50, 59, k);
  ExpectEndX(w, r.path, 50, 1.5);
}

// Scenario: unknown_inflation_2d_m = 0. Expected: the prefix ends at the last known sample before the
// run (column ~49), no back-off. Why: documented "<= 0 = no back-off" (N3 / parameter docs).
TEST(PlannerPrefixTrim, ZeroInflationMeansNoBackOff) {
  PlannerWorld w;
  Knobs k;
  k.unknown_inflation_2d_m = 0.0;
  auto r = SolveWithBand(w, 50, 59, k);
  ExpectEndX(w, r.path, 50, 0.0);
}

// Scenario: unknown band exactly trim_min_unknown_run_cells (3) wide. Expected: cut + back-off.
// Why: N3 - a run of >= trim_min_unknown_run_cells marks the edge of observed space.
TEST(PlannerPrefixTrim, RunOfExactlyMinLengthCuts) {
  PlannerWorld w;
  auto r = SolveWithBand(w, 50, 52, Knobs{});
  ExpectEndX(w, r.path, 50, 0.5);
}

// Scenario: unknown band of 2 cells (< 3), i.e. a deliberate mapper hole. Expected: driven through:
// the prefix is not shortened and ends at the goal. Why: N3 - "Shorter unknown runs ... are driven
// through".
TEST(PlannerPrefixTrim, ShortUnknownRunIsDrivenThrough) {
  PlannerWorld w;
  auto r = SolveWithBand(w, 50, 51, Knobs{});
  EXPECT_NEAR(r.path.back()(0), CellX(w, kGoalX), 0.5 * kRes);
}

// Scenario: a 4-cell band cut with min_run = 3 but driven through with min_run = 5.
// Expected: the parameter decides. Why: N3 - trim_min_unknown_run_cells is reused for the prefix.
TEST(PlannerPrefixTrim, MinRunParameterIsHonoured) {
  {
    PlannerWorld w;
    Knobs k;
    k.trim_min_unknown_run_cells = 3;
    auto r = SolveWithBand(w, 50, 53, k);
    ExpectEndX(w, r.path, 50, 0.5);
  }
  {
    PlannerWorld w;
    Knobs k;
    k.trim_min_unknown_run_cells = 5;
    auto r = SolveWithBand(w, 50, 53, k);
    EXPECT_NEAR(r.path.back()(0), CellX(w, kGoalX), 0.5 * kRes);
  }
}

// Scenario: unknown starts 3 cells (0.3 m) from the start, U = 0.5 m. Expected: the back-off would go
// behind the start, so the prefix is only the start point. Why: E - "never behind the start".
TEST(PlannerPrefixTrim, PrefixNeverEndsBehindStart) {
  PlannerWorld w;
  auto r = SolveWithBand(w, kStartX + 3, kStartX + 12, Knobs{});
  ASSERT_FALSE(r.path.empty());
  for (const auto& p : r.path) EXPECT_GE(p(0), CellX(w, kStartX) - 1e-3) << "point behind the start";
  EXPECT_NEAR(r.path.back()(0), CellX(w, kStartX), 1e-3);
}

// Scenario: last free sample is 0.5 m from the start, U = 0.45 m: remainder 0.05 m (< one 0.1 m cell).
// Expected: collapses to a single point at the start (robot waits). Why: E - "a remainder shorter
// than one cell collapses to the start point".
TEST(PlannerPrefixTrim, SubCellRemainderCollapsesToStart) {
  PlannerWorld w;
  Knobs k;
  k.unknown_inflation_2d_m = 0.45;
  auto r = SolveWithBand(w, kStartX + 6, kStartX + 20, k);  // last free sample = start + 5 cells
  ASSERT_EQ(r.path.size(), 1u);
  EXPECT_NEAR(r.path[0](0), CellX(w, kStartX), 1e-3);
}

// Scenario: last free sample is 0.6 m from the start, U = 0.45 m: remainder 0.15 m (>= one cell).
// Expected: a real two-point prefix of length ~0.15 m. Why: E complement of the sub-cell rule.
TEST(PlannerPrefixTrim, RemainderOfAtLeastOneCellIsKept) {
  PlannerWorld w;
  Knobs k;
  k.unknown_inflation_2d_m = 0.45;
  auto r = SolveWithBand(w, kStartX + 7, kStartX + 20, k);  // last free sample = start + 6 cells
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_NEAR(PathLength(r.path), 0.15, 0.5 * kRes);
}

// Scenario: an unknown patch lying across a path waypoint. The waypoint column comes from a control
// run on the free map. Variants (cells before / waypoint / after, min_run):
//   1 before + waypoint + 1 after, min_run 3;  2 before + waypoint + 2 after, min_run 5;
//   2 before + waypoint + 1 after, min_run 4;  1 before + waypoint + 2 after, min_run 4.
// Expected: in every variant the run is counted ACROSS the waypoint, so the path is cut before the
// patch (prefix end left of the patch's first column, not at the goal). Why: E - "A short unknown patch
// lying across a path waypoint (e.g. 2 unknown samples before the waypoint and 2 after) must still cut
// the path." ASSUMPTION: the waypoint's own cell is part of the patch (otherwise the free waypoint
// sample would break the run and no cut is expected).
TEST(PlannerPrefixTrim, UnknownPatchStraddlingWaypointStillCuts) {
  // Control: find an interior waypoint of the free-map path.
  int wp_col = -1;
  {
    PlannerWorld w;
    w.build(Knobs{});
    auto r = w.solve(kStartX, kRow, kGoalX, kRow);
    ASSERT_TRUE(r.ok);
    ASSERT_GE(r.path.size(), 5u);
    wp_col = w.cell(r.path[4])(0);  // an interior waypoint, ~ column 50
  }
  ASSERT_GT(wp_col, kStartX + 10);
  struct Variant {
    int before, after, min_run;
  };
  const Variant variants[] = {{1, 1, 3}, {2, 2, 5}, {2, 1, 4}, {1, 2, 4}};
  for (const auto& v : variants) {
    SCOPED_TRACE(testing::Message() << "before=" << v.before << " after=" << v.after
                                    << " min_run=" << v.min_run);
    PlannerWorld w;
    Knobs k;
    k.trim_min_unknown_run_cells = v.min_run;
    auto r = SolveWithBand(w, wp_col - v.before, wp_col + v.after, k);
    ASSERT_FALSE(r.path.empty());
    EXPECT_LT(r.path.back()(0), CellX(w, wp_col - v.before)) << "path must be cut before the patch";
    ExpectEndX(w, r.path, wp_col - v.before, 0.5);
  }
}

// Scenario: cut prefix on a straight row. Expected: every prefix point is on the global path's line and
// the prefix is shorter than the global path. Why: N3 - the executable plan is a PREFIX of the global
// path (points stay on the path, monotone in x).
TEST(PlannerPrefixTrim, PrefixIsPolylineOfGlobalPath) {
  PlannerWorld w;
  auto r = SolveWithBand(w, 50, 59, Knobs{});
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_LT(PathLength(r.path), PathLength(r.raw_path));
  for (size_t i = 0; i < r.path.size(); ++i) {
    EXPECT_NEAR(r.path[i](1), w.world(0, kRow)(1), 1e-3);
    if (i > 0) EXPECT_GT(r.path[i](0), r.path[i - 1](0));
  }
}
