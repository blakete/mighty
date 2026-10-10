// Tests for the selector's frontier map and reachability (New Plan, Phase 5).
//
// Everything goes through the PUBLIC GoalSelector API (onState / onOccGrid / selectorMap() /
// frontierManager().records()); GoalSelector::buildFrontierGrid is private. Independent "oracles" in this file
// (brute-force band computation, BFS) are written from the spec definitions, not copied from the code.
//
// Source of truth
//   * goal_selector_refactor_spec.md, "New Plan: detailed design": N1 (maps, occupied band, unknown band) and
//     N5 (selector: safe-free cell, frontier cell, reachability bullet, obstacle_clearance_cells bullet, the
//     "unknown edges within inflation_2d_m of an obstacle produce no frontier" bullet, VISITED verify) and
//     section 11 items 16, 18, 19, 20.
//   * goal_selector_test_ideas.md, section C "Maps and reachability".
//
// Conventions used by every test
//   * Grids are written as ASCII rows. The FIRST string is y = 0 (the lowest-y row; so the picture is flipped
//     vertically compared with RViz). Legend of RAW grids: '.' free, '#' occupied, '?' unknown.
//     Legend of DERIVED maps (selectorMap): '.' 0 free, 'b' 99 occupied band, 'u' 50 unknown band,
//     '?' -1 real unknown, '#' 100 occupied.
//   * Resolution 0.1 m, origin (0,0): cell (x, y) has its centre at ((x+0.5)*0.1, (y+0.5)*0.1).
//   * Production defaults are used where possible: detection runs on the persistent visited map
//     (expl_detect_on_visited_map = true); the visited map is sized to be exactly the test grid, so the detect
//     grid equals the raw grid. Border margin 0 and obstacle_clearance_cells 0 (hw_goal_selector.yaml sets the
//     latter to 0 as well, N5).
//   * Default radii in these tests: unknown_inflation_2d_m = 0.2 (2 cells), inflation_2d_m varies per test.
//
// Test map
//   SelectorMap.ValuesAreOnlyTheDocumentedCodesAndMatchTheDefinition   -> N1, section 11 item 20
//   SelectorMap.EachCodeHasTheDocumentedMeaning                         -> item 20 (0/50/99/100/-1)
//   SelectorMap.OccupiedBandWinsOverUnknownBandOnFreeCells             -> ASSUMPTION (see test)
//   SelectorMap.ExplorationOffStillPublishesTheSameMapAndRunsNoDetection -> item 19, notes D
//   SelectorMap.NoMapBeforeTheFirstGrid
//   FrontierMap.FrontierSitsTwoCellsFromRealUnknownNextToTheUnknownBand -> notes C bullet 1, N5
//   FrontierMap.FrontierCellsAreFreeSafeAndAdjacentToTheBandOnAnIrregularMap -> notes C bullet 1 (property + completeness)
//   Reachability.FrontierBeyondOneCellHoleIsFound                        -> N5 Reachability bullet
//   Reachability.RealUnknownAcrossTheWholeCorridorIsNeverCrossed         -> N5 Reachability bullet ("never real unknown")
//   Reachability.OccupiedBandIsNeverCrossedEvenThroughAGap               -> N5 Reachability bullet ("never the occupied band")
//   Lifecycle.NotInstantlyVisitedAndVisitedOnlyOnceTheBandIsGone         -> notes C bullet 2, N5 VISITED verify, item 18
//   Lifecycle.CentroidInObstacleMarginInvalidatedThenKeepOutThenNewCandidate -> notes C bullet 3
//   NoFrontierNearObstacles.UnknownEdgeWithinInflationOfAnObstacleGivesNoFrontier -> notes C bullet 4 (+ control)

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "goal_selector/goal_selector.hpp"
#include "mighty/frontier_detector.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

using goal_selector::GoalSelector;
using goal_selector::GridInput;
using goal_selector::Pose;
using goal_selector::SelectorMap;
using goal_selector::SelectorParams;
using Rows = std::vector<std::string>;

constexpr double kRes = 0.1;

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

/** ASCII rows ('.' free, '#' occupied, '?' unknown; first row = y 0) -> raw tri-state buffer. */
std::vector<int8_t> rawFromRows(const Rows& rows) {
  const int h = static_cast<int>(rows.size());
  const int w = static_cast<int>(rows[0].size());
  std::vector<int8_t> v(static_cast<size_t>(w) * h, 0);
  for (int y = 0; y < h; ++y) {
    EXPECT_EQ(static_cast<int>(rows[y].size()), w) << "ragged ASCII row " << y;
    for (int x = 0; x < w; ++x) {
      const char c = rows[y][x];
      v[x + static_cast<size_t>(w) * y] = (c == '#') ? 100 : (c == '?') ? -1 : 0;
    }
  }
  return v;
}

GridInput gridFromRaw(const std::vector<int8_t>& raw, int w, int h) {
  GridInput g;
  g.width = w;
  g.height = h;
  g.resolution = kRes;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.origin_z = 0.0;
  g.data = raw;
  return g;
}

GridInput gridFromRows(const Rows& rows) {
  return gridFromRaw(rawFromRows(rows), static_cast<int>(rows[0].size()), static_cast<int>(rows.size()));
}

/** Free columns [0, k) and unknown columns [k, w), h rows:
 *      ..............??????????      (k = 14, w = 24)
 *  Every row identical. */
Rows halfUnknown(int w, int h, int k) {
  return Rows(h, std::string(k, '.') + std::string(w - k, '?'));
}

/** Selector parameters for these tests: exploration on, tiny clusters allowed, no border margin, no
 *  obstacle_clearance, visited map == the test grid (so detection sees exactly the raw grid). */
SelectorParams makeParams(int w, int h, double inflation_m, double unknown_inflation_m = 0.2) {
  SelectorParams p;
  p.robot_id = "T";
  p.expl_enabled = true;
  p.expl_cluster_min_cells = 3;
  p.expl_border_margin_cells = 0;
  p.expl_obstacle_clearance_cells = 0;
  p.expl_robot_snap_radius_m = 1.0;
  p.expl_bounds_enabled = false;
  p.expl_merge_radius_m = 0.5;
  p.expl_visit_radius_m = 0.3;
  p.expl_visit_dwell_sec = 1.0;
  p.expl_verify_radius_cells = 2;
  p.expl_invalidation_keep_out_radius_m = 1.5;
  p.expl_invalidation_cooldown_sec = 10.0;
  p.expl_peer_visit_radius_m = 0.0;  // no peers anyway
  p.expl_publish_markers = false;
  p.expl_publish_visited_map = false;
  p.expl_fuse_persistent_into_local = false;
  p.expl_detect_on_visited_map = true;
  p.expl_visited_map_center_x = 0.5 * w * kRes;
  p.expl_visited_map_center_y = 0.5 * h * kRes;
  p.expl_visited_map_width_m = w * kRes;
  p.expl_visited_map_height_m = h * kRes;
  p.expl_visited_map_resolution_m = kRes;
  p.inflation_2d_m = inflation_m;
  p.unknown_inflation_2d_m = unknown_inflation_m;
  return p;
}

/** A selector plus its robot, ready to be fed grids. */
struct Rig {
  std::unique_ptr<GoalSelector> sel;
  explicit Rig(const SelectorParams& p, double robot_x = 0.35, double robot_y = 0.45) {
    sel = std::make_unique<GoalSelector>(p);
    sel->onState(0.0, Pose{robot_x, robot_y, 0.0, 0.0});
  }
  void feed(const Rows& rows, double t) { sel->onOccGrid(t, gridFromRows(rows)); }
  void feed(const std::vector<int8_t>& raw, int w, int h, double t) { sel->onOccGrid(t, gridFromRaw(raw, w, h)); }
  const std::vector<FrontierRecord>& records() const { return sel->frontierManager().records(); }
};

/** Independent oracle for the derived frontier map (spec N1/N5), brute force.
 *  Codes: 100 occupied, 99 cell within occ_m of an occupied cell (any non-occupied cell, free or not),
 *  -1 real unknown, 50 non-unknown cell within unk_m of an unknown cell, 0 safe free.
 *  Precedence: 100 > 99 > -1 > 50 > 0 (see the ASSUMPTION test for the overlap cases). */
std::vector<int8_t> expectedMap(const std::vector<int8_t>& raw, int w, int h, double occ_m, double unk_m) {
  std::vector<int8_t> out(raw.size(), 0);
  auto near = [&](int x, int y, int8_t kind, double radius_m) {
    if (radius_m <= 0.0) return false;
    for (int yy = 0; yy < h; ++yy)
      for (int xx = 0; xx < w; ++xx) {
        const int8_t r = raw[xx + static_cast<size_t>(w) * yy];
        const bool is_kind = (kind == -1) ? (r < 0) : (r >= 100);
        if (!is_kind || (xx == x && yy == y)) continue;
        if (kRes * std::hypot(double(xx - x), double(yy - y)) <= radius_m + 1e-6) return true;
      }
    return false;
  };
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const size_t i = x + static_cast<size_t>(w) * y;
      if (raw[i] >= 100) { out[i] = 100; continue; }
      if (near(x, y, 100, occ_m)) { out[i] = 99; continue; }
      if (raw[i] < 0) { out[i] = -1; continue; }
      out[i] = near(x, y, -1, unk_m) ? 50 : 0;
    }
  return out;
}

/** 'b' 'u' '?' '#' '.' art -> derived map codes. */
std::vector<int8_t> mapFromArt(const Rows& art) {
  std::vector<int8_t> v;
  for (const auto& r : art)
    for (char c : r) v.push_back(c == '#' ? 100 : c == 'b' ? 99 : c == 'u' ? 50 : c == '?' ? -1 : 0);
  return v;
}

std::string artOf(const std::vector<int8_t>& m, int w) {
  std::string s;
  for (size_t i = 0; i < m.size(); ++i) {
    if (i % w == 0) s += "\n";
    s += (m[i] == 100 ? '#' : m[i] == 99 ? 'b' : m[i] == 50 ? 'u' : m[i] == -1 ? '?' : '.');
  }
  return s;
}

bool recordNear(const std::vector<FrontierRecord>& recs, double x, double y, double tol = 1e-6) {
  for (const auto& r : recs)
    if (std::abs(r.centroid_xy.x() - x) < tol && std::abs(r.centroid_xy.y() - y) < tol) return true;
  return false;
}

size_t countState(const std::vector<FrontierRecord>& recs, FrontierState s) {
  return static_cast<size_t>(std::count_if(recs.begin(), recs.end(), [&](const auto& r) { return r.state == s; }));
}

/** Run the detector on a grid built from an expected derived map (99/100 -> occupied, 50/-1 -> unknown,
 *  passable = the 50 cells), i.e. the spec's definition of the frontier grid, independent of the selector. */
std::vector<FrontierCluster> detectOnExpected(const std::vector<int8_t>& exp, int w, int h,
                                              const Eigen::Vector2d& robot, int min_cells) {
  std::vector<int8_t> tri(exp.size());
  std::vector<uint8_t> passable(exp.size(), 0);
  for (size_t i = 0; i < exp.size(); ++i) {
    tri[i] = (exp[i] >= 99) ? 100 : (exp[i] < 0 || exp[i] == 50) ? -1 : 0;
    passable[i] = (exp[i] == 50);
  }
  auto grid = OccGrid2D::fromTristate(w, h, kRes, 0.0, 0.0, tri);
  FrontierDetectorParams dp;
  dp.cluster_min_cells = min_cells;
  dp.border_margin_cells = 0;
  dp.obstacle_clearance_cells = 0;
  dp.robot_snap_radius_m = 1.0;
  return FrontierDetector(dp).detect(*grid, robot, nullptr, &passable);
}

}  // namespace

// =============================================================================================
// selectorMap()
// =============================================================================================

// A broad irregular map shared by the selectorMap and frontier-cell tests (34 x 16). The radius-2
// neighbourhoods of the different features do not overlap, so precedence questions do not arise.
// Features (x range, y range; y = 0 is the first row):
//   A: unknown block   x 20..27, y 0..4    (touches the top border)
//   B: unknown block   x 29..33, y 8..15   (touches the right and bottom borders)
//   W: occupied wall   x 6..7,   y 9..12
//   H: one-cell unknown hole at (14, 10)
//   P: one occupied cell at (24, 12)
// Everything else is free; the robot starts at (0.25, 0.25) = cell (2, 2).
Rows irregularMap() {
  const int W = 34, H = 16;
  std::vector<std::string> rows(H, std::string(W, '.'));
  auto fill = [&](int x0, int x1, int y0, int y1, char c) {
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) rows[y][x] = c;
  };
  fill(20, 27, 0, 4, '?');    // A
  fill(29, 33, 8, 15, '?');   // B
  fill(6, 7, 9, 12, '#');     // W
  fill(14, 14, 10, 10, '?');  // H
  fill(24, 24, 12, 12, '#');  // P
  return rows;
}

// Scenario: irregular map, occupied inflation 0.2 m (2 cells), unknown inflation 0.2 m (2 cells).
// Expected: selectorMap() equals the independent brute-force oracle cell by cell, so every value is one of
// {0, 50, 99, 100, -1} with the documented meaning, and the geometry fields are copied from the grid.
// Why: spec section 11 item 20 and N1 (occupied band = within inflation_2d_m of an occupied cell; unknown band
// = non-occupied cells within unknown_inflation_2d_m of unknown; real unknown stays -1).
TEST(SelectorMap, ValuesAreOnlyTheDocumentedCodesAndMatchTheDefinition) {
  const Rows rows = irregularMap();
  const int W = static_cast<int>(rows[0].size()), H = static_cast<int>(rows.size());
  Rig rig(makeParams(W, H, 0.2, 0.2), 0.25, 0.25);
  rig.feed(rows, 1.0);

  SelectorMap m;
  ASSERT_TRUE(rig.sel->selectorMap(m));
  EXPECT_EQ(m.width, W);
  EXPECT_EQ(m.height, H);
  EXPECT_DOUBLE_EQ(m.resolution, kRes);
  EXPECT_DOUBLE_EQ(m.origin_x, 0.0);
  EXPECT_DOUBLE_EQ(m.origin_y, 0.0);
  ASSERT_EQ(m.data.size(), static_cast<size_t>(W) * H);

  const std::set<int8_t> allowed = {0, 50, 99, 100, -1};
  for (size_t i = 0; i < m.data.size(); ++i)
    ASSERT_TRUE(allowed.count(m.data[i])) << "cell " << i << " has undocumented value " << int(m.data[i]);

  const auto exp = expectedMap(rawFromRows(rows), W, H, 0.2, 0.2);
  EXPECT_EQ(m.data, exp) << "got:" << artOf(m.data, W) << "\nexpected:" << artOf(exp, W);
  // all five codes occur in this map
  for (int8_t code : allowed) EXPECT_TRUE(std::count(m.data.begin(), m.data.end(), code) > 0) << int(code);
}

// Scenario: 16 x 5 map with one occupied cell, inflation 0.1 m (1 cell), unknown strip, unknown_inflation 0.2 m.
//   raw:                         expected selectorMap():
//   y0 ................          ............uuuu
//   y1 ................          ..b........uuuuu
//   y2 ..#.........????          .b#b......uu????
//   y3 ................          ..b........uuuuu
//   y4 ................          ............uuuu
// Expected: exactly the picture on the right (hand derived from the radius discs).
// Why: item 20 "free 0, obstacle 100, obstacle band 99, unknown band 50, unknown -1".
TEST(SelectorMap, EachCodeHasTheDocumentedMeaning) {
  const Rows raw = {"................",
                    "................",
                    "..#.........????",
                    "................",
                    "................"};
  const Rows art = {"............uuuu",
                    "..b........uuuuu",
                    ".b#b......uu????",
                    "..b........uuuuu",
                    "............uuuu"};
  Rig rig(makeParams(16, 5, 0.1, 0.2), 0.25, 0.45);
  rig.feed(raw, 1.0);
  SelectorMap m;
  ASSERT_TRUE(rig.sel->selectorMap(m));
  const auto exp = mapFromArt(art);
  EXPECT_EQ(m.data, exp) << "got:" << artOf(m.data, 16) << "\nexpected:" << artOf(exp, 16);
}

// Scenario: a free cell that is within BOTH radii (occupied cell at x=5, unknown cell at x=7 in a 12 x 3 map,
// inflation 0.1 m, unknown_inflation 0.2 m): cell x=6 is 1 from each.
//   raw: y1 "...... " -> ".....#.?....."  (x=5 occupied, x=7 unknown), rows 0 and 2 free.
// ASSUMPTION: the occupied band wins (99): it is blocked, a stricter state than "frontier-generating, not an
// obstacle" (N1 table). The spec does not spell out the overlap. Real unknown inside an occupied band (x=7 is
// not within 1 cell of x=5, so not tested here) is deliberately not asserted: the spec does not say.
TEST(SelectorMap, OccupiedBandWinsOverUnknownBandOnFreeCells) {
  const Rows raw = {"............",
                    ".....#.?....",
                    "............"};
  Rig rig(makeParams(12, 3, 0.1, 0.2), 0.05, 0.05);
  rig.feed(raw, 1.0);
  SelectorMap m;
  ASSERT_TRUE(rig.sel->selectorMap(m));
  EXPECT_EQ(m.data[6 + 12 * 1], 99) << "free cell within both bands";
  EXPECT_EQ(m.data[5 + 12 * 1], 100);
  EXPECT_EQ(m.data[7 + 12 * 1], -1);
  EXPECT_EQ(m.data[8 + 12 * 1], 50) << "within 1 of unknown, outside the occupied band";
}

// Scenario: exploration disabled. Feed the irregular map; ask for selectorMap().
// Expected: a map identical to the one the exploring selector builds, no frontier detection (no records), and
// hasOccGrid() true. Why: section 11 item 19 "The selector always subscribes to occ_2d_topic, also with
// exploration off ... With exploration off it runs no frontier detection" and notes D ("still subscribes,
// runs no frontier detection, publishes selector_map_2d").
TEST(SelectorMap, ExplorationOffStillPublishesTheSameMapAndRunsNoDetection) {
  const Rows rows = irregularMap();
  const int W = static_cast<int>(rows[0].size()), H = static_cast<int>(rows.size());

  SelectorParams on = makeParams(W, H, 0.2, 0.2);
  SelectorParams off = on;
  off.expl_enabled = false;
  Rig rig_on(on, 0.25, 0.25), rig_off(off, 0.25, 0.25);
  rig_on.feed(rows, 1.0);
  rig_off.feed(rows, 1.0);

  EXPECT_TRUE(rig_off.sel->hasOccGrid());
  EXPECT_TRUE(rig_off.records().empty()) << "no detection with exploration off";
  EXPECT_FALSE(rig_on.records().empty()) << "control: exploration on does detect";

  SelectorMap a, b;
  ASSERT_TRUE(rig_on.sel->selectorMap(a));
  ASSERT_TRUE(rig_off.sel->selectorMap(b));
  EXPECT_EQ(a.data, b.data);
  EXPECT_EQ(b.data, expectedMap(rawFromRows(rows), W, H, 0.2, 0.2));
}

// Scenario: selectorMap() before any occ_2d grid was received.
// Expected: false (nothing to publish). Why: header doc "False if no grid has been received".
TEST(SelectorMap, NoMapBeforeTheFirstGrid) {
  Rig rig(makeParams(10, 5, 0.1));
  SelectorMap m;
  EXPECT_FALSE(rig.sel->hasOccGrid());
  EXPECT_FALSE(rig.sel->selectorMap(m));
}

// =============================================================================================
// Frontier cells
// =============================================================================================

// Scenario: free columns x 0..13, unknown columns x 14..23, 9 rows, no obstacles, unknown_inflation 0.2 m.
//   y0..y8: ..............??????????      band = free x 12,13   frontier = x 11
// Expected: exactly one frontier cluster = the column x = 11 (centroid (1.15, 0.45), 9 cells), NOT the column
// next to the real unknown (x = 13) and not the band column x = 12.
// Why: notes C bullet 1 "Frontiers must sit next to inflated unknown"; N5 "frontier cell = safe-free cell
// with an 8-neighbour in the unknown band ... at least unknown_inflation_2d_m from real unknown".
TEST(FrontierMap, FrontierSitsTwoCellsFromRealUnknownNextToTheUnknownBand) {
  Rig rig(makeParams(24, 9, 0.0, 0.2));
  rig.feed(halfUnknown(24, 9, 14), 1.0);

  ASSERT_EQ(rig.records().size(), 1u);
  const auto& r = rig.records()[0];
  EXPECT_EQ(r.state, FrontierState::ACTIVE);
  EXPECT_EQ(r.size_cells, 9);
  EXPECT_NEAR(r.centroid_xy.x(), 1.15, 1e-9);
  EXPECT_NEAR(r.centroid_xy.y(), 0.45, 1e-9);
  EXPECT_NEAR(r.aabb_min.x(), 1.15, 1e-9);
  EXPECT_NEAR(r.aabb_max.x(), 1.15, 1e-9);
}

// Scenario: the irregular map (unknown blocks touching borders, a one-cell hole, a wall, a lone obstacle).
// The detector is run on the grid the SPEC defines (oracle bands) with cluster_min_cells = 1, and the selector
// with cluster_min_cells = 1 on the raw grid.
// Expected (a) every cell of every detected cluster is raw FREE, derived 0 (outside both bands) and has an
// 8-neighbour with derived 50 (the unknown band); never occupied, band or unknown. (b) Completeness: the union of
// cluster cells equals exactly the set of reachable safe-free cells with an unknown-band neighbour (everything is
// reachable in this map). (c) The selector's records are those clusters (same count, sizes, centroids).
// Why: notes C bullet 1 ("not in real or inflated occupied or unknown; only in free space; next to inflated
// unknown"), N5 frontier definition.
TEST(FrontierMap, FrontierCellsAreFreeSafeAndAdjacentToTheBandOnAnIrregularMap) {
  const Rows rows = irregularMap();
  const int W = static_cast<int>(rows[0].size()), H = static_cast<int>(rows.size());
  const auto raw = rawFromRows(rows);
  const auto exp = expectedMap(raw, W, H, 0.2, 0.2);
  const Eigen::Vector2d robot(0.25, 0.25);

  const auto clusters = detectOnExpected(exp, W, H, robot, 1);
  ASSERT_FALSE(clusters.empty());

  std::set<std::pair<int, int>> found;
  for (const auto& c : clusters)
    for (const auto& p : c.cells) {
      const int x = static_cast<int>(std::floor(p.x() / kRes));
      const int y = static_cast<int>(std::floor(p.y() / kRes));
      found.insert({x, y});
      const size_t i = x + static_cast<size_t>(W) * y;
      EXPECT_EQ(raw[i], 0) << "frontier cell (" << x << "," << y << ") is not raw free";
      EXPECT_EQ(exp[i], 0) << "frontier cell (" << x << "," << y << ") is in a band";
      bool band_nbr = false;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if ((dx || dy) && nx >= 0 && nx < W && ny >= 0 && ny < H && exp[nx + W * ny] == 50) band_nbr = true;
        }
      EXPECT_TRUE(band_nbr) << "frontier cell (" << x << "," << y << ") has no unknown-band neighbour";
    }

  // (b) completeness: BFS over free + band cells (8-connected) from the robot cell, candidates = safe-free
  // cells with a band neighbour.
  std::vector<uint8_t> reach(exp.size(), 0);
  std::vector<std::pair<int, int>> q = {{2, 2}};
  reach[2 + W * 2] = 1;
  for (size_t h = 0; h < q.size(); ++h) {
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = q[h].first + dx, ny = q[h].second + dy;
        if (nx < 0 || nx >= W || ny < 0 || ny >= H || reach[nx + W * ny]) continue;
        if (exp[nx + W * ny] != 0 && exp[nx + W * ny] != 50) continue;
        reach[nx + W * ny] = 1;
        q.push_back({nx, ny});
      }
  }
  std::set<std::pair<int, int>> want;
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      if (!reach[x + W * y] || exp[x + W * y] != 0) continue;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if ((dx || dy) && nx >= 0 && nx < W && ny >= 0 && ny < H && exp[nx + W * ny] == 50) want.insert({x, y});
        }
    }
  EXPECT_EQ(found, want);

  // (c) the selector, fed the raw grid, ends up with the same clusters as records.
  SelectorParams p = makeParams(W, H, 0.2, 0.2);
  p.expl_cluster_min_cells = 1;
  Rig rig(p, 0.25, 0.25);
  rig.feed(rows, 1.0);
  ASSERT_EQ(rig.records().size(), clusters.size());
  for (const auto& c : clusters) {
    EXPECT_TRUE(recordNear(rig.records(), c.centroid.x(), c.centroid.y(), 1e-9))
        << "no record at cluster centroid (" << c.centroid.x() << "," << c.centroid.y() << ")";
  }
  int total_records = 0, total_clusters = 0;
  for (const auto& r : rig.records()) total_records += r.size_cells;
  for (const auto& c : clusters) total_clusters += c.size_cells;
  EXPECT_EQ(total_records, total_clusters);
}

// =============================================================================================
// Reachability (N5 Reachability bullet)
// =============================================================================================

// Corridor used by the reachability tests: 30 x 7 cells, walls on rows 0 and 6, free corridor rows 1..5, real
// unknown on the right (x 24..29), robot at x = 3 on the left, unknown_inflation 0.2 m. The test-specific
// feature sits in column x = 14. Schematic (y = 0 on top, '*' marks where the variants differ):
//
//   y0  ##############################
//   y1  ..............*.........??????
//   y2  ..............*.........??????
//   y3  ..............*.........??????
//   y4  ..............*.........??????
//   y5  ..............*.........??????
//   y6  ##############################
//
// The far frontier is the column x = 21 (rows 1..5): centroid (2.15, 0.35), 5 cells.
// corridor(c, true) puts char c only at (14, 3) (a one-cell hole when c = '?'); corridor(c, false) fills the whole
// column x = 14, rows 1..5.
Rows corridor(char c, bool single_cell_hole) {
  Rows rows(7, std::string(30, '.'));
  rows[0] = std::string(30, '#');
  rows[6] = std::string(30, '#');
  for (int y = 1; y <= 5; ++y)
    for (int x = 24; x < 30; ++x) rows[y][x] = '?';
  if (single_cell_hole) {
    rows[3][14] = c;
  } else {
    for (int y = 1; y <= 5; ++y) rows[y][14] = c;
  }
  return rows;
}

// Scenario: corridor with a one-cell unknown hole in the middle (14, 3), real unknown at the far end.
// The hole's own unknown band (radius 2 cells) covers (14, 1) and (14, 5), the only ways around it, so the
// walk MUST cross unknown-band cells to get past the hole.
// Expected: the frontier at the far end (x = 21) is found.
// Why: N5 Reachability bullet: "the reachability walk from the robot may cross unknown-band cells (they are known
// free) ... a frontier beyond a 1-cell unknown hole in a corridor is found" (replay finding: 1-2 clusters vs
// 17-22 with safe-free-only walking).
TEST(Reachability, FrontierBeyondOneCellHoleIsFound) {
  Rig rig(makeParams(30, 7, 0.0, 0.2), 0.35, 0.35);
  rig.feed(corridor('?', true), 1.0);

  EXPECT_TRUE(recordNear(rig.records(), 2.15, 0.35)) << "far-end frontier behind the hole was not found";
  for (const auto& r : rig.records())
    if (std::abs(r.centroid_xy.x() - 2.15) < 1e-6) EXPECT_EQ(r.size_cells, 5);
}

// Scenario: the same corridor but the "hole" is a full-width column of real unknown (x = 14, rows 1..5).
// Expected: the far-end frontier is NOT found (the walk never crosses real unknown); frontier clusters around
// the hole on the robot's side still exist.
// Why: N5 Reachability bullet "... but never real unknown".
TEST(Reachability, RealUnknownAcrossTheWholeCorridorIsNeverCrossed) {
  Rig rig(makeParams(30, 7, 0.0, 0.2), 0.35, 0.35);
  rig.feed(corridor('?', false), 1.0);

  EXPECT_FALSE(recordNear(rig.records(), 2.15, 0.35));
  ASSERT_FALSE(rig.records().empty()) << "control: frontier on the robot's side of the hole exists";
  for (const auto& r : rig.records())
    EXPECT_LT(r.centroid_xy.x(), 1.6) << "a frontier beyond the real-unknown column was reachable";
}

// Scenario: a full-height wall at x = 14 (rows 1..5) with a one-cell gap at (14, 3) and inflation 0.2 m.
// The gap is within 1 cell of wall cells, so it is occupied band. Control: same map with inflation 0 (gap is safe
// free).
// Expected: with the band the far frontier is NOT found (band is never crossed -> robot side has no unknown
// at all -> no records); without the band it IS found.
// Why: N5 Reachability bullet "... never ... the occupied band".
TEST(Reachability, OccupiedBandIsNeverCrossedEvenThroughAGap) {
  Rows rows = corridor('#', false);
  rows[3][14] = '.';  // the gap

  Rig blocked(makeParams(30, 7, 0.2, 0.2), 0.35, 0.35);
  blocked.feed(rows, 1.0);
  EXPECT_TRUE(blocked.records().empty()) << "frontier reached through the occupied band";

  Rig open(makeParams(30, 7, 0.0, 0.2), 0.35, 0.35);
  open.feed(rows, 1.0);
  EXPECT_TRUE(recordNear(open.records(), 2.15, 0.35)) << "control: the gap is passable without inflation";
}

// =============================================================================================
// Lifecycle on the derived grid
// =============================================================================================

// Scenario (three cycles, merge_radius 0.05 m so a one-cell shift is a NEW record; verify_radius_cells 2;
// unknown_inflation 0.2 m = 2 cells; robot far away so the dwell check never fires):
//   cycle 1 (t=1): free x<14, unknown x>=14  -> frontier column x=11 (record A, ACTIVE)
//   cycle 2 (t=2): free x<15                 -> frontier column x=12 (record B). A is not re-matched. Raw unknown
//                  is 4 cells from A's centroid cell (x=11) but the unknown BAND (x 13,14) is 2 cells away.
//   cycle 3 (t=3): free x<20                 -> frontier column x=17 (record C). Band (x 18,19) is >2 cells from A
//                  and B.
// Expected: A is ACTIVE right after cycle 1 (no instant VISITED) and stays ACTIVE in cycle 2 (the band is within
// verify_radius_cells; a raw-unknown test would already say VISITED), and A and B are VISITED after cycle 3.
// Why: notes C bullet 2; N5 "VISITED verify ... must test the unknown band instead of raw unknown"; item 18.
TEST(Lifecycle, NotInstantlyVisitedAndVisitedOnlyOnceTheBandIsGone) {
  SelectorParams p = makeParams(24, 9, 0.0, 0.2);
  p.expl_merge_radius_m = 0.05;
  Rig rig(p);

  rig.feed(halfUnknown(24, 9, 14), 1.0);
  ASSERT_EQ(rig.records().size(), 1u);
  ASSERT_TRUE(recordNear(rig.records(), 1.15, 0.45));
  EXPECT_EQ(rig.records()[0].state, FrontierState::ACTIVE) << "VISITED on the cycle it was detected";

  rig.feed(halfUnknown(24, 9, 15), 2.0);
  ASSERT_EQ(rig.records().size(), 2u);
  ASSERT_TRUE(recordNear(rig.records(), 1.25, 0.45));
  EXPECT_EQ(rig.records()[0].state, FrontierState::ACTIVE)
      << "unknown-band cell within verify_radius_cells of the centroid must keep it ACTIVE";
  EXPECT_EQ(rig.records()[1].state, FrontierState::ACTIVE);

  rig.feed(halfUnknown(24, 9, 20), 3.0);
  ASSERT_EQ(rig.records().size(), 3u);
  EXPECT_EQ(rig.records()[0].state, FrontierState::VISITED);
  EXPECT_EQ(rig.records()[1].state, FrontierState::VISITED);
  EXPECT_EQ(rig.records()[2].state, FrontierState::ACTIVE);
  EXPECT_NEAR(rig.records()[2].centroid_xy.x(), 1.75, 1e-9);
}

// Scenario (inflation 0.2 m, unknown_inflation 0.2 m, keep-out radius 1.5 m, cooldown 10 s):
//   t=1   free x<14, unknown x>=14           -> frontier A at (1.15, 0.45), ACTIVE
//   t=2   an occupied column appears at x=13 (unknown behind it) -> A's centroid cell x=11 is now in the
//         occupied band (raw cell is free!) and nothing is re-detected -> A INVALIDATED (invalidated_at 2)
//   t=3   obstacle gone again (free x<14 as at t=1) -> the area is a frontier again, but the keep-out is active
//         -> no new record
//   t=11.9 still inside the cooldown (2 + 10 = 12) -> still no new record
//   t=12.1 cooldown over -> a new ACTIVE candidate appears at (1.15, 0.45); the old record stays INVALIDATED.
// Why: notes C bullet 3 ("INVALIDATED when it is not re-detected ... new candidate appears there once the
// invalidation keep-out has expired, not before; current behaviour kept 2026-10-10").
// ASSUMPTION: the notes name `merge_radius_m` and `invalidation_cooldown_sec`; the code's keep-out radius is
// `invalidation_keep_out_radius_m` (1.5 m here, larger than merge_radius 0.5 m), the re-detected cluster is at
// the same spot, so both readings give the same result in this test.
TEST(Lifecycle, CentroidInObstacleMarginInvalidatedThenKeepOutThenNewCandidate) {
  Rig rig(makeParams(24, 9, 0.2, 0.2));

  rig.feed(halfUnknown(24, 9, 14), 1.0);
  ASSERT_EQ(rig.records().size(), 1u);
  ASSERT_EQ(rig.records()[0].state, FrontierState::ACTIVE);
  ASSERT_TRUE(recordNear(rig.records(), 1.15, 0.45));

  const Rows walled(9, std::string(13, '.') + "#" + std::string(10, '?'));
  rig.feed(walled, 2.0);
  ASSERT_EQ(rig.records().size(), 1u);
  EXPECT_EQ(rig.records()[0].state, FrontierState::INVALIDATED);
  EXPECT_DOUBLE_EQ(rig.records()[0].invalidated_at_t, 2.0);

  rig.feed(halfUnknown(24, 9, 14), 3.0);
  EXPECT_EQ(rig.records().size(), 1u) << "new candidate spawned inside the keep-out";
  EXPECT_EQ(countState(rig.records(), FrontierState::ACTIVE), 0u);

  rig.feed(halfUnknown(24, 9, 14), 11.9);
  EXPECT_EQ(rig.records().size(), 1u) << "new candidate spawned before the cooldown expired";

  rig.feed(halfUnknown(24, 9, 14), 12.1);
  ASSERT_EQ(rig.records().size(), 2u) << "no new candidate after the cooldown expired";
  EXPECT_EQ(rig.records()[0].state, FrontierState::INVALIDATED);
  EXPECT_EQ(rig.records()[1].state, FrontierState::ACTIVE);
  EXPECT_NEAR(rig.records()[1].centroid_xy.x(), 1.15, 1e-9);
}

// =============================================================================================
// Obstacles next to unknown
// =============================================================================================

// Scenario: free x 0..9, occupied wall column x = 10, real unknown x 11..19 (7 rows). The unknown edge is
// directly behind the wall.
//   .........?#??????????      (y0..y6; wall at x=10)
// With inflation 0.2 m (2 cells) the occupied band covers x 8,9 which is exactly where the unknown band
// (x 9,10 ...) would be, so no free cell outside both bands has an unknown-band neighbour.
// Expected: no frontier. Control (inflation 0): the band cell x = 9 is free, the safe cell x = 8 is a frontier
// column (centroid (0.85, 0.35), 7 cells).
// Why: notes C bullet 4 and N5 "unknown edges within inflation_2d_m of an obstacle produce no frontier (the
// occupied band covers their unknown band)".
TEST(NoFrontierNearObstacles, UnknownEdgeWithinInflationOfAnObstacleGivesNoFrontier) {
  const Rows rows(7, std::string(10, '.') + "#" + std::string(9, '?'));

  Rig inflated(makeParams(20, 7, 0.2, 0.2), 0.35, 0.35);
  inflated.feed(rows, 1.0);
  EXPECT_TRUE(inflated.records().empty());

  Rig plain(makeParams(20, 7, 0.0, 0.2), 0.35, 0.35);
  plain.feed(rows, 1.0);
  ASSERT_EQ(plain.records().size(), 1u) << "control: without occupied inflation the same edge yields a frontier";
  EXPECT_EQ(plain.records()[0].size_cells, 7);
  EXPECT_NEAR(plain.records()[0].centroid_xy.x(), 0.85, 1e-9);
}
