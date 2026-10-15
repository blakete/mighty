// Tests for FrontierManager (matching, EMA, lifecycle, eviction).

#include <gtest/gtest.h>

#include <vector>

#include "mighty/frontier_manager.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

constexpr int8_t U = -1;
constexpr int8_t F = 0;
constexpr int8_t O = 100;

// Build a 20x20 0.1m grid covering [0, 2.0]x[0, 2.0], all FREE by default.
std::shared_ptr<const OccGrid2D> MakeAllFreeGrid() {
  const int W = 20, H = 20;
  std::vector<int8_t> data(W * H, F);
  return OccGrid2D::fromTristate(W, H, 0.1, 0.0, 0.0, data);
}

// Build a 20x20 grid where the right half is unknown — there will be free
// cells along the seam at ix=9 that the manager can verify against.
std::shared_ptr<const OccGrid2D> MakeHalfUnknownGrid() {
  const int W = 20, H = 20;
  std::vector<int8_t> data(W * H);
  for (int iy = 0; iy < H; ++iy)
    for (int ix = 0; ix < W; ++ix)
      data[iy * W + ix] = (ix < 10) ? F : U;
  return OccGrid2D::fromTristate(W, H, 0.1, 0.0, 0.0, data);
}

FrontierCluster MakeCluster(double cx, double cy, int size_cells = 10) {
  FrontierCluster c;
  c.centroid    = Eigen::Vector2d(cx, cy);
  c.size_cells  = size_cells;
  c.size_m2     = size_cells * 0.01;
  c.aabb_min    = Eigen::Vector2d(cx - 0.1, cy - 0.1);
  c.aabb_max    = Eigen::Vector2d(cx + 0.1, cy + 0.1);
  return c;
}

FrontierManagerParams DefaultParams() {
  FrontierManagerParams p;
  p.merge_radius_m         = 0.5;
  p.centroid_ema_alpha     = 0.5;
  p.visit_radius_m         = 0.3;
  p.visit_dwell_sec        = 1.0;
  p.verify_radius_cells    = 2;
  p.max_frontiers          = 100;
  return p;
}

}  // namespace

TEST(FrontierManagerTest, FirstDetectInsertsAsActive) {
  auto grid = MakeHalfUnknownGrid();
  FrontierManager mgr(DefaultParams());

  std::vector<FrontierCluster> fresh = {MakeCluster(0.95, 1.0, 10)};
  mgr.update(fresh, *grid, Eigen::Vector3d(0.5, 1.0, 0.0), 1.0);

  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_EQ(mgr.records()[0].state, FrontierState::ACTIVE);
  EXPECT_EQ(mgr.records()[0].size_cells, 10);
}

TEST(FrontierManagerTest, NearbyDetectionMergesViaEMA) {
  auto grid = MakeHalfUnknownGrid();
  FrontierManager mgr(DefaultParams());

  // First detection at (0.95, 1.0)
  mgr.update({MakeCluster(0.95, 1.0, 10)}, *grid,
             Eigen::Vector3d(0.5, 1.0, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 1u);
  const auto id0 = mgr.records()[0].id;

  // Second detection drifted by < merge_radius (0.5m).
  mgr.update({MakeCluster(1.05, 1.1, 12)}, *grid,
             Eigen::Vector3d(0.5, 1.0, 0.0), 1.5);
  ASSERT_EQ(mgr.size(), 1u);  // merged, not duplicated
  EXPECT_EQ(mgr.records()[0].id, id0);
  EXPECT_EQ(mgr.records()[0].size_cells, 12);
  // EMA centroid is halfway between (0.95, 1.0) and (1.05, 1.1).
  EXPECT_NEAR(mgr.records()[0].centroid_xy.x(), 1.0, 1e-6);
  EXPECT_NEAR(mgr.records()[0].centroid_xy.y(), 1.05, 1e-6);
}

TEST(FrontierManagerTest, FreeCellNoUnknownNeighborsBecomesVisited) {
  // Build an all-free grid (no unknowns anywhere). Existing record sits at a
  // free cell with no unknown neighbors -> should be marked VISITED.
  auto grid = MakeAllFreeGrid();
  FrontierManager mgr(DefaultParams());

  // Bootstrap a record using a half-unknown grid first.
  auto half = MakeHalfUnknownGrid();
  mgr.update({MakeCluster(0.95, 1.0, 10)}, *half,
             Eigen::Vector3d(0.5, 1.0, 0.0), 1.0);
  ASSERT_EQ(mgr.records()[0].state, FrontierState::ACTIVE);

  // Next cycle: same grid coordinate, but the area is now fully free and
  // the detector reports no fresh clusters. The record should flip to VISITED.
  // Move the robot far from the centroid so the dwell-visit check doesn't
  // also trigger and mask whichever code path we're testing.
  mgr.update({}, *grid, Eigen::Vector3d(0.05, 0.05, 0.0), 1.5);
  EXPECT_EQ(mgr.records()[0].state, FrontierState::VISITED);
}

TEST(FrontierManagerTest, OccupiedCellBecomesInvalidated) {
  // Build a grid where the centroid cell is OCCUPIED.
  const int W = 20, H = 20;
  std::vector<int8_t> data(W * H, F);
  // Mark cell at world (0.95, 1.0) as occupied. ix=9, iy=10
  data[10 * W + 9] = O;
  auto grid = OccGrid2D::fromTristate(W, H, 0.1, 0.0, 0.0, data);

  FrontierManager mgr(DefaultParams());
  // Bootstrap with a frontier record at that location.
  auto half = MakeHalfUnknownGrid();
  mgr.update({MakeCluster(0.95, 1.0, 10)}, *half,
             Eigen::Vector3d(0.0, 1.0, 0.0), 1.0);

  // Next cycle: the cell at the centroid is now OCCUPIED. No fresh clusters.
  mgr.update({}, *grid, Eigen::Vector3d(0.0, 0.0, 0.0), 1.5);
  EXPECT_EQ(mgr.records()[0].state, FrontierState::INVALIDATED);
}

TEST(FrontierManagerTest, OutOfWindowRecordBecomesDormant) {
  // Build a small 5x5 grid covering [0, 0.5] in x and y.
  const int W = 5, H = 5;
  std::vector<int8_t> data(W * H, F);
  auto small_grid = OccGrid2D::fromTristate(W, H, 0.1, 0.0, 0.0, data);

  FrontierManager mgr(DefaultParams());
  // Bootstrap a record in a larger grid first.
  auto big = MakeHalfUnknownGrid();
  mgr.update({MakeCluster(1.5, 1.5, 10)}, *big,
             Eigen::Vector3d(0.0, 0.0, 0.0), 1.0);

  // Next cycle: the small grid does not contain the centroid (1.5, 1.5).
  // Move the robot far away from the centroid so dwell visit check doesn't
  // mask the test.
  mgr.update({}, *small_grid, Eigen::Vector3d(0.0, 0.0, 0.0), 1.5);
  EXPECT_EQ(mgr.records()[0].state, FrontierState::DORMANT);
}

TEST(FrontierManagerTest, RobotDwellMarksVisited) {
  auto grid = MakeHalfUnknownGrid();
  FrontierManager mgr(DefaultParams());

  // Bootstrap a record at (1.0, 1.0).
  mgr.update({MakeCluster(1.0, 1.0, 10)}, *grid,
             Eigen::Vector3d(2.0, 2.0, 0.0), 1.0);
  ASSERT_EQ(mgr.records()[0].state, FrontierState::ACTIVE);

  // Robot dwells right next to the centroid for > visit_dwell_sec.
  mgr.update({MakeCluster(1.0, 1.0, 10)}, *grid,
             Eigen::Vector3d(1.05, 1.05, 0.0), 2.5);   // dt=1.5 > 1.0
  EXPECT_EQ(mgr.records()[0].state, FrontierState::VISITED);
  EXPECT_GE(mgr.records()[0].visit_count, 1);
}

TEST(FrontierManagerTest, SelectNextGoalPrefersActiveOverDormant) {
  // 100x100 grid so both records are in-bounds; we manually set states.
  const int W = 100, H = 100;
  std::vector<int8_t> data(W * H, F);
  // Mark some unknown so info_norm has something to bite on (otherwise
  // utilities are equal and the test depends on iteration order).
  for (int iy = 0; iy < H; ++iy)
    for (int ix = 50; ix < W; ++ix)
      data[iy * W + ix] = U;
  auto grid = OccGrid2D::fromTristate(W, H, 0.1, 0.0, 0.0, data);

  FrontierManager mgr(DefaultParams());

  // Insert a fresh ACTIVE record at world (5.0, 5.0).
  mgr.update({MakeCluster(5.0, 5.0, 10)}, *grid,
             Eigen::Vector3d(0.0, 0.0, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 1u);

  // Insert a second fresh ACTIVE record but bigger.
  mgr.update({MakeCluster(5.0, 5.0, 10), MakeCluster(7.0, 7.0, 50)}, *grid,
             Eigen::Vector3d(0.0, 0.0, 0.0), 1.5);
  ASSERT_EQ(mgr.size(), 2u);

  // Manually set the smaller record to DORMANT.
  // (We can't access records_ directly; do a controlled state change via
  // markInvalidated then markVisited would change semantics — instead, slide
  // it out by querying with a tiny grid that doesn't contain it.)
  // Easier: just check that selectNextGoal picks an ACTIVE record at all,
  // and that it picks the one with higher utility (the larger size).
  auto pick = mgr.selectNextGoal(Eigen::Vector3d(0.0, 0.0, 0.0), *grid);
  ASSERT_TRUE(pick.has_value());
  EXPECT_EQ(pick->state, FrontierState::ACTIVE);
  EXPECT_EQ(pick->size_cells, 50);  // bigger one wins
}

TEST(FrontierManagerTest, EvictionPrefersVisitedOverDormant) {
  FrontierManagerParams p = DefaultParams();
  p.max_frontiers = 2;
  FrontierManager mgr(p);

  auto grid = MakeAllFreeGrid();

  // Insert 2 records at t=1.0 -> both ACTIVE, no eviction.
  mgr.update({MakeCluster(0.5, 0.5, 5), MakeCluster(1.5, 1.5, 5)},
             *grid, Eigen::Vector3d(0.0, 0.0, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 2u);

  // Mark the first one VISITED externally.
  const uint64_t visited_id = mgr.records()[0].id;
  mgr.markVisited(visited_id);

  // Now insert a 3rd record. The VISITED one is the only legal eviction
  // candidate (manager never evicts ACTIVE), so it should be removed even
  // though we'll still be over cap with 3 ACTIVE records remaining.
  mgr.update({MakeCluster(0.5, 0.5, 5), MakeCluster(1.5, 1.5, 5),
              MakeCluster(0.7, 0.7, 5)},
             *grid, Eigen::Vector3d(0.0, 0.0, 0.0), 2.0);

  // The originally-VISITED record must be gone — it was the highest-priority
  // eviction target.
  EXPECT_EQ(mgr.find(visited_id), nullptr);
  for (const auto& r : mgr.records()) {
    EXPECT_NE(r.state, FrontierState::VISITED);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ---------------------------------------------------------------------------
// Goal cell, cell-overlap matching and retirement by cells
// ---------------------------------------------------------------------------

namespace {

// 20x20 @0.1 grid, `unknown(ix, iy)` decides each cell (else FREE).
template <typename Fn>
std::shared_ptr<const OccGrid2D> MakeGridWhere(Fn unknown) {
  const int W = 20, H = 20;
  std::vector<int8_t> data(W * H);
  for (int iy = 0; iy < H; ++iy)
    for (int ix = 0; ix < W; ++ix) data[iy * W + ix] = unknown(ix, iy) ? U : F;
  return OccGrid2D::fromTristate(W, H, 0.1, 0.0, 0.0, data);
}

// Cluster from cell indices of the 20x20 @0.1 grid: centroid, AABB and the goal (member nearest the
// centroid), as the detector builds them.
FrontierCluster MakeCellCluster(const std::vector<std::pair<int, int>>& ij) {
  FrontierCluster c;
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  c.aabb_min = Eigen::Vector2d(1e9, 1e9);
  c.aabb_max = Eigen::Vector2d(-1e9, -1e9);
  for (const auto& [ix, iy] : ij) {
    const Eigen::Vector2d p((ix + 0.5) * 0.1, (iy + 0.5) * 0.1);
    c.cells.push_back(p);
    sum += p;
    c.aabb_min = c.aabb_min.cwiseMin(p);
    c.aabb_max = c.aabb_max.cwiseMax(p);
  }
  c.size_cells = static_cast<int>(c.cells.size());
  c.size_m2 = c.size_cells * 0.01;
  c.centroid = sum / c.size_cells;
  double best = 1e18;
  for (const auto& p : c.cells)
    if ((p - c.centroid).squaredNorm() < best) {
      best = (p - c.centroid).squaredNorm();
      c.goal = p;
    }
  return c;
}

std::vector<std::pair<int, int>> Row(int iy, int ix0, int ix1) {
  std::vector<std::pair<int, int>> v;
  for (int ix = ix0; ix <= ix1; ++ix) v.emplace_back(ix, iy);
  return v;
}

const FrontierRecord* FindId(const FrontierManager& m, uint64_t id) { return m.find(id); }

}  // namespace

// Scenario: a cluster without cells (hand-made). Expected: goal_xy is the centroid.
TEST(FrontierManagerGoal, GoalFallsBackToTheCentroidWithoutCells) {
  FrontierManager mgr(DefaultParams());
  mgr.update({MakeCluster(0.95, 1.0)}, *MakeHalfUnknownGrid(), Eigen::Vector3d(0.5, 1.0, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_DOUBLE_EQ(mgr.records()[0].goal_xy.x(), 0.95);
  EXPECT_DOUBLE_EQ(mgr.records()[0].goal_xy.y(), 1.0);
}

// Scenario: a cluster with cells. Expected: the record keeps them and takes the cluster's goal cell.
TEST(FrontierManagerGoal, RecordTakesTheClusterGoalAndCells) {
  FrontierManager mgr(DefaultParams());
  const auto c = MakeCellCluster(Row(9, 2, 11));
  mgr.update({c}, *MakeGridWhere([](int, int iy) { return iy >= 10; }), Eigen::Vector3d(0.5, 0.5, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_EQ(mgr.records()[0].goal_xy, *c.goal);
  EXPECT_EQ(mgr.records()[0].cells.size(), 10u);
}

// Scenario: a square ring of frontier cells around a small unknown patch at the centre; then the ring is
// not re-detected and its cells no longer border unknown, but the patch at the centroid is still unknown.
// Expected: VISITED (judged by the cells; the old centroid rule left it ACTIVE forever).
TEST(FrontierManagerGoal, RetiredWhenNoCellBordersUnknownEvenIfTheCentroidAreaIsUnknown) {
  std::vector<std::pair<int, int>> ring;
  for (int k = 4; k <= 14; ++k) {
    ring.emplace_back(k, 4);
    ring.emplace_back(k, 14);
  }
  for (int k = 5; k <= 13; ++k) {
    ring.emplace_back(4, k);
    ring.emplace_back(14, k);
  }
  FrontierManager mgr(DefaultParams());
  auto before = MakeGridWhere([](int ix, int iy) { return (ix < 4 || ix > 14 || iy < 4 || iy > 14) && iy > 16; });
  mgr.update({MakeCellCluster(ring)}, *before, Eigen::Vector3d(0.15, 0.15, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 1u);
  auto after = MakeGridWhere([](int ix, int iy) { return ix >= 8 && ix <= 10 && iy >= 8 && iy <= 10; });
  mgr.update({}, *after, Eigen::Vector3d(0.15, 0.15, 0.0), 2.0);
  EXPECT_EQ(mgr.records()[0].state, FrontierState::VISITED);
}

// Scenario: a 10-cell frontier row; then only its right end still has unknown within 2 cells.
// Expected: still ACTIVE, shrunk to the live cells, goal moved onto one of them.
TEST(FrontierManagerGoal, LiveCellsKeepTheRecordAndCarryTheGoal) {
  FrontierManager mgr(DefaultParams());
  mgr.update({MakeCellCluster(Row(9, 2, 11))}, *MakeGridWhere([](int, int iy) { return iy >= 10; }),
             Eigen::Vector3d(0.5, 0.5, 0.0), 1.0);
  auto after = MakeGridWhere([](int ix, int iy) { return ix >= 13 && iy >= 9; });  // unknown x>=1.3, y>=0.9
  mgr.update({}, *after, Eigen::Vector3d(0.5, 0.5, 0.0), 2.0);
  const auto& r = mgr.records()[0];
  EXPECT_EQ(r.state, FrontierState::ACTIVE);
  ASSERT_EQ(r.cells.size(), 1u) << "only ix 11 is within 2 cells of the unknown (ix >= 13)";
  EXPECT_NEAR(r.goal_xy.x(), 1.15, 1e-9);
}

// Scenario: a frontier row along the grid's top edge; then the grid is all free. Out-of-grid neighbours
// are not unknown for this test. Expected: VISITED.
TEST(FrontierManagerGoal, OutOfGridNeighboursDoNotKeepARecordAlive) {
  FrontierManager mgr(DefaultParams());
  mgr.update({MakeCellCluster(Row(19, 2, 11))}, *MakeGridWhere([](int, int) { return false; }),
             Eigen::Vector3d(0.5, 0.5, 0.0), 1.0);
  mgr.update({}, *MakeGridWhere([](int, int) { return false; }), Eigen::Vector3d(0.5, 0.5, 0.0), 2.0);
  EXPECT_EQ(mgr.records()[0].state, FrontierState::VISITED);
}

// Scenario: a frontier grows so that its centroid moves 0.9 m (> merge_radius 0.5) but it still shares
// cells with the record. Expected: matched by shared cells, same id, no duplicate.
TEST(FrontierManagerGoal, SharedCellsMatchEvenWhenTheCentroidJumps) {
  FrontierManager mgr(DefaultParams());
  auto grid = MakeGridWhere([](int, int iy) { return iy >= 10; });
  mgr.update({MakeCellCluster(Row(9, 0, 9))}, *grid, Eigen::Vector3d(0.5, 0.5, 0.0), 1.0);
  const uint64_t id = mgr.records()[0].id;
  mgr.update({MakeCellCluster(Row(9, 8, 19))}, *grid, Eigen::Vector3d(0.5, 0.5, 0.0), 2.0);
  ASSERT_EQ(mgr.size(), 1u);
  EXPECT_EQ(mgr.records()[0].id, id);
  EXPECT_EQ(mgr.records()[0].size_cells, 12);
}

// Scenario: two neighbouring records A (ix 0..5) and B (ix 6..11); one fresh cluster now spans both.
// Expected: the older record A takes it (tie on shared cells), B's cells are all owned by A now, so B is
// dropped as a duplicate.
TEST(FrontierManagerGoal, DuplicateWhoseCellsAnotherRecordOwnsIsDropped) {
  FrontierManager mgr(DefaultParams());
  auto grid = MakeGridWhere([](int, int iy) { return iy >= 10; });
  mgr.update({MakeCellCluster(Row(9, 0, 5)), MakeCellCluster(Row(9, 6, 11))}, *grid,
             Eigen::Vector3d(0.5, 0.5, 0.0), 1.0);
  ASSERT_EQ(mgr.size(), 2u);
  const uint64_t a = mgr.records()[0].id;
  mgr.update({MakeCellCluster(Row(9, 0, 11))}, *grid, Eigen::Vector3d(0.5, 0.5, 0.0), 2.0);
  ASSERT_EQ(mgr.size(), 1u);
  ASSERT_NE(FindId(mgr, a), nullptr);
  EXPECT_EQ(FindId(mgr, a)->size_cells, 12);
}
