// Tests for FrontierDetector (Wavefront Frontier Detection).
//
// All tests build a synthetic OccGrid2D directly via fromTristate — no ROS, no
// rclcpp, no real planner.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "mighty/frontier_detector.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

constexpr int8_t U = -1;   // unknown
constexpr int8_t F = 0;    // free
constexpr int8_t O = 100;  // occupied

// Build a 10x10 OccGrid2D from a flat tristate buffer (row-major, iy=0 first).
std::shared_ptr<const OccGrid2D> MakeGrid(int w, int h,
                                          const std::vector<int8_t>& data,
                                          double res = 0.1) {
  return OccGrid2D::fromTristate(w, h, res, 0.0, 0.0, data);
}

// Detector with cluster_min_cells lowered for tiny test grids.
FrontierDetector MakeDetector(int min_cells = 2, int border_margin = 0) {
  FrontierDetectorParams p;
  p.cluster_min_cells   = min_cells;
  p.border_margin_cells = border_margin;
  p.robot_snap_radius_m = 5.0;  // generous snap so we never bail
  return FrontierDetector(p);
}

}  // namespace

TEST(FrontierDetectorTest, EmptyAllFreeGridYieldsNoFrontiers) {
  // 10x10, all free. No unknown anywhere -> no frontiers.
  const std::vector<int8_t> data(100, F);
  auto grid = MakeGrid(10, 10, data);
  auto det = MakeDetector();
  auto out = det.detect(*grid, Eigen::Vector2d(0.5, 0.5));
  EXPECT_EQ(out.size(), 0u);
}

TEST(FrontierDetectorTest, HalfFreeHalfUnknownYieldsOneCluster) {
  // 10x10. Left 5 columns FREE, right 5 UNKNOWN.
  // Frontier seam should run along ix=4 (the rightmost free column).
  std::vector<int8_t> data(100);
  for (int iy = 0; iy < 10; ++iy) {
    for (int ix = 0; ix < 10; ++ix) {
      data[iy * 10 + ix] = (ix < 5) ? F : U;
    }
  }
  auto grid = MakeGrid(10, 10, data);
  auto det = MakeDetector();
  auto out = det.detect(*grid, Eigen::Vector2d(0.15, 0.5));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].size_cells, 10);  // 10 cells along iy=0..9 at ix=4
  // Centroid x should be near (4 + 0.5) * 0.1 = 0.45m
  EXPECT_NEAR(out[0].centroid.x(), 0.45, 1e-6);
}

TEST(FrontierDetectorTest, TwoDisconnectedFrontiersYieldTwoClusters) {
  // 11x11. Layout:
  //   columns 0..4 FREE, column 5 OCCUPIED (wall), columns 6..10 FREE
  //   row 0..3 FREE everywhere (left+right halves), row 4 OCCUPIED wall,
  //   row 5..10 alternating to make UNKNOWN above the wall.
  // Simpler: build two free pockets each surrounded by unknown on one side.
  //
  // 11x5 grid:
  //   row 0:  F F F F F O F F F F F
  //   row 1:  F F F F F O F F F F F
  //   row 2:  U U U U F O F U U U U
  //   row 3:  U U U U F O F U U U U
  //   row 4:  U U U U U U U U U U U
  //
  // Two free pockets connected only via row 0/1 around the wall — but the
  // wall blocks horizontal travel. Make the wall shorter so the BFS can
  // reach both halves: wall only on row 2 onwards.
  //
  // Actually, for two truly DISCONNECTED frontier clusters reachable from a
  // single robot pose, we need both pockets connected to the robot through
  // free space. Use a U-shaped corridor.
  //
  // 11x5 with two unknown blobs flanking a free corridor:
  //   row 0: F F F F F F F F F F F     <- free corridor (robot here)
  //   row 1: F F F F F F F F F F F
  //   row 2: U U U F F F F F U U U     <- frontier seams at iy=2 around blobs
  //   row 3: U U U F F F F F U U U
  //   row 4: U U U F F F F F U U U
  // Two unknown blobs: (0..2, 2..4) and (8..10, 2..4). Each adjacent to free
  // cells along iy=1 and iy=2 boundary. The frontier seeds are the free cells
  // adjacent to unknown.
  const int W = 11, H = 5;
  std::vector<int8_t> data(W * H, F);
  auto setCell = [&](int ix, int iy, int8_t v) { data[iy * W + ix] = v; };
  for (int iy = 2; iy <= 4; ++iy) {
    for (int ix = 0; ix <= 2; ++ix)  setCell(ix, iy, U);
    for (int ix = 8; ix <= 10; ++ix) setCell(ix, iy, U);
  }
  auto grid = MakeGrid(W, H, data);
  auto det = MakeDetector(/*min_cells=*/2, /*border_margin=*/0);
  auto out = det.detect(*grid, Eigen::Vector2d(0.55, 0.05));
  EXPECT_EQ(out.size(), 2u);
}

TEST(FrontierDetectorTest, MinClusterSizeFiltersTinyClusters) {
  // 10x10. Most free, but a single-cell unknown island. The frontier (the
  // free cell adjacent to that unknown) is one cluster of 1 cell — should be
  // filtered if cluster_min_cells > 1.
  std::vector<int8_t> data(100, F);
  data[5 * 10 + 5] = U;   // single unknown cell at (5,5)
  // Surround it with free; the 8 neighbors are frontier cells (one cluster).
  auto grid = MakeGrid(10, 10, data);

  // With min_cells=2, the cluster of 8 should pass.
  {
    auto det = MakeDetector(/*min_cells=*/2);
    auto out = det.detect(*grid, Eigen::Vector2d(0.05, 0.05));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].size_cells, 8);
  }
  // With min_cells=20, the cluster should be filtered.
  {
    auto det = MakeDetector(/*min_cells=*/20);
    auto out = det.detect(*grid, Eigen::Vector2d(0.05, 0.05));
    EXPECT_EQ(out.size(), 0u);
  }
}

TEST(FrontierDetectorTest, BorderMarginExcludesEdgeFrontiers) {
  // 10x10. Inner 8x8 is free; outer ring is unknown (simulating fell-off
  // window). With border_margin=0, the inner-ring free cells along ix=1 or
  // iy=1 etc. become frontiers. With border_margin=2, all of those should
  // be excluded.
  std::vector<int8_t> data(100, U);
  for (int iy = 1; iy <= 8; ++iy)
    for (int ix = 1; ix <= 8; ++ix)
      data[iy * 10 + ix] = F;

  auto grid = MakeGrid(10, 10, data);

  // border_margin=0 -> seeds along inner ring -> non-empty.
  {
    FrontierDetectorParams p;
    p.cluster_min_cells   = 2;
    p.border_margin_cells = 0;
    p.robot_snap_radius_m = 5.0;
    FrontierDetector det(p);
    auto out = det.detect(*grid, Eigen::Vector2d(0.45, 0.45));
    EXPECT_GT(out.size(), 0u);
  }

  // border_margin=2 -> excludes the inner-ring; the only inner cells left are
  // (2..7, 2..7), none of which touch unknown (their neighbors are all free).
  // So 0 frontiers.
  {
    FrontierDetectorParams p;
    p.cluster_min_cells   = 2;
    p.border_margin_cells = 2;
    p.robot_snap_radius_m = 5.0;
    FrontierDetector det(p);
    auto out = det.detect(*grid, Eigen::Vector2d(0.45, 0.45));
    EXPECT_EQ(out.size(), 0u);
  }
}

TEST(FrontierDetectorTest, RobotOnOccupiedCellSnapsAndDetects) {
  // 10x10. Left 5 columns free, right 5 unknown, with the very center
  // occupied so the robot pose lands on it.
  std::vector<int8_t> data(100);
  for (int iy = 0; iy < 10; ++iy)
    for (int ix = 0; ix < 10; ++ix)
      data[iy * 10 + ix] = (ix < 5) ? F : U;
  data[5 * 10 + 2] = O;  // an obstacle in the free area near robot

  auto grid = MakeGrid(10, 10, data);
  auto det = MakeDetector();
  // Place robot directly on the occupied cell -> snap should find a free
  // neighbor and detection should still find the right-half frontier.
  auto out = det.detect(*grid, Eigen::Vector2d(0.25, 0.55));
  EXPECT_GT(out.size(), 0u);
}

TEST(FrontierDetectorTest, ObstacleClearanceFiltersWallHuggingFrontiers) {
  // 11x11 grid. Left half FREE, right half UNKNOWN. Plus a single OCCUPIED
  // cell at (5, 5) — right on the would-be frontier seam.
  //
  // With obstacle_clearance_cells=0, the frontier seam is the column ix=4
  // (10 cells along iy=0..9). Some of those cells have an occupied 8-neighbor.
  //
  // With obstacle_clearance_cells=1, every cell within 1 of (5,5) is
  // disqualified, so the frontier shrinks (or disappears for very small
  // sizes).
  const int W = 11, H = 11;
  std::vector<int8_t> data(W * H);
  for (int iy = 0; iy < H; ++iy)
    for (int ix = 0; ix < W; ++ix)
      data[iy * W + ix] = (ix < 5) ? F : U;
  data[5 * W + 5] = O;  // wall right on the seam at (5, 5)
  auto grid = MakeGrid(W, H, data);

  // No clearance -> the seam survives intact.
  {
    FrontierDetectorParams p;
    p.cluster_min_cells       = 2;
    p.border_margin_cells     = 0;
    p.obstacle_clearance_cells = 0;
    p.robot_snap_radius_m     = 5.0;
    FrontierDetector det(p);
    auto out = det.detect(*grid, Eigen::Vector2d(0.15, 0.55));
    ASSERT_EQ(out.size(), 1u);
    const int n_no_clearance = out[0].size_cells;
    EXPECT_GT(n_no_clearance, 0);
  }

  // Clearance=1 -> all seam cells within 1 of the wall are dropped.
  {
    FrontierDetectorParams p;
    p.cluster_min_cells       = 2;
    p.border_margin_cells     = 0;
    p.obstacle_clearance_cells = 1;
    p.robot_snap_radius_m     = 5.0;
    FrontierDetector det(p);
    auto out = det.detect(*grid, Eigen::Vector2d(0.15, 0.55));
    // The seam is 11 cells (iy=0..10 at ix=4); 3 of them (iy=4,5,6) are
    // within clearance=1 of the wall at (5,5) and get filtered. The
    // remainder is 8 cells, possibly split into 2 sub-clusters.
    int total_cells = 0;
    for (const auto& c : out) total_cells += c.size_cells;
    EXPECT_EQ(total_cells, 8);
    // No surviving cell may have an *in-bounds* occupied 8-neighbor.
    // (OOB doesn't count — the detector treats out-of-window as "outside",
    // not "occupied", consistent with the unknown-neighbor handling.)
    for (const auto& c : out) {
      for (const auto& cell : c.cells) {
        int ix, iy;
        grid->worldToGrid(cell.x(), cell.y(), ix, iy);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int nx = ix + dx;
            const int ny = iy + dy;
            if (!grid->inBounds(nx, ny)) continue;
            EXPECT_FALSE(grid->isOccupied(nx, ny))
                << "frontier cell (" << ix << "," << iy
                << ") has occupied neighbor at (" << nx << "," << ny << ")";
          }
        }
      }
    }
  }
}

TEST(FrontierDetectorTest, FreeIslandDisconnectedFromUnknownYieldsNothing) {
  // 11x11. Center 3x3 is a free island, surrounded by occupied wall, with
  // unknown outside the wall. Robot is inside the island. The BFS can reach
  // none of the unknown cells (wall blocks it), so 0 frontiers.
  const int W = 11, H = 11;
  std::vector<int8_t> data(W * H, U);
  // wall ring at offset 4..6
  for (int iy = 3; iy <= 7; ++iy) {
    for (int ix = 3; ix <= 7; ++ix) {
      if (iy == 3 || iy == 7 || ix == 3 || ix == 7)
        data[iy * W + ix] = O;
      else
        data[iy * W + ix] = F;
    }
  }
  auto grid = MakeGrid(W, H, data);
  auto det = MakeDetector();
  auto out = det.detect(*grid, Eigen::Vector2d(0.55, 0.55));
  EXPECT_EQ(out.size(), 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ---------------------------------------------------------------------------
// Goal cell and long-cluster splitting (max_cluster_extent_m)
// ---------------------------------------------------------------------------

namespace {

// 20x20 @0.1: free x < 1.2, y < 1.2, unknown to the east and north -> one L-shaped frontier
// (column ix = 11 and row iy = 11) whose centroid lies inside the free area.
std::shared_ptr<const OccGrid2D> MakeLGrid() {
  std::vector<int8_t> data(400);
  for (int iy = 0; iy < 20; ++iy)
    for (int ix = 0; ix < 20; ++ix) data[iy * 20 + ix] = (ix < 12 && iy < 12) ? F : U;
  return MakeGrid(20, 20, data);
}

// W x 4 @0.1 at origin (ox, 0): rows 0-1 free, rows 2-3 unknown -> one straight frontier along
// row 1, W cells long.
std::shared_ptr<const OccGrid2D> MakeLineGrid(int W, double ox) {
  std::vector<int8_t> data(static_cast<size_t>(W) * 4);
  for (int iy = 0; iy < 4; ++iy)
    for (int ix = 0; ix < W; ++ix) data[iy * W + ix] = (iy < 2) ? F : U;
  return OccGrid2D::fromTristate(W, 4, 0.1, ox, 0.0, data);
}

FrontierDetector MakeSplitDetector(double max_extent_m, int min_cells = 6) {
  FrontierDetectorParams p;
  p.cluster_min_cells = min_cells;
  p.border_margin_cells = 0;
  p.robot_snap_radius_m = 5.0;
  p.max_cluster_extent_m = max_extent_m;
  return FrontierDetector(p);
}

bool IsMember(const FrontierCluster& c, const Eigen::Vector2d& p) {
  for (const auto& q : c.cells)
    if ((q - p).norm() < 1e-9) return true;
  return false;
}

// The goal is a member cell and no member cell is closer to the centroid.
void ExpectGoalIsNearestMember(const FrontierCluster& c) {
  ASSERT_TRUE(c.goal.has_value());
  EXPECT_TRUE(IsMember(c, *c.goal));
  const double d = (*c.goal - c.centroid).norm();
  for (const auto& q : c.cells) EXPECT_GE((q - c.centroid).norm(), d - 1e-12);
}

}  // namespace

// Scenario: L-shaped frontier. Expected: the centroid is inside known free space (not a frontier cell),
// the goal is the member cell nearest it.
TEST(FrontierDetectorGoal, LShapedClusterGoalIsTheMemberCellNearestTheCentroid) {
  auto grid = MakeLGrid();
  auto out = MakeDetector().detect(*grid, Eigen::Vector2d(0.25, 0.25));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].size_cells, 23);
  EXPECT_FALSE(IsMember(out[0], out[0].centroid)) << "an L's centroid is off the frontier";
  ExpectGoalIsNearestMember(out[0]);
}

// Scenario: 6.4 m straight frontier, splitting off (0). Expected: one cluster, as before.
TEST(FrontierDetectorGoal, SplitOffKeepsOneCluster) {
  auto out = MakeSplitDetector(0.0).detect(*MakeLineGrid(64, 0.0), Eigen::Vector2d(0.5, 0.05));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].size_cells, 64);
  ExpectGoalIsNearestMember(out[0]);
}

// Scenario: the same 64-cell line (x 0.05 .. 6.35) with 3 m tiles. Tiles [0,3) and [3,6) hold 30 cells
// each, [6,9) only 4 (< cluster_min_cells 6), which merge into the adjacent [3,6) piece.
// Expected: pieces of 30 and 34 cells that partition the line, each with its own goal.
TEST(FrontierDetectorGoal, LongClusterSplitsIntoWorldTilesAndSliversMerge) {
  auto out = MakeSplitDetector(3.0).detect(*MakeLineGrid(64, 0.0), Eigen::Vector2d(0.5, 0.05));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].size_cells, 34);
  EXPECT_EQ(out[1].size_cells, 30);
  std::vector<Eigen::Vector2d> all;
  for (const auto& c : out) {
    ExpectGoalIsNearestMember(c);
    EXPECT_EQ(static_cast<int>(c.cells.size()), c.size_cells);
    all.insert(all.end(), c.cells.begin(), c.cells.end());
  }
  ASSERT_EQ(all.size(), 64u);
  for (size_t i = 0; i < all.size(); ++i)
    for (size_t j = i + 1; j < all.size(); ++j) EXPECT_GT((all[i] - all[j]).norm(), 1e-9);
  for (const auto& p : out[1].cells) EXPECT_LT(p.x(), 3.0) << "the 30-cell piece is tile [0, 3)";
}

// Scenario: the line starts at world x = 1.0 instead (origin shifted). Tiles are anchored in the world, so
// the cut points stay at x = 3 and 6: pieces of 20 (x 1..3), 30 (3..6) and 14 (6..7.4) cells, each inside
// one tile.
TEST(FrontierDetectorGoal, TileBoundariesAreWorldAnchored) {
  auto out = MakeSplitDetector(3.0).detect(*MakeLineGrid(64, 1.0), Eigen::Vector2d(1.5, 0.05));
  ASSERT_EQ(out.size(), 3u);
  std::vector<int> sizes;
  for (const auto& c : out) {
    sizes.push_back(c.size_cells);
    const double t = std::floor(c.cells.front().x() / 3.0);
    for (const auto& p : c.cells) EXPECT_EQ(std::floor(p.x() / 3.0), t);
  }
  std::sort(sizes.begin(), sizes.end());
  EXPECT_EQ(sizes, (std::vector<int>{14, 20, 30}));
}

// Scenario: a 2 m frontier that straddles the tile boundary x = 3. Expected: not split (only clusters
// longer than max_cluster_extent_m are cut).
TEST(FrontierDetectorGoal, ShortClusterAcrossATileBoundaryIsNotSplit) {
  auto out = MakeSplitDetector(3.0).detect(*MakeLineGrid(20, 2.0), Eigen::Vector2d(2.5, 0.05));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].size_cells, 20);
}
