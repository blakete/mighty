// Hard inflation of occupied cells in the 2D planning map (occ2d_inflation_m, 2026-09-29).
//  - VoxelMapUtil::inflateOccupied2D(r) blocks every cell whose centre is within r of a cell
//    that was OCCUPIED before the call (round footprint), overwriting FREE and UNKNOWN, and
//    records those cells in inflated_2d_. Dilation starts from the raw set only (no cascade).
//  - r <= 0 (or r below one cell) changes nothing.
//  - free2DCell(..., keep_inflated=true) never clears inflated cells; the copy constructor
//    (the HGP planning copy) keeps the mask.
//  - A* on the inflated map cannot pass a gap narrower than the band allows.
//
// Most cases write map_2d_ directly on a built window so the expected shapes do not depend
// on buildMap2DFromOcc2D's sampling rules (tested in test_map_2d_tristate.cpp).

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hgp/graph_search.hpp"
#include "hgp/map_util.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

constexpr int N = 30;  // 3 x 3 m window at 0.1 m
constexpr double RES = 0.1;

std::shared_ptr<mighty::VoxelMapUtil> MakeWindow(int8_t fill) {
  auto mu = std::make_shared<mighty::VoxelMapUtil>(static_cast<float>(RES), -1000.f, 1000.f,
                                                   -1000.f, 1000.f, -1.f, 2.f, 0.f, 0.f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
  mu->readMap(empty, empty, N, N, 1, Vec3f(0, 0, 0), 0.0, RES, 0.0, vec_Vecf<3>(), vec_Vecf<3>(),
              0.0);
  // Build a real 2D map so has_2d_map_ and the buffers are set up, then overwrite it.
  const auto o = mu->getOrigin();
  std::vector<int8_t> src(N * N, fill);
  auto grid = OccGrid2D::fromTristate(N, N, RES, o(0), o(1), src);
  mu->buildMap2DFromOcc2D(*grid, 0.5, 100.0);
  for (auto& v : mu->map_2d_) v = fill;
  return mu;
}

int8_t& Cell(const std::shared_ptr<mighty::VoxelMapUtil>& mu, int x, int y) {
  return mu->map_2d_[static_cast<size_t>(x) + static_cast<size_t>(N) * y];
}

int CountInflated(const std::shared_ptr<mighty::VoxelMapUtil>& mu) {
  int n = 0;
  for (int y = 0; y < N; ++y)
    for (int x = 0; x < N; ++x) n += mu->is2DInflated(x, y) ? 1 : 0;
  return n;
}

}  // namespace

TEST(Map2DInflation, ZeroAndSubCellRadiusChangeNothing) {
  for (double r : {0.0, -1.0, 0.05}) {
    auto mu = MakeWindow(0);
    Cell(mu, 15, 15) = 100;
    Cell(mu, 3, 3) = -1;
    const auto before = mu->map_2d_;
    mu->inflateOccupied2D(r);
    EXPECT_EQ(mu->map_2d_, before) << "r=" << r;
    EXPECT_EQ(CountInflated(mu), 0) << "r=" << r;
  }
}

TEST(Map2DInflation, RoundFootprint) {
  // r = 0.10 -> 4 neighbours, 0.15 -> full 3x3, 0.40 -> disc of radius 4 cells (49 cells incl. centre).
  const std::vector<std::pair<double, int>> cases = {{0.10, 4}, {0.15, 8}, {0.40, 48}};
  for (const auto& c : cases) {
    auto mu = MakeWindow(0);
    Cell(mu, 15, 15) = 100;
    mu->inflateOccupied2D(c.first);
    EXPECT_EQ(CountInflated(mu), c.second) << "r=" << c.first;
    EXPECT_FALSE(mu->is2DInflated(15, 15)) << "the raw cell is not an inflated cell";
    EXPECT_TRUE(mu->is2DOccupied(15, 15));
    for (int y = 0; y < N; ++y) {
      for (int x = 0; x < N; ++x) {
        const int d2 = (x - 15) * (x - 15) + (y - 15) * (y - 15);
        const double r_cells = c.first / RES;
        const bool within = d2 > 0 && d2 <= r_cells * r_cells + 1e-6;
        EXPECT_EQ(mu->is2DInflated(x, y), within) << "r=" << c.first << " cell " << x << "," << y;
        EXPECT_EQ(mu->is2DOccupied(x, y), within || d2 == 0) << "r=" << c.first << " cell " << x << "," << y;
      }
    }
  }
}

TEST(Map2DInflation, OverwritesUnknownOnlyWithinRadius) {
  auto mu = MakeWindow(-1);
  Cell(mu, 15, 15) = 100;
  mu->inflateOccupied2D(0.2);
  EXPECT_TRUE(mu->is2DOccupied(17, 15));
  EXPECT_TRUE(mu->is2DInflated(17, 15));
  EXPECT_TRUE(mu->is2DUnknown(18, 15)) << "unknown beyond r stays unknown";
  EXPECT_TRUE(mu->is2DUnknown(17, 17)) << "diagonal at 2.83 cells > 2: stays unknown";
}

TEST(Map2DInflation, NoCascade) {
  auto mu = MakeWindow(0);
  Cell(mu, 10, 10) = 100;
  mu->inflateOccupied2D(0.1);
  EXPECT_TRUE(mu->is2DOccupied(11, 10));
  EXPECT_FALSE(mu->is2DOccupied(12, 10)) << "inflated cells must not inflate further";
  // A second call re-derives from what is occupied now (raw + previous band): callers
  // (HGPManager::solveHGP) apply it exactly once per fresh planning copy.
}

TEST(Map2DInflation, CopyKeepsMaskAndFreeKeepsInflated) {
  auto mu = MakeWindow(0);
  Cell(mu, 15, 15) = 100;
  mu->inflateOccupied2D(0.15);

  mighty::VoxelMapUtil copy(*mu);
  EXPECT_TRUE(copy.is2DInflated(16, 16)) << "planning copy must keep the inflation mask";

  copy.free2DCell(15, 15, static_cast<float>(RES), /*keep_inflated=*/true);
  EXPECT_FALSE(copy.is2DOccupied(15, 15)) << "raw cell is freed";
  EXPECT_TRUE(copy.is2DOccupied(16, 15)) << "inflated cell stays blocked";
  EXPECT_TRUE(copy.is2DOccupied(14, 14));

  mighty::VoxelMapUtil legacy(*mu);
  legacy.free2DCell(15, 15, static_cast<float>(RES));  // default: previous behaviour
  EXPECT_FALSE(legacy.is2DOccupied(16, 15)) << "default free2DCell still clears everything";
}

namespace {

bool PlanOnCurrentMap(const std::shared_ptr<mighty::VoxelMapUtil>& mu, int xs, int ys, int xg,
                      int yg, std::vector<std::pair<int, int>>* cells) {
  auto gs = std::make_shared<mighty::GraphSearch>(mu->get2DMapData(), mu, N, N, 1, 1.0, false,
                                                  "astar_heat", 2.0, 0.0, 100.0, 0.0);
  gs->setStartAndGoal(mu->intToFloat(Veci<3>(xs, ys, 0)), mu->intToFloat(Veci<3>(xg, yg, 0)));
  double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
  gs->plan(xs, ys, 0, xg, yg, 0, 0.0, t1, t2, t3, t4, t5, 0.0, Vec3f(0, 0, 0), -1, 2000);
  if (cells) {
    for (const auto& s : gs->getPath()) cells->emplace_back(s->x, s->y);
  }
  return gs->reachedGoal();
}

// Wall at x = 15 across the window except a 3-cell gap (rows 13..15, 0.3 m).
std::shared_ptr<mighty::VoxelMapUtil> GapWorld() {
  auto mu = MakeWindow(0);
  for (int y = 0; y < N; ++y)
    if (y < 13 || y > 15) Cell(mu, 15, y) = 100;
  return mu;
}

}  // namespace

TEST(Map2DInflation, AStarRespectsTheBand) {
  {
    auto mu = GapWorld();
    EXPECT_TRUE(PlanOnCurrentMap(mu, 5, 14, 25, 14, nullptr)) << "no inflation: gap is open";
  }
  {
    auto mu = GapWorld();
    mu->inflateOccupied2D(0.1);  // gap rows 13 and 15 blocked, row 14 still open
    std::vector<std::pair<int, int>> cells;
    ASSERT_TRUE(PlanOnCurrentMap(mu, 5, 14, 25, 14, &cells));
    for (const auto& c : cells) {
      EXPECT_FALSE(mu->is2DOccupied(c.first, c.second)) << "path in blocked cell " << c.first << "," << c.second;
      if (c.first == 15) EXPECT_EQ(c.second, 14) << "only the middle of the gap is passable";
    }
  }
  {
    auto mu = GapWorld();
    mu->inflateOccupied2D(0.2);  // the whole 0.3 m gap is inside the band
    EXPECT_FALSE(PlanOnCurrentMap(mu, 5, 14, 25, 14, nullptr)) << "gap narrower than the band must close";
  }
}
