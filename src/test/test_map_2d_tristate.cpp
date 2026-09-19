// Tri-state 2D planning map: VoxelMapUtil::buildMap2DFromOcc2D must emit
// val_unknown_ for window cells the mapper does not cover and for mapper cells
// reported as -1, val_occ_ when any source cell is occupied, val_free_ when any
// strictly-overlapping source cell is known-free (precedence occupied > unknown >
// free), and the is2DOccupied / is2DUnknown helpers must treat out-of-bounds
// conservatively. Before 2026-09 everything not occupied was FREE, which let A*
// route goals beyond the mapper's coverage through unobserved space.
//
// Everything here is expressed in CELL indices on lattices tied to the window
// origin, so the test exercises the builder's sampling rules and not float
// arithmetic on world coordinates.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hgp/map_util.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

constexpr int DIM = 100;       // 10 x 10 m planner window at 0.1 m
constexpr double RES = 0.1;

// One z layer, centred on the origin, built from an EMPTY cloud -- the hardware
// 2D-only pipeline.
std::shared_ptr<mighty::VoxelMapUtil> MakeWindow() {
  auto mu = std::make_shared<mighty::VoxelMapUtil>(static_cast<float>(RES), -1000.f, 1000.f,
                                                   -1000.f, 1000.f, -1.f, 2.f, 0.f, 0.f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
  mu->readMap(empty, empty, DIM, DIM, 1, Vec3f(0, 0, 0), 0.0, RES, 0.0, vec_Vecf<3>(),
              vec_Vecf<3>(), 0.0);
  return mu;
}

}  // namespace

TEST(Map2DTristate, CoverageAndValues) {
  auto mu = MakeWindow();
  ASSERT_EQ(mu->getDim()(0), DIM);
  ASSERT_EQ(mu->getDim()(1), DIM);
  const auto o = mu->getOrigin();

  // Source grid: same 0.1 m lattice, 40 x 40 cells, covering planner cells [30, 70).
  const int W = 40, H = 40, OFF = 30;
  std::vector<int8_t> data(W * H, 0);
  auto at = [&](int ix, int iy) -> int8_t& { return data[iy * W + ix]; };
  at(15, 15) = 100;  // -> planner cell (45,45)
  for (int iy = 9; iy <= 11; ++iy)
    for (int ix = 9; ix <= 11; ++ix) at(ix, iy) = -1;  // 3x3 unknown -> planner (39..41, 39..41)
  auto grid = OccGrid2D::fromTristate(W, H, RES, o(0) + OFF * RES, o(1) + OFF * RES, data);
  mu->buildMap2DFromOcc2D(*grid, 0.5, 100.0);
  ASSERT_TRUE(mu->has2DMap());

  // covered, free
  EXPECT_EQ(mu->get2DOccupancy(50, 50), 0);
  EXPECT_FALSE(mu->is2DOccupied(50, 50));
  EXPECT_FALSE(mu->is2DUnknown(50, 50));
  // covered, occupied
  EXPECT_EQ(mu->get2DOccupancy(45, 45), 100);
  EXPECT_TRUE(mu->is2DOccupied(45, 45));
  EXPECT_FALSE(mu->is2DUnknown(45, 45));
  // covered, mapper says unknown (centre of the 3x3 block and its corners)
  for (int c : {39, 40, 41}) {
    EXPECT_EQ(mu->get2DOccupancy(c, c), -1) << "cell " << c;
    EXPECT_TRUE(mu->is2DUnknown(c, c));
    EXPECT_FALSE(mu->is2DOccupied(c, c));
  }
  // the cell just outside the block is free again (no low-edge leak)
  EXPECT_EQ(mu->get2DOccupancy(38, 40), 0);
  EXPECT_EQ(mu->get2DOccupancy(42, 40), 0);
  // inside the window, outside mapper coverage: UNKNOWN, never free
  for (int c : {5, 29, 70, 95}) {
    EXPECT_EQ(mu->get2DOccupancy(c, c), -1) << "no coverage at cell " << c;
    EXPECT_TRUE(mu->is2DUnknown(c, c));
    EXPECT_FALSE(mu->is2DOccupied(c, c));
  }
  // out of the window: both helpers answer conservatively
  EXPECT_TRUE(mu->is2DOccupied(-1, 0));
  EXPECT_TRUE(mu->is2DUnknown(-1, 0));
  EXPECT_TRUE(mu->is2DOccupied(0, DIM));
  EXPECT_EQ(mu->get2DOccupancy(0, DIM), 100);
}

TEST(Map2DTristate, PrecedenceOnMixedOverlap) {
  auto mu = MakeWindow();
  const auto o = mu->getOrigin();
  // Finer source grid at 0.05 m sharing the window origin: planner cell (x,y)
  // covers source cells (2x..2x+1, 2y..2y+1).
  const int W = 2 * DIM, H = 2 * DIM;
  std::vector<int8_t> data(W * H, 0);
  auto at = [&](int ix, int iy) -> int8_t& { return data[iy * W + ix]; };
  at(40, 40) = -1;                                      // planner (20,20): 1 unknown + 3 free -> free
  for (int iy = 50; iy <= 51; ++iy)
    for (int ix = 50; ix <= 51; ++ix) at(ix, iy) = -1;  // planner (25,25): all unknown -> unknown
  for (int iy = 60; iy <= 61; ++iy)
    for (int ix = 60; ix <= 61; ++ix) at(ix, iy) = -1;  // planner (30,30): 3 unknown + 1 occupied
  at(61, 61) = 100;                                     //   -> occupied
  auto grid = OccGrid2D::fromTristate(W, H, RES / 2.0, o(0), o(1), data);
  mu->buildMap2DFromOcc2D(*grid, 0.5, 100.0);

  EXPECT_EQ(mu->get2DOccupancy(20, 20), 0) << "any known-free source cell -> free";
  EXPECT_EQ(mu->get2DOccupancy(25, 25), -1) << "all-unknown -> unknown";
  EXPECT_EQ(mu->get2DOccupancy(24, 25), 0) << "neighbour of the unknown block stays free";
  EXPECT_EQ(mu->get2DOccupancy(30, 30), 100) << "any occupied -> occupied";
}
