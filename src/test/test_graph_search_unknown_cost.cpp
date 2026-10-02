// A* on the tri-state 2D planning map.
//  - Regression for graph_search.cpp (2026-09): the search loop used to re-point
//    cMap_ at the 3D voxel map, so walls that exist only in map_2d_ were invisible
//    to isOccupied() and every cell looked UNKNOWN. In 2D mode A* must read the 2D
//    map it was constructed with.
//  - UNKNOWN is traversable at w_unknown x geometric step (never free, never a
//    wall); a goal inside unknown is reachable; corner-cutting through OCCUPIED is
//    still rejected while unknown side cells do not block a diagonal.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hgp/graph_search.hpp"
#include "hgp/map_util.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

constexpr int N = 30;         // 3 x 3 m window at 0.1 m
constexpr double RES = 0.1;

struct World {
  std::shared_ptr<mighty::VoxelMapUtil> mu;
  std::vector<int8_t> src;  // tri-state source grid, same lattice as the window
  World() : src(N * N, 0) {
    mu = std::make_shared<mighty::VoxelMapUtil>(static_cast<float>(RES), -1000.f, 1000.f,
                                                -1000.f, 1000.f, -1.f, 2.f, 0.f, 0.f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
    mu->readMap(empty, empty, N, N, 1, Vec3f(0, 0, 0), 0.0, RES, 0.0, vec_Vecf<3>(),
                vec_Vecf<3>(), 0.0);
  }
  int8_t& at(int x, int y) { return src[y * N + x]; }
  void build() {
    const auto o = mu->getOrigin();
    auto grid = OccGrid2D::fromTristate(N, N, RES, o(0), o(1), src);
    mu->buildMap2DFromOcc2D(*grid, 0.5, 100.0);
  }
};

struct Result {
  bool reached = false;
  double cost = 0.0;
  std::vector<std::pair<int, int>> cells;  // start -> goal
};

Result Plan(World& w, int xs, int ys, int xg, int yg, double w_unknown, int max_expand = -1) {
  w.build();
  if (std::getenv("DUMP_MAP")) {
    for (int y = 23; y >= 7; --y) {
      std::string row;
      for (int x = 4; x <= 26; ++x) {
        const int8_t v = w.mu->get2DOccupancy(x, y);
        row += (v == 100 ? '#' : v == -1 ? '?' : '.');
      }
      std::printf("y=%2d %s\n", y, row.c_str());
    }
  }
  auto gs = std::make_shared<mighty::GraphSearch>(w.mu->get2DMapData(), w.mu, N, N, 1, 1.0,
                                                  false, "astar_heat", w_unknown, 0.0, 100.0, 0.0);
  gs->setStartAndGoal(w.mu->intToFloat(Veci<3>(xs, ys, 0)), w.mu->intToFloat(Veci<3>(xg, yg, 0)));
  double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
  gs->plan(xs, ys, 0, xg, yg, 0, 0.0, t1, t2, t3, t4, t5, 0.0, Vec3f(0, 0, 0), max_expand, 2000);
  Result r;
  r.reached = gs->reachedGoal();
  auto path = gs->getPath();  // goal -> start
  if (!path.empty()) {
    r.cost = path.front()->g;
    for (auto it = path.rbegin(); it != path.rend(); ++it) r.cells.emplace_back((*it)->x, (*it)->y);
  }
  return r;
}

bool Visits(const Result& r, int x, int y) {
  for (const auto& c : r.cells)
    if (c.first == x && c.second == y) return true;
  return false;
}

}  // namespace

TEST(GraphSearchUnknownCost, WallOnlyIn2DMapBlocks) {
  World w;
  for (int y = 0; y <= 24; ++y) w.at(15, y) = 100;  // wall with a gap at the top
  auto r = Plan(w, 5, 15, 25, 15, 2.0);
  ASSERT_TRUE(r.reached);
  for (int y = 0; y <= 24; ++y) EXPECT_FALSE(Visits(r, 15, y)) << "path crossed the 2D-only wall at y=" << y;
  EXPECT_GT(r.cost, 20.0 + 5.0) << "detour must cost more than the straight line";
}

TEST(GraphSearchUnknownCost, UnknownPricedNotBlocked) {
  World w;
  for (int y = 10; y <= 20; ++y)
    for (int x = 10; x <= 20; ++x) w.at(x, y) = -1;  // unknown band across the straight line
  auto cheap = Plan(w, 5, 15, 25, 15, 0.0);
  ASSERT_TRUE(cheap.reached);
  EXPECT_TRUE(Visits(cheap, 15, 15)) << "with w_unknown=0 the straight line through unknown wins";
  EXPECT_NEAR(cheap.cost, 20.0, 1e-6);

  auto priced = Plan(w, 5, 15, 25, 15, 2.0);
  ASSERT_TRUE(priced.reached);
  for (const auto& c : priced.cells)
    EXPECT_FALSE(c.first >= 10 && c.first <= 20 && c.second >= 10 && c.second <= 20)
        << "with w_unknown=2 the free detour must win, path entered unknown at " << c.first << "," << c.second;
  EXPECT_LT(priced.cost, 9.0 + 11.0 * 3.0) << "detour cheaper than the priced straight line";
}

TEST(GraphSearchUnknownCost, PenaltyScalesWithGeometricStep) {
  World w;
  for (auto& v : w.src) v = -1;  // everything unknown
  auto ortho = Plan(w, 5, 5, 5, 10, 2.0);
  auto diag = Plan(w, 5, 5, 10, 10, 2.0);
  ASSERT_TRUE(ortho.reached && diag.reached);
  EXPECT_NEAR(ortho.cost, 5.0 * 3.0, 1e-6);
  EXPECT_NEAR(diag.cost, 5.0 * std::sqrt(2.0) * 3.0, 1e-6);
}

TEST(GraphSearchUnknownCost, GoalInsideUnknownIsReachable) {
  World w;
  for (int y = 0; y < N; ++y)
    for (int x = 20; x < N; ++x) w.at(x, y) = -1;  // right third unobserved
  auto r = Plan(w, 5, 15, 27, 15, 2.0);
  EXPECT_TRUE(r.reached);
  EXPECT_TRUE(Visits(r, 27, 15));
}

TEST(GraphSearchUnknownCost, CornerCutBlockedByOccupiedNotByUnknown) {
  // Two offset walls whose only crossing is the diagonal (15,16)->(16,15), whose side
  // cells (15,15) and (16,16) are wall. Occupied walls: no path (fallback, goal not
  // reached). Unknown walls: the diagonal is allowed.
  for (int8_t wall : {int8_t(100), int8_t(-1)}) {
    World w;
    for (int y = 0; y <= 15; ++y) w.at(15, y) = wall;
    for (int y = 16; y < N; ++y) w.at(16, y) = wall;
    auto r = Plan(w, 5, 15, 25, 15, 2.0);
    if (wall == 100) {
      EXPECT_FALSE(r.reached) << "occupied corner cut must stay rejected";
    } else {
      EXPECT_TRUE(r.reached) << "unknown side cells must not reject the diagonal";
    }
  }
}

// Hard heat cutoff (heat_cutoff_ratio). World::build() uses d_safe 0.5 m and
// Hmax 100, so heat = 100 * (1 - d / 0.5) and ratio 0.3 blocks every cell closer
// than 0.35 m to an obstacle.
void EnableCutoff(World& w, float ratio) {
  w.mu->setStaticHeatEnabled(true);
  w.mu->setStaticHeatParams(1.0f, 1, 100.0f, 1.0f);
  w.mu->heat_cutoff_ratio_ = ratio;
}

TEST(GraphSearchHeatCutoff, NarrowGapIsAWall) {
  World w;
  for (int y = 0; y < N; ++y)
    if (y < 13 || y > 16) w.at(15, y) = 100;  // 0.4 m gap: every gap cell is < 0.35 m from a wall
  auto open = Plan(w, 5, 15, 25, 15, 2.0);
  ASSERT_TRUE(open.reached) << "without the cutoff the gap is passable";

  EnableCutoff(w, 0.3f);
  auto r = Plan(w, 5, 15, 25, 15, 2.0);
  EXPECT_FALSE(r.reached) << "a gap narrower than the cutoff must not be crossed";
}

TEST(GraphSearchHeatCutoff, StartInsideBandEscapes) {
  // Start 0.2 m from a wall: the start cell and all eight neighbours are above
  // the cutoff, so without the escape rule A* cannot leave the start cell.
  World w;
  for (int y = 0; y < N; ++y) w.at(8, y) = 100;
  EnableCutoff(w, 0.3f);
  auto r = Plan(w, 10, 15, 25, 15, 2.0);
  EXPECT_TRUE(r.reached) << "a start inside the cutoff band must be able to step out of it";
}

TEST(GraphSearchHeatCutoff, EscapeDoesNotOpenNarrowGaps) {
  // Same start inside the band, but the only way to the goal is a gap narrower
  // than the cutoff: escaping the start band must not let A* climb into it.
  World w;
  for (int y = 0; y < N; ++y) w.at(8, y) = 100;
  for (int y = 0; y < N; ++y)
    if (y < 13 || y > 16) w.at(20, y) = 100;
  EnableCutoff(w, 0.3f);
  auto r = Plan(w, 10, 15, 25, 15, 2.0);
  EXPECT_FALSE(r.reached) << "the escape rule must not reach the goal through a blocked gap";
}

TEST(GraphSearchHeatCutoff, BlockedGoalRequiresAnEscapeEndpoint) {
  World w;
  for (int y = 0; y < N; ++y) w.at(8, y) = 100;
  EnableCutoff(w, 0.3f);
  for (const int goal_y : {5, 25}) {
    auto r = Plan(w, 10, 5, 10, goal_y, 2.0);
    ASSERT_GT(w.mu->getHeat2D(10, goal_y), 30.0f);
    EXPECT_FALSE(r.reached);  // Includes start == goal inside the blocked band.
    ASSERT_GT(r.cells.size(), 1u) << "a partial escape is still available";
    EXPECT_LE(w.mu->getHeat2D(r.cells.back().first, r.cells.back().second), 30.0f);
  }
}

TEST(GraphSearchHeatCutoff, NoExitDoesNotReturnABlockedPartialPath) {
  World w;
  for (int y = 0; y < N; ++y) w.at(8, y) = w.at(13, y) = 100;
  EnableCutoff(w, 0.3f);  // The entire corridor lies inside the cutoff band.
  for (const int limit : {-1, 5}) {  // Exhausted queue and expansion-limit recovery.
    auto r = Plan(w, 10, 5, 10, 25, 2.0, limit);
    EXPECT_FALSE(r.reached);
    EXPECT_TRUE(r.cells.empty()) << "no usable path exists until escape completes";
  }
}
