// HGPManager's mapper-grid handoff: setOccGrid2D() / setEsdfGrid() are called
// from both the map and the replan thread while updateMap() rebuilds the 2D map,
// so the concurrency test below is the harness to run under ThreadSanitizer.
//
// The window is 100 x 100 cells at 0.1 m (as in test_map_2d_tristate.cpp) and
// mapper grids are built on lattices tied to its origin.

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hgp/hgp_manager.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

constexpr double RES = 0.1;
constexpr double WD = 10.0;  // window width [m] -> 100 cells

parameters GroundParams() {
  parameters p{};
  p.vehicle_type = "ground_robot";
  p.use_2d_planning = true;
  p.res = RES;
  p.factor_hgp = 1.0;
  p.x_min = -1000.0;
  p.x_max = 1000.0;
  p.y_min = -1000.0;
  p.y_max = 1000.0;
  p.z_min = -1.0;
  p.z_max = 2.0;
  p.local_box_size = {1.0, 1.0, 1.0};
  p.default_goal_z = 0.0;
  return p;
}

// 40 x 40 mapper grid covering planner cells [30, 70) with an occupied source
// cell at each given planner cell (the builder turns each into a 2x2 block
// ending at that cell, see Map2DInflation.DiskIsExactAndMarkedInflatedOnly).
std::shared_ptr<const OccGrid2D> Grid(const Vec3f& o, const std::vector<std::pair<int, int>>& occ) {
  const int W = 40, H = 40, OFF = 30;
  std::vector<int8_t> data(W * H, 0);
  for (const auto& c : occ) data[(c.second - OFF) * W + (c.first - OFF)] = 100;
  return OccGrid2D::fromTristate(W, H, RES, o(0) + OFF * RES, o(1) + OFF * RES, data);
}

// The hardware 2D-only pipeline: an empty cloud, the 2D map built from the grid.
void UpdateMap(HGPManager& mgr) {
  static const pcl::PointCloud<pcl::PointXYZ>::ConstPtr empty(
      new pcl::PointCloud<pcl::PointXYZ>());
  mgr.updateMap(WD, WD, RES, Vec3f(0, 0, 0), empty, empty, vec_Vecf<3>(), vec_Vecf<3>(), 0.0);
}

struct Ground2D {
  HGPManager mgr;
  Vec3f origin;

  Ground2D() {
    mgr.setParameters(GroundParams());
    mgr.setEsdfGrid(nullptr, 1.0, 0.5);  // d_safe for the heat ramp
    UpdateMap(mgr);                      // sets the window
    origin = mgr.getMapUtilSharedPtr()->getOrigin();
  }
  void Build(const std::vector<std::pair<int, int>>& occ) {
    mgr.setOccGrid2D(Grid(origin, occ));
    UpdateMap(mgr);
  }
  // World point at fractional cell coordinates of the window.
  Vec3f At(double fx, double fy) const {
    return Vec3f(origin(0) + fx * RES, origin(1) + fy * RES, 0.0);
  }
};

}  // namespace


TEST(HGPManager2D, GridSettersConcurrentWithMapUpdates) {
  Ground2D g;
  g.Build({{45, 45}});
  constexpr int N = 300;
  std::atomic<bool> stop{false};

  // map thread: MIGHTY::updateMap2DOnly -- a fresh grid each time, so the old
  // one is really freed if a reader still uses it
  std::thread map_thread([&] {
    for (int i = 0; i < N; ++i) {
      g.mgr.setOccGrid2D(Grid(g.origin, {{45, 45}}));
      UpdateMap(g.mgr);
    }
  });
  // replan thread: MIGHTY::generateGlobalPath -> setters + setupHGPPlanner
  std::thread replan_thread([&] {
    for (int i = 0; i < N; ++i) {
      g.mgr.setOccGrid2D(Grid(g.origin, {{45, 45}}));
      g.mgr.setEsdfGrid(nullptr, 1.0, 0.5);
      // geofence keep-out handoff (MIGHTY::updateMap2DOnly), on and off
      if (i % 2 == 0) {
        auto f = std::make_shared<goal_selector::GoalFence>();
        f->active = f->ready = true;
        f->offset_m = 0.3;
        f->keep_in = {{-100.0, -100.0}, {100.0, -100.0}, {100.0, 100.0}, {-100.0, 100.0}};
        g.mgr.setKeepOut2D(f, 0.3);
      } else {
        g.mgr.setKeepOut2D(nullptr, 0.0);
      }
      g.mgr.setupHGPPlanner("astar_heat", false, RES, 1.0, 1.0, 1.0, 100, 1000, 2.0, 0.0, 0.0,
                            0.0);
    }
  });
  // another thread polling the map-ready flag (atomic)
  std::thread goal_thread([&] {
    while (!stop.load()) (void)g.mgr.isMapInitialized();
  });

  map_thread.join();
  replan_thread.join();
  stop = true;
  goal_thread.join();

  EXPECT_TRUE(g.mgr.isMapInitialized());
  EXPECT_TRUE(g.mgr.getMapUtilSharedPtr()->is2DOccupiedInWindow(g.At(45.5, 45.5)));
}

