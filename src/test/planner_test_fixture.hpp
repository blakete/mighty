// Shared fixture for the planner "New Plan" tests (test_planner_unknown_paths.cpp and
// test_planner_prefix_trim.cpp). Drives the REAL planner stack through its public API:
//   HGPManager::setParameters -> updateMap -> setupHGPPlanner -> solveHGP
// on a hand-built tri-state 2D grid (the same grid the mapper's occ_2d would deliver).
//
// Cell coordinates (cx, cy) are indices into the planning window; the window is N x N cells at
// `res` metres, and the source grid has exactly the same lattice, so source cell == window cell.
// Source values: 0 free, -1 unknown, 100 occupied.
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hgp/hgp_manager.hpp"
#include "mighty/mighty_type.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace plannertest {

/** Knobs that the tests vary; defaults mirror config/hw_mighty_ground_robot.yaml. */
struct Knobs {
  double w_unknown = 2.0;                ///< A* extra cost per unknown step (x geometric step)
  double unknown_inflation_2d_m = 0.5;   ///< prefix back-off from the start of an unknown run
  int trim_min_unknown_run_cells = 3;    ///< unknown run length that ends the observed space
  double inflation_2d_m = 0.0;           ///< occupied-band radius
  double max_dist_vertexes = 0.95;       ///< path waypoint spacing -> one waypoint every 10 cells
};

/** A 2D world plus a planner stack; call build() after editing `src`. */
class PlannerWorld {
 public:
  int n;
  double res;
  std::vector<int8_t> src;  ///< tri-state source grid, index = y * n + x

  explicit PlannerWorld(int n_cells = 100, double resolution = 0.1)
      : n(n_cells), res(resolution), src(static_cast<size_t>(n_cells) * n_cells, 0) {}

  int8_t& at(int x, int y) { return src[static_cast<size_t>(y) * n + x]; }

  /** Fill the full-height column band [x0, x1] with `value`. */
  void fillColumns(int x0, int x1, int8_t value) {
    for (int y = 0; y < n; ++y)
      for (int x = x0; x <= x1; ++x) at(x, y) = value;
  }

  /** Create the manager, feed it the grid and set up the planner. Safe to call once per world. */
  void build(const Knobs& k) {
    knobs_ = k;
    parameters par{};
    par.vehicle_type = "ground_robot";
    par.use_2d_planning = true;
    par.res = res;
    par.factor_hgp = 1.0;
    par.inflation_hgp = 0.0;
    par.x_min = -1000.0;
    par.x_max = 1000.0;
    par.y_min = -1000.0;
    par.y_max = 1000.0;
    par.z_min = -1.0;
    par.z_max = 2.0;
    par.obst_max_vel = 0.0;
    par.local_box_size = {1.0, 1.0, 0.5};
    par.max_dist_vertexes = k.max_dist_vertexes;
    par.default_goal_z = 0.0;
    par.static_heat_Hmax = 0.0f;  // no heat: path cost is pure geometry + unknown pricing
    par.w_unknown = k.w_unknown;
    par.trim_min_unknown_run_cells = k.trim_min_unknown_run_cells;
    par.unknown_inflation_2d_m = k.unknown_inflation_2d_m;
    par.inflation_2d_m = k.inflation_2d_m;
    par.hgp_stop_distance_m = 0.0;
    par.disable_all_smoothing = false;
    par.skip_path_smoothing = false;
    par.corridor_hop_enabled = false;

    mgr.setParameters(par);
    mgr.setEsdfGrid(nullptr, 0.0, 0.5);  // only sets d_safe for the heat ramp

    pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
    vec_Vecf<3> no_pos, no_bbox;
    const Vec3f centre(0, 0, 0);
    // First call only to learn the window origin; the source grid must share its lattice.
    mgr.updateMap(n * res, n * res, res, centre, empty, empty, no_pos, no_bbox, 0.0);
    mu = mgr.getMapUtilSharedPtr();
    const Vec3f o = mu->getOrigin();
    mgr.setOccGrid2D(OccGrid2D::fromTristate(n, n, res, o(0), o(1), src));
    mgr.updateMap(n * res, n * res, res, centre, empty, empty, no_pos, no_bbox, 0.0);

    mgr.setupHGPPlanner("astar_heat", false, res, 1.0, 1.0, 1.0, 10000, -1, k.w_unknown, 0.0,
                        100.0, 0.0);
    mgr.setMaxDistVertexes2D(k.max_dist_vertexes);
  }

  /** World position of a cell centre. */
  Vec3f world(int cx, int cy) const { return mu->intToFloat(Veci<3>(cx, cy, 0)); }

  /** Cell of a world position. */
  Veci<3> cell(const Vec3f& p) const { return mu->floatToInt(p); }

  struct Result {
    bool ok = false;            ///< solveHGP return value (false = planner FAILED)
    bool reached_goal = false;  ///< A* reached the exact goal (false with ok = PARTIAL)
    double final_g = 0.0;
    vec_Vecf<3> path;      ///< executable prefix (after the trim)
    vec_Vecf<3> raw_path;  ///< full global A* path, start -> goal
  };

  /** Run solveHGP from cell (sx, sy) to cell (gx, gy). */
  Result solve(int sx, int sy, int gx, int gy) {
    Result r;
    mgr.setupHGPPlanner("astar_heat", false, res, 1.0, 1.0, 1.0, 10000, -1, knobs_.w_unknown, 0.0,
                        100.0, 0.0);  // re-copies the (pristine) planning map
    mgr.setMaxDistVertexes2D(knobs_.max_dist_vertexes);
    r.ok = mgr.solveHGP(world(sx, sy), Vec3f(0, 0, 0), world(gx, gy), r.final_g, 1.0, 0.0, r.path,
                        r.raw_path);
    r.reached_goal = mgr.reachedGoal();
    return r;
  }

  HGPManager mgr;
  std::shared_ptr<mighty::VoxelMapUtil> mu;

 private:
  Knobs knobs_;
};

}  // namespace plannertest
