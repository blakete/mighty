// Terminal-goal A* over a large (composite) occ_2d grid, through the REAL planner stack
// (HGPManager::updateMap -> setupHGPPlanner -> solveHGP), with the window MIGHTY builds for it
// (map2d::terminalGoalWindow) against today's robot-centred window and straight-line subgoal.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <string>
#include <memory>
#include <random>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hgp/hgp_manager.hpp"
#include "map2d/map2d.hpp"
#include "mighty/mighty_type.hpp"
#include "mighty/occ_grid_2d.hpp"

namespace {

/** A source grid of n x n cells at (ox, oy) (both multiples of res: the composite's lattice), and a
 *  planner window built from a centre + extent, like MIGHTY::updateMap2DOnly. */
class BigWorld {
 public:
  int n;
  double res, ox, oy;
  std::vector<int8_t> src;

  BigWorld(int n_cells, double resolution, double origin_x, double origin_y)
      : n(n_cells), res(resolution), ox(origin_x), oy(origin_y), src(static_cast<size_t>(n_cells) * n_cells, 0) {}

  int cx(double x) const { return static_cast<int>(std::floor((x - ox) / res + 1e-9)); }
  int cy(double y) const { return static_cast<int>(std::floor((y - oy) / res + 1e-9)); }
  void set(int x, int y, int8_t v) {
    if (x >= 0 && x < n && y >= 0 && y < n) src[static_cast<size_t>(y) * n + x] = v;
  }
  /** Occupy [x0, x1) x [y0, y1) (metres). */
  void block(double x0, double x1, double y0, double y1, int8_t v = 100) {
    for (int y = cy(y0); y < cy(y1); ++y)
      for (int x = cx(x0); x < cx(x1); ++x) set(x, y, v);
  }
  double x1() const { return ox + n * res; }
  double y1() const { return oy + n * res; }

  void build(double centre_x, double centre_y, double wdx, double wdy, bool heat = false) {
    parameters par{};
    par.vehicle_type = "ground_robot";
    par.use_2d_planning = true;
    par.res = res;
    par.factor_hgp = 1.0;
    par.inflation_hgp = 0.0;
    par.x_min = par.y_min = -1000.0;
    par.x_max = par.y_max = 1000.0;
    par.z_min = -1.0;
    par.z_max = 2.0;
    par.obst_max_vel = 0.0;
    par.local_box_size = {1.0, 1.0, 0.5};
    par.max_dist_vertexes = 0.5;
    par.default_goal_z = 0.0;
    par.static_heat_Hmax = heat ? 1.0f : 0.0f;  // rover: 1.0 with heat_weight 500
    par.heat_weight = heat ? 500.0f : 1.0f;
    par.w_unknown = 2.0;
    par.trim_min_unknown_run_cells = 3;
    par.unknown_inflation_2d_m = 0.5;
    par.inflation_2d_m = 0.0;
    par.hgp_stop_distance_m = 0.0;
    par.corridor_hop_enabled = false;
    mgr.setParameters(par);
    mgr.setEsdfGrid(nullptr, 0.0, heat ? 0.7 : 0.5);  // d_safe of the heat ramp
    mgr.setOccGrid2D(OccGrid2D::fromTristate(n, n, res, ox, oy, src));
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
    vec_Vecf<3> no_pos, no_bbox;
    mgr.updateMap(wdx, wdy, res, Vec3f(centre_x, centre_y, 0), empty, empty, no_pos, no_bbox, 0.0);
  }

  struct Result {
    bool ok = false, reached_goal = false;
    int status = 0;
    double ms = 0.0, ms_setup = 0.0;
    vec_Vecf<3> path, raw_path;
  };

  Result solve(double sx, double sy, double gx, double gy, double heuristic_weight = 1.0) {
    Result r;
    const auto t0 = std::chrono::steady_clock::now();
    mgr.setupHGPPlanner("astar_heat", false, res, 1.0, 1.0, 1.0, 10000, -1, 2.0, 0.0, 100.0, 0.0);
    mgr.setMaxDistVertexes2D(0.5);
    const auto t1 = std::chrono::steady_clock::now();
    double g = 0.0;
    r.ok = mgr.solveHGP(Vec3f(sx, sy, 0), Vec3f(0, 0, 0), Vec3f(gx, gy, 0), g, heuristic_weight, 0.0, r.path,
                        r.raw_path);
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    r.ms_setup = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.reached_goal = mgr.reachedGoal();
    r.status = mgr.plannerStatus();
    return r;
  }

  HGPManager mgr;
};

/** On a 30 m composite (300 x 300 at (-15, -15)): a 0.3 m wall across it at y = 2 m with one 1 m gap
 *  at x in [9, 10). */
void addWallWithFarGap(BigWorld& w) {
  w.block(-15.0, 9.0, 2.0, 2.3);
  w.block(10.0, 15.0, 2.0, 2.3);
}

}  // namespace

TEST(TerminalGoal, GoalBehindWallRoutedThroughFarGap) {
  // robot at the origin, goal 8 m straight ahead behind the wall; the only way round is 9 m off to
  // the side. The terminal-goal window holds the whole map, so the A* finds the gap and reaches the goal.
  BigWorld w(300, 0.1, -15.0, -15.0);
  addWallWithFarGap(w);
  const map2d::GoalWindow gw = map2d::terminalGoalWindow(0, 0, 0, 8, w.ox, w.oy, w.x1(), w.y1(), 3.0, 15, 15);
  w.build(gw.center_x, gw.center_y, gw.wdx, gw.wdy);
  const auto r = w.solve(0, 0, 0, 8);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.reached_goal);
  bool through_gap = false;
  for (const auto& p : r.raw_path) through_gap |= (p.x() >= 8.9 && p.x() <= 10.1 && p.y() > 1.5 && p.y() < 2.8);
  EXPECT_TRUE(through_gap) << "the raw A* path should pass the gap at x in [9, 10)";
}

TEST(TerminalGoal, TodaysRobotCentredWindowCannotSeeTheGap) {
  // the same world with today's window (min_wdx 15 around the robot) and the straight-line subgoal:
  // the gap is outside the window, so the A* never reaches the goal
  BigWorld w(300, 0.1, -15.0, -15.0);
  addWallWithFarGap(w);
  w.build(0, 0, 15, 15);
  const auto r = w.solve(0, 0, 0, 8);
  EXPECT_FALSE(r.ok && r.reached_goal);
}

TEST(TerminalGoal, OccupiedTerminalGoalReportsStatus2) {
  // the goal on the wall (e.g. a remembered obstacle): FAILED with status 2, which MIGHTY answers by
  // re-solving to the straight-line subgoal in the same tick
  BigWorld w(300, 0.1, -15.0, -15.0);
  addWallWithFarGap(w);
  const map2d::GoalWindow gw = map2d::terminalGoalWindow(0, 0, 0, 2.1, w.ox, w.oy, w.x1(), w.y1(), 3.0, 15, 15);
  w.build(gw.center_x, gw.center_y, gw.wdx, gw.wdy);
  const auto r = w.solve(0, 0, 0, 2.15);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.status, 2);
  const auto r2 = w.solve(0, 0, 0, 1.0);  // the fallback target is free
  EXPECT_TRUE(r2.ok);
  EXPECT_EQ(r2.status, 0);
}

TEST(TerminalGoal, RememberedFreeIsNotTrimmed) {
  // the accepted trade-off: the composite does not say which cells are seen now, so the executed
  // prefix runs through remembered free space up to the goal (only true unknown runs cut it)
  BigWorld w(300, 0.1, -15.0, -15.0);  // all known free
  const map2d::GoalWindow gw = map2d::terminalGoalWindow(-10, -10, 10, 10, w.ox, w.oy, w.x1(), w.y1(), 3.0, 15, 15);
  w.build(gw.center_x, gw.center_y, gw.wdx, gw.wdy);
  const auto r = w.solve(-10, -10, 10, 10);
  ASSERT_TRUE(r.ok);
  ASSERT_FALSE(r.path.empty());
  EXPECT_LT((r.path.back() - Vec3f(10, 10, 0)).head<2>().norm(), 0.3);
  // an unknown band across the way still ends the executed prefix before it
  BigWorld u(300, 0.1, -15.0, -15.0);
  u.block(-15.0, 15.0, 0.0, 1.0, -1);
  u.build(gw.center_x, gw.center_y, gw.wdx, gw.wdy);
  const auto ru = u.solve(-10, -10, 10, 10);
  ASSERT_TRUE(ru.ok);
  EXPECT_LT(ru.path.back().y(), 0.0);
}

TEST(TerminalGoal, LargeWindowTiming) {
  // rooms with doorways, rover heat settings; prints the A* time corner to corner (no assert on time)
  for (const int n : {400, 600}) {
    BigWorld w(n, 0.1, -0.05 * n, -0.05 * n);
    std::mt19937 rng(7);
    const double h = 0.05 * n;
    for (double c = -h + 5.0; c < h; c += 5.0) {
      w.block(c, c + 0.2, -h, h);  // walls every 5 m both ways
      w.block(-h, h, c, c + 0.2);
    }
    std::uniform_real_distribution<double> u(0.5, 4.0);
    for (double a = -h; a < h - 5.0; a += 5.0)
      for (double b = -h + 5.0; b < h; b += 5.0) {
        const double d = a + u(rng);
        w.block(d, d + 1.0, b, b + 0.2, 0);  // a doorway in every wall segment
        w.block(b, b + 0.2, d, d + 1.0, 0);
      }
    const double s = -h + 1.0, g = h - 1.0;
    const map2d::GoalWindow gw = map2d::terminalGoalWindow(s, s, g, g, w.ox, w.oy, w.x1(), w.y1(), 3.0, 15, 15);
    w.build(gw.center_x, gw.center_y, gw.wdx, gw.wdy, /*heat=*/true);
    // corner to corner (the worst case: across every room), and a goal ~10 m away (two rooms)
    for (const double hw : {1.0, 1.5}) {
      const auto r = w.solve(s, s, g, g, hw);
      const auto m = w.solve(s, s, s + 7.0, s + 7.0, hw);
      std::printf("[ timing ] %dx%d cells (%.0f m), window %.0f x %.0f m, heuristic %.1f: corner-to-corner "
                  "ok=%d reached=%d %.1f ms (setup %.1f) | 10 m goal ok=%d reached=%d %.1f ms (setup %.1f)\n",
                  n, n, 0.1 * n, gw.wdx, gw.wdy, hw, r.ok, r.reached_goal, r.ms, r.ms_setup, m.ok,
                  m.reached_goal, m.ms, m.ms_setup);
      EXPECT_TRUE(r.ok);
      EXPECT_TRUE(m.ok);
    }
  }
}

TEST(TerminalGoal, RealCompositeBench) {
  // Optional: MIGHTY_BENCH_DIR holds composites dumped from a bag (map_fusion's
  // tools/composite_replay.py --npz, converted to NAME.bin + NAME.txt "w h origin_x origin_y res robot_x
  // robot_y"). From the robot to the 20 farthest-spread known-free cells of each, terminal-goal window.
  const char* dir = std::getenv("MIGHTY_BENCH_DIR");
  if (dir == nullptr) GTEST_SKIP() << "MIGHTY_BENCH_DIR not set";
  DIR* d = opendir(dir);
  ASSERT_NE(d, nullptr);
  std::vector<std::string> names;
  while (dirent* e = readdir(d)) {
    std::string f = e->d_name;
    if (f.size() > 4 && f.substr(f.size() - 4) == ".txt") names.push_back(f.substr(0, f.size() - 4));
  }
  closedir(d);
  std::sort(names.begin(), names.end());
  std::vector<double> all_ms;
  int reached = 0, total = 0;
  for (const auto& name : names) {
    std::ifstream hdr(std::string(dir) + "/" + name + ".txt");
    int w = 0, h = 0;
    double ox, oy, res, rx, ry;
    hdr >> w >> h >> ox >> oy >> res >> rx >> ry;
    std::ifstream bin(std::string(dir) + "/" + name + ".bin", std::ios::binary);
    BigWorld world(1, res, ox, oy);  // n is reset below: the grid may be non-square
    std::vector<int8_t> data(static_cast<size_t>(w) * h);
    bin.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    // pad to a square n x n source (BigWorld is square), unknown outside
    const int n = std::max(w, h);
    world.n = n;
    world.src.assign(static_cast<size_t>(n) * n, -1);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) world.src[static_cast<size_t>(y) * n + x] = data[static_cast<size_t>(y) * w + x];
    // goals: known-free cells at least 5 m from the robot, spread over the map
    std::vector<std::pair<double, double>> goals;
    std::mt19937 rng(1);
    std::vector<int> free_cells;
    for (int i = 0; i < w * h; ++i)
      if (data[i] == 0) free_cells.push_back(i);
    std::shuffle(free_cells.begin(), free_cells.end(), rng);
    for (int i : free_cells) {
      const double gx = ox + (i % w + 0.5) * res, gy = oy + (i / w + 0.5) * res;
      if (std::hypot(gx - rx, gy - ry) >= 5.0) goals.emplace_back(gx, gy);
      if (goals.size() == 20) break;
    }
    std::vector<double> ms;
    for (const auto& [gx, gy] : goals) {
      const map2d::GoalWindow gw =
          map2d::terminalGoalWindow(rx, ry, gx, gy, ox, oy, ox + w * res, oy + h * res, 3.0, 15, 15);
      world.build(gw.center_x, gw.center_y, gw.wdx, gw.wdy, /*heat=*/true);
      const auto r = world.solve(rx, ry, gx, gy);
      ms.push_back(r.ms);
      all_ms.push_back(r.ms);
      reached += r.ok && r.reached_goal;
      ++total;
    }
    std::sort(ms.begin(), ms.end());
    if (!ms.empty())
      std::printf("[ bench  ] %s %dx%d: %zu goals, A* ms p50 %.1f max %.1f\n", name.c_str(), w, h, ms.size(),
                  ms[ms.size() / 2], ms.back());
  }
  std::sort(all_ms.begin(), all_ms.end());
  if (!all_ms.empty())
    std::printf("[ bench  ] all: %d/%d reached, A* ms p50 %.1f p95 %.1f max %.1f\n", reached, total,
                all_ms[all_ms.size() / 2], all_ms[static_cast<size_t>(0.95 * (all_ms.size() - 1))], all_ms.back());
}
