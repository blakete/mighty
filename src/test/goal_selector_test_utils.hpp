// Shared helpers for the goal_selector lifecycle / contract tests.
//
// Everything here drives the ROS-free core `goal_selector::GoalSelector` with a fake clock, hand
// drawn grids and scripted planner statuses. No ROS, no node, no real time.
//
// ----------------------------------------------------------------------------------------------
// The test world: a straight corridor (41 x 7 cells, 0.5 m per cell, origin (0, 0), so 20.5 m x 3.5 m)
//
//   Legend:  '#' occupied   '.' free   '?' unknown        (first text row = TOP = highest y)
//
//   Cell (cx, cy) has its centre at x = (cx + 0.5) * 0.5, y = (cy + 0.5) * 0.5.
//   Rows cy = 0 and cy = 6 are always walls. The corridor interior is rows 1..5 (y = 0.75 .. 2.75).
//
//   kOpenBoth (both ends open, two frontiers):
//        0         1         2         3         4
//        01234567890123456789012345678901234567890
//     y6 #########################################
//     y5 ???...................................???      <- rows 1..5 are all identical
//     ..
//     y1 ???...................................???
//     y0 #########################################
//
//   With unknown_inflation_2d_m = 0.5 (one cell) the first free column next to the unknown is the
//   "unknown band" (cx = 3 on the left, cx = 37 on the right). Frontier cells are the safe-free cells
//   next to the band: cx = 4 (x = 2.25) and cx = 36 (x = 18.25). Each frontier is a vertical cluster
//   of 5 cells, centroid y = 1.75. So:
//        LEFT  frontier centroid = ( 2.25, 1.75)
//        RIGHT frontier centroid = (18.25, 1.75)
//
//   The other maps are variants used by the return-home / keep-out tests (see each factory below).
//
// ----------------------------------------------------------------------------------------------
// Ranking is made trivial on purpose: BaseParams() zeroes every utility weight except distance and
// chooses w_dist = dist_ref so that
//        utility = - (distance robot -> frontier centroid)   [metres]
// i.e. the NEAREST frontier wins and preemption margins are plain metres.
// ----------------------------------------------------------------------------------------------
#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "goal_selector/goal_selector.hpp"

namespace gs_test {

using goal_selector::GoalCommand;
using goal_selector::GoalKind;
using goal_selector::GoalSelector;
using goal_selector::GridInput;
using goal_selector::LogMessage;
using goal_selector::Output;
using goal_selector::PlannerStatus;
using goal_selector::Pose;
using goal_selector::SelectorParams;

// Frontier centroids of the corridor maps (see the file comment).
constexpr double kLeftX = 2.25;
constexpr double kRightX = 18.25;
constexpr double kCorridorY = 1.75;
// Where the robot starts unless a test says otherwise: closer to the LEFT frontier (5.0 m) than to
// the RIGHT one (11.0 m).
constexpr double kStartX = 7.25;
constexpr double kStartY = 1.75;

/** @brief Build a GridInput from ASCII rows ('#' occupied, '.' free, '?' unknown).
 *  `rows` are given top first (highest y first), so the text looks like the map. All rows must have
 *  the same length. Occupied is encoded 100, free 0, unknown -1 (as nav_msgs/OccupancyGrid). */
inline GridInput GridFromAscii(const std::vector<std::string>& rows, double res = 0.5,
                               double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0) {
  GridInput g;
  g.height = static_cast<int>(rows.size());
  g.width = rows.empty() ? 0 : static_cast<int>(rows[0].size());
  g.resolution = res;
  g.origin_x = origin_x;
  g.origin_y = origin_y;
  g.origin_z = origin_z;
  g.data.assign(static_cast<size_t>(g.width) * g.height, -1);
  for (int r = 0; r < g.height; ++r) {
    EXPECT_EQ(static_cast<int>(rows[r].size()), g.width) << "ragged ASCII map, row " << r;
    const int cy = g.height - 1 - r;  // first text row is the top row
    for (int cx = 0; cx < g.width && cx < static_cast<int>(rows[r].size()); ++cx) {
      const char c = rows[r][cx];
      g.data[static_cast<size_t>(cx) + static_cast<size_t>(g.width) * cy] =
          c == '#' ? 100 : (c == '.' ? 0 : -1);
    }
  }
  return g;
}

/** @brief 41 x 7 corridor whose five interior rows all read `interior` (41 chars), walls above and
 *  below. */
inline GridInput Corridor(const std::string& interior, double origin_z = 0.0) {
  const std::string wall(interior.size(), '#');
  return GridFromAscii({wall, interior, interior, interior, interior, interior, wall}, 0.5, 0.0,
                       0.0, origin_z);
}

/** @brief Both ends unknown: frontiers LEFT (2.25, 1.75) and RIGHT (18.25, 1.75).
 *  interior = "???" + 35 x '.' + "???" */
inline GridInput OpenBoth(double origin_z = 0.0) {
  return Corridor("???" + std::string(35, '.') + "???", origin_z);
}

/** @brief Left end unknown (frontier LEFT only). The right end is closed by a wall plug at cx = 35 with
 *  never-observed unknown behind it (cx 36..40), so a later map can "open the door" there.
 *  interior = "???" + 32 x '.' + "#" + 5 x '?' */
inline GridInput LeftOnly() {
  return Corridor("???" + std::string(32, '.') + "#" + std::string(5, '?'));
}

/** @brief LeftOnly after the left end was explored: left is now a closed wall at cx = 0, no frontier
 *  anywhere. interior = "#" + 34 x '.' + "#" + 5 x '?' */
inline GridInput ExploredLeft() {
  return Corridor("#" + std::string(34, '.') + "#" + std::string(5, '?'));
}

/** @brief ExploredLeft with the right plug removed: new frontier RIGHT2 at cx = 34 (x = 17.25,
 *  y = 1.75). The unknown behind the plug was never observed, so the visited-map fusion cannot
 *  fill it. interior = "#" + 35 x '.' + 5 x '?' */
inline GridInput RightOpened() {
  return Corridor("#" + std::string(35, '.') + std::string(5, '?'));
}
constexpr double kRight2X = 17.25;

/** @brief Selector parameters used by all these tests (see the file comment for the ranking trick).
 *  Frontier detection works on a visited map that exactly overlays the 41 x 7 corridor grid, so the
 *  derived frontier map equals the drawn one. Pursuit timeout is armed as in hw_goal_selector.yaml
 *  (factor 3, v_ref 0.5, floor 10 s); preemption is OFF; the manual start timeout is 15 s. */
inline SelectorParams BaseParams() {
  SelectorParams p;
  p.robot_id = "RR01";
  p.expl_enabled = true;
  p.expl_default_goal_z = 0.0;
  // detector
  p.expl_cluster_min_cells = 3;
  p.expl_border_margin_cells = 0;
  p.expl_obstacle_clearance_cells = 0;
  p.expl_robot_snap_radius_m = 1.0;
  p.expl_bounds_enabled = false;
  // ranking: utility = -distance_m
  p.expl_w_size = 0.0;
  p.expl_w_dist = 25.0;
  p.expl_w_info = 0.0;
  p.expl_w_revisit = 0.0;
  p.expl_w_heading = 0.0;
  p.expl_dist_ref_m = 25.0;
  p.expl_goal_select_threshold = -1.0e9;
  // manager
  p.expl_merge_radius_m = 0.75;
  p.expl_centroid_ema_alpha = 0.5;
  p.expl_visit_radius_m = 0.3;
  p.expl_visit_dwell_sec = 1.0;
  p.expl_verify_radius_cells = 2;
  p.expl_unreachable_consec_thresh = 5;
  p.expl_pursuit_timeout_factor = 3.0;
  p.expl_pursuit_timeout_v_ref = 0.5;
  p.expl_pursuit_timeout_min_sec = 10.0;
  p.expl_invalidation_keep_out_radius_m = 1.5;
  p.expl_invalidation_cooldown_sec = 30.0;
  // preemption off by default
  p.expl_preempt_enabled = false;
  p.expl_preempt_margin = 2.0;
  p.expl_preempt_min_commit_sec = 2.0;
  // watchdogs
  p.expl_stuck_timeout_sec = 5.0;
  p.expl_stuck_move_thresh_m = 0.10;
  p.manual_start_timeout_sec = 15.0;
  // visited map == the corridor grid (origin (0,0), 20.5 x 3.5 m, 0.5 m cells)
  p.expl_visited_map_center_x = 10.25;
  p.expl_visited_map_center_y = 1.75;
  p.expl_visited_map_width_m = 20.5;
  p.expl_visited_map_height_m = 3.5;
  p.expl_visited_map_resolution_m = 0.5;
  p.expl_publish_visited_map = true;
  p.expl_fuse_persistent_into_local = true;
  p.expl_detect_on_visited_map = true;
  p.expl_use_minpos = false;
  p.expl_publish_markers = true;
  // shared
  p.goal_radius = 0.5;
  p.inflation_2d_m = 0.0;
  p.unknown_inflation_2d_m = 0.5;
  return p;
}

/** @brief Fake-clock driver around a GoalSelector. It owns the clock (`now`), the robot pose and the
 *  current map, forwards calls the way the ROS node does, and records every Output it saw.
 *
 *  Typical use:  Harness h(BaseParams()); h.setMap(OpenBoth()); h.setRobot(7.25, 1.75);
 *                h.cycle();   // state + occ grid + select tick at `now`  -> first frontier goal
 */
class Harness {
 public:
  explicit Harness(const SelectorParams& p, double start_time = 100.0) : sel(p), now(start_time) {}

  GoalSelector sel;
  double now;
  /// Every GoalCommand the selector ever published (term_goal), in order.
  std::vector<GoalCommand> goals;
  /// Every log line the selector ever emitted.
  std::vector<LogMessage> logs;
  /// The latest Output of each kind of call (for flag checks).
  Output last_out;

  void setMap(const GridInput& g) { map_ = g; }
  void setRobot(double x, double y, double yaw = 0.0) { pose_ = Pose{x, y, 0.0, yaw}; }
  const Pose& robot() const { return pose_; }
  void advance(double dt) { now += dt; }

  /** @brief onState with the current pose. */
  Output state() { return record(sel.onState(now, pose_)); }
  /** @brief onOccGrid with the current map (pass-by-value copy, like the node). */
  Output feed() { return record(sel.onOccGrid(now, map_)); }
  /** @brief onSelectTick. */
  Output tick() { return record(sel.onSelectTick(now)); }
  /** @brief The node's normal sequence at one instant: state, then occ grid, then select tick. */
  void cycle() {
    state();
    feed();
    tick();
  }
  /** @brief Manual goal at (x, y, z). */
  Output manual(double x, double y, double z = 0.0) { return record(sel.onManualGoal(now, x, y, z)); }
  Output returnHome() { return record(sel.onReturnHome(now)); }
  /** @brief Planner status carrying an explicit goal stamp. */
  Output status(PlannerStatus s, int64_t stamp) { return record(sel.onPlannerStatus(now, s, stamp)); }
  /** @brief Planner status for the selector's CURRENT commitment (what a healthy planner sends). */
  Output statusCur(PlannerStatus s) { return status(s, sel.currentStampNs()); }
  /** @brief Send the same current-commitment status `n` times. */
  void statusCur(PlannerStatus s, int n) {
    for (int i = 0; i < n; ++i) statusCur(s);
  }

  /** @brief Run `steps` cycles `dt` seconds apart; `before_step(i)` (optional) runs after the clock
   *  advanced and before the cycle, e.g. to move the robot. */
  void run(int steps, double dt, const std::function<void(int)>& before_step = nullptr) {
    for (int i = 0; i < steps; ++i) {
      advance(dt);
      if (before_step) before_step(i);
      cycle();
    }
  }

  /** @brief The frontier record the selector is currently pursuing (nullptr if none). */
  const FrontierRecord* currentRecord() const {
    return sel.frontierManager().find(sel.currentExploreId());
  }
  /** @brief The record whose centroid is closest to (x, y), or nullptr if there are none. */
  const FrontierRecord* recordNear(double x, double y) const {
    const FrontierRecord* best = nullptr;
    double best_d = 1e18;
    for (const auto& r : sel.frontierManager().records()) {
      const double d = std::hypot(r.centroid_xy.x() - x, r.centroid_xy.y() - y);
      if (d < best_d) {
        best_d = d;
        best = &r;
      }
    }
    return best;
  }
  /** @brief Number of records in `state`. */
  int countRecords(FrontierState state) const {
    int n = 0;
    for (const auto& r : sel.frontierManager().records()) n += (r.state == state);
    return n;
  }
  /** @brief True if any recorded log line has this level and contains `needle`. */
  bool hasLog(LogMessage::Level level, const std::string& needle) const {
    for (const auto& l : logs)
      if (l.level == level && l.text.find(needle) != std::string::npos) return true;
    return false;
  }
  /** @brief Count of recorded goals of a kind. */
  int goalsOfKind(GoalKind k) const {
    int n = 0;
    for (const auto& g : goals) n += (g.kind == k);
    return n;
  }

 private:
  Output record(Output o) {
    if (o.term_goal) goals.push_back(*o.term_goal);
    for (const auto& l : o.logs) logs.push_back(l);
    last_out = o;
    return o;
  }

  GridInput map_;
  Pose pose_;
};

/** @brief True if the goal's xy is within `tol` metres of (x, y). */
inline bool GoalNear(const GoalCommand& g, double x, double y, double tol = 0.3) {
  return std::hypot(g.position.x() - x, g.position.y() - y) <= tol;
}

}  // namespace gs_test
