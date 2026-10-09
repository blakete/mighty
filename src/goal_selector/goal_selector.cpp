#include "goal_selector/goal_selector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace goal_selector {

namespace {

std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return std::string(buf);
}

// Same arithmetic as mighty_utils::projectPointToSphere (src/mighty/utils.cpp).
Eigen::Vector3d projectPointToSphere(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                                     double radius) {
  if ((P2 - P1).norm() <= radius) return P2;
  Eigen::Vector3d v = (P2 - P1).normalized();
  return P1 + v * radius;
}

}  // namespace

// ----------------------------------------------------------------------------

GoalSelector::GoalSelector(const SelectorParams& params) : params_(params) {
  FrontierDetectorParams dp;
  dp.cluster_min_cells = params_.expl_cluster_min_cells;
  dp.border_margin_cells = params_.expl_border_margin_cells;
  dp.obstacle_clearance_cells = params_.expl_obstacle_clearance_cells;
  dp.robot_snap_radius_m = params_.expl_robot_snap_radius_m;
  dp.bounds_enabled = params_.expl_bounds_enabled;
  dp.bounds_min_x = params_.expl_bounds_min_x;
  dp.bounds_max_x = params_.expl_bounds_max_x;
  dp.bounds_min_y = params_.expl_bounds_min_y;
  dp.bounds_max_y = params_.expl_bounds_max_y;
  frontier_detector_ = std::make_unique<FrontierDetector>(dp);

  FrontierManagerParams mp;
  mp.merge_radius_m = params_.expl_merge_radius_m;
  mp.centroid_ema_alpha = params_.expl_centroid_ema_alpha;
  mp.visit_radius_m = params_.expl_visit_radius_m;
  mp.visit_dwell_sec = params_.expl_visit_dwell_sec;
  mp.verify_radius_cells = params_.expl_verify_radius_cells;
  mp.max_frontiers = params_.expl_max_frontiers;
  mp.w_size = params_.expl_w_size;
  mp.w_dist = params_.expl_w_dist;
  mp.w_info = params_.expl_w_info;
  mp.w_revisit = params_.expl_w_revisit;
  mp.w_heading = params_.expl_w_heading;
  mp.size_ref_m2 = params_.expl_size_ref_m2;
  mp.dist_ref_m = params_.expl_dist_ref_m;
  mp.sensor_radius_m = params_.expl_sensor_radius_m;
  mp.goal_select_threshold = params_.expl_goal_select_threshold;
  mp.pursuit_timeout_factor = params_.expl_pursuit_timeout_factor;
  mp.pursuit_timeout_v_ref = params_.expl_pursuit_timeout_v_ref;
  mp.pursuit_timeout_min_sec = params_.expl_pursuit_timeout_min_sec;
  mp.invalidation_keep_out_radius_m = params_.expl_invalidation_keep_out_radius_m;
  mp.invalidation_cooldown_sec = params_.expl_invalidation_cooldown_sec;
  mp.peer_visit_radius_m = params_.expl_peer_visit_radius_m;
  frontier_manager_ = std::make_unique<FrontierManager>(mp);

  // Persistent visited bitmap: records every cell ever observed so the detector can suppress
  // re-detection along the sliding-window seam.
  visited_map_ = std::make_unique<VisitedMap>(
      params_.expl_visited_map_center_x, params_.expl_visited_map_center_y,
      params_.expl_visited_map_width_m, params_.expl_visited_map_height_m,
      params_.expl_visited_map_resolution_m);
}

void GoalSelector::log(Output& out, LogMessage::Level level, std::string text) const {
  out.logs.push_back({level, std::move(text)});
}

int64_t GoalSelector::nextStampNs(double now) {
  int64_t ns = static_cast<int64_t>(std::llround(now * 1.0e9));
  if (ns <= last_stamp_ns_) ns = last_stamp_ns_ + 1;  // strictly increasing, never 0
  last_stamp_ns_ = ns;
  return ns;
}

// ----------------------------------------------------------------------------
// State / peers
// ----------------------------------------------------------------------------

Output GoalSelector::onState(double now, const Pose& pose) {
  Output out;
  pose_ = pose;
  state_initialized_ = true;

  // MinPos: broadcast own position to peers (throttled). (stateCallback in mighty_node)
  if (params_.expl_enabled && params_.expl_use_minpos && params_.expl_peer_publish_rate_hz > 0.0) {
    const double period = 1.0 / params_.expl_peer_publish_rate_hz;
    if (now - last_peer_pose_publish_t_ >= period) {
      out.peer_pose = Eigen::Vector3d(pose.x, pose.y, pose.z);
      last_peer_pose_publish_t_ = now;
    }
  }
  return out;
}

void GoalSelector::onPeerPose(const std::string& peer_id, double x, double y, double t) {
  if (!params_.expl_enabled || !params_.expl_use_minpos) return;
  if (peer_id == params_.robot_id) return;  // filter out own messages
  peer_tracker_.updatePeer(peer_id, x, y, t);
}

void GoalSelector::onPeerVisitedMap(const std::string& sender_id, const GridInput& grid) {
  if (!params_.expl_enabled || !params_.expl_use_minpos) return;
  if (sender_id == params_.robot_id || !visited_map_) return;
  visited_map_->mergeFrom(grid.data.data(), grid.width, grid.height, grid.origin_x, grid.origin_y,
                          grid.resolution);
}

// ----------------------------------------------------------------------------
// Raw occ_2d: fuse, absorb, detect, update, publish flags, select  (occ2DCallback)
// ----------------------------------------------------------------------------

void GoalSelector::maybePublishVisitedMap(double now, Output& out) {
  // Publish the persistent occupancy map ~1 Hz (each publish copies the whole buffer).
  if (visited_map_ && params_.expl_publish_visited_map) {
    if (now - last_visited_publish_t_ >= 1.0) {
      out.publish_visited_map = true;
      last_visited_publish_t_ = now;
    }
  }
  // Broadcast our visited map to peers, same 1 Hz throttle.
  if (params_.expl_use_minpos && visited_map_ && !visited_map_->empty()) {
    if (now - last_peer_visited_publish_t_ >= 1.0) {
      out.broadcast_visited_map = true;
      last_peer_visited_publish_t_ = now;
    }
  }
}

Output GoalSelector::onOccGrid(double now, GridInput grid) {
  Output out;
  if (!params_.expl_enabled) return out;

  // Remember the mapper's ground-plane z so the visited map renders at the same height.
  occ2d_origin_z_ = grid.origin_z;

  // Persistent-map fusion: cells that arrive UNKNOWN but were previously observed are restored
  // from the visited map before the grid is consumed (static environments only; parameter-gated).
  if (visited_map_ && params_.expl_fuse_persistent_into_local && !grid.data.empty()) {
    const double res = grid.resolution;
    const double ox = grid.origin_x;
    const double oy = grid.origin_y;
    const unsigned W = static_cast<unsigned>(grid.width);
    const unsigned H = static_cast<unsigned>(grid.height);
    for (unsigned iy = 0; iy < H; ++iy) {
      for (unsigned ix = 0; ix < W; ++ix) {
        const size_t i = static_cast<size_t>(iy) * W + ix;
        if (i >= grid.data.size()) break;
        if (grid.data[i] >= 0) continue;  // already known (FREE or OCCUPIED)
        const double wx = ox + (ix + 0.5) * res;
        const double wy = oy + (iy + 0.5) * res;
        const int8_t v = visited_map_->getStateWorld(wx, wy);
        if (v != VisitedMap::kUnknown) grid.data[i] = v;  // restore old persistent value
      }
    }
  }

  occ_grid_2d_ = OccGrid2D::fromTristate(grid.width, grid.height, grid.resolution, grid.origin_x,
                                         grid.origin_y, grid.data);

  if (!(occ_grid_2d_ && frontier_detector_ && frontier_manager_ && state_initialized_)) return out;

  const Eigen::Vector3d robot_pose(pose_.x, pose_.y, pose_.yaw);
  const Eigen::Vector2d robot_xy(pose_.x, pose_.y);

  // Absorb the freshly observed cells into the visited bitmap *before* detection.
  if (visited_map_) visited_map_->absorb(*occ_grid_2d_);

  // Pick the grid the detector runs on (persistent map or the local sliding window).
  if (params_.expl_detect_on_visited_map && visited_map_ && !visited_map_->empty()) {
    current_detect_grid_ = OccGrid2D::fromTristate(
        visited_map_->width(), visited_map_->height(), visited_map_->resolution(),
        visited_map_->originX(), visited_map_->originY(), visited_map_->data());
  } else {
    current_detect_grid_ = occ_grid_2d_;
  }
  const auto& detect_grid = *current_detect_grid_;
  const VisitedMap* visited_filter =
      params_.expl_detect_on_visited_map ? nullptr : visited_map_.get();
  auto clusters = frontier_detector_->detect(detect_grid, robot_xy, visited_filter);

  // (The ESDF clearance filter is dropped, D3. Frontiers in the inflated band are invalidated in
  //  onPlanningOccGrid, D8b.)

  const auto active_peers = params_.expl_use_minpos
                                ? peer_tracker_.getActivePeers(now, params_.expl_peer_timeout_sec)
                                : std::vector<PeerPose>{};
  frontier_manager_->update(clusters, detect_grid, robot_pose, now, active_peers);

  maybePublishVisitedMap(now, out);

  // Throttled (2 s) diagnostic of the cell-state distribution.
  if (now - last_diag_log_t_ >= 2.0) {
    last_diag_log_t_ = now;
    int n_unknown = 0, n_free = 0, n_occ = 0;
    const auto& occ = occ_grid_2d_->occupiedData();
    const auto& unk = occ_grid_2d_->unknownData();
    for (size_t i = 0; i < occ.size(); ++i) {
      if (unk[i])
        ++n_unknown;
      else if (occ[i])
        ++n_occ;
      else
        ++n_free;
    }
    log(out, LogMessage::Level::kInfo,
        fmt("[expl] grid %dx%d  free=%d  occ=%d  unknown=%d  fresh=%zu  db=%zu", occ_grid_2d_->width(),
            occ_grid_2d_->height(), n_free, n_occ, n_unknown, clusters.size(),
            frontier_manager_->size()));
  }

  // Markers reflect the state before selection (as the planner did). The caller then drives the
  // selection immediately with onSelectTick(), so the robot starts moving as soon as the first
  // frontier exists (the planner called exploreSelectCallback() at this point).
  if (params_.expl_publish_markers) out.publish_markers = true;
  return out;
}

// ----------------------------------------------------------------------------
// Select tick (exploreSelectCallback)
// ----------------------------------------------------------------------------

Output GoalSelector::onSelectTick(double now) {
  Output out;
  selectAndCommit(now, out);
  return out;
}

void GoalSelector::selectAndCommit(double now, Output& out) {
  if (!params_.expl_enabled) return;
  if (manual_goal_active_) return;  // manual override: no selection / preemption / watchdog
  if (!occ_grid_2d_ || !frontier_manager_) return;
  if (!current_detect_grid_) return;
  if (!state_initialized_) return;

  // Never commit an in-band frontier (spec 4.4): invalidate those first, then select among the rest.
  invalidateFrontiersInBand(now, out);

  // If we still have an in-progress exploration goal that hasn't been marked VISITED/INVALIDATED
  // yet, either leave it alone (legacy hard-commit) or, with preemption enabled, keep re-ranking
  // every tick and switch when a different frontier beats the current one by preempt_margin.
  if (exploration_active_) {
    auto* r = frontier_manager_->find(current_explore_id_);
    if (r && (r->state == FrontierState::ACTIVE || r->state == FrontierState::DORMANT)) {
      // --- Static stuck watchdog (runs regardless of preemption) -----------------------------
      if (params_.expl_stuck_timeout_sec > 0.0) {
        const double t_stuck = now;
        const Eigen::Vector2d xy(pose_.x, pose_.y);
        if ((xy - explore_last_progress_xy_).norm() > params_.expl_stuck_move_thresh_m) {
          explore_last_progress_xy_ = xy;  // progressed -> reset the clock
          explore_last_progress_t_ = t_stuck;
          explore_has_moved_ = true;
        } else if (explore_has_moved_ &&
                   t_stuck - explore_last_progress_t_ >= params_.expl_stuck_timeout_sec) {
          log(out, LogMessage::Level::kWarn,
              fmt("Exploration: frontier %lu stuck (no motion > %.2f m for %.1f s) "
                  "-> invalidating, re-selecting",
                  static_cast<unsigned long>(current_explore_id_), params_.expl_stuck_move_thresh_m,
                  params_.expl_stuck_timeout_sec));
          frontier_manager_->markInvalidated(current_explore_id_, t_stuck);
          exploration_active_ = false;
          explore_last_progress_t_ = -1.0;
          explore_has_moved_ = false;
          // fall through to the normal selection path below (picks a new goal now)
        }
      }

      // Preemption / hold-goal logic only applies if the watchdog above didn't just release.
      if (exploration_active_) {
        if (!params_.expl_preempt_enabled) return;

        const double t_now = now;
        if (explore_committed_at_t_ >= 0.0 &&
            t_now - explore_committed_at_t_ < params_.expl_preempt_min_commit_sec) {
          return;  // inside the commit dwell — no switching yet
        }

        const Eigen::Vector3d pose_p(pose_.x, pose_.y, pose_.yaw);

        // Same selector the commit path below uses.
        std::optional<FrontierRecord> best;
        if (params_.expl_use_minpos) {
          auto peers = peer_tracker_.getActivePeers(t_now, params_.expl_peer_timeout_sec);
          best = frontier_manager_->selectNextGoalMinPos(
              pose_p, *current_detect_grid_, peers, params_.expl_min_frontier_dist_to_peers_m);
        } else {
          best = frontier_manager_->selectNextGoal(pose_p, *current_detect_grid_);
        }
        if (!best || best->id == current_explore_id_) return;

        const auto u_cur =
            frontier_manager_->utilityOf(current_explore_id_, pose_p, *current_detect_grid_);
        if (!u_cur) return;
        if (best->cached_utility < *u_cur + params_.expl_preempt_margin) return;

        // Preempt: release the old pursuit WITHOUT invalidating it (clear the armed deadline).
        log(out, LogMessage::Level::kInfo,
            fmt("Exploration: preempting frontier %lu (u=%.2f) for %lu (u=%.2f, margin %.2f)",
                static_cast<unsigned long>(current_explore_id_), *u_cur,
                static_cast<unsigned long>(best->id), best->cached_utility,
                params_.expl_preempt_margin));
        frontier_manager_->clearPursuit(current_explore_id_);
        exploration_active_ = false;
        // Fall through to the normal selection path.
      }
    }
    // Otherwise (record gone or already terminal) fall through and pick a new one.
  }

  const Eigen::Vector3d robot_pose(pose_.x, pose_.y, pose_.yaw);

  // --- Return-home re-arm ------------------------------------------------------------------
  // home_return_requested_ commits the agent to its return trip. Release the latch once the
  // robot is parked within goal_radius of the captured start, so a new session can begin if
  // frontiers remain. While still en route, bail out.
  if (home_return_requested_) {
    const double d_home =
        (Eigen::Vector2d(pose_.x, pose_.y) - exploration_start_pos_.head<2>()).norm();
    if (d_home > params_.goal_radius) return;  // still en route — stay latched
    home_return_requested_ = false;
    exploration_start_captured_ = false;  // next goal opens a fresh session
    log(out, LogMessage::Level::kInfo,
        fmt("Exploration: return-home complete at (%.2f, %.2f) — re-arming; "
            "will resume if frontiers reappear",
            pose_.x, pose_.y));
  }

  // Multi-agent timing guard: with peers heard, do not capture the start pose while the state
  // is still at the (0,0,0) default.
  if (!exploration_start_captured_ && params_.expl_use_minpos) {
    const auto peers = peer_tracker_.getActivePeers(now, params_.expl_peer_timeout_sec);
    if (!peers.empty()) {
      const double dist_to_origin = std::sqrt(pose_.x * pose_.x + pose_.y * pose_.y);
      if (dist_to_origin < 0.5) {
        // (The original logged this throttled at 2 s; here it is logged at most every 2 s.)
        if (now - last_defer_log_t_ >= 2.0) {
          last_defer_log_t_ = now;
          log(out, LogMessage::Level::kInfo,
              fmt("Exploration deferred: robot pose (%.3f, %.3f) within 0.5m of origin "
                  "with %zu peer(s) active — waiting for state to settle",
                  pose_.x, pose_.y, peers.size()));
        }
        return;
      }
    }
  }

  std::optional<FrontierRecord> next;
  if (params_.expl_use_minpos) {
    auto peers = peer_tracker_.getActivePeers(now, params_.expl_peer_timeout_sec);
    next = frontier_manager_->selectNextGoalMinPos(robot_pose, *current_detect_grid_, peers,
                                                   params_.expl_min_frontier_dist_to_peers_m);
  } else {
    next = frontier_manager_->selectNextGoal(robot_pose, *current_detect_grid_);
  }
  if (!next) {
    if (exploration_active_) {
      log(out, LogMessage::Level::kInfo,
          fmt("[mighty] No frontiers left. Robot now at (%.2f, %.2f, %.2f). Returning to captured "
              "start (%.2f, %.2f, %.2f)",
              pose_.x, pose_.y, pose_.z, exploration_start_pos_.x(), exploration_start_pos_.y(),
              exploration_start_pos_.z()));
      commit(GoalKind::kReturnHome,
             Eigen::Vector3d(exploration_start_pos_.x(), exploration_start_pos_.y(),
                             params_.expl_default_goal_z),
             now, out);
      // Lock in the return-home (same flag the /exploration/return_home topic uses).
      home_return_requested_ = true;
    }
    exploration_active_ = false;
    // exploration_start_captured_ is intentionally NOT reset here (see the planner original):
    // only a manual goal (true session boundary) resets it.
    return;
  }

  const Eigen::Vector3d g(next->centroid_xy.x(), next->centroid_xy.y(), params_.expl_default_goal_z);

  log(out, LogMessage::Level::kInfo,
      fmt("Exploration: -> frontier %lu at (%.2f, %.2f), state=%d, u=%.3f",
          static_cast<unsigned long>(next->id), next->centroid_xy.x(), next->centroid_xy.y(),
          static_cast<int>(next->state), next->cached_utility));

  commit(GoalKind::kFrontier, g, now, out);

  if (!exploration_start_captured_) {
    exploration_start_pos_ = Eigen::Vector3d(pose_.x, pose_.y, pose_.z);
    exploration_start_captured_ = true;
    log(out, LogMessage::Level::kInfo,
        fmt("Exploration: starting from (%.2f, %.2f, %.2f) — will return here when done",
            exploration_start_pos_.x(), exploration_start_pos_.y(), exploration_start_pos_.z()));
  }

  current_explore_id_ = next->id;
  exploration_active_ = true;
  explore_committed_at_t_ = now;
  unreachable_consec_count_ = 0;
  // Arm the stuck watchdog fresh for this pursuit.
  explore_last_progress_xy_ = Eigen::Vector2d(robot_pose.x(), robot_pose.y());
  explore_last_progress_t_ = now;
  explore_has_moved_ = false;
  frontier_manager_->markSelected(next->id, Eigen::Vector2d(robot_pose.x(), robot_pose.y()), now);
  out.exploration_current_goal = g;
}

// ----------------------------------------------------------------------------
// Commit / manual / return-home
// ----------------------------------------------------------------------------

void GoalSelector::commit(GoalKind kind, const Eigen::Vector3d& target, double now, Output& out) {
  current_ = Commitment{};
  current_.kind = kind;
  current_.stamp_ns = nextStampNs(now);
  current_.target = target;
  current_.published = target;

  // (commit never touches manual_goal_active_: callers set it, as terminalGoalCallbackImpl did.)
  if (kind == GoalKind::kManual) manual_unreachable_ = false;

  bool relocated = false;
  if (params_.relocate_occupied_goal) {
    // All goal kinds are relocated (frontier, manual, return-home), switched by
    // relocate_occupied_goal. Rebuild first so the window is centred for this target.
    rebuildRelocationMap();
    const RelocResult r = relocate(target);
    if (!r.ok) {
      // D10: relocation failed -> publish the unrelocated goal; the planner's consecutive
      // failures handle it.
      log(out, LogMessage::Level::kError,
          fmt("Goal at (%.2f,%.2f,%.2f) is in occupied space and could not be relocated; "
              "publishing it unrelocated.",
              target.x(), target.y(), target.z()));
    } else if ((r.position - target).norm() > 1e-6) {
      current_.published = r.position;
      relocated = true;
      log(out, LogMessage::Level::kInfo,
          fmt("Goal relocated (2D map) from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)", target.x(),
              target.y(), target.z(), r.position.x(), r.position.y(), r.position.z()));
    }
  }

  GoalCommand cmd;
  cmd.position = current_.published;
  cmd.stamp_ns = current_.stamp_ns;
  cmd.kind = kind;
  cmd.relocated = relocated;
  out.term_goal = cmd;
}

Output GoalSelector::onManualGoal(double now, double x, double y, double z) {
  Output out;
  // The manual goal preempts exploration (terminalGoalCallbackImpl(.., from_user=true)): the
  // active frontier is released without invalidating it (pursuit deadline cleared), the exploration session resets, and an
  // operator override of a return-home starts a fresh session.
  if (exploration_active_ && frontier_manager_) {
    frontier_manager_->clearPursuit(current_explore_id_);  // release without invalidating (§11.10)
  }
  exploration_active_ = false;
  exploration_start_captured_ = false;
  unreachable_consec_count_ = 0;
  home_return_requested_ = false;

  commit(GoalKind::kManual, Eigen::Vector3d(x, y, z), now, out);
  manual_goal_active_ = true;
  log(out, LogMessage::Level::kInfo,
      fmt("Manual goal (%.2f, %.2f, %.2f) committed (stamp %lld)", x, y, z,
          static_cast<long long>(current_.stamp_ns)));
  return out;
}

Output GoalSelector::onReturnHome(double now) {
  Output out;
  if (home_return_requested_) {
    log(out, LogMessage::Level::kInfo,
        "Return-home trigger received, but already returning home");
    return out;
  }
  if (!exploration_start_captured_) {
    log(out, LogMessage::Level::kWarn,
        "Return-home trigger received before exploration started — no start pose captured, "
        "ignoring");
    return out;
  }
  home_return_requested_ = true;
  commit(GoalKind::kReturnHome,
         Eigen::Vector3d(exploration_start_pos_.x(), exploration_start_pos_.y(),
                         params_.expl_default_goal_z),
         now, out);
  exploration_active_ = false;
  log(out, LogMessage::Level::kInfo,
      fmt("Return-home: heading to (%.2f, %.2f, %.2f)", exploration_start_pos_.x(),
          exploration_start_pos_.y(), params_.expl_default_goal_z));
  return out;
}

// ----------------------------------------------------------------------------
// Planner status (spec 3, D8)
// ----------------------------------------------------------------------------

Output GoalSelector::onPlannerStatus(double now, PlannerStatus status, int64_t goal_stamp_ns) {
  Output out;
  // Ignore statuses for other commitments, and those for no goal at all (stamp 0).
  if (goal_stamp_ns == 0 || current_.kind == GoalKind::kNone || goal_stamp_ns != current_.stamp_ns)
    return out;

  const bool frontier_pursuit =
      params_.expl_enabled && current_.kind == GoalKind::kFrontier && exploration_active_ &&
      frontier_manager_;
  const bool manual_pursuit = current_.kind == GoalKind::kManual && manual_goal_active_;

  switch (status) {
    case PlannerStatus::kFailed:
      if (frontier_pursuit) {
        // Frontier-unreachable detection: consecutive global-plan failures invalidate it.
        if (++unreachable_consec_count_ >= params_.expl_unreachable_consec_thresh) {
          log(out, LogMessage::Level::kWarn,
              fmt("Exploration: frontier %lu unreachable after %d HGP failures, invalidating",
                  static_cast<unsigned long>(current_explore_id_), unreachable_consec_count_));
          frontier_manager_->markInvalidated(current_explore_id_, now);
          exploration_active_ = false;
          unreachable_consec_count_ = 0;
        }
      } else if (manual_pursuit) {
        // Manual goals are never released or resent on failure: only warn + flag (D8a).
        ++unreachable_consec_count_;
        if (unreachable_consec_count_ >= params_.expl_unreachable_consec_thresh &&
            !manual_unreachable_) {
          manual_unreachable_ = true;
          log(out, LogMessage::Level::kWarn,
              fmt("Manual goal (stamp %lld) unreachable after %d consecutive planner failures; "
                  "keeping it active — send a new goal to override",
                  static_cast<long long>(current_.stamp_ns), unreachable_consec_count_));
        }
      }
      break;

    case PlannerStatus::kSuccess:
    case PlannerStatus::kPartial:
      if (frontier_pursuit || manual_pursuit) unreachable_consec_count_ = 0;
      if (manual_pursuit) manual_unreachable_ = false;
      break;

    case PlannerStatus::kReached:
      // If the robot reached an exploration goal, mark it visited and clear the active flag so
      // the next select tick can pick the next frontier.
      if (frontier_pursuit) {
        frontier_manager_->markVisited(current_explore_id_);
        exploration_active_ = false;
        unreachable_consec_count_ = 0;
      }
      // A manual goal that just completed releases the override.
      if (manual_pursuit) {
        manual_goal_active_ = false;
        manual_unreachable_ = false;
        unreachable_consec_count_ = 0;
      }
      break;

    case PlannerStatus::kSkipped:
      break;  // neutral (D8): neither increments nor resets
  }
  return out;
}

// ----------------------------------------------------------------------------
// Relocation (spec 4.4)
// ----------------------------------------------------------------------------

Output GoalSelector::onPlanningOccGrid(double now, const GridInput& grid) {
  Output out;
  PlanningSource& s = planning_src_;
  s.width = grid.width;
  s.height = grid.height;
  s.resolution = grid.resolution;
  s.inv_resolution = 1.0 / grid.resolution;  // as OccGrid2D::fromOccupancyGrid
  s.origin_x = grid.origin_x;
  s.origin_y = grid.origin_y;
  const size_t n = static_cast<size_t>(std::max(0, grid.width)) * std::max(0, grid.height);
  s.occupied.assign(n, false);
  s.unknown.assign(n, true);  // cells missing from a short message count as unknown
  for (size_t i = 0; i < n && i < grid.data.size(); ++i) {
    s.occupied[i] = (grid.data[i] >= 100);
    s.unknown[i] = (grid.data[i] < 0);
  }
  s.valid = grid.width > 0 && grid.height > 0 && grid.resolution > 0.0;

  if (!state_initialized_ || !s.valid) return out;

  rebuildRelocationMap();
  invalidateFrontiersInBand(now, out);
  rerelocateCurrent(out);
  return out;
}

map2d::WindowGeometry GoalSelector::planningWindow(double& map_res) const {
  // Planner window geometry (MIGHTY::computeMapSize / HGPManager::updateMap / MapUtil::readMap).
  const Eigen::Vector3d pos(pose_.x, pose_.y, pose_.z);
  Eigen::Vector3d g_term = pos;  // no commitment: window is the minimum one
  if (current_.kind != GoalKind::kNone) {
    g_term = current_.published;  // the planner's G_term is the relocated goal
    if (params_.force_goal_z) g_term.z() = params_.default_goal_z;  // planner forces the goal z
  }
  const Eigen::Vector3d G = projectPointToSphere(pos, g_term, params_.horizon);

  const double dynamic_buffer = params_.map_buffer;
  const double dist_x = std::abs(pos[0] - G[0]);
  const double dist_y = std::abs(pos[1] - G[1]);
  const double wdx = std::max(dist_x + 2 * dynamic_buffer, params_.min_wdx);
  const double wdy = std::max(dist_y + 2 * dynamic_buffer, params_.min_wdy);

  // HGPManager uses par.res for the cell count; the MapUtil is constructed with
  // float(factor_hgp * res) and uses that as its resolution for everything else.
  const int cells_x = map2d::windowCells(wdx, params_.res);
  const int cells_y = map2d::windowCells(wdy, params_.res);
  map_res = static_cast<double>(static_cast<float>(params_.factor_hgp * params_.res));
  return map2d::windowGeometry(pos.x(), pos.y(), cells_x, cells_y, map_res, params_.inflation_hgp);
}

void GoalSelector::rebuildRelocationMap() {
  if (!planning_src_.valid || !state_initialized_) return;
  double map_res = 0.0;
  const map2d::WindowGeometry win = planningWindow(map_res);

  map2d::SourceGrid src;
  src.width = planning_src_.width;
  src.height = planning_src_.height;
  src.resolution = planning_src_.resolution;
  src.inv_resolution = planning_src_.inv_resolution;
  src.origin_x = planning_src_.origin_x;
  src.origin_y = planning_src_.origin_y;
  src.occupied = &planning_src_.occupied;
  src.unknown = &planning_src_.unknown;

  map2d::Grid2D g;
  g.dim_x = win.dim_x;
  g.dim_y = win.dim_y;
  g.res = map_res;
  g.origin_x = win.origin_x;
  g.origin_y = win.origin_y;
  map2d::buildFromOccupancy(src, g.dim_x, g.dim_y, g.res, g.origin_x, g.origin_y, g.values);
  // MapUtil::setInflation2D stores max(0, float(inflation_2d_m)).
  map2d::inflate(g, std::max(0.0f, static_cast<float>(params_.inflation_2d_m)));
  reloc_map_ = std::move(g);
}

GoalSelector::RelocResult GoalSelector::relocate(const Eigen::Vector3d& goal) const {
  RelocResult r;
  r.position = goal;
  if (!params_.relocate_occupied_goal) return r;
  if (reloc_map_.values.empty()) return r;  // no map yet: trust the goal (as the planner did)

  const double step = params_.res > 0.0 ? params_.res : 0.1;
  const map2d::SanitizeResult s =
      map2d::sanitizeGoal2D(reloc_map_.view(), goal, params_.goal_relocation_clearance_m, step);
  switch (s.outcome) {
    case map2d::GoalOutcome::kUnchanged:
      break;
    case map2d::GoalOutcome::kRelocated:
      r.position = s.goal;
      break;
    case map2d::GoalOutcome::kDropped:
      r.ok = false;
      break;
  }
  return r;
}

void GoalSelector::ensureBandMap() {
  if (!params_.expl_enabled || !occ_grid_2d_ || !state_initialized_) return;

  // Rebuild only when the raw grid or the window changed (the window follows the robot / goal).
  double map_res = 0.0;
  const map2d::WindowGeometry win = planningWindow(map_res);
  if (!band_map_.values.empty() && band_src_ == occ_grid_2d_.get() && band_map_.dim_x == win.dim_x &&
      band_map_.dim_y == win.dim_y && band_map_.origin_x == win.origin_x &&
      band_map_.origin_y == win.origin_y && band_map_.res == map_res)
    return;

  // Real obstacles only: occupied cells of occ_2d (unknown and free are not obstacles).
  map2d::SourceGrid src;
  src.width = occ_grid_2d_->width();
  src.height = occ_grid_2d_->height();
  src.resolution = occ_grid_2d_->resolution();
  src.inv_resolution = occ_grid_2d_->invResolution();
  src.origin_x = occ_grid_2d_->originX();
  src.origin_y = occ_grid_2d_->originY();
  src.occupied = &occ_grid_2d_->occupiedData();
  src.unknown = &occ_grid_2d_->unknownData();

  map2d::Grid2D g;
  g.dim_x = win.dim_x;
  g.dim_y = win.dim_y;
  g.res = map_res;
  g.origin_x = win.origin_x;
  g.origin_y = win.origin_y;
  map2d::buildFromOccupancy(src, g.dim_x, g.dim_y, g.res, g.origin_x, g.origin_y, g.values);
  map2d::inflate(g, std::max(0.0f, static_cast<float>(params_.expl_frontier_band_radius_m)));
  band_map_ = std::move(g);
  band_src_ = occ_grid_2d_.get();
}

void GoalSelector::invalidateFrontiersInBand(double now, Output& out) {
  if (!params_.expl_enabled || !frontier_manager_) return;
  ensureBandMap();
  if (band_map_.values.empty()) return;

  // D8b: ACTIVE / DORMANT frontiers whose centroid is within frontier_band_radius_m of a REAL
  // obstacle (band map above). Cells outside the window are NOT in the band (is2DOccupied would
  // report them occupied), so only in-bounds cells count.
  const auto view = band_map_.view();
  std::vector<uint64_t> to_invalidate;
  for (const auto& r : frontier_manager_->records()) {
    if (r.state != FrontierState::ACTIVE && r.state != FrontierState::DORMANT) continue;
    const int cx = map2d::worldToCell(r.centroid_xy.x(), view.origin_x, view.res);
    const int cy = map2d::worldToCell(r.centroid_xy.y(), view.origin_y, view.res);
    if (cx < 0 || cx >= view.dim_x || cy < 0 || cy >= view.dim_y) continue;
    if (map2d::is2DOccupied(view, cx, cy)) to_invalidate.push_back(r.id);
  }
  for (uint64_t id : to_invalidate) {
    frontier_manager_->markInvalidated(id, now);
  }
  if (!to_invalidate.empty()) {
    log(out, LogMessage::Level::kInfo,
        fmt("Exploration: invalidated %zu frontier(s) in the occupied/inflated band",
            to_invalidate.size()));
  }
}

void GoalSelector::rerelocateCurrent(Output& out) {
  if (!params_.relocate_occupied_goal) return;
  if (current_.kind == GoalKind::kNone) return;

  const RelocResult r = relocate(current_.published);  // from the stored (relocated) goal, as the planner does
  if (!r.ok) return;  // keep the previous goal; the next map update may make a cell available
  if ((r.position - current_.published).norm() <= 1e-6) return;

  current_.published = r.position;
  GoalCommand cmd;
  cmd.position = r.position;
  cmd.stamp_ns = current_.stamp_ns;  // same commitment -> same stamp (4.3)
  cmd.kind = current_.kind;
  cmd.relocated = (r.position - current_.target).norm() > 1e-6;
  out.term_goal = cmd;
  log(out, LogMessage::Level::kInfo,
      fmt("Goal re-relocated to (%.2f,%.2f,%.2f) (stamp %lld)", r.position.x(), r.position.y(),
          r.position.z(), static_cast<long long>(cmd.stamp_ns)));
}

// TODO(step 2): build a reachability-check map (inflated planning map) and flood-fill from the
// robot, so only frontiers reachable on it are selected. Detection stays on the detection grid.

}  // namespace goal_selector
