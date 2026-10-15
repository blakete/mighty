#include "goal_selector/goal_selector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <utility>

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

  // Manual-goal stuck watchdog (N4): runs here (not on the select tick) so it also works with
  // exploration disabled, where there is no select timer.
  // Two checks: start timeout (never moved after commit), then the normal stuck watchdog.
  const bool manual_start_timeout = manual_goal_active_ && manualStartTimeoutFired(now);
  const bool manual_stuck = manual_goal_active_ && !manual_start_timeout && stuckWatchdogFired(now);
  if (manual_start_timeout || manual_stuck) {
    if (manual_start_timeout) {
      log(out, LogMessage::Level::kWarn,
          fmt("Manual goal (stamp %lld) start timeout (no motion > %.2f m within %.1f s of commit) "
              "-> releasing it%s",
              static_cast<long long>(current_.stamp_ns), params_.expl_stuck_move_thresh_m,
              params_.manual_start_timeout_sec,
              params_.expl_enabled ? ", resuming exploration" : ", idle"));
    } else {
      log(out, LogMessage::Level::kWarn,
          fmt("Manual goal (stamp %lld) stuck (no motion > %.2f m for %.1f s) -> releasing it%s",
              static_cast<long long>(current_.stamp_ns), params_.expl_stuck_move_thresh_m,
              params_.expl_stuck_timeout_sec,
              params_.expl_enabled ? ", resuming exploration" : ", idle"));
    }
    manual_goal_active_ = false;
    manual_unreachable_ = false;
    unreachable_consec_count_ = 0;
    selectAndCommit(now, out);  // no-op unless exploration is enabled and ready
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
  if (params_.expl_persistent_map_enabled) return;  // the VisitedMap is unused (never absorbed)
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

void GoalSelector::detectAndUpdate(double now, const std::shared_ptr<const OccGrid2D>& detect_src,
                                   const OccGrid2D& diag_src, Output& out) {
  const Eigen::Vector3d robot_pose(pose_.x, pose_.y, pose_.yaw);
  const Eigen::Vector2d robot_xy(pose_.x, pose_.y);

  frontier_grid_ = buildFrontierGrid(*detect_src, frontier_passable_, frontier_inflated_only_);
  const auto& detect_grid = *frontier_grid_;
  const VisitedMap* visited_filter =
      params_.expl_detect_on_visited_map ? nullptr : visited_map_.get();
  auto clusters = frontier_detector_->detect(detect_grid, robot_xy, visited_filter, &frontier_passable_);

  // (The ESDF clearance filter is dropped, D3. FrontierManager::update INVALIDATES a record whose
  //  centroid falls in the occupied band only when the record is not re-matched this cycle; a
  //  matched record keeps its centroid even in the band.)

  const auto active_peers = params_.expl_use_minpos
                                ? peer_tracker_.getActivePeers(now, params_.expl_peer_timeout_sec)
                                : std::vector<PeerPose>{};
  frontier_manager_->update(clusters, detect_grid, robot_pose, now, active_peers);
  invalidateNogoFrontiers(now, out);

  maybePublishVisitedMap(now, out);

  // Throttled (2 s) diagnostic of the cell-state distribution.
  if (now - last_diag_log_t_ >= 2.0) {
    last_diag_log_t_ = now;
    int n_unknown = 0, n_free = 0, n_occ = 0;
    const auto& occ = diag_src.occupiedData();
    const auto& unk = diag_src.unknownData();
    for (size_t i = 0; i < occ.size(); ++i) {
      if (unk[i])
        ++n_unknown;
      else if (occ[i])
        ++n_occ;
      else
        ++n_free;
    }
    log(out, LogMessage::Level::kInfo,
        fmt("[expl] grid %dx%d  free=%d  occ=%d  unknown=%d  fresh=%zu  db=%zu", diag_src.width(),
            diag_src.height(), n_free, n_occ, n_unknown, clusters.size(),
            frontier_manager_->size()));
  }

  // Markers reflect the state before selection (as the planner did). The caller then drives the
  // selection immediately with onSelectTick(), so the robot starts moving as soon as the first
  // frontier exists (the planner called exploreSelectCallback() at this point).
  if (params_.expl_publish_markers) out.publish_markers = true;
}

Output GoalSelector::onOccGrid(double now, GridInput grid) {
  Output out;

  // Remember the mapper's ground-plane z so the visited map renders at the same height.
  occ2d_origin_z_ = grid.origin_z;

  // Exploration off: only keep the latest grid (for selectorMap()); no fusion, detection or
  // visited-map work.
  if (!params_.expl_enabled) {
    occ_grid_2d_ = OccGrid2D::fromTristate(grid.width, grid.height, grid.resolution, grid.origin_x,
                                           grid.origin_y, grid.data);
    return out;
  }

  // Persistent-map mode: detection is driven by onPersistentGrid. The live grid is only stored
  // (selection gate, selectorMap fallback); the VisitedMap is not fused, absorbed or published.
  if (params_.expl_persistent_map_enabled) {
    occ_grid_2d_ = OccGrid2D::fromTristate(grid.width, grid.height, grid.resolution, grid.origin_x,
                                           grid.origin_y, grid.data);
    return out;
  }

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

  // Absorb the freshly observed cells into the visited bitmap *before* detection.
  if (visited_map_) visited_map_->absorb(*occ_grid_2d_);

  // Pick the grid the detector runs on (persistent map or the local sliding window), then derive
  // the frontier map from it (occupied band -> occupied, unknown band -> unknown; N1/N5).
  std::shared_ptr<const OccGrid2D> detect_src;
  if (params_.expl_detect_on_visited_map && visited_map_ && !visited_map_->empty()) {
    detect_src = OccGrid2D::fromTristate(
        visited_map_->width(), visited_map_->height(), visited_map_->resolution(),
        visited_map_->originX(), visited_map_->originY(), visited_map_->data());
  } else {
    detect_src = occ_grid_2d_;
  }
  detectAndUpdate(now, detect_src, *occ_grid_2d_, out);
  return out;
}

Output GoalSelector::onPersistentGrid(double now, GridInput grid) {
  Output out;
  if (!params_.expl_enabled || !params_.expl_persistent_map_enabled) return out;
  if (!(frontier_detector_ && frontier_manager_ && state_initialized_)) return out;
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
      grid.data.size() < static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height)) {
    return out;
  }

  // The grid grows and its origin moves, so nothing here assumes a fixed extent. Radius mode crops
  // the raw cells before building the OccGrid2D, so the cost follows the window, not the map. The
  // crop edge is not a frontier: the detector's border_margin_cells keeps it out.
  if (!params_.expl_persistent_detect_whole_map) {
    const double r = params_.expl_persistent_detect_radius_m;
    const double res = grid.resolution;
    const int x0 = std::max(0, static_cast<int>(std::floor((pose_.x - r - grid.origin_x) / res)));
    const int y0 = std::max(0, static_cast<int>(std::floor((pose_.y - r - grid.origin_y) / res)));
    const int x1 = std::min(grid.width - 1,
                            static_cast<int>(std::floor((pose_.x + r - grid.origin_x) / res)));
    const int y1 = std::min(grid.height - 1,
                            static_cast<int>(std::floor((pose_.y + r - grid.origin_y) / res)));
    if (x1 < x0 || y1 < y0) {
      log(out, LogMessage::Level::kWarn,
          fmt("[expl] persistent grid does not cover the robot at (%.2f, %.2f) -> no detection",
              pose_.x, pose_.y));
      return out;
    }
    const int w = x1 - x0 + 1;
    const int h = y1 - y0 + 1;
    std::vector<int8_t> window(static_cast<size_t>(w) * h);
    for (int iy = 0; iy < h; ++iy) {
      const auto src = grid.data.begin() + static_cast<size_t>(y0 + iy) * grid.width + x0;
      std::copy(src, src + w, window.begin() + static_cast<size_t>(iy) * w);
    }
    grid.origin_x += x0 * res;
    grid.origin_y += y0 * res;
    grid.width = w;
    grid.height = h;
    grid.data = std::move(window);
  }

  auto src = OccGrid2D::fromTristate(grid.width, grid.height, grid.resolution, grid.origin_x,
                                     grid.origin_y, grid.data);
  detectAndUpdate(now, src, *src, out);
  return out;
}

// ----------------------------------------------------------------------------
// Select tick (exploreSelectCallback)
// ----------------------------------------------------------------------------

void GoalSelector::onGoalFence(double now, GoalFence fence, const Transform2D& map_to_fence) {
  goal_fence_ = std::move(fence);
  map_to_fence_ = map_to_fence;
  fence_t_ = now;
}

void GoalSelector::onGateEcho(double now, bool engaged) {
  gate_engaged_ = engaged;
  gate_t_ = now;
}

GoalSelector::FenceGate GoalSelector::fenceGate(double now, const char** why) const {
  const auto out = [why](FenceGate g, const char* w) {
    if (why) *why = w;
    return g;
  };
  if (!params_.expl_geofence_enabled) return out(FenceGate::kOpen, "disabled");
  if (goal_fence_ && now - fence_t_ <= params_.expl_geofence_max_age_sec) {
    if (!goal_fence_->active) return out(FenceGate::kOpen, "fence not enforcing");
    if (goal_fence_->ready) return out(FenceGate::kFilter, "fence enforcing");
    return out(FenceGate::kPause, "fence enforcing but not ready (localizing / anchor)");
  }
  // No fresh goal_fence. The drive gate's echo tells a restarting geofence from one that is off.
  const bool gate_fresh = gate_t_ >= 0.0 && now - gate_t_ <= params_.expl_geofence_gate_max_age_sec;
  if (gate_fresh && !gate_engaged_) return out(FenceGate::kOpen, "geofence not engaged");
  if (goal_fence_) {
    if (gate_fresh) return out(FenceGate::kPause, "no fresh goal_fence while the drive gate is engaged (geofence restarting?)");
    return out(FenceGate::kOpen, "no fresh goal_fence and no drive gate echo");
  }
  // Never heard a goal_fence: wait a little for the first one (startup), then fail open (geofence off
  // or an older image without goal_fence; the planner's grid and the drive gate still hold the fence).
  if (select_start_t_ >= 0.0 && now - select_start_t_ < params_.expl_geofence_startup_wait_sec)
    return out(FenceGate::kPause, "waiting for the first goal_fence");
  return out(FenceGate::kOpen, "no goal_fence received (geofence off or an older image)");
}

bool GoalSelector::fenceAllowsMap(double x, double y) const {
  const double c = std::cos(map_to_fence_.yaw);
  const double s = std::sin(map_to_fence_.yaw);
  return goalFenceAllows(*goal_fence_, c * x - s * y + map_to_fence_.x, s * x + c * y + map_to_fence_.y,
                         params_.expl_geofence_margin_m);
}

void GoalSelector::noteFenceGate(double now, FenceGate gate, const char* why, Output& out) {
  const bool changed = gate != last_fence_gate_ || last_fence_why_ != why;
  last_fence_gate_ = gate;
  last_fence_why_ = why;
  if (gate == FenceGate::kFilter && changed)
    log(out, LogMessage::Level::kInfo,
        fmt("Geofence: goal filter on (fence enforcing, offset %.2f m + margin %.2f m)",
            goal_fence_->offset_m, params_.expl_geofence_margin_m));
  else if (gate == FenceGate::kOpen && changed)
    log(out, LogMessage::Level::kInfo, fmt("Geofence: goal filter off (%s)", why));
  if (gate == FenceGate::kPause && (changed || now - last_fence_pause_log_t_ >= 5.0)) {
    last_fence_pause_log_t_ = now;
    log(out, LogMessage::Level::kWarn, fmt("Geofence: %s -> pausing new exploration goals", why));
  }
}

// A frontier that is not a valid goal (outside the fence, or inside it but closer to the fence line
// than the gate lets the rover drive) can never be reached, so drop it before ranking instead of
// letting FAILED statuses invalidate it one by one. Runs after every manager update and at every
// select tick, so re-detected ones are dropped again. Also checks the published target of the current
// frontier pursuit (record centroids keep moving by EMA after the commit).
void GoalSelector::invalidateNogoFrontiers(double now, Output& out) {
  if (!frontier_manager_ || fenceGate(now) != FenceGate::kFilter) return;

  std::vector<std::pair<uint64_t, Eigen::Vector2d>> nogo;
  for (const auto& r : frontier_manager_->records()) {
    if (r.state != FrontierState::ACTIVE && r.state != FrontierState::DORMANT) continue;
    if (!fenceAllowsMap(r.centroid_xy.x(), r.centroid_xy.y())) nogo.emplace_back(r.id, r.centroid_xy);
  }
  if (exploration_active_ && current_.kind == GoalKind::kFrontier &&
      !fenceAllowsMap(current_.target.x(), current_.target.y())) {
    const auto* r = frontier_manager_->find(current_explore_id_);
    const bool listed = std::any_of(nogo.begin(), nogo.end(),
                                    [&](const auto& n) { return n.first == current_explore_id_; });
    if (r && !listed && (r->state == FrontierState::ACTIVE || r->state == FrontierState::DORMANT))
      nogo.emplace_back(current_explore_id_, current_.target.head<2>());
  }
  if (nogo.empty()) return;

  std::string list;
  for (size_t i = 0; i < nogo.size(); ++i) {
    frontier_manager_->markInvalidated(nogo[i].first, now);
    if (i < 6)
      list += fmt("%s%lu (%.2f, %.2f)", i ? ", " : "", static_cast<unsigned long>(nogo[i].first),
                  nogo[i].second.x(), nogo[i].second.y());
  }
  if (nogo.size() > 6) list += fmt(", ... (+%zu)", nogo.size() - 6);
  log(out, LogMessage::Level::kInfo,
      fmt("Exploration: %zu frontier(s) outside the geofence goal area -> invalidated: %s", nogo.size(),
          list.c_str()));
}

Output GoalSelector::onSelectTick(double now) {
  Output out;
  if (select_start_t_ < 0.0) select_start_t_ = now;
  selectAndCommit(now, out);
  return out;
}

// Stuck watchdog (frontier and manual pursuit): armed after the robot first moves more than
// stuck_move_thresh_m; fires (once, then disarms) after stuck_timeout_sec without such motion.
// stuck_timeout_sec <= 0 disables it.
void GoalSelector::armStuckWatchdog(double now) {
  explore_last_progress_xy_ = Eigen::Vector2d(pose_.x, pose_.y);
  explore_last_progress_t_ = now;
  explore_has_moved_ = false;
}

// Manual start check: armed at commit (from the commit pose); fires once if the robot has not moved
// more than stuck_move_thresh_m within manual_goal.start_timeout_sec. Moving past the threshold
// marks the pursuit as started (explore_has_moved_, clock restarted) and hands over to
// stuckWatchdogFired(). start_timeout_sec <= 0 disables the check.
bool GoalSelector::manualStartTimeoutFired(double now) {
  if (explore_has_moved_) return false;
  const Eigen::Vector2d xy(pose_.x, pose_.y);
  if ((xy - explore_last_progress_xy_).norm() > params_.expl_stuck_move_thresh_m) {
    explore_last_progress_xy_ = xy;
    explore_last_progress_t_ = now;
    explore_has_moved_ = true;
    return false;
  }
  if (params_.manual_start_timeout_sec <= 0.0 || explore_last_progress_t_ < 0.0) return false;
  if (now - explore_last_progress_t_ >= params_.manual_start_timeout_sec) {
    explore_last_progress_t_ = -1.0;
    return true;
  }
  return false;
}

bool GoalSelector::stuckWatchdogFired(double now) {
  if (params_.expl_stuck_timeout_sec <= 0.0) return false;
  const Eigen::Vector2d xy(pose_.x, pose_.y);
  if ((xy - explore_last_progress_xy_).norm() > params_.expl_stuck_move_thresh_m) {
    explore_last_progress_xy_ = xy;  // progressed -> reset the clock
    explore_last_progress_t_ = now;
    explore_has_moved_ = true;
    return false;
  }
  if (explore_has_moved_ && now - explore_last_progress_t_ >= params_.expl_stuck_timeout_sec) {
    explore_last_progress_t_ = -1.0;
    explore_has_moved_ = false;
    return true;
  }
  return false;
}

// While the fence pause holds the current frontier pursuit, the gate is holding the robot too: keep
// the pursuit's stuck and pursuit-timeout clocks from running, or the first tick after the fence is
// ready again drops a valid goal as stuck / timed out. Both clocks run normally again on resume.
void GoalSelector::holdPursuitClocks(double now) {
  const double dt = fence_pause_tick_t_ >= 0.0 ? now - fence_pause_tick_t_ : 0.0;
  fence_pause_tick_t_ = now;
  if (!exploration_active_) return;
  if (explore_last_progress_t_ >= 0.0) explore_last_progress_t_ = now;
  frontier_manager_->extendPursuit(current_explore_id_, dt);
}

void GoalSelector::selectAndCommit(double now, Output& out) {
  if (!params_.expl_enabled) return;
  if (manual_goal_active_) return;  // manual override: no selection / preemption / watchdog
  if (!occ_grid_2d_ || !frontier_manager_) return;
  if (!frontier_grid_) return;
  if (!state_initialized_) return;

  // Geofence goal filter: pause while the fence is enforcing but not usable (holds the current
  // pursuit; no commits, preemption or watchdog switches); otherwise drop no-go frontiers before
  // ranking, also when the fence arrived after the last detection pass.
  const char* why = "";
  const FenceGate gate = fenceGate(now, &why);
  noteFenceGate(now, gate, why, out);
  if (gate == FenceGate::kPause) {
    holdPursuitClocks(now);
    return;
  }
  fence_pause_tick_t_ = -1.0;
  if (gate == FenceGate::kFilter) invalidateNogoFrontiers(now, out);

  // If we still have an in-progress exploration goal that hasn't been marked VISITED/INVALIDATED
  // yet, either leave it alone (legacy hard-commit) or, with preemption enabled, keep re-ranking
  // every tick and switch when a different frontier beats the current one by preempt_margin.
  if (exploration_active_) {
    auto* r = frontier_manager_->find(current_explore_id_);
    if (r && (r->state == FrontierState::ACTIVE || r->state == FrontierState::DORMANT)) {
      // --- Static stuck watchdog (runs regardless of preemption) -----------------------------
      if (stuckWatchdogFired(now)) {
        log(out, LogMessage::Level::kWarn,
            fmt("Exploration: frontier %lu stuck (no motion > %.2f m for %.1f s) "
                "-> invalidating, re-selecting",
                static_cast<unsigned long>(current_explore_id_), params_.expl_stuck_move_thresh_m,
                params_.expl_stuck_timeout_sec));
        frontier_manager_->markInvalidated(current_explore_id_, now);
        exploration_active_ = false;
        // fall through to the normal selection path below (picks a new goal now)
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
              pose_p, *frontier_grid_, peers, params_.expl_min_frontier_dist_to_peers_m);
        } else {
          best = frontier_manager_->selectNextGoal(pose_p, *frontier_grid_);
        }
        if (!best || best->id == current_explore_id_) return;

        const auto u_cur =
            frontier_manager_->utilityOf(current_explore_id_, pose_p, *frontier_grid_);
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
    next = frontier_manager_->selectNextGoalMinPos(robot_pose, *frontier_grid_, peers,
                                                   params_.expl_min_frontier_dist_to_peers_m);
  } else {
    next = frontier_manager_->selectNextGoal(robot_pose, *frontier_grid_);
  }
  if (!next) {
    // A frontier pursuit that ended in REACHED (exploration_active_ already cleared) counts the same
    // as one still in progress: with no frontier left, go home.
    if ((exploration_active_ || frontier_reached_pending_) && !params_.expl_return_home_enabled) {
      // Automatic return home switched off: stay where we are (idle) until a frontier appears.
      log(out, LogMessage::Level::kInfo,
          fmt("[mighty] No frontiers left. Robot at (%.2f, %.2f, %.2f). Automatic return home is off "
              "(exploration.return_home.enabled=false): idle",
              pose_.x, pose_.y, pose_.z));
    } else if (exploration_active_ || frontier_reached_pending_) {
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
    frontier_reached_pending_ = false;
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
  frontier_reached_pending_ = false;
  explore_committed_at_t_ = now;
  unreachable_consec_count_ = 0;
  armStuckWatchdog(now);  // fresh for this pursuit
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

  // (commit never touches manual_goal_active_: callers set it, as terminalGoalCallbackImpl did.)
  if (kind == GoalKind::kManual) manual_unreachable_ = false;

  GoalCommand cmd;
  cmd.position = target;  // published exactly as given
  cmd.stamp_ns = current_.stamp_ns;
  cmd.kind = kind;
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
  frontier_reached_pending_ = false;
  exploration_start_captured_ = false;
  unreachable_consec_count_ = 0;
  home_return_requested_ = false;

  commit(GoalKind::kManual, Eigen::Vector3d(x, y, z), now, out);
  manual_goal_active_ = true;
  armStuckWatchdog(now);  // N4: start timer runs from the commit pose, then the stuck watchdog
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
  frontier_reached_pending_ = false;
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
        // Manual goals are never released or resent on FAILED: only warn + flag (D8a). (They are
        // released by REACHED or the stuck watchdog.)
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
        // Remember that the session's frontier was completed, so the next select tick still sends
        // the robot home if no frontier is left (same as a frontier cleared by a map update).
        frontier_reached_pending_ = true;
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
// Frontier map (N1/N5)
// ----------------------------------------------------------------------------

std::shared_ptr<const OccGrid2D> GoalSelector::buildFrontierGrid(const OccGrid2D& g,
                                                                    std::vector<uint8_t>& passable,
                                                                    std::vector<uint8_t>& inflated_only) const {
  const int W = g.width();
  const int H = g.height();
  const auto& occ = g.occupiedData();
  const auto& unk = g.unknownData();
  const size_t n = static_cast<size_t>(W) * H;

  std::vector<int8_t> values(n, map2d::kFree);
  for (size_t i = 0; i < n; ++i) {
    if (occ[i])
      values[i] = map2d::kOccupied;
    else if (unk[i])
      values[i] = map2d::kUnknown;
  }

  // Unknown band from the raw values (before occupied inflation); it never marks cells occupied.
  std::vector<uint8_t> unknown_band;
  map2d::inflateUnknown(values, W, H, g.resolution(),
                        std::max(0.0f, static_cast<float>(params_.unknown_inflation_2d_m)),
                        unknown_band);

  // Occupied band: cells within inflation_2d_m of an occupied cell become occupied.
  map2d::inflate(values, inflated_only, W, H, g.resolution(),
                 std::max(0.0f, static_cast<float>(params_.inflation_2d_m)));

  // Remaining non-occupied cells in the unknown band become unknown. Those cells are known free
  // (the band excludes real unknown), so flag them passable: the detector's reachability walk may
  // cross them, but never real unknown or the occupied band.
  passable.assign(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (unknown_band[i] && values[i] != map2d::kOccupied) {
      values[i] = map2d::kUnknown;
      passable[i] = 1;
    }
  }

  return OccGrid2D::fromTristate(W, H, g.resolution(), g.originX(), g.originY(), values);
}

bool GoalSelector::selectorMap(SelectorMap& m) const {
  // Exploring: the grid the detector last used. Otherwise (or before the first detection cycle):
  // the same derivation from the latest raw grid.
  std::shared_ptr<const OccGrid2D> g = frontier_grid_;
  const std::vector<uint8_t>* passable = &frontier_passable_;
  const std::vector<uint8_t>* inflated_only = &frontier_inflated_only_;
  std::vector<uint8_t> passable_raw, inflated_only_raw;
  if (!g) {
    if (!occ_grid_2d_) return false;
    g = buildFrontierGrid(*occ_grid_2d_, passable_raw, inflated_only_raw);
    passable = &passable_raw;
    inflated_only = &inflated_only_raw;
  }

  const size_t n = static_cast<size_t>(g->width()) * g->height();
  const auto& occ = g->occupiedData();
  const auto& unk = g->unknownData();
  m.width = g->width();
  m.height = g->height();
  m.resolution = g->resolution();
  m.origin_x = g->originX();
  m.origin_y = g->originY();
  m.origin_z = occ2dOriginZ();
  m.data.assign(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (occ[i])
      m.data[i] = (*inflated_only)[i] ? 99 : 100;  // 99 = band, so RViz's costmap scheme shows it
    else if (unk[i])
      m.data[i] = (*passable)[i] ? 50 : -1;        // 50 = unknown band, -1 = real unknown
  }
  return true;
}

}  // namespace goal_selector
