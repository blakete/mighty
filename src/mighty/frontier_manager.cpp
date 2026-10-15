#include "mighty/frontier_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace {

inline double clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

// Goal of a fresh cluster: its frontier cell nearest the centroid, or the
// centroid for clusters without cells.
inline Eigen::Vector2d goalOf(const FrontierCluster& c) {
  return c.goal.value_or(c.centroid);
}

}  // namespace

bool FrontierManager::isInsideMap(const FrontierRecord& r,
                                  const OccGrid2D& grid) const {
  int ix, iy;
  grid.worldToGrid(r.goal_xy.x(), r.goal_xy.y(), ix, iy);
  // Account for the same border margin the detector uses by treating cells
  // very close to the edge as "not really inside" — they may have just slid
  // in and not been observed yet.
  return ix >= 0 && ix < grid.width() && iy >= 0 && iy < grid.height();
}

void FrontierManager::update(const std::vector<FrontierCluster>& fresh,
                             const OccGrid2D& current_grid,
                             const Eigen::Vector3d& robot_pose,
                             double t_now,
                             const std::vector<PeerPose>& peers) {
  const Eigen::Vector2d robot_xy = robot_pose.head<2>();
  const double dt = (last_update_t_ > 0.0) ? std::max(0.0, t_now - last_update_t_)
                                           : 0.0;

  // Cell key of a world point in the current grid (-1 outside it).
  const int W = current_grid.width();
  auto cellKey = [&](const Eigen::Vector2d& p) -> long {
    int ix, iy;
    current_grid.worldToGrid(p.x(), p.y(), ix, iy);
    if (!current_grid.inBounds(ix, iy)) return -1;
    return static_cast<long>(iy) * W + ix;
  };
  std::unordered_map<long, int> fresh_of_cell;  // cell key -> fresh cluster index
  for (size_t i = 0; i < fresh.size(); ++i) {
    for (const auto& c : fresh[i].cells) {
      const long k = cellKey(c);
      if (k >= 0) fresh_of_cell.emplace(k, static_cast<int>(i));
    }
  }

  // ---- Step a: match fresh -> existing by shared cells ----
  // A fresh cluster that shares cells with an ACTIVE/DORMANT record is the same
  // frontier even if its centroid moved more than merge_radius_m (a split piece
  // changed shape). Greedy by shared-cell count, ties to the older record.
  // VISITED records are left to the centroid pass so they only absorb clusters
  // at the place they were visited; INVALIDATED records never match.
  std::vector<int> match_for_fresh(fresh.size(), -1);
  std::vector<uint8_t> existing_matched(records_.size(), 0);
  if (!fresh_of_cell.empty()) {
    struct Overlap {
      int count;
      size_t j;
      int i;
    };
    std::vector<Overlap> pairs;
    std::unordered_map<int, int> counts;
    for (size_t j = 0; j < records_.size(); ++j) {
      const auto& rec = records_[j];
      if (rec.state != FrontierState::ACTIVE && rec.state != FrontierState::DORMANT) continue;
      counts.clear();
      for (const auto& c : rec.cells) {
        const long k = cellKey(c);
        if (k < 0) continue;
        const auto it = fresh_of_cell.find(k);
        if (it != fresh_of_cell.end()) ++counts[it->second];
      }
      for (const auto& [i, n] : counts) pairs.push_back({n, j, i});
    }
    std::sort(pairs.begin(), pairs.end(), [&](const Overlap& a, const Overlap& b) {
      if (a.count != b.count) return a.count > b.count;
      if (records_[a.j].id != records_[b.j].id) return records_[a.j].id < records_[b.j].id;
      return a.i < b.i;
    });
    for (const auto& o : pairs) {
      if (match_for_fresh[o.i] >= 0 || existing_matched[o.j]) continue;
      match_for_fresh[o.i] = static_cast<int>(o.j);
      existing_matched[o.j] = 1;
    }
  }

  // ---- Step b: brute-force greedy centroid match for the rest ----
  for (size_t i = 0; i < fresh.size(); ++i) {
    if (match_for_fresh[i] >= 0) continue;
    double best_d = params_.merge_radius_m;  // strictly less than = match
    int best_j = -1;
    for (size_t j = 0; j < records_.size(); ++j) {
      if (existing_matched[j]) continue;
      const auto& rec = records_[j];
      // INVALIDATED records (wall-huggers etc.) stay skipped — we want fresh
      // clusters near them to be re-evaluated, not re-bound to a bad record.
      // VISITED records DO participate in matching so that fresh clusters at
      // an already-visited location get absorbed into the existing record
      // instead of spawning a brand-new ACTIVE ghost every dwell cycle.
      if (rec.state == FrontierState::INVALIDATED) {
        continue;
      }
      const double d = (fresh[i].centroid - rec.centroid_xy).norm();
      if (d < best_d) {
        best_d = d;
        best_j = static_cast<int>(j);
      }
    }
    if (best_j >= 0) {
      match_for_fresh[i] = best_j;
      existing_matched[best_j] = 1;
    }
  }

  // Apply matches with EMA, then insert unmatched fresh as new records.
  // claimed[i]: fresh cluster i now belongs to a pending record (matched to a
  // non-VISITED record or inserted). Clusters absorbed by a VISITED record or
  // dropped by the keep-out claim nothing.
  std::vector<uint8_t> claimed(fresh.size(), 0);
  const double alpha = params_.centroid_ema_alpha;
  for (size_t i = 0; i < fresh.size(); ++i) {
    const auto& c = fresh[i];
    if (match_for_fresh[i] >= 0) {
      auto& r = records_[match_for_fresh[i]];
      r.last_seen_t = t_now;
      // Don't revive a VISITED record or overwrite its centroid/size —
      // absorbing the fresh cluster is enough to keep it from spawning a
      // duplicate. Its VISITED state is preserved.
      if (r.state != FrontierState::VISITED) {
        r.centroid_xy = alpha * c.centroid + (1.0 - alpha) * r.centroid_xy;
        r.goal_xy     = goalOf(c);
        r.cells       = c.cells;
        r.size_cells  = c.size_cells;
        r.aabb_min    = c.aabb_min;
        r.aabb_max    = c.aabb_max;
        r.state       = FrontierState::ACTIVE;
        claimed[i] = 1;
      }
    } else {
      // Suppress fresh clusters that fall inside the keep-out radius of a
      // recently-INVALIDATED record. The matching loop above intentionally
      // skipped INVALIDATED so we wouldn't revive a bad record; without this
      // check we'd just spawn a brand-new ACTIVE ghost in the same place and
      // the agent would re-pursue the goal it just gave up on.
      if (params_.invalidation_keep_out_radius_m > 0.0) {
        bool suppressed = false;
        const double r2 = params_.invalidation_keep_out_radius_m
                        * params_.invalidation_keep_out_radius_m;
        for (const auto& rec : records_) {
          if (rec.state != FrontierState::INVALIDATED) continue;
          if (params_.invalidation_cooldown_sec > 0.0) {
            if (rec.invalidated_at_t < 0.0) continue;
            if (t_now - rec.invalidated_at_t
                > params_.invalidation_cooldown_sec) continue;
          }
          if ((c.centroid - rec.centroid_xy).squaredNorm() < r2) {
            suppressed = true;
            break;
          }
        }
        if (suppressed) continue;  // drop fresh cluster, don't spawn new record
      }
      FrontierRecord r;
      r.id           = next_id_++;
      r.centroid_xy  = c.centroid;
      r.goal_xy      = goalOf(c);
      r.cells        = c.cells;
      r.size_cells   = c.size_cells;
      r.aabb_min     = c.aabb_min;
      r.aabb_max     = c.aabb_max;
      r.first_seen_t = t_now;
      r.last_seen_t  = t_now;
      r.state        = FrontierState::ACTIVE;
      records_.push_back(r);
      claimed[i] = 1;
      // The new record is implicitly "matched" — don't classify it as
      // unmatched-in-window in step d. Mark its slot as matched.
      existing_matched.push_back(1);
    }
  }

  // ---- Step d/e: classify previously-known unmatched records ----
  // A frontier cell borders unknown, so the liveness test needs at least one
  // cell of neighbourhood even when verify_radius_cells is 0.
  const int live_rcells = std::max(1, params_.verify_radius_cells);
  std::vector<uint8_t> duplicate(records_.size(), 0);
  for (size_t j = 0; j < records_.size(); ++j) {
    if (existing_matched[j]) continue;
    auto& r = records_[j];
    if (r.state == FrontierState::VISITED ||
        r.state == FrontierState::INVALIDATED) {
      continue;
    }

    if (!isInsideMap(r, current_grid)) {
      // Step e: outside local window -> DORMANT, untouched.
      r.state = FrontierState::DORMANT;
      continue;
    }

    // Step d: inside the local window but the detector didn't pick it up.
    int ix, iy;
    current_grid.worldToGrid(r.goal_xy.x(), r.goal_xy.y(), ix, iy);
    if (current_grid.isOccupied(ix, iy)) {
      r.state = FrontierState::INVALIDATED;
      r.invalidated_at_t = t_now;
      continue;
    }

    if (r.cells.empty()) {
      // No cells (hand-made cluster): judge the goal point's neighbourhood.
      if (current_grid.isFree(ix, iy)) {
        // Check for unknown neighbors within verify_radius_cells. If none, the
        // area is fully explored -> VISITED. Otherwise leave ACTIVE for a later
        // detection cycle.
        bool has_unknown_nbr = false;
        const int rcells = params_.verify_radius_cells;
        for (int dy = -rcells; dy <= rcells && !has_unknown_nbr; ++dy) {
          for (int dx = -rcells; dx <= rcells && !has_unknown_nbr; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (current_grid.isUnknown(ix + dx, iy + dy)) {
              has_unknown_nbr = true;
            }
          }
        }
        if (!has_unknown_nbr) {
          r.state = FrontierState::VISITED;
        }
        // else: keep ACTIVE, just don't bump last_seen_t.
      }
      // else: cell is itself unknown — leave the record alone. Will be re-matched
      // on a later cycle when the detector picks it up.
      continue;
    }

    // Judge the record by its cells. Live = still a free cell with an UNKNOWN cell
    // (inside the grid) within live_rcells. A cell outside the grid gives no
    // evidence and stays live.
    std::vector<Eigen::Vector2d> live;
    bool all_taken = true;  // every live cell belongs to a fresh cluster another record claimed
    for (const auto& c : r.cells) {
      int cx, cy;
      current_grid.worldToGrid(c.x(), c.y(), cx, cy);
      if (!current_grid.inBounds(cx, cy)) {
        live.push_back(c);
        all_taken = false;
        continue;
      }
      if (!current_grid.isFree(cx, cy)) continue;
      bool has_unknown_nbr = false;
      for (int dy = -live_rcells; dy <= live_rcells && !has_unknown_nbr; ++dy) {
        for (int dx = -live_rcells; dx <= live_rcells && !has_unknown_nbr; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const int nx = cx + dx, ny = cy + dy;
          if (current_grid.inBounds(nx, ny) && current_grid.isUnknown(nx, ny)) {
            has_unknown_nbr = true;
          }
        }
      }
      if (!has_unknown_nbr) continue;
      live.push_back(c);
      const auto it = fresh_of_cell.find(static_cast<long>(cy) * W + cx);
      if (it == fresh_of_cell.end() || !claimed[it->second]) all_taken = false;
    }
    if (live.empty()) {
      // Everything this frontier bordered has been observed.
      r.state = FrontierState::VISITED;
      continue;
    }
    if (all_taken) {
      // Another record now owns all of what is left: drop this duplicate.
      duplicate[j] = 1;
      continue;
    }
    // Keep it, shrunk to the cells that still border unknown, with the goal on one.
    double best = std::numeric_limits<double>::infinity();
    for (const auto& c : live) {
      const double d2 = (c - r.centroid_xy).squaredNorm();
      if (d2 < best) {
        best = d2;
        r.goal_xy = c;
      }
    }
    r.cells = std::move(live);
  }
  if (std::find(duplicate.begin(), duplicate.end(), 1) != duplicate.end()) {
    size_t k = 0;
    for (size_t j = 0; j < records_.size(); ++j) {
      if (!duplicate[j]) records_[k++] = std::move(records_[j]);
    }
    records_.resize(k);
  }

  // ---- Step f: robot-proximity dwell-based visit check ----
  for (auto& r : records_) {
    if (r.state == FrontierState::VISITED ||
        r.state == FrontierState::INVALIDATED) {
      continue;
    }
    const double d = (robot_xy - r.goal_xy).norm();
    if (d < params_.visit_radius_m) {
      r.dwell_time_sec += dt;
      if (r.dwell_time_sec >= params_.visit_dwell_sec) {
        ++r.visit_count;
        r.state = FrontierState::VISITED;
      }
    } else {
      // Reset dwell when the robot leaves the visit radius — visits must be
      // contiguous, not the sum of brief drive-bys.
      r.dwell_time_sec = 0.0;
    }
  }

  // ---- Peer-presence visit suppression ----
  // If any active peer's pose is within peer_visit_radius_m of a record's
  // goal, treat the record as VISITED. Sticky — once flipped, the record
  // stays VISITED for the rest of the run (or until eviction). This catches
  // peers that have already explored an area we still see as a frontier
  // (e.g. when peer-shared visited_maps don't propagate cleanly under
  // frame drift). No dwell — peer pose history is implicit in the FrontierRecord
  // state, so a single sample where a peer is in radius is enough.
  if (params_.peer_visit_radius_m > 0.0 && !peers.empty()) {
    const double r2 = params_.peer_visit_radius_m * params_.peer_visit_radius_m;
    for (auto& r : records_) {
      if (r.state != FrontierState::ACTIVE &&
          r.state != FrontierState::DORMANT) continue;
      for (const auto& p : peers) {
        if ((p.position - r.goal_xy).squaredNorm() < r2) {
          r.state = FrontierState::VISITED;
          ++r.visit_count;
          break;
        }
      }
    }
  }

  // ---- Pursuit-timeout check ----
  // Any record with an armed deadline that has elapsed is INVALIDATED if it's
  // still ACTIVE or DORMANT — once we've given up trying to reach it, the
  // tombstone participates in the invalidation keep-out so the next detection
  // cycle doesn't immediately spawn a fresh ACTIVE ghost at the same spot.
  // Eviction (evictIfOverCap) prefers INVALIDATED → DORMANT → VISITED, so the
  // DB stays bounded. Records already VISITED/INVALIDATED just have their
  // deadlines cleared.
  if (params_.pursuit_timeout_factor > 0.0) {
    for (auto& r : records_) {
      if (r.pursuit_deadline_t <= 0.0) continue;
      if (r.state != FrontierState::ACTIVE &&
          r.state != FrontierState::DORMANT) {
        r.pursuit_deadline_t = -1.0;
        r.pursuit_budget_sec = 0.0;
        continue;
      }
      if (t_now >= r.pursuit_deadline_t) {
        r.state = FrontierState::INVALIDATED;
        r.invalidated_at_t = t_now;
        r.pursuit_deadline_t = -1.0;
        r.pursuit_budget_sec = 0.0;
      }
    }
  }

  evictIfOverCap();
  last_update_t_ = t_now;
}

double FrontierManager::computeUtility(const FrontierRecord& r,
                                       const Eigen::Vector3d& robot_pose,
                                       const OccGrid2D& current_grid) const {
  const Eigen::Vector2d robot_xy = robot_pose.head<2>();
  const double yaw = robot_pose.z();

  const double size_norm = clamp01(r.size_cells * current_grid.resolution()
                                       * current_grid.resolution()
                                       / params_.size_ref_m2);

  const double dist = (robot_xy - r.goal_xy).norm();
  const double dist_norm = clamp01(dist / params_.dist_ref_m);

  // Info gain: count UNKNOWN cells in a disk of radius sensor_radius_m around
  // the goal in the CURRENT grid. If the goal is outside the current
  // map, the disk count will be 0 — that's intentional (we have no fresh
  // info-gain estimate for DORMANT frontiers, so they win on size/distance).
  double info_norm = 0.0;
  {
    int cx, cy;
    current_grid.worldToGrid(r.goal_xy.x(), r.goal_xy.y(), cx, cy);
    if (current_grid.inBounds(cx, cy)) {
      const int rcells = static_cast<int>(
          std::ceil(params_.sensor_radius_m / current_grid.resolution()));
      const double r2 = static_cast<double>(rcells) * rcells;
      int count = 0;
      int capacity = 0;
      for (int dy = -rcells; dy <= rcells; ++dy) {
        for (int dx = -rcells; dx <= rcells; ++dx) {
          if (dx * dx + dy * dy > r2) continue;
          ++capacity;
          if (current_grid.isUnknown(cx + dx, cy + dy)) ++count;
        }
      }
      if (capacity > 0) {
        info_norm = clamp01(static_cast<double>(count) / capacity);
      }
    }
  }

  // Heading alignment: cosine of angle between robot heading and direction
  // to centroid. Clamped at 0 (no penalty for behind-robot frontiers; only a
  // bonus for in-front).
  double heading_term = 0.0;
  {
    const Eigen::Vector2d to = r.goal_xy - robot_xy;
    if (to.norm() > 1e-6) {
      const double angle_to = std::atan2(to.y(), to.x());
      heading_term = std::max(0.0, std::cos(angle_to - yaw));
    }
  }

  return params_.w_size    * size_norm
       - params_.w_dist    * dist_norm
       + params_.w_info    * info_norm
       - params_.w_revisit * r.visit_count
       + params_.w_heading * heading_term;
}

std::optional<FrontierRecord> FrontierManager::selectNextGoal(
    const Eigen::Vector3d& robot_pose, const OccGrid2D& current_grid) const {
  auto pickBest = [&](FrontierState want) -> std::optional<FrontierRecord> {
    double best_u = -std::numeric_limits<double>::infinity();
    int best_idx = -1;
    for (size_t i = 0; i < records_.size(); ++i) {
      if (records_[i].state != want) continue;
      const double u = computeUtility(records_[i], robot_pose, current_grid);
      if (u > best_u) {
        best_u = u;
        best_idx = static_cast<int>(i);
      }
    }
    if (best_idx < 0) return std::nullopt;
    if (best_u < params_.goal_select_threshold) return std::nullopt;
    FrontierRecord r = records_[best_idx];
    r.cached_utility = best_u;
    return r;
  };

  // Two-tier: exhaust ACTIVE before falling back to DORMANT.
  if (auto r = pickBest(FrontierState::ACTIVE)) return r;
  if (auto r = pickBest(FrontierState::DORMANT)) return r;
  return std::nullopt;
}

std::optional<FrontierRecord> FrontierManager::selectNextGoalMinPos(
    const Eigen::Vector3d& robot_pose,
    const OccGrid2D& current_grid,
    const std::vector<PeerPose>& peers,
    double min_dist_to_peers_m) const {
  const Eigen::Vector2d robot_xy = robot_pose.head<2>();

  struct Candidate {
    int idx;
    int rank;
    double dist;
    double utility;
  };

  auto collectCandidates = [&](FrontierState want) -> std::vector<Candidate> {
    std::vector<Candidate> cands;
    for (size_t i = 0; i < records_.size(); ++i) {
      if (records_[i].state != want) continue;

      if (min_dist_to_peers_m > 0.0) {
        bool too_close = false;
        for (const auto& p : peers) {
          if ((p.position - records_[i].goal_xy).norm() < min_dist_to_peers_m) {
            too_close = true;
            break;
          }
        }
        if (too_close) continue;
      }

      const double d_self = (robot_xy - records_[i].goal_xy).norm();
      int rank = 0;
      for (const auto& p : peers) {
        if ((p.position - records_[i].goal_xy).norm() < d_self) ++rank;
      }
      const double u = computeUtility(records_[i], robot_pose, current_grid);
      if (u < params_.goal_select_threshold) continue;  // same cutoff as selectNextGoal
      cands.push_back({static_cast<int>(i), rank, d_self, u});
    }
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
      if (a.rank != b.rank) return a.rank < b.rank;
      if (a.dist != b.dist) return a.dist < b.dist;
      return a.utility > b.utility;
    });
    return cands;
  };

  // Two-tier: exhaust ACTIVE before falling back to DORMANT.
  for (FrontierState tier : {FrontierState::ACTIVE, FrontierState::DORMANT}) {
    auto cands = collectCandidates(tier);
    if (!cands.empty()) {
      FrontierRecord r = records_[cands[0].idx];
      r.cached_utility = cands[0].utility;
      return r;
    }
  }
  return std::nullopt;
}

void FrontierManager::markVisited(uint64_t id) {
  for (auto& r : records_) {
    if (r.id == id) {
      ++r.visit_count;
      r.state = FrontierState::VISITED;
      r.pursuit_deadline_t = -1.0;
      r.pursuit_budget_sec = 0.0;
      return;
    }
  }
}

void FrontierManager::markInvalidated(uint64_t id, double t_now) {
  for (auto& r : records_) {
    if (r.id == id) {
      r.state = FrontierState::INVALIDATED;
      r.invalidated_at_t = t_now;
      r.pursuit_deadline_t = -1.0;
      r.pursuit_budget_sec = 0.0;
      return;
    }
  }
}

void FrontierManager::markSelected(uint64_t id, const Eigen::Vector2d& robot_xy,
                                   double t_now) {
  if (params_.pursuit_timeout_factor <= 0.0) return;  // feature disabled
  for (auto& r : records_) {
    if (r.id != id) continue;
    // Don't clobber a deadline that's already armed for this record — we keep
    // ticking from first selection even if the select tick re-affirms the
    // same goal later (e.g. after a re-match).
    if (r.pursuit_deadline_t > 0.0) return;
    const double dist = (r.goal_xy - robot_xy).norm();
    const double v_ref = std::max(1e-3, params_.pursuit_timeout_v_ref);
    const double budget = std::max(params_.pursuit_timeout_min_sec,
                                   dist / v_ref * params_.pursuit_timeout_factor);
    r.pursuit_budget_sec = budget;
    r.pursuit_deadline_t = t_now + budget;
    return;
  }
}

void FrontierManager::extendPursuit(uint64_t id, double dt) {
  if (dt <= 0.0) return;
  for (auto& r : records_) {
    if (r.id == id) {
      if (r.pursuit_deadline_t > 0.0) r.pursuit_deadline_t += dt;
      return;
    }
  }
}

void FrontierManager::clearPursuit(uint64_t id) {
  for (auto& r : records_) {
    if (r.id == id) {
      r.pursuit_deadline_t = -1.0;
      r.pursuit_budget_sec = 0.0;
      return;
    }
  }
}

std::optional<double> FrontierManager::utilityOf(
    uint64_t id, const Eigen::Vector3d& robot_pose,
    const OccGrid2D& current_grid) const {
  for (const auto& r : records_) {
    if (r.id == id) return computeUtility(r, robot_pose, current_grid);
  }
  return std::nullopt;
}

const FrontierRecord* FrontierManager::find(uint64_t id) const {
  for (const auto& r : records_) {
    if (r.id == id) return &r;
  }
  return nullptr;
}

void FrontierManager::evictIfOverCap() {
  if (static_cast<int>(records_.size()) <= params_.max_frontiers) return;

  // Eviction priority: oldest VISITED > oldest INVALIDATED > oldest DORMANT.
  // Never evict ACTIVE.
  auto evictOne = [&](FrontierState target) -> bool {
    int oldest_idx = -1;
    double oldest_t = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < records_.size(); ++i) {
      if (records_[i].state != target) continue;
      if (records_[i].first_seen_t < oldest_t) {
        oldest_t = records_[i].first_seen_t;
        oldest_idx = static_cast<int>(i);
      }
    }
    if (oldest_idx < 0) return false;
    records_.erase(records_.begin() + oldest_idx);
    return true;
  };

  while (static_cast<int>(records_.size()) > params_.max_frontiers) {
    if (evictOne(FrontierState::VISITED))     continue;
    if (evictOne(FrontierState::INVALIDATED)) continue;
    if (evictOne(FrontierState::DORMANT))     continue;
    // Only ACTIVE left and we're still over cap. Stop — we never evict ACTIVE.
    break;
  }
}
