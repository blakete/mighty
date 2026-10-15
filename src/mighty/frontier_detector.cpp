#include "mighty/frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

#include "mighty/visited_map.hpp"

namespace {

// 8-connected neighbor offsets.
constexpr int kDx8[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
constexpr int kDy8[8] = {-1, -1, -1,  0, 0,  1, 1, 1};

inline size_t flatIdx(int ix, int iy, int width) {
  return static_cast<size_t>(iy) * width + ix;
}

// Spiral search outward from (cx, cy) for the nearest walkable cell within
// `max_radius_cells` (FREE, or flagged in `passable` when given). Returns true
// on success and writes the result into (*ox, *oy). Naive O(R²) scan — fine
// for the small radii we use here.
bool snapToFree(const OccGrid2D& grid, const std::vector<uint8_t>* passable,
                int cx, int cy, int max_radius_cells, int* ox, int* oy) {
  auto walkable = [&](int x, int y) {
    if (grid.isFree(x, y)) return true;
    return passable != nullptr && grid.inBounds(x, y) &&
           (*passable)[flatIdx(x, y, grid.width())] != 0;
  };
  if (walkable(cx, cy)) {
    *ox = cx;
    *oy = cy;
    return true;
  }
  for (int r = 1; r <= max_radius_cells; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        // Only consider cells on the ring of radius r.
        if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
        int nx = cx + dx;
        int ny = cy + dy;
        if (walkable(nx, ny)) {
          *ox = nx;
          *oy = ny;
          return true;
        }
      }
    }
  }
  return false;
}


// Cluster from grid cell indices (in BFS order): world cells, centroid, AABB and the goal (the member
// cell nearest the centroid; first such cell on ties).
FrontierCluster makeCluster(const OccGrid2D& grid, const std::vector<size_t>& members) {
  FrontierCluster cluster;
  const int W = grid.width();
  double sum_wx = 0.0, sum_wy = 0.0;
  double min_wx = std::numeric_limits<double>::infinity();
  double min_wy = std::numeric_limits<double>::infinity();
  double max_wx = -std::numeric_limits<double>::infinity();
  double max_wy = -std::numeric_limits<double>::infinity();
  cluster.cells.reserve(members.size());
  for (const size_t idx : members) {
    double wx, wy;
    grid.gridToWorld(static_cast<int>(idx % static_cast<size_t>(W)),
                     static_cast<int>(idx / static_cast<size_t>(W)), wx, wy);
    cluster.cells.emplace_back(wx, wy);
    sum_wx += wx;
    sum_wy += wy;
    min_wx = std::min(min_wx, wx);
    min_wy = std::min(min_wy, wy);
    max_wx = std::max(max_wx, wx);
    max_wy = std::max(max_wy, wy);
  }
  cluster.size_cells = static_cast<int>(cluster.cells.size());
  cluster.size_m2 = cluster.size_cells * grid.resolution() * grid.resolution();
  cluster.centroid = Eigen::Vector2d(sum_wx / cluster.size_cells, sum_wy / cluster.size_cells);
  cluster.aabb_min = Eigen::Vector2d(min_wx, min_wy);
  cluster.aabb_max = Eigen::Vector2d(max_wx, max_wy);
  double best = std::numeric_limits<double>::infinity();
  for (const auto& c : cluster.cells) {
    const double d2 = (c - cluster.centroid).squaredNorm();
    if (d2 < best) {
      best = d2;
      cluster.goal = c;
    }
  }
  return cluster;
}

// Emit one connected cluster (grid indices in BFS order), split along world-anchored tiles of side
// tile_m when its extent exceeds tile_m (see FrontierDetectorParams::max_cluster_extent_m).
void emitCluster(const OccGrid2D& grid, const std::vector<size_t>& members, double tile_m,
                 int min_cells, std::vector<FrontierCluster>& out) {
  FrontierCluster whole = makeCluster(grid, members);
  const Eigen::Vector2d ext = whole.aabb_max - whole.aabb_min;
  if (tile_m <= 0.0 || std::max(ext.x(), ext.y()) <= tile_m + 1e-9) {
    out.push_back(std::move(whole));
    return;
  }

  const int W = grid.width();
  const size_t M = members.size();
  std::unordered_map<size_t, size_t> pos;  // grid index -> position in `members`
  pos.reserve(M * 2);
  for (size_t i = 0; i < M; ++i) pos.emplace(members[i], i);
  std::vector<std::pair<long, long>> tile(M);
  for (size_t i = 0; i < M; ++i) {
    tile[i] = {static_cast<long>(std::floor(whole.cells[i].x() / tile_m)),
               static_cast<long>(std::floor(whole.cells[i].y() / tile_m))};
  }
  auto neighbours = [&](size_t i, auto&& fn) {
    const int cx = static_cast<int>(members[i] % static_cast<size_t>(W));
    const int cy = static_cast<int>(members[i] / static_cast<size_t>(W));
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kDx8[k];
      const int ny = cy + kDy8[k];
      if (!grid.inBounds(nx, ny)) continue;
      const auto it = pos.find(flatIdx(nx, ny, W));
      if (it != pos.end()) fn(it->second);
    }
  };

  // Pieces: 8-connected components inside one tile.
  constexpr size_t kNone = std::numeric_limits<size_t>::max();
  std::vector<size_t> label(M, kNone);
  std::vector<std::vector<size_t>> pieces;
  for (size_t s = 0; s < M; ++s) {
    if (label[s] != kNone) continue;
    const size_t id = pieces.size();
    pieces.emplace_back();
    std::queue<size_t> q;
    q.push(s);
    label[s] = id;
    while (!q.empty()) {
      const size_t i = q.front();
      q.pop();
      pieces[id].push_back(i);
      neighbours(i, [&](size_t j) {
        if (label[j] != kNone || tile[j] != tile[i]) return;
        label[j] = id;
        q.push(j);
      });
    }
  }

  // Merge pieces below min_cells (tile-edge slivers) into their largest 8-adjacent piece, smallest
  // first. The cluster is connected, so every piece has a neighbour while more than one is left.
  while (true) {
    size_t small = kNone;
    for (size_t p = 0; p < pieces.size(); ++p) {
      if (pieces[p].empty() || static_cast<int>(pieces[p].size()) >= min_cells) continue;
      if (small == kNone || pieces[p].size() < pieces[small].size()) small = p;
    }
    if (small == kNone) break;
    size_t into = kNone;
    for (const size_t i : pieces[small]) {
      neighbours(i, [&](size_t j) {
        const size_t q = label[j];
        if (q == small) return;
        if (into == kNone || pieces[q].size() > pieces[into].size() ||
            (pieces[q].size() == pieces[into].size() && q < into))
          into = q;
      });
    }
    if (into == kNone) break;  // single piece left
    for (const size_t i : pieces[small]) label[i] = into;
    pieces[into].insert(pieces[into].end(), pieces[small].begin(), pieces[small].end());
    pieces[small].clear();
  }

  for (auto& piece : pieces) {
    if (piece.empty()) continue;
    std::sort(piece.begin(), piece.end());  // BFS order of the whole cluster
    std::vector<size_t> idx;
    idx.reserve(piece.size());
    for (const size_t i : piece) idx.push_back(members[i]);
    out.push_back(makeCluster(grid, idx));
  }
}

}  // namespace

std::vector<FrontierCluster> FrontierDetector::detect(
    const OccGrid2D& grid, const Eigen::Vector2d& robot_xy,
    const VisitedMap* visited_map,
    const std::vector<uint8_t>* passable) const {
  std::vector<FrontierCluster> out;

  const int W = grid.width();
  const int H = grid.height();
  if (W <= 0 || H <= 0) return out;
  const size_t N = static_cast<size_t>(W) * H;
  if (passable != nullptr && passable->size() != N) passable = nullptr;  // dim mismatch: ignore

  // ---- Snap robot pose to nearest walkable cell (FREE, or passable) ----
  int rx, ry;
  grid.worldToGrid(robot_xy.x(), robot_xy.y(), rx, ry);
  const int snap_cells = std::max(
      0, static_cast<int>(std::ceil(params_.robot_snap_radius_m
                                    / grid.resolution())));
  int seed_x = -1, seed_y = -1;
  if (!snapToFree(grid, passable, rx, ry, snap_cells, &seed_x, &seed_y)) {
    // Robot not in or near any free cell — bail out silently. Caller may log.
    return out;
  }

  // ---- Pass A: BFS over walkable cells, tag frontier seeds ----
  std::vector<uint8_t> visited(N, 0);
  std::vector<uint8_t> is_frontier(N, 0);

  auto onBorder = [&](int ix, int iy) {
    const int m = params_.border_margin_cells;
    return ix < m || ix >= W - m || iy < m || iy >= H - m;
  };

  std::queue<std::pair<int, int>> q;
  q.push({seed_x, seed_y});
  visited[flatIdx(seed_x, seed_y, W)] = 1;

  while (!q.empty()) {
    auto [cx, cy] = q.front();
    q.pop();

    // Tag as frontier seed if any 8-neighbor is *in-bounds* and UNKNOWN, and
    // this cell is not on the border ring. We deliberately do NOT count
    // out-of-bounds neighbors as unknown — OOB is "outside the mapper window"
    // and is handled by border_margin_cells, not by the per-cell tag check.
    // Without this restriction, edge cells of an otherwise-free grid would
    // all be flagged as frontiers because their OOB neighbors look unknown.
    //
    // VisitedMap filter: if the unknown neighbor is already in the persistent
    // visited bitmap, treat it as known — it just slid out of the local
    // window in a past observation. Without this check, the robot would
    // re-create frontiers along the seam every time it revisits an explored
    // area, because the sliding window re-blanks those cells to UNKNOWN.
    bool has_unknown_nbr = false;
    for (int i = 0; i < 8; ++i) {
      const int nx = cx + kDx8[i];
      const int ny = cy + kDy8[i];
      if (!grid.inBounds(nx, ny)) continue;
      if (!grid.isUnknown(nx, ny)) continue;
      if (visited_map != nullptr) {
        double wx_n, wy_n;
        grid.gridToWorld(nx, ny, wx_n, wy_n);
        if (visited_map->isVisitedWorld(wx_n, wy_n)) continue;
      }
      has_unknown_nbr = true;
      break;
    }
    // Only FREE cells can be frontier seeds; passable (non-FREE) cells are walked through but
    // never tagged.
    bool is_seed = has_unknown_nbr && !onBorder(cx, cy) && grid.isFree(cx, cy);

    // Reject frontier cells that have any OCCUPIED neighbor within
    // obstacle_clearance_cells. Frontiers hugging walls are usually noise
    // (wall-thickness slop, raycast endpoints) and not worth visiting.
    // NOTE: this only filters seed-tagging — the BFS expansion below still
    // runs through every free cell, so disqualified cells don't sever the
    // BFS frontier and the rest of the reachable region is still explored.
    if (is_seed && params_.obstacle_clearance_cells > 0) {
      const int rcells = params_.obstacle_clearance_cells;
      bool near_obstacle = false;
      for (int dy = -rcells; dy <= rcells && !near_obstacle; ++dy) {
        for (int dx = -rcells; dx <= rcells && !near_obstacle; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (!grid.inBounds(nx, ny)) continue;
          if (grid.isOccupied(nx, ny)) near_obstacle = true;
        }
      }
      if (near_obstacle) is_seed = false;
    }

    // Bounding-box filter: drop seeds outside the user-specified exploration
    // box. Uses world coords so it stays correct as the local mapper window
    // slides. Like obstacle_clearance_cells above, this only gates seed-
    // tagging; BFS expansion below still walks the entire reachable region.
    if (is_seed && params_.bounds_enabled) {
      double wx_seed, wy_seed;
      grid.gridToWorld(cx, cy, wx_seed, wy_seed);
      if (wx_seed < params_.bounds_min_x || wx_seed > params_.bounds_max_x ||
          wy_seed < params_.bounds_min_y || wy_seed > params_.bounds_max_y) {
        is_seed = false;
      }
    }

    if (is_seed) is_frontier[flatIdx(cx, cy, W)] = 1;

    // Expand BFS through FREE cells, plus cells flagged in `passable` when given.
    for (int i = 0; i < 8; ++i) {
      const int nx = cx + kDx8[i];
      const int ny = cy + kDy8[i];
      if (!grid.inBounds(nx, ny)) continue;
      const size_t nidx = flatIdx(nx, ny, W);
      if (visited[nidx]) continue;
      if (!grid.isFree(nx, ny) && !(passable != nullptr && (*passable)[nidx])) continue;
      visited[nidx] = 1;
      q.push({nx, ny});
    }
  }

  // ---- Pass B: BFS over frontier seeds to form clusters ----
  std::vector<uint8_t> assigned(N, 0);
  for (int sy = 0; sy < H; ++sy) {
    for (int sx = 0; sx < W; ++sx) {
      const size_t sidx = flatIdx(sx, sy, W);
      if (!is_frontier[sidx] || assigned[sidx]) continue;

      std::vector<size_t> members;
      std::queue<std::pair<int, int>> cq;
      cq.push({sx, sy});
      assigned[sidx] = 1;

      while (!cq.empty()) {
        auto [cx, cy] = cq.front();
        cq.pop();
        members.push_back(flatIdx(cx, cy, W));

        for (int i = 0; i < 8; ++i) {
          const int nx = cx + kDx8[i];
          const int ny = cy + kDy8[i];
          if (!grid.inBounds(nx, ny)) continue;
          const size_t nidx = flatIdx(nx, ny, W);
          if (assigned[nidx]) continue;
          if (!is_frontier[nidx]) continue;
          assigned[nidx] = 1;
          cq.push({nx, ny});
        }
      }

      if (static_cast<int>(members.size()) < params_.cluster_min_cells) continue;
      emitCluster(grid, members, params_.max_cluster_extent_m, params_.cluster_min_cells, out);
    }
  }

  // Sort by descending size for stable downstream behavior.
  std::sort(out.begin(), out.end(),
            [](const FrontierCluster& a, const FrontierCluster& b) {
              return a.size_cells > b.size_cells;
            });

  return out;
}
