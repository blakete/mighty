#include "map2d/map2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace map2d {

// ----------------------------------------------------------------------------
// Window geometry
// ----------------------------------------------------------------------------

int windowCells(double wd, double res) { return std::max(1, (int)std::lround(wd / res)); }

WindowGeometry windowGeometry(double center_x, double center_y, int cells_x, int cells_y,
                              double res, double inflation_hgp) {
  // Compute X/Y dims with inflation pad
  int pad = int(std::ceil(5.0 * inflation_hgp / res));
  WindowGeometry w;
  w.dim_x = cells_x + pad;
  w.dim_y = cells_y + pad;
  // Origin (global coords of cell (0,0)), snapped to the resolution grid
  w.origin_x = std::floor((center_x - (w.dim_x * res) / 2.0f) / res) * res;
  w.origin_y = std::floor((center_y - (w.dim_y * res) / 2.0f) / res) * res;
  return w;
}

// ----------------------------------------------------------------------------
// Occupancy rasterisation
// ----------------------------------------------------------------------------

bool sourceRangeForCell(const SourceGrid& src, double wx, double wy, double half, SourceRange& r) {
  const double inv_occ_res = src.inv_resolution;
  // Range of underlying occ-grid cells overlapping this cell.
  // kEdgeEps (in source-cell units) makes the edge arithmetic deterministic: planner and mapper
  // lattices are typically aligned, so an edge lands exactly on a source-cell boundary and float
  // noise (20.9999 vs 21.0000) would otherwise decide which cells are sampled.
  constexpr double kEdgeEps = 1e-6;
  const double lo_x = (wx - half - src.origin_x) * inv_occ_res;
  const double hi_x = (wx + half - src.origin_x) * inv_occ_res;
  const double lo_y = (wy - half - src.origin_y) * inv_occ_res;
  const double hi_y = (wy + half - src.origin_y) * inv_occ_res;
  int ox_min = static_cast<int>(std::floor(lo_x + kEdgeEps));
  int ox_max = static_cast<int>(std::floor(hi_x + kEdgeEps));
  int oy_min = static_cast<int>(std::floor(lo_y + kEdgeEps));
  int oy_max = static_cast<int>(std::floor(hi_y + kEdgeEps));

  // The inclusive [ox_min, ox_max] range deliberately takes in the next source cell when this
  // cell's upper edge lands exactly on a source-cell boundary (aligned lattices => a 2x2 block):
  // a one-cell conservative inflation for OCCUPIED. FREE vs UNKNOWN must not inherit that bias,
  // so that decision uses the strict half-open overlap below.
  const int ox_max_strict = static_cast<int>(std::floor(hi_x - kEdgeEps));
  const int oy_max_strict = static_cast<int>(std::floor(hi_y - kEdgeEps));
  ox_min = std::max(ox_min, 0);
  oy_min = std::max(oy_min, 0);
  ox_max = std::min(ox_max, src.width - 1);
  oy_max = std::min(oy_max, src.height - 1);
  if (ox_min > ox_max || oy_min > oy_max) return false;
  r.ox_min = ox_min;
  r.ox_max = ox_max;
  r.oy_min = oy_min;
  r.oy_max = oy_max;
  r.ox_max_strict = ox_max_strict;
  r.oy_max_strict = oy_max_strict;
  return true;
}

void buildFromOccupancy(const SourceGrid& src, int dim_x, int dim_y, double res, double origin_x,
                        double origin_y, std::vector<int8_t>& values) {
  const size_t n2d = static_cast<size_t>(dim_x) * dim_y;

  // TRI-STATE build. Start UNKNOWN; a cell becomes OCCUPIED if any source cell under it is
  // occupied, FREE if any source cell under it is known-free, and stays UNKNOWN when the source
  // reports unknown for everything under it or does not cover it at all.
  // Precedence: occupied > unknown > free.
  values.assign(n2d, kUnknown);

  const int occ_w = src.width;
  const std::vector<bool>& occ_data = *src.occupied;
  const std::vector<bool>& unk_data = *src.unknown;

  for (int y = 0; y < dim_y; ++y) {
    for (int x = 0; x < dim_x; ++x) {
      // Convert grid cell to world coordinates (cell extent is [wx-h, wx+h])
      const double wx = origin_x + (x + 0.5) * res;
      const double wy = origin_y + (y + 0.5) * res;
      const double half = 0.5 * res;
      const size_t idx = static_cast<size_t>(x) + static_cast<size_t>(dim_x) * y;

      SourceRange r;
      if (!sourceRangeForCell(src, wx, wy, half, r)) continue;  // no overlap: stays UNKNOWN

      // Occupied if ANY underlying source cell is occupied. Otherwise FREE if any source cell is
      // known-free (strict overlap), else UNKNOWN.
      bool any_occ = false;
      bool any_free = false;
      for (int oy = r.oy_min; oy <= r.oy_max && !any_occ; ++oy) {
        for (int ox = r.ox_min; ox <= r.ox_max; ++ox) {
          const size_t oidx = static_cast<size_t>(oy) * occ_w + ox;
          if (occ_data[oidx]) {
            any_occ = true;
            break;
          }
          if (!unk_data[oidx] && ox <= r.ox_max_strict && oy <= r.oy_max_strict) any_free = true;
        }
      }

      if (any_occ) {
        values[idx] = kOccupied;
      } else if (any_free) {
        values[idx] = kFree;  // else: stays kUnknown
      }
    }
  }
}

// ----------------------------------------------------------------------------
// Inflation
// ----------------------------------------------------------------------------

void inflate(std::vector<int8_t>& values, std::vector<uint8_t>& inflated_only, int dim_x,
             int dim_y, double res, float inflation_m) {
  const int dimX = dim_x;
  const int dimY = dim_y;
  const size_t n2d = static_cast<size_t>(dimX) * dimY;
  inflated_only.assign(n2d, 0);
  if (inflation_m <= 0.0f || res <= 0.0 || values.size() != n2d) return;

  const double r_cells = inflation_m / res;
  const int r = static_cast<int>(std::floor(r_cells + 1e-6));
  if (r < 1) return;
  const double r2 = r_cells * r_cells + 1e-6;
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if ((dx != 0 || dy != 0) && dx * dx + dy * dy <= r2) offsets.emplace_back(dx, dy);
    }
  }

  // Seed only from occupied cells with a non-occupied 4-neighbour: the nearest occupied cell to
  // any non-occupied cell is always one of these, so skipping interior cells gives the same
  // result for a fraction of the work.
  std::vector<size_t> seeds;
  for (int y = 0; y < dimY; ++y) {
    for (int x = 0; x < dimX; ++x) {
      const size_t idx = static_cast<size_t>(x) + static_cast<size_t>(dimX) * y;
      if (values[idx] != kOccupied) continue;
      if ((x > 0 && values[idx - 1] != kOccupied) ||
          (x + 1 < dimX && values[idx + 1] != kOccupied) ||
          (y > 0 && values[idx - dimX] != kOccupied) ||
          (y + 1 < dimY && values[idx + dimX] != kOccupied)) {
        seeds.push_back(idx);
      }
    }
  }

  for (const size_t s : seeds) {
    const int sx = static_cast<int>(s % static_cast<size_t>(dimX));
    const int sy = static_cast<int>(s / static_cast<size_t>(dimX));
    for (const auto& o : offsets) {
      const int nx = sx + o.first;
      const int ny = sy + o.second;
      if (nx < 0 || nx >= dimX || ny < 0 || ny >= dimY) continue;
      const size_t n = static_cast<size_t>(nx) + static_cast<size_t>(dimX) * ny;
      if (values[n] == kOccupied) continue;
      values[n] = kOccupied;
      inflated_only[n] = 1;
    }
  }
}

// ----------------------------------------------------------------------------
// Queries
// ----------------------------------------------------------------------------

bool findClosestNonOccupied2DPoint(const GridView& g, const Eigen::Vector3d& point,
                                   Eigen::Vector3d& closest_non_occupied_point) {
  closest_non_occupied_point = point;

  if (g.empty()) return false;

  const int dimX = g.dim_x;
  const int dimY = g.dim_y;
  const Eigen::Vector2d origin(g.origin_x, g.origin_y);
  const float res = static_cast<float>(g.res);
  if (res <= 0.0f) return false;

  const int cx = static_cast<int>(std::floor((point.x() - origin(0)) / res));
  const int cy = static_cast<int>(std::floor((point.y() - origin(1)) / res));

  // Already non-occupied (free or unknown): nothing to do.
  if (!is2DOccupied(g, cx, cy)) return true;

  // Expanding-radius (in cells) search, ~5 m cap.
  const int max_radius_cells = static_cast<int>(std::ceil(5.0f / res));
  for (int r = 1; r <= max_radius_cells; ++r) {
    float min_dist = std::numeric_limits<float>::max();
    bool found = false;
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        // Only the outer ring of this radius -- interior cells were already checked (and would
        // have returned) at a smaller r.
        if (dx != -r && dx != r && dy != -r && dy != r) continue;
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || nx >= dimX || ny < 0 || ny >= dimY) continue;
        if (is2DOccupied(g, nx, ny)) continue;  // still occupied

        const float wx = origin(0) + (nx + 0.5f) * res;
        const float wy = origin(1) + (ny + 0.5f) * res;
        const Eigen::Vector3d candidate(wx, wy, static_cast<float>(point.z()));
        const float dist = (candidate - point).norm();
        if (dist < min_dist) {
          min_dist = dist;
          closest_non_occupied_point = candidate;
          found = true;
        }
      }
    }
    if (found) return true;
  }
  return false;  // nothing non-occupied found within the search radius
}

bool isClearOfOccupied2D(const GridView& g, const Eigen::Vector3d& point, double clearance_m) {
  if (g.empty()) return true;

  const double res = g.res;
  if (res <= 0.0 || clearance_m <= 0.0) return true;

  const int cx = static_cast<int>(std::floor((point.x() - g.origin_x) / res));
  const int cy = static_cast<int>(std::floor((point.y() - g.origin_y) / res));

  // Disk scan in cell units via is2DOccupied() specifically -- NOT "value != free", which also
  // catches UNKNOWN and would reject goals that are merely near unmapped space.
  const int r = std::max(1, static_cast<int>(std::ceil(clearance_m / res)));
  const int r2 = r * r;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r2) continue;
      if (is2DOccupied(g, cx + dx, cy + dy)) return false;
    }
  }
  return true;
}

// ----------------------------------------------------------------------------
// Goal relocation
// ----------------------------------------------------------------------------

namespace {
// Planner's checkIfPointOccupied2D: point -> cell -> is2DOccupied (false if no usable map).
bool pointOccupied2D(const GridView& g, const Eigen::Vector3d& point) {
  if (g.empty()) return false;
  const double res = g.res;
  if (res <= 0.0) return false;
  const int x = static_cast<int>(std::floor((point.x() - g.origin_x) / res));
  const int y = static_cast<int>(std::floor((point.y() - g.origin_y) / res));
  return is2DOccupied(g, x, y);
}
}  // namespace

SanitizeResult sanitizeGoal2D(const GridView& g, const Eigen::Vector3d& goal, double clearance,
                              double step) {
  SanitizeResult out;
  out.goal = goal;

  // Map not initialized yet: keep the goal.
  if (g.empty()) return out;

  // Already free or unknown: nothing to do.
  if (!pointOccupied2D(g, goal)) return out;

  // Expanding-radius search for the closest free/unknown 2D cell.
  Eigen::Vector3d closest;
  bool found = findClosestNonOccupied2DPoint(g, goal, closest);

  if (!found || (closest - goal).norm() < 1e-6) {
    out.outcome = GoalOutcome::kDropped;
    out.drop_reason = DropReason::kNoFreeCell;
    return out;
  }

  // Direction outward from the occupied goal toward free/unknown space.
  const Eigen::Vector3d dir = (closest - goal).normalized();

  // `closest` is the nearest cell that is not occupied in the (inflated) map, so it already has
  // the inflation clearance. `clearance` adds standoff beyond that.
  Eigen::Vector3d new_pos = closest + dir * clearance;

  // Safety loop: keep walking outward by one step until a point that is ACTUALLY clear (disk of
  // radius `clearance`) is found.
  const int max_iters = std::max(20, static_cast<int>(std::ceil(3.0 * clearance / step)) + 10);
  out.max_iters = max_iters;
  int i = 0;
  for (; i < max_iters &&
         (pointOccupied2D(g, new_pos) || !isClearOfOccupied2D(g, new_pos, clearance));
       ++i) {
    new_pos += dir * step;
  }

  if (pointOccupied2D(g, new_pos) || !isClearOfOccupied2D(g, new_pos, clearance)) {
    out.outcome = GoalOutcome::kDropped;
    out.drop_reason = DropReason::kNoClearance;
    return out;
  }

  out.outcome = GoalOutcome::kRelocated;
  out.goal = new_pos;
  return out;
}

}  // namespace map2d
