/**
 * @file map2d.hpp
 * @brief ROS-free 2D tri-state grid library shared by the planner (MapUtil) and the goal selector.
 *
 * Dependencies: standard library + Eigen only. Nothing here includes mighty/, hgp/, ROS or PCL, so
 * the library can be moved to its own package.
 *
 * Grid convention: row-major, index = x + dim_x * y, cell (x, y) covers
 * [origin + x*res, origin + (x+1)*res). Values are kFree (0), kUnknown (-1), kOccupied (100).
 *
 * The functions are a strict extraction of the 2D code that used to live in hgp/map_util.hpp
 * (MapUtil) and the goal-relocation code in the planner; arithmetic types and expression order are
 * deliberately unchanged so results are bit-identical.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace map2d {

constexpr int8_t kOccupied = 100;
constexpr int8_t kFree = 0;
constexpr int8_t kUnknown = -1;

// ----------------------------------------------------------------------------
// Grid types
// ----------------------------------------------------------------------------

/** @brief Non-owning read-only view of a tri-state grid. Empty (values == nullptr or non-positive
 *  dims) means "no map". */
struct GridView {
  const int8_t* values = nullptr;
  int dim_x = 0;
  int dim_y = 0;
  double res = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;

  bool empty() const { return values == nullptr || dim_x <= 0 || dim_y <= 0; }
};

/** @brief Owning grid: tri-state values plus the inflated-only mask (1 = occupied only because of
 *  inflate2DMap). Convenience for callers (e.g. the selector) that do not already own storage. */
struct Grid2D {
  int dim_x = 0;
  int dim_y = 0;
  double res = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> values;
  std::vector<uint8_t> inflated_only;

  GridView view() const {
    GridView v;
    v.values = values.empty() ? nullptr : values.data();
    v.dim_x = dim_x;
    v.dim_y = dim_y;
    v.res = res;
    v.origin_x = origin_x;
    v.origin_y = origin_y;
    return v;
  }
};

/** @brief Plain description of a source occupancy grid (e.g. the mapper's 2D occupancy). The
 *  vectors are borrowed, not copied. */
struct SourceGrid {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double inv_resolution = 0.0;  // supplied by the caller so it matches the source bit for bit
  double origin_x = 0.0;
  double origin_y = 0.0;
  const std::vector<bool>* occupied = nullptr;  // size width*height
  const std::vector<bool>* unknown = nullptr;   // size width*height
};

// ----------------------------------------------------------------------------
// Window geometry (extracted from MapUtil::readMap x/y part)
// ----------------------------------------------------------------------------

struct WindowGeometry {
  int dim_x = 0;
  int dim_y = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
};

/** @brief Cell count for a window extent: max(1, lround(wd / res)). */
int windowCells(double wd, double res);

/** @brief X/Y dims (with inflation pad) and snapped origin from the window centre. */
WindowGeometry windowGeometry(double center_x, double center_y, int cells_x, int cells_y,
                              double res, double inflation_hgp);

/** @brief floor((p - origin) / res) as int. */
inline int worldToCell(double p, double origin, double res) {
  return static_cast<int>(std::floor((p - origin) / res));
}

// ----------------------------------------------------------------------------
// Occupancy rasterisation (extracted from MapUtil::buildMap2DFromOcc2D, occupancy part)
// ----------------------------------------------------------------------------

/** @brief Source-cell range overlapping one destination cell. */
struct SourceRange {
  int ox_min, ox_max, oy_min, oy_max;  // inclusive, clipped to the source grid
  int ox_max_strict, oy_max_strict;    // strict half-open upper bounds (unclipped)
};

/** @brief Source cells overlapping the destination cell centred at (wx, wy) with half-width half.
 *  Returns false if there is no overlap (the cell stays unknown). */
bool sourceRangeForCell(const SourceGrid& src, double wx, double wy, double half, SourceRange& r);

/** @brief Fill `values` (resized to dim_x*dim_y, initialised unknown) with the tri-state window
 *  grid. Precedence: occupied > unknown > free; uncovered cells stay unknown. */
void buildFromOccupancy(const SourceGrid& src, int dim_x, int dim_y, double res, double origin_x,
                        double origin_y, std::vector<int8_t>& values);

// ----------------------------------------------------------------------------
// Inflation (extracted from MapUtil::inflate2DMap)
// ----------------------------------------------------------------------------

/** @brief Mark every cell within inflation_m of an occupied cell as occupied; `inflated_only` is
 *  reset to dim_x*dim_y zeros and set to 1 for cells set only by this call. */
void inflate(std::vector<int8_t>& values, std::vector<uint8_t>& inflated_only, int dim_x,
             int dim_y, double res, float inflation_m);

inline void inflate(Grid2D& g, float inflation_m) {
  inflate(g.values, g.inflated_only, g.dim_x, g.dim_y, g.res, inflation_m);
}

/** @brief Unknown band: `band` is resized to dim_x*dim_y and set to 1 for every NON-unknown cell
 *  within radius_m of an unknown cell (cell centre to cell centre, same distance convention as
 *  inflate), 0 elsewhere (unknown cells themselves are 0). `values` is not modified and nothing is
 *  marked occupied. radius_m <= 0 (or less than one cell) gives an all-zero band. */
void inflateUnknown(const std::vector<int8_t>& values, int dim_x, int dim_y, double res,
                    float radius_m, std::vector<uint8_t>& band);

// ----------------------------------------------------------------------------
// Queries
// ----------------------------------------------------------------------------

/** @brief True if the cell is occupied. Out of bounds counts as occupied. */
inline bool is2DOccupied(const GridView& g, int x, int y) {
  if (x < 0 || x >= g.dim_x || y < 0 || y >= g.dim_y) return true;
  return g.values[static_cast<size_t>(x) + static_cast<size_t>(g.dim_x) * y] == kOccupied;
}

/** @brief Nearest non-occupied cell centre to `point` (expanding ring search, ~5 m cap). z is
 *  passed through unchanged. Returns false if the grid is empty, res <= 0, or nothing is found;
 *  `closest` is then set to `point`. */
bool findClosestNonOccupied2DPoint(const GridView& g, const Eigen::Vector3d& point,
                                   Eigen::Vector3d& closest);

/** @brief True if no occupied cell lies within a disk of clearance_m around `point`
 *  (out of bounds counts as occupied). True also for an empty grid, res <= 0, clearance <= 0. */
bool isClearOfOccupied2D(const GridView& g, const Eigen::Vector3d& point, double clearance_m);

// ----------------------------------------------------------------------------
// Goal relocation (extracted from MIGHTY::sanitizeTerminalGoal2D)
// ----------------------------------------------------------------------------

enum class GoalOutcome { kUnchanged, kRelocated, kDropped };
enum class DropReason { kNone, kNoFreeCell, kNoClearance };

struct SanitizeResult {
  GoalOutcome outcome = GoalOutcome::kUnchanged;
  DropReason drop_reason = DropReason::kNone;
  Eigen::Vector3d goal;  // relocated position if kRelocated, else the input goal
  int max_iters = 0;     // push-out iteration cap (for logging)
};

/** @brief Relocate an occupied/inflated goal outward to a point with `clearance` standoff.
 *  `step` is the push-out step in metres (caller resolves any fallback; requires step > 0).
 *  Empty grid => unchanged. */
SanitizeResult sanitizeGoal2D(const GridView& g, const Eigen::Vector3d& goal, double clearance,
                              double step);

}  // namespace map2d
