// rasterizeGoalFence (goal_fence.cpp) classifies every cell centre of a window with the arithmetic of
// goalFenceAllows: forbidden = not inside keep_in or inside a keep_out, band = allowed but closer than
// band_m to an edge, allowed = the core. The planner's keep-out (MapUtil::applyKeepOut2D) relies on the
// two agreeing exactly, so the selector's goals and the planner's map use the same fence.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "goal_selector/goal_fence.hpp"

using goal_selector::FencePoint;
using goal_selector::GoalFence;
using goal_selector::goalFenceAllows;
using goal_selector::kFenceAllowed;
using goal_selector::kFenceBand;
using goal_selector::kFenceForbidden;
using goal_selector::rasterizeGoalFence;

namespace {

constexpr double kRes = 0.1;
constexpr int kW = 140;
constexpr int kH = 120;
constexpr double kOx = -2.03;  // off-lattice origin: edges never sit exactly on cell centres by design
constexpr double kOy = -1.97;

std::vector<FencePoint> Box(double x0, double x1, double y0, double y1) {
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

GoalFence Ready(std::vector<FencePoint> keep_in, std::vector<std::vector<FencePoint>> keep_out = {}) {
  GoalFence f;
  f.active = f.ready = true;
  f.keep_in = std::move(keep_in);
  f.keep_out = std::move(keep_out);
  return f;
}

/** Every cell centre: raster class == the per-point rule. Returns the counts {allowed, band, forbidden}. */
std::vector<int> ExpectMatchesPointRule(const GoalFence& f, double band_m) {
  std::vector<uint8_t> cls;
  rasterizeGoalFence(f, kW, kH, kRes, kOx, kOy, band_m, cls);
  EXPECT_EQ(cls.size(), static_cast<size_t>(kW) * kH);
  GoalFence inside = f;  // offset 0: goalFenceAllows == inside keep_in and outside every keep_out
  inside.offset_m = 0.0;
  GoalFence core = f;
  core.offset_m = band_m;
  std::vector<int> counts(3, 0);
  for (int y = 0; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      const double cx = kOx + (x + 0.5) * kRes, cy = kOy + (y + 0.5) * kRes;
      const uint8_t want = !goalFenceAllows(inside, cx, cy) ? kFenceForbidden
                           : !goalFenceAllows(core, cx, cy) ? kFenceBand
                                                            : kFenceAllowed;
      const uint8_t got = cls[static_cast<size_t>(x) + static_cast<size_t>(kW) * y];
      EXPECT_EQ(got, want) << "cell (" << x << ", " << y << ") centre (" << cx << ", " << cy << ")";
      ++counts[got];
    }
  }
  return counts;
}

}  // namespace

TEST(GoalFenceRaster, SquareWithHole) {
  const auto c = ExpectMatchesPointRule(Ready(Box(0.0, 10.0, 0.0, 8.0), {Box(4.0, 6.0, 3.0, 5.0)}), 0.36);
  EXPECT_GT(c[kFenceAllowed], 0);
  EXPECT_GT(c[kFenceBand], 0);
  EXPECT_GT(c[kFenceForbidden], 0);
}

TEST(GoalFenceRaster, KeepOutCrossingTheKeepInEdgeAndOverlapping) {
  ExpectMatchesPointRule(Ready(Box(0.0, 10.0, 0.0, 8.0),
                               {Box(8.5, 12.0, 2.0, 4.0), Box(3.0, 5.0, 3.0, 6.0), Box(4.0, 6.5, 5.0, 7.0)}),
                         0.36);
}

TEST(GoalFenceRaster, ConcaveL) {
  ExpectMatchesPointRule(Ready({{0, 0}, {10, 0}, {10, 3}, {4, 3}, {4, 8}, {0, 8}}), 0.36);
}

TEST(GoalFenceRaster, RotatedPolygonAndSlit) {
  // a rotated square (no axis-aligned edges) with a keep_out slit narrower than 2 * band
  const double c = std::cos(0.4), s = std::sin(0.4);
  std::vector<FencePoint> sq;
  for (auto [x, y] : std::vector<std::pair<double, double>>{{-4, -4}, {4, -4}, {4, 4}, {-4, 4}})
    sq.push_back({5.0 + c * x - s * y, 4.0 + s * x + c * y});
  ExpectMatchesPointRule(Ready(sq, {Box(4.9, 5.3, 1.0, 7.0)}), 0.36);
}

TEST(GoalFenceRaster, WindowPartlyOutsideKeepInAndZeroBand) {
  const auto c = ExpectMatchesPointRule(Ready(Box(3.0, 20.0, 2.0, 20.0)), 0.0);
  EXPECT_EQ(c[kFenceBand], 0) << "band_m 0: no band";
  EXPECT_GT(c[kFenceForbidden], 0);
}

TEST(GoalFenceRaster, NotReadyIsAllAllowed) {
  GoalFence f = Ready(Box(0.0, 1.0, 0.0, 1.0));
  f.ready = false;
  std::vector<uint8_t> cls;
  rasterizeGoalFence(f, kW, kH, kRes, kOx, kOy, 0.36, cls);
  ASSERT_EQ(cls.size(), static_cast<size_t>(kW) * kH);
  for (uint8_t v : cls) ASSERT_EQ(v, kFenceAllowed);
}
