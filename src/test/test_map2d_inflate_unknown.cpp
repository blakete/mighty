// Tests for map2d::inflateUnknown (the "unknown band" used by the selector's frontier map).
//
// Source of truth
//   * goal_selector_refactor_spec.md, "New Plan: detailed design" N1: "`map2d` gets an unknown inflation
//     (`inflateUnknown`) that records an unknown-band mask. It does not mark cells occupied. Occupied
//     inflation is unchanged."
//   * goal_selector_test_ideas.md, section C: "`map2d::inflateUnknown`: never changes the input grid; radius 0
//     gives no band; grid borders handled; same distance convention as `map2d::inflate`."
//   * map2d.hpp contract: band[i] = 1 for every NON-unknown cell within radius_m of an unknown cell (cell
//     centre to cell centre); unknown cells themselves are 0.
//
// Tests (all on tiny synthetic grids, row-major, index = x + dim_x * y, y = 0 is the FIRST row of the ASCII
// art; legend: '.' free, '#' occupied, '?' unknown, 'B' = expected band cell):
//
//   InputGridIsNotModified             -> notes C "never changes the input grid"
//   ZeroOrNegativeRadiusGivesEmptyBand -> notes C "radius 0 gives no band"
//   RadiusSmallerThanOneCellGivesEmptyBand -> map2d.hpp "or less than one cell"
//   BandHasExactlyTheDiscAroundASingleUnknownCell -> N1 distance / disc shape (own brute-force oracle)
//   OccupiedCellsNextToUnknownAreBandCellsButUnknownCellsAreNot -> "band only marks non-unknown cells"
//   BorderUnknownCellBandIsClippedAndNothingIsMarkedOutside -> notes C "grid borders handled"
//   OutOfGridIsNotUnknown                -> border: no unknown inside => no band, even on the grid edge
//   BandMatchesBruteForceOracleOnIrregularGrid -> N1 definition over blocks / lines / holes
//   SameDistanceConventionAsInflate      -> notes C "same distance convention as inflate" (identical geometry)
//   OutputBufferIsOverwrittenNotAccumulated -> band is rebuilt on every call
//   NeverMarksValuesOccupied             -> N1 "It does not mark cells occupied"
//
// No ROS, no mighty/ headers: map2d only.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "map2d/map2d.hpp"

namespace {

using map2d::kFree;
using map2d::kOccupied;
using map2d::kUnknown;

/** Parse ASCII rows ('.' free, '#' occupied, '?' unknown) into a row-major grid. The first string is
 *  y = 0. All rows must have the same length. */
std::vector<int8_t> parse(const std::vector<std::string>& rows, int& w, int& h) {
  h = static_cast<int>(rows.size());
  w = static_cast<int>(rows[0].size());
  std::vector<int8_t> v(static_cast<size_t>(w) * h, kFree);
  for (int y = 0; y < h; ++y) {
    EXPECT_EQ(static_cast<int>(rows[y].size()), w);
    for (int x = 0; x < w; ++x) {
      const char c = rows[y][x];
      v[static_cast<size_t>(x) + static_cast<size_t>(w) * y] =
          (c == '#') ? kOccupied : (c == '?') ? kUnknown : kFree;
    }
  }
  return v;
}

/** Render a band as ASCII: 'B' band, '?' unknown input cell, '#' occupied input cell, '.' otherwise. */
std::vector<std::string> render(const std::vector<int8_t>& v, const std::vector<uint8_t>& band, int w,
                                int h) {
  std::vector<std::string> out(h, std::string(w, '.'));
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const size_t i = static_cast<size_t>(x) + static_cast<size_t>(w) * y;
      out[y][x] = band[i] ? 'B' : (v[i] == kUnknown ? '?' : (v[i] == kOccupied ? '#' : '.'));
    }
  return out;
}

/** Independent oracle: band[i] = 1 iff cell i is NOT unknown and some unknown cell lies within radius_m
 *  (Euclidean distance between cell centres = res * sqrt(dx^2 + dy^2), tiny tolerance for float noise).
 *  Brute force, no seed shortcuts, no offset tables. */
std::vector<uint8_t> oracleBand(const std::vector<int8_t>& v, int w, int h, double res,
                                double radius_m) {
  std::vector<uint8_t> band(static_cast<size_t>(w) * h, 0);
  if (radius_m <= 0.0) return band;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const size_t i = static_cast<size_t>(x) + static_cast<size_t>(w) * y;
      if (v[i] == kUnknown) continue;
      for (int uy = 0; uy < h && !band[i]; ++uy)
        for (int ux = 0; ux < w; ++ux) {
          if (v[static_cast<size_t>(ux) + static_cast<size_t>(w) * uy] != kUnknown) continue;
          const double d = res * std::hypot(double(ux - x), double(uy - y));
          if (d <= radius_m + 1e-6) {
            band[i] = 1;
            break;
          }
        }
    }
  return band;
}

}  // namespace

// Scenario: run inflateUnknown on a grid with unknown cells and occupied cells; compare the grid before/after.
// Expected: byte-identical. Why: notes C "never changes the input grid" (the parameter is const& and the band
// is a separate output).
TEST(InflateUnknown, InputGridIsNotModified) {
  int w, h;
  auto v = parse({"..........",
                  "..?.......",
                  "....#.....",
                  ".......??.",
                  ".........."},
                 w, h);
  const auto before = v;
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.3f, band);
  EXPECT_EQ(v, before);
  ASSERT_EQ(band.size(), v.size());
  // sanity: the call did something
  size_t n = 0;
  for (auto b : band) n += b;
  EXPECT_GT(n, 0u);
}

// Scenario: radius 0 (and negative) with plenty of unknown cells.
// Expected: band has the grid size and is all zero. Why: notes C "radius 0 gives no band".
TEST(InflateUnknown, ZeroOrNegativeRadiusGivesEmptyBand) {
  int w, h;
  auto v = parse({"....?....",
                  ".........",
                  "..???....",
                  "........."},
                 w, h);
  for (float r : {0.0f, -0.5f}) {
    std::vector<uint8_t> band(5, 1);  // stale content of the wrong size must not leak out
    map2d::inflateUnknown(v, w, h, 0.1, r, band);
    ASSERT_EQ(band.size(), v.size()) << "radius " << r;
    for (size_t i = 0; i < band.size(); ++i) EXPECT_EQ(band[i], 0) << "radius " << r << " cell " << i;
  }
}

// Scenario: positive radius that is smaller than one cell (0.05 m at 0.1 m resolution).
// Expected: empty band. Why: map2d.hpp: "radius_m <= 0 (or less than one cell) gives an all-zero band"
// (cell centres are a whole cell apart, so no neighbour can be within 0.05 m).
TEST(InflateUnknown, RadiusSmallerThanOneCellGivesEmptyBand) {
  int w, h;
  auto v = parse({".....",
                  "..?..",
                  "....."},
                 w, h);
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.05f, band);
  for (auto b : band) EXPECT_EQ(b, 0);
}

// Scenario: a single unknown cell in the middle of a free 9x9 grid, radius 0.2 m at 0.1 m (= 2 cells).
// Expected band = every other cell whose centre is within 2 cells of the unknown cell U (a small disc:
// (+-1,0),(0,+-1),(+-1,+-1),(+-2,0),(0,+-2); NOT (+-2,+-1), which is sqrt(5) = 2.24 cells away):
//
//        . . . . .          . . B . .
//        . . . . .          . B B B .
//        . . U . .   ->     B B U B B      (B = band; the 5x5 neighbourhood of U is shown)
//        . . . . .          . B B B .
//        . . . . .          . . B . .
//
TEST(InflateUnknown, BandHasExactlyTheDiscAroundASingleUnknownCell) {
  const int w = 9, h = 9;
  std::vector<int8_t> v(w * h, kFree);
  v[4 + w * 4] = kUnknown;
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.2f, band);

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const int dx = x - 4, dy = y - 4;
      const bool expect = (dx != 0 || dy != 0) && (dx * dx + dy * dy <= 4);
      EXPECT_EQ(band[x + w * y] != 0, expect) << "cell (" << x << "," << y << ")";
    }
  // The 12 cells of the radius-2 disc (without centre): 4 at distance 1, 4 at sqrt2, 4 at 2.
  size_t n = 0;
  for (auto b : band) n += b;
  EXPECT_EQ(n, 12u);
}

// Scenario: an occupied cell sits next to an unknown cell. Unknown cell at (2,1), occupied at (3,1).
//   .....
//   ..?#.
//   .....
// Expected: the occupied cell is a band cell (the band marks every NON-unknown cell, occupied included;
// the selector lets "occupied" win later), and the unknown cell itself is NOT in the band.
// Why: map2d.hpp "band ... set to 1 for every NON-unknown cell ... unknown cells themselves are 0".
TEST(InflateUnknown, OccupiedCellsNextToUnknownAreBandCellsButUnknownCellsAreNot) {
  int w, h;
  auto v = parse({".....",
                  "..?#.",
                  "....."},
                 w, h);
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.1f, band);  // radius 1 cell: 4-neighbours + diagonals (sqrt2 > 1 -> no)
  EXPECT_EQ(band[2 + w * 1], 0) << "the unknown cell itself must not be in the band";
  EXPECT_EQ(band[3 + w * 1], 1) << "occupied neighbour is a band cell";
  EXPECT_EQ(band[1 + w * 1], 1);
  EXPECT_EQ(band[2 + w * 0], 1);
  EXPECT_EQ(band[2 + w * 2], 1);
  EXPECT_EQ(band[3 + w * 0], 0) << "diagonal is sqrt(2) cells away: outside a radius of exactly 1 cell";
  // Every unknown cell has band 0 in general.
  for (size_t i = 0; i < v.size(); ++i)
    if (v[i] == kUnknown) EXPECT_EQ(band[i], 0);
}

// Scenario: unknown cells in the grid corner (0,0) and on the right border (x = w-1), radius 2 cells.
// Expected: the band is the disc clipped to the grid (no wrap-around to the opposite side, no write outside
// the vector), e.g. for the corner cell: (1,0),(2,0),(0,1),(0,2),(1,1) only.
// Why: notes C "grid borders handled".
TEST(InflateUnknown, BorderUnknownCellBandIsClippedAndNothingIsMarkedOutside) {
  int w, h;
  auto v = parse({"?.........",
                  "..........",
                  "..........",
                  "..........",
                  ".........?"},
                 w, h);
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.2f, band);
  ASSERT_EQ(band.size(), v.size());
  // corner (0,0)
  EXPECT_EQ(band[1 + w * 0], 1);
  EXPECT_EQ(band[2 + w * 0], 1);
  EXPECT_EQ(band[0 + w * 1], 1);
  EXPECT_EQ(band[0 + w * 2], 1);
  EXPECT_EQ(band[1 + w * 1], 1);
  EXPECT_EQ(band[3 + w * 0], 0);
  EXPECT_EQ(band[0 + w * 3], 0);
  EXPECT_EQ(band[2 + w * 1], 0) << "(2,1) is sqrt(5) cells away";
  // wrap-around check: the cell at the far end of row 0 / row 1 must not be marked because of (0,0)
  EXPECT_EQ(band[w - 1 + w * 0], 0);
  EXPECT_EQ(band[w - 1 + w * 1], 0);
  // right/bottom corner (9,4): clipped disc
  EXPECT_EQ(band[8 + w * 4], 1);
  EXPECT_EQ(band[7 + w * 4], 1);
  EXPECT_EQ(band[9 + w * 3], 1);
  EXPECT_EQ(band[9 + w * 2], 1);
  EXPECT_EQ(band[8 + w * 3], 1);
  EXPECT_EQ(band[0 + w * 4], 0) << "next row's start must not be marked (no index wrap-around)";
  // whole-grid oracle
  EXPECT_EQ(band, oracleBand(v, w, h, 0.1, 0.2));
}

// Scenario: grid with NO unknown cell inside (all free + a wall). Cells outside the grid are not "unknown
// cells of the grid".
// Expected: empty band, even right at the border. Why: border handling in N1 (the mapper window edge is
// handled by the detector's border_margin_cells, not by the band): inflateUnknown only looks at cells it has.
TEST(InflateUnknown, OutOfGridIsNotUnknown) {
  int w, h;
  auto v = parse({"..........",
                  "....#.....",
                  ".........."},
                 w, h);
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.5f, band);
  for (auto b : band) EXPECT_EQ(b, 0);
}

// Scenario: an irregular grid: a solid unknown block, a one-cell hole, a line of unknown, an occupied wall,
// at several radii and resolutions (including a non-0.1 resolution and a fractional cell radius).
// Expected: identical to the brute-force oracle (every non-unknown cell with an unknown cell within
// radius_m). Why: N1 definition of the unknown band; also checks that the "seed only boundary unknown cells"
// shortcut in the implementation loses nothing for the block interior.
TEST(InflateUnknown, BandMatchesBruteForceOracleOnIrregularGrid) {
  int w, h;
  auto v = parse({"..............................",
                  "..????????....................",
                  "..????????.........#..........",
                  "..????????.........#..........",
                  "..????????.........#...?......",
                  "..????????....................",
                  "..............................",
                  "..........?...................",
                  "..............................",
                  "....................??????????",
                  "....................??????????"},
                 w, h);
  struct Case { double res; float radius; };
  for (const Case c : {Case{0.1, 0.2f}, Case{0.1, 0.25f}, Case{0.1, 0.3f}, Case{0.1, 0.5f},
                       Case{0.15, 0.5f}, Case{0.2, 0.3f}}) {
    std::vector<uint8_t> band;
    map2d::inflateUnknown(v, w, h, c.res, c.radius, band);
    const auto expect = oracleBand(v, w, h, c.res, c.radius);
    EXPECT_EQ(band, expect) << "res " << c.res << " radius " << c.radius << "\n got:\n"
                            << [&] {
                                 std::string s;
                                 for (auto& r : render(v, band, w, h)) s += r + "\n";
                                 return s;
                               }();
  }
}

// Scenario: "identical geometry" comparison with map2d::inflate. Take a grid that contains only free and
// unknown cells, build a copy where every unknown cell is OCCUPIED, run map2d::inflate on the copy and
// inflateUnknown on the original.
// Expected: the cells inflate marks "inflated_only" are exactly the cells inflateUnknown marks as band (same
// radius, same disc convention incl. the fractional-cell radius cases 0.25 m, 0.15 m resolution).
// Why: notes C "same distance convention as map2d::inflate"; map2d.hpp "same distance convention as
// inflate".
TEST(InflateUnknown, SameDistanceConventionAsInflate) {
  int w, h;
  auto v = parse({"....................",
                  "..???...............",
                  "..???.........?.....",
                  "..???...............",
                  "....................",
                  "................??..",
                  "................??..",
                  "...................."},
                 w, h);
  struct Case { double res; float radius; };
  for (const Case c : {Case{0.1, 0.1f}, Case{0.1, 0.2f}, Case{0.1, 0.25f}, Case{0.1, 0.3f},
                       Case{0.1, 0.45f}, Case{0.15, 0.5f}, Case{0.05, 0.12f}}) {
    std::vector<uint8_t> band;
    map2d::inflateUnknown(v, w, h, c.res, c.radius, band);

    std::vector<int8_t> as_occ = v;
    for (auto& x : as_occ)
      if (x == kUnknown) x = kOccupied;
    std::vector<uint8_t> inflated_only;
    map2d::inflate(as_occ, inflated_only, w, h, c.res, c.radius);

    ASSERT_EQ(band.size(), inflated_only.size());
    EXPECT_EQ(band, inflated_only) << "res " << c.res << " radius " << c.radius;
  }
}

// Scenario: reuse an output buffer that still holds a previous band (all ones), call with a grid that has a
// single unknown cell.
// Expected: only the new disc is set. Why: map2d.hpp "`band` is resized ... and set to 1 ... 0 elsewhere".
TEST(InflateUnknown, OutputBufferIsOverwrittenNotAccumulated) {
  const int w = 8, h = 8;
  std::vector<int8_t> v(w * h, kFree);
  v[1 + w * 1] = kUnknown;
  std::vector<uint8_t> band(w * h, 1);
  map2d::inflateUnknown(v, w, h, 0.1, 0.1f, band);
  size_t n = 0;
  for (auto b : band) n += b;
  EXPECT_EQ(n, 4u) << "radius 1 cell around one unknown cell: its 4 neighbours";
  EXPECT_EQ(band[7 + w * 7], 0);
}

// Scenario: grid with unknown, free and occupied cells; compare `values` before/after (already covered by
// InputGridIsNotModified) and assert no cell becomes 100 -- phrased on the band side: the function takes the
// values by const reference, so the only way to check "does not mark cells occupied" is that the occupied
// count of the input is unchanged and unknown cells stay unknown.
// Why: N1 "It does not mark cells occupied".
TEST(InflateUnknown, NeverMarksValuesOccupied) {
  int w, h;
  auto v = parse({"..?..",
                  ".....",
                  "..#.."},
                 w, h);
  auto count = [&](int8_t val) {
    size_t n = 0;
    for (auto x : v) n += (x == val);
    return n;
  };
  const size_t occ = count(kOccupied), unk = count(kUnknown), fr = count(kFree);
  std::vector<uint8_t> band;
  map2d::inflateUnknown(v, w, h, 0.1, 0.3f, band);
  EXPECT_EQ(count(kOccupied), occ);
  EXPECT_EQ(count(kUnknown), unk);
  EXPECT_EQ(count(kFree), fr);
}
