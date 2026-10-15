// Tests for FrontierManager frontier ranking (utility terms, threshold, tie-break, MinPos, utilityOf).
//
// Source of truth
//   * goal_selector_test_ideas.md, section A "Ranking (FrontierManager)":
//       - each utility term moves the ranking the expected way with the others held equal:
//         size, distance, info gain, revisit count, heading;
//       - goal_select_threshold: returns nothing when every candidate scores below it;
//       - ties break deterministically;
//       - MinPos: ordered by rank, then distance, then utility; min_frontier_dist_to_peers_m excludes frontiers
//         near peers; with no peers it matches single-robot selection;
//       - utilityOf agrees with the cached_utility returned by selection (preemption depends on this).
//   * frontier_manager.hpp documentation of the additive utility:
//       u = w_size*size_norm - w_dist*dist_norm + w_info*info_norm - w_revisit*visit_count + w_heading*heading
//     with size_norm = clamp01(size_m2 / size_ref_m2), dist_norm = clamp01(dist / dist_ref_m),
//     info_norm = clamp01(unknown cells / cells in a disc of sensor_radius_m around the centroid),
//     heading = max(0, cos(bearing to centroid - yaw)).
//   Existing coverage (test_frontier_manager.cpp: lifecycle, ACTIVE-before-DORMANT for selectNextGoal) is not
//   repeated; ACTIVE-before-DORMANT is added only for the MinPos variant.
//
// Test map
//   Utility.SizeTerm...                                     -> A bullet 1 (size)
//   Utility.DistanceTerm...                                 -> A bullet 1 (distance)
//   Utility.InfoGainTerm...                                 -> A bullet 1 (info gain)
//   Utility.RevisitTermLowersUtilityOfARevisitedFrontier    -> A bullet 1 (revisit count)
//   Utility.HeadingTerm...                                  -> A bullet 1 (heading)
//   Utility.TermsAddUp                                      -> additive combination
//   Threshold.*                                             -> A bullet 2
//   TieBreak.*                                              -> A bullet 3
//   MinPos.*                                                -> A bullet 4
//   UtilityOf.*                                             -> A bullet 5
//
// Setup used throughout: a 100 x 60 cell all-free grid (resolution 0.1 m, covers [0,10] x [0,6] m) unless a test
// says otherwise; records are created by one FrontierManager::update() with hand-made clusters while the robot is
// far away (so the dwell check never fires); then the robot is "moved" to the pose given to the selection call.
// All weights are 0 except the ones a test is about, so each term is isolated.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "mighty/frontier_manager.hpp"
#include "mighty/occ_grid_2d.hpp"
#include "mighty/peer_tracker.hpp"

namespace {

constexpr double kRes = 0.1;
const Eigen::Vector3d kFarAway(-100.0, -100.0, 0.0);  // robot pose during update(): never near anything

/** All-free grid of w x h cells, resolution 0.1, origin (0,0). Optionally `unknown_from_col` marks every
 *  column >= that index as unknown (all rows). */
std::shared_ptr<const OccGrid2D> makeGrid(int w = 100, int h = 60, int unknown_from_col = -1) {
  std::vector<int8_t> d(static_cast<size_t>(w) * h, 0);
  if (unknown_from_col >= 0)
    for (int y = 0; y < h; ++y)
      for (int x = unknown_from_col; x < w; ++x) d[x + static_cast<size_t>(w) * y] = -1;
  return OccGrid2D::fromTristate(w, h, kRes, 0.0, 0.0, d);
}

FrontierCluster cluster(double x, double y, int size_cells = 100) {
  FrontierCluster c;
  c.centroid = Eigen::Vector2d(x, y);
  c.size_cells = size_cells;
  c.size_m2 = size_cells * kRes * kRes;
  c.aabb_min = c.centroid - Eigen::Vector2d(0.1, 0.1);
  c.aabb_max = c.centroid + Eigen::Vector2d(0.1, 0.1);
  return c;
}

/** Params with every weight zero: tests switch on the one they study. Pursuit timeout and peer-visit off. */
FrontierManagerParams zeroWeights() {
  FrontierManagerParams p;
  p.w_size = p.w_dist = p.w_info = p.w_revisit = p.w_heading = 0.0;
  p.size_ref_m2 = 5.0;
  p.dist_ref_m = 25.0;
  p.sensor_radius_m = 1.0;
  p.goal_select_threshold = -1.0e9;
  p.merge_radius_m = 0.5;
  p.visit_radius_m = 0.3;
  p.visit_dwell_sec = 1.0;
  p.pursuit_timeout_factor = 0.0;
  p.peer_visit_radius_m = 0.0;
  p.invalidation_keep_out_radius_m = 0.0;
  p.max_frontiers = 1000;
  return p;
}

/** Create a manager holding one ACTIVE record per cluster, in the given order (record i has id i). */
FrontierManager makeManager(const FrontierManagerParams& p, const std::vector<FrontierCluster>& cs,
                            const OccGrid2D& grid) {
  FrontierManager m(p);
  m.update(cs, grid, kFarAway, 1.0);
  return m;
}

}  // namespace

// =============================================================================================
// Utility terms (each isolated)
// =============================================================================================

// Scenario: two clusters, same everything except size (10 cells vs 200 cells); only w_size = 1.
// Expected: the larger one is selected; utilityOf = size_m2 / size_ref (0.02/... see numbers); sizes above the
// reference clamp to 1 so two huge clusters tie.
// Why: notes A "each utility term moves the ranking the expected way" (size: bigger is better).
TEST(Utility, SizeTermPrefersTheLargerClusterAndClampsAtTheReference) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(3, 3, 10), cluster(3, 4, 200), cluster(3, 5, 800), cluster(3, 2, 1000)}, *grid);
  const Eigen::Vector3d robot(5.0, 3.0, 0.0);

  // 10 cells = 0.1 m2 -> 0.02; 200 cells = 2.0 m2 -> 0.4; 800 cells = 8 m2 and 1000 cells = 10 m2 -> clamped 1.
  EXPECT_NEAR(*m.utilityOf(0, robot, *grid), 0.02, 1e-12);
  EXPECT_NEAR(*m.utilityOf(1, robot, *grid), 0.40, 1e-12);
  EXPECT_NEAR(*m.utilityOf(2, robot, *grid), 1.00, 1e-12);
  EXPECT_NEAR(*m.utilityOf(3, robot, *grid), 1.00, 1e-12);

  auto best = m.selectNextGoal(robot, *grid);
  ASSERT_TRUE(best.has_value());
  EXPECT_TRUE(best->id == 2 || best->id == 3) << "a clamped (huge) cluster must win, got id " << best->id;

  // only the small and the medium one: the medium one wins
  auto m2 = makeManager(p, {cluster(3, 3, 10), cluster(3, 4, 200)}, *grid);
  EXPECT_EQ(m2.selectNextGoal(robot, *grid)->id, 1u);
}

// Scenario: three clusters of equal size at distances 5 m, 10 m and 60 m (beyond dist_ref 25 m) from the robot;
// only w_dist = 2.
// Expected: the nearest wins; utility = -w_dist * dist / dist_ref, clamped: beyond the reference it stays -2.
// Why: notes A (distance: closer is better).
TEST(Utility, DistanceTermPrefersTheNearerClusterAndClampsAtTheReference) {
  auto p = zeroWeights();
  p.w_dist = 2.0;
  auto grid = makeGrid();
  const Eigen::Vector3d robot(1.0, 1.0, 0.0);
  auto m = makeManager(p, {cluster(61.0, 1.0), cluster(11.0, 1.0), cluster(6.0, 1.0), cluster(-9.0, 1.0)}, *grid);

  EXPECT_NEAR(*m.utilityOf(0, robot, *grid), -2.0, 1e-12) << "60 m: clamped";
  EXPECT_NEAR(*m.utilityOf(1, robot, *grid), -2.0 * 10.0 / 25.0, 1e-12);
  EXPECT_NEAR(*m.utilityOf(2, robot, *grid), -2.0 * 5.0 / 25.0, 1e-12);
  EXPECT_NEAR(*m.utilityOf(3, robot, *grid), -2.0 * 10.0 / 25.0, 1e-12);
  auto best = m.selectNextGoal(robot, *grid);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(best->id, 2u);
}

// Scenario: 8 m x 4 m grid where columns >= 50 (x >= 5 m) are unknown. Sensor radius 1 m (10 cells). Cluster A
// at x = 4.5 (half of its sensor disc reaches into the unknown), cluster C at x = 4.0 (disc just touches it),
// cluster B at x = 1.0 (disc fully known). Only w_info = 1; equal sizes, distance ignored.
// Expected: info(A) > info(C) > info(B) = 0; A is selected.
// Why: notes A (info gain: more unknown around the centroid is better; the count is over a disc of
// sensor_radius_m on the CURRENT grid).
TEST(Utility, InfoGainTermPrefersClustersWithMoreUnknownAroundThem) {
  auto p = zeroWeights();
  p.w_info = 1.0;
  p.sensor_radius_m = 1.0;
  auto grid = makeGrid(80, 40, 50);
  const Eigen::Vector3d robot(0.5, 0.5, 0.0);
  auto m = makeManager(p, {cluster(1.0, 2.0), cluster(4.0, 2.0), cluster(4.5, 2.0)}, *grid);
  const double uB = *m.utilityOf(0, robot, *grid);
  const double uC = *m.utilityOf(1, robot, *grid);
  const double uA = *m.utilityOf(2, robot, *grid);
  EXPECT_DOUBLE_EQ(uB, 0.0);
  EXPECT_GT(uC, uB);
  EXPECT_GT(uA, uC);
  EXPECT_LE(uA, 1.0);
  auto best = m.selectNextGoal(robot, *grid);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(best->id, 2u);
}

// Scenario: two clusters with identical size and distance; one of them has been visited once
// (markVisited -> visit_count 1, state VISITED). Only w_revisit = 0.5.
// Expected: utilityOf(visited) = utilityOf(other) - 0.5 * visit_count.
// Why: notes A (revisit count lowers the utility). NOTE (observation for the report): selection only ever ranks
// ACTIVE/DORMANT records and a VISITED record is never revived, so in practice visit_count > 0 never reaches the
// ranking; the term is observable through utilityOf only.
TEST(Utility, RevisitTermLowersUtilityOfARevisitedFrontier) {
  auto p = zeroWeights();
  p.w_revisit = 0.5;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(3, 3), cluster(3, 3.2)}, *grid);
  const Eigen::Vector3d robot(5, 3, 0);
  m.markVisited(1);
  ASSERT_EQ(m.find(1)->visit_count, 1);
  EXPECT_NEAR(*m.utilityOf(0, robot, *grid), 0.0, 1e-12);
  EXPECT_NEAR(*m.utilityOf(1, robot, *grid), -0.5, 1e-12);
  // and a second visit lowers it further
  m.markVisited(1);
  EXPECT_NEAR(*m.utilityOf(1, robot, *grid), -1.0, 1e-12);
}

// Scenario: robot at (5,3) heading +x (yaw 0); clusters straight ahead (7,3), straight behind (3,3), and to the
// side (5,5), all 2 m away; only w_heading = 0.3. Then the robot turns to yaw 90 deg and 45 deg.
// Expected: ahead scores 0.3, behind and side score 0 (clamped at 0: no penalty for behind), so ahead is
// selected; with yaw 90 deg the side cluster is selected; at 45 deg between ahead and side both get 0.3*cos(45).
// Why: notes A (heading) and the header comment of computeUtility: "no penalty for behind-robot frontiers; only a
// bonus for in-front".
TEST(Utility, HeadingTermRewardsFrontiersInFrontAndNeverPenalisesBehind) {
  auto p = zeroWeights();
  p.w_heading = 0.3;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(3, 3), cluster(5, 5), cluster(7, 3)}, *grid);  // behind, side, ahead

  Eigen::Vector3d robot(5.0, 3.0, 0.0);
  EXPECT_NEAR(*m.utilityOf(2, robot, *grid), 0.3, 1e-12);
  EXPECT_NEAR(*m.utilityOf(0, robot, *grid), 0.0, 1e-12);
  EXPECT_NEAR(*m.utilityOf(1, robot, *grid), 0.0, 1e-9);
  EXPECT_EQ(m.selectNextGoal(robot, *grid)->id, 2u);

  robot.z() = M_PI / 2.0;
  EXPECT_EQ(m.selectNextGoal(robot, *grid)->id, 1u);

  robot.z() = M_PI / 4.0;
  EXPECT_NEAR(*m.utilityOf(2, robot, *grid), 0.3 * std::cos(M_PI / 4.0), 1e-12);
  EXPECT_NEAR(*m.utilityOf(1, robot, *grid), 0.3 * std::cos(M_PI / 4.0), 1e-12);
}

// Scenario: all five weights non-zero; one cluster 3 m ahead of the robot, size 200 cells, on an all-free grid
// (info = 0), visit_count 0.
// Expected: utility = w_size*0.4 - w_dist*(3/25) + w_heading*1 (the terms add).
// Why: documented additive utility.
TEST(Utility, TermsAddUp) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  p.w_dist = 2.0;
  p.w_info = 1.0;
  p.w_revisit = 0.5;
  p.w_heading = 0.3;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(5.0, 3.0, 200)}, *grid);
  const Eigen::Vector3d robot(2.0, 3.0, 0.0);
  const double expect = 1.0 * 0.4 - 2.0 * (3.0 / 25.0) + 1.0 * 0.0 - 0.5 * 0 + 0.3 * 1.0;
  EXPECT_NEAR(*m.utilityOf(0, robot, *grid), expect, 1e-12);
}

// =============================================================================================
// goal_select_threshold
// =============================================================================================

// Scenario: two clusters with utilities -0.2 (5 m away) and -0.4 (10 m away) (only w_dist = 1).
// Expected: threshold above both -> nullopt; threshold equal to the best -> that one is returned (strictly
// below the threshold is rejected, equal is accepted); threshold between -> still the best.
// Why: notes A "goal_select_threshold: returns nothing when every candidate scores below it".
TEST(Threshold, NothingIsReturnedWhenEveryCandidateScoresBelowIt) {
  auto p = zeroWeights();
  p.w_dist = 1.0;
  auto grid = makeGrid();
  const Eigen::Vector3d robot(0.0, 0.0, 0.0);
  const std::vector<FrontierCluster> cs = {cluster(5.0, 0.0), cluster(10.0, 0.0)};

  const double best = *makeManager(p, cs, *grid).utilityOf(0, robot, *grid);  // -0.2
  EXPECT_NEAR(best, -0.2, 1e-12);

  p.goal_select_threshold = best + 1e-6;
  EXPECT_FALSE(makeManager(p, cs, *grid).selectNextGoal(robot, *grid).has_value());

  p.goal_select_threshold = best;
  auto at = makeManager(p, cs, *grid).selectNextGoal(robot, *grid);
  ASSERT_TRUE(at.has_value());
  EXPECT_EQ(at->id, 0u);

  p.goal_select_threshold = -0.3;
  auto between = makeManager(p, cs, *grid).selectNextGoal(robot, *grid);
  ASSERT_TRUE(between.has_value());
  EXPECT_EQ(between->id, 0u);
}

// Scenario: same, MinPos variant, threshold above every utility.
// ASSUMPTION: the threshold applies to the MinPos selection too (notes A speaks of selection in general, and
// the frontier_manager.hpp doc of selectNextGoal "nullopt if nothing scores above goal_select_threshold" is the
// only place it is stated; the MinPos doc says nothing). If MinPos ignores the threshold this test fails and
// that is a finding to report.
TEST(Threshold, MinPosAlsoReturnsNothingWhenEveryCandidateScoresBelowTheThreshold) {
  auto p = zeroWeights();
  p.w_dist = 1.0;
  p.goal_select_threshold = -0.1;  // best utility is -0.2
  auto grid = makeGrid();
  const Eigen::Vector3d robot(0.0, 0.0, 0.0);
  auto m = makeManager(p, {cluster(5.0, 0.0), cluster(10.0, 0.0)}, *grid);
  EXPECT_FALSE(m.selectNextGoal(robot, *grid).has_value()) << "control: single-robot selection honours it";
  EXPECT_FALSE(m.selectNextGoalMinPos(robot, *grid, {}).has_value());
}

// =============================================================================================
// Tie-break
// =============================================================================================

// Scenario: two clusters that score exactly the same (same size, mirror-image positions 0.5 m left/right of the
// robot, w_size = w_dist = 1). Ask repeatedly, and also with the insertion order swapped.
// Expected: the same, earliest-created record wins every time (strict '>' comparison keeps the first), i.e.
// the result depends only on record order, never on call count.
// Why: notes A "Ties break deterministically".
// ASSUMPTION: "deterministic" means "the first record in the database (lowest id) wins".
TEST(TieBreak, EqualUtilityPicksTheFirstRecordEveryTime) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  p.w_dist = 1.0;
  auto grid = makeGrid();
  const Eigen::Vector3d robot(1.0, 1.0, 0.0);

  auto m = makeManager(p, {cluster(1.5, 1.0), cluster(0.5, 1.0)}, *grid);
  ASSERT_DOUBLE_EQ(*m.utilityOf(0, robot, *grid), *m.utilityOf(1, robot, *grid));
  for (int i = 0; i < 5; ++i) EXPECT_EQ(m.selectNextGoal(robot, *grid)->id, 0u) << "call " << i;

  auto swapped = makeManager(p, {cluster(0.5, 1.0), cluster(1.5, 1.0)}, *grid);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(swapped.selectNextGoal(robot, *grid)->id, 0u) << "call " << i;
  EXPECT_NEAR(swapped.selectNextGoal(robot, *grid)->centroid_xy.x(), 0.5, 1e-12);
}

// Scenario: MinPos, two clusters with identical rank (no peers), distance and utility.
// Expected: the same record every call, the first one.
// Why: notes A "Ties break deterministically" (MinPos sort is by rank, distance, utility; equal elements).
// ASSUMPTION: first record wins (std::sort is not guaranteed stable, so this also guards against an unstable
// order for equal keys).
TEST(TieBreak, MinPosFullTieIsStable) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto grid = makeGrid();
  const Eigen::Vector3d robot(1.0, 1.0, 0.0);
  auto m = makeManager(p, {cluster(1.5, 1.0), cluster(0.5, 1.0)}, *grid);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(m.selectNextGoalMinPos(robot, *grid, {})->id, 0u) << "call " << i;
}

// =============================================================================================
// MinPos
// =============================================================================================

// Scenario: robot (0,0). A at (2,0) is the nearest frontier but a peer at (2.5,0) is closer to it (0.5 m < 2 m:
// our rank at A is 1). B at (0,-4) is farther (4 m) but no peer is closer to it than we are (rank 0).
// Expected: B is selected (rank first, then distance). Control: without the peer A is selected.
// Why: notes A "MinPos: ordered by rank, then distance, then utility".
TEST(MinPos, RankComesBeforeDistance) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto grid = makeGrid(100, 100);  // [0,10]^2 ; clusters may lie outside, utility terms used here ignore that
  auto m = makeManager(p, {cluster(2.0, 0.0), cluster(0.0, -4.0)}, *grid);
  const Eigen::Vector3d robot(0, 0, 0);
  const std::vector<PeerPose> peers = {{Eigen::Vector2d(2.5, 0.0)}};

  EXPECT_EQ(m.selectNextGoalMinPos(robot, *grid, peers)->id, 1u);
  EXPECT_EQ(m.selectNextGoalMinPos(robot, *grid, {})->id, 0u) << "control: no peer, nearest wins";
}

// Scenario: same rank (no peers); a near small cluster (1 m, 10 cells) and a far huge cluster (2 m, 800 cells);
// w_size = 1 so the huge one has the higher utility.
// Expected: MinPos picks the NEAR one (distance before utility), whereas single-robot selectNextGoal (utility
// only, w_dist = 0) picks the huge one.
// Why: notes A "ordered by rank, then distance, then utility".
TEST(MinPos, DistanceComesBeforeUtility) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(3.0, 3.0, 10), cluster(4.0, 3.0, 800)}, *grid);
  const Eigen::Vector3d robot(2.0, 3.0, 0.0);
  EXPECT_EQ(m.selectNextGoalMinPos(robot, *grid, {})->id, 0u);
  EXPECT_EQ(m.selectNextGoal(robot, *grid)->id, 1u);
}

// Scenario: same rank and exactly the same distance (0.5 m left and right of the robot); the sizes differ.
// Expected: the higher-utility (bigger) cluster wins, whichever was inserted first.
// Why: notes A (utility is the last key).
TEST(MinPos, UtilityBreaksADistanceTie) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto grid = makeGrid();
  const Eigen::Vector3d robot(1.0, 1.0, 0.0);
  auto a = makeManager(p, {cluster(1.5, 1.0, 10), cluster(0.5, 1.0, 100)}, *grid);
  EXPECT_EQ(a.selectNextGoalMinPos(robot, *grid, {})->id, 1u);
  auto b = makeManager(p, {cluster(1.5, 1.0, 100), cluster(0.5, 1.0, 10)}, *grid);
  EXPECT_EQ(b.selectNextGoalMinPos(robot, *grid, {})->id, 0u);
}

// Scenario: nearest frontier A (2,0) has a peer 0.5 m from it; B (0,3) does not. min_dist_to_peers_m = 1.0.
// Expected: A is excluded outright (even though, without the peer rank, it would be selected), so B is
// returned; when the peer is near every frontier, nothing is returned.
// Why: notes A "min_frontier_dist_to_peers_m excludes frontiers near peers".
TEST(MinPos, FrontiersNearPeersAreExcluded) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto grid = makeGrid(100, 100);
  auto m = makeManager(p, {cluster(2.0, 0.0), cluster(0.0, 3.0)}, *grid);
  const Eigen::Vector3d robot(0, 0, 0);
  const std::vector<PeerPose> peer_at_a = {{Eigen::Vector2d(2.0, 0.5)}};

  EXPECT_EQ(m.selectNextGoalMinPos(robot, *grid, peer_at_a, 1.0)->id, 1u);
  // control: a peer far from both frontiers excludes nothing, and A (nearest, rank 0) is chosen
  const std::vector<PeerPose> far_peer = {{Eigen::Vector2d(9.0, 9.0)}};
  EXPECT_EQ(m.selectNextGoalMinPos(robot, *grid, far_peer, 1.0)->id, 0u) << "peer far away excludes nothing";

  // peers sitting at both frontiers: everything excluded
  const std::vector<PeerPose> both = {{Eigen::Vector2d(2.0, 0.2)}, {Eigen::Vector2d(0.0, 3.2)}};
  EXPECT_FALSE(m.selectNextGoalMinPos(robot, *grid, both, 1.0).has_value());
}

// Scenario: no peers, only the distance weight non-zero (so the single-robot utility ranking is a pure distance
// ranking); three frontiers; several robot poses.
// Expected: MinPos and selectNextGoal choose the same frontier, and MinPos' cached_utility equals the
// single-robot one.
// Why: notes A "with no peers it matches single-robot selection".
// ASSUMPTION: "matches" is meant for the case where the utility is a pure distance ranking; with size/info
// weights MinPos (nearest first) legitimately differs from utility-first selection (see
// MinPos.DistanceComesBeforeUtility).
TEST(MinPos, WithoutPeersItMatchesSingleRobotSelection) {
  auto p = zeroWeights();
  p.w_dist = 2.0;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(2.0, 1.0, 30), cluster(8.0, 5.0, 500), cluster(5.0, 3.0, 5)}, *grid);
  for (const Eigen::Vector3d robot : {Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(9, 5, 1.0),
                                      Eigen::Vector3d(5, 3.4, 2.0), Eigen::Vector3d(4, 5.5, -1.0)}) {
    auto single = m.selectNextGoal(robot, *grid);
    auto minpos = m.selectNextGoalMinPos(robot, *grid, {}, 1.0);  // exclusion distance irrelevant without peers
    ASSERT_TRUE(single && minpos);
    EXPECT_EQ(single->id, minpos->id) << "robot (" << robot.x() << "," << robot.y() << ")";
    EXPECT_DOUBLE_EQ(single->cached_utility, minpos->cached_utility);
  }
}

// Scenario: an ACTIVE frontier far from the robot and a DORMANT one right next to it (the DORMANT record is
// outside the current 2 m x 2 m window, so it went DORMANT on the second update), no peers.
// Expected: MinPos returns the ACTIVE record (tier first) although the DORMANT one is nearer and, with
// the ACTIVE tier empty, the DORMANT one is returned.
// Why: two-tier selection (ACTIVE exhausted before DORMANT) -- covered for selectNextGoal by
// test_frontier_manager.cpp; added here for the MinPos variant (notes A MinPos).
TEST(MinPos, ActiveTierIsExhaustedBeforeDormant) {
  auto p = zeroWeights();
  p.w_size = 1.0;
  auto big = makeGrid();  // [0,10] x [0,6]
  FrontierManager m(p);
  m.update({cluster(1.0, 1.0), cluster(8.0, 5.0)}, *big, kFarAway, 1.0);
  auto small = makeGrid(20, 20);  // [0,2]^2: the (8,5) record is outside
  m.update({cluster(1.0, 1.0)}, *small, kFarAway, 2.0);
  ASSERT_EQ(m.records().size(), 2u);
  ASSERT_EQ(m.records()[0].state, FrontierState::ACTIVE);
  ASSERT_EQ(m.records()[1].state, FrontierState::DORMANT);

  const Eigen::Vector3d robot(7.9, 5.0, 0.0);  // right next to the DORMANT record, far from the ACTIVE one
  EXPECT_EQ(m.selectNextGoalMinPos(robot, *small, {})->id, 0u);

  m.markVisited(0);  // ACTIVE tier now empty
  EXPECT_EQ(m.selectNextGoalMinPos(robot, *small, {})->id, 1u);
}

// =============================================================================================
// utilityOf vs cached_utility
// =============================================================================================

// Scenario: all weights non-zero, grid with an unknown half (so the info term is non-zero for some records),
// several records, several robot poses. For each pose, select (single-robot and MinPos with and without peers).
// Expected: the returned record's cached_utility == utilityOf(record id) for the same pose and grid, exactly.
// Why: notes A "utilityOf agrees with the cached_utility returned by selection (preemption depends on this)".
TEST(UtilityOf, EqualsCachedUtilityReturnedBySelection) {
  FrontierManagerParams p = zeroWeights();
  p.w_size = 1.0;
  p.w_dist = 2.0;
  p.w_info = 1.0;
  p.w_revisit = 0.5;
  p.w_heading = 0.3;
  p.sensor_radius_m = 1.5;
  auto grid = makeGrid(80, 40, 50);
  auto m = makeManager(p, {cluster(1.0, 1.0, 30), cluster(4.4, 2.0, 300), cluster(2.5, 3.5, 90),
                           cluster(4.9, 0.5, 40)}, *grid);
  const std::vector<PeerPose> peers = {{Eigen::Vector2d(4.0, 2.0)}};

  for (const Eigen::Vector3d robot : {Eigen::Vector3d(0.5, 0.5, 0.0), Eigen::Vector3d(3.0, 3.0, 1.2),
                                      Eigen::Vector3d(4.8, 1.0, -2.0)}) {
    auto single = m.selectNextGoal(robot, *grid);
    ASSERT_TRUE(single.has_value());
    ASSERT_TRUE(m.utilityOf(single->id, robot, *grid).has_value());
    EXPECT_DOUBLE_EQ(single->cached_utility, *m.utilityOf(single->id, robot, *grid));

    auto mp0 = m.selectNextGoalMinPos(robot, *grid, {});
    ASSERT_TRUE(mp0.has_value());
    EXPECT_DOUBLE_EQ(mp0->cached_utility, *m.utilityOf(mp0->id, robot, *grid));

    auto mp1 = m.selectNextGoalMinPos(robot, *grid, peers, 0.0);
    ASSERT_TRUE(mp1.has_value());
    EXPECT_DOUBLE_EQ(mp1->cached_utility, *m.utilityOf(mp1->id, robot, *grid));
  }
}

// Scenario: utilityOf for an id that does not exist, and for a record after the robot moved.
// Expected: nullopt for the unknown id; the value follows the CURRENT pose (not a cached value).
// Why: frontier_manager.hpp "Utility of a specific record against the CURRENT robot pose/grid ... nullopt if the
// id is unknown."
TEST(UtilityOf, UnknownIdIsNulloptAndValueTracksTheCurrentPose) {
  auto p = zeroWeights();
  p.w_dist = 1.0;
  auto grid = makeGrid();
  auto m = makeManager(p, {cluster(5.0, 3.0)}, *grid);
  EXPECT_FALSE(m.utilityOf(42, Eigen::Vector3d(0, 0, 0), *grid).has_value());
  EXPECT_NEAR(*m.utilityOf(0, Eigen::Vector3d(0, 3, 0), *grid), -5.0 / 25.0, 1e-12);
  EXPECT_NEAR(*m.utilityOf(0, Eigen::Vector3d(4, 3, 0), *grid), -1.0 / 25.0, 1e-12);
}
