// Node-contract tests for the ROS-free goal selector core (goal_selector::GoalSelector).
//
// "Contract" = what the ROS node (and the planner on the other side of planner_status / term_goal)
// can rely on: how each planner status is consumed, which statuses are ignored, how goals and stamps
// are published, which inputs drive selection, and what changes when exploration is on or off.
//
// Sources of truth (not the code): goal_selector_refactor_spec.md (D8 "SKIPPED is neutral", section 3
// "Consuming it in the selector", section 4.2/4.3 stamps, section 11 items 7, 15, 19, 20) and
// goal_selector_test_ideas.md section "D. Node contract". Read the header comment of
// goal_selector_test_utils.hpp for the ASCII corridor maps and the "utility = -distance" ranking.
//
// Test map -> notes/spec item
// -----------------------------------------------------------------------------------------------
//  Status_*                       D: "Each planner status maps to the right lifecycle event"
//  Status_SkippedIsNeutral        D: SKIPPED never counts and never resets (D8, 11.7)
//  Status_StaleStampsIgnored      D: "A status for an older goal ... is ignored" (incl. stamp 0)
//  Status_BeforeAnyGoal           spec 3: "before any goal arrives [stamp] is zero, selector ignores"
//  Goal_OnePerCommit_AsSelected   D: "One goal published per commit, exactly as selected"
//  Goal_StampRules                spec 4.3 / header: stamp = round(now*1e9), strictly increasing, != 0
//  Pose_*                         D: "Pose from state drives the watchdog, utility and selection"
//  ExplorationOff_*               D + spec 11.19/11.20: occ_2d stored, no detection, selector_map_2d
//  ExplorationOn_*                D: "Exploration on: detection runs"

#include <gtest/gtest.h>

#include <cmath>

#include "goal_selector_test_utils.hpp"

using namespace gs_test;

namespace {

/** Harness with the LEFT frontier committed (robot at the default start pose). */
void StartOnLeft(Harness& h) {
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 1u);
  ASSERT_EQ(h.goals[0].kind, GoalKind::kFrontier);
  ASSERT_TRUE(GoalNear(h.goals[0], kLeftX, kCorridorY));
}

/** Value of selector-map cell (cx, cy). */
int8_t Cell(const goal_selector::SelectorMap& m, int cx, int cy) {
  return m.data[static_cast<size_t>(cx) + static_cast<size_t>(m.width) * cy];
}

}  // namespace

// =================================================================================================
// Planner status -> lifecycle event
// =================================================================================================

// Scenario: FAILED statuses for the current frontier goal each add one to the unreachable counter.
// Why: spec 3 "FAILED: increment the unreachable counter."
TEST(GoalSelectorContract, Status_FailedIncrementsUnreachableCounter) {
  Harness h(BaseParams());
  StartOnLeft(h);
  for (int i = 1; i <= 4; ++i) {
    h.statusCur(PlannerStatus::kFailed);
    EXPECT_EQ(h.sel.unreachableCount(), i);
  }
  EXPECT_TRUE(h.sel.explorationActive());
}

// Scenario: SUCCESS and PARTIAL (planner produced a path) reset the counter. After either, a full
// five new FAILED are needed to invalidate (4 are not enough).
// Why: spec 3 "SUCCESS or PARTIAL: reset the counter."; notes B "The count resets on a success".
TEST(GoalSelectorContract, Status_SuccessAndPartialResetTheCounter) {
  for (PlannerStatus resetter : {PlannerStatus::kSuccess, PlannerStatus::kPartial}) {
    SCOPED_TRACE(resetter == PlannerStatus::kSuccess ? "SUCCESS" : "PARTIAL");
    Harness h(BaseParams());
    StartOnLeft(h);
    h.statusCur(PlannerStatus::kFailed, 4);
    ASSERT_EQ(h.sel.unreachableCount(), 4);

    h.statusCur(resetter);
    EXPECT_EQ(h.sel.unreachableCount(), 0);

    h.statusCur(PlannerStatus::kFailed, 4);
    EXPECT_TRUE(h.sel.explorationActive()) << "4 failures after the reset must not invalidate";
    EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 0);
    h.statusCur(PlannerStatus::kFailed);
    EXPECT_FALSE(h.sel.explorationActive()) << "5th failure after the reset invalidates";
    EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 1);
  }
}

// Scenario: REACHED for the current frontier: VISITED, active flag cleared, counter reset.
// Why: spec 3 "REACHED: the logic of goalReachedCheckCallback (mark VISITED, clear active, reset
// counter)".
TEST(GoalSelectorContract, Status_ReachedVisitsFrontierClearsActiveResetsCounter) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.statusCur(PlannerStatus::kFailed, 3);
  const uint64_t id = h.sel.currentExploreId();

  h.statusCur(PlannerStatus::kReached);
  EXPECT_EQ(h.sel.frontierManager().find(id)->state, FrontierState::VISITED);
  EXPECT_FALSE(h.sel.explorationActive());
  EXPECT_EQ(h.sel.unreachableCount(), 0);
  EXPECT_EQ(h.goals.size(), 1u) << "the status itself publishes nothing";
}

// Scenario: SKIPPED (replan did not run: not ready, at goal, GOAL_SEEN, yawing) is neutral. It does
// not add to the counter and does not reset it. 3 FAILED, 50 SKIPPED -> count still 3; then 2 more
// FAILED invalidate (5 FAILED in total, SKIPPED in between ignored). 50 SKIPPED alone never
// invalidates anything.
// Why: D8 "SKIPPED is neutral: it neither increments nor resets the unreachable counter"; 11.7.
TEST(GoalSelectorContract, Status_SkippedIsNeutral) {
  {
    Harness h(BaseParams());
    StartOnLeft(h);
    h.statusCur(PlannerStatus::kSkipped, 50);
    EXPECT_EQ(h.sel.unreachableCount(), 0) << "SKIPPED must not count as a failure";
    EXPECT_TRUE(h.sel.explorationActive());
    EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 0);
  }
  {
    Harness h(BaseParams());
    StartOnLeft(h);
    h.statusCur(PlannerStatus::kFailed, 3);
    h.statusCur(PlannerStatus::kSkipped, 50);
    EXPECT_EQ(h.sel.unreachableCount(), 3) << "SKIPPED must not reset the count either";
    h.statusCur(PlannerStatus::kFailed);
    h.statusCur(PlannerStatus::kSkipped, 5);
    EXPECT_TRUE(h.sel.explorationActive());
    h.statusCur(PlannerStatus::kFailed);  // 5th FAILED overall
    EXPECT_FALSE(h.sel.explorationActive());
    EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 1);
  }
}

// Scenario: statuses whose goal_stamp is not the current commitment's are ignored: an older stamp, a
// newer (unknown) stamp, and stamp 0, for FAILED, REACHED and SUCCESS alike. A flood of FAILED with
// the wrong stamp must not invalidate the goal, and a wrong-stamp REACHED must not complete it.
// Why: spec 3 "Ignore statuses whose goal_stamp differs from the stamp of the current commitment";
// "before any goal arrives it is zero, and the selector ignores such statuses".
TEST(GoalSelectorContract, Status_StaleStampsIgnored) {
  Harness h(BaseParams());
  StartOnLeft(h);
  const int64_t cur = h.sel.currentStampNs();
  const uint64_t id = h.sel.currentExploreId();

  for (int64_t wrong : std::vector<int64_t>{0, cur - 1, cur + 1, cur - 1000000000LL, cur + 1000000000LL}) {
    SCOPED_TRACE("wrong stamp " + std::to_string(wrong));
    for (int i = 0; i < 10; ++i) h.status(PlannerStatus::kFailed, wrong);
    h.status(PlannerStatus::kReached, wrong);
    h.status(PlannerStatus::kSuccess, wrong);
    EXPECT_EQ(h.sel.unreachableCount(), 0);
    EXPECT_TRUE(h.sel.explorationActive());
    EXPECT_EQ(h.sel.frontierManager().find(id)->state, FrontierState::ACTIVE);
  }
  // counter intact: wrong-stamp SUCCESS must not reset real failures either
  h.statusCur(PlannerStatus::kFailed, 3);
  h.status(PlannerStatus::kSuccess, 0);
  h.status(PlannerStatus::kSuccess, cur + 7);
  EXPECT_EQ(h.sel.unreachableCount(), 3);
}

// Scenario: feedback that refers to a PREVIOUS goal is not counted against the new one. Frontier A is
// invalidated, frontier B is committed; late FAILED statuses carrying A's stamp are dropped.
// Why: notes D/B "Feedback that refers to a previous goal is not counted against the current goal".
TEST(GoalSelectorContract, Status_LateFeedbackForPreviousGoalNotCountedAgainstNewGoal) {
  Harness h(BaseParams());
  StartOnLeft(h);
  const int64_t stamp_a = h.sel.currentStampNs();
  h.statusCur(PlannerStatus::kFailed, 5);  // A invalidated
  h.tick();                                  // B committed
  ASSERT_EQ(h.goals.size(), 2u);
  const int64_t stamp_b = h.sel.currentStampNs();
  ASSERT_NE(stamp_a, stamp_b);

  for (int i = 0; i < 20; ++i) h.status(PlannerStatus::kFailed, stamp_a);
  EXPECT_EQ(h.sel.unreachableCount(), 0);
  EXPECT_TRUE(h.sel.explorationActive());
  EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 1) << "only A";
}

// Scenario: a freshly built selector (no goal yet) receives statuses with stamp 0 and with an
// arbitrary stamp. Nothing happens: no goal, no state change, no crash.
// Why: spec 3 (stamp 0 before any goal; selector ignores); header "Statuses whose goal_stamp is 0 or
// differs from the current commitment's stamp are ignored".
TEST(GoalSelectorContract, Status_BeforeAnyGoalIsIgnored) {
  Harness h(BaseParams());
  for (PlannerStatus s : {PlannerStatus::kSuccess, PlannerStatus::kFailed, PlannerStatus::kPartial,
                          PlannerStatus::kReached, PlannerStatus::kSkipped}) {
    for (int64_t stamp : {int64_t{0}, int64_t{123456789}}) {
      Output o = h.status(s, stamp);
      EXPECT_FALSE(o.term_goal.has_value());
    }
  }
  EXPECT_TRUE(h.goals.empty());
  EXPECT_EQ(h.sel.unreachableCount(), 0);
  EXPECT_FALSE(h.sel.explorationActive());
  EXPECT_FALSE(h.sel.manualGoalActive());
  EXPECT_EQ(h.sel.currentKind(), GoalKind::kNone);
}

// =================================================================================================
// Goals and stamps
// =================================================================================================

// Scenario: one frontier commit = exactly one term_goal, equal to the selected record's centroid with
// z = expl_default_goal_z (0.25 here), no relocation. The same value goes to exploration_current_goal.
// Repeated state / grid / tick / status calls afterwards publish nothing more.
// Why: notes D "One goal published per commit, exactly as selected (no relocation)"; spec 11.15.
TEST(GoalSelectorContract, Goal_OnePerCommit_AsSelected) {
  SelectorParams p = BaseParams();
  p.expl_default_goal_z = 0.25;
  Harness h(p);
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);

  h.state();
  EXPECT_FALSE(h.last_out.term_goal.has_value());
  h.feed();
  EXPECT_FALSE(h.last_out.term_goal.has_value()) << "onOccGrid never selects (the node ticks next)";
  h.tick();
  ASSERT_TRUE(h.last_out.term_goal.has_value());
  ASSERT_EQ(h.goals.size(), 1u);

  const FrontierRecord* r = h.currentRecord();
  ASSERT_NE(r, nullptr);
  const GoalCommand& g = h.goals[0];
  EXPECT_EQ(g.kind, GoalKind::kFrontier);
  EXPECT_DOUBLE_EQ(g.position.x(), r->centroid_xy.x());
  EXPECT_DOUBLE_EQ(g.position.y(), r->centroid_xy.y());
  EXPECT_DOUBLE_EQ(g.position.z(), 0.25);
  EXPECT_EQ(g.stamp_ns, h.sel.currentStampNs());
  ASSERT_TRUE(h.last_out.exploration_current_goal.has_value());
  EXPECT_DOUBLE_EQ(h.last_out.exploration_current_goal->x(), g.position.x());
  EXPECT_DOUBLE_EQ(h.last_out.exploration_current_goal->y(), g.position.y());
  EXPECT_DOUBLE_EQ(h.sel.currentTarget().x(), g.position.x());

  // nothing else publishes a goal while the commitment stands
  h.run(10, 1.0);
  h.statusCur(PlannerStatus::kSuccess);
  h.statusCur(PlannerStatus::kSkipped);
  h.statusCur(PlannerStatus::kFailed);
  h.tick();
  EXPECT_EQ(h.goals.size(), 1u);
}

// Scenario: stamps. A commit at now = 123.456 s is stamped 123456000000 ns (round(now*1e9)). Two
// commits at the SAME now get strictly increasing stamps. A commit at now = 0 is not stamped 0.
// Why: header "stamp_ns = round(now * 1e9), forced to be strictly greater than the previous
// commitment's stamp (and never 0), even if `now` does not advance"; spec 4.3.
TEST(GoalSelectorContract, Goal_StampRules) {
  {
    Harness h(BaseParams(), 123.456);
    h.manual(9.0, 1.75);
    ASSERT_EQ(h.goals.size(), 1u);
    EXPECT_EQ(h.goals[0].stamp_ns, 123456000000LL);
    EXPECT_EQ(h.sel.currentStampNs(), 123456000000LL);
  }
  {
    Harness h(BaseParams(), 50.0);
    h.manual(9.0, 1.75);
    h.manual(9.5, 1.75);  // same now
    h.manual(10.0, 1.75);
    ASSERT_EQ(h.goals.size(), 3u);
    EXPECT_GT(h.goals[1].stamp_ns, h.goals[0].stamp_ns);
    EXPECT_GT(h.goals[2].stamp_ns, h.goals[1].stamp_ns);
  }
  {
    Harness h(BaseParams(), 0.0);
    h.manual(9.0, 1.75);
    ASSERT_EQ(h.goals.size(), 1u);
    EXPECT_GT(h.goals[0].stamp_ns, 0) << "stamp 0 means 'no goal' to the planner";
  }
}

// Scenario: every commitment (frontier, manual, return-home) gets its own distinct stamp, and only the
// latest one is "current" for status matching.
// Why: spec 4.3 "header.stamp of a published goal identifies one commitment".
TEST(GoalSelectorContract, Goal_EachCommitmentHasItsOwnStamp) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.advance(1.0);
  h.manual(9.0, 1.75);
  h.advance(1.0);
  h.statusCur(PlannerStatus::kReached);  // release manual
  h.advance(1.0);
  h.cycle();                             // frontier again
  h.advance(1.0);
  h.returnHome();
  ASSERT_EQ(h.goals.size(), 4u);
  EXPECT_EQ(h.goals[0].kind, GoalKind::kFrontier);
  EXPECT_EQ(h.goals[1].kind, GoalKind::kManual);
  EXPECT_EQ(h.goals[2].kind, GoalKind::kFrontier);
  EXPECT_EQ(h.goals[3].kind, GoalKind::kReturnHome);
  for (size_t i = 1; i < h.goals.size(); ++i) EXPECT_GT(h.goals[i].stamp_ns, h.goals[i - 1].stamp_ns);
  EXPECT_EQ(h.sel.currentStampNs(), h.goals[3].stamp_ns);
  EXPECT_EQ(h.sel.currentKind(), GoalKind::kReturnHome);
}

// =================================================================================================
// Pose from state
// =================================================================================================

// Scenario: selection ranks with the pose delivered by onState. The same map gives LEFT when the robot
// reports x = 3.0 and RIGHT when it reports x = 17.0.
// Why: notes D "Pose from state drives the watchdog, utility and selection" (utility = -distance).
TEST(GoalSelectorContract, Pose_StateDrivesSelection) {
  {
    Harness h(BaseParams());
    h.setMap(OpenBoth());
    h.setRobot(3.0, kStartY);
    h.cycle();
    ASSERT_EQ(h.goals.size(), 1u);
    EXPECT_TRUE(GoalNear(h.goals[0], kLeftX, kCorridorY));
  }
  {
    Harness h(BaseParams());
    h.setMap(OpenBoth());
    h.setRobot(17.0, kStartY);
    h.cycle();
    ASSERT_EQ(h.goals.size(), 1u);
    EXPECT_TRUE(GoalNear(h.goals[0], kRightX, kCorridorY));
  }
}

// Scenario: before the first state message the selector does not detect or select: a grid and a select
// tick produce no frontiers, no markers and no goal. After a state message the same grid is processed.
// Why: header "The first call [onState] marks the state as initialised (selection and detection wait
// for it)".
TEST(GoalSelectorContract, Pose_NothingHappensBeforeFirstState) {
  Harness h(BaseParams());
  h.setMap(OpenBoth());
  EXPECT_FALSE(h.sel.stateInitialized());

  h.feed();
  h.tick();
  EXPECT_TRUE(h.goals.empty());
  EXPECT_EQ(h.sel.frontierManager().size(), 0u);
  EXPECT_FALSE(h.last_out.publish_markers);

  h.setRobot(kStartX, kStartY);
  h.state();
  EXPECT_TRUE(h.sel.stateInitialized());
  h.feed();
  EXPECT_EQ(h.sel.frontierManager().size(), 2u);
  h.tick();
  EXPECT_EQ(h.goals.size(), 1u);
}

// Scenario: the watchdog reads the pose from state: with the pose unchanged between states the
// frontier is never released; with a pose update that moves the robot > 0.10 m and then stalls, it is.
// (Complements the lifecycle watchdog tests: here the point is that only onState changes the pose.)
// Why: notes D "Pose from state drives the watchdog".
TEST(GoalSelectorContract, Pose_WatchdogSeesMotionOnlyThroughState) {
  SelectorParams p = BaseParams();
  p.expl_pursuit_timeout_factor = 0.0;
  Harness h(p);
  StartOnLeft(h);

  // pose is only reported by onState; calling feed/tick with a different "harness pose" does nothing
  h.setRobot(kStartX - 2.0, kStartY);  // harness pose changed, but NOT sent via state
  for (int i = 0; i < 10; ++i) {
    h.advance(1.0);
    h.feed();
    h.tick();
  }
  EXPECT_EQ(h.sel.pose().x, kStartX) << "selector pose changes only through onState";

  h.state();  // now the selector learns the pose (first motion registered)
  EXPECT_NEAR(h.sel.pose().x, kStartX - 2.0, 1e-12);
  h.run(8, 1.0);  // stands still at the new pose -> stuck after 5 s
  EXPECT_GE(h.goals.size(), 2u);
}

// =================================================================================================
// Exploration off / on
// =================================================================================================

// Scenario: exploration disabled. onOccGrid only stores the grid: empty Output (no goal, no markers,
// no visited-map flags, no current-goal, no logs), no frontier records, nothing absorbed into the
// visited map, and onSelectTick does nothing. selectorMap() is available and describes the grid.
// Why: notes D "Exploration off: the selector still subscribes to occ_2d_topic, runs no frontier
// detection and publishes selector_map_2d"; spec 11.19, 11.20.
TEST(GoalSelectorContract, ExplorationOff_OccGridStoredNoDetection) {
  SelectorParams p = BaseParams();
  p.expl_enabled = false;
  Harness h(p);
  h.setMap(OpenBoth(/*origin_z=*/0.2));
  h.setRobot(kStartX, kStartY);
  h.state();

  goal_selector::SelectorMap m;
  EXPECT_FALSE(h.sel.hasOccGrid());
  EXPECT_FALSE(h.sel.selectorMap(m)) << "no grid yet";

  Output o = h.feed();
  EXPECT_FALSE(o.term_goal.has_value());
  EXPECT_FALSE(o.exploration_current_goal.has_value());
  EXPECT_FALSE(o.publish_markers);
  EXPECT_FALSE(o.publish_visited_map);
  EXPECT_FALSE(o.broadcast_visited_map);
  EXPECT_TRUE(o.logs.empty());
  EXPECT_TRUE(h.sel.hasOccGrid());
  EXPECT_EQ(h.sel.frontierManager().size(), 0u) << "no frontier detection";
  EXPECT_EQ(h.sel.visitedMap().getStateWorld(5.25, 1.75), VisitedMap::kUnknown)
      << "no visited-map absorb with exploration off";

  h.tick();
  EXPECT_TRUE(h.goals.empty());

  ASSERT_TRUE(h.sel.selectorMap(m));
  EXPECT_EQ(m.width, 41);
  EXPECT_EQ(m.height, 7);
  EXPECT_DOUBLE_EQ(m.resolution, 0.5);
  EXPECT_DOUBLE_EQ(m.origin_x, 0.0);
  EXPECT_DOUBLE_EQ(m.origin_y, 0.0);
  EXPECT_DOUBLE_EQ(m.origin_z, 0.2);
  ASSERT_EQ(m.data.size(), 41u * 7u);
}

// Scenario: encoding of selector_map_2d (derived frontier map) for the corridor with
// unknown_inflation_2d_m = 0.5 and inflation_2d_m = 0: 100 wall, 0 free, -1 real unknown, 50 unknown
// band (the first free column next to the unknown), no 99 cells. Checked with exploration OFF (map
// built from the raw grid) and ON (map the detector used): identical.
// Why: spec 11.20 "free 0, obstacle 100, obstacle band 99, unknown band 50, unknown -1"; header
// SelectorMap doc.
TEST(GoalSelectorContract, SelectorMap_EncodesFreeObstacleUnknownAndBand) {
  for (bool exploring : {false, true}) {
    SCOPED_TRACE(exploring ? "exploration on" : "exploration off");
    SelectorParams p = BaseParams();
    p.expl_enabled = exploring;
    Harness h(p);
    h.setMap(OpenBoth());
    h.setRobot(kStartX, kStartY);
    h.state();
    h.feed();

    goal_selector::SelectorMap m;
    ASSERT_TRUE(h.sel.selectorMap(m));
    EXPECT_EQ(Cell(m, 10, 0), 100);  // wall, bottom row
    EXPECT_EQ(Cell(m, 10, 6), 100);  // wall, top row
    EXPECT_EQ(Cell(m, 10, 3), 0);    // free interior
    EXPECT_EQ(Cell(m, 4, 3), 0);     // frontier cell (free, outside the band)
    EXPECT_EQ(Cell(m, 3, 3), 50);    // unknown band (free next to unknown)
    EXPECT_EQ(Cell(m, 37, 3), 50);
    EXPECT_EQ(Cell(m, 0, 3), -1);    // real unknown
    EXPECT_EQ(Cell(m, 2, 3), -1);
    EXPECT_EQ(Cell(m, 40, 3), -1);
    for (int8_t v : m.data) EXPECT_NE(v, 99) << "inflation_2d_m = 0: no obstacle band";
  }
}

// Scenario: with inflation_2d_m = 0.5 (one cell) the free cells next to a wall become the obstacle
// band, encoded 99; the wall itself stays 100.
// Why: spec 11.20 "obstacle band 99".
TEST(GoalSelectorContract, SelectorMap_ObstacleBandIs99) {
  SelectorParams p = BaseParams();
  p.expl_enabled = false;
  p.inflation_2d_m = 0.5;
  Harness h(p);
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.state();
  h.feed();
  goal_selector::SelectorMap m;
  ASSERT_TRUE(h.sel.selectorMap(m));
  EXPECT_EQ(Cell(m, 10, 0), 100);
  EXPECT_EQ(Cell(m, 10, 1), 99);
  EXPECT_EQ(Cell(m, 10, 5), 99);
  EXPECT_EQ(Cell(m, 10, 6), 100);
  EXPECT_EQ(Cell(m, 10, 3), 0);
}

// Scenario: manual goals work with exploration off: the goal is published as given, FAILED only warns
// (goal stays active), REACHED releases it, and nothing explores afterwards.
// Why: spec 4 (user decision 2026-10-08: manual-goal path not gated on exploration.enabled), D8a.
TEST(GoalSelectorContract, ExplorationOff_ManualGoalsStillWork) {
  SelectorParams p = BaseParams();
  p.expl_enabled = false;
  Harness h(p);
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  EXPECT_TRUE(h.goals.empty());

  h.manual(9.0, 1.5, 0.1);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_EQ(h.goals[0].kind, GoalKind::kManual);
  EXPECT_DOUBLE_EQ(h.goals[0].position.x(), 9.0);
  EXPECT_DOUBLE_EQ(h.goals[0].position.y(), 1.5);
  EXPECT_DOUBLE_EQ(h.goals[0].position.z(), 0.1);
  EXPECT_TRUE(h.sel.manualGoalActive());

  h.statusCur(PlannerStatus::kFailed, 6);
  EXPECT_TRUE(h.sel.manualGoalActive());
  EXPECT_TRUE(h.sel.manualGoalUnreachable());

  h.statusCur(PlannerStatus::kReached);
  EXPECT_FALSE(h.sel.manualGoalActive());
  h.run(5, 1.0);
  EXPECT_EQ(h.goals.size(), 1u) << "idle: no exploration";
}

// Scenario: exploration on. One raw grid is enough for detection to run: both corridor frontiers are
// recorded at the expected places, markers are requested, the visited map absorbed the grid, and
// selectorMap() is available. (No selection happens in onOccGrid itself.)
// Why: notes D "Exploration on: detection runs"; header onOccGrid doc.
TEST(GoalSelectorContract, ExplorationOn_DetectionRuns) {
  Harness h(BaseParams());
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.state();
  Output o = h.feed();

  EXPECT_TRUE(o.publish_markers);
  EXPECT_FALSE(o.term_goal.has_value());
  ASSERT_EQ(h.sel.frontierManager().size(), 2u);
  const FrontierRecord* l = h.recordNear(kLeftX, kCorridorY);
  const FrontierRecord* r = h.recordNear(kRightX, kCorridorY);
  ASSERT_NE(l, nullptr);
  ASSERT_NE(r, nullptr);
  EXPECT_NE(l->id, r->id);
  EXPECT_NEAR(l->centroid_xy.x(), kLeftX, 0.3);
  EXPECT_NEAR(l->centroid_xy.y(), kCorridorY, 0.3);
  EXPECT_NEAR(r->centroid_xy.x(), kRightX, 0.3);
  EXPECT_NEAR(r->centroid_xy.y(), kCorridorY, 0.3);
  EXPECT_EQ(l->state, FrontierState::ACTIVE);
  EXPECT_EQ(r->state, FrontierState::ACTIVE);
  EXPECT_NE(h.sel.visitedMap().getStateWorld(5.25, 1.75), VisitedMap::kUnknown) << "grid absorbed";

  goal_selector::SelectorMap m;
  EXPECT_TRUE(h.sel.selectorMap(m));
}

// Scenario: frontier cells sit in free space next to the unknown band, never inside it and never in
// walls: every cell of both corridor clusters lies on a safe-free cell of the selector map (value 0)
// and has a band cell (50) as an 8-neighbour.
// Why: spec N5 "Frontier cell: a safe-free cell ... with an 8-neighbour in the unknown band."
// (selector level; detector internals are tested elsewhere.) ASSUMPTION: the cluster aabb/centroid
// are enough to locate cells: centroid at cx=4 / cx=36, y mid-corridor.
TEST(GoalSelectorContract, ExplorationOn_FrontierCentroidsAreSafeFreeNextToBand) {
  Harness h(BaseParams());
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.state();
  h.feed();
  goal_selector::SelectorMap m;
  ASSERT_TRUE(h.sel.selectorMap(m));
  for (const auto& rec : h.sel.frontierManager().records()) {
    const int cx = static_cast<int>(std::floor(rec.centroid_xy.x() / m.resolution));
    const int cy = static_cast<int>(std::floor(rec.centroid_xy.y() / m.resolution));
    EXPECT_EQ(Cell(m, cx, cy), 0) << "frontier centroid cell must be safe free";
    bool near_band = false;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx)
        if (Cell(m, cx + dx, cy + dy) == 50) near_band = true;
    EXPECT_TRUE(near_band) << "adjacent to the unknown band";
  }
}
