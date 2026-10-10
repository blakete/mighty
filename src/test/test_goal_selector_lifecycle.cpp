// Selection lifecycle tests for the ROS-free goal selector core (goal_selector::GoalSelector).
//
// Sources of truth (not the code): goal_selector_refactor_spec.md (D8, D8a, section 3 "Consuming it in
// the selector", section 4.2, section 11 items 7, 10, 15, 17, 21, New Plan N4/N5) and
// goal_selector_test_ideas.md section "B. Selection lifecycle".
//
// The selector is driven with a fake clock, the ASCII corridor maps of goal_selector_test_utils.hpp
// (read the header comment first: it documents the map and the "utility = -distance" ranking trick)
// and scripted planner statuses. Nothing here uses ROS.
//
// Test map -> notes/spec item
// -----------------------------------------------------------------------------------------------
//  FailedThreshold_*            B: "N failures in a row -> frontier invalidated, new goal"; spec 3
//  CountResetOnNewCommit        B: "the count resets ... on a new commit"
//  ReachedMarksVisited_*        B: "goal reached -> frontier VISITED -> next goal"; spec 3 (REACHED)
//  StuckWatchdog_*              B: stuck watchdog (frontier arms after first motion; never-moved ok)
//  PursuitBudget_*              B: budget = max(min_sec, dist / v_ref * factor); only inside update()
//  PursuitDeadline_NotRearmed   B: "the deadline is not re-armed when the same frontier is re-selected"
//  Preempt_*                    B: preemption rules (min commit time, margin, released not invalidated,
//                                  disabled -> holds)
//  ReturnHome_*                 B: no frontiers left -> return home, latch holds, re-arms on arrival;
//                                  return_home trigger stops new frontier goals
//  Manual_*                     B: manual goal suppresses exploration, REACHED releases it; published
//                                  unchanged (spec 11.15); FAILED only warns (D8a); start timeout
//                                  (spec 11.21); stuck timeout after first motion (N4 Resolved,
//                                  11.17); clears the interrupted frontier's pursuit deadline (11.10)
//  KeepOut_*                    B: "an invalidated frontier is not immediately re-selected"
//
// Expectations were derived from the documents above, never copied from the code's output.

#include <gtest/gtest.h>

#include "goal_selector_test_utils.hpp"

using namespace gs_test;

namespace {

/** Commit the LEFT frontier from the default start pose and return the harness ready for events. */
void StartOnLeft(Harness& h) {
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 1u);
  ASSERT_EQ(h.goals[0].kind, GoalKind::kFrontier);
  ASSERT_TRUE(GoalNear(h.goals[0], kLeftX, kCorridorY));
}

}  // namespace

// =================================================================================================
// FAILED x unreachable_consec_thresh
// =================================================================================================

// Scenario: the planner reports FAILED for the current frontier goal. Below the threshold (5) nothing
// happens; the 5th consecutive FAILED invalidates the frontier, clears the active flag, resets the
// counter, and the next select tick picks a different frontier with a fresh, larger stamp.
// Why: spec section 3 "FAILED: increment the unreachable counter. At unreachable_consec_thresh:
// invalidate the frontier, clear the active flag, reset the counter."
TEST(GoalSelectorLifecycle, FailedThreshold_InvalidatesFrontierAndSelectsNextGoal) {
  Harness h(BaseParams());
  StartOnLeft(h);
  const int64_t first_stamp = h.sel.currentStampNs();
  const uint64_t left_id = h.sel.currentExploreId();

  h.statusCur(PlannerStatus::kFailed, 4);
  EXPECT_EQ(h.sel.unreachableCount(), 4);
  EXPECT_TRUE(h.sel.explorationActive());
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::ACTIVE);
  EXPECT_EQ(h.goals.size(), 1u) << "no new goal below the threshold";

  h.statusCur(PlannerStatus::kFailed);  // 5th in a row
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_FALSE(h.sel.explorationActive());
  EXPECT_EQ(h.sel.unreachableCount(), 0);
  EXPECT_EQ(h.goals.size(), 1u) << "the status itself does not publish; the next select tick does";

  h.tick();
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_EQ(h.goals[1].kind, GoalKind::kFrontier);
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY)) << "the other frontier is chosen";
  EXPECT_GT(h.goals[1].stamp_ns, first_stamp);
  EXPECT_NE(h.sel.currentExploreId(), left_id);
}

// Scenario: FAILED count is per commitment. 4 FAILED on frontier A, then A is preempted by B (a new
// commit). The 4 old failures must not carry over: B needs its own 5 FAILED to be invalidated.
// Why: notes B "The count resets ... on a new commit"; commit path sets unreachable_consec_count_ = 0.
TEST(GoalSelectorLifecycle, CountResetOnNewCommit) {
  SelectorParams p = BaseParams();
  p.expl_preempt_enabled = true;      // the easiest way to get a second frontier commit
  p.expl_preempt_min_commit_sec = 2.0;
  p.expl_preempt_margin = 2.0;
  p.expl_stuck_timeout_sec = 1000.0;  // keep the watchdog out of the way
  Harness h(p);
  StartOnLeft(h);
  const int64_t stamp_a = h.sel.currentStampNs();
  h.statusCur(PlannerStatus::kFailed, 4);
  ASSERT_EQ(h.sel.unreachableCount(), 4);

  // Robot drives to x = 14.25: RIGHT is now 4.0 m away, LEFT 12.0 m -> utility gap 8 > margin 2.
  h.advance(3.0);
  h.setRobot(14.25, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 2u) << "preempted onto the RIGHT frontier";
  ASSERT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
  const int64_t stamp_b = h.sel.currentStampNs();
  EXPECT_NE(stamp_a, stamp_b);
  EXPECT_EQ(h.sel.unreachableCount(), 0) << "new commit resets the count";

  h.statusCur(PlannerStatus::kFailed, 4);  // 4 for B, still below the threshold
  EXPECT_TRUE(h.sel.explorationActive());
  EXPECT_EQ(h.currentRecord()->state, FrontierState::ACTIVE);
  h.statusCur(PlannerStatus::kFailed);     // 5th for B -> invalidated
  EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 1);
  EXPECT_FALSE(h.sel.explorationActive());
}

// =================================================================================================
// REACHED -> VISITED -> next goal
// =================================================================================================

// Scenario: REACHED for the current frontier's stamp. The frontier becomes VISITED (even though the
// map still shows the unknown behind it), the active flag clears, and the next tick selects the other
// frontier. Later detection cycles must not bring the visited frontier back.
// Why: spec section 3 "REACHED: mark VISITED, clear active, reset counter".
TEST(GoalSelectorLifecycle, ReachedMarksVisited_ThenNextGoal) {
  Harness h(BaseParams());
  StartOnLeft(h);
  const uint64_t left_id = h.sel.currentExploreId();
  h.statusCur(PlannerStatus::kFailed, 2);

  h.statusCur(PlannerStatus::kReached);
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::VISITED);
  EXPECT_FALSE(h.sel.explorationActive());
  EXPECT_EQ(h.sel.unreachableCount(), 0);

  h.advance(1.0);
  h.cycle();  // grid still shows the left unknown; the visited record absorbs the fresh cluster
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::VISITED);
}

// =================================================================================================
// Stuck watchdog (frontier goals)
// =================================================================================================

// Scenario: a frontier goal is committed and the robot never moves (only sensor jitter well below
// stuck_move_thresh_m, 0.10 m). The watchdog arms only after the first real motion, so even after 60 s
// nothing is invalidated and no new goal appears. (Pursuit timeout disabled to isolate the watchdog.)
// Why: notes B "A robot that never moved does not trip it"; N4: "stuck watchdog ... armed after first
// motion".
TEST(GoalSelectorLifecycle, StuckWatchdog_NeverMovedRobotDoesNotTrip) {
  SelectorParams p = BaseParams();
  p.expl_pursuit_timeout_factor = 0.0;  // disable pursuit timeout
  Harness h(p);
  StartOnLeft(h);
  const uint64_t id = h.sel.currentExploreId();

  h.run(60, 1.0, [&](int i) { h.setRobot(kStartX + ((i % 2) ? 0.05 : 0.0), kStartY); });

  EXPECT_EQ(h.goals.size(), 1u);
  EXPECT_TRUE(h.sel.explorationActive());
  EXPECT_EQ(h.sel.frontierManager().find(id)->state, FrontierState::ACTIVE);
}

// Scenario: the robot moves 0.5 m (> 0.10 m), then stands still (with sub-threshold jitter). After
// stuck_timeout_sec (5 s) without progress the frontier is invalidated and a new goal is published.
// Not before 5 s, not later than one step after.
// Why: notes B "Robot has moved, then moves less than stuck_move_thresh_m for stuck_timeout_sec ->
// new goal"; the watchdog logs and calls markInvalidated.
TEST(GoalSelectorLifecycle, StuckWatchdog_MovedThenStoppedInvalidatesAfterTimeout) {
  SelectorParams p = BaseParams();
  p.expl_pursuit_timeout_factor = 0.0;
  Harness h(p);
  StartOnLeft(h);
  const uint64_t left_id = h.sel.currentExploreId();

  h.advance(1.0);
  h.setRobot(kStartX - 0.5, kStartY);  // first real motion, registered at this cycle
  h.cycle();
  const double t_motion = h.now;
  ASSERT_EQ(h.goals.size(), 1u);

  double fired_at = -1.0;
  for (int i = 1; i <= 40 && fired_at < 0.0; ++i) {  // 0.5 s steps, robot (almost) still
    h.advance(0.5);
    h.setRobot(kStartX - 0.5 + ((i % 2) ? 0.05 : 0.0), kStartY);
    h.cycle();
    if (h.goals.size() > 1u) fired_at = h.now - t_motion;
  }
  ASSERT_GT(fired_at, 0.0) << "watchdog never fired";
  EXPECT_GE(fired_at, 5.0 - 1e-9) << "must not fire before stuck_timeout_sec";
  EXPECT_LE(fired_at, 5.5 + 1e-9) << "must fire as soon as the timeout elapsed";
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "stuck"));
}

// Scenario: continuous slow progress (0.2 m per second, above the 0.10 m threshold) resets the clock
// every time, so the watchdog never fires however long it lasts.
// Why: watchdog semantic "displacement below thresh counts as no progress"; progress resets the clock.
TEST(GoalSelectorLifecycle, StuckWatchdog_ProgressKeepsResettingTheClock) {
  SelectorParams p = BaseParams();
  p.expl_pursuit_timeout_factor = 0.0;
  Harness h(p);
  StartOnLeft(h);
  // 0.2 m per second for 15 s: the robot ends at x = 4.25, still > visit radius (0.3 m) from the frontier.
  h.run(15, 1.0, [&](int i) { h.setRobot(kStartX - 0.2 * (i + 1), kStartY); });
  EXPECT_EQ(h.goals.size(), 1u);
  EXPECT_TRUE(h.sel.explorationActive());
  EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 0);
}

// Scenario: stuck_timeout_sec = 0 disables the watchdog (documented in the YAML: "0 disables").
// A robot that moved and then stopped for 60 s keeps its frontier goal.
TEST(GoalSelectorLifecycle, StuckWatchdog_ZeroTimeoutDisables) {
  SelectorParams p = BaseParams();
  p.expl_pursuit_timeout_factor = 0.0;
  p.expl_stuck_timeout_sec = 0.0;
  Harness h(p);
  StartOnLeft(h);
  h.advance(1.0);
  h.setRobot(kStartX - 0.5, kStartY);
  h.cycle();
  h.run(60, 1.0);
  EXPECT_EQ(h.goals.size(), 1u);
  EXPECT_TRUE(h.sel.explorationActive());
}

// =================================================================================================
// Pursuit budget
// =================================================================================================

// Scenario: when a frontier is selected its pursuit budget is max(pursuit_timeout_min_sec,
// dist / v_ref * factor) with v_ref 0.5, factor 3, floor 10 s. Three robot positions: 1.5 m (6 s of
// formula -> floor 10 s), 5.0 m (30 s) and 7.75 m (46.5 s). The deadline is commit time + budget.
// Why: notes B "Budget = max(min_sec, dist / v_ref x factor)"; FrontierManager::markSelected comment.
TEST(GoalSelectorLifecycle, PursuitBudget_FollowsTheFormula) {
  struct Case {
    double robot_x;
    double expected_budget;
  };
  // LEFT centroid at x = 2.25 is the nearest frontier in all three cases.
  const Case cases[] = {{3.75, 10.0}, {7.25, 30.0}, {10.0, 46.5}};
  for (const auto& c : cases) {
    SCOPED_TRACE("robot_x = " + std::to_string(c.robot_x));
    Harness h(BaseParams());
    h.setMap(OpenBoth());
    h.setRobot(c.robot_x, kStartY);
    h.cycle();
    ASSERT_EQ(h.goals.size(), 1u);
    ASSERT_TRUE(GoalNear(h.goals[0], kLeftX, kCorridorY));
    const FrontierRecord* r = h.currentRecord();
    ASSERT_NE(r, nullptr);
    EXPECT_NEAR(r->pursuit_budget_sec, c.expected_budget, 1e-6);
    EXPECT_NEAR(r->pursuit_deadline_t, h.now + c.expected_budget, 1e-6);
  }
}

// Scenario: a pursuit overruns its budget (30 s). The deadline is only evaluated inside
// FrontierManager::update(), i.e. when a raw occupancy grid arrives. Time far past the deadline with
// only select ticks (no grid) changes nothing; the next grid invalidates the frontier and the
// following tick publishes the other frontier. Just before the deadline the frontier is still alive.
// Why: notes B "Pursuit budget exceeded -> new goal ... it is only evaluated inside update()".
TEST(GoalSelectorLifecycle, PursuitBudget_ExpiryIsEvaluatedOnlyInUpdate) {
  Harness h(BaseParams());
  StartOnLeft(h);  // committed at t = 100, deadline t = 130
  const uint64_t left_id = h.sel.currentExploreId();

  h.advance(29.0);  // t = 129: still inside the budget
  h.state();
  h.feed();
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::ACTIVE);

  h.advance(20.0);  // t = 149: well past the deadline, but no grid has arrived yet
  h.state();
  h.tick();
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::ACTIVE)
      << "select ticks alone must not evaluate the pursuit deadline";
  EXPECT_EQ(h.goals.size(), 1u);

  h.feed();  // grid -> update() -> deadline passed -> INVALIDATED
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  h.tick();
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
}

// Scenario: the same frontier is selected again while its deadline is still armed (here: a
// return-home trigger releases the frontier without clearing its pursuit; the robot is still at the
// start pose so the home latch re-arms at once and the same frontier is the best again). The new
// commit publishes a new goal, but the old deadline (t0 + 30) is kept, NOT moved to t + 30.
// Why: notes B "the deadline is not re-armed when the same frontier is re-selected" (markSelected
// "Don't clobber a deadline that's already armed"). ASSUMPTION: the return-home route is a legitimate
// way to re-select an armed frontier (the trigger does not call clearPursuit).
TEST(GoalSelectorLifecycle, PursuitDeadline_NotRearmedWhenSameFrontierReselected) {
  Harness h(BaseParams());
  StartOnLeft(h);  // t0 = 100, deadline 130
  const uint64_t left_id = h.sel.currentExploreId();
  const double deadline = h.currentRecord()->pursuit_deadline_t;
  ASSERT_NEAR(deadline, 130.0, 1e-6);

  h.advance(5.0);
  h.returnHome();  // robot still at the start pose
  ASSERT_EQ(h.goals.size(), 2u);
  ASSERT_EQ(h.goals[1].kind, GoalKind::kReturnHome);

  h.advance(5.0);  // t = 110
  h.cycle();       // at home -> latch released -> best frontier (LEFT again)
  ASSERT_EQ(h.goals.size(), 3u);
  EXPECT_EQ(h.goals[2].kind, GoalKind::kFrontier);
  EXPECT_EQ(h.sel.currentExploreId(), left_id) << "same frontier re-selected";
  EXPECT_NEAR(h.currentRecord()->pursuit_deadline_t, deadline, 1e-6)
      << "deadline must not be re-armed to 110 + 30";
}

// =================================================================================================
// Preemption
// =================================================================================================

namespace {
SelectorParams PreemptParams() {
  SelectorParams p = BaseParams();
  p.expl_preempt_enabled = true;
  p.expl_preempt_margin = 2.0;
  p.expl_preempt_min_commit_sec = 2.0;
  p.expl_stuck_timeout_sec = 1000.0;  // moving the robot around must not trip the watchdog test
  return p;
}
}  // namespace

// Scenario: preemption is enabled but the robot jumps right after the commit so that RIGHT is far
// better (utility gap 8 > margin 2). Within preempt_min_commit_sec (2 s) the commitment holds; once
// 2 s have passed the selector switches.
// Why: notes B "Preemption: only after preempt_min_commit_sec".
TEST(GoalSelectorLifecycle, Preempt_OnlyAfterMinCommitTime) {
  Harness h(PreemptParams());
  StartOnLeft(h);  // committed at t = 100

  h.advance(1.0);
  h.setRobot(14.25, kStartY);  // LEFT 12.0 m, RIGHT 4.0 m
  h.cycle();
  EXPECT_EQ(h.goals.size(), 1u) << "1 s after the commit: inside the commit dwell, hold";

  h.advance(1.5);  // t = 102.5, 2.5 s after the commit
  h.cycle();
  ASSERT_EQ(h.goals.size(), 2u) << "after min commit time the better frontier preempts";
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
}

// Scenario: after the min commit time, a better frontier that only beats the current one by 1 m
// (margin 2 m) does not preempt; one that beats it by 4 m does.
// Why: notes B "only when the new utility beats the current one by preempt_margin". Utility is -dist.
TEST(GoalSelectorLifecycle, Preempt_RequiresUtilityMargin) {
  Harness h(PreemptParams());
  StartOnLeft(h);

  h.advance(3.0);
  h.setRobot(10.75, kStartY);  // LEFT 8.5 m, RIGHT 7.5 m -> gap 1 < 2
  h.cycle();
  EXPECT_EQ(h.goals.size(), 1u) << "gap below the margin: hold";

  h.advance(1.0);
  h.setRobot(12.25, kStartY);  // LEFT 10.0 m, RIGHT 6.0 m -> gap 4 > 2
  h.cycle();
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
}

// Scenario: after a preemption the abandoned frontier is released, not invalidated: still ACTIVE, its
// pursuit deadline cleared, and it is selectable again (here: robot drives back and it preempts back).
// Why: notes B "the old frontier is released (clearPursuit), not invalidated".
TEST(GoalSelectorLifecycle, Preempt_ReleasedFrontierIsNotInvalidated) {
  Harness h(PreemptParams());
  StartOnLeft(h);
  const uint64_t left_id = h.sel.currentExploreId();
  ASSERT_GT(h.currentRecord()->pursuit_deadline_t, 0.0);

  h.advance(3.0);
  h.setRobot(14.25, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 2u);

  const FrontierRecord* left = h.sel.frontierManager().find(left_id);
  ASSERT_NE(left, nullptr);
  EXPECT_EQ(left->state, FrontierState::ACTIVE);
  EXPECT_LE(left->pursuit_deadline_t, 0.0) << "pursuit deadline cleared (clearPursuit)";
  EXPECT_EQ(h.countRecords(FrontierState::INVALIDATED), 0);

  h.advance(3.0);
  h.setRobot(3.75, kStartY);  // back near LEFT: gap large the other way
  h.cycle();
  ASSERT_EQ(h.goals.size(), 3u);
  EXPECT_TRUE(GoalNear(h.goals[2], kLeftX, kCorridorY)) << "released frontier can be chosen again";
}

// Scenario: preemption disabled (the default). The same big utility gap, long after the commit, does
// not change the commitment (legacy hard-commit).
// Why: notes B "with preemption disabled the commitment holds".
TEST(GoalSelectorLifecycle, Preempt_DisabledHoldsTheCommitment) {
  SelectorParams p = PreemptParams();
  p.expl_preempt_enabled = false;
  Harness h(p);
  StartOnLeft(h);

  h.advance(10.0);
  h.setRobot(14.25, kStartY);
  h.cycle();
  h.run(10, 1.0);
  EXPECT_EQ(h.goals.size(), 1u);
  EXPECT_TRUE(GoalNear(h.goals[0], kLeftX, kCorridorY));
  EXPECT_TRUE(h.sel.explorationActive());
}

// =================================================================================================
// Return home
// =================================================================================================

// Scenario: the only frontier (LEFT) is explored: the map update shows the left end closed and free
// of unknown, the frontier record turns VISITED through the map verification while the goal is still
// the active one, and nothing else exists. The next tick commits a return-home goal at the captured
// start pose (z = expl_default_goal_z), with a new stamp, and sets the return-home latch.
// Why: notes B "No frontiers left -> return home"; spec 13 "returns home the first moment no
// ACTIVE/DORMANT frontier exists, stays latched until the robot reaches the start".
TEST(GoalSelectorLifecycle, ReturnHome_WhenNoFrontiersLeft) {
  SelectorParams p = BaseParams();
  p.expl_default_goal_z = 0.4;
  Harness h(p);
  h.setMap(LeftOnly());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 1u);
  ASSERT_TRUE(GoalNear(h.goals[0], kLeftX, kCorridorY));
  const int64_t frontier_stamp = h.sel.currentStampNs();

  h.advance(1.0);
  h.setRobot(3.75, kStartY);  // drove toward the frontier, 1.5 m away from it
  h.setMap(ExploredLeft());
  h.cycle();

  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_EQ(h.goals[1].kind, GoalKind::kReturnHome);
  EXPECT_NEAR(h.goals[1].position.x(), kStartX, 1e-9);
  EXPECT_NEAR(h.goals[1].position.y(), kStartY, 1e-9);
  EXPECT_NEAR(h.goals[1].position.z(), 0.4, 1e-9);
  EXPECT_GT(h.goals[1].stamp_ns, frontier_stamp);
  EXPECT_TRUE(h.sel.homeReturnRequested());
  EXPECT_FALSE(h.sel.explorationActive());
}

// Scenario: after the return-home commit the robot is en route (outside goal_radius of the start).
// A new frontier appears (the right door opens). The latch holds: no frontier goal and no second
// return-home goal while en route. When the robot arrives (within goal_radius 0.5 of the start) the
// latch is released and the new frontier is committed.
// Why: notes B "the latch holds while en route and re-arms on arrival" (current behaviour; spec 13
// says it may change later).
TEST(GoalSelectorLifecycle, ReturnHome_LatchHoldsEnRouteAndRearmsOnArrival) {
  Harness h(BaseParams());
  h.setMap(LeftOnly());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  h.advance(1.0);
  h.setRobot(3.75, kStartY);
  h.setMap(ExploredLeft());
  h.cycle();
  ASSERT_EQ(h.goals.size(), 2u);
  ASSERT_EQ(h.goals[1].kind, GoalKind::kReturnHome);

  // en route, a new frontier appears on the right (door opened)
  h.setMap(RightOpened());
  for (int i = 0; i < 5; ++i) {
    h.advance(1.0);
    h.setRobot(3.75 + 0.5 * (i + 1), kStartY);  // 4.25 .. 6.25: still > 0.5 m from the start 7.25
    h.cycle();
  }
  EXPECT_EQ(h.goals.size(), 2u) << "latched: nothing new while en route";
  EXPECT_TRUE(h.sel.homeReturnRequested());
  const FrontierRecord* right2 = h.recordNear(kRight2X, kCorridorY);
  ASSERT_NE(right2, nullptr);
  EXPECT_LT(std::hypot(right2->centroid_xy.x() - kRight2X, right2->centroid_xy.y() - kCorridorY), 0.5)
      << "the new frontier is known meanwhile";

  h.advance(1.0);
  h.setRobot(kStartX - 0.3, kStartY);  // within goal_radius
  h.cycle();
  EXPECT_FALSE(h.sel.homeReturnRequested());
  ASSERT_EQ(h.goals.size(), 3u) << "re-armed: the new frontier is committed";
  EXPECT_EQ(h.goals[2].kind, GoalKind::kFrontier);
  EXPECT_TRUE(GoalNear(h.goals[2], kRight2X, kCorridorY));
}

// Scenario: same re-arm, but no frontier exists on arrival: the latch is released and the selector
// stays idle (no goal published).
// Why: spec 13 / code comment "will resume if frontiers reappear". ASSUMPTION: idle, no repeated
// return-home goal, once the robot is parked at the start.
TEST(GoalSelectorLifecycle, ReturnHome_ArrivalWithoutFrontiersStaysIdle) {
  Harness h(BaseParams());
  h.setMap(LeftOnly());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  h.advance(1.0);
  h.setRobot(3.75, kStartY);
  h.setMap(ExploredLeft());
  h.cycle();
  ASSERT_EQ(h.goals.size(), 2u);

  h.advance(1.0);
  h.setRobot(kStartX, kStartY);
  h.cycle();
  h.run(5, 1.0);
  EXPECT_FALSE(h.sel.homeReturnRequested());
  EXPECT_EQ(h.goals.size(), 2u);
}

// Scenario: the LAST frontier is completed by REACHED from the planner (not by the map clearing it).
// Expected: with no frontier left the selector goes home just the same.
// Why: notes B "No frontiers left -> return home"; spec 13 "returns home the first moment no
// ACTIVE/DORMANT frontier exists". ASSUMPTION: the way the last frontier disappears (REACHED vs map
// update) must not matter. (Current code only returns home if exploration_active_ is still set, and
// REACHED clears it; if this test fails that is the finding.)
TEST(GoalSelectorLifecycle, ReturnHome_AlsoAfterLastFrontierReached) {
  Harness h(BaseParams());
  h.setMap(LeftOnly());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 1u);

  h.statusCur(PlannerStatus::kReached);
  h.advance(1.0);
  h.tick();
  ASSERT_EQ(h.goals.size(), 2u) << "no frontier left after REACHED -> return home";
  EXPECT_EQ(h.goals[1].kind, GoalKind::kReturnHome);
}

// Scenario: the operator publishes /exploration/return_home while a frontier is being pursued. A
// return-home goal at the start pose is published; the active frontier is dropped; later select ticks
// (robot en route, frontiers still available) publish no new frontier goals; a second trigger while
// returning is ignored.
// Why: notes B "return_home trigger stops new frontier goals".
TEST(GoalSelectorLifecycle, ReturnHome_TriggerStopsNewFrontierGoals) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.advance(2.0);
  h.setRobot(5.0, kStartY);  // 2.25 m from the start pose
  h.cycle();

  h.returnHome();
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_EQ(h.goals[1].kind, GoalKind::kReturnHome);
  EXPECT_NEAR(h.goals[1].position.x(), kStartX, 1e-9);
  EXPECT_FALSE(h.sel.explorationActive());

  h.run(10, 1.0, [&](int i) { h.setRobot(5.0 + 0.1 * i, kStartY); });  // still far from home
  h.returnHome();                                                         // repeated trigger
  EXPECT_EQ(h.goals.size(), 2u);
  EXPECT_EQ(h.goalsOfKind(GoalKind::kFrontier), 1);
}

// Scenario: return_home is triggered before any exploration goal was committed (no start captured).
// Expected: warning, no goal.
// Why: code/spec: no start pose to return to. ASSUMPTION: ignoring is the intended behaviour.
TEST(GoalSelectorLifecycle, ReturnHome_TriggerBeforeExplorationStartedIsIgnored) {
  Harness h(BaseParams());
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.returnHome();
  EXPECT_TRUE(h.goals.empty());
  EXPECT_FALSE(h.sel.homeReturnRequested());
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "Return-home"));
}

// =================================================================================================
// Manual goals
// =================================================================================================

// Scenario: a frontier is being pursued, then a manual goal arrives. The manual goal is published at
// once, unchanged, with a new stamp; exploration stops: while it is active no frontier is selected
// (even with preemption on and a much better frontier), and statuses of the old frontier stamp are
// ignored. REACHED for the manual stamp releases the override and exploration resumes.
// Why: spec 4.2 "While the manual goal is active, no frontier is selected, preempted ... It is
// released when REACHED arrives for its stamp. Exploration then resumes."
TEST(GoalSelectorLifecycle, Manual_SuppressesExplorationUntilReached) {
  SelectorParams p = PreemptParams();
  Harness h(p);
  StartOnLeft(h);
  const int64_t frontier_stamp = h.sel.currentStampNs();

  h.advance(1.0);
  h.manual(9.0, 2.0, 0.0);
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_EQ(h.goals[1].kind, GoalKind::kManual);
  EXPECT_TRUE(h.sel.manualGoalActive());
  EXPECT_FALSE(h.sel.explorationActive());
  EXPECT_NE(h.goals[1].stamp_ns, frontier_stamp);

  // robot creeps along (so neither the start nor the stuck timeout fires) toward a spot where RIGHT
  // is far better than LEFT; nothing may be selected or preempted.
  h.run(8, 1.0, [&](int i) { h.setRobot(kStartX + 0.5 * (i + 1), kStartY); });
  EXPECT_EQ(h.goals.size(), 2u);

  // old frontier's status must not release or affect anything
  h.status(PlannerStatus::kReached, frontier_stamp);
  EXPECT_TRUE(h.sel.manualGoalActive());

  h.statusCur(PlannerStatus::kReached);  // REACHED for the manual stamp
  EXPECT_FALSE(h.sel.manualGoalActive());
  h.advance(1.0);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 3u) << "exploration resumed";
  EXPECT_EQ(h.goals[2].kind, GoalKind::kFrontier);
}

// Scenario: a manual goal on a wall cell (row 0, occupied), one inside real unknown and one at an odd z
// are each published exactly as given: same x, y, z; kind manual; nonzero stamp. No relocation.
// Why: spec 11.15 "No goal relocation anywhere ... A goal on an occupied or occupied-band cell gives
// FAILED" and N5 "Manual goals are published as given".
TEST(GoalSelectorLifecycle, Manual_PublishedUnchangedEvenOnObstacleOrUnknown) {
  struct G {
    double x, y, z;
  };
  const G targets[] = {{3.25, 0.25, 0.0},   // wall cell
                       {0.75, 1.75, 0.0},   // real unknown
                       {9.9, 1.1, 0.37}};   // free, odd z
  for (const auto& t : targets) {
    SCOPED_TRACE("goal " + std::to_string(t.x) + "," + std::to_string(t.y));
    Harness h(BaseParams());
    h.setMap(OpenBoth());
    h.setRobot(kStartX, kStartY);
    h.cycle();
    h.advance(1.0);
    h.manual(t.x, t.y, t.z);
    ASSERT_EQ(h.goals.size(), 2u);
    const GoalCommand& g = h.goals.back();
    EXPECT_EQ(g.kind, GoalKind::kManual);
    EXPECT_DOUBLE_EQ(g.position.x(), t.x);
    EXPECT_DOUBLE_EQ(g.position.y(), t.y);
    EXPECT_DOUBLE_EQ(g.position.z(), t.z);
    EXPECT_GT(g.stamp_ns, 0);
    EXPECT_DOUBLE_EQ(h.sel.currentTarget().x(), t.x);
  }
}

// Scenario: the planner reports FAILED for the manual goal at least unreachable_consec_thresh (5)
// times. The selector only warns and flags manualGoalUnreachable(); the goal stays active, it is not
// resent or replaced, and exploration does not resume.
// Why: D8a "An unreachable manual goal is only reported, never released automatically"; spec 4.2;
// notes B "Manual goal, unreachable_consec_thresh FAILED in a row: the selector only warns".
TEST(GoalSelectorLifecycle, Manual_FailedThresholdOnlyWarns) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.advance(1.0);
  h.manual(3.25, 0.25);  // on a wall, will FAIL forever
  const size_t n_goals = h.goals.size();

  h.statusCur(PlannerStatus::kFailed, 4);
  EXPECT_FALSE(h.sel.manualGoalUnreachable());
  h.statusCur(PlannerStatus::kFailed, 10);
  EXPECT_TRUE(h.sel.manualGoalUnreachable());
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "unreachable"));
  EXPECT_TRUE(h.sel.manualGoalActive()) << "D8a: never released for FAILED";
  EXPECT_EQ(h.goals.size(), n_goals) << "not resent";

  // robot moves a bit each second (so neither timeout fires within 8 s)
  h.run(8, 1.0, [&](int i) { h.setRobot(kStartX - 0.3 * (i + 1), kStartY); });
  EXPECT_TRUE(h.sel.manualGoalActive());
  EXPECT_EQ(h.goals.size(), n_goals) << "exploration did not resume";
}

// Scenario: manual start timeout (15 s). The robot sits still (< stuck_move_thresh_m 0.10 m of jitter)
// after a manual commit. At 14 s the goal is still active; at 15 s it is released, a warning is
// logged and, with exploration enabled, a frontier goal is committed in the same state update.
// Why: spec 11.21 "On a manual commit the timer starts at once; if the robot has not moved more than
// stuck_move_thresh_m from the commit pose within that time, the manual goal is released like a stuck
// release"; N4 Resolved "resumes exploration if exploration is enabled".
TEST(GoalSelectorLifecycle, Manual_StartTimeoutReleasesAndExplorationResumes) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.advance(1.0);
  const double t_commit = h.now;
  h.manual(9.0, 1.75);
  const size_t n_goals = h.goals.size();

  h.now = t_commit + 14.0;
  h.setRobot(kStartX + 0.05, kStartY);  // jitter only
  h.state();
  EXPECT_TRUE(h.sel.manualGoalActive());

  h.now = t_commit + 15.0;
  h.state();
  EXPECT_FALSE(h.sel.manualGoalActive());
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "start timeout"));
  ASSERT_EQ(h.goals.size(), n_goals + 1) << "exploration resumed in the same onState";
  EXPECT_EQ(h.goals.back().kind, GoalKind::kFrontier);
}

// Scenario: same start timeout with exploration disabled. The goal is released and the selector goes
// idle: nothing new is published.
// Why: N4 Resolved "otherwise stays idle"; spec 11.19 manual goals work with exploration off.
TEST(GoalSelectorLifecycle, Manual_StartTimeoutWithExplorationOffGoesIdle) {
  SelectorParams p = BaseParams();
  p.expl_enabled = false;
  Harness h(p);
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  h.manual(9.0, 1.75);
  ASSERT_EQ(h.goals.size(), 1u);

  h.now += 15.0;
  h.state();
  EXPECT_FALSE(h.sel.manualGoalActive());
  h.tick();
  h.feed();
  EXPECT_EQ(h.goals.size(), 1u) << "idle: nothing new published";
}

// Scenario: the robot moves more than stuck_move_thresh_m soon after the manual commit, which cancels
// the start check; it then stops. The normal stuck watchdog (5 s) applies, so the goal is released
// ~5 s after the last progress, long before the 15 s start timeout would have been due again.
// Why: spec 11.21 "After the first motion the normal stuck watchdog applies"; 11.17 / N4 Resolved.
TEST(GoalSelectorLifecycle, Manual_StuckTimeoutAfterFirstMotion) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.advance(1.0);
  const double t0 = h.now;
  h.manual(9.0, 1.75);

  h.now = t0 + 3.0;
  h.setRobot(kStartX + 0.5, kStartY);  // first motion
  h.state();
  ASSERT_TRUE(h.sel.manualGoalActive());

  h.now = t0 + 7.5;  // 4.5 s since the last progress
  h.state();
  EXPECT_TRUE(h.sel.manualGoalActive());

  h.now = t0 + 8.5;  // 5.5 s since the last progress
  h.state();
  EXPECT_FALSE(h.sel.manualGoalActive());
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "stuck"));
}

// Scenario: manual_start_timeout_sec = 0 disables the start check. A robot that never moves keeps the
// manual goal forever (the stuck watchdog only arms after first motion).
// Why: spec 11.21 / YAML "0 disables this start check; afterwards the normal stuck_timeout_sec applies".
TEST(GoalSelectorLifecycle, Manual_StartTimeoutZeroDisablesCheck) {
  SelectorParams p = BaseParams();
  p.manual_start_timeout_sec = 0.0;
  Harness h(p);
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  h.manual(9.0, 1.75);
  h.now += 600.0;
  h.state();
  EXPECT_TRUE(h.sel.manualGoalActive());
}

// Scenario: a manual goal interrupts a frontier pursuit. The frontier's pursuit deadline is cleared
// (it stays ACTIVE and is not blamed). After the manual goal completes the frontier is selected
// again with a fresh deadline and is NOT invalidated at the old deadline time.
// Why: spec 11.10 "A manual goal clears the interrupted frontier's pursuit deadline".
TEST(GoalSelectorLifecycle, Manual_ClearsInterruptedFrontiersPursuitDeadline) {
  Harness h(BaseParams());
  StartOnLeft(h);  // deadline = 130
  const uint64_t left_id = h.sel.currentExploreId();
  ASSERT_NEAR(h.currentRecord()->pursuit_deadline_t, 130.0, 1e-6);

  h.advance(5.0);  // t = 105
  h.manual(9.0, 1.75);
  const FrontierRecord* left = h.sel.frontierManager().find(left_id);
  EXPECT_LE(left->pursuit_deadline_t, 0.0);
  EXPECT_EQ(left->state, FrontierState::ACTIVE);

  h.advance(5.0);  // t = 110
  h.statusCur(PlannerStatus::kReached);
  h.cycle();       // frontier re-selected, new deadline 140
  ASSERT_EQ(h.goals.size(), 3u);
  ASSERT_EQ(h.sel.currentExploreId(), left_id);
  EXPECT_NEAR(h.currentRecord()->pursuit_deadline_t, 140.0, 1e-6);

  h.now = 131.0;  // past the OLD deadline
  h.state();
  h.feed();
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::ACTIVE);
}

// =================================================================================================
// Keep-out around invalidated frontiers
// =================================================================================================

// Scenario: the only frontier (LEFT) is invalidated by 5 FAILED. While the 30 s cooldown lasts the
// detector keeps seeing the same unknown edge but no new ACTIVE record is spawned inside the 1.5 m
// keep-out, so it is not re-selected: no new goals. After the cooldown the area may be re-explored:
// a fresh record appears and is selected again.
// Why: notes B "An invalidated frontier is not immediately re-selected (keep-out)"; notes C
// "...once the invalidation keep-out has expired (invalidation_cooldown_sec); not before".
TEST(GoalSelectorLifecycle, KeepOut_InvalidatedFrontierNotImmediatelyReselected) {
  Harness h(BaseParams());
  h.setMap(LeftOnly());
  h.setRobot(kStartX, kStartY);
  h.cycle();
  ASSERT_EQ(h.goals.size(), 1u);
  h.statusCur(PlannerStatus::kFailed, 5);  // invalidated at t = 100
  ASSERT_EQ(h.countRecords(FrontierState::INVALIDATED), 1);

  h.run(20, 1.0);  // t = 120, inside the cooldown
  EXPECT_EQ(h.goals.size(), 1u) << "no re-selection inside the keep-out window";
  EXPECT_EQ(h.countRecords(FrontierState::ACTIVE), 0);

  h.run(15, 1.0);  // t = 135 > 100 + 30
  ASSERT_EQ(h.goals.size(), 2u) << "keep-out expired: the area is a candidate again";
  EXPECT_TRUE(GoalNear(h.goals[1], kLeftX, kCorridorY));
}

// Scenario: two frontiers, LEFT invalidated while the robot stays nearer to LEFT. Every following cycle
// within the cooldown selects RIGHT, never LEFT.
// Why: same as above; the keep-out must outrank "nearest frontier wins".
TEST(GoalSelectorLifecycle, KeepOut_NearestButInvalidatedFrontierIsSkipped) {
  Harness h(BaseParams());
  StartOnLeft(h);
  h.statusCur(PlannerStatus::kFailed, 5);
  h.tick();
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_TRUE(GoalNear(h.goals[1], kRightX, kCorridorY));
  // stay put near LEFT, keep feeding maps for 20 s: still on RIGHT, no new goals toward LEFT
  h.run(20, 1.0);
  for (const auto& g : h.goals)
    if (&g != &h.goals[0]) EXPECT_FALSE(GoalNear(g, kLeftX, kCorridorY));
}
