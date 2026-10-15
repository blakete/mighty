// Stale-map pause (map_max_age_sec): while the latest occ_2d header stamp is older than
// map_max_age_sec, the planner holds the robot (map_hold), so the selector commits no new goal and
// holds the stuck / pursuit-timeout clocks of the current frontier pursuit and of a manual goal --
// otherwise the outage itself would release them as "stuck" or timed out. Covers both failure modes:
// grids stop (silence), and grids keep arriving with a frozen stamp (dead lidar / DLIO: the
// elevation mapper republishes on its timer with the last cloud stamp).
//
// World: the OpenBoth corridor of goal_selector_test_utils.hpp (LEFT frontier (2.25, 1.75), RIGHT
// frontier (18.25, 1.75), robot at (7.25, 1.75) -> LEFT is nearer and wins).

#include <gtest/gtest.h>

#include "goal_selector/goal_fence.hpp"
#include "goal_selector_test_utils.hpp"

using namespace gs_test;
using goal_selector::FencePoint;
using goal_selector::GoalFence;
using goal_selector::Transform2D;

namespace {

constexpr double kMaxAge = 1.5;
constexpr double kLatency = 0.43;  // JR02: occ_2d stamp age at 5 Hz

SelectorParams MapAgeParams() {
  SelectorParams p = BaseParams();
  p.map_max_age_sec = kMaxAge;
  return p;
}

/** @brief OpenBoth corridor, robot at the default start (Harness is not movable: set up in place). */
void InitCorridor(Harness& h) {
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
}

/** @brief One normal cycle with a fresh grid (stamped kLatency ago). */
void FreshCycle(Harness& h) {
  h.setMapStamp(h.now - kLatency);
  h.cycle();
}

}  // namespace

TEST(GoalSelectorMapAge, FreshMapsCommitNormally) {
  Harness h(MapAgeParams());
  InitCorridor(h);
  FreshCycle(h);
  EXPECT_FALSE(h.sel.mapPaused());
  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
}

TEST(GoalSelectorMapAge, StaleMapPausesCommitsThenResumes) {
  Harness h(MapAgeParams());
  InitCorridor(h);
  h.setMapStamp(h.now - 3.0);  // the only grid is already 3 s old
  h.cycle();
  EXPECT_TRUE(h.sel.mapPaused());
  EXPECT_TRUE(h.goals.empty());
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "Maps stale"));

  h.advance(1.0);
  FreshCycle(h);
  EXPECT_FALSE(h.sel.mapPaused());
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kInfo, "Maps fresh again"));
  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
}

TEST(GoalSelectorMapAge, FrozenStampPausesWhileGridsKeepArriving) {
  Harness h(MapAgeParams());
  InitCorridor(h);
  FreshCycle(h);
  ASSERT_EQ(h.goals.size(), 1u);
  const double frozen = h.now - kLatency;
  h.setMapStamp(frozen);
  for (int i = 0; i < 15; ++i) {  // 3 s of grids at 5 Hz, all carrying the frozen stamp
    h.advance(0.2);
    h.cycle();
  }
  EXPECT_TRUE(h.sel.mapPaused());
  EXPECT_EQ(h.goals.size(), 1u);
}

TEST(GoalSelectorMapAge, StaleMapHoldsPursuitClocks) {
  // The fence-pause scenario of GoalSelectorGeofence.PauseStopsPursuitClocks with a map outage: the
  // robot had moved (stuck watchdog armed), then the maps froze for 40 s (longer than the stuck
  // timeout and the 30 s pursuit budget) while the planner held it.
  Harness h(MapAgeParams());
  InitCorridor(h);
  FreshCycle(h);
  ASSERT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
  const auto left_id = h.sel.currentExploreId();
  h.advance(1.0);
  h.setRobot(kStartX - 1.0, kStartY);
  FreshCycle(h);
  ASSERT_EQ(h.sel.currentExploreId(), left_id);

  h.setMapStamp(h.now - kLatency);  // frozen from here
  for (int i = 0; i < 40; ++i) {
    h.advance(1.0);
    h.cycle();
  }
  ASSERT_TRUE(h.sel.mapPaused());
  h.advance(1.0);
  FreshCycle(h);
  EXPECT_FALSE(h.sel.mapPaused());
  EXPECT_EQ(h.sel.currentExploreId(), left_id);
  EXPECT_NE(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_FALSE(h.hasLog(LogMessage::Level::kWarn, "stuck"));

  // The clocks run again after resume: no motion for longer than the stuck timeout -> released.
  for (int i = 0; i < 6; ++i) {
    h.advance(1.0);
    FreshCycle(h);
  }
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "stuck"));
}

TEST(GoalSelectorMapAge, StaleMapKeepsManualGoal) {
  // A C2 manual goal, the robot never got going, maps stale for 20 s (> the 15 s start timeout).
  Harness h(MapAgeParams());
  InitCorridor(h);
  FreshCycle(h);
  h.manual(9.0, 2.0, 0.0);
  ASSERT_TRUE(h.sel.manualGoalActive());

  h.setMapStamp(h.now - kLatency);
  for (int i = 0; i < 20; ++i) {
    h.advance(1.0);
    h.cycle();
  }
  EXPECT_TRUE(h.sel.manualGoalActive());
  EXPECT_FALSE(h.hasLog(LogMessage::Level::kWarn, "start timeout"));

  // Clocks run again once the maps are fresh: still no motion after 16 s -> released.
  for (int i = 0; i < 16; ++i) {
    h.advance(1.0);
    FreshCycle(h);
  }
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "start timeout"));
  EXPECT_FALSE(h.sel.manualGoalActive());
}

TEST(GoalSelectorMapAge, ManualGoalSurvivesStateDropout) {
  // Dead DLIO: maps freeze AND state stops; only the select timer (and the frozen grids) tick. On
  // recovery the fresh grid arrives before the first state, so the select tick must have kept the
  // manual goal's stuck clock held.
  Harness h(MapAgeParams());
  InitCorridor(h);
  FreshCycle(h);
  h.manual(9.0, 2.0, 0.0);
  ASSERT_TRUE(h.sel.manualGoalActive());
  h.advance(1.0);
  h.setRobot(kStartX + 1.0, kStartY);  // moved: start check passed, stuck watchdog armed
  FreshCycle(h);
  ASSERT_TRUE(h.sel.manualGoalActive());

  h.setMapStamp(h.now - kLatency);
  for (int i = 0; i < 10; ++i) {  // 10 s: no state, frozen grids + select ticks only
    h.advance(1.0);
    h.feed();
    h.tick();
  }
  ASSERT_TRUE(h.sel.mapPaused());

  h.advance(1.0);
  h.setMapStamp(h.now - kLatency);
  h.feed();
  h.tick();   // fresh grid first: the pause ends here
  h.state();  // then the first state after the outage
  EXPECT_FALSE(h.sel.mapPaused());
  EXPECT_TRUE(h.sel.manualGoalActive());
  EXPECT_FALSE(h.hasLog(LogMessage::Level::kWarn, "stuck"));
}

TEST(GoalSelectorMapAge, ManualGoalHeldWithExplorationOffAndSilentMaps) {
  // Exploration off and the mapper silent: no grids, so no select ticks either (the node ticks on
  // each occ_2d); only states arrive. The manual goal's start check runs in onState and must hold.
  SelectorParams p = MapAgeParams();
  p.expl_enabled = false;
  Harness h(p);
  InitCorridor(h);
  FreshCycle(h);
  h.manual(9.0, 1.75);
  ASSERT_TRUE(h.sel.manualGoalActive());

  for (int i = 0; i < 20; ++i) {  // 20 s of states only (> the 15 s start timeout)
    h.advance(1.0);
    h.state();
  }
  EXPECT_TRUE(h.sel.mapPaused());
  EXPECT_TRUE(h.sel.manualGoalActive());
  EXPECT_FALSE(h.hasLog(LogMessage::Level::kWarn, "start timeout"));
}

TEST(GoalSelectorMapAge, MapPauseThenFencePauseKeepsThePursuit) {
  // 20 s map outage straight into 20 s of an enforcing-but-not-ready fence (40 s > the 30 s pursuit
  // budget): both pauses hold the same clocks, the frontier survives.
  Harness h(MapAgeParams());
  InitCorridor(h);
  FreshCycle(h);
  const auto left_id = h.sel.currentExploreId();
  h.advance(1.0);
  h.setRobot(kStartX - 1.0, kStartY);
  FreshCycle(h);
  ASSERT_EQ(h.sel.currentExploreId(), left_id);

  h.setMapStamp(h.now - kLatency);
  for (int i = 0; i < 20; ++i) {
    h.advance(1.0);
    h.cycle();
  }
  ASSERT_TRUE(h.sel.mapPaused());

  GoalFence f;
  f.active = true;
  f.ready = false;
  for (int i = 0; i < 20; ++i) {
    h.advance(1.0);
    h.sel.onGoalFence(h.now, f, Transform2D{});
    FreshCycle(h);
  }
  EXPECT_FALSE(h.sel.mapPaused());

  GoalFence ready;
  ready.active = true;
  ready.ready = true;
  ready.offset_m = 0.36;
  ready.keep_in = std::vector<FencePoint>{{-10.0, -5.0}, {40.0, -5.0}, {40.0, 10.0}, {-10.0, 10.0}};
  h.advance(1.0);
  h.sel.onGoalFence(h.now, ready, Transform2D{});
  FreshCycle(h);
  EXPECT_EQ(h.sel.currentExploreId(), left_id);
  EXPECT_NE(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_FALSE(h.hasLog(LogMessage::Level::kWarn, "stuck"));
}

TEST(GoalSelectorMapAge, NoStampNeverPauses) {
  // Grids without a stamp (older callers, tests): the check never engages.
  Harness h(MapAgeParams());
  InitCorridor(h);
  for (int i = 0; i < 10; ++i) {
    h.advance(1.0);
    h.cycle();
  }
  EXPECT_FALSE(h.sel.mapPaused());
  EXPECT_FALSE(h.goals.empty());
}

TEST(GoalSelectorMapAge, OffByDefaultInTheCore) {
  Harness h(BaseParams());  // map_max_age_sec 0: off
  InitCorridor(h);
  h.setMapStamp(h.now - 100.0);
  h.cycle();
  EXPECT_FALSE(h.sel.mapPaused());
  EXPECT_FALSE(h.goals.empty());
}
