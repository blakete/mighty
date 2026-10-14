// Geofence goal filter (exploration.geofence.*, /<R>/geofence/goal_fence): while the fence is enforcing,
// a frontier is a valid goal only if it is inside keep_in, outside every keep_out and at least offset_m
// (footprint radius + margin) from every fence edge -- the gate's core. Others are INVALIDATED before
// ranking; while the fence is enforcing but not ready, no new goal is committed.
//
// World: the OpenBoth corridor of goal_selector_test_utils.hpp (LEFT frontier (2.25, 1.75), RIGHT
// frontier (18.25, 1.75), robot at (7.25, 1.75) -> LEFT is nearer and wins without a fence).

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "goal_selector/goal_fence.hpp"
#include "goal_selector_test_utils.hpp"

using namespace gs_test;
using goal_selector::FencePoint;
using goal_selector::GoalFence;
using goal_selector::Transform2D;
using goal_selector::goalFenceAllows;
using goal_selector::parseGoalFence;
using Gate = goal_selector::GoalSelector::FenceGate;

namespace {

constexpr double kOffset = 0.36;  // JR02: radius 0.35 + margin 0.01

std::vector<FencePoint> Box(double x0, double x1, double y0, double y1) {
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

/** @brief Active + ready fence with keep_in box [x0, x1] x [-5, 10] (corridor y is 0..3.5). */
GoalFence KeepIn(double x0, double x1, std::vector<std::vector<FencePoint>> keep_out = {}) {
  GoalFence f;
  f.active = true;
  f.ready = true;
  f.offset_m = kOffset;
  f.keep_in = Box(x0, x1, -5.0, 10.0);
  f.keep_out = std::move(keep_out);
  return f;
}

/** @brief OpenBoth corridor, robot at the default start (Harness is not movable: set up in place). */
void InitCorridor(Harness& h) {
  h.setMap(OpenBoth());
  h.setRobot(kStartX, kStartY);
}

}  // namespace

TEST(GoalSelectorGeofence, NoGoFrontierOutsideKeepInIsNeverCommitted) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.sel.onGoalFence(h.now, KeepIn(4.0, 25.0), Transform2D{});  // LEFT (x 2.25) is outside keep_in
  h.cycle();
  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
  for (const auto& g : h.goals) EXPECT_FALSE(GoalNear(g, kLeftX, kCorridorY, 1.0));
  EXPECT_EQ(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kInfo, "outside the geofence goal area"));
}

TEST(GoalSelectorGeofence, OffsetIsHonoured) {
  {
    Harness h(BaseParams());  // edge at 1.90: LEFT is 0.35 m inside, < offset 0.36 -> invalidated
    InitCorridor(h);
    h.sel.onGoalFence(h.now, KeepIn(1.90, 25.0), Transform2D{});
    h.cycle();
    EXPECT_EQ(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
    EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
  }
  {
    Harness h(BaseParams());  // edge at 1.85: 0.40 m inside -> a valid goal
    InitCorridor(h);
    h.sel.onGoalFence(h.now, KeepIn(1.85, 25.0), Transform2D{});
    h.cycle();
    EXPECT_NE(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
}

TEST(GoalSelectorGeofence, KeepOutInvalidates) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.sel.onGoalFence(h.now, KeepIn(-5.0, 25.0, {Box(1.5, 3.0, 1.0, 2.5)}), Transform2D{});
  h.cycle();
  EXPECT_EQ(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
}

TEST(GoalSelectorGeofence, FailsOpen) {
  const GoalFence excludes_left = KeepIn(4.0, 25.0);
  {
    Harness h(BaseParams());  // no message at all
    InitCorridor(h);
    h.cycle();
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
  {
    Harness h(BaseParams());  // older than max_age (2 s)
    InitCorridor(h);
    h.sel.onGoalFence(h.now - 3.0, excludes_left, Transform2D{});
    h.cycle();
    EXPECT_EQ(h.sel.fenceGate(h.now), Gate::kOpen);
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
  {
    Harness h(BaseParams());  // fence not enforcing (shadow / off)
    InitCorridor(h);
    GoalFence f = excludes_left;
    f.active = false;
    h.sel.onGoalFence(h.now, f, Transform2D{});
    h.cycle();
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
  {
    SelectorParams p = BaseParams();  // filter disabled
    p.expl_geofence_enabled = false;
    Harness h(p);
    InitCorridor(h);
    h.sel.onGoalFence(h.now, excludes_left, Transform2D{});
    h.cycle();
    EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
  }
}

TEST(GoalSelectorGeofence, PausesWhileActiveNotReady) {
  Harness h(BaseParams());
  InitCorridor(h);
  GoalFence f;
  f.active = true;
  f.ready = false;
  h.sel.onGoalFence(h.now, f, Transform2D{});
  h.cycle();
  EXPECT_EQ(h.sel.fenceGate(h.now), Gate::kPause);
  EXPECT_TRUE(h.goals.empty()) << "no goal while the enforcing fence is not ready";
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "pausing new exploration goals"));

  h.advance(1.0);
  h.sel.onGoalFence(h.now, KeepIn(4.0, 25.0), Transform2D{});  // ready now; LEFT is outside
  h.cycle();
  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
}

TEST(GoalSelectorGeofence, PauseHoldsCurrentPursuit) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.cycle();  // no fence: LEFT committed
  ASSERT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
  const auto left_id = h.sel.currentExploreId();
  const size_t n_goals = h.goals.size();

  GoalFence f;
  f.active = true;
  f.ready = false;
  for (int i = 0; i < 20; ++i) {  // 20 s of "not ready": longer than the stuck watchdog (5 s)
    h.advance(1.0);
    h.sel.onGoalFence(h.now, f, Transform2D{});
    h.cycle();
  }
  EXPECT_EQ(h.sel.currentExploreId(), left_id);
  EXPECT_NE(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_EQ(h.goals.size(), n_goals) << "no new commits while paused";
}

TEST(GoalSelectorGeofence, PauseStopsPursuitClocks) {
  // JR02 2026-10-14: the robot had moved (stuck watchdog armed), the geofence restarted and the gate
  // held the robot; on resume the held frontier was invalidated as "stuck". The pause must hold the
  // stuck clock and the pursuit deadline (5 m / 0.5 m/s * 3 = 30 s budget here).
  Harness h(BaseParams());
  InitCorridor(h);
  h.cycle();  // no fence: LEFT committed
  ASSERT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
  const auto left_id = h.sel.currentExploreId();
  h.advance(1.0);
  h.setRobot(kStartX - 1.0, kStartY);  // moved 1 m: the stuck watchdog is armed
  h.cycle();
  ASSERT_EQ(h.sel.currentExploreId(), left_id);

  GoalFence f;
  f.active = true;
  f.ready = false;
  for (int i = 0; i < 40; ++i) {  // 40 s held: longer than the stuck timeout and the pursuit budget
    h.advance(1.0);
    h.sel.onGoalFence(h.now, f, Transform2D{});
    h.cycle();
  }
  h.advance(1.0);
  h.sel.onGoalFence(h.now, KeepIn(-10.0, 40.0), Transform2D{});  // ready; LEFT allowed
  h.cycle();
  EXPECT_EQ(h.sel.currentExploreId(), left_id);
  EXPECT_NE(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_FALSE(h.hasLog(LogMessage::Level::kWarn, "stuck"));

  // The clocks run again after resume: no motion for longer than the stuck timeout -> released.
  for (int i = 0; i < 6; ++i) {
    h.advance(1.0);
    h.sel.onGoalFence(h.now, KeepIn(-10.0, 40.0), Transform2D{});
    h.cycle();
  }
  EXPECT_TRUE(h.hasLog(LogMessage::Level::kWarn, "stuck"));
}

TEST(GoalSelectorGeofence, StalePauseFailsOpen) {
  Harness h(BaseParams());
  InitCorridor(h);
  GoalFence f;
  f.active = true;
  f.ready = false;
  h.sel.onGoalFence(h.now - 3.0, f, Transform2D{});  // the geofence went silent
  h.cycle();
  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.front(), kLeftX, kCorridorY));
}

TEST(GoalSelectorGeofence, FilterAppliesOnSelectTickWithoutNewDetection) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.state();
  h.feed();  // detection without a fence
  h.sel.onGoalFence(h.now, KeepIn(4.0, 25.0), Transform2D{});
  h.tick();  // no new detection pass: the select tick itself must drop LEFT
  ASSERT_FALSE(h.goals.empty());
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
}

TEST(GoalSelectorGeofence, UsesTheMapToFenceTransform) {
  // map -> fence: rotate 90 deg, then shift (10, 0):  (x, y) -> (10 - y, x).
  //   LEFT  (2.25, 1.75)  -> (8.25,  2.25)  below the keep_in box -> invalidated
  //   RIGHT (18.25, 1.75) -> (8.25, 18.25)  inside                -> kept
  // Without the transform RIGHT (18.25, 1.75) would be outside the box.
  GoalFence f = KeepIn(0.0, 0.0);
  f.keep_in = Box(5.0, 12.0, 5.0, 25.0);
  Harness h(BaseParams());
  InitCorridor(h);
  h.sel.onGoalFence(h.now, f, Transform2D{10.0, 0.0, M_PI / 2.0});
  h.cycle();
  EXPECT_EQ(h.recordNear(kLeftX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_NE(h.recordNear(kRightX, kCorridorY)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals.front(), kRightX, kCorridorY));
}

TEST(GoalSelectorGeofence, CurrentPursuitTurningNoGoIsDroppedAndReplaced) {
  Harness h(BaseParams());
  InitCorridor(h);
  h.cycle();  // no fence yet: LEFT committed
  ASSERT_TRUE(GoalNear(h.goals.back(), kLeftX, kCorridorY));
  const auto left_id = h.sel.currentExploreId();
  h.advance(1.0);
  h.sel.onGoalFence(h.now, KeepIn(4.0, 25.0), Transform2D{});
  h.cycle();
  EXPECT_EQ(h.sel.frontierManager().find(left_id)->state, FrontierState::INVALIDATED);
  EXPECT_TRUE(GoalNear(h.goals.back(), kRightX, kCorridorY));
}

TEST(GoalSelectorGeofence, GoalFenceAllowsRules) {
  GoalFence f;
  f.active = f.ready = true;
  f.offset_m = 0.8;
  f.keep_in = Box(0.0, 20.0, 0.0, 20.0);
  f.keep_out = {{{8, 8}, {12, 8}, {10, 13}}};
  EXPECT_TRUE(goalFenceAllows(f, 3.0, 3.0));
  // on edges / vertices: never valid
  for (const auto& p : std::vector<FencePoint>{{0, 0}, {10, 0}, {20, 10}, {8, 8}, {10, 8}})
    EXPECT_FALSE(goalFenceAllows(f, p.x, p.y)) << p.x << "," << p.y;
  // offset from a keep_in edge and from a keep_out edge
  EXPECT_FALSE(goalFenceAllows(f, 0.79, 5.0));
  EXPECT_TRUE(goalFenceAllows(f, 0.81, 5.0));
  EXPECT_FALSE(goalFenceAllows(f, 10.0, 7.3));   // 0.7 below the keep_out base
  EXPECT_TRUE(goalFenceAllows(f, 10.0, 7.1));    // 0.9 below
  EXPECT_FALSE(goalFenceAllows(f, 10.0, 10.0));  // inside the keep_out
  EXPECT_FALSE(goalFenceAllows(f, 25.0, 5.0));   // outside keep_in
  // extra margin
  EXPECT_FALSE(goalFenceAllows(f, 1.0, 5.0, 0.3));
  EXPECT_TRUE(goalFenceAllows(f, 1.2, 5.0, 0.3));
  // keep_out crossing the keep_in edge, overlapping keep_outs
  f.keep_out = {Box(-2, 5, 8, 12), Box(4, 9, 10, 14)};
  EXPECT_FALSE(goalFenceAllows(f, 2.0, 10.0));
  EXPECT_FALSE(goalFenceAllows(f, 4.5, 11.0));   // in both keep_outs (even-odd per ring, then OR)
  EXPECT_TRUE(goalFenceAllows(f, 2.0, 4.0));
  // concave keep_in (L): the inner corner keeps its distance
  f.keep_out.clear();
  f.keep_in = {{0, 0}, {20, 0}, {20, 6}, {6, 6}, {6, 20}, {0, 20}};
  EXPECT_FALSE(goalFenceAllows(f, 5.5, 5.5));    // 0.71 m from the inner corner (6, 6)
  EXPECT_TRUE(goalFenceAllows(f, 5.0, 5.0));
  // degenerate keep_in
  f.keep_in = {{0, 0}, {10, 0}};
  EXPECT_FALSE(goalFenceAllows(f, 5.0, 0.0));
}

TEST(GoalSelectorGeofence, ParseGoalFence) {
  GoalFence f;
  std::string frame, err;
  ASSERT_TRUE(parseGoalFence(
      R"({"v":1,"frame_id":"JR02/odom","active":true,"ready":true,"offset_m":0.36,)"
      R"("keep_in":[[0,0],[20,0],[20,20],[0,20]],"keep_out":[[[8,8],[12,8],[10,13]]]})",
      f, frame, err));
  EXPECT_TRUE(f.active && f.ready);
  EXPECT_DOUBLE_EQ(f.offset_m, 0.36);
  EXPECT_EQ(f.keep_in.size(), 4u);
  ASSERT_EQ(f.keep_out.size(), 1u);
  EXPECT_EQ(frame, "JR02/odom");
  EXPECT_TRUE(err.empty());

  EXPECT_FALSE(parseGoalFence("not json", f, frame, err));
  EXPECT_FALSE(parseGoalFence(R"({"ready":true})", f, frame, err)) << "no boolean active";

  // active but unusable -> ready = false (pause), with a reason
  ASSERT_TRUE(parseGoalFence(R"({"frame_id":"o","active":true,"ready":true,"offset_m":0.36,)"
                             R"("keep_in":[[0,0],[1,0]]})", f, frame, err));
  EXPECT_TRUE(f.active);
  EXPECT_FALSE(f.ready);
  EXPECT_FALSE(err.empty());
  ASSERT_TRUE(parseGoalFence(R"({"frame_id":"o","active":true,"ready":true,"offset_m":-1,)"
                             R"("keep_in":[[0,0],[1,0],[1,1]]})", f, frame, err));
  EXPECT_FALSE(f.ready);
  ASSERT_TRUE(parseGoalFence(R"({"active":true,"ready":true,"offset_m":0.3,"keep_in":[[0,0],[1,0],[1,1]]})",
                             f, frame, err));
  EXPECT_FALSE(f.ready) << "no frame_id";
  ASSERT_TRUE(parseGoalFence(R"({"frame_id":"o","active":true,"ready":true,"offset_m":0.3,)"
                             R"("keep_in":[[0,0],[1,0],[1,1]],"keep_out":[[[0,0],["x",1],[1,1]]]})", f, frame, err));
  EXPECT_FALSE(f.ready) << "bad keep_out ring";

  // not active: parsed, no geometry needed
  ASSERT_TRUE(parseGoalFence(R"({"frame_id":"o","active":false,"ready":false})", f, frame, err));
  EXPECT_FALSE(f.active);
  EXPECT_FALSE(f.ready);
}
