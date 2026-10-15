// Map-age check and hold state machine (map2d/map_age.hpp), shared by the planner's map hold and the
// goal selector's stale-map pause. Times are seconds on one clock; max_age 1.5 s as on hardware.

#include <gtest/gtest.h>

#include "map2d/map_age.hpp"

using map2d::MapAgeGuard;
using map2d::stampTooOld;
using Transition = map2d::MapAgeGuard::Transition;

namespace {
constexpr double kMax = 1.5;
constexpr double kT0 = 1000.0;
}  // namespace

TEST(MapAge, StampTooOld) {
  EXPECT_FALSE(stampTooOld(kT0, kT0 - 0.4, kMax));
  EXPECT_FALSE(stampTooOld(kT0, kT0 - kMax, kMax));  // exactly max_age: still fresh
  EXPECT_TRUE(stampTooOld(kT0, kT0 - 1.6, kMax));
  EXPECT_TRUE(stampTooOld(kT0, kT0 + 1.6, kMax));    // stamp from the future (clock step)
  EXPECT_FALSE(stampTooOld(kT0, kT0 - 100.0, 0.0));  // check off
  EXPECT_FALSE(stampTooOld(kT0, kT0 - 100.0, -1.0));
  EXPECT_FALSE(stampTooOld(kT0, 0.0, kMax));         // no map yet
  EXPECT_FALSE(stampTooOld(kT0, -1.0, kMax));
}

TEST(MapAge, FreshMapsNeverHold) {
  MapAgeGuard g;
  for (int i = 0; i < 50; ++i) {
    const double now = kT0 + 0.2 * i;
    EXPECT_EQ(g.update(now, kMax, now - 0.45, now - 0.43), Transition::kNone);
  }
  EXPECT_FALSE(g.held());
  EXPECT_NEAR(g.age(), 0.45, 1e-9);
}

TEST(MapAge, NoMapYetNeverHolds) {
  MapAgeGuard g;
  EXPECT_EQ(g.update(kT0, kMax, 0.0, 0.0), Transition::kNone);
  EXPECT_FALSE(g.held());
}

TEST(MapAge, SilenceEntersOnceThenStaysHeld) {
  MapAgeGuard g;
  const double last = kT0 - 0.43;  // last grid, then nothing arrives
  int enters = 0;
  for (int i = 0; i <= 50; ++i) {  // 10 s at 5 Hz
    if (g.update(kT0 + 0.2 * i, kMax, last) == Transition::kEnterHold) ++enters;
  }
  EXPECT_EQ(enters, 1);
  EXPECT_TRUE(g.held());
  EXPECT_NEAR(g.age(), 10.43, 1e-9);
}

TEST(MapAge, EntersAtTheThresholdNotBefore) {
  MapAgeGuard g;
  const double last = kT0;
  EXPECT_EQ(g.update(last + 1.5, kMax, last), Transition::kNone);
  EXPECT_EQ(g.update(last + 1.6, kMax, last), Transition::kEnterHold);
  EXPECT_NEAR(g.heldFor(last + 2.1), 0.5, 1e-9);
}

TEST(MapAge, UpdateIsIdempotentAtOneInstant) {
  MapAgeGuard g;
  EXPECT_EQ(g.update(kT0, kMax, kT0 - 3.0), Transition::kEnterHold);
  EXPECT_EQ(g.update(kT0, kMax, kT0 - 3.0), Transition::kNone);
  EXPECT_TRUE(g.held());
}

TEST(MapAge, ExitNeedsMinHoldAndMargin) {
  MapAgeGuard g;
  ASSERT_EQ(g.update(kT0, kMax, kT0 - 2.0), Transition::kEnterHold);
  // Fresh at once, but the hold is younger than kMinHoldSec.
  EXPECT_EQ(g.update(kT0 + 0.5, kMax, kT0 + 0.1), Transition::kNone);
  EXPECT_TRUE(g.held());
  // Old enough, but the age (1.1 s) is inside the exit margin [1.0, 1.5].
  EXPECT_EQ(g.update(kT0 + 1.2, kMax, kT0 + 0.1), Transition::kNone);
  EXPECT_TRUE(g.held());
  // Below max_age - 0.5 and held >= 1 s: released.
  EXPECT_EQ(g.update(kT0 + 1.3, kMax, kT0 + 0.9), Transition::kExitHold);
  EXPECT_FALSE(g.held());
  EXPECT_NEAR(g.lastHoldSec(), 1.3, 1e-9);
  EXPECT_DOUBLE_EQ(g.heldFor(kT0 + 2.0), 0.0);
}

TEST(MapAge, AgeHoveringAtTheThresholdDoesNotFlap) {
  // Stamp age alternating 1.4 / 1.6 s at 5 Hz (a slow mapper right at the limit): one hold, no
  // release while the age stays above max_age - margin, a single release once it drops below.
  MapAgeGuard g;
  int enters = 0, exits = 0;
  for (int i = 0; i < 100; ++i) {
    const double now = kT0 + 0.2 * i;
    const double age = (i % 2) ? 1.6 : 1.4;
    const Transition t = g.update(now, kMax, now - age);
    enters += (t == Transition::kEnterHold);
    exits += (t == Transition::kExitHold);
  }
  EXPECT_EQ(enters, 1);
  EXPECT_EQ(exits, 0);
  const double now = kT0 + 20.0;
  EXPECT_EQ(g.update(now, kMax, now - 0.45), Transition::kExitHold);
}

TEST(MapAge, EitherStampHolds) {
  MapAgeGuard occ_stale, esdf_stale;
  EXPECT_EQ(occ_stale.update(kT0, kMax, kT0 - 2.0, kT0 - 0.4), Transition::kEnterHold);
  EXPECT_EQ(esdf_stale.update(kT0, kMax, kT0 - 0.4, kT0 - 2.0), Transition::kEnterHold);
  EXPECT_NEAR(esdf_stale.age(), 2.0, 1e-9);
}

TEST(MapAge, MissingSecondStampIsIgnored) {
  MapAgeGuard g;
  EXPECT_EQ(g.update(kT0, kMax, kT0 - 0.4, 0.0), Transition::kNone);
  EXPECT_FALSE(g.held());
}

TEST(MapAge, ClockStepEitherWayHoldsThenHeals) {
  for (const double step : {+5.0, -5.0}) {
    MapAgeGuard g;
    const double stamp = kT0 - 0.4;
    // The clock jumps by `step` while the last grid still carries the old-clock stamp.
    EXPECT_EQ(g.update(kT0 + step, kMax, stamp), Transition::kEnterHold) << step;
    // Grids stamped on the new clock arrive: released after the minimum hold.
    const double now = kT0 + step + 1.2;
    EXPECT_EQ(g.update(now, kMax, now - 0.4), Transition::kExitHold) << step;
  }
}

TEST(MapAge, DisablingWhileHeldReleases) {
  MapAgeGuard g;
  ASSERT_EQ(g.update(kT0, kMax, kT0 - 3.0), Transition::kEnterHold);
  EXPECT_EQ(g.update(kT0 + 0.1, 0.0, kT0 - 3.0), Transition::kExitHold);
  EXPECT_FALSE(g.held());
}
