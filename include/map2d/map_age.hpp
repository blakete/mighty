/**
 * @file map_age.hpp
 * @brief ROS-free map-age check shared by the planner (MIGHTY_NODE map hold) and the goal selector
 *        (stale-map pause). Header-only, standard library only.
 *
 * Age = |now - header.stamp| of the newest occ_2d / esdf_2d grid, NOT the time since the last message:
 * the stamp is the DLIO cloud time, copied by the elevation mapper and map_fusion. When the lidar or
 * DLIO stops, the elevation mapper keeps republishing on its timer with that last (frozen) stamp, so
 * grids keep arriving on time while their content is old; only the stamp shows it. When the mapper or
 * map_fusion stops (silence), the last stamp simply ages.
 */
#pragma once

#include <algorithm>
#include <cmath>

namespace map2d {

/** @brief True when a map stamp is more than max_age away from now (either direction: a clock step
 *  also reads as stale and heals with the next message). max_age <= 0: check off. stamp <= 0: no map
 *  received yet (or no stamp) -> not stale. All values in seconds on one clock. */
inline bool stampTooOld(double now, double stamp, double max_age) {
  if (max_age <= 0.0 || stamp <= 0.0) return false;
  return std::abs(now - stamp) > max_age;
}

/** @brief Hold state machine over one or two map stamps (e.g. occ_2d and esdf_2d).
 *
 *  Enter the hold as soon as any stamp is too old (stampTooOld). Leave it only once every stamp is
 *  younger than max_age - exit margin (margin = min(0.5 s, max_age / 2)) AND the hold has lasted at
 *  least kMinHoldSec, so an age hovering at the threshold does not flap at the map rate. A stamp <= 0
 *  is ignored (that map never arrived). update() is idempotent for a given now. */
class MapAgeGuard {
 public:
  enum class Transition { kNone, kEnterHold, kExitHold };
  static constexpr double kMinHoldSec = 1.0;
  static constexpr double kMaxExitMarginSec = 0.5;

  Transition update(double now, double max_age, double stamp_a, double stamp_b = 0.0) {
    age_ = 0.0;
    bool any = false;
    for (const double s : {stamp_a, stamp_b}) {
      if (s <= 0.0) continue;
      any = true;
      age_ = std::max(age_, std::abs(now - s));
    }
    if (max_age <= 0.0 || !any) {
      if (!held_) return Transition::kNone;
      held_ = false;  // check switched off: never hold on a disabled guard
      last_hold_sec_ = now - since_;
      return Transition::kExitHold;
    }
    if (!held_) {
      if (age_ <= max_age) return Transition::kNone;
      held_ = true;
      since_ = now;
      return Transition::kEnterHold;
    }
    const double margin = std::min(kMaxExitMarginSec, 0.5 * max_age);
    if (age_ < max_age - margin && now - since_ >= kMinHoldSec) {
      held_ = false;
      last_hold_sec_ = now - since_;
      return Transition::kExitHold;
    }
    return Transition::kNone;
  }

  bool held() const { return held_; }
  /** @brief Worst stamp age seen by the last update() [s]. */
  double age() const { return age_; }
  /** @brief How long the current hold has lasted [s]; 0 when not held. */
  double heldFor(double now) const { return held_ ? now - since_ : 0.0; }
  /** @brief Duration of the hold that ended at the last kExitHold [s]. */
  double lastHoldSec() const { return last_hold_sec_; }

 private:
  bool held_{false};
  double since_{-1.0};
  double age_{0.0};
  double last_hold_sec_{0.0};
};

}  // namespace map2d
