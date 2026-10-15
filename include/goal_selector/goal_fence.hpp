#pragma once
/**
 * @file goal_fence.hpp
 * @brief Geofence geometry for exploration goals (`/<R>/geofence/goal_fence`, std_msgs/String JSON).
 *
 * The geofence (geofence_ws, Fence.goal_fence) publishes its keep_in / keep_out rings already moved into
 * the robot's odom frame, plus offset_m = footprint radius + margin (+ goal standoff). A goal is valid iff
 * it is inside keep_in, outside every keep_out and at least offset_m from every ring edge -- exactly the
 * gate's `core` (the area where the rover drives normally), with no rasterization and over the whole fence.
 *
 *   {"v": 1, "frame_id": "JR02/odom", "active": true, "ready": true, "offset_m": 0.36,
 *    "keep_in": [[x, y], ...], "keep_out": [[[x, y], ...], ...], ...}
 *
 * active: the fence is enforced. ready: the rings are usable (anchored, localization OK); while active and
 * not ready the selector pauses new goals. ROS-free (nlohmann_json only).
 */

#include <cstdint>
#include <string>
#include <vector>

namespace goal_selector {

struct FencePoint {
  double x{0.0}, y{0.0};
};

struct GoalFence {
  bool active{false};
  bool ready{false};
  double offset_m{0.0};
  std::vector<FencePoint> keep_in;                ///< open ring (last vertex != first)
  std::vector<std::vector<FencePoint>> keep_out;  ///< open rings
};

/** @brief True if (x, y) (in the fence's frame) is a valid goal: inside keep_in, outside every keep_out
 *  (even-odd per ring) and at least offset_m + extra_m from every ring edge. Points on an edge are never
 *  valid. False if keep_in has fewer than 3 vertices. */
bool goalFenceAllows(const GoalFence& f, double x, double y, double extra_m = 0.0);

/** @brief Parse a goal_fence JSON message.
 *  @return false if it is not one (not JSON, or no boolean "active"): ignore it.
 *  true otherwise; if "ready" but offset/rings/frame are missing, malformed or non-finite, `out.ready` is
 *  false and `error` says why (an active fence that cannot be used pauses goals instead of being trusted). */
bool parseGoalFence(const std::string& json, GoalFence& out, std::string& frame_id, std::string& error);

/** @brief Parse the drive gate's echo (`/<R>/safety_node/geofence_gate`, std_msgs/String JSON, 2 Hz from
 *  the drive, independent of the geofence process): {"gate": 1, "engaged": true, "holding": ..., ...}.
 *  engaged = the geofence is required for AUTO (ENGAGED intent present; false when off or in shadow).
 *  @return false if it is not one (not JSON, or no boolean "engaged"): ignore it. */
bool parseGateEcho(const std::string& json, bool& engaged);

/** @brief Cell classes of rasterizeGoalFence(). */
enum FenceCell : uint8_t {
  kFenceAllowed = 0,    ///< inside the core: at least band_m from every fence edge
  kFenceBand = 1,       ///< allowed area, but closer than band_m to a fence edge
  kFenceForbidden = 2,  ///< outside keep_in or inside a keep_out
};

/** @brief Classify every cell CENTRE of a dim_x x dim_y window (cell (x, y) centre =
 *  origin + (x + 0.5, y + 0.5) * res, index x + dim_x * y) against the fence, with the arithmetic of
 *  goalFenceAllows(): kFenceForbidden where the centre is not inside keep_in or is inside a keep_out
 *  (even-odd per ring), else kFenceBand where it is closer than band_m to a ring edge, else kFenceAllowed.
 *  So goalFenceAllows(f, cx, cy, extra) == (cell == kFenceAllowed) when band_m == f.offset_m + extra.
 *  A fence that is not ready (or has no keep_in) gives all kFenceAllowed. band_m <= 0: no band.
 *  Cost: one pass over the window plus the cells near each edge. */
void rasterizeGoalFence(const GoalFence& f, int dim_x, int dim_y, double res, double origin_x,
                        double origin_y, double band_m, std::vector<uint8_t>& out);

}  // namespace goal_selector
