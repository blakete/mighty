/**
 * @file goal_selector.hpp
 * @brief ROS-free core of the goal selector (frontier exploration, manual override, return-home,
 *        goal relocation, planner-status feedback).
 *
 * GoalSelector contains no rclcpp, no clock, no publishers and no threads. Everything it needs is
 * passed in as arguments and everything it wants done is returned as values:
 *
 *   - Time is an explicit `double now` (seconds) on every call. The selector never reads a clock.
 *   - Robot pose, grids, planner statuses, manual goals and peer data are passed to the on*()
 *     methods.
 *   - Each on*() method returns an Output: "publish this goal with this stamp", "publish markers",
 *     "publish / broadcast the visited map", "broadcast my pose", plus log lines. The caller (the
 *     GoalSelectorNode, or a test) performs the side effects.
 *
 * The class is not thread safe. The ROS node calls it from one mutually exclusive callback group.
 * (PeerTracker has its own mutex, but VisitedMap and FrontierManager do not.)
 *
 * Reused as-is: FrontierDetector, FrontierManager, VisitedMap, PeerTracker, OccGrid2D (these pull in
 * nav_msgs message headers through OccGrid2D, but no rclcpp), and the map2d library.
 *
 * ---------------------------------------------------------------------------------------------
 * Commitments and stamps (spec 4.3)
 *   A "commitment" is one frontier, one manual goal or one return-home goal. The selector stamps it
 *   when it commits: stamp_ns = round(now * 1e9), forced to be strictly greater than the previous
 *   commitment's stamp (and never 0), even if `now` does not advance. A re-relocation of the same
 *   commitment republishes with the same stamp.
 *
 * Planner status feedback (spec 3, decision D8)
 *   Statuses whose goal_stamp is 0 or differs from the current commitment's stamp are ignored.
 *   FAILED increments the unreachable counter; SKIPPED is neutral; SUCCESS/PARTIAL reset it;
 *   REACHED completes the commitment.
 *
 * Manual override (spec 4.2)
 *   onManualGoal() relocates + commits immediately. While a manual goal is active no frontier is
 *   selected, preempted or released by the watchdog / pursuit timeout. REACHED for its stamp
 *   releases it. FAILED statuses reaching the threshold only log a warning and raise
 *   manualGoalUnreachable(); the goal is never released or resent.
 *
 * Relocation (spec 4.4)
 *   On every onPlanningOccGrid() (and when a manual / return-home goal is committed) the selector
 *   rebuilds the planner's 2D window map from the latest planning_occ_2d grid with the planner's
 *   window geometry and inflation (map2d library). Manual and return-home goals are relocated on it;
 *   the current such goal is re-relocated (from its last published, relocated position, as the
 *   planner does from its stored G_term; also after REACHED) on every rebuild and republished with the same stamp if
 *   it moved. Relocation failure publishes the unrelocated goal (D10). ACTIVE / DORMANT frontiers
 *   whose centroid lies in an occupied or inflated cell of the map are invalidated (D8b).
 *   The window is centred on the robot and sized from the robot->goal distance, where "goal" is the
 *   current commitment's *stored* (unrelocated) target, projected on the horizon sphere exactly as
 *   the planner does; with no commitment the goal is taken to be the robot position.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "map2d/map2d.hpp"
#include "mighty/frontier_detector.hpp"
#include "mighty/frontier_manager.hpp"
#include "mighty/occ_grid_2d.hpp"
#include "mighty/peer_tracker.hpp"
#include "mighty/visited_map.hpp"

namespace goal_selector {

// ----------------------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------------------

/** @brief Everything the selector reads. Names and defaults follow the planner's parameters
 *  (`expl_*` = the planner's `exploration.*`; the rest are shared with the planner's YAML). */
struct SelectorParams {
  // --- identity / frames ---
  std::string robot_id;        ///< robot namespace tail (e.g. "RR04"); used on the peer topics
  std::string map_frame_id{"map"};

  // --- exploration (moved from the planner's expl_* fields; min_obstacle_distance_m dropped, D3) ---
  bool   expl_enabled{false};
  double expl_select_rate_hz{1.0};
  double expl_default_goal_z{0.0};
  // Detector
  int    expl_cluster_min_cells{6};
  int    expl_border_margin_cells{2};
  int    expl_obstacle_clearance_cells{1};
  double expl_robot_snap_radius_m{1.0};
  bool   expl_bounds_enabled{false};
  double expl_bounds_min_x{-50.0};
  double expl_bounds_max_x{50.0};
  double expl_bounds_min_y{-50.0};
  double expl_bounds_max_y{50.0};
  // Utility weights
  double expl_w_size{1.0};
  double expl_w_dist{2.0};
  double expl_w_info{1.0};
  double expl_w_revisit{0.5};
  double expl_w_heading{0.3};
  double expl_size_ref_m2{5.0};
  double expl_dist_ref_m{25.0};
  double expl_sensor_radius_m{5.0};
  double expl_goal_select_threshold{-1.0e9};
  // Manager / lifecycle
  double expl_merge_radius_m{1.0};
  double expl_centroid_ema_alpha{0.5};
  double expl_visit_radius_m{2.0};
  double expl_visit_dwell_sec{1.0};
  int    expl_verify_radius_cells{2};
  int    expl_max_frontiers{1000};
  int    expl_unreachable_consec_thresh{5};
  double expl_pursuit_timeout_factor{10.0};
  double expl_pursuit_timeout_v_ref{0.5};
  double expl_pursuit_timeout_min_sec{10.0};
  double expl_invalidation_keep_out_radius_m{1.5};
  double expl_invalidation_cooldown_sec{30.0};
  // Preemption
  bool   expl_preempt_enabled{false};
  double expl_preempt_margin{2.0};
  double expl_preempt_min_commit_sec{2.0};
  // Stuck watchdog
  double expl_stuck_timeout_sec{5.0};
  double expl_stuck_move_thresh_m{0.15};
  // Persistent visited map
  double expl_visited_map_center_x{0.0};
  double expl_visited_map_center_y{0.0};
  double expl_visited_map_width_m{100.0};
  double expl_visited_map_height_m{100.0};
  double expl_visited_map_resolution_m{0.15};
  bool   expl_publish_visited_map{true};
  bool   expl_fuse_persistent_into_local{true};
  bool   expl_detect_on_visited_map{true};
  // MinPos
  bool   expl_use_minpos{false};
  double expl_peer_timeout_sec{5.0};
  double expl_peer_publish_rate_hz{5.0};
  double expl_min_frontier_dist_to_peers_m{0.0};
  double expl_peer_visit_radius_m{2.0};
  // Visualization
  bool   expl_publish_markers{true};

  // --- shared with the planner's YAML ---
  double goal_radius{0.5};                   ///< return-home re-arm distance
  bool   relocate_occupied_goal{true};
  double goal_relocation_clearance_m{1.0};
  // Planner window geometry (spec 4.4). NOTE: `res` is the planner parameter `mighty_map_res`.
  double horizon{20.0};
  double map_buffer{6.0};
  double min_wdx{10.0};
  double min_wdy{10.0};
  double res{0.1};
  double factor_hgp{1.0};
  double inflation_hgp{0.5};
  double inflation_2d_m{0.0};
  // Goal z handling. Only used to reproduce the planner's window (the planner keeps its own z
  // handling: force_goal_z / z_min / z_max are applied by the planner, not here).
  bool   force_goal_z{true};
  double default_goal_z{2.5};
};

// ----------------------------------------------------------------------------
// Inputs
// ----------------------------------------------------------------------------

/** @brief Robot pose from `state` (D11). */
struct Pose {
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
};

/** @brief Plain copy of a nav_msgs/OccupancyGrid (values -1 unknown, 0 free, >=100 occupied). */
struct GridInput {
  int width{0};
  int height{0};
  double resolution{0.0};  ///< pass the message's float resolution converted to double
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  std::vector<int8_t> data;  ///< row-major, index = x + width * y
};

/** @brief Mirrors goal_selector_msgs/PlannerStatus constants. */
enum class PlannerStatus : uint8_t { kSuccess = 0, kFailed = 1, kPartial = 2, kReached = 3, kSkipped = 4 };

// ----------------------------------------------------------------------------
// Outputs
// ----------------------------------------------------------------------------

enum class GoalKind { kNone, kFrontier, kManual, kReturnHome };

/** @brief "Publish this goal on term_goal" (header.stamp = stamp_ns; frame = map_frame_id). */
struct GoalCommand {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  int64_t stamp_ns{0};
  GoalKind kind{GoalKind::kNone};
  bool relocated{false};  ///< position differs from the commitment's stored target
};

struct LogMessage {
  enum class Level { kInfo, kWarn, kError };
  Level level{Level::kInfo};
  std::string text;
};

struct Output {
  std::optional<GoalCommand> term_goal;               ///< publish on term_goal
  std::optional<Eigen::Vector3d> exploration_current_goal;  ///< publish on exploration/current_goal
  bool publish_markers{false};          ///< publish exploration/frontiers (see accessors below)
  bool publish_visited_map{false};      ///< publish exploration/visited_map (visitedMap(), occ2dOriginZ())
  bool broadcast_visited_map{false};    ///< broadcast visitedMap() on /exploration/visited_maps
  std::optional<Eigen::Vector3d> peer_pose;  ///< broadcast own position on /exploration/peer_poses
  std::vector<LogMessage> logs;

};

// ----------------------------------------------------------------------------
// GoalSelector
// ----------------------------------------------------------------------------

class GoalSelector {
 public:
  explicit GoalSelector(const SelectorParams& params);

  // ---- inputs -----------------------------------------------------------------------------

  /** @brief New robot pose (`state`). Returns the throttled peer-pose broadcast, if due.
   *  The first call marks the state as initialised (selection and detection wait for it). */
  Output onState(double now, const Pose& pose);

  /** @brief Raw occ_2d_topic grid: visited-map fusion/absorb, frontier detection, manager update,
   *  throttled visited-map publish / broadcast, markers, 
   *  (as occ2DCallback did) EXCEPT the selection pass: call onSelectTick() right after, so markers
   *  are published before selection as in the planner. The grid is taken by value (fusion edits it). */
  Output onOccGrid(double now, GridInput grid);

  /** @brief planning_occ_2d_topic grid: rebuild the relocation map (spec 4.4), invalidate frontiers
   *  in the occupied / inflated band (D8b), re-relocate the current manual / return-home goal and
   *  republish it with the same stamp if it moved. */
  Output onPlanningOccGrid(double now, const GridInput& grid);

  /** @brief Exploration select tick (exploreSelectCallback), called at expl_select_rate_hz. */
  Output onSelectTick(double now);

  /** @brief One planner_status message. goal_stamp_ns is the message's goal_stamp in nanoseconds. */
  Output onPlannerStatus(double now, PlannerStatus status, int64_t goal_stamp_ns);

  /** @brief Manual goal from term_goal_rviz. Relocated and committed immediately; overrides
   *  exploration until REACHED for its stamp. */
  Output onManualGoal(double now, double x, double y, double z);

  /** @brief /exploration/return_home trigger. */
  Output onReturnHome(double now);

  /** @brief Peer pose (`/exploration/peer_poses`). `peer_id` is the message's frame_id, `t` the
   *  message stamp in seconds. Own messages (peer_id == robot_id) are ignored. */
  void onPeerPose(const std::string& peer_id, double x, double y, double t);

  /** @brief Peer visited map (`/exploration/visited_maps`). `sender_id` is the message's frame_id.
   *  Own messages are ignored. Merged into unknown cells of the local visited map only. */
  void onPeerVisitedMap(const std::string& sender_id, const GridInput& grid);

  // ---- state accessors (for visualisation and tests) --------------------------------------

  bool stateInitialized() const { return state_initialized_; }
  const SelectorParams& params() const { return params_; }

  bool explorationActive() const { return exploration_active_; }
  bool manualGoalActive() const { return manual_goal_active_; }
  bool homeReturnRequested() const { return home_return_requested_; }
  uint64_t currentExploreId() const { return current_explore_id_; }
  int unreachableCount() const { return unreachable_consec_count_; }
  /** @brief True once FAILED statuses for the active manual goal reached the threshold (warning only). */
  bool manualGoalUnreachable() const { return manual_unreachable_; }

  GoalKind currentKind() const { return current_.kind; }
  int64_t currentStampNs() const { return current_.stamp_ns; }
  /** @brief The stored (unrelocated) target of the current commitment. */
  const Eigen::Vector3d& currentTarget() const { return current_.target; }
  /** @brief The position last published for the current commitment. */
  const Eigen::Vector3d& currentPublished() const { return current_.published; }

  const FrontierManager& frontierManager() const { return *frontier_manager_; }
  FrontierManager& frontierManager() { return *frontier_manager_; }
  const VisitedMap& visitedMap() const { return *visited_map_; }
  const Eigen::Vector3d& explorationStartPos() const { return exploration_start_pos_; }
  Pose pose() const { return pose_; }
  /** @brief origin.z of the latest raw occ_2d grid (visited-map plane), or expl_default_goal_z. */
  double occ2dOriginZ() const { return occ2d_origin_z_.value_or(params_.expl_default_goal_z); }
  /** @brief Relocation map built from the latest planning grid (empty before the first rebuild). */
  const map2d::Grid2D& relocationMap() const { return reloc_map_; }
  /** @brief True if a raw occ_2d grid has been received. */
  bool hasOccGrid() const { return occ_grid_2d_ != nullptr; }

 private:
  struct Commitment {
    GoalKind kind{GoalKind::kNone};
    int64_t stamp_ns{0};
    Eigen::Vector3d target{Eigen::Vector3d::Zero()};     // stored, unrelocated
    Eigen::Vector3d published{Eigen::Vector3d::Zero()};  // last published (possibly relocated)
    };

  struct PlanningSource {
    bool valid{false};
    int width{0}, height{0};
    double resolution{0.0}, inv_resolution{0.0}, origin_x{0.0}, origin_y{0.0};
    std::vector<bool> occupied, unknown;
  };

  struct RelocResult {
    bool ok{true};     // false = relocation failed (goal would be dropped)
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  };

  int64_t nextStampNs(double now);
  void log(Output& out, LogMessage::Level level, std::string text) const;

  // commit a goal: new stamp, relocate (non-frontier kinds), fill out.term_goal
  void commit(GoalKind kind, const Eigen::Vector3d& target, double now, Output& out);

  // relocation (spec 4.4)
  void rebuildRelocationMap();
  RelocResult relocate(const Eigen::Vector3d& goal) const;
  void invalidateFrontiersInBand(double now, Output& out);
  void rerelocateCurrent(Output& out);

  // moved exploration logic
  void selectAndCommit(double now, Output& out);          // exploreSelectCallback
  void maybePublishVisitedMap(double now, Output& out);

  SelectorParams params_;

  std::unique_ptr<FrontierDetector> frontier_detector_;
  std::unique_ptr<FrontierManager> frontier_manager_;
  std::unique_ptr<VisitedMap> visited_map_;
  PeerTracker peer_tracker_;

  std::shared_ptr<const OccGrid2D> occ_grid_2d_;
  std::shared_ptr<const OccGrid2D> current_detect_grid_;
  std::optional<double> occ2d_origin_z_;

  Pose pose_;
  bool state_initialized_{false};

  Commitment current_;
  int64_t last_stamp_ns_{0};

  bool exploration_active_{false};
  bool manual_goal_active_{false};
  bool home_return_requested_{false};
  bool manual_unreachable_{false};
  uint64_t current_explore_id_{0};
  int unreachable_consec_count_{0};
  double explore_committed_at_t_{-1.0};
  Eigen::Vector2d explore_last_progress_xy_{Eigen::Vector2d::Zero()};
  double explore_last_progress_t_{-1.0};
  bool explore_has_moved_{false};
  Eigen::Vector3d exploration_start_pos_{0.0, 0.0, 0.0};
  bool exploration_start_captured_{false};

  double last_peer_pose_publish_t_{0.0};
  double last_visited_publish_t_{0.0};
  double last_peer_visited_publish_t_{0.0};
  double last_diag_log_t_{-1.0e18};
  double last_defer_log_t_{-1.0e18};

  PlanningSource planning_src_;
  map2d::Grid2D reloc_map_;
};

}  // namespace goal_selector
