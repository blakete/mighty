#include "goal_selector/goal_selector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace goal_selector {

namespace {

std_msgs::msg::ColorRGBA makeColor(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

std_msgs::msg::ColorRGBA colorForState(FrontierState s) {
  switch (s) {
    case FrontierState::ACTIVE:      return makeColor(0.0, 0.8, 1.0, 0.9);  // cyan
    case FrontierState::DORMANT:     return makeColor(0.6, 0.6, 0.6, 0.7);  // gray
    case FrontierState::VISITED:     return makeColor(0.0, 0.9, 0.0, 0.7);  // green
    case FrontierState::INVALIDATED: return makeColor(0.9, 0.0, 0.0, 0.7);  // red
  }
  return makeColor(1.0, 1.0, 1.0, 0.7);
}

builtin_interfaces::msg::Time stampFromNs(int64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

}  // namespace

// ----------------------------------------------------------------------------

GoalSelectorNode::GoalSelectorNode() : Node("goal_selector") {
  // Robot id from the namespace (e.g. /RR04 -> RR04), as mighty_node's ns_.
  std::string ns = this->get_namespace();
  ns = ns.substr(ns.find_last_of("/") + 1);
  par_.robot_id = ns;

  declareParameters();
  readParameters();
  printParameters();

  selector_ = std::make_unique<GoalSelector>(par_);

  // Gate (spec 4): the manual-goal path (state, term_goal_rviz, planner_status, term_goal)
  // needs only a ground robot with 2D planning; frontier,
  // visited-map, peer and return-home subscriptions and the select timer also need
  // exploration.enabled.
  if (!(vehicle_type_ == "ground_robot" && use_2d_planning_)) {
    RCLCPP_WARN(this->get_logger(),
                "goal_selector idle: needs vehicle_type=ground_robot (is '%s') and "
                "use_2d_planning=true (is %d)",
                vehicle_type_.c_str(), use_2d_planning_);
    return;
  }

  // QoS: critical_qos as in mighty_node (reliable, KeepLast(10), volatile).
  rclcpp::QoS critical_qos(rclcpp::KeepLast(10));
  critical_qos.reliable().durability_volatile();
  // Mapper grids are published BEST_EFFORT: SensorDataQoS = BEST_EFFORT + VOLATILE + KEEP_LAST(5).
  auto map_qos = rclcpp::SensorDataQoS();

  // One mutually exclusive group for everything -> the core is never entered concurrently.
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group_;

  // Publishers
  pub_term_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("term_goal", critical_qos);
  // Visualisation, QoS as the planner's planning_map_2d / point_G_term.
  pub_selector_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "selector_map_2d", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  rclcpp::QoS viz_qos(rclcpp::KeepLast(1));
  viz_qos.best_effort().durability_volatile();
  pub_selector_goal_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("point_selector_goal", viz_qos);
  if (par_.expl_enabled) {
    pub_frontiers_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("exploration/frontiers", 10);
    pub_explore_current_goal_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("exploration/current_goal", 10);
    pub_visited_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "exploration/visited_map", rclcpp::QoS(1).transient_local());
  }

  // Subscriptions
  sub_state_ = this->create_subscription<dynus_interfaces::msg::State>(
      "state", critical_qos,
      std::bind(&GoalSelectorNode::stateCallback, this, std::placeholders::_1), options);
  sub_term_goal_rviz_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "term_goal_rviz", critical_qos,
      std::bind(&GoalSelectorNode::manualGoalCallback, this, std::placeholders::_1), options);
  sub_planner_status_ = this->create_subscription<goal_selector_msgs::msg::PlannerStatus>(
      "planner_status", critical_qos,
      std::bind(&GoalSelectorNode::plannerStatusCallback, this, std::placeholders::_1), options);

  // Always subscribed: with exploration off the grid is only stored for selector_map_2d.
  sub_occ_2d_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "occ_2d_topic", map_qos,
      std::bind(&GoalSelectorNode::occ2DCallback, this, std::placeholders::_1), options);

  if (!par_.expl_enabled) {
    RCLCPP_INFO(this->get_logger(), "Exploration disabled: manual-goal path only");
    return;
  }

  // MinPos peer pose / visited-map sharing (global topics, all agents pub+sub)
  if (par_.expl_use_minpos) {
    auto peer_qos = rclcpp::QoS(10).reliable();
    pub_peer_pose_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/exploration/peer_poses", peer_qos);
    sub_peer_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/exploration/peer_poses", peer_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          selector_->onPeerPose(msg->header.frame_id, msg->pose.position.x, msg->pose.position.y,
                                rclcpp::Time(msg->header.stamp).seconds());
        },
        options);

    pub_peer_visited_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/exploration/visited_maps", rclcpp::QoS(1).reliable());
    sub_peer_visited_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/exploration/visited_maps", rclcpp::QoS(1).reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          selector_->onPeerVisitedMap(msg->header.frame_id, toGridInput(*msg));
        },
        options);
    RCLCPP_INFO(this->get_logger(),
                "Exploration MinPos: enabled, peer_timeout=%.1fs, publish_rate=%.1fHz, "
                "visited-map sharing enabled",
                par_.expl_peer_timeout_sec, par_.expl_peer_publish_rate_hz);
  }

  // Global return-home trigger (reliable so one shot reaches every agent).
  sub_return_home_ = this->create_subscription<std_msgs::msg::Empty>(
      "/exploration/return_home", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Empty::SharedPtr) {
        apply(selector_->onReturnHome(this->now().seconds()));
      },
      options);

  const double rate_hz = std::max(0.1, par_.expl_select_rate_hz);
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_select_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GoalSelectorNode::selectTimerCallback, this), cb_group_);

  RCLCPP_INFO(this->get_logger(),
              "Exploration: enabled, select rate=%.1f Hz, min_cells=%d, merge_radius=%.2fm, "
              "visit_radius=%.2fm",
              rate_hz, par_.expl_cluster_min_cells, par_.expl_merge_radius_m,
              par_.expl_visit_radius_m);
}

// ----------------------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------------------

void GoalSelectorNode::declareParameters() {
  // Shared with the planner's YAML (same names, same defaults).
  this->declare_parameter("vehicle_type", "uav");
  this->declare_parameter("use_2d_planning", false);
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("goal_radius", 0.5);
  this->declare_parameter("inflation_2d_m", 0.0);
  this->declare_parameter("unknown_inflation_2d_m", 0.5);

  // exploration.* (min_obstacle_distance_m is not carried over, D3)
  this->declare_parameter("exploration.enabled", false);
  this->declare_parameter("exploration.select_rate_hz", 1.0);
  this->declare_parameter("exploration.default_goal_z", 0.0);
  this->declare_parameter("exploration.detector.cluster_min_cells", 6);
  this->declare_parameter("exploration.detector.border_margin_cells", 2);
  this->declare_parameter("exploration.detector.obstacle_clearance_cells", 1);
  this->declare_parameter("exploration.detector.robot_snap_radius_m", 1.0);
  this->declare_parameter("exploration.bounds.enabled", false);
  this->declare_parameter("exploration.bounds.min_x", -50.0);
  this->declare_parameter("exploration.bounds.max_x", 50.0);
  this->declare_parameter("exploration.bounds.min_y", -50.0);
  this->declare_parameter("exploration.bounds.max_y", 50.0);
  this->declare_parameter("exploration.utility.w_size", 1.0);
  this->declare_parameter("exploration.utility.w_dist", 2.0);
  this->declare_parameter("exploration.utility.w_info", 1.0);
  this->declare_parameter("exploration.utility.w_revisit", 0.5);
  this->declare_parameter("exploration.utility.w_heading", 0.3);
  this->declare_parameter("exploration.utility.size_ref_m2", 5.0);
  this->declare_parameter("exploration.utility.dist_ref_m", 25.0);
  this->declare_parameter("exploration.utility.sensor_radius_m", 5.0);
  this->declare_parameter("exploration.utility.goal_select_threshold", -1.0e9);
  this->declare_parameter("exploration.manager.merge_radius_m", 1.0);
  this->declare_parameter("exploration.manager.centroid_ema_alpha", 0.5);
  this->declare_parameter("exploration.manager.visit_radius_m", 2.0);
  this->declare_parameter("exploration.manager.visit_dwell_sec", 1.0);
  this->declare_parameter("exploration.manager.verify_radius_cells", 2);
  this->declare_parameter("exploration.manager.max_frontiers", 1000);
  this->declare_parameter("exploration.manager.unreachable_consec_thresh", 5);
  this->declare_parameter("exploration.manager.pursuit_timeout_factor", 10.0);
  this->declare_parameter("exploration.manager.pursuit_timeout_v_ref", 0.5);
  this->declare_parameter("exploration.manager.pursuit_timeout_min_sec", 10.0);
  this->declare_parameter("exploration.manager.invalidation_keep_out_radius_m", 1.5);
  this->declare_parameter("exploration.manager.invalidation_cooldown_sec", 30.0);
  this->declare_parameter("exploration.manager.preempt_enabled", false);
  this->declare_parameter("exploration.manager.preempt_margin", 2.0);
  this->declare_parameter("exploration.manager.preempt_min_commit_sec", 2.0);
  this->declare_parameter("exploration.manager.stuck_timeout_sec", 5.0);
  this->declare_parameter("exploration.manager.stuck_move_thresh_m", 0.15);
  this->declare_parameter("exploration.visited_map.center_x", 0.0);
  this->declare_parameter("exploration.visited_map.center_y", 0.0);
  this->declare_parameter("exploration.visited_map.width_m", 100.0);
  this->declare_parameter("exploration.visited_map.height_m", 100.0);
  this->declare_parameter("exploration.visited_map.resolution_m", 0.15);
  this->declare_parameter("exploration.visited_map.publish", true);
  this->declare_parameter("exploration.visited_map.fuse_into_local", true);
  this->declare_parameter("exploration.visited_map.detect_on_visited_map", true);
  this->declare_parameter("exploration.visualization.publish_markers", true);
  this->declare_parameter("exploration.minpos.enabled", false);
  this->declare_parameter("exploration.minpos.peer_timeout_sec", 5.0);
  this->declare_parameter("exploration.minpos.peer_publish_rate_hz", 5.0);
  this->declare_parameter("exploration.minpos.min_frontier_dist_to_peers_m", 0.0);
  this->declare_parameter("exploration.minpos.peer_visit_radius_m", 2.0);
}

void GoalSelectorNode::readParameters() {
  auto d = [this](const char* n) { return this->get_parameter(n).as_double(); };
  auto i = [this](const char* n) { return static_cast<int>(this->get_parameter(n).as_int()); };
  auto b = [this](const char* n) { return this->get_parameter(n).as_bool(); };

  vehicle_type_ = this->get_parameter("vehicle_type").as_string();
  use_2d_planning_ = b("use_2d_planning");
  par_.map_frame_id = this->get_parameter("map_frame_id").as_string();
  par_.goal_radius = d("goal_radius");
  par_.inflation_2d_m = d("inflation_2d_m");
  par_.unknown_inflation_2d_m = d("unknown_inflation_2d_m");

  par_.expl_enabled = b("exploration.enabled");
  par_.expl_select_rate_hz = d("exploration.select_rate_hz");
  par_.expl_default_goal_z = d("exploration.default_goal_z");
  par_.expl_cluster_min_cells = i("exploration.detector.cluster_min_cells");
  par_.expl_border_margin_cells = i("exploration.detector.border_margin_cells");
  par_.expl_obstacle_clearance_cells = i("exploration.detector.obstacle_clearance_cells");
  par_.expl_robot_snap_radius_m = d("exploration.detector.robot_snap_radius_m");
  par_.expl_bounds_enabled = b("exploration.bounds.enabled");
  par_.expl_bounds_min_x = d("exploration.bounds.min_x");
  par_.expl_bounds_max_x = d("exploration.bounds.max_x");
  par_.expl_bounds_min_y = d("exploration.bounds.min_y");
  par_.expl_bounds_max_y = d("exploration.bounds.max_y");
  par_.expl_w_size = d("exploration.utility.w_size");
  par_.expl_w_dist = d("exploration.utility.w_dist");
  par_.expl_w_info = d("exploration.utility.w_info");
  par_.expl_w_revisit = d("exploration.utility.w_revisit");
  par_.expl_w_heading = d("exploration.utility.w_heading");
  par_.expl_size_ref_m2 = d("exploration.utility.size_ref_m2");
  par_.expl_dist_ref_m = d("exploration.utility.dist_ref_m");
  par_.expl_sensor_radius_m = d("exploration.utility.sensor_radius_m");
  par_.expl_goal_select_threshold = d("exploration.utility.goal_select_threshold");
  par_.expl_merge_radius_m = d("exploration.manager.merge_radius_m");
  par_.expl_centroid_ema_alpha = d("exploration.manager.centroid_ema_alpha");
  par_.expl_visit_radius_m = d("exploration.manager.visit_radius_m");
  par_.expl_visit_dwell_sec = d("exploration.manager.visit_dwell_sec");
  par_.expl_verify_radius_cells = i("exploration.manager.verify_radius_cells");
  par_.expl_max_frontiers = i("exploration.manager.max_frontiers");
  par_.expl_unreachable_consec_thresh = i("exploration.manager.unreachable_consec_thresh");
  par_.expl_pursuit_timeout_factor = d("exploration.manager.pursuit_timeout_factor");
  par_.expl_pursuit_timeout_v_ref = d("exploration.manager.pursuit_timeout_v_ref");
  par_.expl_pursuit_timeout_min_sec = d("exploration.manager.pursuit_timeout_min_sec");
  par_.expl_invalidation_keep_out_radius_m = d("exploration.manager.invalidation_keep_out_radius_m");
  par_.expl_invalidation_cooldown_sec = d("exploration.manager.invalidation_cooldown_sec");
  par_.expl_preempt_enabled = b("exploration.manager.preempt_enabled");
  par_.expl_preempt_margin = d("exploration.manager.preempt_margin");
  par_.expl_preempt_min_commit_sec = d("exploration.manager.preempt_min_commit_sec");
  par_.expl_stuck_timeout_sec = d("exploration.manager.stuck_timeout_sec");
  par_.expl_stuck_move_thresh_m = d("exploration.manager.stuck_move_thresh_m");
  par_.expl_visited_map_center_x = d("exploration.visited_map.center_x");
  par_.expl_visited_map_center_y = d("exploration.visited_map.center_y");
  par_.expl_visited_map_width_m = d("exploration.visited_map.width_m");
  par_.expl_visited_map_height_m = d("exploration.visited_map.height_m");
  par_.expl_visited_map_resolution_m = d("exploration.visited_map.resolution_m");
  par_.expl_publish_visited_map = b("exploration.visited_map.publish");
  par_.expl_fuse_persistent_into_local = b("exploration.visited_map.fuse_into_local");
  par_.expl_detect_on_visited_map = b("exploration.visited_map.detect_on_visited_map");
  par_.expl_publish_markers = b("exploration.visualization.publish_markers");
  par_.expl_use_minpos = b("exploration.minpos.enabled");
  par_.expl_peer_timeout_sec = d("exploration.minpos.peer_timeout_sec");
  par_.expl_peer_publish_rate_hz = d("exploration.minpos.peer_publish_rate_hz");
  par_.expl_min_frontier_dist_to_peers_m = d("exploration.minpos.min_frontier_dist_to_peers_m");
  par_.expl_peer_visit_radius_m = d("exploration.minpos.peer_visit_radius_m");
}

void GoalSelectorNode::printParameters() const {
  auto lg = this->get_logger();
  RCLCPP_INFO(lg, "robot_id=%s vehicle_type=%s use_2d_planning=%d map_frame_id=%s",
              par_.robot_id.c_str(), vehicle_type_.c_str(), use_2d_planning_,
              par_.map_frame_id.c_str());
  RCLCPP_INFO(lg, "Exploration enabled: %d  select_rate_hz: %.2f  default_goal_z: %.2f",
              par_.expl_enabled, par_.expl_select_rate_hz, par_.expl_default_goal_z);
  RCLCPP_INFO(lg, "goal_radius: %.2f m  inflation_2d_m: %.2f  unknown_inflation_2d_m: %.2f",
              par_.goal_radius, par_.inflation_2d_m, par_.unknown_inflation_2d_m);
  RCLCPP_INFO(lg, "Unreachable thresh: %d  stuck_timeout: %.1f s  preempt: %d  minpos: %d",
              par_.expl_unreachable_consec_thresh, par_.expl_stuck_timeout_sec,
              par_.expl_preempt_enabled, par_.expl_use_minpos);
}

// ----------------------------------------------------------------------------
// Callbacks (message -> core input)
// ----------------------------------------------------------------------------

GridInput GoalSelectorNode::toGridInput(const nav_msgs::msg::OccupancyGrid& msg) {
  GridInput g;
  g.width = static_cast<int>(msg.info.width);
  g.height = static_cast<int>(msg.info.height);
  g.resolution = static_cast<double>(msg.info.resolution);
  g.origin_x = msg.info.origin.position.x;
  g.origin_y = msg.info.origin.position.y;
  g.origin_z = msg.info.origin.position.z;
  g.data = msg.data;
  return g;
}

void GoalSelectorNode::stateCallback(const dynus_interfaces::msg::State::SharedPtr msg) {
  Pose p;
  p.x = msg->pos.x;
  p.y = msg->pos.y;
  p.z = msg->pos.z;
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->quat.x, msg->quat.y, msg->quat.z, msg->quat.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  p.yaw = yaw;
  apply(selector_->onState(this->now().seconds(), p));
}

void GoalSelectorNode::occ2DCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_occ_header_ = msg->header;
  apply(selector_->onOccGrid(this->now().seconds(), toGridInput(*msg)));  // markers first
  maybePublishSelectorMap();
  // Then drive selection immediately (as occ2DCallback called exploreSelectCallback()).
  // (A no-op with exploration off.)
  apply(selector_->onSelectTick(this->now().seconds()));
}

void GoalSelectorNode::manualGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  apply(selector_->onManualGoal(this->now().seconds(), msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z));
}

void GoalSelectorNode::plannerStatusCallback(
    const goal_selector_msgs::msg::PlannerStatus::SharedPtr msg) {
  const int64_t stamp_ns = static_cast<int64_t>(msg->goal_stamp.sec) * 1000000000LL +
                           static_cast<int64_t>(msg->goal_stamp.nanosec);
  PlannerStatus s;
  switch (msg->status) {
    case goal_selector_msgs::msg::PlannerStatus::SUCCESS: s = PlannerStatus::kSuccess; break;
    case goal_selector_msgs::msg::PlannerStatus::FAILED:  s = PlannerStatus::kFailed; break;
    case goal_selector_msgs::msg::PlannerStatus::PARTIAL: s = PlannerStatus::kPartial; break;
    case goal_selector_msgs::msg::PlannerStatus::REACHED: s = PlannerStatus::kReached; break;
    case goal_selector_msgs::msg::PlannerStatus::SKIPPED: s = PlannerStatus::kSkipped; break;
    default:
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "planner_status: unknown status %u ignored", msg->status);
      return;
  }
  apply(selector_->onPlannerStatus(this->now().seconds(), s, stamp_ns));
}

void GoalSelectorNode::selectTimerCallback() {
  apply(selector_->onSelectTick(this->now().seconds()));
}

// ----------------------------------------------------------------------------
// Side effects
// ----------------------------------------------------------------------------

void GoalSelectorNode::apply(Output&& out) {
  for (const auto& l : out.logs) {
    switch (l.level) {
      case LogMessage::Level::kInfo:  RCLCPP_INFO(this->get_logger(), "%s", l.text.c_str()); break;
      case LogMessage::Level::kWarn:  RCLCPP_WARN(this->get_logger(), "%s", l.text.c_str()); break;
      case LogMessage::Level::kError: RCLCPP_ERROR(this->get_logger(), "%s", l.text.c_str()); break;
    }
  }
  if (out.publish_markers) publishFrontierMarkers();
  if (out.term_goal) publishGoal(*out.term_goal);
  if (out.exploration_current_goal) publishExplorationCurrentGoal(*out.exploration_current_goal);
  if (out.publish_visited_map) publishVisitedMap();
  if (out.broadcast_visited_map) broadcastVisitedMap();
  if (out.peer_pose && pub_peer_pose_) {
    geometry_msgs::msg::PoseStamped m;
    m.header.stamp = this->now();
    m.header.frame_id = par_.robot_id;
    m.pose.position.x = out.peer_pose->x();
    m.pose.position.y = out.peer_pose->y();
    m.pose.position.z = out.peer_pose->z();
    pub_peer_pose_->publish(m);
  }
}

void GoalSelectorNode::publishGoal(const GoalCommand& cmd) {
  geometry_msgs::msg::PoseStamped g;
  g.header.frame_id = par_.map_frame_id;
  g.header.stamp = stampFromNs(cmd.stamp_ns);  // identifies the commitment (spec 4.3)
  g.pose.position.x = cmd.position.x();
  g.pose.position.y = cmd.position.y();
  g.pose.position.z = cmd.position.z();
  g.pose.orientation.w = 1.0;
  pub_term_goal_->publish(g);

  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = g.header.frame_id;
  p.header.stamp = this->now();
  p.point = g.pose.position;
  pub_selector_goal_->publish(p);
}

void GoalSelectorNode::maybePublishSelectorMap() {
  // At most 1 Hz; the grid is only derived and converted when it is going to be published.
  const double t = this->now().seconds();
  if (t - last_selector_map_pub_t_ < 1.0) return;
  SelectorMap sm;
  if (!selector_->selectorMap(sm)) return;
  last_selector_map_pub_t_ = t;

  nav_msgs::msg::OccupancyGrid msg;
  // Exploring: the visited map's geometry in the map frame; otherwise the raw grid's header.
  if (par_.expl_enabled) {
    msg.header.stamp = this->now();
  } else {
    msg.header = last_occ_header_;
  }
  if (par_.expl_enabled || msg.header.frame_id.empty()) msg.header.frame_id = par_.map_frame_id;
  msg.info.map_load_time = msg.header.stamp;
  msg.info.resolution = static_cast<float>(sm.resolution);
  msg.info.width = static_cast<uint32_t>(sm.width);
  msg.info.height = static_cast<uint32_t>(sm.height);
  msg.info.origin.position.x = sm.origin_x;
  msg.info.origin.position.y = sm.origin_y;
  msg.info.origin.position.z = sm.origin_z;
  msg.info.origin.orientation.w = 1.0;
  msg.data = std::move(sm.data);
  pub_selector_map_->publish(msg);
}

void GoalSelectorNode::publishExplorationCurrentGoal(const Eigen::Vector3d& p) {
  if (!pub_explore_current_goal_) return;
  geometry_msgs::msg::PoseStamped g;
  g.header.frame_id = par_.map_frame_id;
  g.header.stamp = this->now();
  g.pose.position.x = p.x();
  g.pose.position.y = p.y();
  g.pose.position.z = p.z();
  g.pose.orientation.w = 1.0;
  pub_explore_current_goal_->publish(g);
}

void GoalSelectorNode::publishVisitedMap() {
  const VisitedMap& vm = selector_->visitedMap();
  if (!pub_visited_map_ || vm.empty()) return;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = par_.map_frame_id;
  msg.header.stamp = this->now();
  msg.info.resolution = static_cast<float>(vm.resolution());
  msg.info.width = static_cast<unsigned>(vm.width());
  msg.info.height = static_cast<unsigned>(vm.height());
  msg.info.origin.position.x = vm.originX();
  msg.info.origin.position.y = vm.originY();
  // Match the z of the live occ_2d layer (global_mapper uses z_ground).
  msg.info.origin.position.z = selector_->occ2dOriginZ();
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(vm.data().begin(), vm.data().end());  // raw tristate copy
  pub_visited_map_->publish(msg);
}

void GoalSelectorNode::broadcastVisitedMap() {
  const VisitedMap& vm = selector_->visitedMap();
  if (!pub_peer_visited_map_ || vm.empty()) return;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = par_.robot_id;
  msg.header.stamp = this->now();
  msg.info.resolution = static_cast<float>(vm.resolution());
  msg.info.width = static_cast<unsigned>(vm.width());
  msg.info.height = static_cast<unsigned>(vm.height());
  msg.info.origin.position.x = vm.originX();
  msg.info.origin.position.y = vm.originY();
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(vm.data().begin(), vm.data().end());
  pub_peer_visited_map_->publish(msg);
}

/**
 * @brief Publish frontier visualization markers (ACTIVE and DORMANT centroids + id labels, the
 *        robot->goal line, and the exploration bounds). One MarkerArray per cycle, prefixed with
 *        DELETEALL. Moved from MIGHTY_NODE::publishFrontierMarkers.
 */
void GoalSelectorNode::publishFrontierMarkers() {
  if (!pub_frontiers_ || !selector_->hasOccGrid()) return;
  const auto& fm = selector_->frontierManager();
  const double z0 = par_.expl_default_goal_z;
  const auto stamp = this->now();

  visualization_msgs::msg::MarkerArray arr;

  {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = par_.map_frame_id;
    del.header.stamp = stamp;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);
  }

  const auto& records = fm.records();

  visualization_msgs::msg::Marker centroids;
  centroids.header.frame_id = par_.map_frame_id;
  centroids.header.stamp = stamp;
  centroids.ns = "frontier_centroids";
  centroids.id = 0;
  centroids.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  centroids.action = visualization_msgs::msg::Marker::ADD;
  centroids.scale.x = 0.3;
  centroids.scale.y = 0.3;
  centroids.scale.z = 0.3;
  centroids.pose.orientation.w = 1.0;

  int label_id = 0;
  for (const auto& r : records) {
    // Only ACTIVE and DORMANT are visualized.
    if (r.state != FrontierState::ACTIVE && r.state != FrontierState::DORMANT) continue;

    geometry_msgs::msg::Point p;
    p.x = r.centroid_xy.x();
    p.y = r.centroid_xy.y();
    p.z = z0 + 0.1;
    centroids.points.push_back(p);
    centroids.colors.push_back(colorForState(r.state));

    visualization_msgs::msg::Marker label;
    label.header.frame_id = par_.map_frame_id;
    label.header.stamp = stamp;
    label.ns = "frontier_labels";
    label.id = label_id++;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = r.centroid_xy.x();
    label.pose.position.y = r.centroid_xy.y();
    label.pose.position.z = z0 + 0.6;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.4;
    label.color = makeColor(0.0, 0.0, 0.0, 1.0);
    {
      char buf[64];
      if (r.pursuit_deadline_t > 0.0 && r.pursuit_budget_sec > 0.0) {
        const double t_now = stamp.seconds();
        const double elapsed =
            r.pursuit_budget_sec - std::max(0.0, r.pursuit_deadline_t - t_now);
        std::snprintf(buf, sizeof(buf), "id=%lu\n%.1fs/%.1fs", static_cast<unsigned long>(r.id),
                      elapsed, r.pursuit_budget_sec);
      } else {
        std::snprintf(buf, sizeof(buf), "id=%lu", static_cast<unsigned long>(r.id));
      }
      label.text = buf;
    }
    arr.markers.push_back(label);
  }
  if (!centroids.points.empty()) arr.markers.push_back(centroids);

  // Yellow line from robot to current exploration goal.
  if (selector_->explorationActive() && selector_->stateInitialized()) {
    const Pose cur = selector_->pose();
    const auto* r = fm.find(selector_->currentExploreId());
    if (r) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = par_.map_frame_id;
      line.header.stamp = stamp;
      line.ns = "frontier_goal_line";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.15;
      line.color = makeColor(1.0, 1.0, 0.0, 0.9);
      line.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point a;
      a.x = cur.x;
      a.y = cur.y;
      a.z = z0 + 0.1;
      geometry_msgs::msg::Point b;
      b.x = r->centroid_xy.x();
      b.y = r->centroid_xy.y();
      b.z = z0 + 0.1;
      line.points.push_back(a);
      line.points.push_back(b);
      arr.markers.push_back(line);
    }
  }

  // Yellow rectangle showing the exploration bounds, plus a text label.
  if (par_.expl_bounds_enabled) {
    const double z = z0 + 0.05;
    const double x0 = par_.expl_bounds_min_x;
    const double x1 = par_.expl_bounds_max_x;
    const double y0 = par_.expl_bounds_min_y;
    const double y1 = par_.expl_bounds_max_y;

    visualization_msgs::msg::Marker rect;
    rect.header.frame_id = par_.map_frame_id;
    rect.header.stamp = stamp;
    rect.ns = "exploration_bounds";
    rect.id = 0;
    rect.type = visualization_msgs::msg::Marker::LINE_STRIP;
    rect.action = visualization_msgs::msg::Marker::ADD;
    rect.scale.x = 0.10;
    rect.color = makeColor(1.0, 1.0, 0.0, 0.9);
    rect.pose.orientation.w = 1.0;
    auto pt = [&](double x, double y) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      return p;
    };
    rect.points.push_back(pt(x0, y0));
    rect.points.push_back(pt(x1, y0));
    rect.points.push_back(pt(x1, y1));
    rect.points.push_back(pt(x0, y1));
    rect.points.push_back(pt(x0, y0));
    arr.markers.push_back(rect);

    visualization_msgs::msg::Marker label;
    label.header.frame_id = par_.map_frame_id;
    label.header.stamp = stamp;
    label.ns = "exploration_bounds_label";
    label.id = 0;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = 0.5 * (x0 + x1);
    label.pose.position.y = y1 + 0.5;
    label.pose.position.z = z + 0.5;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.6;
    label.color = makeColor(1.0, 1.0, 0.0, 0.9);
    label.text = "Exploration Area";
    arr.markers.push_back(label);
  }

  pub_frontiers_->publish(arr);
}

}  // namespace goal_selector

// ----------------------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<goal_selector::GoalSelectorNode>();
  rclcpp::spin(node);  // single-threaded: all callbacks share one mutually exclusive group
  rclcpp::shutdown();
  return 0;
}
