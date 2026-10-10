/**
 * @file goal_selector_node.hpp
 * @brief Thin ROS 2 wrapper around goal_selector::GoalSelector.
 *
 * Declares parameters, creates subscriptions / publishers / the select timer, converts messages
 * to the core's plain inputs and performs the side effects the core returns in its Output.
 * All callbacks and the timer share one mutually exclusive callback group, so the (not thread
 * safe) core is never entered concurrently.
 *
 * Topics (relative names resolve in the robot namespace, e.g. /RR04):
 *   sub  state                    dynus_interfaces/State         critical_qos (reliable, KeepLast(10), volatile)
 *   sub  occ_2d_topic             nav_msgs/OccupancyGrid         SensorDataQoS
 *   sub  term_goal_rviz           geometry_msgs/PoseStamped      reliable, KeepLast(10), volatile
 *   sub  planner_status           goal_selector_msgs/PlannerStatus  critical_qos (reliable)
 *   pub  term_goal                geometry_msgs/PoseStamped      critical_qos (as the planner's subscription)
 *   pub  selector_map_2d          nav_msgs/OccupancyGrid         QoS(1).reliable().transient_local, <= 1 Hz
 *                                 (inflated grid used for frontiers; 0 free, 100 obstacle, 99 occupied
 *                                 band, 50 unknown band, -1 unknown; like the planner's planning_map_2d)
 *   pub  point_selector_goal      geometry_msgs/PointStamped     KeepLast(1), best effort (viz; with each term_goal)
 *   pub  exploration/frontiers    visualization_msgs/MarkerArray KeepLast(10)
 *   pub  exploration/current_goal geometry_msgs/PoseStamped      KeepLast(10)
 *   pub  exploration/visited_map  nav_msgs/OccupancyGrid         QoS(1).transient_local
 *   MinPos (exploration.minpos.enabled), global names:
 *   pub+sub /exploration/peer_poses    geometry_msgs/PoseStamped  QoS(10).reliable()
 *   pub+sub /exploration/visited_maps  nav_msgs/OccupancyGrid     QoS(1).reliable()
 *   sub     /exploration/return_home   std_msgs/Empty             QoS(1).reliable()
 *
 * Gate (spec 4): state, term_goal_rviz, planner_status and the term_goal publisher
 * need only a ground robot with 2D planning (manual goals work with exploration off).
 * occ_2d_topic and the selector_map_2d / point_selector_goal publishers are also created with
 * exploration off (the map is only stored and published, no frontier detection runs). The
 * visited-map, peer and return-home topics, the exploration/... publishers and the select timer
 * additionally need exploration.enabled. Otherwise the node idles.
 */
#pragma once

#include <memory>
#include <string>

#include <dynus_interfaces/msg/state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <goal_selector_msgs/msg/planner_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "goal_selector/goal_selector.hpp"

namespace goal_selector {

class GoalSelectorNode : public rclcpp::Node {
 public:
  GoalSelectorNode();

 private:
  void declareParameters();
  void readParameters();
  void printParameters() const;

  // callbacks
  void stateCallback(const dynus_interfaces::msg::State::SharedPtr msg);
  void occ2DCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void manualGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void plannerStatusCallback(const goal_selector_msgs::msg::PlannerStatus::SharedPtr msg);
  void selectTimerCallback();

  // side effects
  void apply(Output&& out);
  void publishGoal(const GoalCommand& cmd);
  void publishFrontierMarkers();
  void publishExplorationCurrentGoal(const Eigen::Vector3d& g);
  void publishVisitedMap();
  void maybePublishSelectorMap();
  void broadcastVisitedMap();

  static GridInput toGridInput(const nav_msgs::msg::OccupancyGrid& msg);

  SelectorParams par_;
  std::string vehicle_type_{"uav"};
  bool use_2d_planning_{false};
  std::unique_ptr<GoalSelector> selector_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp::Subscription<dynus_interfaces::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occ_2d_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_term_goal_rviz_;
  rclcpp::Subscription<goal_selector_msgs::msg::PlannerStatus>::SharedPtr sub_planner_status_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_peer_pose_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_peer_visited_map_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_return_home_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_term_goal_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_selector_map_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_selector_goal_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frontiers_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_explore_current_goal_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_visited_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_peer_pose_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_peer_visited_map_;

  rclcpp::TimerBase::SharedPtr timer_select_;

  std_msgs::msg::Header last_occ_header_;   // header of the latest raw occ_2d (selector_map_2d, exploration off)
  double last_selector_map_pub_t_{-1.0e18}; // 1 Hz throttle of selector_map_2d
};

}  // namespace goal_selector
