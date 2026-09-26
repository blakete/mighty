#pragma once

#include <dynus_interfaces/msg/state.hpp>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

/** @brief ROS 2 node that converts nav_msgs/Odometry to dynus_interfaces/State.
 *
 *  Subscribes to an odometry topic and republishes it as a State message
 *  suitable for consumption by the MIGHTY planner, and its pose as a
 *  PoseStamped on `pose` (the MPC's pose input), so no consumer needs to know
 *  which localization stack produced the odometry.
 */
class OdometryToStateNode : public rclcpp::Node {
 public:
  /** @brief Construct the node, set up subscriber and publisher. */
  OdometryToStateNode();

 private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};
