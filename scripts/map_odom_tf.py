#!/usr/bin/env python3
"""Broadcast a constant identity <ns>/map -> <ns>/odom on DYNAMIC /tf.

    map_odom_tf.py <ns>

For a vehicle whose localization publishes odometry in <ns>/odom but nothing
publishes <ns>/map -> <ns>/odom: the planner plans in <ns>/map and the MPC
resolves it, so without this edge both wait in wait_for_tf and the MPC commands
0/0. docker/mighty_hw.sh runs it as an extra pane when
MIGHTY_PUBLISH_MAP_ODOM_TF=true. Do not enable it where something else already
publishes that edge (on the RR fleet, dlio.service does).

Why periodic on /tf and not a one-shot static_transform_publisher on /tf_static:
the latter latches ONE TRANSIENT_LOCAL sample, and under rmw_zenoh a listener
that starts after it was published frequently never receives it (seen on the RR
fleet 2026-09-17 and 09-19, also with every container on the same rmw build).
tf2 listeners take /tf as VOLATILE, so a periodic broadcast reaches every
consumer regardless of start order.

Stamped slightly in the FUTURE (like robot_localization's transform_time_offset)
so a lookup at an odom sample's own timestamp interpolates instead of throwing
"extrapolation into the future"; the transform is constant, so that is exact.

Adopted from dlio_ws scripts/map_odom_tf.py (d86594c).
"""
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

RATE_HZ = 20.0
AHEAD = Duration(seconds=0.2)


def main() -> None:
    if len(sys.argv) < 2 or not sys.argv[1]:
        print('usage: map_odom_tf.py <namespace>   (e.g. RR08)', file=sys.stderr)
        sys.exit(2)
    ns = sys.argv[1].strip('/')

    rclpy.init()
    node = Node('map_odom_tf_broadcaster')
    pub = node.create_publisher(TFMessage, '/tf', 100)
    node.get_logger().info(
        'publishing identity %s/map -> %s/odom on /tf at %.0f Hz (stamped +%.1fs)'
        % (ns, ns, RATE_HZ, AHEAD.nanoseconds / 1e9))

    def tick() -> None:
        t = TransformStamped()
        t.header.stamp = (node.get_clock().now() + AHEAD).to_msg()
        t.header.frame_id = '%s/map' % ns
        t.child_frame_id = '%s/odom' % ns
        t.transform.rotation.w = 1.0
        pub.publish(TFMessage(transforms=[t]))

    node.create_timer(1.0 / RATE_HZ, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
