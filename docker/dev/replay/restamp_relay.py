#!/usr/bin/env python3
"""Re-stamp replayed bag topics onto the wall clock, preserving relative timing.

Why: the rover services (dlio.service, the Orin elevation mapper, mighty) run on
wall time, and a weeks-old bag played with --clock would need every consumer on
use_sim_time. Instead, play the bag with each topic remapped under IN_PREFIX
(ros2 bag play --remap /RR08/livox/lidar:=/replay/RR08/livox/lidar ...) and let
this node republish it on the original name with ONE constant offset added to
every stamp:  offset = now - stamp_of_first_message.  Relative timing between
messages and topics is therefore exact; only the epoch moves.

Handles: any message with header.stamp; tf2_msgs/TFMessage (each transform);
sensor_msgs/PointCloud2 with a Livox per-point float64 'timestamp' field in ns
(DLIO deskews with it — shifting only the header would break odometry).

    restamp_relay.py [--in-prefix /replay] TOPIC=TYPE [TOPIC=TYPE ...]
    e.g. restamp_relay.py /RR08/livox/lidar=sensor_msgs/msg/PointCloud2 \
                          /RR08/livox/imu=sensor_msgs/msg/Imu
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message


NS = 1_000_000_000


def stamp_to_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def set_stamp_ns(stamp, ns):
    stamp.sec = int(ns // NS)
    stamp.nanosec = int(ns % NS)


class RestampRelay(Node):
    def __init__(self, in_prefix, topics):
        super().__init__('restamp_relay')
        self.offset_ns = None          # fixed once, from the first message seen
        self.count = {}
        self.pubs = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50,
                         durability=DurabilityPolicy.VOLATILE)
        # /tf_static is latched: tf2 listeners subscribe transient-local, and a
        # volatile publisher never reaches them. Match the bag player's QoS.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=100,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, type_name in topics:
            msg_type = get_message(type_name)
            q = latched if topic.endswith('tf_static') else qos
            self.pubs[topic] = self.create_publisher(msg_type, topic, q)
            self.create_subscription(msg_type, in_prefix + topic,
                                     lambda m, t=topic: self.relay(t, m), q)
            self.count[topic] = 0
            self.get_logger().info(f'{in_prefix}{topic} -> {topic} ({type_name})')
        self.create_timer(5.0, self.report)

    def first_stamp_ns(self, msg):
        if hasattr(msg, 'header'):
            return stamp_to_ns(msg.header.stamp)
        if hasattr(msg, 'transforms') and msg.transforms:
            return stamp_to_ns(msg.transforms[0].header.stamp)
        return None

    def shift(self, msg):
        off = self.offset_ns
        if hasattr(msg, 'header'):
            set_stamp_ns(msg.header.stamp, stamp_to_ns(msg.header.stamp) + off)
        if hasattr(msg, 'transforms'):
            for tr in msg.transforms:
                set_stamp_ns(tr.header.stamp, stamp_to_ns(tr.header.stamp) + off)
        # Livox PointCloud2: per-point absolute timestamp (float64 ns)
        if hasattr(msg, 'fields') and hasattr(msg, 'point_step'):
            ts = [f for f in msg.fields if f.name == 'timestamp' and f.datatype == 8]
            if ts and msg.width * msg.height > 0:
                n = msg.width * msg.height
                buf = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=n * msg.point_step).copy()
                view = np.ndarray((n,), dtype='<f8', buffer=buf, offset=ts[0].offset,
                                  strides=(msg.point_step,))
                view += float(off)
                msg.data = buf.tobytes()

    def relay(self, topic, msg):
        if self.offset_ns is None:
            first = self.first_stamp_ns(msg)
            if first is None:
                return
            self.offset_ns = self.get_clock().now().nanoseconds - first
            self.get_logger().info(f'offset fixed from {topic}: +{self.offset_ns / NS:.3f} s')
        self.shift(msg)
        self.pubs[topic].publish(msg)
        self.count[topic] += 1

    def report(self):
        self.get_logger().info('relayed ' + ', '.join(f'{t}:{n}' for t, n in self.count.items()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in-prefix', default='/replay')
    ap.add_argument('pairs', nargs='+', metavar='TOPIC=TYPE')
    args = ap.parse_args()
    topics = []
    for p in args.pairs:
        if '=' not in p:
            sys.exit(f'bad TOPIC=TYPE: {p}')
        t, ty = p.split('=', 1)
        topics.append((t, ty))
    rclpy.init()
    node = RestampRelay(args.in_prefix, topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
