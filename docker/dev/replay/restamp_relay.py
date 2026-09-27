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

    restamp_relay.py [--in-prefix /replay] [--hold HZ] TOPIC=TYPE [TOPIC=TYPE ...]
    e.g. restamp_relay.py /RR08/livox/lidar=sensor_msgs/msg/PointCloud2 \
                          /RR08/livox/imu=sensor_msgs/msg/Imu

--hold HZ: once the bag stops (no input for 1 s), keep republishing the last
message of every topic at HZ, stamped with the current time (for /tf, the last
transform of every parent/child pair). The vehicle then stays at its final pose
with its final map, so goals can be sent to a planner for as long as needed.

Looped playback (ros2 bag play --loop): when the bag's time jumps back by more
than LOOP_JUMP_S, the offset is fixed again from that message, so the relayed
time keeps moving forward (the vehicle jumps back to its start pose each loop).
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
LOOP_JUMP_S = 3.0          # bag time going back this far = a new loop


def stamp_to_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def set_stamp_ns(stamp, ns):
    stamp.sec = int(ns // NS)
    stamp.nanosec = int(ns % NS)


class RestampRelay(Node):
    def __init__(self, in_prefix, topics, hold_hz=0.0):
        super().__init__('restamp_relay')
        self.offset_ns = None          # fixed from the first message seen (and at each loop)
        self.max_raw_ns = None         # latest bag stamp seen, for loop detection
        self.loops = 0
        self.count = {}
        self.pubs = {}
        self.last = {}                 # topic -> last relayed message (for --hold)
        self.last_tf = {}              # (parent, child) -> last TransformStamped on /tf
        self.last_input_ns = None
        self.holding = False
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
        if hold_hz > 0:
            self.create_timer(1.0 / hold_hz, self.hold)

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
        self.last_input_ns = self.get_clock().now().nanoseconds
        self.holding = False
        raw = self.first_stamp_ns(msg)
        if self.offset_ns is None:
            if raw is None:
                return
            self.offset_ns = self.get_clock().now().nanoseconds - raw
            self.max_raw_ns = raw
            self.get_logger().info(f'offset fixed from {topic}: +{self.offset_ns / NS:.3f} s')
        elif raw is not None and not topic.endswith('tf_static'):
            # /tf_static is stamped whenever its publisher started: never a loop signal.
            if raw < self.max_raw_ns - LOOP_JUMP_S * NS:
                self.loops += 1
                back_s = (self.max_raw_ns - raw) / NS
                self.offset_ns = self.get_clock().now().nanoseconds - raw
                self.max_raw_ns = raw
                self.get_logger().info(f'bag looped (#{self.loops}, bag time back {back_s:.1f} s '
                                       f'at {topic}): offset now +{self.offset_ns / NS:.3f} s')
            self.max_raw_ns = max(self.max_raw_ns, raw)
        self.shift(msg)
        self.pubs[topic].publish(msg)
        self.count[topic] += 1
        if topic == '/tf':
            for tr in msg.transforms:
                self.last_tf[(tr.header.frame_id, tr.child_frame_id)] = tr
        else:
            self.last[topic] = msg

    def hold(self):
        now = self.get_clock().now()
        if self.last_input_ns is None or now.nanoseconds - self.last_input_ns < NS:
            return
        if not self.holding:
            self.holding = True
            self.get_logger().info('bag finished: holding the last pose, TF and maps '
                                   '(republished with the current time)')
        stamp = now.to_msg()
        for topic, msg in self.last.items():
            if topic.endswith('tf_static'):
                continue                   # latched: already delivered
            if hasattr(msg, 'header'):
                msg.header.stamp = stamp
            if hasattr(msg, 'twist') and hasattr(msg, 'pose'):   # Odometry: parked
                msg.twist.twist.linear.x = msg.twist.twist.linear.y = msg.twist.twist.linear.z = 0.0
                msg.twist.twist.angular.x = msg.twist.twist.angular.y = msg.twist.twist.angular.z = 0.0
            self.pubs[topic].publish(msg)
        if self.last_tf:
            msg = self.pubs['/tf'].msg_type()
            for tr in self.last_tf.values():
                tr.header.stamp = stamp
                msg.transforms.append(tr)
            self.pubs['/tf'].publish(msg)

    def report(self):
        self.get_logger().info('relayed ' + ', '.join(f'{t}:{n}' for t, n in self.count.items()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in-prefix', default='/replay')
    ap.add_argument('--hold', type=float, default=0.0, metavar='HZ',
                    help='after the bag ends, republish the last messages at HZ')
    ap.add_argument('pairs', nargs='+', metavar='TOPIC=TYPE')
    args = ap.parse_args()
    topics = []
    for p in args.pairs:
        if '=' not in p:
            sys.exit(f'bad TOPIC=TYPE: {p}')
        t, ty = p.split('=', 1)
        topics.append((t, ty))
    rclpy.init()
    node = RestampRelay(args.in_prefix, topics, args.hold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
