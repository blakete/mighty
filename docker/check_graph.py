"""ROS-graph half of `mighty_hw.sh check`: piped into the container's python3.

Read-only. Joins the graph for a few seconds, then reports one line per check,
"PASS|WARN|FAIL <message>", which mighty_hw.sh folds into its report (other
lines are printed as notes). Settings
arrive as environment variables (NS, PLATFORM, STATE_SOURCE, ODOM_TOPIC,
POSE_TOPIC, TWIST_TOPIC, TF_GATE, MAP_ODOM_PANE, STACK_UP).
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

NS = os.environ['NS'].strip('/')
PLATFORM = os.environ.get('PLATFORM', 'ground_robot')
STATE_SOURCE = os.environ.get('STATE_SOURCE', 'odom')
TF_GATE = os.environ.get('TF_GATE', 'none').split()
STACK_UP = os.environ.get('STACK_UP') == '1'
MAP_ODOM_PANE = os.environ.get('MAP_ODOM_PANE') == 'true'
DISCOVERY_S = 4.0


def topic(name):
    return name if name.startswith('/') else f'/{NS}/{name}'


def report(level, msg):
    print(f'{level} {msg}', flush=True)


rclpy.init()
node = Node('mighty_check')
tf_buffer = Buffer()
TransformListener(tf_buffer, node)
end = time.time() + DISCOVERY_S
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)


def publishers(name):
    return node.count_publishers(topic(name))


def subscribers(name):
    return node.count_subscribers(topic(name))


def need_publisher(name, level, why):
    n = publishers(name)
    report('PASS' if n else level, f'{topic(name)}: {n} publisher(s)' + ('' if n else f' — {why}'))


# State source
if STATE_SOURCE == 'odom':
    need_publisher(os.environ.get('ODOM_TOPIC') or 'odom', 'FAIL',
                   'no odometry, so the planner never gets a state')
else:
    need_publisher(os.environ.get('POSE_TOPIC') or 'world', 'FAIL', 'no mocap pose')
    need_publisher(os.environ.get('TWIST_TOPIC') or 'twist', 'FAIL', 'no mocap twist')

# Maps and outputs
if PLATFORM == 'ground_robot':
    need_publisher('planning_occ_2d_topic', 'WARN',
                   'the planner will not plan without it (REPLAN ok=0)')
    need_publisher('occ_2d_topic', 'WARN', 'no 2D occupancy (frontiers, collision map)')
    need_publisher('esdf_2d_topic', 'WARN', 'no ESDF (obstacle cost)')
    n = subscribers('cmd_vel_auto')  # the vehicle's drive stack
    report('PASS' if n else 'WARN',
           f'{topic("cmd_vel_auto")}: {n} subscriber(s)'
           + ('' if n else ' — nothing drives the vehicle from the MPC output'))
else:
    need_publisher('occupancy_grid', 'WARN', 'the planner waits for the first cloud')
    need_publisher('unknown_grid', 'WARN', 'no unknown-space cloud')
    n = subscribers('goal')
    if STACK_UP:
        report('PASS' if n else 'WARN', f'{topic("goal")}: {n} subscriber(s)'
               + ('' if n else ' — nothing consumes the planner setpoints'))

# TF the planner and MPC wait for
if TF_GATE and TF_GATE != ['none']:
    pairs = list(zip(TF_GATE[0::2], TF_GATE[1::2]))
    end = time.time() + 3.0
    while time.time() < end and not all(
            tf_buffer.can_transform(t, s, Time()) for t, s in pairs):
        rclpy.spin_once(node, timeout_sec=0.1)
    for target, source in pairs:
        ok = tf_buffer.can_transform(target, source, Time())
        if ok and MAP_ODOM_PANE and not STACK_UP and source.endswith('/odom'):
            report('WARN', f'TF {target} -> {source} already exists, but '
                           'MIGHTY_PUBLISH_MAP_ODOM_TF=true would publish it a second time')
        elif ok:
            report('PASS', f'TF {target} -> {source}')
        elif MAP_ODOM_PANE and not STACK_UP:
            report('PASS', f'TF {target} -> {source} missing (the map->odom pane will publish it)')
        else:
            report('FAIL', f'TF {target} -> {source} missing — planner and MPC wait 60 s, '
                           'then run degraded (MPC commands 0)')

# A running stack: exactly one of each output
if STACK_UP:
    n = publishers('state')
    report('PASS' if n == 1 else 'FAIL',
           f'{topic("state")}: {n} publisher(s)' + ('' if n == 1 else ' — expected exactly 1'))
    if PLATFORM == 'ground_robot':
        n = publishers('cmd_vel_auto')
        report('PASS' if n == 1 else 'FAIL', f'{topic("cmd_vel_auto")}: {n} publisher(s)'
               + ('' if n == 1 else ' — expected exactly 1 (0: MPC down; 2+: double stack)'))

print('note: with use_frame_alignment (on in both platform configs) peer trajectories on '
      '/trajs are ignored until /frame_align/<ns>/<peer> arrives; nothing publishes that '
      'on hardware, so vehicles do not avoid each other\'s plans', flush=True)
node.destroy_node()
rclpy.shutdown()
