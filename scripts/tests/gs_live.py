#!/usr/bin/env python3
"""Live recorder node shared by the hw_check_*.py hardware monitors (not run directly).

Subscribes to everything the selector / planner expose for ONE robot and keeps timestamped
histories (receipt time from the node clock, so wall time on a robot, sim time with
`--ros-args -p use_sim_time:=true`). It also prints a one-line live log of every event so the
operator sees what the monitor sees ("[  12.3s] term_goal ...").

Topics (relative to /<ns>/): state (dynus_interfaces/State), term_goal, term_goal_rviz,
exploration/current_goal (PoseStamped), planner_status (goal_selector_msgs/PlannerStatus),
goal_reached (Empty), /exploration/return_home (Empty, global), exploration/frontiers
(MarkerArray, only the marker count is kept).
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray
from dynus_interfaces.msg import State
from goal_selector_msgs.msg import PlannerStatus

import gs_test_common as C

STATUS = {0: 'SUCCESS', 1: 'FAILED', 2: 'PARTIAL', 3: 'REACHED', 4: 'SKIPPED'}


class Live(Node):
    def __init__(self, ns, name='hw_check', quiet_status=True):
        super().__init__(name)
        self.ns = ns.strip('/')
        self.t0 = C.now_s(self)
        self.quiet_status = quiet_status
        self.pose = None                  # (t, x, y)
        self.pose_hist = []               # (t, x, y), ~every state msg
        self.tg, self.rviz, self.cg = [], [], []       # (t, stamp_ns, x, y) / (t, x, y) / (t, x, y)
        self.st = []                      # (t, status, goal_stamp_ns)
        self.reached = []                 # (t,) goal_reached Empty
        self.return_home = []             # (t,)
        self.n_frontier_markers = 0
        p = f'/{self.ns}/'
        qr = C.qos_reliable
        self.create_subscription(State, p + 'state', self._state, qr(50))
        self.create_subscription(PoseStamped, p + 'term_goal', self._tg, qr(100))
        self.create_subscription(PoseStamped, p + 'term_goal_rviz', self._rviz, qr(100))
        self.create_subscription(PoseStamped, p + 'exploration/current_goal', self._cg, qr(100))
        self.create_subscription(PlannerStatus, p + 'planner_status', self._st, qr(500))
        self.create_subscription(Empty, p + 'goal_reached', self._reached, qr(100))
        self.create_subscription(Empty, '/exploration/return_home', self._rh, qr(10))
        self.create_subscription(MarkerArray, p + 'exploration/frontiers', self._fr, qr(5))

    # -- time helpers
    def now(self):
        return C.now_s(self)

    def rel(self, t=None):
        return (self.now() if t is None else t) - self.t0

    def say(self, text, t=None):
        print(f'[{self.rel(t):7.1f}s] {text}', flush=True)

    # -- callbacks
    def _state(self, m):
        t = self.now()
        self.pose = (t, m.pos.x, m.pos.y)
        self.pose_hist.append(self.pose)
        if len(self.pose_hist) > 20000:
            del self.pose_hist[:5000]

    def _tg(self, m):
        e = (self.now(), C.stamp_ns(m.header.stamp), m.pose.position.x, m.pose.position.y)
        self.tg.append(e)
        self.say(f'term_goal stamp={e[1]} ({e[2]:.2f}, {e[3]:.2f})', e[0])

    def _rviz(self, m):
        e = (self.now(), m.pose.position.x, m.pose.position.y)
        self.rviz.append(e)
        self.say(f'term_goal_rviz ({e[1]:.2f}, {e[2]:.2f})', e[0])

    def _cg(self, m):
        e = (self.now(), m.pose.position.x, m.pose.position.y)
        self.cg.append(e)
        self.say(f'exploration/current_goal ({e[1]:.2f}, {e[2]:.2f})', e[0])

    def _st(self, m):
        e = (self.now(), int(m.status), C.stamp_ns(m.goal_stamp))
        self.st.append(e)
        if e[1] == 3 or not self.quiet_status:
            self.say(f'planner_status {STATUS.get(e[1], e[1])} goal_stamp={e[2]}', e[0])

    def _reached(self, _):
        self.reached.append(self.now())
        self.say('goal_reached')

    def _rh(self, _):
        self.return_home.append(self.now())
        self.say('/exploration/return_home trigger')

    def _fr(self, m):
        self.n_frontier_markers = sum(1 for mk in m.markers if mk.action == 0)

    # -- queries
    def dist_to(self, x, y):
        if self.pose is None:
            return None
        return math.hypot(self.pose[1] - x, self.pose[2] - y)

    def pose_at(self, t):
        """Last pose received at or before t (None if none)."""
        best = None
        for p in self.pose_hist:
            if p[0] <= t:
                best = p
            else:
                break
        return best

    def max_displacement(self, t_from, t_to=None):
        """Largest distance of any pose in [t_from, t_to] from the first pose of that interval."""
        t_to = self.now() if t_to is None else t_to
        pts = [p for p in self.pose_hist if t_from <= p[0] <= t_to]
        if len(pts) < 2:
            return 0.0
        x0, y0 = pts[0][1], pts[0][2]
        return max(math.hypot(p[1] - x0, p[2] - y0) for p in pts)

    def is_frontier_goal(self, tg):
        """term_goal tuple -> True when an exploration/current_goal with the same x, y exists within 2 s."""
        return any(abs(c[0] - tg[0]) <= 2.0 and math.hypot(c[1] - tg[2], c[2] - tg[3]) < 1e-3 for c in self.cg)

    def is_manual_goal(self, tg):
        return any(-0.5 <= tg[0] - r[0] <= 3.0 and math.hypot(r[1] - tg[2], r[2] - tg[3]) < 1e-3
                   for r in self.rviz)

    def first_status(self, stamp_ns, status, after=0.0):
        for (t, s, g) in self.st:
            if g == stamp_ns and s == status and t >= after:
                return t
        return None

    def count_status(self, stamp_ns, status):
        return sum(1 for (_, s, g) in self.st if g == stamp_ns and s == status)


def run(node, flag, tick, period=0.2):
    """Spin until flag is set or tick() returns True (= finished)."""
    last = 0.0
    while rclpy.ok() and not flag:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time()
        if now - last >= period:
            last = now
            if tick():
                return


def wait_for_robot(node, flag, timeout=15.0):
    t0 = time.time()
    while rclpy.ok() and not flag and node.pose is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.pose is not None
