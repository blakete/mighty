#!/usr/bin/env python3
"""HARDWARE test (two robots): peer sharing -- peer poses and visited maps (MinPos).

Spec D12 (peer tracking, visited maps moved to the selector), notes C ("visited_map_: absorb, peer
merge (only fills our UNKNOWN cells), fuse into local window, publish/broadcast") and notes A
(MinPos: min_frontier_dist_to_peers_m excludes frontiers near peers).

SETUP: two robots (e.g. RR08 and RR09) on the same zenoh graph, BOTH with
  exploration.enabled: true and exploration.minpos.enabled: true (it is FALSE in hw_goal_selector.yaml
  today; with it off nothing is shared and every check below is INCONCLUSIVE),
  the same map frame / start frame. Let both explore for a few minutes, robots closer than a few
  metres at some point if you want to see the frontier exclusion.

RUN (anywhere on the graph, e.g. a laptop container with the fleet zenoh session):
    python3 hw_check_multirobot.py --ns RR08 --peer RR09 [--timeout 300] [--min-peer-dist 3.0] [--out f.json]
  Ctrl-C (or --timeout) ends it; the report is printed on exit.

TOPICS (global): /exploration/peer_poses (PoseStamped, header.frame_id = robot id),
  /exploration/visited_maps (OccupancyGrid, frame_id = robot id), plus per robot
  /<ns>/exploration/visited_map, /<ns>/exploration/current_goal, /<ns>/state.

CRITERIA
  peer-poses       PASS: both robots' ids are heard on /exploration/peer_poses at >= --min-pose-hz
                   (2 Hz; configured peer_publish_rate_hz is 5) and the published position equals that
                   robot's own state position within --pose-tol (0.5 m). FAIL: one id silent while the
                   other is heard, rate too low, or position mismatch. INCONCLUSIVE: nobody heard.
  peer-visited-maps PASS: both ids published on /exploration/visited_maps at least once with the same
                   geometry (size, resolution, origin). FAIL: only one robot publishes. INCONCLUSIVE: none.
  visited-merge    PASS: of the cells KNOWN (>= 0) in the peer's last broadcast map, at least
                   --merge-frac (95%) are known in OUR own /<ns>/exploration/visited_map (and vice versa),
                   i.e. peer coverage was merged ("only fills our UNKNOWN cells" => ours is a superset).
                   Checked for both directions; needs maps of equal geometry (ASSUMPTION: both robots use
                   the same exploration.visited_map.* settings and frame).
  frontier-vs-peer PASS: every frontier goal of OUR robot (exploration/current_goal) was farther than
                   --min-peer-dist (3.0 = min_frontier_dist_to_peers_m) from the peer's pose (from
                   /exploration/peer_poses) at commit time; FAIL on any violation; checked for both
                   robots. INCONCLUSIVE when no goal was committed while the peer pose was known.

OUTPUT: verdicts + JSON (--out). Exit 1 on FAIL.
"""
import math
import sys
from collections import defaultdict

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import gs_test_common as C
import gs_live as L


class Multi(L.Live):
    def __init__(self, ns, peer):
        super().__init__(ns, 'hw_check_multirobot')
        self.peer_ns = peer.strip('/')
        self.ids = {self.ns, self.peer_ns}
        self.poses = defaultdict(list)      # id -> [(t, x, y)]
        self.own_state = defaultdict(list)  # id -> [(t, x, y)]
        self.goals = defaultdict(list)      # id -> [(t, x, y)]
        self.pmaps = {}                     # id -> last Grid from /exploration/visited_maps
        self.pmaps_n = defaultdict(int)
        self.vmaps = {}                     # id -> last Grid from /<id>/exploration/visited_map
        self.create_subscription(PoseStamped, '/exploration/peer_poses', self._pp, C.qos_reliable(100))
        self.create_subscription(OccupancyGrid, '/exploration/visited_maps', self._pm, C.qos_reliable(2))
        from dynus_interfaces.msg import State
        for rid in self.ids:
            self.create_subscription(PoseStamped, f'/{rid}/exploration/current_goal',
                                     lambda m, rid=rid: self.goals[rid].append(
                                         (self.now(), m.pose.position.x, m.pose.position.y)), C.qos_reliable(50))
            self.create_subscription(OccupancyGrid, f'/{rid}/exploration/visited_map',
                                     lambda m, rid=rid: self.vmaps.__setitem__(rid, C.Grid(m, self.now())),
                                     C.qos_reliable(1, transient_local=True))
            if rid != self.ns:      # own state already handled by Live
                self.create_subscription(State, f'/{rid}/state',
                                         lambda m, rid=rid: self.own_state[rid].append(
                                             (self.now(), m.pos.x, m.pos.y)), C.qos_reliable(50))

    def _pp(self, m):
        self.poses[m.header.frame_id].append((self.now(), m.pose.position.x, m.pose.position.y))

    def _pm(self, m):
        self.pmaps[m.header.frame_id] = C.Grid(m, self.now())
        self.pmaps_n[m.header.frame_id] += 1


def at(hist, t):
    best = None
    for h in hist:
        if h[0] <= t:
            best = h
        else:
            break
    return best


def main():
    ap = C.base_parser('HW: multi-robot peer sharing', __doc__)
    ap.add_argument('--peer', required=True, help='namespace of the second robot, e.g. RR09')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--min-pose-hz', type=float, default=2.0)
    ap.add_argument('--pose-tol', type=float, default=0.5)
    ap.add_argument('--merge-frac', type=float, default=0.95)
    ap.add_argument('--min-peer-dist', type=float, default=None)
    a, ros_args = ap.parse_known_args()
    a.duration = a.duration or a.timeout
    mpd = a.min_peer_dist if a.min_peer_dist is not None else float(
        C.load_param('exploration.minpos.min_frontier_dist_to_peers_m', 3.0, '/cfg/hw_goal_selector.yaml'))
    rclpy.init(args=ros_args)
    n = Multi(a.ns, a.peer)
    flag = C.StopFlag(a.duration)
    L.run(n, flag, lambda: False)
    n.own_state[n.ns] = n.pose_hist
    rep = C.Report('HW: multi-robot peer sharing', dict(robots=sorted(n.ids), min_frontier_dist_to_peers_m=mpd))
    # peer poses
    heard = {rid: n.poses.get(rid, []) for rid in n.ids}
    if not any(heard.values()):
        for nm in ('peer-poses', 'peer-visited-maps', 'visited-merge', 'frontier-vs-peer'):
            rep.check(nm, C.INCONCLUSIVE, 'nothing heard on /exploration/peer_poses: is exploration.minpos.enabled '
                      'true on both robots?')
        sys.exit(rep.finish(a.out))
    ev, bad = [], False
    for rid, h in heard.items():
        if not h:
            ev.append(f'{rid}: SILENT')
            bad = True
            continue
        span = h[-1][0] - h[0][0]
        hz = (len(h) - 1) / span if span > 0 else 0.0
        st = n.own_state.get(rid, [])
        errs = [math.hypot(x - s[1], y - s[2]) for (t, x, y) in h[::5] if (s := at(st, t))]
        worst = max(errs) if errs else None
        ev.append(f'{rid}: {len(h)} poses, {hz:.1f} Hz, max |peer_pose - state| = '
                  f'{"n/a (no state)" if worst is None else "%.2f m" % worst}')
        if hz < a.min_pose_hz or (worst is not None and worst > a.pose_tol):
            bad = True
    rep.check('peer-poses', C.FAIL if bad else C.PASS, 'peer pose broadcast per robot', ev, ref='spec D12, notes A')
    # visited maps
    have = [rid for rid in n.ids if rid in n.pmaps]
    if not have:
        rep.check('peer-visited-maps', C.INCONCLUSIVE, 'no map on /exploration/visited_maps')
    else:
        geo = {rid: (g.w, g.h, round(g.res, 4), round(g.ox, 2), round(g.oy, 2)) for rid, g in n.pmaps.items()}
        same = len(set(geo.values())) == 1
        rep.check('peer-visited-maps', C.PASS if (len(have) == 2 and same) else C.FAIL,
                  f'published by {sorted(have)} (counts {dict(n.pmaps_n)}); geometry {geo}', ref='notes C')
    # merge
    if len(n.pmaps) == 2 and len(n.vmaps) == 2:
        ev, ok = [], True
        for me, other in ((n.ns, n.peer_ns), (n.peer_ns, n.ns)):
            mine, theirs = n.vmaps[me], n.pmaps[other]
            if mine.data.shape != theirs.data.shape:
                ev.append(f'{me} vs {other}: shapes differ {mine.data.shape} / {theirs.data.shape}')
                ok = False
                continue
            known_o = theirs.data >= 0
            frac = float((mine.data[known_o] >= 0).mean()) if known_o.any() else 1.0
            ev.append(f'{me}: {frac * 100:.1f}% of the {other} broadcast known cells ({int(known_o.sum())}) are known in {me}\'s map')
            ok &= frac >= a.merge_frac
        rep.check('visited-merge', C.PASS if ok else C.FAIL, 'peer coverage merged into our visited map', ev,
                  ref='notes C (peer merge only fills our UNKNOWN cells)')
    else:
        rep.check('visited-merge', C.INCONCLUSIVE,
                  f'need both robots\' broadcast maps ({len(n.pmaps)}) and own visited maps ({len(n.vmaps)}); '
                  f'is exploration.visited_map.publish true?')
    # frontier vs peer
    viol, checked = [], 0
    for me, other in ((n.ns, n.peer_ns), (n.peer_ns, n.ns)):
        for (t, gx, gy) in n.goals[me]:
            pp = at(heard.get(other, []), t)
            if pp is None or t - pp[0] > 5.0:
                continue
            checked += 1
            d = math.hypot(gx - pp[1], gy - pp[2])
            if d < mpd:
                viol.append(f'{me} goal ({gx:.1f},{gy:.1f}) at +{t - n.t0:.0f}s is {d:.2f} m from {other}')
    if checked == 0:
        rep.check('frontier-vs-peer', C.INCONCLUSIVE, 'no frontier goal committed while a fresh peer pose was known')
    else:
        rep.check('frontier-vs-peer', C.FAIL if viol else C.PASS,
                  f'{checked} goals checked against the peer pose (limit {mpd} m); {len(viol)} violations',
                  viol[:10], ref='notes A (min_frontier_dist_to_peers_m)')
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
