#!/usr/bin/env python3
"""Bag test (1) and (4): every published local trajectory / executable prefix versus the
occ_2d_topic grid of that moment.

WHAT IT CHECKS
  Channels (all in the map frame, compared with the latest raw `occ_2d_topic` grid received
  BEFORE the message; the grid age is recorded):
    trajectory      <ns>/trajectory        dynus_interfaces/Trajectory  (the local plan = the
                    "executed" trajectory MPC follows; goals[i].p is the sample, resampled here
                    every res/2 along straight segments)
    mpc_waypoints   <ns>/mpc_waypoints     dynus_interfaces/SpeedyPath  (path handed to MPC)
    prefix          <ns>/hgp_path_marker   MarkerArray LINE_STRIP: the global path AFTER the
                    unknown-run trim = the executable prefix (mighty_node.cpp publishGlobalPath)
    global          <ns>/original_hgp_path_marker  untrimmed global path (INFO only)

  Per channel and per message: number of samples that are UNKNOWN (-1), OCCUPIED (>= 50),
  outside the grid, and inside the occupied inflation band (inflation_2d_m, INFO only: the
  start-escape lets the robot leave a band by design).

PASS / FAIL CRITERIA
  trajectory-unknown   INFO  Any crossing of unknown is flagged and COUNTED (messages, distinct
                       trajectory_ids, samples). Not a failure: runs shorter than
                       trim_min_unknown_run_cells are driven through by design and L-BFGS is not
                       hard-constrained to the prefix (spec N3, Answer 4; test_ideas E).
  trajectory-occupied  FAIL if ANY sample of ANY trajectory is in an occupied cell (test_ideas E;
                       spec N2 "global path does not enter occupied space"). Reported prominently
                       with the first offending positions.
  prefix-unknown-run   FAIL if any prefix contains a run of >= trim_min_unknown_run_cells
                       consecutive UNKNOWN cells (spec N3 "Resolved 2026-10-09", §11.14).
                       Shorter runs are INFO (driven through by design).
  prefix-occupied      FAIL if any prefix sample is in an occupied cell (spec N3: an occupied
                       sample stops the path immediately).
  prefix-unknown-short-runs  INFO: prefixes that cross any unknown (short runs) and the grid class of
                       the prefix end cell. (The exact back-off of unknown_inflation_2d_m is not checked:
                       it needs the untrimmed path of the SAME plan; compare 'global' vs 'prefix' by eye.)
  INCONCLUSIVE when no message of the channel (or no map) was received.

  Measurement caveat: the planner builds its map a few hundred ms before it publishes, the
  monitor holds the newest grid, so a crossing of a cell that changed in between is possible
  in principle; the report gives grid_age_s so you can judge. Counts are per message, and the
  planner republishes the same plan every replan tick, so "distinct" counts are given too.

HOW TO RUN (inside mighty-hw:local, ROS sourced, same RMW/zenoh session as the robot):
    python3 monitor_trajectory_vs_map.py --ns RR08 --out /out/trajectory_vs_map.json
  Stop with Ctrl-C (or SIGINT/SIGTERM); the report is printed and written on exit.
  Optional: --duration 600, --trim-cells 3 (default read from config), --unknown-infl 0.5,
  --infl 0.5, --config path/to/hw_mighty_ground_robot.yaml, --grid-topic occ_2d_topic.

OUTPUT
  Text report on stdout + JSON (--out) with per-channel statistics, in `data`.
  Exit code 1 if a FAIL check exists.
"""
import math
import sys
from collections import defaultdict

import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import Trajectory, SpeedyPath
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

import gs_test_common as C


class TrajMonitor(Node):
    def __init__(self, a):
        super().__init__('monitor_trajectory_vs_map')
        self.a = a
        self.grid = None
        self.prev_grid_stamp = None
        ns = '/' + a.ns.strip('/')
        self.stats = {k: self._new() for k in ('trajectory', 'mpc_waypoints', 'prefix', 'global')}
        self.maps_seen = 0
        self.frame_mismatch = set()
        self.create_subscription(OccupancyGrid, f'{ns}/{a.grid_topic}', self.on_grid, C.qos_best_effort(5))
        self.create_subscription(Trajectory, f'{ns}/trajectory', self.on_traj, C.qos_reliable(200))
        self.create_subscription(SpeedyPath, f'{ns}/mpc_waypoints', self.on_wp, C.qos_reliable(200))
        self.create_subscription(MarkerArray, f'{ns}/hgp_path_marker',
                                 lambda m: self.on_marker('prefix', m), C.qos_best_effort(200))
        self.create_subscription(MarkerArray, f'{ns}/original_hgp_path_marker',
                                 lambda m: self.on_marker('global', m), C.qos_best_effort(200))

    @staticmethod
    def _new():
        return dict(msgs=0, no_map=0, short=0, samples=0, unknown_samples=0, occ_samples=0, out_samples=0,
                    band_samples=0, msgs_unknown=0, msgs_occ=0, msgs_band=0, msgs_run=0,
                    max_run=0, ids=set(), ids_unknown=set(), ids_occ=set(), ids_run=set(),
                    offenders_occ=[], offenders_run=[], offenders_unknown=[],
                    backoff=[], ages=[], run_hist=defaultdict(int))

    def on_grid(self, m):
        self.grid = C.Grid(m, C.now_s(self))
        self.maps_seen += 1

    def _eval(self, chan, pts, key, t):
        s = self.stats[chan]
        s['msgs'] += 1
        if len(pts) < 2:
            s['short'] += 1
            return
        g = self.grid
        if g is None:
            s['no_map'] += 1
            return
        step = max(g.res * 0.5, 0.01)
        r = C.classify_polyline(g, pts, step, self.a.infl)
        s['samples'] += r['n']
        s['unknown_samples'] += r['unknown']
        s['occ_samples'] += r['occ']
        s['out_samples'] += r['out']
        s['band_samples'] += r['band']
        s['ages'].append(t - g.recv_t)
        s['ids'].add(key)
        if r['unknown']:
            s['msgs_unknown'] += 1
            s['ids_unknown'].add(key)
            if len(s['offenders_unknown']) < 20:
                s['offenders_unknown'].append(
                    dict(t=round(t, 2), id=key, first_xy=r['first_unknown_xy'], s=round(r['first_unknown'], 2),
                         unknown_samples=r['unknown'], max_run=r['max_unknown_run_cells']))
        if r['occ']:
            s['msgs_occ'] += 1
            s['ids_occ'].add(key)
            if len(s['offenders_occ']) < 20:
                s['offenders_occ'].append(dict(t=round(t, 2), id=key, first_xy=r['first_occ_xy'],
                                               s=round(r['first_occ'], 2), occ_samples=r['occ'],
                                               start_xy=(round(pts[0][0], 2), round(pts[0][1], 2)),
                                               end_xy=(round(pts[-1][0], 2), round(pts[-1][1], 2))))
        if r['band']:
            s['msgs_band'] += 1
        s['max_run'] = max(s['max_run'], r['max_unknown_run_cells'])
        s['run_hist'][r['max_unknown_run_cells']] += 1
        if r['max_unknown_run_cells'] >= self.a.trim_cells:
            s['msgs_run'] += 1
            s['ids_run'].add(key)
            if len(s['offenders_run']) < 20:
                s['offenders_run'].append(dict(t=round(t, 2), id=key, max_run=r['max_unknown_run_cells'],
                                               first_xy=r['first_unknown_xy'],
                                               start_xy=(round(pts[0][0], 2), round(pts[0][1], 2))))
        if chan == 'prefix':
            # Distance (m, along the path, from the END) back to the last sample before the first
            # >= trim run is not recoverable without the untrimmed path, so report the end cell class.
            end = pts[-1]
            s['backoff'].append(g.classify(end[0], end[1]))

    def on_traj(self, m):
        pts = [(g.p.x, g.p.y) for g in m.goals]
        self._eval('trajectory', pts, ('id', int(m.trajectory_id)), C.now_s(self))

    def on_wp(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        self._eval('mpc_waypoints', pts, ('stamp', C.stamp_ns(m.header.stamp)), C.now_s(self))

    def on_marker(self, chan, ma):
        for mk in ma.markers:
            if mk.type == 4 and mk.action == 0 and len(mk.points) >= 2:
                pts = [(p.x, p.y) for p in mk.points]
                self._eval(chan, pts, ('n', len(pts), round(pts[-1][0], 2), round(pts[-1][1], 2)),
                           C.now_s(self))
                return


def build_report(n, a):
    rep = C.Report('Trajectory / prefix versus occ_2d_topic grid',
                   dict(ns=a.ns, grid_topic=a.grid_topic, trim_min_unknown_run_cells=a.trim_cells,
                        unknown_inflation_2d_m=a.unknown_infl, inflation_2d_m=a.infl,
                        grids_received=n.maps_seen))
    D = {}
    for chan, s in n.stats.items():
        ages = sorted(s['ages'])
        D[chan] = dict(msgs=s['msgs'], evaluated=len(s['ages']), samples=s['samples'],
                       msgs_unknown=s['msgs_unknown'], distinct_unknown=len(s['ids_unknown']),
                       msgs_occ=s['msgs_occ'], distinct_occ=len(s['ids_occ']),
                       msgs_run_ge_trim=s['msgs_run'], distinct_run_ge_trim=len(s['ids_run']),
                       msgs_in_band=s['msgs_band'], distinct=len(s['ids']),
                       unknown_samples=s['unknown_samples'], occ_samples=s['occ_samples'],
                       out_samples=s['out_samples'], band_samples=s['band_samples'],
                       max_unknown_run_cells=s['max_run'],
                       run_hist={str(k): v for k, v in sorted(s['run_hist'].items())},
                       grid_age_median_s=ages[len(ages) // 2] if ages else None,
                       grid_age_max_s=ages[-1] if ages else None)
    rep.data = D

    def frac(a_, b_):
        return f'{a_}/{b_} ({100.0 * a_ / b_:.1f}%)' if b_ else f'{a_}/0'

    for chan in ('trajectory', 'mpc_waypoints'):
        s, d = n.stats[chan], D[chan]
        if d['evaluated'] == 0:
            rep.check(f'{chan}-received', C.INCONCLUSIVE,
                      f'no evaluated {chan} message (received {s["msgs"]}, no map yet {s["no_map"]}, '
                      f'<2 samples {s["short"]}). Is the planner producing plans / is occ_2d_topic arriving?')
            continue
        ev = [f'messages evaluated: {d["evaluated"]}, distinct plans: {d["distinct"]}, samples: {d["samples"]}',
              f'grid age at evaluation: median {d["grid_age_median_s"]:.2f}s max {d["grid_age_max_s"]:.2f}s',
              f'samples in occupied band (INFO, start-escape is by design): {d["band_samples"]} '
              f'in {d["msgs_in_band"]} messages']
        ev += [f'unknown offender: {o}' for o in s['offenders_unknown'][:8]]
        rep.check(f'{chan}-unknown', C.INFO,
                  f'unknown crossed in {frac(d["msgs_unknown"], d["evaluated"])} messages, '
                  f'{frac(d["distinct_unknown"], d["distinct"])} distinct plans, '
                  f'{d["unknown_samples"]} samples; longest unknown run {d["max_unknown_run_cells"]} cells '
                  f'(trim threshold {a.trim_cells}); outside-grid samples {d["out_samples"]}',
                  ev, data=d, ref='test_ideas E; spec N3 Answer 4')
        ev = [f'occupied offender: {o}' for o in s['offenders_occ'][:10]]
        if d['msgs_occ']:
            rep.check(f'{chan}-occupied', C.FAIL,
                      f'!!! OCCUPIED CROSSING: {frac(d["msgs_occ"], d["evaluated"])} messages, '
                      f'{frac(d["distinct_occ"], d["distinct"])} distinct plans, {d["occ_samples"]} samples',
                      ev, ref='test_ideas E; spec N2/N3')
        else:
            rep.check(f'{chan}-occupied', C.PASS,
                      f'no occupied sample in {d["evaluated"]} messages / {d["samples"]} samples',
                      ref='test_ideas E; spec N2/N3')

    s, d = n.stats['prefix'], D['prefix']
    if d['evaluated'] == 0:
        rep.check('prefix-received', C.INCONCLUSIVE,
                  f'no evaluated hgp_path_marker (received {s["msgs"]}); cannot test the prefix')
    else:
        rep.check('prefix-unknown-run', C.FAIL if d['msgs_run_ge_trim'] else C.PASS,
                  f'prefixes with a run >= {a.trim_cells} unknown cells: '
                  f'{frac(d["msgs_run_ge_trim"], d["evaluated"])} messages, '
                  f'{d["distinct_run_ge_trim"]} distinct; longest run {d["max_unknown_run_cells"]} cells; '
                  f'histogram of per-prefix longest run {d["run_hist"]}',
                  [f'offender: {o}' for o in s['offenders_run'][:10]], ref='spec N3, §11.14, test_ideas E')
        rep.check('prefix-occupied', C.FAIL if d['msgs_occ'] else C.PASS,
                  f'prefixes with an occupied sample: {frac(d["msgs_occ"], d["evaluated"])} messages, '
                  f'{d["occ_samples"]} samples',
                  [f'offender: {o}' for o in s['offenders_occ'][:10]], ref='spec N3')
        ends = defaultdict(int)
        for e in s['backoff']:
            ends[e] += 1
        rep.check('prefix-unknown-short-runs', C.INFO,
                  f'prefixes that cross any unknown (runs < {a.trim_cells} cells, driven through by '
                  f'design): {frac(d["msgs_unknown"], d["evaluated"])}; end-point cell class {dict(ends)}',
                  ref='spec N3')
    s, d = n.stats['global'], D['global']
    if d['evaluated']:
        rep.check('global-path-unknown', C.INFO,
                  f'untrimmed global path crosses unknown in {frac(d["msgs_unknown"], d["evaluated"])} messages, '
                  f'occupied in {frac(d["msgs_occ"], d["evaluated"])} (A* may price unknown at w_unknown, '
                  f'never occupied: spec N2)', ref='spec N2')
    return rep


def main():
    ap = C.base_parser('Check published trajectories / prefix against occ_2d_topic', __doc__)
    ap.add_argument('--grid-topic', default='occ_2d_topic')
    ap.add_argument('--trim-cells', type=int, default=None)
    ap.add_argument('--unknown-infl', type=float, default=None)
    ap.add_argument('--infl', type=float, default=None)
    a, ros_args = ap.parse_known_args()
    a.trim_cells = a.trim_cells if a.trim_cells is not None else int(
        C.load_param('trim_min_unknown_run_cells', 3, a.config))
    a.unknown_infl = a.unknown_infl if a.unknown_infl is not None else float(
        C.load_param('unknown_inflation_2d_m', 0.5, a.config))
    a.infl = a.infl if a.infl is not None else float(C.load_param('inflation_2d_m', 0.5, a.config))
    rclpy.init(args=ros_args)
    n = TrajMonitor(a)
    flag = C.StopFlag(a.duration)
    C.spin_until(n, flag)
    rep = build_report(n, a)
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
