#!/usr/bin/env python3
"""Bag tests (2) and (5), plus the topic-level part of (3): the planner <-> goal_selector
contract (statuses, goal stamps, published goals) and the two visualisation topics.

WHAT IT RECORDS (all under /<ns>/):
    planner_status (goal_selector_msgs/PlannerStatus, reliable)   planner -> selector
    term_goal / term_goal_rviz (PoseStamped)                      selector -> planner / RViz -> selector
    exploration/current_goal (PoseStamped)                        selector, one per frontier commit
    selector_map_2d (OccupancyGrid, transient_local)              selector viz
    point_selector_goal (PointStamped, best effort)               selector viz
    goal_reached (Empty), state (dynus_interfaces/State)

CHECKS AND CRITERIA  (spec = goal_selector_refactor_spec.md, notes = goal_selector_test_ideas.md)
  status-counts            INFO  SUCCESS/FAILED/PARTIAL/SKIPPED/REACHED counts and rate; zero-stamp
                           statuses (sent before the planner holds any goal; the selector must ignore them).
  failed-bursts            INFO  Runs of consecutive FAILED (SKIPPED is neutral, spec D8; SUCCESS/PARTIAL or
                           a new goal ends the run). Bursts >= unreachable_consec_thresh (default 5) listed.
  failed-burst-reaction    FAIL if a FAILED burst reaching the threshold on a FRONTIER goal (not a manual
                           goal) is not followed by a new term_goal within --react-sec (default 4 s = one
                           1 Hz select tick + margin): spec §3 "FAILED: ... At the threshold: invalidate the
                           frontier ... re-select". Manual goals are exempt (notes B: only warns).
                           INCONCLUSIVE if no such burst occurred.
  status-goal-stamp        FAIL if a status carries a non-zero goal_stamp that no observed term_goal ever had
                           (spec §3: the status echoes the goal's header.stamp). Statuses before the first
                           observed term_goal are not judged (monitor may have joined late).
  stale-status             INFO count of statuses whose goal_stamp is an OLDER goal (not the current one);
                           FAIL if one arrives more than --stale-tol seconds (default 2.0, ASSUMPTION) after
                           a newer term_goal was published (planner not adopting the new goal).
  term-goal-stamps         FAIL if two term_goals share a header.stamp or stamps are not increasing (the
                           stamp is the goal identity, spec §3, notes B/D "older goal ignored").
  no-relocation            FAIL if a term_goal differs by > 1e-3 m but < --reloc-near (3 m) from the
                           exploration/current_goal / term_goal_rviz it follows (= a relocated goal; "No
                           relocation anywhere", spec N5/§11.15, notes B/D). term_goals matching no
                           current_goal and no rviz goal are listed as INFO (return home, or a goal that
                           was published before this monitor started).
  rviz-to-term-goal        PASS if every term_goal_rviz was followed within 2 s by a term_goal with the same
                           x, y (spec D5, notes B "manual goal published unchanged"); INCONCLUSIVE if no
                           manual goal was sent.
  selector-map-values      FAIL if any selector_map_2d cell value is outside {-1, 0, 50, 99, 100} (user's
                           list for the viz map; test-idea item 5). INCONCLUSIVE if never received.
  selector-map-rate        FAIL if more than --map-max-hz*10 maps arrive in any 10 s window (default
                           1.5 Hz -> 15; "at most ~1 Hz"); INFO mean rate.
  point-selector-goal      FAIL unless every term_goal has EXACTLY ONE point_selector_goal within [-1, +3] s
                           with the same x, y (1e-3) and no orphan points (once per term_goal, same
                           position; the topic is best-effort, a loss on a local link would be a finding too).

HOW TO RUN (hw image, ROS sourced; on a live robot or during pass2_laptop.sh):
    python3 monitor_selector_contract.py --ns RR08 --out /out/selector_contract.json
  Stop with Ctrl-C / SIGTERM; the report prints on exit. Options: --thresh 5 (read from config by
  default), --duration S.

OUTPUT: text report + JSON (checks, and `data` with counts for summarize_run.py). Exit 1 if any FAIL.
"""
import math
import sys
from collections import defaultdict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from goal_selector_msgs.msg import PlannerStatus

import gs_test_common as C

NAMES = {0: 'SUCCESS', 1: 'FAILED', 2: 'PARTIAL', 3: 'REACHED', 4: 'SKIPPED'}
ALLOWED_MAP_VALUES = {-1, 0, 50, 99, 100}


class ContractMonitor(Node):
    def __init__(self, a):
        super().__init__('monitor_selector_contract')
        ns = '/' + a.ns.strip('/')
        self.st = []        # (t, status, goal_stamp_ns)
        self.tg = []        # (t, stamp_ns, x, y)
        self.rviz = []      # (t, x, y)
        self.cg = []        # (t, x, y)
        self.pts = []       # (t, x, y)
        self.maps = []      # (t, w, h, frame)
        self.map_values = set()
        self.reached_empty = 0
        qr = C.qos_reliable
        self.create_subscription(PlannerStatus, f'{ns}/planner_status', self.on_status, qr(1000))
        self.create_subscription(PoseStamped, f'{ns}/term_goal', self.on_tg, qr(200))
        self.create_subscription(PoseStamped, f'{ns}/term_goal_rviz', self.on_rviz, qr(200))
        self.create_subscription(PoseStamped, f'{ns}/exploration/current_goal', self.on_cg, qr(200))
        self.create_subscription(OccupancyGrid, f'{ns}/selector_map_2d', self.on_map,
                                 qr(20, transient_local=True))
        self.create_subscription(PointStamped, f'{ns}/point_selector_goal', self.on_pt, C.qos_best_effort(200))
        self.create_subscription(Empty, f'{ns}/goal_reached', self.on_reached, qr(200))

    def on_status(self, m):
        self.st.append((C.now_s(self), int(m.status), C.stamp_ns(m.goal_stamp)))

    def on_tg(self, m):
        self.tg.append((C.now_s(self), C.stamp_ns(m.header.stamp), m.pose.position.x, m.pose.position.y))

    def on_rviz(self, m):
        self.rviz.append((C.now_s(self), m.pose.position.x, m.pose.position.y))

    def on_cg(self, m):
        self.cg.append((C.now_s(self), m.pose.position.x, m.pose.position.y))

    def on_pt(self, m):
        self.pts.append((C.now_s(self), m.point.x, m.point.y))

    def on_reached(self, _):
        self.reached_empty += 1

    def on_map(self, m):
        import numpy as np
        vals = set(int(v) for v in np.unique(np.asarray(m.data, dtype=np.int8)))
        self.map_values |= vals
        self.maps.append((C.now_s(self), m.info.width, m.info.height, m.header.frame_id))


def near(x1, y1, x2, y2, tol):
    return math.hypot(x1 - x2, y1 - y2) <= tol


def evaluate(n, a):
    rep = C.Report('Planner <-> goal_selector contract and visualisation topics',
                   dict(ns=a.ns, unreachable_consec_thresh=a.thresh, react_sec=a.react_sec,
                        stale_tol=a.stale_tol, status_msgs=len(n.st), term_goals=len(n.tg)))
    # ---- status counts
    cnt = defaultdict(int)
    zero = 0
    for (_, s, gs) in n.st:
        cnt[NAMES.get(s, f'?{s}')] += 1
        zero += gs == 0
    span = (n.st[-1][0] - n.st[0][0]) if len(n.st) > 1 else 0.0
    rep.data = dict(status_counts=dict(cnt), zero_stamp_statuses=zero, status_total=len(n.st),
                    status_rate_hz=(len(n.st) / span if span > 0 else None), term_goals=len(n.tg),
                    goal_reached_msgs=n.reached_empty, rviz_goals=len(n.rviz),
                    current_goals=len(n.cg))
    if not n.st:
        rep.check('status-counts', C.INCONCLUSIVE, 'no planner_status received (planner not running?)')
    else:
        rep.check('status-counts', C.INFO,
                  ' '.join(f'{k}={cnt.get(k, 0)}' for k in NAMES.values()) +
                  f'  total={len(n.st)}  rate={len(n.st) / span:.1f} Hz  zero-stamp={zero}' if span > 0 else
                  f'total={len(n.st)}', ref='spec §3')

    # ---- goal identity: stamps, increasing
    tg_sorted = sorted(n.tg)
    stamps = [s for (_, s, _, _) in tg_sorted]
    dup = len(stamps) - len(set(stamps))
    nonmono = sum(1 for i in range(1, len(stamps)) if stamps[i] <= stamps[i - 1])
    if not n.tg:
        rep.check('term-goal-stamps', C.INCONCLUSIVE, 'no term_goal published during the run')
    else:
        rep.check('term-goal-stamps', C.FAIL if (dup or nonmono) else C.PASS,
                  f'{len(n.tg)} term_goals, {dup} duplicate stamps, {nonmono} non-increasing', ref='spec §3')

    # Goal manual-ness: a term_goal is MANUAL when it matches a term_goal_rviz received up to 3 s earlier.
    manual_stamps = set()
    rviz_ok, rviz_bad = 0, []
    for (tr, rx, ry) in n.rviz:
        m = [g for g in n.tg if 0 <= g[0] - tr <= 2.0 and near(g[2], g[3], rx, ry, 1e-3)]
        if m:
            rviz_ok += 1
            manual_stamps.add(m[0][1])
        else:
            rviz_bad.append((round(tr, 1), round(rx, 2), round(ry, 2)))
    if not n.rviz:
        rep.check('rviz-to-term-goal', C.INCONCLUSIVE, 'no term_goal_rviz seen (no manual goal sent in this run)')
    else:
        rep.check('rviz-to-term-goal', C.PASS if not rviz_bad else C.FAIL,
                  f'{rviz_ok}/{len(n.rviz)} manual goals followed by a term_goal with the same x,y within 2 s',
                  [f'no term_goal for rviz goal at t={t} ({x},{y})' for t, x, y in rviz_bad], ref='spec D5')

    # ---- relocation / explained goals
    reloc, unexpl = [], []
    for (t, s, x, y) in tg_sorted:
        srcs = [(tc, cx, cy) for (tc, cx, cy) in n.cg if abs(tc - t) <= 2.0]
        srcs += [(tr, rx, ry) for (tr, rx, ry) in n.rviz if -0.5 <= t - tr <= 3.0]
        if any(near(x, y, cx, cy, 1e-3) for (_, cx, cy) in srcs):
            continue
        cl = [(math.hypot(x - cx, y - cy), cx, cy) for (_, cx, cy) in srcs]
        cl = [c for c in cl if c[0] < a.reloc_near]
        if cl:
            reloc.append((round(t, 1), round(x, 3), round(y, 3), 'moved %.3f m from' % min(cl)[0], min(cl)[1:]))
        else:
            unexpl.append((round(t, 1), round(x, 2), round(y, 2)))
    if not n.tg:
        rep.check('no-relocation', C.INCONCLUSIVE, 'no term_goal to compare')
    else:
        rep.check('no-relocation', C.FAIL if reloc else C.PASS,
                  f'{len(n.tg) - len(reloc) - len(unexpl)} goals identical to their source, {len(reloc)} RELOCATED, '
                  f'{len(unexpl)} with no source seen',
                  [f'relocated: {r}' for r in reloc] +
                  [f'no source (return home, or published before the monitor started): {u}' for u in unexpl[:10]],
                  ref='spec N5/§11.15')

    # ---- status vs goal stamp
    known = {}
    for (t, s, _, _) in tg_sorted:
        known.setdefault(s, t)
    first_tg_t = tg_sorted[0][0] if tg_sorted else None
    unknown_stamp, stale, stale_late = [], 0, []
    tg_t = sorted((t, s) for (t, s, _, _) in tg_sorted)
    for (t, st, gs) in n.st:
        if gs == 0 or first_tg_t is None or t < first_tg_t:
            continue
        if gs not in known:
            unknown_stamp.append((round(t, 1), NAMES.get(st, st), gs))
            continue
        newer = [tt for (tt, ss) in tg_t if ss > gs and tt <= t]
        if newer:
            stale += 1
            if t - min(newer) > a.stale_tol:
                stale_late.append((round(t, 1), NAMES.get(st, st), round(t - min(newer), 2)))
    rep.data['stale_statuses'] = stale
    if first_tg_t is None:
        rep.check('status-goal-stamp', C.INCONCLUSIVE, 'no term_goal observed, stamps cannot be matched')
    else:
        rep.check('status-goal-stamp', C.FAIL if unknown_stamp else C.PASS,
                  f'{len(unknown_stamp)} statuses carry a goal_stamp that no observed term_goal had',
                  [f'status {s} at t={t} stamp={g}' for t, s, g in unknown_stamp[:10]], ref='spec §3')
        rep.check('stale-status', C.FAIL if stale_late else C.INFO,
                  f'{stale} statuses refer to an older goal than the newest published; '
                  f'{len(stale_late)} arrived more than {a.stale_tol}s after the newer goal',
                  [f'{s} at t={t}, {d}s after the newer goal' for t, s, d in stale_late[:10]],
                  ref='spec §3 (ASSUMPTION tolerance)')

    # ---- FAILED bursts
    bursts = []     # (t_start, t_end, count, stamp)
    cur = None
    for (t, st, gs) in n.st:
        if gs == 0:
            continue
        if cur and cur[3] != gs:
            bursts.append(cur)
            cur = None
        if st == 1:
            cur = [t, t, 1, gs] if cur is None else [cur[0], t, cur[2] + 1, gs]
        elif st in (0, 2, 3):
            if cur:
                bursts.append(cur)
            cur = None
        # SKIPPED (4): neutral
    if cur:
        bursts.append(cur)
    big = [b for b in bursts if b[2] >= a.thresh]
    rep.data['failed_bursts'] = len(bursts)
    rep.data['failed_bursts_ge_thresh'] = len(big)
    rep.check('failed-bursts', C.INFO,
              f'{len(bursts)} bursts of consecutive FAILED (SKIPPED ignored); {len(big)} reached the threshold '
              f'{a.thresh}; longest {max([b[2] for b in bursts], default=0)}',
              [f'burst t={b[0]:.1f}..{b[1]:.1f}s n={b[2]} stamp={b[3]}{" (manual)" if b[3] in manual_stamps else ""}'
               for b in big[:15]], ref='spec D8')
    front_big = [b for b in big if b[3] not in manual_stamps]
    if not front_big:
        rep.check('failed-burst-reaction', C.INCONCLUSIVE,
                  'no FAILED burst >= threshold on a frontier goal occurred, so the invalidation reaction '
                  'was not exercised', ref='spec §3')
    else:
        bad = []
        for b in front_big:
            # threshold reached at the a.thresh-th FAILED; we only have the burst end, use it as the lower bound
            nxt = [s for (t, s, _, _) in tg_sorted if b[1] < t <= b[1] + a.react_sec and s != b[3]]
            if not nxt:
                bad.append(b)
        rep.check('failed-burst-reaction', C.FAIL if bad else C.PASS,
                  f'{len(front_big) - len(bad)}/{len(front_big)} threshold bursts on frontier goals were followed '
                  f'by a new term_goal within {a.react_sec}s',
                  [f'no new goal after burst t={b[0]:.1f}..{b[1]:.1f}s n={b[2]}' for b in bad[:10]],
                  ref='spec §3 FAILED handling')

    # ---- selector_map_2d
    if not n.maps:
        rep.check('selector-map-values', C.INCONCLUSIVE, 'no selector_map_2d received')
        rep.check('selector-map-rate', C.INCONCLUSIVE, 'no selector_map_2d received')
    else:
        bad = sorted(n.map_values - ALLOWED_MAP_VALUES)
        rep.check('selector-map-values', C.FAIL if bad else C.PASS,
                  f'values seen {sorted(n.map_values)}; outside {{-1,0,50,99,100}}: {bad}',
                  ref='test notes item 5 (viz sanity)')
        ts = [t for (t, *_r) in n.maps]
        worst = 0
        for i, t0 in enumerate(ts):
            worst = max(worst, sum(1 for t in ts[i:] if t - t0 < 10.0))
        dur = ts[-1] - ts[0]
        mean = (len(ts) - 1) / dur if dur > 0 else 0.0
        lim = int(a.map_max_hz * 10)
        rep.check('selector-map-rate', C.FAIL if worst > lim else C.PASS,
                  f'{len(ts)} maps, mean {mean:.2f} Hz, max {worst} in any 10 s window (limit {lim} = '
                  f'{a.map_max_hz} Hz); size {n.maps[-1][1]}x{n.maps[-1][2]} frame {n.maps[-1][3]!r}',
                  ref='spec N5 / notes item 5 ("<= ~1 Hz")')

    # ---- point_selector_goal
    if not n.tg:
        rep.check('point-selector-goal', C.INCONCLUSIVE, 'no term_goal published')
    else:
        used, missing, multi = set(), [], []
        for (t, s, x, y) in tg_sorted:
            m = [i for i, p in enumerate(n.pts) if -1.0 <= p[0] - t <= 3.0 and near(p[1], p[2], x, y, 1e-3)]
            if not m:
                missing.append((round(t, 1), round(x, 2), round(y, 2)))
            elif len(m) > 1:
                multi.append((round(t, 1), len(m)))
            used.update(m)
        orphans = [(round(p[0], 1), round(p[1], 2), round(p[2], 2)) for i, p in enumerate(n.pts) if i not in used]
        bad = missing or multi or orphans
        rep.check('point-selector-goal', C.FAIL if bad else C.PASS,
                  f'{len(n.tg)} term_goals / {len(n.pts)} points: {len(missing)} without a point, '
                  f'{len(multi)} with several, {len(orphans)} orphan points',
                  [f'missing: {m}' for m in missing[:8]] + [f'several: {m}' for m in multi[:8]] +
                  [f'orphan: {o}' for o in orphans[:8]], ref='notes item 5')
    return rep


def main():
    ap = C.base_parser('Planner/goal_selector contract monitor', __doc__)
    ap.add_argument('--thresh', type=int, default=None, help='unreachable_consec_thresh')
    ap.add_argument('--react-sec', type=float, default=4.0)
    ap.add_argument('--stale-tol', type=float, default=2.0)
    ap.add_argument('--reloc-near', type=float, default=3.0)
    ap.add_argument('--map-max-hz', type=float, default=1.5)
    a, ros_args = ap.parse_known_args()
    if a.thresh is None:
        a.thresh = int(C.load_param('exploration.manager.unreachable_consec_thresh', 5,
                                    a.config or _sel_cfg()))
    rclpy.init(args=ros_args)
    n = ContractMonitor(a)
    flag = C.StopFlag(a.duration)
    C.spin_until(n, flag)
    code = evaluate(n, a).finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


def _sel_cfg():
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.path.join(here, '../../config/hw_goal_selector.yaml'), '/cfg/hw_goal_selector.yaml'):
        if os.path.exists(p):
            return p
    return ''


if __name__ == '__main__':
    main()
