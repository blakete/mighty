#!/usr/bin/env python3
"""Shared helpers for the goal-selector / planner bag and hardware monitors.

Not run directly. Imported by monitor_*.py, hw_check_*.py and summarize_run.py (all in this
directory, so `python3 /tests/monitor_x.py` finds it). Needs ROS 2 Humble (rclpy,
dynus_interfaces, goal_selector_msgs) only for the monitor scripts; `Report`, `Grid` and the
parsing helpers are pure Python (numpy) and are also used by summarize_run.py on the host.

Time: every "now" in the monitors is `node.get_clock().now()`, so they work on a live robot
(wall time), during pass2_laptop.sh replays (the restamp relay moves the bag onto the wall
clock) and, if you pass `--ros-args -p use_sim_time:=true`, under `ros2 bag play --clock`.

Verdicts used everywhere: PASS, FAIL, INCONCLUSIVE (the situation the test needs never
occurred, so nothing can be said) and INFO (a measurement that is reported, not judged).
Exit code of every monitor: 0 = no FAIL, 1 = at least one FAIL, 2 = nothing could be run.
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import numpy as np

# ----------------------------------------------------------------------------- verdicts

PASS, FAIL, INCONCLUSIVE, INFO = 'PASS', 'FAIL', 'INCONCLUSIVE', 'INFO'


class Report:
    """Collects named checks (verdict + evidence) and prints / saves them."""

    def __init__(self, title, meta=None):
        self.title = title
        self.meta = dict(meta or {})
        self.checks = []          # dicts: name, verdict, summary, evidence(list[str]), data(dict)
        self.data = {}            # free-form numbers for summarize_run.py
        self.t_wall0 = time.time()

    def check(self, name, verdict, summary, evidence=None, data=None, ref=''):
        self.checks.append({'name': name, 'verdict': verdict, 'summary': summary,
                            'evidence': list(evidence or [])[:60], 'data': data or {}, 'ref': ref})

    def overall(self):
        vs = [c['verdict'] for c in self.checks]
        if FAIL in vs:
            return FAIL
        if PASS in vs:
            return PASS
        return INCONCLUSIVE

    def text(self):
        out = ['=' * 78, self.title, '=' * 78]
        for k, v in self.meta.items():
            out.append(f'  {k}: {v}')
        for c in self.checks:
            ref = f'   [{c["ref"]}]' if c['ref'] else ''
            out.append(f'\n[{c["verdict"]}] {c["name"]}{ref}')
            out.append(f'    {c["summary"]}')
            for e in c['evidence']:
                out.append(f'      - {e}')
        out.append(f'\nOVERALL: {self.overall()}')
        return '\n'.join(out)

    def save(self, path):
        if not path:
            return
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        with open(path, 'w') as f:
            json.dump({'title': self.title, 'meta': self.meta, 'overall': self.overall(),
                       'checks': self.checks, 'data': self.data}, f, indent=1, default=str)

    def finish(self, out_json=None):
        print(self.text(), flush=True)
        self.save(out_json)
        return 1 if self.overall() == FAIL else 0


# ----------------------------------------------------------------------------- CLI helpers

def base_parser(desc, doc):
    ap = argparse.ArgumentParser(description=desc, epilog=doc,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ns', default=os.environ.get('ROBOT_NAME', 'RR08'),
                    help='robot namespace, e.g. RR08 (default: $ROBOT_NAME or RR08)')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='stop after this many seconds (0 = run until Ctrl-C / SIGINT / SIGTERM)')
    ap.add_argument('--out', default='', help='write the JSON report here (default: none)')
    ap.add_argument('--config', default='',
                    help='YAML with planner/selector parameters (default: config/'
                         'hw_mighty_ground_robot.yaml found next to this repo)')
    return ap


def load_param(name, default, config=''):
    """Look `name` up (any nesting depth) in the config YAML; fall back to `default`."""
    try:
        import yaml
    except ImportError:
        return default
    cands = [config] if config else []
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ('../../config/hw_mighty_ground_robot.yaml', '/cfg/hw_mighty_ground_robot.yaml'):
        cands.append(os.path.normpath(os.path.join(here, rel)))
    for p in cands:
        try:
            with open(p) as f:
                found = _find(yaml.safe_load(f), name)
            if found is not None:
                return found
        except Exception:
            continue
    return default


def _find(node, key):
    if isinstance(node, dict):
        if key in node and not isinstance(node[key], dict):
            return node[key]
        for v in node.values():
            r = _find(v, key)
            if r is not None:
                return r
    return None


class StopFlag:
    """Set by SIGINT/SIGTERM or when the duration is over."""

    def __init__(self, duration=0.0):
        self.stop = False
        self.duration = duration
        self.t0 = time.time()
        signal.signal(signal.SIGINT, self._h)
        signal.signal(signal.SIGTERM, self._h)

    def _h(self, *_):
        self.stop = True

    def __bool__(self):
        if self.duration > 0 and time.time() - self.t0 >= self.duration:
            self.stop = True
        return self.stop


def spin_until(node, flag, tick=None, period=0.2):
    """rclpy spin loop that ends on `flag`; `tick()` is called every `period` s."""
    import rclpy
    last = 0.0
    while rclpy.ok() and not flag:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time()
        if tick and now - last >= period:
            last = now
            tick()


# ----------------------------------------------------------------------------- QoS

def qos_reliable(depth=100, transient_local=False):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                      depth=depth,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if transient_local
                      else DurabilityPolicy.VOLATILE)


def qos_best_effort(depth=100):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                      depth=depth, durability=DurabilityPolicy.VOLATILE)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def now_s(node):
    return node.get_clock().now().nanoseconds * 1e-9


# ----------------------------------------------------------------------------- grids

UNKNOWN, FREE, OCC = -1, 0, 100
OCC_THRESH = 50   # ASSUMPTION: >= 50 counts as occupied (the mapper publishes 0 / 100 / -1)


class Grid:
    """Immutable snapshot of a nav_msgs/OccupancyGrid (row-major, origin at the lower-left)."""

    def __init__(self, msg, recv_t):
        self.w = msg.info.width
        self.h = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.frame = msg.header.frame_id
        self.stamp_ns = stamp_ns(msg.header.stamp)
        self.recv_t = recv_t
        self.data = np.asarray(msg.data, dtype=np.int8).reshape(self.h, self.w)
        self._band = None

    def cell(self, x, y):
        """(row, col) of a world point, or None when outside the grid."""
        c = int(math.floor((x - self.ox) / self.res))
        r = int(math.floor((y - self.oy) / self.res))
        if 0 <= r < self.h and 0 <= c < self.w:
            return r, c
        return None

    def value(self, x, y):
        rc = self.cell(x, y)
        return None if rc is None else int(self.data[rc])

    def classify(self, x, y):
        """'out' | 'unknown' | 'occ' | 'free' for a world point."""
        v = self.value(x, y)
        if v is None:
            return 'out'
        if v < 0:
            return 'unknown'
        if v >= OCC_THRESH:
            return 'occ'
        return 'free'

    def occ_band(self, radius_m):
        """Boolean mask of cells within radius_m of an occupied cell (cached for one radius)."""
        if self._band is not None and self._band[0] == radius_m:
            return self._band[1]
        occ = self.data >= OCC_THRESH
        k = int(math.ceil(radius_m / self.res))
        band = occ.copy()
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if (dx * dx + dy * dy) * self.res * self.res > radius_m * radius_m:
                    continue
                if dx == 0 and dy == 0:
                    continue
                sh = np.zeros_like(occ)
                ys, yd = (slice(dy, None), slice(0, self.h - dy)) if dy >= 0 else \
                         (slice(0, self.h + dy), slice(-dy, None))
                xs, xd = (slice(dx, None), slice(0, self.w - dx)) if dx >= 0 else \
                         (slice(0, self.w + dx), slice(-dx, None))
                sh[ys, xs] = occ[yd, xd]
                band |= sh
        self._band = (radius_m, band)
        return band


def densify(points, step):
    """Polyline [(x,y),...] -> samples every <= step metres including the end points.
    Returns list of (x, y, s) with s = arc length from the first point."""
    out = []
    s0 = 0.0
    for i, (x, y) in enumerate(points):
        if i == 0:
            out.append((x, y, 0.0))
            continue
        px, py = points[i - 1]
        d = math.hypot(x - px, y - py)
        n = max(1, int(math.ceil(d / step)))
        for k in range(1, n + 1):
            t = k / n
            out.append((px + (x - px) * t, py + (y - py) * t, s0 + d * t))
        s0 += d
    return out


def classify_polyline(grid, points, step, band_radius=0.0):
    """Sample `points` on `grid` every `step` m and return a dict:
       n (samples), unknown, occ, out, band (samples in the occupied band when band_radius>0),
       first_unknown / first_occ (arc length or None), max_unknown_run_cells (longest run of
       consecutive *distinct cells* classified unknown) and the first offending (x, y)."""
    samples = densify(points, step)
    res = dict(n=len(samples), unknown=0, occ=0, out=0, band=0, first_unknown=None,
               first_occ=None, first_unknown_xy=None, first_occ_xy=None,
               max_unknown_run_cells=0, first_run_s=None, first_run_xy=None)
    band = grid.occ_band(band_radius) if band_radius > 0 else None
    run, last_cell, run_cells_max = 0, None, 0
    for (x, y, s) in samples:
        rc = grid.cell(x, y)
        if rc is None:
            res['out'] += 1
            run, last_cell = 0, None
            continue
        v = int(grid.data[rc])
        if v < 0:
            res['unknown'] += 1
            if res['first_unknown'] is None:
                res['first_unknown'], res['first_unknown_xy'] = s, (round(x, 2), round(y, 2))
            if rc != last_cell:
                run += 1
            last_cell = rc
            if run > run_cells_max:
                run_cells_max = run
        else:
            run, last_cell = 0, None
            if v >= OCC_THRESH:
                res['occ'] += 1
                if res['first_occ'] is None:
                    res['first_occ'], res['first_occ_xy'] = s, (round(x, 2), round(y, 2))
            if band is not None and band[rc]:
                res['band'] += 1
    res['max_unknown_run_cells'] = run_cells_max
    return res


def poly_length(points):
    return sum(math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
               for i in range(1, len(points)))


def ansi_strip(s):
    import re
    return re.sub(r'\x1b\[[0-9;?]*[A-Za-z]', '', s)
