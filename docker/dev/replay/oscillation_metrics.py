#!/usr/bin/env python3
"""Score a rosbag2 (sqlite3) recording for the HGP exit-edge oscillation.

Reads the bag's .db3 directly (no rosbag2_py) and deserializes with the ROS message
classes, so it runs anywhere with rclpy + nav_msgs + visualization_msgs (e.g. inside
the rviz or mighty-hw image). Metrics, per the 2026-09 fix plan:

  * heading switches: consecutive <robot>/hgp_path_marker messages whose first-segment
    bearing differs by > --switch-deg (default 30)
  * exit edges: which edge(s) of the local grid the <robot>/original_hgp_path_marker
    path leaves through (S/N/W/E), and how often that edge changes
  * global path arc length vs the straight-line distance to its endpoint

Baseline (RR04, 2026-09-18, bag hgp_edge_oscillation_20260918_220254): 14 switches in
50 s, 3 distinct exit edges, arc length up to ~29 m for a 12 m goal. Target after the
fix: <= 1 switch, 1 edge, arc length within ~20% of the straight line.

    python3 oscillation_metrics.py <bag_dir> --robot RR04 [--grid planning] [--switch-deg 30]
"""
import argparse
import glob
import math
import sqlite3

from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray


def load(bag):
    con = sqlite3.connect(glob.glob(bag + "/*.db3")[0])
    topics = {n: i for i, n in con.execute("select id,name from topics")}
    t0 = con.execute("select min(timestamp) from messages").fetchone()[0]

    def rows(name):
        if name not in topics:
            return []
        return [((ts - t0) / 1e9, bytes(d)) for ts, d in con.execute(
            "select timestamp,data from messages where topic_id=? order by timestamp", (topics[name],))]
    return topics, rows


def longest_polyline(data):
    m = deserialize_message(data, MarkerArray)
    best = max((mk for mk in m.markers if len(mk.points) >= 2), key=lambda k: len(k.points), default=None)
    return [(p.x, p.y) for p in best.points] if best else None


def bearing(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def ang_diff(a, b):
    return abs((b - a + 180.0) % 360.0 - 180.0)


def arc_len(pts):
    return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))


def exit_edge(grid, pts):
    """Edge (S/N/W/E) at which the polyline first leaves the grid, or None."""
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    w, h, r = grid.info.width * grid.info.resolution, grid.info.height * grid.info.resolution, grid.info.resolution
    prev = None
    for p in pts:
        inside = ox <= p[0] < ox + w and oy <= p[1] < oy + h
        if not inside and prev is not None:
            d = {"W": prev[0] - ox, "E": ox + w - prev[0], "S": prev[1] - oy, "N": oy + h - prev[1]}
            return min(d, key=d.get)
        if not inside:
            return None
        prev = p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--robot", default="RR04")
    ap.add_argument("--grid", choices=["planning", "raw"], default="planning",
                    help="planning_occ_2d_topic (default, falls back to raw) or occ_2d_topic")
    ap.add_argument("--switch-deg", type=float, default=30.0)
    ap.add_argument("--bearing", choices=["endpoint", "first"], default="endpoint",
                    help="heading = start->endpoint of the local path (default) or its first grid "
                         "segment; the first segment of an 8-connected A* path alternates between "
                         "orthogonal and diagonal steps (90 vs 59 deg) and is not the oscillation")
    a = ap.parse_args()
    topics, rows = load(a.bag)
    ns = "/" + a.robot

    gname = ns + ("/planning_occ_2d_topic" if a.grid == "planning" else "/occ_2d_topic")
    if gname not in topics:
        gname = ns + "/occ_2d_topic"
    grids = [(t, deserialize_message(d, OccupancyGrid)) for t, d in rows(gname)]

    def grid_at(t):
        g = grids[0][1] if grids else None
        for gt, gg in grids:
            if gt <= t:
                g = gg
        return g

    # --- local path: heading switches ---
    loc = [(t, longest_polyline(d)) for t, d in rows(ns + "/hgp_path_marker")]
    loc = [(t, p) for t, p in loc if p]
    switches = []
    def head(p):
        return bearing(p[0], p[-1] if a.bearing == "endpoint" else p[1])
    for (ta, pa), (tb, pb) in zip(loc, loc[1:]):
        if ang_diff(head(pa), head(pb)) > a.switch_deg:
            switches.append(tb)

    # --- global path: exit edges + arc length ---
    glo = [(t, longest_polyline(d)) for t, d in rows(ns + "/original_hgp_path_marker")]
    glo = [(t, p) for t, p in glo if p]
    edges, edge_changes, ratios = [], 0, []
    last = None
    for t, p in glo:
        g = grid_at(t)
        e = exit_edge(g, p) if g else None
        edges.append(e)
        if e is not None and last is not None and e != last:
            edge_changes += 1
        if e is not None:
            last = e
        chord = math.dist(p[0], p[-1])
        if chord > 1e-3:
            ratios.append(arc_len(p) / chord)

    dur = max([t for t, _ in loc] + [t for t, _ in glo] + [0.0])
    used = sorted({e for e in edges if e})
    print(f"bag: {a.bag}")
    print(f"robot: {a.robot}  grid topic: {gname} ({len(grids)} msgs)  duration: {dur:.1f}s")
    print(f"local path msgs: {len(loc)}   heading switches >{a.switch_deg:.0f} deg: {len(switches)}"
          + (f"  at t=" + ", ".join(f"{t:.1f}" for t in switches[:12]) if switches else ""))
    print(f"global path msgs: {len(glo)}   exit edges used: {used or ['none (goal inside grid)']}"
          f"   edge changes: {edge_changes}")
    if ratios:
        print(f"global arc/chord ratio: median {sorted(ratios)[len(ratios)//2]:.2f}  max {max(ratios):.2f}")
    ok = len(switches) <= 1 and len(used) <= 1 and (not ratios or max(ratios) <= 1.2)
    print("RESULT:", "PASS" if ok else "FAIL", "(target: <=1 switch, <=1 edge, arc/chord <= 1.2)")


if __name__ == "__main__":
    main()
