#!/usr/bin/env python3
"""HARDWARE test: do small unknown holes in the occ_2d_topic grid resolve as the robot approaches?

Background: the mapper leaves holes of <= ~4 cells as unknown (-1) inside known space on purpose
(hw_mighty_ground_robot.yaml trim_min_unknown_run_cells); the prefix drives through them and the
selector keeps them as small unknown bands (spec N1 Q2, "Answer 2: do not ignore small patches for now";
note: "may be a desired next step"). The open question (notes C/E) is whether they actually fill in once
the robot gets close and sees them. Needs the robot driving.

SETUP: stack running, mapper publishing <ns>/occ_2d_topic, robot drives through an area with such holes
(exploration or manual goals). Start the monitor first.

RUN:
    python3 hw_check_unknown_holes.py --ns RR08 [--max-hole-cells 8] [--approach 1.5] [--out f.json]
  Ctrl-C when the drive is finished; the report is printed on exit.

METHOD
  Every new grid: unknown cells are labelled (8-connected). A component with <= --max-hole-cells cells
  whose whole 1-cell ring is KNOWN (so it is a hole, not the edge of observed space) is a HOLE. Holes are
  tracked in world coordinates (matched when a cell is shared). For each hole the monitor records the
  closest approach of the robot (state pose) and whether its cells are known in the latest grid.

CRITERIA (ASSUMPTION: the user's notes give no number, this is the interpretation "a hole the robot has
  come close to should no longer be unknown")
  holes-approached   INCONCLUSIVE if no hole was seen or none was approached within --approach m.
  holes-resolved     PASS if every hole approached within --approach m is fully known (no unknown cell) in
                     the last grid that still covers it, FAIL if any approached hole is still unknown;
                     the evidence lists each hole (world x, y, size, closest approach, status).
                     Holes never approached closer than --approach are reported as INFO only.
OUTPUT: table of holes, verdicts; JSON with --out (data.holes). Exit 1 on FAIL.
"""
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid

import gs_test_common as C
import gs_live as L


def label(unk):
    try:
        from scipy import ndimage
        return ndimage.label(unk, structure=np.ones((3, 3)))
    except ImportError:
        lab = np.zeros(unk.shape, dtype=np.int32)
        cur = 0
        H, W = unk.shape
        for r in range(H):
            for c in range(W):
                if unk[r, c] and lab[r, c] == 0:
                    cur += 1
                    stack = [(r, c)]
                    lab[r, c] = cur
                    while stack:
                        y, x = stack.pop()
                        for dy in (-1, 0, 1):
                            for dx in (-1, 0, 1):
                                yy, xx = y + dy, x + dx
                                if 0 <= yy < H and 0 <= xx < W and unk[yy, xx] and lab[yy, xx] == 0:
                                    lab[yy, xx] = cur
                                    stack.append((yy, xx))
        return lab, cur


class Holes:
    def __init__(self, a):
        self.a = a
        self.holes = []     # dict: cells(set of (ix,iy) world-index at res), x,y,size,min_d,first_t,last_known

    def update(self, g, robot):
        unk = g.data < 0
        lab, nlab = label(unk)
        res = g.res
        sizes = np.bincount(lab.ravel(), minlength=nlab + 1)
        small = [k for k in range(1, nlab + 1) if sizes[k] <= self.a.max_hole_cells]
        flat = lab.ravel()
        idx = np.nonzero(np.isin(flat, small))[0] if small else np.array([], dtype=int)
        labs = flat[idx]
        order = np.argsort(labs, kind='stable')
        idx, labs = idx[order], labs[order]
        groups = np.split(idx, np.nonzero(np.diff(labs))[0] + 1) if len(idx) else []
        for grp in groups:
            ys, xs = np.divmod(grp, g.w)
            if ys.min() == 0 or xs.min() == 0 or ys.max() == g.h - 1 or xs.max() == g.w - 1:
                continue
            # ring must be known: any unknown neighbour outside the component would have joined it (8-conn), so
            # the ring is known by construction; the border test above excludes edge-of-grid components.
            wx = g.ox + (xs + 0.5) * res
            wy = g.oy + (ys + 0.5) * res
            cells = set(zip(np.round(wx / res).astype(int).tolist(), np.round(wy / res).astype(int).tolist()))
            hit = next((h for h in self.holes if h['cells'] & cells), None)
            if hit is None:
                hit = dict(cells=set(cells), x=float(wx.mean()), y=float(wy.mean()), size=len(cells),
                           min_d=1e9, first_t=g.recv_t, resolved_t=None, status='unknown')
                self.holes.append(hit)
            else:
                hit['cells'] |= cells
        # evaluate every tracked hole against this grid
        for h in self.holes:
            if robot:
                h['min_d'] = min(h['min_d'], math.hypot(robot[1] - h['x'], robot[2] - h['y']))
            covered, still_unknown = 0, 0
            for (ix, iy) in h['cells']:
                x, y = ix * res, iy * res
                v = g.value(x, y)
                if v is None:
                    continue
                covered += 1
                if v < 0:
                    still_unknown += 1
            if covered == 0:
                h['status'] = 'out-of-grid' if h['status'] != 'resolved' else 'resolved'
            elif still_unknown == 0:
                h['status'] = 'resolved'
                h['resolved_t'] = h['resolved_t'] or g.recv_t
            else:
                h['status'] = 'unknown'
                h['resolved_t'] = None


def main():
    ap = C.base_parser('HW: unknown holes resolve on approach', __doc__)
    ap.add_argument('--max-hole-cells', type=int, default=8)
    ap.add_argument('--approach', type=float, default=1.5)
    ap.add_argument('--grid-topic', default='occ_2d_topic')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    n = L.Live(a.ns, 'hw_check_unknown_holes')
    H = Holes(a)
    n.create_subscription(OccupancyGrid, f'/{a.ns.strip("/")}/{a.grid_topic}',
                          lambda m: H.update(C.Grid(m, n.now()), n.pose), C.qos_best_effort(2))
    flag = C.StopFlag(a.duration)
    L.run(n, flag, lambda: False)
    rep = C.Report('HW: unknown holes vs robot approach', dict(ns=a.ns, max_hole_cells=a.max_hole_cells,
                                                              approach_m=a.approach, holes_tracked=len(H.holes)))
    near = [h for h in H.holes if h['min_d'] <= a.approach]
    rows = [dict(x=round(h['x'], 2), y=round(h['y'], 2), cells=h['size'], closest_m=round(h['min_d'], 2)
                 if h['min_d'] < 1e8 else None, status=h['status']) for h in H.holes]
    rep.data = dict(holes=rows)
    if not H.holes:
        rep.check('holes-approached', C.INCONCLUSIVE, 'no unknown hole <= %d cells seen in the grid' % a.max_hole_cells)
    elif not near:
        rep.check('holes-approached', C.INCONCLUSIVE,
                  f'{len(H.holes)} hole(s) seen but none approached within {a.approach} m '
                  f'(closest {min(h["min_d"] for h in H.holes):.2f} m)')
    else:
        bad = [h for h in near if h['status'] == 'unknown']
        ev = [f'hole at ({h["x"]:.2f},{h["y"]:.2f}) {h["size"]} cells, closest {h["min_d"]:.2f} m: {h["status"]}'
              for h in sorted(near, key=lambda h: h['min_d'])[:30]]
        rep.check('holes-resolved', C.FAIL if bad else C.PASS,
                  f'{len(near) - len(bad)}/{len(near)} approached holes are now known; {len(bad)} still unknown',
                  ev, ref='notes C/E, spec N1 Q2 (ASSUMPTION: approached holes should resolve)')
        rep.check('holes-not-approached', C.INFO,
                  f'{len(H.holes) - len(near)} hole(s) were never within {a.approach} m '
                  f'({sum(1 for h in H.holes if h["min_d"] > a.approach and h["status"] == "resolved")} resolved anyway)')
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
