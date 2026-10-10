#!/usr/bin/env python3
"""HARDWARE test: goal reached -> frontier VISITED -> next goal, end to end.

Cannot be exercised on a bag replay (the robot follows the recording and never arrives at the
planner's goals), so it needs the real robot driving. Spec §3 (REACHED: mark VISITED, clear
active, reset counter), notes B ("Planner reports goal reached -> frontier VISITED -> next goal"),
spec N6 ("REACHED/VISITED behaviour ... hardware test is needed").

SETUP: full stack running (mighty_hw.sh start), exploration.enabled: true, robot armed in an area
with at least two frontiers. See docs/testing/goal_selector_test_runbook.md (HW-1).

RUN (inside the hw container or any shell with ROS + the zenoh session of the robot):
    python3 hw_check_goal_cycle.py --ns RR08 [--cycles 2] [--timeout 600] [--out /tmp/goal_cycle.json]
  Start it BEFORE (or just after) the robot starts exploring; it follows the live topics and
  prints one line per event. Ctrl-C ends it early (verdicts then use what was seen).

CRITERIA (per cycle; a cycle = one FRONTIER term_goal that gets REACHED)
  reached-status     PASS: a planner_status REACHED whose goal_stamp == that term_goal's header.stamp
                     arrives, and a goal_reached (Empty) message with it; FAIL: the robot stands
                     within --arrive-tol (goal_radius 0.5 + 0.3 m) of the goal for > --reach-wait s
                     without REACHED (checked only if the goal had a FAILED-free plan).
  robot-at-goal      PASS: robot is within goal_radius + 0.3 m of the goal at the REACHED message.
  next-goal          PASS: a new term_goal (stamp > old) is published within --next-sec (15 s) of
                     REACHED and its x, y is more than merge_radius_m (0.75) from the reached goal
                     (a VISITED frontier is not re-selected, spec §3/notes B). It is either a frontier
                     (matches exploration/current_goal) or the return-home goal ("No frontiers left").
                     FAIL: re-selected within merge_radius, or the same stamp/position published
                     again. INCONCLUSIVE: nothing published (no frontiers left and no home goal?
                     check the selector pane / exploration/frontiers marker count shown as evidence).
  OVERALL            PASS needs at least one full cycle; INCONCLUSIVE if no frontier goal was reached.

OUTPUT: live event log, then the verdict list; JSON with --out. Exit 1 on FAIL.
"""
import math
import sys

import rclpy

import gs_test_common as C
import gs_live as L


def main():
    ap = C.base_parser('HW: goal reached -> VISITED -> next goal', __doc__)
    ap.add_argument('--cycles', type=int, default=1, help='stop after this many completed cycles')
    ap.add_argument('--timeout', type=float, default=600.0, help='give up after this many seconds')
    ap.add_argument('--arrive-tol', type=float, default=0.8)
    ap.add_argument('--reach-wait', type=float, default=10.0)
    ap.add_argument('--next-sec', type=float, default=15.0)
    ap.add_argument('--merge-radius', type=float, default=None)
    a, ros_args = ap.parse_known_args()
    a.duration = a.duration or a.timeout
    sel = '/cfg/hw_goal_selector.yaml'
    mr = a.merge_radius if a.merge_radius is not None else float(
        C.load_param('exploration.manager.merge_radius_m', 0.75, sel))
    rclpy.init(args=ros_args)
    n = L.Live(a.ns, 'hw_check_goal_cycle')
    flag = C.StopFlag(a.duration)
    rep = C.Report('HW: goal reached -> VISITED -> next goal', dict(ns=a.ns, merge_radius_m=mr))
    done = []        # per cycle dict
    seen_reached = set()

    def tick():
        # a cycle starts at a frontier term_goal that got a REACHED status
        for tg in n.tg:
            t, stamp, gx, gy = tg
            if stamp in seen_reached or not n.is_frontier_goal(tg):
                continue
            tr = n.first_status(stamp, 3)
            if tr is None:
                continue
            seen_reached.add(stamp)
            cyc = dict(stamp=stamp, goal=(gx, gy), t_commit=t, t_reached=tr)
            p = n.pose_at(tr)
            cyc['dist_at_reached'] = math.hypot(p[1] - gx, p[2] - gy) if p else None
            cyc['goal_reached_msg'] = any(abs(r - tr) < 2.0 for r in n.reached)
            cyc['final'] = False
            done.append(cyc)
            n.say(f'goal {stamp} REACHED after {tr - t:.1f}s; robot {cyc["dist_at_reached"]} m from it')
        # complete a cycle when the next goal appeared or its window elapsed
        for c in done:
            if c['final']:
                continue
            nxt = [g for g in n.tg if g[1] > c['stamp'] and g[0] >= c['t_reached'] - 0.5]
            if nxt:
                g = nxt[0]
                c['next'] = dict(t=g[0] - c['t_reached'], xy=(g[2], g[3]), frontier=n.is_frontier_goal(g),
                                 dist=math.hypot(g[2] - c['goal'][0], g[3] - c['goal'][1]))
                c['final'] = True
            elif n.now() - c['t_reached'] > a.next_sec:
                c['next'] = None
                c['final'] = True
        return sum(1 for c in done if c['final']) >= a.cycles

    L.run(n, flag, tick)
    tick()
    # robot standing at the goal without REACHED (only for the last unfinished goal)
    stalled = None
    if n.tg:
        t, stamp, gx, gy = n.tg[-1]
        d = n.dist_to(gx, gy)
        if stamp not in seen_reached and d is not None and d <= a.arrive_tol and n.now() - t > a.reach_wait \
                and n.first_status(stamp, 3) is None and n.is_frontier_goal(n.tg[-1]):
            stalled = (stamp, round(d, 2), round(n.now() - t, 1))

    if not done:
        rep.check('reached-status', C.FAIL if stalled else C.INCONCLUSIVE,
                  'robot stood at the goal but no REACHED status arrived: ' + str(stalled) if stalled else
                  'no frontier goal was REACHED during the run (robot did not arrive, or exploration is off)',
                  ref='spec §3')
    for i, c in enumerate(done, 1):
        tag = f'cycle{i}'
        rep.check(f'{tag}-reached-status', C.PASS if c['goal_reached_msg'] else C.FAIL,
                  f'REACHED status for goal stamp {c["stamp"]} {c["t_reached"] - c["t_commit"]:.1f}s after commit; '
                  f'goal_reached message {"seen" if c["goal_reached_msg"] else "MISSING"}', ref='spec §3, §11.11')
        d = c['dist_at_reached']
        rep.check(f'{tag}-robot-at-goal', C.PASS if (d is not None and d <= a.arrive_tol) else C.FAIL,
                  f'robot {d if d is None else round(d, 2)} m from the goal at REACHED (tolerance {a.arrive_tol})')
        nx = c.get('next')
        if nx is None:
            rep.check(f'{tag}-next-goal', C.INCONCLUSIVE,
                      f'no new term_goal within {a.next_sec}s of REACHED; exploration/frontiers marker count '
                      f'{n.n_frontier_markers}. Check the selector pane for "No frontiers left" / errors.',
                      ref='notes B')
        else:
            bad = nx['dist'] <= mr
            kind = 'frontier' if nx['frontier'] else 'not a frontier goal: return home or manual'
            rep.check(f'{tag}-next-goal', C.FAIL if bad else C.PASS,
                      f'next term_goal {nx["t"]:.1f}s after REACHED at ({nx["xy"][0]:.2f},{nx["xy"][1]:.2f}), '
                      f'{nx["dist"]:.2f} m from the reached goal ({kind}); merge radius {mr}',
                      ['reached goal re-selected: it was not marked VISITED'] if bad else [], ref='spec §3, notes B')
    rep.data = dict(cycles=done)
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
