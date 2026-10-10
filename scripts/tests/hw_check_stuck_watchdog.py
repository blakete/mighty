#!/usr/bin/env python3
"""HARDWARE test: the stuck watchdog in a real situation (frontier goal or manual goal).

Spec N4 / §11.17 and notes B: "Robot has moved, then moves less than stuck_move_thresh_m for
stuck_timeout_sec -> new goal. A robot that never moved does not trip it." For a MANUAL goal the
watchdog is armed at commit (decided 2026-10-10) and releases the goal after the timeout;
exploration resumes if enabled (otherwise the selector stays idle and publishes nothing).
Config (hw_goal_selector.yaml): exploration.manager.stuck_timeout_sec 5.0, stuck_move_thresh_m 0.10.

SETUP (see runbook HW-3): stack running, robot armed.
  --kind frontier : exploration on. Let the robot drive towards a frontier; once it has MOVED,
                    block it (hold it / push it against a wall / lift the wheels / put a box in
                    front so it cannot progress) for > 10 s.
  --kind manual   : click a 2D Goal Pose in RViz at a free place a few metres away, let the robot
                    start driving, then block it the same way. (A robot that never moves is the
                    separate START-timeout test: hw_check_manual_goal.py.)

RUN:
    python3 hw_check_stuck_watchdog.py --ns RR08 --kind frontier [--timeout 300] [--out f.json]
    python3 hw_check_stuck_watchdog.py --ns RR08 --kind manual [--exploration off] [--selector-log pane.log]
  --selector-log FILE : optional text file with the selector pane output (tmux pipe-pane); used to
  confirm the log line ("frontier N stuck ..." / "Manual goal ... stuck ...") and as the only
  evidence when exploration is off after a manual release.

CRITERIA
  The monitor follows the active goal of the chosen kind. When a NEWER term_goal replaces it, the stall
  length is computed back from that moment: how long the robot had stayed within stuck_move_thresh_m of
  where it was at the replacement. A replacement with stall < 1 s is another reason (preemption, pursuit
  timeout, planner FAILED x N, REACHED) and is ignored (tracking continues with the new goal).
  stall-observed     INCONCLUSIVE unless a stall >= 1 s of an active goal was seen (and, for frontier
                     goals, the robot had moved more than the threshold before it, spec N4 "armed
                     after first motion"; a manual goal is armed at commit).
  release-timing     PASS: the replacement happens when the stall is between stuck_timeout_sec-0.5 s and
                     stuck_timeout_sec + --react-sec (default 4 s = one 1 Hz select tick + margin).
                     FAIL: stall shorter than stuck_timeout_sec-0.5 s but >= 1 s (premature), or no
                     release although the robot stood still for > stuck_timeout_sec + react-sec at the
                     end of the run. Frontier: replacement = any newer term_goal. Manual with
                     exploration on: the newer term_goal must be a FRONTIER goal. Manual with
                     --exploration off: nothing is published after the release; the verdict then comes
                     only from --selector-log (INCONCLUSIVE without it).
  log-line           (only with --selector-log) PASS if a "... stuck (no motion ...)" line appears.

OUTPUT: live log, verdicts, JSON with --out. Exit 1 on FAIL.
"""
import math
import re
import sys

import rclpy

import gs_test_common as C
import gs_live as L


def main():
    ap = C.base_parser('HW: stuck watchdog', __doc__)
    ap.add_argument('--kind', choices=('frontier', 'manual'), required=True)
    ap.add_argument('--exploration', choices=('on', 'off'), default='on',
                    help='is exploration enabled? (decides what a manual release looks like)')
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--react-sec', type=float, default=4.0)
    ap.add_argument('--stuck-timeout', type=float, default=None)
    ap.add_argument('--stuck-thresh', type=float, default=None)
    ap.add_argument('--selector-log', default='')
    a, ros_args = ap.parse_known_args()
    a.duration = a.duration or a.timeout
    sel = '/cfg/hw_goal_selector.yaml'
    T = a.stuck_timeout if a.stuck_timeout is not None else float(
        C.load_param('exploration.manager.stuck_timeout_sec', 5.0, sel))
    thr = a.stuck_thresh if a.stuck_thresh is not None else float(
        C.load_param('exploration.manager.stuck_move_thresh_m', 0.10, sel))
    rclpy.init(args=ros_args)
    n = L.Live(a.ns, 'hw_check_stuck')
    flag = C.StopFlag(a.duration)
    L.wait_for_robot(n, flag)
    S = dict(goal=None, cands=[])

    def stall_before(t_end):
        """(seconds the robot stayed within thr of its pose at t_end, ending at t_end; moved-before flag)."""
        p_end = n.pose_at(t_end)
        if p_end is None or not n.pose_hist:
            return 0.0, False
        last_far = None
        for p in n.pose_hist:
            if p[0] > t_end:
                break
            if math.hypot(p[1] - p_end[1], p[2] - p_end[2]) > thr:
                last_far = p[0]
        return t_end - (last_far if last_far is not None else n.pose_hist[0][0]), last_far is not None

    def kind_ok(g):
        return n.is_manual_goal(g) if a.kind == 'manual' else not n.is_manual_goal(g)

    def tick():
        if n.pose is None:
            return False
        if S['goal'] is None:
            for g in reversed(n.tg):
                if kind_ok(g) and n.first_status(g[1], 3) is None:
                    S['goal'] = g
                    n.say(f'tracking {a.kind} goal stamp {g[1]} ({g[2]:.2f},{g[3]:.2f})')
                break
            return False
        g = S['goal']
        newer = [x for x in n.tg if x[1] > g[1]]
        if not newer:
            return False
        nx = newer[0]
        stall, _ = stall_before(nx[0])
        t_stall_start = nx[0] - stall
        moved_before = t_stall_start > g[0] and n.max_displacement(g[0], t_stall_start) > thr
        armed = a.kind == 'manual' or moved_before
        c = dict(goal=g, next=nx, stall=stall, armed=armed,
                 next_is_frontier=n.is_frontier_goal(nx), reached=n.first_status(g[1], 3) is not None)
        if stall >= 1.0 and armed and not c['reached']:
            S['cands'].append(c)
            n.say(f'goal {g[1]} replaced after the robot had been still for {stall:.1f}s')
            return True
        n.say(f'goal {g[1]} replaced while robot moving/not stalled (stall {stall:.1f}s): other reason, ignored')
        S['goal'] = nx if kind_ok(nx) else None
        return False

    L.run(n, flag, tick)
    tick()
    rep = C.Report('HW: stuck watchdog', dict(ns=a.ns, kind=a.kind, exploration=a.exploration,
                                              stuck_timeout_sec=T, stuck_move_thresh_m=thr))
    log_lines = []
    if a.selector_log:
        try:
            for ln in open(a.selector_log, errors='replace'):
                ln = C.ansi_strip(ln)
                if re.search(r'stuck \(no motion', ln):
                    log_lines.append(ln.strip()[:200])
        except OSError as e:
            log_lines = [f'cannot read log: {e}']
    lo, hi = T - 0.5, T + a.react_sec
    if S['cands']:
        c = S['cands'][0]
        rep.check('stall-observed', C.PASS, f'goal {c["goal"][1]} followed by a stall of {c["stall"]:.1f}s '
                  f'before being replaced', ref='spec N4')
        ok_kind = a.kind == 'frontier' or c['next_is_frontier'] or a.exploration == 'off'
        in_window = lo <= c['stall'] <= hi
        ev = []
        if not ok_kind:
            ev.append('the replacing goal is not a frontier goal: exploration did not resume after the manual release')
        rep.check('release-timing', C.PASS if (in_window and ok_kind) else C.FAIL,
                  f'goal replaced when the robot had been still {c["stall"]:.1f}s '
                  f'(window {lo:.1f}..{hi:.1f}s; stuck_timeout_sec={T})', ev, ref='spec N4, notes B')
    else:
        g = S['goal']
        now = n.now()
        stall = stall_before(now)[0] if n.pose else 0.0
        armed = g is not None and (a.kind == 'manual' or
                                   (now - stall > g[0] and n.max_displacement(g[0], now - stall) > thr))
        if g is not None and armed and stall >= 1.0 and n.first_status(g[1], 3) is None:
            rep.check('stall-observed', C.PASS, f'goal {g[1]}: robot still for {stall:.1f}s at the end of the run')
            if a.kind == 'manual' and a.exploration == 'off':
                if a.selector_log:
                    rep.check('release-timing', C.PASS if (log_lines and stall >= lo) else
                              (C.FAIL if stall > hi else C.INCONCLUSIVE),
                              f'exploration off: no new goal expected; {len(log_lines)} stuck-release log line(s)',
                              log_lines[:3], ref='spec N4')
                else:
                    rep.check('release-timing', C.INCONCLUSIVE,
                              'exploration off: the release publishes nothing; re-run with --selector-log',
                              ref='spec N4')
            elif stall > hi:
                rep.check('release-timing', C.FAIL,
                          f'robot still for {stall:.1f}s and the goal was NOT replaced (limit {hi:.1f}s)',
                          ref='spec N4')
            else:
                rep.check('release-timing', C.INCONCLUSIVE,
                          f'robot still only {stall:.1f}s when the run ended; hold it longer', ref='spec N4')
        else:
            rep.check('stall-observed', C.INCONCLUSIVE,
                      f'no stalled {a.kind} goal seen (goal tracked: {g is not None}). Block the robot AFTER it has '
                      f'moved (frontier) / after the goal was sent (manual) for > {T + a.react_sec:.0f}s.',
                      ref='spec N4')
    if a.selector_log:
        rep.check('log-line', C.PASS if log_lines else C.FAIL,
                  f'{len(log_lines)} "stuck (no motion ...)" line(s) in {a.selector_log}', log_lines[:3])
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
