#!/usr/bin/env python3
"""HARDWARE tests for MANUAL goals: (a) RViz term_goal_rviz reaches the planner, (b) start timeout.

--scenario rviz           A manual goal clicked in RViz ("2D Goal Pose" -> /<ns>/term_goal_rviz) becomes the
                          planner's goal unchanged (spec D5, §4.2, notes B "manual goal published
                          unchanged", "manual goal suppresses exploration").
--scenario start-timeout  A manual goal the robot cannot start towards (click on an OBSTACLE, or keep the
                          robot disarmed / held) is RELEASED manual_goal.start_timeout_sec (15 s) after the
                          commit when the robot has not moved more than stuck_move_thresh_m, and exploration
                          resumes (notes B "Manual goal stuck timeout: armed at commit ... A robot that never
                          moves toward the manual goal is released too"; config manual_goal.start_timeout_sec;
                          the planner FAILED x unreachable_consec_thresh only WARNS, it does not release).

SETUP: stack running (runbook HW-4 / HW-5). For start-timeout the robot must stay (nearly) still after the
click: click on an obstacle in RViz (RR08 Planner Map (inflated) shows it), or hold the robot. Exploration
enabled (otherwise the release is invisible on topics, pass --exploration off and --selector-log).

RUN (start BEFORE clicking in RViz):
    python3 hw_check_manual_goal.py --ns RR08 --scenario rviz
    python3 hw_check_manual_goal.py --ns RR08 --scenario start-timeout [--exploration off --selector-log pane.log]

CRITERIA (rviz)
  forwarded          PASS: a term_goal with x, y equal to the clicked pose (1e-3 m) appears within 2 s with a
                     header.stamp newer than all earlier ones (no relocation, spec N5). FAIL: missing, or
                     moved (reports the offset).
  planner-received   PASS: a planner_status whose goal_stamp == that term_goal's stamp arrives within 3 s
                     (the planner holds the new goal; spec §3). FAIL otherwise.
  suppresses-frontier  PASS: no frontier term_goal (matching exploration/current_goal) in the following
                     --suppress-sec (10 s) while the manual goal is neither REACHED nor released.
CRITERIA (start-timeout)
  forwarded          as above.
  kept-before-timeout  PASS: no new term_goal during the first start_timeout_sec-1 s although the planner
                     reported FAILED (the goal is kept, notes B). FAIL: a different goal appears earlier
                     (unless it is another click).
  released-on-time   PASS: the goal is released start_timeout_sec .. start_timeout_sec + --react-sec (4 s)
                     after the commit: with exploration ON a new FRONTIER term_goal appears; with
                     --exploration off nothing is published -> verdict from --selector-log
                     ("start timeout" line) or INCONCLUSIVE. FAIL: no release within the window, or the
                     robot moved more than the threshold during it (precondition violated -> INCONCLUSIVE).

OUTPUT: live log, verdicts, JSON with --out. Exit 1 on FAIL.
"""
import math
import re
import sys

import rclpy

import gs_test_common as C
import gs_live as L


def main():
    ap = C.base_parser('HW: manual goals', __doc__)
    ap.add_argument('--scenario', choices=('rviz', 'start-timeout'), required=True)
    ap.add_argument('--exploration', choices=('on', 'off'), default='on')
    ap.add_argument('--timeout', type=float, default=180.0, help='total run time (s)')
    ap.add_argument('--react-sec', type=float, default=4.0)
    ap.add_argument('--suppress-sec', type=float, default=10.0)
    ap.add_argument('--start-timeout', type=float, default=None)
    ap.add_argument('--stuck-thresh', type=float, default=None)
    ap.add_argument('--selector-log', default='')
    a, ros_args = ap.parse_known_args()
    a.duration = a.duration or a.timeout
    sel = '/cfg/hw_goal_selector.yaml'
    T0 = a.start_timeout if a.start_timeout is not None else float(C.load_param('manual_goal.start_timeout_sec', 15.0, sel))
    thr = a.stuck_thresh if a.stuck_thresh is not None else float(
        C.load_param('exploration.manager.stuck_move_thresh_m', 0.10, sel))
    rclpy.init(args=ros_args)
    n = L.Live(a.ns, 'hw_check_manual_goal')
    flag = C.StopFlag(a.duration)
    L.wait_for_robot(n, flag)
    first_rviz = [None]

    def tick():
        if n.rviz and first_rviz[0] is None:
            first_rviz[0] = n.rviz[0]
        if first_rviz[0] is None:
            return False
        need = (3.0 if a.scenario == 'rviz' else 0) + (a.suppress_sec if a.scenario == 'rviz' else T0 + a.react_sec + 1.0)
        return n.now() - first_rviz[0][0] > need

    n.say(f'waiting for a term_goal_rviz click on /{a.ns}/term_goal_rviz ...')
    L.run(n, flag, tick)
    tick()
    rep = C.Report(f'HW: manual goal / {a.scenario}', dict(ns=a.ns, start_timeout_sec=T0, stuck_move_thresh_m=thr))
    if first_rviz[0] is None:
        rep.check('forwarded', C.INCONCLUSIVE, 'no term_goal_rviz message was seen (no click in the run)')
        sys.exit(rep.finish(a.out) or 0)
    t_click, cx, cy = first_rviz[0]
    older = [g[1] for g in n.tg if g[0] < t_click - 0.5]
    cand = [g for g in n.tg if 0 <= g[0] - t_click <= 2.0]
    exact = [g for g in cand if math.hypot(g[2] - cx, g[3] - cy) <= 1e-3]
    if not cand:
        rep.check('forwarded', C.FAIL, f'no term_goal within 2 s of the click at ({cx:.2f},{cy:.2f})', ref='spec D5')
        sys.exit(rep.finish(a.out) or 1)
    g = exact[0] if exact else cand[0]
    mono = all(g[1] > s for s in older)
    off = math.hypot(g[2] - cx, g[3] - cy)
    rep.check('forwarded', C.PASS if (exact and mono) else C.FAIL,
              f'click ({cx:.3f},{cy:.3f}) -> term_goal ({g[2]:.3f},{g[3]:.3f}) after {g[0] - t_click:.2f}s, offset {off:.3f} m, '
              f'stamp newer than earlier goals: {mono}', ref='spec D5, N5 (published as given)')
    stamp = g[1]
    if a.scenario == 'rviz':
        t_ps = [t for (t, s, gs) in n.st if gs == stamp and 0 <= t - g[0] <= 3.0]
        rep.check('planner-received', C.PASS if t_ps else C.FAIL,
                  f'planner_status with the new goal_stamp {"after %.2fs" % (t_ps[0] - g[0]) if t_ps else "NOT seen within 3 s"}',
                  ref='spec §3')
        front = [x for x in n.tg if x[1] > stamp and x[0] - g[0] <= a.suppress_sec and n.is_frontier_goal(x)]
        reached = n.first_status(stamp, 3)
        released = reached is not None and reached - g[0] <= a.suppress_sec
        rep.check('suppresses-frontier', C.FAIL if (front and not released) else C.PASS,
                  f'{len(front)} frontier goal(s) within {a.suppress_sec}s after the manual goal'
                  f'{" (manual goal REACHED earlier, so exploration may resume)" if released else ""}',
                  [f'frontier goal at +{x[0] - g[0]:.1f}s ({x[2]:.2f},{x[3]:.2f})' for x in front[:5]],
                  ref='spec D6 (manual overrides frontiers)')
    else:
        failed = n.count_status(stamp, 1)
        disp = n.max_displacement(g[0], g[0] + T0 + a.react_sec)
        after = [x for x in n.tg if x[1] > stamp]
        early = [x for x in after if x[0] - g[0] < T0 - 1.0 and not any(abs(r[0] - x[0]) < 3.0 for r in n.rviz[1:])]
        rep.check('kept-before-timeout', C.FAIL if early else C.PASS,
                  f'{failed} FAILED status(es) for the goal; {len(early)} replacement(s) before {T0 - 1:.0f}s '
                  f'(unreachable_consec_thresh FAILED only warns for a manual goal)',
                  [f'early replacement at +{x[0] - g[0]:.1f}s' for x in early[:5]], ref='notes B, spec D8a')
        log_lines = []
        if a.selector_log:
            try:
                log_lines = [C.ansi_strip(l).strip()[:200] for l in open(a.selector_log, errors='replace')
                             if re.search(r'start timeout', l)]
            except OSError as e:
                log_lines = [str(e)]
        if disp > thr:
            rep.check('released-on-time', C.INCONCLUSIVE,
                      f'precondition violated: the robot moved {disp:.2f} m (> {thr}) after the click; the START '
                      f'timeout only applies to a robot that never moves. Use hw_check_stuck_watchdog.py --kind manual.')
        elif a.exploration == 'off':
            rep.check('released-on-time', C.PASS if log_lines else C.INCONCLUSIVE,
                      f'exploration off: release publishes nothing; {len(log_lines)} "start timeout" log line(s)',
                      log_lines[:3], ref='notes B')
        else:
            rel = [x for x in after if n.is_frontier_goal(x)]
            if not rel:
                waited = n.now() - g[0]
                rep.check('released-on-time', C.FAIL if waited > T0 + a.react_sec else C.INCONCLUSIVE,
                          f'no frontier goal after {waited:.0f}s (limit {T0 + a.react_sec:.0f}s)' if waited > T0 + a.react_sec
                          else f'run ended after {waited:.0f}s, before the timeout window', ref='notes B')
            else:
                dt = rel[0][0] - g[0]
                rep.check('released-on-time', C.PASS if T0 - 0.5 <= dt <= T0 + a.react_sec else C.FAIL,
                          f'manual goal replaced by a frontier goal {dt:.1f}s after the commit '
                          f'(window {T0 - 0.5:.1f}..{T0 + a.react_sec:.1f}s), robot moved {disp:.2f} m',
                          log_lines[:2], ref='notes B (start timeout)')
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
