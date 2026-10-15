#!/usr/bin/env python3
"""HARDWARE test: return home actually arrives, and the latch resets.

Needs real motion (spec §4 return-home, §11.10; notes B "No frontiers left -> return home; the
latch holds while en route and re-arms on arrival", "return_home trigger stops new frontier goals").

SETUP: stack running with exploration.enabled: true; the robot has been exploring for a bit
(the selector captured its START pose when it committed its first frontier, log line
"Exploration: starting from (x, y, z)"). Start this monitor, then trigger the return:
    ros2 topic pub --once /exploration/return_home std_msgs/msg/Empty "{}"
(or let the frontiers run out). The monitor can also send the trigger itself: --send-trigger-after 20.
AUTOMATIC RETURN HOME can be switched off (hw_goal_selector.yaml exploration.return_home.enabled, false
for the 2026-10 hardware tests; spec §11 item 23). The monitor reads that switch: when it is off,
"let the frontiers run out" does NOT send the robot home (the selector goes idle), so only the
/exploration/return_home trigger can exercise this test -- use --send-trigger-after or publish it.
Give the start pose with --home X Y (read it from that log line); without it the pose of the first
state message seen by this monitor is used, which is only right if you start the monitor at the
start position.

RUN:
    python3 hw_check_return_home.py --ns RR08 --home 0.0 0.0 [--timeout 300] [--out /tmp/rh.json]
  Optional second trigger test: after the robot arrived, drive/send it away (manual 2D Goal Pose
  in RViz) and publish the trigger again; the monitor waits --retrigger-wait s (default 60) for it.

CRITERIA
  home-goal-published  PASS: after the trigger (or, with no trigger seen, a term_goal that is NOT a
                       frontier goal and lies within --home-tol 0.3 m of home) a term_goal at home
                       is published within --trigger-sec (default 5 s). x, y equal to home within
                       --home-tol (no relocation, spec N5). INCONCLUSIVE if no trigger/home goal.
                       With automatic return home OFF: a home goal with no trigger seen is a FAIL
                       (the selector must go idle instead), and "no home goal, no trigger" is
                       INCONCLUSIVE (not applicable without the trigger).
  no-frontier-goals    PASS: no other term_goal (frontier or otherwise) is published between the home
                       goal and arrival ("return_home trigger stops new frontier goals").
  arrival              PASS: planner_status REACHED with goal_stamp == the home goal's stamp, and the
                       robot is within goal_radius + 0.3 m of home then. FAIL: no REACHED within
                       --arrive-timeout (default 240 s) of the home goal, or REACHED but robot far.
  latch-reset          PASS: after arrival no further home goal is published for --quiet-sec (20 s)
                       (the latch re-armed instead of re-sending home forever) -- AND, if the operator
                       triggers a second return after moving the robot away, a new home goal follows
                       within --trigger-sec. Without a second trigger the second part is INCONCLUSIVE
                       and the overall latch verdict reports only the first part (stated in the text).

OUTPUT: live event log, verdicts, JSON with --out. Exit 1 on FAIL.
"""
import math
import sys

import rclpy
from std_msgs.msg import Empty

import gs_test_common as C
import gs_live as L


def main():
    ap = C.base_parser('HW: return home', __doc__)
    ap.add_argument('--home', type=float, nargs=2, metavar=('X', 'Y'), default=None)
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--send-trigger-after', type=float, default=0.0,
                    help='publish /exploration/return_home after this many seconds (0 = operator does it)')
    ap.add_argument('--home-tol', type=float, default=0.3)
    ap.add_argument('--trigger-sec', type=float, default=5.0)
    ap.add_argument('--arrive-timeout', type=float, default=240.0)
    ap.add_argument('--arrive-tol', type=float, default=0.8)
    ap.add_argument('--quiet-sec', type=float, default=20.0)
    ap.add_argument('--retrigger-wait', type=float, default=60.0)
    ap.add_argument('--auto-return', choices=['auto', 'on', 'off'], default='auto',
                    help='automatic return home (exploration.return_home.enabled); auto = read it from '
                         '/cfg/hw_goal_selector.yaml')
    a, ros_args = ap.parse_known_args()
    if a.auto_return == 'auto':
        v = C.load_param('exploration.return_home.enabled', True, '/cfg/hw_goal_selector.yaml')
        auto_return = v if isinstance(v, bool) else str(v).lower() == 'true'
    else:
        auto_return = a.auto_return == 'on'
    a.duration = a.duration or a.timeout
    rclpy.init(args=ros_args)
    n = L.Live(a.ns, 'hw_check_return_home')
    flag = C.StopFlag(a.duration)
    pub = n.create_publisher(Empty, '/exploration/return_home', C.qos_reliable(1))
    L.wait_for_robot(n, flag)
    home = tuple(a.home) if a.home else ((n.pose[1], n.pose[2]) if n.pose else None)
    n.say(f'home = {home}  (from {"--home" if a.home else "first state message"})')
    sent = [False]
    st = dict(home_goal=None, arrived=None, retrigger_goal=None, state='wait_trigger', t_arrive=None)

    def is_home(g):
        return home and math.hypot(g[2] - home[0], g[3] - home[1]) <= a.home_tol

    def tick():
        t = n.now()
        if a.send_trigger_after and not sent[0] and n.rel() >= a.send_trigger_after:
            pub.publish(Empty())
            sent[0] = True
        if st['home_goal'] is None:
            cands = [g for g in n.tg if is_home(g) and not n.is_frontier_goal(g)]
            if cands:
                st['home_goal'] = cands[0]
                n.say(f'home goal published, stamp {cands[0][1]}')
        elif st['arrived'] is None:
            tr = n.first_status(st['home_goal'][1], 3)
            if tr is not None:
                p = n.pose_at(tr)
                st['arrived'] = dict(t=tr, dist=math.hypot(p[1] - home[0], p[2] - home[1]) if p else None)
                st['t_arrive'] = tr
                n.say(f'REACHED at home, robot {st["arrived"]["dist"]} m from it')
        else:
            # after arrival: look for a second trigger and a second home goal
            later = [r for r in n.return_home if r > st['t_arrive'] + 1.0]
            if later and st['retrigger_goal'] is None:
                g = [g for g in n.tg if g[0] >= later[0] - 0.2 and is_home(g)]
                if g:
                    st['retrigger_goal'] = g[0]
                elif t - later[0] > a.trigger_sec + 1:
                    st['retrigger_goal'] = False
            if st['retrigger_goal'] is not None:
                return True
            if t - st['t_arrive'] > a.quiet_sec + a.retrigger_wait:
                return True
        return False

    L.run(n, flag, tick)
    tick()
    rep = C.Report('HW: return home', dict(ns=a.ns, home=home, auto_return=auto_return))
    hg = st['home_goal']
    trig = [r for r in n.return_home]
    if hg is None:
        why = ('automatic return home is OFF, so only the /exploration/return_home trigger sends the robot '
               'home (not applicable without it; use --send-trigger-after). ' if not auto_return and not trig
               else 'no trigger sent, or frontiers did not run out, or --home wrong. ')
        rep.check('home-goal-published', C.INCONCLUSIVE,
                  f'no home term_goal observed ({why}triggers seen: {len(trig)}; term_goals: {len(n.tg)})',
                  ref='notes B; spec §11 item 23')
    elif not auto_return and not [r for r in trig if r <= hg[0] + 0.2]:
        rep.check('home-goal-published', C.FAIL,
                  f'home goal at ({hg[2]:.2f}, {hg[3]:.2f}) published WITHOUT a trigger although automatic '
                  'return home is off (exploration.return_home.enabled=false): the selector should have gone idle',
                  ref='spec §11 item 23')
    else:
        lat = None
        if trig:
            before = [r for r in trig if r <= hg[0] + 0.2]
            lat = hg[0] - before[-1] if before else None
        ok = lat is None or lat <= a.trigger_sec
        rep.check('home-goal-published', C.PASS if ok else C.FAIL,
                  f'home goal at ({hg[2]:.2f}, {hg[3]:.2f}), within {a.home_tol} m of home; '
                  f'{"latency after trigger %.1fs" % lat if lat is not None else "no trigger message observed (goal came from exhausted frontiers or the trigger preceded the monitor)"}',
                  ref='spec §4, N5 (no relocation)')
        end_t = st['arrived']['t'] if st['arrived'] else n.now()
        extra = [g for g in n.tg if g[1] > hg[1] and hg[0] < g[0] <= end_t]
        rep.check('no-frontier-goals', C.FAIL if extra else C.PASS,
                  f'{len(extra)} other term_goal(s) between the home goal and arrival',
                  [f'term_goal at ({g[2]:.2f},{g[3]:.2f}) t=+{g[0] - hg[0]:.1f}s' for g in extra[:8]],
                  ref='notes B (return_home stops new frontier goals)')
        ar = st['arrived']
        if ar is None:
            waited = n.now() - hg[0]
            rep.check('arrival', C.FAIL if waited >= a.arrive_timeout else C.INCONCLUSIVE,
                      f'no REACHED status for the home goal after {waited:.0f}s '
                      f'(robot now {n.dist_to(*home) if n.pose else "?"} m from home). '
                      + ('Timeout exceeded.' if waited >= a.arrive_timeout else 'Run ended before arrival.'),
                      ref='spec §3 REACHED')
        else:
            good = ar['dist'] is not None and ar['dist'] <= a.arrive_tol
            rep.check('arrival', C.PASS if good else C.FAIL,
                      f'REACHED {ar["t"] - hg[0]:.0f}s after the home goal; robot {ar["dist"]} m from home '
                      f'(tolerance {a.arrive_tol})', ref='spec §3 REACHED')
            again = [g for g in n.tg if g[1] > hg[1] and is_home(g) and ar['t'] < g[0] <= ar['t'] + a.quiet_sec
                     and not (st['retrigger_goal'] and g[1] == st['retrigger_goal'][1])]
            part2 = st['retrigger_goal']
            msg = f'{len(again)} further home goal(s) within {a.quiet_sec}s of arrival (expect 0). '
            if part2 is None:
                msg += 'Second trigger not tested (operator did not send one) -> only the first half is judged.'
                v = C.FAIL if again else C.PASS
            elif part2 is False:
                msg += 'A second trigger was sent but NO new home goal followed: latch did not re-arm.'
                v = C.FAIL
            else:
                msg += f'second trigger produced a new home goal (stamp {part2[1]}): latch re-armed.'
                v = C.FAIL if again else C.PASS
            rep.check('latch-reset', v, msg, ref='notes B ("latch holds while en route and re-arms on arrival")')
    code = rep.finish(a.out)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
