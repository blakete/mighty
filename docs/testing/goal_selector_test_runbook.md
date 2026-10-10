# Goal selector / planner: bag and hardware test runbook

Sources: `goal_selector_refactor_spec.md` (§3, §4, §11, New Plan N1-N5) and `goal_selector_test_ideas.md`
(sections A-E). Scripts live in `scripts/tests/`; the bag wrapper is `docker/dev/replay/run_bag_tests.sh`.
Rule: a failing check is a finding. Do not edit a check or its criterion to make it pass.

Verdicts: PASS, FAIL, INCONCLUSIVE (the situation never occurred), INFO (measured, not judged).
Every monitor prints a text report and writes JSON with `--out`. Exit code 1 means at least one FAIL.

## 0. Common setup

- Image `mighty-hw:local` (ROS 2 Humble, `dynus_interfaces`, `goal_selector_msgs`, numpy, scipy, PyYAML).
  Monitors need the same zenoh session as the stack: on a robot, run them with `docker exec -it hw-mighty bash`
  and `source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash`, then
  `cd <repo>/scripts/tests` (mount or copy the directory into the container; the wrapper mounts it at `/tests`).
- Namespace: `--ns RR08` (default `$ROBOT_NAME`). Parameters are read from `config/hw_mighty_ground_robot.yaml`
  and `config/hw_goal_selector.yaml` (`/cfg/...` inside the wrapper's container); override on the command line.
- RViz displays (sidecar config): `RR08 Selector Map (inflated)`, `RR08 Planner Map (inflated)`,
  `RR08 Point Selector Goal`, `RR08 Point Term G`. Costmap colours: band 99 cyan, obstacle 100 magenta.
- Capturing the selector pane for `--selector-log`: `tmux pipe-pane -t hw_mighty:main.3 "cat >> /tmp/selector.log"`
  (pane index per `tmux list-panes -t hw_mighty -F '#{pane_index} #{pane_title}'`).
- Do not run the bag wrapper while a docker build is running (take the build lock).

## 1. Bag tests (open loop)

The robot follows the recording; the planner and goal_selector run on top. REACHED/VISITED, return home and real
motion cannot be exercised here.

### BAG-ALL: one command

```
docker/dev/replay/run_bag_tests.sh <augmented bag dir> [--ns RR08] [--env FILE] [--out DIR]
# e.g.
docker/dev/replay/run_bag_tests.sh /home/suchi/code/data/rosbags/bag_20260806_151753_RR08_scene5_planner_replay --ns RR08
```

It starts `pass2_laptop.sh` with `KEEP_RUNNING=1`, pipes the tmux panes into `logs/`, runs
`monitor_trajectory_vs_map.py` and `monitor_selector_contract.py` in a helper container, stops them when the
replay ends, tears the stack down (set `KEEP_RUNNING=1` to keep it for RViz), then runs `summarize_run.py`.
Output directory (default `/tmp/gs_bag_tests/<bag>_<time>`): `REPORT.md`, `summary.json`,
`trajectory_vs_map.json`, `selector_contract.json`, `logs/*.log`, `monitors/*.stdout`, `pass2.log`.
Re-summarise only: `python3 scripts/tests/summarize_run.py <dir>`. Run time is bag duration + about 60 s.
Operator action: none. To watch in RViz use `KEEP_RUNNING=1` and `tmux attach -t hw_mighty`.

| Bag test | Script / check | Pass criteria |
|---|---|---|
| 1 Executed trajectory vs map | `monitor_trajectory_vs_map.py`: `trajectory-unknown` (INFO), `trajectory-occupied` | Unknown crossings are counted (messages, distinct plans, samples, longest run) and not judged. FAIL on any occupied sample (notes E, N2/N3). Same for `mpc_waypoints`. |
| 4 Prefix | `prefix-unknown-run`, `prefix-occupied`, `prefix-unknown-short-runs` | FAIL if a prefix (`hgp_path_marker`) holds a run of at least `trim_min_unknown_run_cells` unknown cells or any occupied sample. |
| 2 Statuses | `monitor_selector_contract.py`: `status-counts`, `failed-bursts`, `failed-burst-reaction`, `status-goal-stamp`, `stale-status`, `term-goal-stamps` | Counts reported. FAIL: status stamp never published as a term_goal; stale status later than 2 s after a newer goal; duplicate/non-increasing term_goal stamps; FAILED burst at threshold on a frontier goal with no new goal within 4 s. |
| 3 Regression totals | `summarize_run.py` from the pane logs | Frontier commits, invalidations by logged reason (unreachable, stuck), preemptions, return-home lines, manual goal lines. FAIL if any log line matches `relocat` (case-insensitive). Pursuit-timeout and occupied-band invalidations are not logged by the selector and cannot be counted from logs. |
| 5 Visualisation | `selector-map-values`, `selector-map-rate`, `point-selector-goal`, `no-relocation` | `selector_map_2d` values within {-1,0,50,99,100}; at most 15 maps in any 10 s; exactly one `point_selector_goal` per `term_goal` with the same x, y; term_goal identical to its `exploration/current_goal`. |

How to read the output: start at `REPORT.md` "Monitor checks" (FAIL rows first), then the count tables. In the
JSON, `trajectory_vs_map.json` `checks[].evidence` lists the first offenders (time, plan id, world x, y,
arc length, start/end of the plan). Caveat: the monitor compares against the newest grid received before the
message; `grid_age` in the evidence shows how old it was.

Manual goal in a bag replay (optional, interactive): run with `KEEP_RUNNING=1`, publish
`ros2 topic pub --once /RR08/term_goal_rviz geometry_msgs/msg/PoseStamped ...` (or RViz 2D Goal Pose) while a
monitor runs; `rviz-to-term-goal` then reports the forwarding.

## 2. Hardware tests (real motion)

Common start: `mighty_hw.sh start` on the robot, exploration enabled unless stated, RViz open with the four
displays above, monitor started BEFORE the action. All monitors print live event lines.

### HW-1 Goal reached -> VISITED -> next goal
- Purpose: spec §3 REACHED path, notes B.
- Setup: exploration on, area with at least two frontiers.
- Operator: arm the robot and let it explore; do nothing else.
- Command: `python3 hw_check_goal_cycle.py --ns RR08 --cycles 2 --out /tmp/goal_cycle.json`
- Pass: per cycle, REACHED status carries the goal's stamp, `goal_reached` arrives, robot within 0.8 m, the next
  term_goal comes within 15 s and is more than `merge_radius_m` from the reached goal. FAIL if the same
  frontier is re-selected. INCONCLUSIVE means nothing was reached or no next goal (see marker count in evidence).

### HW-2 Return home arrives and the latch resets
- Setup: exploration on; note the start pose from the selector log line `Exploration: starting from (x, y, z)`.
- Operator: after some exploring run `ros2 topic pub --once /exploration/return_home std_msgs/msg/Empty "{}"`
  (or let the monitor do it: `--send-trigger-after 20`). For the latch re-arm: after arrival drive the robot away
  (manual goal) and publish the trigger again.
- Command: `python3 hw_check_return_home.py --ns RR08 --home X Y --out /tmp/rh.json`
- Pass: home term_goal within 5 s of the trigger and equal to home within 0.3 m; no other term_goal until
  arrival; REACHED with its stamp and robot within 0.8 m; no repeated home goal for 20 s after arrival; a second
  trigger yields a new home goal. Without a second trigger only the first half is judged (stated in the output).

### HW-3 Stuck watchdog
- Setup: frontier case: exploration on. Manual case: click a 2D Goal Pose a few metres away.
- Operator: let the robot start moving, then block it (hold, wall, lift wheels) for more than 10 s.
- Commands: `python3 hw_check_stuck_watchdog.py --ns RR08 --kind frontier` and
  `... --kind manual [--exploration off --selector-log /tmp/selector.log]`
- Pass: the goal is replaced when the robot has been still between `stuck_timeout_sec - 0.5` and
  `stuck_timeout_sec + 4` s (5 s configured); for a manual goal the new goal is a frontier goal (exploration on).
  FAIL: premature or missing release. A replacement while the robot was moving is ignored as another reason.

### HW-4 Manual goal through RViz reaches the planner
- Operator: 2D Goal Pose on free floor.
- Command: `python3 hw_check_manual_goal.py --ns RR08 --scenario rviz`
- Pass: `term_goal` equals the click within 1e-3 m within 2 s, `planner_status` with that goal_stamp within 3 s,
  no frontier goal in the next 10 s.

### HW-5 Manual goal start timeout
- Operator: click a 2D Goal Pose on an obstacle (visible magenta on `RR08 Planner Map (inflated)`), keep the robot still.
- Command: `python3 hw_check_manual_goal.py --ns RR08 --scenario start-timeout [--selector-log /tmp/selector.log]`
- Pass: goal published unchanged and kept for the first 14 s (planner FAILED does not release it); a frontier goal
  appears 14.5 to 19 s after the commit (`manual_goal.start_timeout_sec` 15). INCONCLUSIVE if the robot moved
  more than 0.10 m (then use HW-3 manual). The release log line is `Manual goal (stamp N) start timeout ...`.

### HW-6 Small unknown holes resolve on approach
- Operator: drive (manual goals or exploration) through an area where the Planner Map shows small unknown holes.
- Command: `python3 hw_check_unknown_holes.py --ns RR08 --max-hole-cells 8 --approach 1.5 --out /tmp/holes.json`
- Pass: every hole the robot came within 1.5 m of is fully known in the last grid. The criterion is an
  ASSUMPTION (no number in the notes). INCONCLUSIVE if no hole was approached.

### HW-7 Multi-robot peer sharing (two robots)
- Setup: both robots with `exploration.minpos.enabled: true` (currently false in `hw_goal_selector.yaml`).
- Command: `python3 hw_check_multirobot.py --ns RR08 --peer RR09 --out /tmp/multi.json`
- Pass: both ids on `/exploration/peer_poses` at 2 Hz or more and equal to their own state within 0.5 m; both
  publish `/exploration/visited_maps`; at least 95% of the peer's known cells are known in our visited map;
  no frontier goal closer than `min_frontier_dist_to_peers_m` (3 m) to the peer's pose at commit.

## 3. Coverage table

gtests are written by other workers (`src/test/test_goal_selector_*.cpp`, `test_frontier_*`,
`test_map2d_inflate_unknown.cpp`, `test_planner_*`); the gtest column states the intended owner, not a verified mapping.

| Test idea (notes) | gtest | Bag | Hardware |
|---|---|---|---|
| A ranking, tie-break, MinPos ordering, utilityOf | yes | - | HW-7 (peer exclusion only) |
| B failure threshold, SKIPPED neutral, stale status ignored | yes | status counts, stamps, FAILED burst reaction | - |
| B stuck watchdog, pursuit budget, preemption | yes | stuck-invalidation and preemption counts (logs) | HW-3 |
| B REACHED -> VISITED -> next goal | yes (core) | - | HW-1 |
| B return home latch, trigger stops frontier goals | yes (core) | log lines (none expected) | HW-2 |
| B manual goal override, unchanged publication, start timeout | yes (core) | `no-relocation`, `rviz-to-term-goal` (if a click is injected) | HW-4, HW-5 |
| C frontier placement, no instant VISITED, keep-out, inflateUnknown | yes | frontier commit counts | HW-6 (holes) |
| C visited_map absorb / peer merge | yes | - | HW-7 |
| D one goal per commit, no relocation, exploration off | yes | `point-selector-goal`, `no-relocation`, `relocat` grep | HW-4 |
| E trajectory vs map, prefix, global path into unknown | planner gtests (prefix trim) | trajectory and prefix monitors | - |
| E planner status correctness | planner gtests | status counts only (correctness of FAILED/PARTIAL vs map is not checked) | - |
| Viz: selector_map_2d values/rate, point_selector_goal | - | contract monitor | RViz displays by eye |

## 4. Known limits

- Bag runs cannot show REACHED beyond the single arrival of the recording, return home, or peer behaviour.
- Trajectory checks use the newest grid before the message; a cell that changed in between can be reported.
- `FrontierManager` pursuit-timeout, occupied-band and VISITED transitions are not logged, so they are not counted.
- Statuses: `stale-status` tolerance 2 s and `failed-burst-reaction` window 4 s are ASSUMPTIONs.
