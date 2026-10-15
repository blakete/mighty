#!/usr/bin/env bash
# Run the goal-selector / planner BAG tests on one recording, in one command.
#
#   run_bag_tests.sh <augmented bag dir> [--ns RR08] [--env FILE] [--out DIR]
#
# What it does (open loop: the robot follows the recording, planner + goal_selector run on top):
#   1. starts docker/dev/replay/pass2_laptop.sh in the background with KEEP_RUNNING=1 (dev stack
#      under the recording rover's name + restamp relay + ros2 bag play of the planner inputs);
#   2. as soon as the hw_mighty tmux session exists, pipes every pane (planner, MPC, goal_selector,
#      state converter) into <out>/logs/*.log;
#   3. starts the monitors in a helper container (mighty-hw:local, same zenoh session as the stack):
#        scripts/tests/monitor_trajectory_vs_map.py  -> trajectory_vs_map.json   (bag tests 1, 4)
#        scripts/tests/monitor_selector_contract.py  -> selector_contract.json   (bag tests 2, 5)
#   4. waits for pass2 to finish (bag duration + 25 s tail), stops the monitors (SIGINT -> they
#      write their reports), tears the stack down (unless KEEP_RUNNING=1), and runs
#      scripts/tests/summarize_run.py (bag test 3 + roll-up) -> <out>/REPORT.md.
#
# Output directory (default /tmp/gs_bag_tests/<bag>_<timestamp>):
#   REPORT.md  summary.json  trajectory_vs_map.json  selector_contract.json
#   logs/*.log  monitors/*.stdout  pass2.log
#
# Requirements: docker, tmux (host), the mighty-hw:local image (code baked in; config/ is read from
# this checkout), python3 on the host for the summary. Do not run while a docker build is running.
# Exit code: 0 = no FAIL check, 1 = at least one FAIL, 2 = could not run.
#
# Env: KEEP_RUNNING=1 leaves the stack up afterwards (as in pass2_laptop.sh); EXTRA_MONITOR_ARGS="..."
# are appended to both monitors.
set -uo pipefail
BAG=""; NS="RR08"; ENVF=""; OUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ns) NS="$2"; shift 2 ;;
    --env) ENVF="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n 2,28p "$0"; exit 0 ;;
    *) if [[ -z "$BAG" ]]; then BAG="$1"; shift; else echo "unexpected arg $1" >&2; exit 2; fi ;;
  esac
done
[[ -d "$BAG" ]] || { echo "usage: $0 <augmented bag dir> [--ns RR08] [--env FILE] [--out DIR]" >&2; exit 2; }
BAG="$(cd "$BAG" && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; D="$(cd "$HERE/../.." && pwd)"; REPO="$(cd "$D/.." && pwd)"
ENVF="${ENVF:-$HERE/rover.$NS.env}"
[[ -f "$ENVF" ]] || { echo "no rover env file $ENVF (pass --env)" >&2; exit 2; }
OUT="${OUT:-/tmp/gs_bag_tests/$(basename "$BAG")_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT/logs" "$OUT/monitors"; OUT="$(cd "$OUT" && pwd)"
command -v tmux >/dev/null || { echo "tmux is required on the host" >&2; exit 2; }
echo "[bag-tests] bag=$BAG ns=$NS env=$ENVF out=$OUT"

# 1. the replay (pass2 sleeps ~11 s after the stack is up before it plays, so the monitors below are ready in time)
KEEP_RUNNING=1 "$HERE/pass2_laptop.sh" "$BAG" "$ENVF" > "$OUT/pass2.log" 2>&1 &
P2=$!

# 2. wait for the tmux session and pipe its panes to files
for i in $(seq 1 180); do
  if tmux list-panes -t hw_mighty -F '#{pane_title}' 2>/dev/null | grep -q goal_selector; then break; fi
  kill -0 $P2 2>/dev/null || { echo "[bag-tests] pass2 exited early:"; tail -5 "$OUT/pass2.log"; exit 2; }
  sleep 1
done
PANES=()
while IFS='|' read -r pid title; do
  f="$OUT/logs/$(echo "$title" | tr -c 'A-Za-z0-9\n' '_').log"
  tmux pipe-pane -t "$pid" "cat >> '$f'"; PANES+=("$pid")
done < <(tmux list-panes -t hw_mighty -F '#{pane_id}|#{pane_title}')
echo "[bag-tests] logging ${#PANES[@]} panes"

# 3. monitors
docker rm -f gs-tests >/dev/null 2>&1 || true
docker run -d --name gs-tests --network host --init --env-file "$ENVF" \
  -e ZENOH_SESSION_CONFIG_URI=/home/swarm/config/zenoh_session_config.json5 \
  -v "$D/dev:/home/swarm/config:ro" -v "$REPO/scripts/tests:/tests:ro" -v "$REPO/config:/cfg:ro" \
  -v "$OUT:/out" mighty-hw:local sleep infinity >/dev/null || { echo "cannot start helper container" >&2; exit 2; }
SRC='source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash'
for m in monitor_trajectory_vs_map:trajectory_vs_map monitor_selector_contract:selector_contract; do
  script="${m%%:*}"; name="${m##*:}"
  docker exec -d gs-tests bash -c "$SRC && cd /tests && python3 $script.py --ns $NS --config /cfg/hw_mighty_ground_robot.yaml --out /out/$name.json ${EXTRA_MONITOR_ARGS:-} > /out/monitors/$script.stdout 2>&1"
done

# 4. wait for the replay to finish
wait $P2; P2RC=$?
sleep 3
docker exec gs-tests pkill -INT -f 'python3 monitor_' >/dev/null 2>&1
for i in $(seq 1 60); do
  [[ -s "$OUT/trajectory_vs_map.json" && -s "$OUT/selector_contract.json" ]] && break
  sleep 1
done
for pid in "${PANES[@]}"; do tmux pipe-pane -t "$pid" 2>/dev/null || true; done
docker rm -f gs-tests >/dev/null 2>&1
if [[ "${KEEP_RUNNING:-0}" != "1" ]]; then (cd "$D" && ./mighty_hw.sh stop 2>&1 | tail -1); fi

# 5. summary
python3 "$REPO/scripts/tests/summarize_run.py" "$OUT" > "$OUT/summary.stdout" 2>&1
RC=$?
cat "$OUT/summary.stdout" | head -80
echo
echo "[bag-tests] pass2 rc=$P2RC; report: $OUT/REPORT.md (rc=$RC)"
exit $RC
