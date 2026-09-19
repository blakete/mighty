#!/usr/bin/env bash
# mighty_hw.sh — SINGLE entrypoint for the CONTAINERIZED MIGHTY hardware stack,
# host-tmux variant (sibling of the rover drive/sensors/dlio services): the
# hw-mighty container just idles (`sleep infinity`, see compose.hw.yaml) and
# every node runs as a `docker exec` whose controlling pane lives in a HOST tmux
# session 'hw_mighty', built right here in bash.
#
#   mighty_hw.sh start [--dev]     # bring the container up + build the session
#   mighty_hw.sh start --monitor   # same, then hold the foreground while the session
#                                  # lives (mighty.service's main process, see mighty.service.in)
#   mighty_hw.sh attach            # tmux attach (Ctrl-b d detaches; stack keeps running)
#   mighty_hw.sh stop              # kill session, then compose down
#   mighty_hw.sh status            # container + pane status
#   mighty_hw.sh logs [...]        # container PID-1 output (idle loop; panes hold the real logs)
#   mighty_hw.sh pull [<tag>]      # fleet registry <tag> (default latest) -> mighty-hw:local
#   mighty_hw.sh rebuild [...]     # stop + compose build + start (start flags forwarded)
#
# PARAMETERS come from this checkout's config/ (bind-mounted over the image's
# copy, see compose.hw.yaml): edit the YAML, Ctrl-C / Up / Enter the pane. No
# rebuild. Code, launch files and mpc.yaml are baked and need one.
#
# FOUR panes, always:
#   MIGHTY planner | convert_odom_to_state | MPC | RViz 2D goal
# The first three are onboard_mighty.launch.py's three nodes, one pane each via
# its only_nodes:= filter, so Ctrl-C -> Up -> Enter restarts ONE node instead of
# all three. The launch file still computes every parameter (mighty_node's are a
# YAML merge plus overrides — never hand-roll them as ros2 run).
#
# ROLLBACK of the per-node split is ASYMMETRIC: revert THIS HOST SCRIPT ONLY
# (git checkout -- docker/mighty_hw.sh) and start again — no rebuild. Do NOT
# revert the launch file while keeping this script: ros2 launch silently ignores
# an argument it does not declare, so every pane would start the FULL node set
# (three publishers on cmd_vel_auto). start() refuses to run against an image
# whose launch file has no only_nodes:=.
#
# THIS STACK RUNS THE PLANNER AND CONTROLLER ONLY. The rest is owned elsewhere:
#   drive.service    zenoh router on :7447 (hard prereq — start fails fast without it)
#   sensors.service  livox + D455 + the base_link->lidar static TF
#   dlio.service     DLIO odometry, the seed pose and tf map->odom (dlio_ws)
#   OX08 Orin        elevation_mapping_cupy -> <ns>/occ_2d_topic, <ns>/esdf_2d_topic
#                    (without it MIGHTY runs but never plans: no occupancy grid)
# MIGHTY consumes <ns>/dlio/odom_node/odom + map->odom over zenoh and does not
# care which container publishes them. The old --odom-type dlio / dlio_in_mocap /
# mocap modes, which ran DLIO or mocap TFs in here, went away with the DLIO layer
# of the image; mocap would come back as static-TF panes if ever needed.
#
# --dev (laptop): no /etc/rover/rover.env, no /home/swarm/config, no
# drive.service. Uses dev/rover.env (ROBOT_NAME=RR99) + dev/zenoh_*_config.json5
# and starts an ISOLATED zenoh router on 127.0.0.1:7448 from the same image
# (compose profile 'dev'). It dials nothing: a dev stack must not join the fleet.
#
# Identity (ROBOT_NAME, RMW) comes from /etc/rover/rover.env via compose
# env_file; nothing in this directory is rover-specific.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=hw-mighty
SESSION=hw_mighty
IMAGE=mighty-hw:local
REGISTRY=registry.gitlab.com/mit-acl/ugv/redrover/rover/mighty-hw
ZENOH_ROUTER_PORT="${ZENOH_ROUTER_PORT:-7447}"
COMPOSE=(docker compose -f "${SCRIPT_DIR}/compose.hw.yaml")

# In-container command preludes. Pane commands are sent single-quoted, so $VARs
# in them expand INSIDE the container (env_file provides ROBOT_NAME etc.) — and
# therefore a pane command must never contain a single quote. Every pane
# spin-waits on the router first; the guard shares the pane's history line so
# Ctrl-C -> Up -> Enter re-runs it too. wait_router is a function, not a
# constant, because --dev moves the port after the flags are parsed.
SETUP='source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash'
wait_router() {
    echo "until (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; do echo \"waiting for zenoh router on :${ZENOH_ROUTER_PORT}...\"; sleep 2; done"
}
dx() { echo "docker exec -it ${CONTAINER} bash -c '${SETUP} && $(wait_router) && $1'"; }

attach() { exec tmux attach -t "${SESSION}"; }

usage() {
    echo "usage: $0 {start [--dev|--monitor] | attach | stop | status | logs | pull [<tag>] | rebuild [start flags]}" >&2
    exit 2
}

start() {
    local dev=false monitor=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --dev) dev=true ;;
            --monitor) monitor=true ;;
            *) echo "[mighty_hw] unknown option: $1" >&2; usage ;;
        esac
        shift
    done

    if [[ "${dev}" == true ]]; then
        # A pre-set ROVER_ENV_FILE wins, so a bag replay can run the dev stack
        # under the recording rover's name (ROBOT_NAME=RR08) — see README.
        export ROVER_ENV_FILE="${ROVER_ENV_FILE:-${SCRIPT_DIR}/dev/rover.env}"
        export ROVER_CONFIG_DIR="${SCRIPT_DIR}/dev"
        ZENOH_ROUTER_PORT=7448
        COMPOSE+=(--profile dev)
        echo "[mighty_hw] --dev: identity from dev/rover.env, isolated zenoh router on 127.0.0.1:${ZENOH_ROUTER_PORT}"
    else
        # Fail fast on the hard prereq, warn on the soft ones — the pane-side
        # spin-waits still guard every node, this is just early readable
        # feedback. grep -c, not grep -q: under pipefail an early-exiting grep -q
        # SIGPIPEs the upstream docker ps and returns 141 ON MATCH.
        # Under --monitor (systemd) the router is a WAIT, not an exit: at boot
        # After=drive.service is satisfied as soon as drive_host_tmux.sh is
        # running, which is before zenohd inside it listens, so failing here
        # would fail the unit on every cold boot.
        if ! (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; then
            if [[ "${monitor}" == true ]]; then
                echo "[mighty_hw] waiting for the zenoh router on 127.0.0.1:${ZENOH_ROUTER_PORT} (drive.service)..."
                until (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; do sleep 2; done
            else
                echo "[mighty_hw] no zenoh router on 127.0.0.1:${ZENOH_ROUTER_PORT} — start drive.service first" >&2
                exit 1
            fi
        fi
        if ! docker ps --format '{{.Names}}' | grep -cx sensors >/dev/null; then
            echo "[mighty_hw] WARNING: no 'sensors' container (sensors.service down?) — no lidar, no odometry" >&2
        fi
        if ! systemctl is-active --quiet dlio.service 2>/dev/null \
           && ! docker ps --format '{{.Names}}' | grep -cx dlio >/dev/null; then
            echo "[mighty_hw] WARNING: dlio.service is not running — MIGHTY and MPC will wait on tf map->odom" >&2
        fi
    fi

    "${COMPOSE[@]}" up -d
    echo "[mighty_hw] waiting for the ${CONTAINER} container..."
    local i
    for (( i = 0; i < 30; i += 2 )); do
        [[ -n "$(docker ps -q --filter "name=^${CONTAINER}$")" ]] && break
        sleep 2
    done
    if [[ -z "$(docker ps -q --filter "name=^${CONTAINER}$")" ]]; then
        echo "[mighty_hw] container did not come up; last logs:" >&2
        docker logs --tail 40 "${CONTAINER}" >&2 || true
        exit 1
    fi
    local robot_name
    robot_name="$(docker exec "${CONTAINER}" printenv ROBOT_NAME || true)"
    if [[ -z "${robot_name}" ]]; then
        echo "[mighty_hw] ROBOT_NAME is empty in the container — /etc/rover/rover.env missing?" \
             "(set ROVER_ENV_FILE, or use --dev on a laptop)" >&2
        "${COMPOSE[@]}" down
        exit 1
    fi

    # This script is HOST-side but onboard_mighty.launch.py is BAKED INTO THE
    # IMAGE, so the two can drift — and ros2 launch SILENTLY IGNORES an argument
    # it does not declare. On a stale image every per-node pane would therefore
    # launch the FULL node set (three publishers on cmd_vel_auto). Refuse to
    # build the session instead.
    if ! docker exec "${CONTAINER}" bash -c \
            "${SETUP} && ros2 launch mighty onboard_mighty.launch.py --show-args \
             2>/dev/null | grep -c only_nodes" >/dev/null 2>&1; then
        echo "[mighty_hw] this image's onboard_mighty.launch.py has no only_nodes:= argument," \
             "so each per-node pane would start the FULL stack (three publishers on" \
             "cmd_vel_auto). Rebuild or pull a newer image first." >&2
        exit 1
    fi

    # ---- pane commands (mind the single-quote rule above) --------------------
    # mighty_node and mpc both resolve <ns>/map -> <ns>/odom, so they gate on
    # wait_for_tf.py (the /tf_static startup-race fix: the static publisher
    # latches one transient-local sample that a too-early subscriber never
    # sees). The state converter is a pure sub->pub relay that never touches TF
    # — gating it too would stall a third pane for the 60 s timeout whenever the
    # TF is missing, and wait_for_tf.py exits 0 on timeout, so silently.
    local tf_gate='ros2 run mighty wait_for_tf.py $ROBOT_NAME/map $ROBOT_NAME/odom && '
    local launch_base='ros2 launch mighty onboard_mighty.launch.py x:=0.0 y:=0.0 z:=0.0 yaw:=0.0 namespace:=$ROBOT_NAME use_hardware:=true use_onboard_localization:=true robot_type:=red_rover depth_camera_name:=d455'

    # titles[i] LABELS cmds[i] — the arrays are positional, so dropping an entry
    # from one and not the other silently mislabels every pane after it. The
    # length check turns any future mismatch into a startup error.
    # NOTE: restarting the MPC pane is not instant — MPCNode builds an
    # IPOPT/collocation NLP before its first control tick.
    local -a titles cmds
    titles=('MIGHTY planner' 'convert_odom_to_state' 'MPC' 'RViz 2D goal')
    cmds=(
        "$(dx "${tf_gate}${launch_base} only_nodes:=mighty_node")"
        "$(dx "${launch_base} only_nodes:=convert_odom_to_state")"
        "$(dx "${tf_gate}${launch_base} only_nodes:=mpc")"
        "$(dx 'ros2 run mighty repub_rviz_2Dgoal.py')"
    )
    if (( ${#titles[@]} != ${#cmds[@]} )); then
        echo "[mighty_hw] BUG: ${#titles[@]} pane titles but ${#cmds[@]} commands —" \
             "panes would be mislabeled; fix the titles/cmds arrays" >&2
        exit 1
    fi

    # ---- build the HOST session (drop any stale one first). -x/-y: a detached
    # session started from a non-tty needs an explicit size; an attaching
    # client resizes it anyway.
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    tmux new-session -d -s "${SESSION}" -n main -x 220 -y 56
    tmux set-option -w -t "${SESSION}:main" pane-border-status top
    tmux set-option -w -t "${SESSION}:main" pane-border-format ' #{pane_index}: #{pane_title} '
    for (( i = 1; i < ${#cmds[@]}; i++ )); do
        tmux split-window -t "${SESSION}:main"
        tmux select-layout -t "${SESSION}:main" tiled
    done
    local -a panes
    mapfile -t panes < <(tmux list-panes -t "${SESSION}:main" -F '#{pane_id}')
    for i in "${!cmds[@]}"; do
        tmux select-pane -t "${panes[$i]}" -T "${titles[$i]}"
        tmux send-keys -t "${panes[$i]}" "${cmds[$i]}" C-m
    done
    tmux select-pane -t "${panes[0]}"

    echo "[mighty_hw] ${SESSION} session up (rover: ${robot_name}, router: 127.0.0.1:${ZENOH_ROUTER_PORT})"
    if [[ "${monitor}" == true ]]; then
        # systemd main process (same shape as drive/sensors/dlio_host_tmux.sh):
        # live exactly as long as the session does, so `tmux kill-session -t
        # hw_mighty` deactivates mighty.service and its ExecStopPost runs `stop`.
        echo "[mighty_hw] --monitor: holding while the session lives — attach with: tmux attach -t ${SESSION}"
        while tmux has-session -t "${SESSION}" 2>/dev/null; do
            sleep 5
        done
        echo "[mighty_hw] session ended."
    elif [[ -t 0 && -t 1 ]]; then
        attach
    else
        echo "[mighty_hw] no tty — attach with: tmux attach -t ${SESSION}"
    fi
}

cmd="${1:-start}"
[[ $# -gt 0 ]] && shift

case "${cmd}" in
    start)
        start "$@"
        ;;
    attach)
        attach
        ;;
    stop)
        # Session first: closing the panes HUPs the docker-exec'd nodes before
        # the container itself goes away. --profile dev so a laptop's router
        # container is removed too (no-op on a rover: nothing in that profile
        # ever ran).
        tmux kill-session -t "${SESSION}" 2>/dev/null || true
        exec "${COMPOSE[@]}" --profile dev down
        ;;
    status)
        docker ps --filter "name=^${CONTAINER}$" --format 'container: {{.Names}}  {{.Status}}  ({{.Image}})' | grep . \
            || { echo "container: not running"; exit 1; }
        tmux list-panes -t "${SESSION}" \
            -F 'pane #{pane_index}  #{pane_title}  (#{pane_current_command})' 2>/dev/null \
            || echo "no ${SESSION} tmux session on the host"
        ;;
    logs)
        exec docker logs "$@" "${CONTAINER}"
        ;;
    pull)
        tag="${1:-latest}"
        docker pull "${REGISTRY}:${tag}"
        docker tag "${REGISTRY}:${tag}" "${IMAGE}"
        echo "[mighty_hw] ${IMAGE} is now ${REGISTRY}:${tag}"
        ;;
    rebuild)
        # The build clones the private mpc repo over SSH: uses your agent, or
        # SSH_KEY=~/.ssh/<key> for a passphrase-free key when there is none.
        "$0" stop || true
        "${COMPOSE[@]}" build --ssh "default${SSH_KEY:+=${SSH_KEY}}"
        exec "$0" start "$@"
        ;;
    *)
        usage
        ;;
esac
