#!/usr/bin/env bash
# mighty_hw.sh — SINGLE entrypoint for the CONTAINERIZED MIGHTY hardware stack,
# host-tmux variant (sibling of the rover drive/sensors services): the hw-mighty
# container just idles (`sleep infinity`, see compose.hw.yaml) and every node
# runs as a `docker exec` whose controlling pane lives in a HOST tmux session
# 'hw_mighty', built right here in bash — no tmuxp, no run_hw_red_rover.py
# (that script is the NATIVE `mighty` alias path and stays untouched).
#
#   mighty_hw.sh start [--odom-type dlio|dlio_in_mocap|mocap|external] [--two-d-only|--no-two-d]
#   mighty_hw.sh attach            # tmux attach (Ctrl-b d detaches; stack keeps running)
#   mighty_hw.sh stop              # kill session, then compose down
#   mighty_hw.sh status            # container + pane status
#   mighty_hw.sh logs [...]        # container PID-1 output (idle loop; panes hold the real logs)
#   mighty_hw.sh rebuild [...]     # stop + compose build + start (start flags forwarded)
#
# Panes by --odom-type (default: AUTO, see 'external'; two_d_only defaults OFF,
# --two-d-only enables):
#   dlio           Onboard MIGHTY | RViz 2D goal | DLIO | seed pose | tf map->odom
#                  Replicates what dlio.service ran: DLIO anchored by a constant
#                  seed-pose spoof on /<ns>/world at z=${DLIO_SEED_Z:-0.4}.
#   dlio_in_mocap  same minus the seed pose pane — a REAL mocap publishes
#                  /<ns>/world, and DLIO anchors to the FIRST pose it receives,
#                  so a running spoof would race it and win.
#   mocap          no DLIO at all: mighty flips to use_onboard_localization:=false
#                  + twist from mocap/twist, and two extra static TFs bridge
#                  the mocap frames (<ns> -> <ns>/base_link, world -> <ns>/map).
#   external       NO odometry panes at all: dlio.service (see dlio_ws) owns DLIO,
#                  the seed pose and map->odom. MIGHTY is launched exactly as in
#                  dlio mode (use_onboard_localization:=true) and consumes
#                  <ns>/dlio/odom_node/odom + the map->odom TF over zenoh, so it
#                  does not care that they come from another container.
#                  AUTO-SELECTED when dlio.service or a 'dlio' container is up.
#
# NO MAPPER PANE. mighty_node subscribes to <ns>/occ_2d_topic + <ns>/esdf_2d_topic
# (mighty_node.cpp:311-321, SensorDataQoS) and does not care who publishes them.
# Those used to come from global_mapper_ros, which is NOT in mighty-hw:local —
# the acl-mapping COPY/build is commented out in Dockerfile.hw, so launching it
# here only ever produced a "package not found" pane. They now come from
# elevation_mapping_cupy on the OX08 Orin (192.168.12.1), which needs a CUDA GPU
# this NUC does not have. To go back to the onboard mapper, uncomment the
# acl-mapping block in Dockerfile.hw, rebuild, and re-add a mapper pane.
#
# PREREQS in EVERY mode (this stack launches NO sensors, NO router, NO mapper):
#   drive.service    hosts the zenoh router on :7447
#   sensors.service  livox + D455 + the base_link->lidar static TF
#   OX08 Orin        elevation_mapping_cupy -> <ns>/occ_2d_topic, <ns>/esdf_2d_topic
#                    (without it MIGHTY runs but never plans: no occupancy grid)
# dlio.service and the DLIO panes here are THE SAME NODES — never run both. When
# dlio.service (or a container named 'dlio') is up, `start` auto-selects
# --odom-type external and launches no odometry panes; an explicit --odom-type
# dlio/dlio_in_mocap is then REFUSED rather than silently double-published. On a
# rover where dlio.service is inactive nothing changes — the default is still dlio.
#
# NOTE: DLIO runs a ~3 s stationary IMU calibration whenever its pane (re)starts
# — keep the rover still. Identity (ROBOT_NAME, RMW, optional DLIO_SEED_Z) comes
# from /etc/rover/rover.env via compose env_file; nothing in this directory is
# rover-specific.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=hw-mighty
SESSION=hw_mighty
COMPOSE=(docker compose -f "${SCRIPT_DIR}/compose.hw.yaml")

# In-container command preludes. Pane commands are sent single-quoted, so $VARs
# in them expand INSIDE the container (env_file provides ROBOT_NAME etc.) — and
# therefore a pane command must never contain a single quote. Every pane
# spin-waits on the router first; the guard shares the pane's history line so
# Ctrl-C -> Up -> Enter re-runs it too.
SETUP='source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash'
DECOMP='source /home/swarm/code/decomp_ws/install/setup.bash'
WAIT_ROUTER='until (echo >/dev/tcp/127.0.0.1/7447) 2>/dev/null; do echo "waiting for zenoh router (drive.service)..."; sleep 2; done'

dx() { echo "docker exec -it ${CONTAINER} bash -c '${SETUP} && ${WAIT_ROUTER} && $1'"; }

attach() { exec tmux attach -t "${SESSION}"; }

usage() {
    echo "usage: $0 {start [--odom-type dlio|dlio_in_mocap|mocap|external] [--two-d-only|--no-two-d] | attach | stop | status | logs | rebuild [start flags]}" >&2
    exit 2
}

start() {
    local odom_type='' two_d=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --odom-type)  odom_type="${2:?--odom-type needs a value}"; shift ;;
            --two-d-only) two_d=true ;;
            --no-two-d)   two_d=false ;;
            *) echo "[mighty_hw] unknown option: $1" >&2; usage ;;
        esac
        shift
    done

    # Who owns DLIO on this rover? dlio.service (dlio_ws) runs the SAME nodes as
    # the DLIO panes below. Detect the live state instead of hardcoding a
    # per-rover default, so this directory stays rover-agnostic and a rover
    # without the service behaves exactly as before. Note `grep -c`, not
    # `grep -q`: under `set -o pipefail` an early-exiting grep -q SIGPIPEs the
    # upstream docker ps and the pipeline returns 141 ON MATCH.
    local dlio_owned=false
    if systemctl is-active --quiet dlio.service 2>/dev/null; then
        dlio_owned=true
    elif docker ps --format '{{.Names}}' 2>/dev/null | grep -cx dlio >/dev/null; then
        dlio_owned=true
    fi

    if [[ -z "${odom_type}" ]]; then
        if [[ "${dlio_owned}" == true ]]; then
            odom_type=external
            echo "[mighty_hw] dlio.service owns DLIO on this rover — starting with --odom-type external"
        else
            odom_type=dlio
        fi
    fi

    case "${odom_type}" in
        dlio|dlio_in_mocap|mocap|external) ;;
        *) echo "[mighty_hw] bad --odom-type '${odom_type}'" >&2; usage ;;
    esac

    # Inverse guard: an EXPLICIT dlio mode while the service owns DLIO would put a
    # second dlio_odom_node in the namespace, double-publishing dlio/odom_node/*
    # and odom->base_link — MIGHTY's state estimate would flip between them.
    if [[ "${dlio_owned}" == true ]] \
       && [[ "${odom_type}" == dlio || "${odom_type}" == dlio_in_mocap ]]; then
        echo "[mighty_hw] REFUSING --odom-type ${odom_type}: dlio.service (or a 'dlio' container)" \
             "is already running the same nodes. Stop it first (sudo systemctl stop dlio)," \
             "or use --odom-type external." >&2
        exit 1
    fi

    # Fail fast on the hard prereq, warn on the soft one — the pane-side
    # spin-waits still guard every node, this is just early readable feedback.
    if ! (echo >/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
        echo "[mighty_hw] no zenoh router on 127.0.0.1:7447 — start drive.service first" >&2
        exit 1
    fi
    if ! docker ps --format '{{.Names}}' | grep -cx sensors >/dev/null; then
        echo "[mighty_hw] WARNING: no 'sensors' container (sensors.service down?) — livox/DLIO/mapper will sit idle" >&2
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
    robot_name="$(docker exec "${CONTAINER}" printenv ROBOT_NAME)"

    # ---- per-mode node commands (mind the single-quote rule above) ----------
    # Consumers gate on the TFs they need via wait_for_tf.py (the /tf_static
    # startup-race fix): the static publishers below latch one transient-local
    # sample, and a subscriber that forms too early would never receive it.
    local mighty_cmd
    if [[ "${odom_type}" == mocap ]]; then
        mighty_cmd="${DECOMP}"' && ros2 run mighty wait_for_tf.py $ROBOT_NAME/map $ROBOT_NAME/odom && ros2 launch mighty onboard_mighty.launch.py x:=0.0 y:=0.0 z:=0.0 yaw:=0.0 namespace:=$ROBOT_NAME use_hardware:=true use_onboard_localization:=false robot_type:=red_rover depth_camera_name:=d455 twist_topic:=mocap/twist'
    else  # dlio | dlio_in_mocap — identical mighty; only the seed differs
        mighty_cmd="${DECOMP}"' && ros2 run mighty wait_for_tf.py $ROBOT_NAME/map $ROBOT_NAME/odom && ros2 launch mighty onboard_mighty.launch.py x:=0.0 y:=0.0 z:=0.0 yaw:=0.0 namespace:=$ROBOT_NAME use_hardware:=true use_onboard_localization:=true robot_type:=red_rover depth_camera_name:=d455'
    fi
    local dlio_cmd='ros2 launch direct_lidar_inertial_odometry dlio.launch.py namespace:=$ROBOT_NAME initial_pose_topic:=world two_d_only:='"${two_d}"
    local seed_cmd='ros2 topic pub -r 2 /$ROBOT_NAME/world geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {z: ${DLIO_SEED_Z:-0.4}}, orientation: {w: 1}}}"'
    local tf_map_odom='ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 $ROBOT_NAME/map $ROBOT_NAME/odom'
    local tf_mocap_base='ros2 run tf2_ros static_transform_publisher --frame-id $ROBOT_NAME --child-frame-id $ROBOT_NAME/base_link'
    local tf_world_map='ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id $ROBOT_NAME/map'

    # titles[i] LABELS cmds[i] — the two arrays are positional, so dropping an
    # entry from one and not the other silently mislabels every pane after it
    # (that is how the mapper removal once left pane 0 titled "Onboard MIGHTY"
    # while running the goal republisher, and MIGHTY never started at all).
    # The length check below turns any future mismatch into a startup error.
    local -a titles cmds
    titles=('Onboard MIGHTY' 'RViz 2D goal')
    cmds=(
        "$(dx "${mighty_cmd}")"
        "$(dx 'ros2 run mighty repub_rviz_2Dgoal.py')"
    )
    case "${odom_type}" in
        dlio)
            titles+=('DLIO (seed-anchored)' 'seed pose' 'tf map->odom')
            cmds+=("$(dx "${dlio_cmd}")" "$(dx "${seed_cmd}")" "$(dx "${tf_map_odom}")")
            ;;
        dlio_in_mocap)
            titles+=('DLIO (mocap-seeded)' 'tf map->odom')
            cmds+=("$(dx "${dlio_cmd}")" "$(dx "${tf_map_odom}")")
            ;;
        mocap)
            titles+=('tf mocap->base_link' 'tf world->map' 'tf map->odom')
            cmds+=("$(dx "${tf_mocap_base}")" "$(dx "${tf_world_map}")" "$(dx "${tf_map_odom}")")
            ;;
        external)
            # Nothing to add: dlio.service publishes DLIO, the seed pose and
            # map->odom. MIGHTY (pane 0) still gates on wait_for_tf.py for
            # <ns>/map -> <ns>/odom, which that service provides.
            if [[ "${two_d}" == true ]]; then
                echo "[mighty_hw] NOTE: --two-d-only is ignored with --odom-type external —" \
                     "two_d_only belongs to dlio.service (DLIO_TWO_D_ONLY, or its own" \
                     "--two-d-only flag)." >&2
            fi
            ;;
    esac
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

    echo "[mighty_hw] ${SESSION} session up (rover: ${robot_name}, odom: ${odom_type}, two_d_only: ${two_d})"
    if [[ -t 0 && -t 1 ]]; then
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
        # the container itself goes away.
        tmux kill-session -t "${SESSION}" 2>/dev/null || true
        exec "${COMPOSE[@]}" down
        ;;
    status)
        docker ps --filter "name=^${CONTAINER}$" --format 'container: {{.Names}}  {{.Status}}' | grep . \
            || { echo "container: not running"; exit 1; }
        tmux list-panes -t "${SESSION}" \
            -F 'pane #{pane_index}  #{pane_title}  (#{pane_current_command})' 2>/dev/null \
            || echo "no ${SESSION} tmux session on the host"
        ;;
    logs)
        exec docker logs "$@" "${CONTAINER}"
        ;;
    rebuild)
        "$0" stop || true
        "${COMPOSE[@]}" build
        exec "$0" start "$@"
        ;;
    *)
        usage
        ;;
esac
