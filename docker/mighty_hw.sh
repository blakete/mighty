#!/usr/bin/env bash
# mighty_hw.sh — SINGLE entrypoint for the containerized MIGHTY hardware stack.
# The hw-mighty container just idles (`sleep infinity`, see compose.hw.yaml);
# every node runs as a `docker exec` whose controlling pane lives in a HOST tmux
# session 'hw_mighty', built right here in bash.
#
#   mighty_hw.sh start [--dev]     # bring the container up + build the session
#   mighty_hw.sh start --monitor   # same, then hold the foreground while the session
#                                  # lives (mighty.service's main process, see mighty.service.in)
#   mighty_hw.sh check [--dev]     # read-only preflight: env, image, router, topics, TF
#   mighty_hw.sh attach            # tmux attach (Ctrl-b d detaches; stack keeps running)
#   mighty_hw.sh stop              # kill session, then compose down
#   mighty_hw.sh status            # container + pane status
#   mighty_hw.sh logs [...]        # container PID-1 output (idle loop; panes hold the real logs)
#   mighty_hw.sh pull [<tag>]      # registry <tag> (default latest) -> mighty-hw:local
#   mighty_hw.sh rebuild [...]     # stop + compose build + start (start flags forwarded)
#
# CONFIGURATION — two env files, both passed to the container (compose env_file;
# the later file wins):
#   $ROVER_ENV_FILE   default /etc/rover/rover.env    identity: ROBOT_NAME, RMW_IMPLEMENTATION,
#                                                     ROS_DOMAIN_ID (the RR fleet's provisioning
#                                                     owns this file)
#   $MIGHTY_ENV_FILE  default /etc/mighty/mighty.env  everything MIGHTY-specific (written by
#                                                     install_service.sh; ROBOT_NAME may go here
#                                                     instead on a vehicle without rover.env)
# MIGHTY_ keys (defaults in brackets):
#   MIGHTY_PLATFORM             ground_robot | uav                       [ground_robot]
#   MIGHTY_STATE_SOURCE         odom | mocap                             [odom]
#   MIGHTY_ODOM_TOPIC           nav_msgs/Odometry, relative to <ns>      [odom]
#   MIGHTY_POSE_TOPIC           MPC pose / mocap pose input              [pose (odom) | world (mocap)]
#   MIGHTY_TWIST_TOPIC          mocap twist                              [twist]
#   MIGHTY_TF_GATE              TARGET SOURCE pairs the planner and MPC wait for, or "none"
#                                                                        [<ns>/map <ns>/odom on ground; none on uav]
#   MIGHTY_PUBLISH_MAP_ODOM_TF  true: extra pane broadcasting identity <ns>/map -> <ns>/odom
#                               (only where nothing else publishes it)   [false]
#   MIGHTY_VEHICLE_CONFIG       parameter overlay: a file in config/vehicles/ [<ns>.yaml if present]
# Host-side keys (read from $MIGHTY_ENV_FILE by this script before compose up):
#   ROVER_CONFIG_DIR            directory mounted at /home/swarm/config: the zenoh session
#                               config (zenoh_session_config.json5)      [/home/swarm/config]
#   ZENOH_ROUTER_PORT           the host zenoh router (rmw_zenoh only)    [7447]
#
# PARAMETERS come from this checkout's config/ (bind-mounted over the image's
# copy, see compose.hw.yaml): edit the YAML, Ctrl-C / Up / Enter the pane. No
# rebuild. Code and launch files are baked and need one.
#
# PANES: MIGHTY planner | <state adapter> | MPC (ground_robot) | map->odom TF (opt-in).
# Each is mighty_hw.launch.py with only_nodes:= one node, so Ctrl-C -> Up -> Enter
# restarts ONE node. The launch file still computes every parameter (never
# hand-roll them as ros2 run).
#
# The script is host-side while the launch file is baked into the image, so the
# two can drift, and ros2 launch SILENTLY IGNORES an argument it does not
# declare. start() therefore refuses an image whose launch file lacks platform:=.
#
# --dev (laptop): identity from dev/rover.env (RR99) + dev/mighty.env, the dev/
# zenoh configs, and an ISOLATED zenoh router on 127.0.0.1:7448 started from the
# same image (compose profile 'dev'). It dials nothing: a dev stack must not
# join a fleet. A pre-set ROVER_ENV_FILE / MIGHTY_ENV_FILE wins (bag replay runs
# the dev stack under the recording rover's name).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=hw-mighty
SESSION=hw_mighty
IMAGE=mighty-hw:local
REGISTRY="${MIGHTY_REGISTRY:-registry.gitlab.com/mit-acl/ugv/redrover/rover/mighty-hw}"
COMPOSE=(docker compose -f "${SCRIPT_DIR}/compose.hw.yaml")

# In-container setup. Pane commands are sent single-quoted, so $VARs in them
# expand INSIDE the container — and therefore a pane command must never contain
# a single quote (settings are validated by safe_value below).
SETUP='source /opt/ros/humble/setup.bash && source ${MIGHTY_WS:-/ws}/install/setup.bash'

attach() { exec tmux attach -t "${SESSION}"; }

usage() {
    echo "usage: $0 {start [--dev|--monitor] | check [--dev] | attach | stop | status | logs | pull [<tag>] | rebuild [start flags]}" >&2
    exit 2
}

# ---- env files --------------------------------------------------------------
# Last assignment of KEY in an env file (compose env_file syntax: KEY=VALUE, no
# interpolation, optional quotes, # comments). Empty if the file or key is absent.
env_get() {
    local file="$1" key="$2" v
    [[ -r "${file}" ]] || return 0
    v="$(sed -n -E "s/^[[:space:]]*(export[[:space:]]+)?${key}=(.*)$/\2/p" "${file}" | tail -n 1)"
    if [[ "${v}" =~ ^\"(.*)\"$ || "${v}" =~ ^\'(.*)\'$ ]]; then
        v="${BASH_REMATCH[1]}"
    else
        v="$(sed -E 's/[[:space:]]+#.*$//; s/[[:space:]]+$//' <<<"${v}")"
    fi
    printf '%s' "${v}"
}
# KEY as the container sees it: $MIGHTY_ENV_FILE wins over $ROVER_ENV_FILE.
setting() {
    local v
    v="$(env_get "${MIGHTY_ENV_FILE}" "$1")"
    [[ -n "${v}" ]] || v="$(env_get "${ROVER_ENV_FILE}" "$1")"
    printf '%s' "${v:-${2:-}}"
}
# Values end up inside single-quoted pane commands and on launch command lines.
safe_value() {
    [[ "$2" =~ ^[A-Za-z0-9_./:-]*$ ]] || {
        echo "[mighty_hw] $1='$2' may only contain letters, digits and _ . / : -" >&2
        exit 1
    }
}

# Resolve every setting into globals (shared by start and check).
resolve() {
    local dev="$1"
    if [[ "${dev}" == true ]]; then
        export ROVER_ENV_FILE="${ROVER_ENV_FILE:-${SCRIPT_DIR}/dev/rover.env}"
        export MIGHTY_ENV_FILE="${MIGHTY_ENV_FILE:-${SCRIPT_DIR}/dev/mighty.env}"
        export ROVER_CONFIG_DIR="${SCRIPT_DIR}/dev"
        ZENOH_ROUTER_PORT=7448
        COMPOSE+=(--profile dev)
    else
        export ROVER_ENV_FILE="${ROVER_ENV_FILE:-/etc/rover/rover.env}"
        export MIGHTY_ENV_FILE="${MIGHTY_ENV_FILE:-/etc/mighty/mighty.env}"
        export ROVER_CONFIG_DIR="${ROVER_CONFIG_DIR:-$(setting ROVER_CONFIG_DIR /home/swarm/config)}"
        ZENOH_ROUTER_PORT="${ZENOH_ROUTER_PORT:-$(setting ZENOH_ROUTER_PORT 7447)}"
    fi
    ROBOT_NAME="$(setting ROBOT_NAME)"
    RMW="$(setting RMW_IMPLEMENTATION)"
    PLATFORM="$(setting MIGHTY_PLATFORM ground_robot)"
    STATE_SOURCE="$(setting MIGHTY_STATE_SOURCE odom)"
    ODOM_TOPIC="$(setting MIGHTY_ODOM_TOPIC odom)"
    POSE_TOPIC="$(setting MIGHTY_POSE_TOPIC)"
    TWIST_TOPIC="$(setting MIGHTY_TWIST_TOPIC twist)"
    PUBLISH_MAP_ODOM_TF="$(setting MIGHTY_PUBLISH_MAP_ODOM_TF false)"
    VEHICLE_CONFIG="$(setting MIGHTY_VEHICLE_CONFIG)"
    TF_GATE="$(setting MIGHTY_TF_GATE)"
    if [[ -z "${TF_GATE}" ]]; then
        [[ "${PLATFORM}" == ground_robot ]] && TF_GATE="${ROBOT_NAME}/map ${ROBOT_NAME}/odom" || TF_GATE=none
    fi
    local k
    for k in ROBOT_NAME PLATFORM STATE_SOURCE ODOM_TOPIC POSE_TOPIC TWIST_TOPIC VEHICLE_CONFIG ZENOH_ROUTER_PORT; do
        safe_value "${k}" "${!k}"
    done
    for k in ${TF_GATE}; do safe_value MIGHTY_TF_GATE "${k}"; done
    ZENOH=false
    [[ "${RMW}" == rmw_zenoh_cpp ]] && ZENOH=true
    return 0
}

print_settings() {
    cat <<EOF
[mighty_hw] identity   ROBOT_NAME=${ROBOT_NAME:-<unset>}  RMW=${RMW:-<default>}  (${ROVER_ENV_FILE}, ${MIGHTY_ENV_FILE})
[mighty_hw] platform   ${PLATFORM}  state_source=${STATE_SOURCE}  odom_topic=${ODOM_TOPIC}  pose_topic=${POSE_TOPIC:-<derived>}  twist_topic=${TWIST_TOPIC}
[mighty_hw] tf gate    ${TF_GATE}   map->odom pane=${PUBLISH_MAP_ODOM_TF}   vehicle_config=${VEHICLE_CONFIG:-<auto>}
EOF
}

wait_router() {
    echo "until (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; do echo \"waiting for zenoh router on :${ZENOH_ROUTER_PORT}...\"; sleep 2; done"
}
# One pane command: docker exec into the container, set up ROS, (zenoh) wait for
# the router — the wait shares the pane's history line, so Up/Enter re-runs it.
dx() {
    local pre="${SETUP}"
    [[ "${ZENOH}" == true ]] && pre="${pre} && $(wait_router)"
    echo "docker exec -it ${CONTAINER} bash -c '${pre} && $1'"
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
    resolve "${dev}"
    print_settings

    if [[ -z "${ROBOT_NAME}" ]]; then
        echo "[mighty_hw] ROBOT_NAME is not set in ${ROVER_ENV_FILE} or ${MIGHTY_ENV_FILE}" \
             "(or use --dev on a laptop)" >&2
        exit 1
    fi
    case "${PLATFORM}" in ground_robot|uav) ;; *)
        echo "[mighty_hw] MIGHTY_PLATFORM=${PLATFORM}: must be ground_robot or uav" >&2; exit 1 ;; esac
    case "${STATE_SOURCE}" in odom|mocap) ;; *)
        echo "[mighty_hw] MIGHTY_STATE_SOURCE=${STATE_SOURCE}: must be odom or mocap" >&2; exit 1 ;; esac
    if [[ "${dev}" != true && ! -d "${ROVER_CONFIG_DIR}" ]]; then
        echo "[mighty_hw] config dir ${ROVER_CONFIG_DIR} does not exist — set ROVER_CONFIG_DIR in" \
             "${MIGHTY_ENV_FILE} (install_service.sh creates one)" >&2
        exit 1
    fi

    # The router is the one hard prereq (rmw_zenoh only; the panes spin-wait on
    # it too — this is early, readable feedback). Under --monitor (systemd) it is
    # a WAIT, not an exit: at boot the unit's After= ordering is satisfied as soon
    # as the service hosting the router starts, before zenohd listens, so failing
    # here would fail the unit on every cold boot.
    if [[ "${dev}" != true && "${ZENOH}" == true ]] \
       && ! (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; then
        if [[ "${monitor}" == true ]]; then
            echo "[mighty_hw] waiting for the zenoh router on 127.0.0.1:${ZENOH_ROUTER_PORT}..."
            until (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; do sleep 2; done
        else
            echo "[mighty_hw] no zenoh router on 127.0.0.1:${ZENOH_ROUTER_PORT} — start it first" \
                 "(on the RR fleet: drive.service)" >&2
            exit 1
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
    # The container got its environment from compose, this script from its own
    # reading of the same files: refuse to run if they disagree.
    local seen
    seen="$(docker exec "${CONTAINER}" printenv ROBOT_NAME || true)"
    if [[ "${seen}" != "${ROBOT_NAME}" ]]; then
        echo "[mighty_hw] the container sees ROBOT_NAME='${seen}', this script read '${ROBOT_NAME}'" \
             "— check ${ROVER_ENV_FILE} and ${MIGHTY_ENV_FILE}" >&2
        "${COMPOSE[@]}" down
        exit 1
    fi

    # Refuse an image whose baked launch file predates mighty_hw.launch.py (see header).
    if ! docker exec "${CONTAINER}" bash -c \
            "${SETUP} && ros2 launch mighty mighty_hw.launch.py --show-args 2>/dev/null \
             | grep -c \"'platform':\"" >/dev/null 2>&1; then
        echo "[mighty_hw] this image has no mighty_hw.launch.py with a platform:= argument" \
             "(it predates this checkout). Pull or rebuild the image first." >&2
        "${COMPOSE[@]}" down >/dev/null 2>&1 || true
        exit 1
    fi

    # ---- pane commands (mind the single-quote rule above) --------------------
    # mighty_node and mpc resolve <ns>/map, so they gate on wait_for_tf.py. The
    # state adapter is a pure sub->pub relay and never touches TF. wait_for_tf.py
    # exits 0 on timeout (60 s): the stack degrades loudly instead of deadlocking.
    local gate="" launch adapter
    [[ "${TF_GATE}" != none ]] && gate="ros2 run mighty wait_for_tf.py ${TF_GATE} && "
    launch="ros2 launch mighty mighty_hw.launch.py namespace:=${ROBOT_NAME} platform:=${PLATFORM} state_source:=${STATE_SOURCE}"
    [[ "${STATE_SOURCE}" == odom ]] && launch+=" odom_topic:=${ODOM_TOPIC}" || launch+=" twist_topic:=${TWIST_TOPIC}"
    [[ -n "${POSE_TOPIC}" ]] && launch+=" pose_topic:=${POSE_TOPIC}"
    [[ -n "${VEHICLE_CONFIG}" ]] && launch+=" vehicle_config:=${VEHICLE_CONFIG}"
    [[ "${STATE_SOURCE}" == odom ]] && adapter=convert_odom_to_state || adapter=convert_vicon_to_state

    # titles[i] LABELS cmds[i]; the length check below keeps them in step.
    # NOTE: restarting the MPC pane is not instant — MPCNode builds an
    # IPOPT/collocation NLP before its first control tick.
    local -a titles cmds
    titles=('MIGHTY planner' "${adapter}")
    cmds=("$(dx "${gate}${launch} only_nodes:=mighty_node")"
          "$(dx "${launch} only_nodes:=${adapter}")")
    if [[ "${PLATFORM}" == ground_robot ]]; then
        titles+=('MPC')
        cmds+=("$(dx "${gate}${launch} only_nodes:=mpc")")
    fi
    if [[ "${PUBLISH_MAP_ODOM_TF}" == true ]]; then
        titles+=('map->odom TF')
        cmds+=("$(dx "ros2 run mighty map_odom_tf.py ${ROBOT_NAME}")")
    fi
    if (( ${#titles[@]} != ${#cmds[@]} )); then
        echo "[mighty_hw] BUG: ${#titles[@]} pane titles but ${#cmds[@]} commands" >&2
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

    echo "[mighty_hw] ${SESSION} session up (${ROBOT_NAME}, ${PLATFORM}, ${#cmds[@]} panes)"
    if [[ "${monitor}" == true ]]; then
        # systemd main process: live exactly as long as the session does, so
        # `tmux kill-session -t hw_mighty` deactivates mighty.service and its
        # ExecStopPost runs `stop`.
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

# ---- check: read-only preflight -------------------------------------------------
check() {
    local dev=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --dev) dev=true ;;
            *) echo "[mighty_hw] unknown option: $1" >&2; usage ;;
        esac
        shift
    done
    local fails=0 warns=0
    ok()   { printf '  PASS  %s\n' "$*"; }
    warn() { printf '  WARN  %s\n' "$*"; warns=$((warns + 1)); }
    bad()  { printf '  FAIL  %s\n' "$*"; fails=$((fails + 1)); }

    echo "== host"
    if command -v docker >/dev/null && docker info >/dev/null 2>&1; then ok "docker $(docker version -f '{{.Server.Version}}' 2>/dev/null)"
    else bad "docker is not installed, not running, or this user is not in the docker group"; fi
    if docker compose version >/dev/null 2>&1; then ok "$(docker compose version | head -n 1)"
    else bad "docker compose v2 plugin missing"; fi
    command -v tmux >/dev/null && ok "tmux" || bad "tmux missing (the panes are a host tmux session)"

    echo "== settings"
    resolve "${dev}"
    [[ -r "${ROVER_ENV_FILE}" ]] && ok "identity file ${ROVER_ENV_FILE}" \
        || warn "no ${ROVER_ENV_FILE} (fine if ROBOT_NAME/RMW are in ${MIGHTY_ENV_FILE})"
    [[ -r "${MIGHTY_ENV_FILE}" ]] && ok "settings file ${MIGHTY_ENV_FILE}" \
        || warn "no ${MIGHTY_ENV_FILE}: every MIGHTY_ setting is at its default (install_service.sh writes it)"
    print_settings | sed 's/^\[mighty_hw\]/       /'
    if [[ -z "${ROBOT_NAME}" ]]; then bad "ROBOT_NAME is not set"
    elif [[ ! "${ROBOT_NAME}" =~ [0-9]$ ]]; then
        warn "ROBOT_NAME=${ROBOT_NAME} does not end in digits: set agent_id in its vehicle config (mighty_node: ros__parameters: agent_id: N)"
    else ok "ROBOT_NAME=${ROBOT_NAME}"; fi
    [[ "${PLATFORM}" =~ ^(ground_robot|uav)$ ]] && ok "platform ${PLATFORM}" || bad "MIGHTY_PLATFORM=${PLATFORM}: must be ground_robot or uav"
    [[ "${STATE_SOURCE}" =~ ^(odom|mocap)$ ]] && ok "state source ${STATE_SOURCE}" || bad "MIGHTY_STATE_SOURCE=${STATE_SOURCE}: must be odom or mocap"
    local vcfg="${SCRIPT_DIR}/../config/vehicles/${VEHICLE_CONFIG:-${ROBOT_NAME}.yaml}"
    [[ "${VEHICLE_CONFIG}" == /* ]] && vcfg="${VEHICLE_CONFIG}"   # absolute = a container path
    [[ -f "${vcfg}" ]] && ok "vehicle overlay $(basename "${vcfg}")" || ok "no vehicle overlay (platform defaults)"
    if [[ "${dev}" != true ]]; then
        if [[ ! -d "${ROVER_CONFIG_DIR}" ]]; then bad "config dir ${ROVER_CONFIG_DIR} missing (ROVER_CONFIG_DIR)"
        elif [[ "${ZENOH}" == true && ! -r "${ROVER_CONFIG_DIR}/zenoh_session_config.json5" ]]; then
            bad "rmw_zenoh without ${ROVER_CONFIG_DIR}/zenoh_session_config.json5"
        else ok "config dir ${ROVER_CONFIG_DIR}"; fi
    fi

    echo "== image"
    local image_ok=false
    if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
        ok "${IMAGE} ($(docker image inspect -f '{{.Id}}' "${IMAGE}" | cut -c8-19))"
        if docker run --rm --entrypoint bash "${IMAGE}" -c "${SETUP} && ros2 launch mighty mighty_hw.launch.py --show-args 2>/dev/null | grep -c \"'platform':\"" >/dev/null 2>&1; then
            ok "image matches this checkout (mighty_hw.launch.py with platform:=)"; image_ok=true
        else bad "image predates this checkout (no mighty_hw.launch.py platform:=) — pull or rebuild"; fi
    else bad "no ${IMAGE} — mighty_hw.sh pull <tag>, or make hw-build"; fi

    echo "== ROS graph"
    if [[ "${ZENOH}" == true ]]; then
        if (echo >/dev/tcp/127.0.0.1/${ZENOH_ROUTER_PORT}) 2>/dev/null; then ok "zenoh router on :${ZENOH_ROUTER_PORT}"
        else bad "no zenoh router on 127.0.0.1:${ZENOH_ROUTER_PORT}"; image_ok=false; fi
    fi
    local stack_up=0
    [[ -n "$(docker ps -q --filter "name=^${CONTAINER}$")" ]] && stack_up=1
    if [[ "${image_ok}" == true && -n "${ROBOT_NAME}" ]]; then
        local -a run
        if (( stack_up )); then
            run=(docker exec -i "${CONTAINER}")
        else
            run=(docker run --rm -i --network host
                 -e ZENOH_SESSION_CONFIG_URI=/home/swarm/config/zenoh_session_config.json5
                 -v "${ROVER_CONFIG_DIR}:/home/swarm/config:ro")
            [[ -r "${ROVER_ENV_FILE}" ]] && run+=(--env-file "${ROVER_ENV_FILE}")
            [[ -r "${MIGHTY_ENV_FILE}" ]] && run+=(--env-file "${MIGHTY_ENV_FILE}")
            run+=("${IMAGE}")
        fi
        local out
        out="$("${run[@]}" bash -c "${SETUP} && NS=${ROBOT_NAME} PLATFORM=${PLATFORM} STATE_SOURCE=${STATE_SOURCE} ODOM_TOPIC=${ODOM_TOPIC} POSE_TOPIC=${POSE_TOPIC} TWIST_TOPIC=${TWIST_TOPIC} TF_GATE='${TF_GATE}' MAP_ODOM_PANE=${PUBLISH_MAP_ODOM_TF} STACK_UP=${stack_up} python3 -" \
               < "${SCRIPT_DIR}/check_graph.py" 2>&1)" || true
        local line
        while IFS= read -r line; do
            case "${line}" in
                PASS\ *) ok "${line#PASS }" ;;
                WARN\ *) warn "${line#WARN }" ;;
                FAIL\ *) bad "${line#FAIL }" ;;
                *) [[ -n "${line}" ]] && printf '        %s\n' "${line}" ;;
            esac
        done <<<"${out}"
    else
        warn "skipped (needs the image and ROBOT_NAME)"
    fi

    # Every container on a vehicle must run the same rmw_zenoh build.
    if [[ "${ZENOH}" == true && "${image_ok}" == true ]]; then
        echo "== rmw_zenoh build"
        local mine other c
        mine="$(docker run --rm --entrypoint dpkg-query "${IMAGE}" -W -f='${Version}' ros-humble-rmw-zenoh-cpp 2>/dev/null || true)"
        ok "${IMAGE}: ${mine:-?}"
        for c in $(docker ps --format '{{.Names}}'); do
            [[ "${c}" == "${CONTAINER}" ]] && continue
            # Containers without ROS (or without rmw_zenoh) are none of our business.
            other="$(docker exec "${c}" dpkg-query -W -f='${Version}' ros-humble-rmw-zenoh-cpp 2>/dev/null)" || continue
            [[ "${other}" =~ ^[0-9] ]] || continue
            [[ "${other}" == "${mine}" ]] && ok "${c}: ${other}" \
                || warn "${c}: ${other} differs from ${IMAGE} — TRANSIENT_LOCAL topics may not cross"
        done
    fi

    echo "== ${fails} FAIL, ${warns} WARN"
    (( fails == 0 ))
}

cmd="${1:-start}"
[[ $# -gt 0 ]] && shift

case "${cmd}" in
    start)
        start "$@"
        ;;
    check)
        check "$@"
        ;;
    attach)
        attach
        ;;
    stop)
        # Session first: closing the panes HUPs the docker-exec'd nodes before
        # the container itself goes away. --profile dev so a laptop's router
        # container is removed too (no-op on a vehicle: nothing in that profile
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
