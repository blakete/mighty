#!/usr/bin/env bash
# goal_session.sh — click goals in RViz for a LOCAL MIGHTY that replans on a
# replayed bag. Laptop only; nothing here reaches a vehicle or a fleet graph.
#
#   goal_session.sh up <bag dir> [--loop] [--rviz-config FILE]
#   goal_session.sh down
#
# 1. MIGHTY --dev (docker/mighty_hw.sh) under the bag's vehicle name, on its
#    ISOLATED zenoh router 127.0.0.1:7448 (loopback only, dials nothing).
# 2. RViz from the containerized-rviz image, on the host network, with its zenoh
#    session pointed at that dev router ONLY (docker/dev/zenoh_session_config.json5).
#    The image's own entrypoint would otherwise connect to whatever listens on
#    127.0.0.1:7447 — on a C2 laptop that is the fleet router — so it is bypassed.
# 3. The bag's planner INPUTS (TF, odometry, the three 2D grids, the deskewed
#    cloud for context) played inside the RViz container through
#    restamp_relay.py --hold: re-stamped onto the wall clock (MIGHTY runs on
#    wall time, as on the vehicles), and after the bag ends the last pose, TF
#    and maps keep being republished, so the vehicle "parks" there. With --loop
#    the bag repeats instead (the relay keeps time moving forward; the vehicle
#    jumps back to its start pose each loop). The bag's recorded planner
#    OUTPUTS are not played; everything planner-side you see comes from the
#    local MIGHTY.
#
# Then use the 2D Goal tool (it publishes /<vehicle>/term_goal) and watch MIGHTY
# replan: tmux attach -t hw_mighty for its panes.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(cd "${HERE}/../.." && pwd)"
NAME=mighty-goal-rviz
CRVIZ_IMAGE="${CRVIZ_IMAGE:-ghcr.io/blakete/containerized-rviz:latest}"
CRVIZ_REPO="${CRVIZ_REPO:-${HOME}/repos/containerized-rviz}"
STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/mighty-goal-session"
die() { echo "[goal_session] $*" >&2; exit 1; }
say() { echo "[goal_session] $*"; }

down() {
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    "${DOCKER_DIR}/mighty_hw.sh" stop >/dev/null 2>&1 || true
    say "down"
}

up() {
    local bag="${1:?usage: goal_session.sh up <bag dir> [--loop] [--rviz-config FILE]}"; shift
    local rviz_cfg="" loop=""
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --rviz-config) rviz_cfg="$(realpath "$2")"; shift ;;
            --loop) loop="--loop" ;;
            *) die "unknown option: $1" ;;
        esac
        shift
    done
    bag="$(realpath "${bag}")"
    [[ -f "${bag}/metadata.yaml" ]] || die "${bag} is not a rosbag2 directory"
    docker image inspect "${CRVIZ_IMAGE}" >/dev/null 2>&1 || die "no ${CRVIZ_IMAGE} (crviz pull)"

    # ---- what is in the bag: the vehicle name and the input topics present
    local info vehicle
    info="$(python3 - "${bag}/metadata.yaml" <<'PY'
import re, sys, yaml
m = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
topics = {t['topic_metadata']['name']: t['topic_metadata']['type']
          for t in m['topics_with_message_count'] if t['message_count'] > 0}
names = sorted({mm.group(1) for t in topics for mm in [re.match(r'^/([A-Z]{2}\d{2})/', t)] if mm})
v = names[0] if names else ''
want = ['/tf', '/tf_static'] + [f'/{v}/{s}' for s in (
    'dlio/odom_node/odom', 'dlio/odom_node/pose', 'occ_2d_topic', 'esdf_2d_topic',
    'planning_occ_2d_topic', 'dlio/odom_node/pointcloud/deskewed')]
print(v)
print(' '.join(f'{t}={topics[t]}' for t in want if t in topics))
print(f"{m['duration']['nanoseconds'] / 1e9:.1f}")
PY
)"
    vehicle="$(sed -n 1p <<<"${info}")"
    local pairs duration
    pairs="$(sed -n 2p <<<"${info}")"
    duration="$(sed -n 3p <<<"${info}")"
    [[ -n "${vehicle}" ]] || die "no /XXNN/ vehicle topics in ${bag}"
    grep -q "planning_occ_2d_topic=" <<<"${pairs}" || die "${bag} has no ${vehicle}/planning_occ_2d_topic: nothing to plan on"
    say "bag $(basename "${bag}"): ${vehicle}, ${duration} s; replaying $(wc -w <<<"${pairs}") input topics"

    # ---- identity for the dev stack and the RViz container (same domain!)
    mkdir -p "${STATE_DIR}"
    local envf="${STATE_DIR}/rover.env"
    printf 'ROBOT_NAME=%s\nROS_DOMAIN_ID=22\nRMW_IMPLEMENTATION=rmw_zenoh_cpp\n' "${vehicle}" > "${envf}"

    # ---- RViz layout: crviz's per-vehicle layout if there is one, else ours
    if [[ -z "${rviz_cfg}" ]]; then
        local lower; lower="$(tr '[:upper:]' '[:lower:]' <<<"${vehicle}")"
        if [[ -f "${CRVIZ_REPO}/config/${lower}.rviz" ]]; then
            rviz_cfg="${CRVIZ_REPO}/config/${lower}.rviz"
        else
            rviz_cfg="${STATE_DIR}/${vehicle}.rviz"
            sed "s/RR00/${vehicle}/g" "${DOCKER_DIR}/../rviz/hw_ground_robot.rviz" > "${rviz_cfg}"
        fi
    fi
    say "RViz layout: ${rviz_cfg}"

    # ---- display + GPU: reuse what `crviz up` resolved on this machine, if anything
    local display="${DISPLAY:-}" xauth="${XAUTHORITY:-}" gpu=""
    if [[ -r "${CRVIZ_REPO}/.run/state.env" ]]; then
        # shellcheck disable=SC1091
        display="$( . "${CRVIZ_REPO}/.run/state.env"; echo "${CRVIZ_DISPLAY}")"
        xauth="$( . "${CRVIZ_REPO}/.run/state.env"; echo "${CRVIZ_XAUTH}")"
        gpu="$( . "${CRVIZ_REPO}/.run/state.env"; echo "${CRVIZ_GPU_ACTIVE}")"
    fi
    [[ -n "${display}" ]] || die "no X display (set DISPLAY, or run crviz up once)"
    local -a gpu_args=()
    if [[ "${gpu}" == nvidia ]]; then
        gpu_args=(--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all
                  -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute
                  -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)
    fi

    down >/dev/null

    # ---- 1. MIGHTY, dev router on :7448
    ROVER_ENV_FILE="${envf}" "${DOCKER_DIR}/mighty_hw.sh" start --dev 2>&1 | grep -E 'session up|rror' || true
    docker ps --format '{{.Names}}' | grep -qx hw-mighty || die "MIGHTY did not start"

    # ---- 2. RViz on the host network, bound to the dev router only
    docker run -d --name "${NAME}" --network host --ipc host --init \
        --user "$(id -u):$(id -g)" --env-file "${envf}" \
        -e ZENOH_SESSION_CONFIG_URI=/zenoh/session.json5 \
        -e DISPLAY="${display}" -e XAUTHORITY=/tmp/.xauth -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "${xauth:-/dev/null}:/tmp/.xauth:ro" \
        -v "${DOCKER_DIR}/dev/zenoh_session_config.json5:/zenoh/session.json5:ro" \
        -v "$(dirname "${bag}"):/bags:ro" -v "${HERE}/restamp_relay.py:/relay.py:ro" \
        -v "${rviz_cfg}:/layout.rviz:ro" \
        "${gpu_args[@]}" --entrypoint /bin/bash "${CRVIZ_IMAGE}" -c \
        'source /opt/ros/humble/setup.bash; source /opt/crviz/setup.bash
         export XDG_RUNTIME_DIR=/tmp/xdg-$(id -u); mkdir -p -m 0700 "$XDG_RUNTIME_DIR"
         exec rviz2 -d /layout.rviz' >/dev/null
    sleep 3

    # Refuse to go on if anything in the RViz container talks to :7447.
    local pids
    pids="$(docker top "${NAME}" -eo pid | tail -n +2 | paste -sd'|')"
    if [[ -n "${pids}" ]] && ss -tnpH state established '( dport = :7447 )' 2>/dev/null | grep -Eq "pid=(${pids}),"; then
        down; die "the RViz container connected to :7447 — aborted (it must only use the dev router)"
    fi

    # ---- 3. bag inputs -> relay -> the real topics, re-stamped, held at the end
    local topics="" remap="" p t
    for p in ${pairs}; do t="${p%%=*}"; topics+=" ${t}"; remap+=" ${t}:=/replay${t}"; done
    local setup='source /opt/ros/humble/setup.bash; source /opt/crviz/setup.bash'
    docker exec -d "${NAME}" bash -c "${setup}; exec python3 /relay.py --hold 10 ${pairs} > /tmp/relay.log 2>&1"
    sleep 3
    docker exec -d "${NAME}" bash -c "${setup}; exec ros2 bag play /bags/$(basename "${bag}") ${loop} --topics${topics} --remap${remap} > /tmp/play.log 2>&1"

    cat <<EOF
[goal_session] up. RViz is on ${display}; MIGHTY (${vehicle}) panes: tmux attach -t hw_mighty
$(if [[ -n "${loop}" ]]; then echo "[goal_session] the bag (${duration} s) loops; ${vehicle} jumps back to its start pose each loop."; else echo "[goal_session] the bag plays for ${duration} s, then ${vehicle} stays parked at its last pose with its last map."; fi)
[goal_session] 2D Goal tool -> /${vehicle}/term_goal (coordinates in ${vehicle}/map). Stop: $0 down
EOF
}

case "${1:-}" in
    up) shift; up "$@" ;;
    down) down ;;
    *) sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
