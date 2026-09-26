#!/usr/bin/env bash
# install_service.sh — install MIGHTY as a systemd service on this vehicle.
#
#   sudo docker/install_service.sh --user swarm --platform ground_robot \
#        --odom-topic dlio/odom_node/odom --enable
#
# 1. Renders docker/mighty.service.in -> /etc/systemd/system/mighty.service
#    (User=<user>, paths into this checkout) and runs daemon-reload.
# 2. Writes /etc/mighty/mighty.env, MIGHTY's settings for this vehicle, from the
#    flags below. An existing file is kept unless --force (which backs it up).
# 3. Optionally enables (--enable) and (re)starts (--start) the unit.
# Nothing is started unless you ask; run `docker/mighty_hw.sh check` first.
#
# Options (defaults in brackets):
#   --user U              service user; must be in the docker group    [owner of the checkout]
#   --dir D               the mighty checkout                          [the one holding this script]
#   --robot-name NAME     ROBOT_NAME, only if /etc/rover/rover.env does not provide it
#   --platform P          ground_robot | uav                           [ground_robot]
#   --state-source S      odom | mocap                                 [odom]
#   --odom-topic T        nav_msgs/Odometry topic, relative to <ns>    [odom]
#   --pose-topic T        MPC pose / mocap pose topic                  [derived]
#   --twist-topic T       mocap twist topic                            [twist]
#   --tf-gate "A B"|none  TF the planner and MPC wait for              [<ns>/map <ns>/odom on ground]
#   --publish-map-odom-tf run the identity <ns>/map -> <ns>/odom pane (only if nothing else publishes it)
#   --config-dir D        host dir with zenoh_session_config.json5     [/home/swarm/config if it exists,
#                                                                       else /etc/mighty/config]
#   --rmw R               RMW_IMPLEMENTATION, only if rover.env does not provide it
#   --force               overwrite an existing /etc/mighty/mighty.env (a timestamped backup is kept)
#   --enable              systemctl enable mighty (start at boot)
#   --start               systemctl restart mighty now
#   --uninstall           disable + remove the unit (keeps /etc/mighty)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT=/etc/systemd/system/mighty.service
ENV_DIR=/etc/mighty
ENV_FILE="${ENV_DIR}/mighty.env"
ROVER_ENV=/etc/rover/rover.env

die() { echo "[install_service] $*" >&2; exit 1; }
say() { echo "[install_service] $*"; }

dir="$(cd "${SCRIPT_DIR}/.." && pwd)"
user="" robot_name="" platform=ground_robot state_source=odom odom_topic="" pose_topic=""
twist_topic="" tf_gate="" map_odom=false config_dir="" rmw="" force=false enable=false
start=false uninstall=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user) user="$2"; shift ;;
        --dir) dir="$(cd "$2" && pwd)"; shift ;;
        --robot-name) robot_name="$2"; shift ;;
        --platform) platform="$2"; shift ;;
        --state-source) state_source="$2"; shift ;;
        --odom-topic) odom_topic="$2"; shift ;;
        --pose-topic) pose_topic="$2"; shift ;;
        --twist-topic) twist_topic="$2"; shift ;;
        --tf-gate) tf_gate="$2"; shift ;;
        --publish-map-odom-tf) map_odom=true ;;
        --config-dir) config_dir="$2"; shift ;;
        --rmw) rmw="$2"; shift ;;
        --force) force=true ;;
        --enable) enable=true ;;
        --start) start=true ;;
        --uninstall) uninstall=true ;;
        -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
        *) die "unknown option: $1 (see --help)" ;;
    esac
    shift
done

[[ "${EUID}" -eq 0 ]] || die "run with sudo (it writes ${UNIT} and ${ENV_FILE})"

if [[ "${uninstall}" == true ]]; then
    systemctl disable --now mighty 2>/dev/null || true
    rm -f "${UNIT}"
    systemctl daemon-reload
    say "removed ${UNIT} (kept ${ENV_DIR})"
    exit 0
fi

# ---- checks -------------------------------------------------------------------
[[ -x "${dir}/docker/mighty_hw.sh" && -f "${dir}/docker/mighty.service.in" ]] \
    || die "${dir} is not a mighty checkout (no docker/mighty_hw.sh)"
user="${user:-$(stat -c %U "${dir}")}"
id "${user}" >/dev/null 2>&1 || die "no such user: ${user}"
[[ "${user}" != root ]] || die "refusing to run the stack as root; pass --user"
id -nG "${user}" | tr ' ' '\n' | grep -qx docker \
    || die "${user} is not in the docker group (sudo usermod -aG docker ${user}, then log in again)"
command -v docker >/dev/null || die "docker is not installed"
command -v tmux >/dev/null || die "tmux is not installed (the panes are a host tmux session)"
case "${platform}" in ground_robot|uav) ;; *) die "--platform must be ground_robot or uav" ;; esac
case "${state_source}" in odom|mocap) ;; *) die "--state-source must be odom or mocap" ;; esac
for v in "${robot_name}" "${odom_topic}" "${pose_topic}" "${twist_topic}" "${tf_gate}" "${rmw}" "${config_dir}"; do
    [[ "${v}" =~ ^[A-Za-z0-9_./:\ -]*$ ]] || die "invalid value: '${v}'"
done

rover_has() { [[ -r "${ROVER_ENV}" ]] && grep -Eq "^[[:space:]]*(export[[:space:]]+)?$1=." "${ROVER_ENV}"; }
if ! rover_has ROBOT_NAME && [[ -z "${robot_name}" && ! -f "${ENV_FILE}" ]]; then
    die "no ROBOT_NAME in ${ROVER_ENV}: pass --robot-name (e.g. JR07; it must end in digits)"
fi

# ---- the unit -----------------------------------------------------------------
tmp="$(mktemp)"
sed -e "s|@SERVICE_USER@|${user}|g" -e "s|@MIGHTY_DIR@|${dir}|g" "${dir}/docker/mighty.service.in" > "${tmp}"
# An empty unit file is reported by systemd as "masked"; never install one.
grep -q '^ExecStart=' "${tmp}" && ! grep -q '@' <(grep -v '^#' "${tmp}") \
    || { rm -f "${tmp}"; die "rendering mighty.service.in failed"; }
if [[ -f "${UNIT}" ]] && ! cmp -s "${tmp}" "${UNIT}"; then
    cp -p "${UNIT}" "${UNIT}.bak-$(date +%Y%m%d-%H%M%S)"
    say "previous unit backed up next to ${UNIT}"
fi
install -m 0644 "${tmp}" "${UNIT}"
rm -f "${tmp}"
say "installed ${UNIT} (User=${user}, ${dir})"

# ---- /etc/mighty/mighty.env ---------------------------------------------------
if [[ -z "${config_dir}" ]]; then
    if [[ -d /home/swarm/config ]]; then config_dir=/home/swarm/config
    else config_dir="${ENV_DIR}/config"; fi
fi
if [[ -f "${ENV_FILE}" && "${force}" != true ]]; then
    say "kept existing ${ENV_FILE} (pass --force to rewrite it from these flags)"
else
    mkdir -p "${ENV_DIR}"
    [[ -d "${config_dir}" ]] || { mkdir -p "${config_dir}"; say "created ${config_dir}"; }
    if [[ -f "${ENV_FILE}" ]]; then
        cp -p "${ENV_FILE}" "${ENV_FILE}.bak-$(date +%Y%m%d-%H%M%S)"
        say "previous ${ENV_FILE} backed up"
    fi
    {
        echo "# MIGHTY settings for this vehicle — written by docker/install_service.sh"
        echo "# on $(date -Iseconds). Read by docker/mighty_hw.sh and passed to the"
        echo "# container after ${ROVER_ENV} (this file wins). Keys: see mighty_hw.sh."
        [[ -n "${robot_name}" ]] && echo "ROBOT_NAME=${robot_name}"
        [[ -n "${rmw}" ]] && echo "RMW_IMPLEMENTATION=${rmw}"
        echo "MIGHTY_PLATFORM=${platform}"
        echo "MIGHTY_STATE_SOURCE=${state_source}"
        [[ -n "${odom_topic}" ]] && echo "MIGHTY_ODOM_TOPIC=${odom_topic}"
        [[ -n "${pose_topic}" ]] && echo "MIGHTY_POSE_TOPIC=${pose_topic}"
        [[ -n "${twist_topic}" ]] && echo "MIGHTY_TWIST_TOPIC=${twist_topic}"
        [[ -n "${tf_gate}" ]] && echo "MIGHTY_TF_GATE=${tf_gate}"
        echo "MIGHTY_PUBLISH_MAP_ODOM_TF=${map_odom}"
        echo "ROVER_CONFIG_DIR=${config_dir}"
    } > "${ENV_FILE}"
    chmod 0644 "${ENV_FILE}"
    say "wrote ${ENV_FILE}:"
    grep -v '^#' "${ENV_FILE}" | sed 's/^/    /'
fi

# ---- systemd --------------------------------------------------------------------
systemctl daemon-reload
if [[ "${enable}" == true ]]; then
    systemctl enable mighty
fi
if [[ "${start}" == true ]]; then
    systemctl restart mighty
    say "mighty.service (re)started: systemctl status mighty; tmux attach -t hw_mighty (as ${user})"
fi

cat <<EOF
[install_service] next:
    ${dir}/docker/mighty_hw.sh pull <tag>     # or: make -C ${dir}/docker hw-build
    ${dir}/docker/mighty_hw.sh check          # preflight: env, image, router, topics, TF
    sudo systemctl start mighty               # enabled at boot: $( [[ "${enable}" == true ]] && echo yes || echo "no (--enable)" )
EOF
