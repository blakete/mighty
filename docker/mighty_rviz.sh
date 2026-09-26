#!/usr/bin/env bash
# mighty_rviz.sh — open RViz on one vehicle's MIGHTY topics.
#
#   mighty_rviz.sh [ROBOT_NAME]     # default: ROBOT_NAME from the env files
#
# Renders rviz/hw_ground_robot.rviz (RR00 placeholder) for the vehicle and runs
# the HOST's rviz2 (a ROS 2 install on the machine you view from, on the same
# ROS graph as the vehicle). The 2D Goal tool publishes /<ROBOT_NAME>/term_goal.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
name="${1:-}"
if [[ -z "${name}" ]]; then
    for f in "${MIGHTY_ENV_FILE:-/etc/mighty/mighty.env}" "${ROVER_ENV_FILE:-/etc/rover/rover.env}"; do
        [[ -r "${f}" ]] || continue
        name="$(sed -n -E 's/^[[:space:]]*(export[[:space:]]+)?ROBOT_NAME=["'\'']?([^"'\''[:space:]]*).*$/\2/p' "${f}" | tail -n 1)"
        [[ -n "${name}" ]] && break
    done
fi
[[ "${name}" =~ ^[A-Za-z0-9_]+$ ]] || { echo "usage: $0 ROBOT_NAME" >&2; exit 2; }
command -v rviz2 >/dev/null || { echo "rviz2 not found: source your ROS 2 setup first" >&2; exit 1; }
out="${TMPDIR:-/tmp}/mighty_${name}.rviz"
sed "s/RR00/${name}/g" "${HERE}/../rviz/hw_ground_robot.rviz" > "${out}"
exec rviz2 -d "${out}"
