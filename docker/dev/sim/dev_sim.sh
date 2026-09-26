#!/usr/bin/env bash
# dev_sim.sh — run MIGHTY's planner in the dev-only UAV simulator.
#
#   dev_sim.sh build            # mighty-devsim:local, layered on mighty-hw:local
#   dev_sim.sh up [--rviz]      # container + tmux session 'mighty_devsim' (sim | goal shell)
#   dev_sim.sh goal X Y Z       # send /NX01/term_goal
#   dev_sim.sh attach | down
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "${HERE}/compose.devsim.yaml")
SESSION=mighty_devsim
NS="${NS:-NX01}"
SETUP='source /opt/ros/humble/setup.bash && source ${MIGHTY_WS}/install/setup.bash && source ${MIGHTY_DEV_WS}/install/setup.bash'

case "${1:-}" in
    build)
        docker image inspect mighty-hw:local >/dev/null \
            || { echo "build or pull mighty-hw:local first (docker/README.hw.md)" >&2; exit 1; }
        exec "${COMPOSE[@]}" build
        ;;
    up)
        rviz=false; [[ "${2:-}" == --rviz ]] && { rviz=true; xhost +local: >/dev/null 2>&1 || true; }
        "${COMPOSE[@]}" up -d
        tmux kill-session -t "${SESSION}" 2>/dev/null || true
        tmux new-session -d -s "${SESSION}" -n main -x 200 -y 50
        tmux send-keys -t "${SESSION}:main" "docker exec -it mighty-devsim bash -c '${SETUP} && ros2 launch mighty_dev_sim dev_sim.launch.py namespace:=${NS} rviz:=${rviz}'" C-m
        tmux split-window -t "${SESSION}:main"
        tmux send-keys -t "${SESSION}:main" "${HERE}/dev_sim.sh goal 8.0 0.0 2.0   # edit and re-run to send another goal" ""
        echo "[dev_sim] up — tmux attach -t ${SESSION}; send a goal with: $0 goal X Y Z"
        ;;
    goal)
        x="${2:?x}" y="${3:?y}" z="${4:?z}"
        exec docker exec mighty-devsim bash -c "${SETUP} && ros2 topic pub --once /${NS}/term_goal geometry_msgs/msg/PoseStamped \
            '{header: {frame_id: map}, pose: {position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {w: 1.0}}}'"
        ;;
    attach) exec tmux attach -t "${SESSION}" ;;
    down)
        tmux kill-session -t "${SESSION}" 2>/dev/null || true
        exec "${COMPOSE[@]}" down
        ;;
    *) sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
