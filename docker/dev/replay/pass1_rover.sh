#!/usr/bin/env bash
# PASS 1 — run ON A ROVER (as blakete). Replays a bag's Livox topics, re-stamped
# onto the wall clock, into the rover's LIVE graph, so the real dlio.service, the
# real Orin elevation mapper and the deployed mighty-hw stack do the rest, and
# records everything the planner needs (+ what it produced) into an augmented bag
# for pass2_laptop.sh. Verified on RR08 2026-09-17 (scene-5 face_plant bag).
#
#   pass1_rover.sh <bag dir on this rover> <play seconds> <output dir>
#
# SAFETY: refuses unless /<ROBOT_NAME>/mode is DISARMED. MPC will publish
# cmd_vel_auto; DISARMED is what keeps it off the motors. The lidar should be
# OFF (no live publisher on livox/lidar) or the two streams will interleave.
# Afterwards dlio.service and the Orin mapper hold replay state — restart them
# before real use.
set -uo pipefail
BAG="${1:?bag dir}"; SECS="${2:?play seconds}"; OUT="${3:?output dir}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set -a; . /etc/rover/rover.env; set +a
R="${ROBOT_NAME:?}"
mkdir -p "$OUT" && chmod 777 "$OUT"
docker rm -f replay >/dev/null 2>&1 || true
docker run -d --name replay --network host --init --shm-size=256m --env-file /etc/rover/rover.env \
  -e ZENOH_SESSION_CONFIG_URI=/home/swarm/config/zenoh_session_config.json5 \
  -v /home/swarm/config:/home/swarm/config:ro -v "$BAG:/bag:ro" -v "$OUT:/out" \
  -v "$HERE/restamp_relay.py:/relay.py:ro" mighty-hw:local sleep infinity >/dev/null
X() { docker exec replay bash -c "source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash && $1"; }
MODE=$(X "timeout 8 ros2 topic echo --once /$R/mode 2>/dev/null | head -1")
echo "mode: $MODE"; [[ "$MODE" == *DISARMED* ]] || { echo "ABORT: $R not DISARMED"; docker rm -f replay; exit 1; }
REC="pass1_$(date +%Y%m%d_%H%M%S)"
docker exec -d replay bash -c "source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash && exec ros2 bag record -o /out/$REC \
  /$R/livox/lidar /$R/livox/imu /$R/dlio/odom_node/odom /$R/dlio/odom_node/pose /$R/dlio/odom_node/pointcloud/deskewed /tf /tf_static \
  /$R/occ_2d_topic /$R/esdf_2d_topic /$R/planning_occ_2d_topic /$R/state /$R/goal /$R/mpc_waypoints /$R/trajectory /$R/cmd_vel_auto \
  /$R/exploration/current_goal /$R/exploration/frontiers /$R/goal_reached /$R/mode /$R/hgp_path_marker > /out/record.log 2>&1"
docker exec -d replay bash -c "source /opt/ros/humble/setup.bash && exec python3 /relay.py /$R/livox/lidar=sensor_msgs/msg/PointCloud2 /$R/livox/imu=sensor_msgs/msg/Imu > /out/relay.log 2>&1"
sleep 4
echo "playing $SECS s (keep the first ~3 s stationary for DLIO's IMU calibration)"
X "timeout $SECS ros2 bag play /bag --topics /$R/livox/lidar /$R/livox/imu --remap /$R/livox/lidar:=/replay/$R/livox/lidar /$R/livox/imu:=/replay/$R/livox/imu 2>&1 | grep -v '^$' | tail -1"
sleep 18
docker exec replay bash -c "pkill -INT -f 'ros2 bag record'; pkill -INT -f restamp_relay.py; sleep 4"
tail -1 "$OUT/relay.log" | cut -c1-140
X "ros2 bag info /out/$REC 2>/dev/null | grep -E 'Duration|Topic:' | sed 's/Serialization.*//'"
docker rm -f replay >/dev/null
echo "augmented bag: $OUT/$REC"
