#!/usr/bin/env bash
# PASS 2 — run ON A LAPTOP. Plays the planner INPUTS of a pass1_rover.sh
# recording (TF, DLIO odom + pose, the three 2-D grids) through the re-stamp
# relay into the --dev stack running under the recording rover's name, and
# counts what the LAPTOP's own planner/MPC emit. No mapper, no sim time.
# Verified on alienware-02 2026-09-17: same 15 exploration goals as the rover run,
# MPC ramps to v_max, 0 TF/solver failures.
#
#   pass2_laptop.sh <augmented bag dir> [rover env file, default rover.RR08.env]
#
# GOTCHA: MPC takes its pose from dlio/odom_node/pose (mpc.yaml pose_topic), NOT
# from /state or TF — leave that topic out and every cmd_vel_auto is exactly 0.
set -uo pipefail
BAG="${1:?augmented bag dir}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; D="$(cd "$HERE/../.." && pwd)"
ENVF="${2:-$HERE/rover.RR08.env}"
set -a; . "$ENVF"; set +a; R="${ROBOT_NAME:?}"
# Count for the whole bag plus the planner's tail, whatever the bag's length.
W=$(python3 -c "import yaml;print(int(yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9)+25)")
cd "$D"
ROVER_ENV_FILE="$ENVF" ./mighty_hw.sh start --dev 2>&1 | grep -E "session up|rror"
sleep 8
docker rm -f replay2 >/dev/null 2>&1 || true
docker run -d --name replay2 --network host --init --shm-size=256m --env-file "$ENVF" \
  -e ZENOH_SESSION_CONFIG_URI=/home/swarm/config/zenoh_session_config.json5 \
  -v "$D/dev:/home/swarm/config:ro" -v "$BAG:/bag:ro" -v "$HERE/restamp_relay.py:/relay.py:ro" \
  mighty-hw:local sleep infinity >/dev/null
X() { docker exec replay2 bash -c "source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash && $1"; }
docker exec -d replay2 bash -c "source /opt/ros/humble/setup.bash && source /home/swarm/code/mighty_ws/install/setup.bash && R=$R W=$W python3 - > /tmp/count.log 2>&1 <<'PY'
import rclpy, time, math, os
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from dynus_interfaces.msg import Goal, Trajectory, SpeedyPath
R=os.environ['R']
class C(Node):
    def __init__(s):
        super().__init__('pass2_counter'); s.n={}; s.vmax=0.0; s.goals=set()
        for name,typ,k in [(f'/{R}/goal',Goal,'goal'),(f'/{R}/trajectory',Trajectory,'traj'),(f'/{R}/mpc_waypoints',SpeedyPath,'wp'),(f'/{R}/cmd_vel_auto',Twist,'cmd'),(f'/{R}/exploration/current_goal',PoseStamped,'explore')]:
            s.n[k]=0; s.create_subscription(typ,name,(lambda m,k=k: s.on(k,m)),10)
    def on(s,k,m):
        s.n[k]+=1
        if k=='cmd': s.vmax=max(s.vmax, math.hypot(m.linear.x, m.angular.z))
        if k=='explore': s.goals.add((round(m.pose.position.x,2), round(m.pose.position.y,2)))
rclpy.init(); c=C(); t0=time.time()
while time.time()-t0<int(os.environ['W']): rclpy.spin_once(c, timeout_sec=0.2)
print('laptop outputs:', c.n); print('max |cmd_vel_auto| (lin+ang):', round(c.vmax,3)); print('distinct exploration goals:', len(c.goals))
PY"
T="/tf /tf_static /$R/dlio/odom_node/odom /$R/dlio/odom_node/pose /$R/occ_2d_topic /$R/esdf_2d_topic /$R/planning_occ_2d_topic"
PAIRS="/tf=tf2_msgs/msg/TFMessage /tf_static=tf2_msgs/msg/TFMessage /$R/dlio/odom_node/odom=nav_msgs/msg/Odometry /$R/dlio/odom_node/pose=geometry_msgs/msg/PoseStamped /$R/occ_2d_topic=nav_msgs/msg/OccupancyGrid /$R/esdf_2d_topic=nav_msgs/msg/OccupancyGrid /$R/planning_occ_2d_topic=nav_msgs/msg/OccupancyGrid"
REMAP=""; for t in $T; do REMAP="$REMAP $t:=/replay$t"; done
docker exec -d replay2 bash -c "source /opt/ros/humble/setup.bash && exec python3 /relay.py $PAIRS > /tmp/relay.log 2>&1"
sleep 3
X "ros2 bag play /bag --topics $T --remap $REMAP 2>&1 | grep -v '^$' | grep -v zenoh_transport | tail -1"
sleep 20
docker exec replay2 tail -1 /tmp/relay.log | cut -c1-140
for i in $(seq 1 60); do docker exec replay2 test -s /tmp/count.log && break; sleep 3; done; docker exec replay2 cat /tmp/count.log
docker exec hw-mighty bash -c "echo mpc CMD steps: \$(grep -c CMD /tmp/mpc_debug.log), failures: \$(grep -c failed /tmp/mpc_debug.log); grep CMD /tmp/mpc_debug.log | tail -1 | cut -c1-140"
docker rm -f replay2 >/dev/null; ./mighty_hw.sh stop 2>&1 | tail -1
