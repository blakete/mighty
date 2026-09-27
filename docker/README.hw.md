# MIGHTY hardware stack (`mighty-hw`)

The MIGHTY planner for real vehicles — ground robots and UAVs — packaged as **one
idling container** driven by **one script**. `mighty_hw.sh` builds a HOST tmux session
`hw_mighty` in which every pane `docker exec`s one node into the container; per-pane
Ctrl-C / Up / Enter restarts that node, Ctrl-b d detaches and the stack keeps running.
Under systemd (`mighty.service`) it starts at boot.

What runs in the container:

| node | when | does |
|---|---|---|
| `mighty_node` | always | the planner |
| `convert_odom_to_state` / `convert_vicon_to_state` | one of them | localization → `<ns>/state` (+ `<ns>/pose`) |
| `mpc` | ground robots | tracks the plan, publishes `<ns>/cmd_vel_auto` |

Localization, sensors, mapping, the ROS/zenoh router and the vehicle's drive or flight
controller are **not** in here; the vehicle provides them (see *Interface* below).

## New vehicle in six steps

**1. Check the vehicle provides what MIGHTY needs** (details in *Interface*):

| | ground robot | UAV |
|---|---|---|
| name | `ROBOT_NAME` ending in digits (e.g. `JR07`; it is the ROS namespace and the agent id) | same |
| state | `nav_msgs/Odometry` on `<ns>/<odom topic>` (or mocap pose + twist) | same |
| TF | `<ns>/map -> <ns>/odom` (or let MIGHTY publish an identity one: `--publish-map-odom-tf`) | not required |
| map | `<ns>/planning_occ_2d_topic`, `occ_2d_topic`, `esdf_2d_topic` (`nav_msgs/OccupancyGrid`) | `<ns>/occupancy_grid`, `<ns>/unknown_grid` (`PointCloud2`) |
| actuation | something consuming `<ns>/cmd_vel_auto` (`geometry_msgs/Twist`) | something consuming `<ns>/goal` (`dynus_interfaces/Goal`) |
| host | Linux amd64, Docker ≥ 23 + compose v2, tmux; a service user in the `docker` group | same |
| ROS graph | the container joins the host network: `RMW_IMPLEMENTATION` + `ROS_DOMAIN_ID` in the env file; with `rmw_zenoh_cpp` a router on the host (default `:7447`) and `zenoh_session_config.json5` in the config dir | same |

**2. Clone** as a user with GitHub access, then hand it to the service user:

```bash
git clone git@github.com:blakete/mighty.git ~/code/mighty     # e.g. /home/swarm/code/mighty
```

**3. Get the image** — pull it (registry login needed), stream it from a machine that has
it, or build it (needs SSH access to the private `mpc` repo, see *The image*):

```bash
docker/mighty_hw.sh pull <tag>                                          # registry
ssh builder 'docker save mighty-hw:<tag> | gzip -1' | gunzip | docker load   # or stream it
make -C docker hw-build                                                 # or build it
```

**4. Install the service** (writes `/etc/mighty/mighty.env` and `mighty.service`; starts
nothing):

```bash
sudo docker/install_service.sh --user swarm --platform ground_robot \
     --odom-topic <your odometry topic> --enable
#   --robot-name JR07        if /etc/rover/rover.env does not set ROBOT_NAME
#   --rmw rmw_zenoh_cpp      if it does not set RMW_IMPLEMENTATION
#   --publish-map-odom-tf    if nothing publishes <ns>/map -> <ns>/odom
#   --platform uav           for a UAV (no MPC; outputs <ns>/goal)
```

**5. Preflight, start, look:**

```bash
docker/mighty_hw.sh check        # PASS/WARN/FAIL: env, image, router, topics, TF
sudo systemctl start mighty
docker/mighty_hw.sh check        # now also: exactly one publisher on state / cmd_vel_auto
tmux attach -t hw_mighty         # as the service user; Ctrl-b d to leave
```

**6. Send a goal** (coordinates in `<ns>/map`; `frame_id` is ignored):

```bash
ros2 topic pub --once /JR07/term_goal geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

or `docker/mighty_rviz.sh` (host rviz2) and use its 2D Goal tool.

## Interface

All topics are relative to the vehicle namespace `<ns>` = `ROBOT_NAME`.

| | ground_robot | uav |
|---|---|---|
| **state in** | `<odom topic>` (`nav_msgs/Odometry`) → `convert_odom_to_state` → `state`, `pose`; or mocap `<pose topic>` + `<twist topic>` → `convert_vicon_to_state` → `state` | same |
| **map in** | `planning_occ_2d_topic` (plans on it; without it the planner never plans), `occ_2d_topic`, `esdf_2d_topic` | `occupancy_grid`, `unknown_grid` (replanning waits for the first cloud) |
| **goal in** | `term_goal` (`PoseStamped`): coordinates are read in `<ns>/map`, `frame_id` is ignored | same; with `provide_goal_in_global_frame` (uav.yaml) the TF `map -> <ns>/init_pose` must exist (`init_pose_parent_frame`) |
| **TF** | `<ns>/map -> <ns>/odom`; the planner and MPC panes wait up to 60 s for it (`MIGHTY_TF_GATE`) | none required |
| **out** | `mpc_waypoints` → MPC → `cmd_vel_auto` (`Twist`, 30 Hz) | `goal` (`dynus_interfaces/Goal`: p, v, a, j, yaw), `trajectory` |
| **both** | `/trajs` (this agent's trajectory, `DynTraj`), `goal_reached`, `exploration/*`, viz markers, `poly_whole`, `poly_safe` | |

- **Agent id**: the trailing digits of `<ns>` (RR08 → 8), used on `/trajs`. A name without
  digits needs `agent_id` in its vehicle overlay (below).
- **Peers**: `use_frame_alignment` is on in both platform configs, so peer trajectories on
  `/trajs` are ignored until `/frame_align/<ns>/<peer>` arrives. Nothing publishes that on
  hardware, i.e. **vehicles do not avoid each other's plans**; turning it off would make
  each vehicle read peers' trajectories in its own map frame.
- **MPC pose**: the MPC tracks against `<ns>/pose`, which `convert_odom_to_state` publishes
  from the odometry (mocap: the mocap pose topic). No localization stack is named anywhere.

## Configuration

Two env files, both passed to the container (the later wins) and read by `mighty_hw.sh`:

| file | holds | written by |
|---|---|---|
| `/etc/rover/rover.env` (`ROVER_ENV_FILE`) | `ROBOT_NAME`, `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` | the vehicle's provisioning (RR fleet: `provision_rover.sh`) |
| `/etc/mighty/mighty.env` (`MIGHTY_ENV_FILE`) | everything MIGHTY-specific (and `ROBOT_NAME`/RMW on a vehicle without rover.env) | `install_service.sh` (`--force` rewrites it) |

| key | default | meaning |
|---|---|---|
| `MIGHTY_PLATFORM` | `ground_robot` | `ground_robot` or `uav` |
| `MIGHTY_STATE_SOURCE` | `odom` | `odom` or `mocap` |
| `MIGHTY_ODOM_TOPIC` | `odom` | odometry topic (relative to `<ns>` unless it starts with `/`) |
| `MIGHTY_POSE_TOPIC` | `pose` / `world` | MPC pose input (odom) or mocap pose topic |
| `MIGHTY_TWIST_TOPIC` | `twist` | mocap twist |
| `MIGHTY_TF_GATE` | ground: `<ns>/map <ns>/odom`; uav: `none` | TF pairs the planner and MPC wait for |
| `MIGHTY_PUBLISH_MAP_ODOM_TF` | `false` | extra pane broadcasting identity `<ns>/map -> <ns>/odom` (`scripts/map_odom_tf.py`) — only where nothing else publishes it |
| `MIGHTY_VEHICLE_CONFIG` | `<ns>.yaml` if present | planner/MPC parameter overlay: a file name in `config/vehicles/` |
| `ROVER_CONFIG_DIR` | `/home/swarm/config` | host dir mounted at `/home/swarm/config` in the container; `zenoh_session_config.json5` lives there |
| `ZENOH_ROUTER_PORT` | `7447` | host zenoh router (rmw_zenoh only) |

**Parameters come from this checkout, not the image.** `compose.hw.yaml` bind-mounts
`config/` over the installed copy, and `launch/mighty_hw.launch.py` layers
`config/mighty.yaml` → `config/platforms/<platform>.yaml` → `config/vehicles/<ns>.yaml`
(see `config/vehicles/README.md`; overlays are git-ignored, and an `mpc:` section there
overrides the MPC's `mpc.yaml`, e.g. `v_max`). So on a vehicle:

```bash
vim config/vehicles/RR08.yaml             # edit
# in that node's pane: Ctrl-C, Up, Enter  # restart one node — no rebuild
```

| | source | a change needs |
|---|---|---|
| `config/**` (planner + MPC overrides) | this checkout | pane restart |
| `launch/`, C++, scripts | image | `make hw-build` + `pull` |
| `mpc.yaml` defaults | mpc repo @ its `mighty.repos` pin | pin bump + rebuild (or override per vehicle) |

## Operating

```bash
docker/mighty_hw.sh start [--dev]     # container + host session (attaches on a tty)
docker/mighty_hw.sh check [--dev]     # read-only preflight
docker/mighty_hw.sh attach            # or: tmux attach -t hw_mighty
docker/mighty_hw.sh status | stop | logs
docker/mighty_hw.sh pull [<tag>]      # registry -> mighty-hw:local (default: latest)
docker/mighty_hw.sh rebuild           # stop + build + start
```

| pane | node | `only_nodes:=` |
|---|---|---|
| `MIGHTY planner` | the planner (waits on the TF gate) | `mighty_node` |
| `convert_odom_to_state` or `convert_vicon_to_state` | localization → `state` | same |
| `MPC` (ground robots) | tracks `mpc_waypoints` (waits on the TF gate) | `mpc` |
| `map->odom TF` (opt-in) | `map_odom_tf.py` | — |

Every pane runs `mighty_hw.launch.py` with `only_nodes:=` its node, so the launch file
stays the single source of truth for parameters (never hand-roll them as `ros2 run`).
Restarting the MPC pane is not instant: it builds an IPOPT/collocation NLP first. The
script refuses an image whose baked launch file predates it (`ros2 launch` silently
ignores undeclared arguments, which would start the full node set in every pane).

### As a service

`mighty.service` runs `mighty_hw.sh start --monitor`: it builds the session and holds the
foreground while the session lives, so `tmux kill-session -t hw_mighty` stops the unit and
`ExecStopPost` runs `stop`. Under `--monitor` a missing zenoh router is a wait, not an
error (at boot the router's service is "started" before it listens). `KillMode=process`
protects the service user's shared tmux server; the journal's `Unit process ... (sleep)
remains running` lines are that loop's `sleep 5` and are expected. Where the unit is
enabled use `systemctl {start,stop,restart} mighty`, not a bare `mighty_hw.sh start`
(both build the same session; a hand-started one is not tracked by the unit).

`After=drive.service sensors.service dlio.service` is ordering only (the RR fleet's
siblings) and harmless where those units do not exist.

### Rollback

Image and checkout move together (the launch-file guard enforces it):

```bash
sudo systemctl stop mighty
git checkout <previous sha>
docker tag mighty-hw:<previous tag> mighty-hw:local
sudo systemctl start mighty
```

## The image

The image is a pure function of *(this checkout, `mighty.repos`)*. The build context is
the repo root; every dependency is cloned inside the build at its pin:

| in the image | source |
|---|---|
| `mighty` | this checkout (`COPY .`, filtered by `Dockerfile.hw.dockerignore`) |
| `mpc` | `git@gitlab.com:mit-acl/ugv/ugv_control/mpc.git` @ pin (**private**: SSH at build) |
| `dynus_interfaces` | github @ pin |
| `decomp_util`, `decomp_ros_msgs` | `DecompROS2` @ pin (its rviz plugin and test node are deleted after cloning) |
| `rmw_zenoh_cpp` | pinned apt build (see *The RMW pin*) |

At run time the planner needs only rclcpp, the message packages, tf2, two PCL libraries
(`libpcl-common`, `libpcl-kdtree`) and, for the MPC, casadi/do-mpc. No rviz, Qt, VTK or
simulator.

### Build

| | requirement | why |
|---|---|---|
| arch | **linux/amd64** | the vehicles are amd64; an arm64 build produces an image they cannot run |
| Docker | Engine ≥ 23 with the **compose v2** plugin | `build --ssh` + BuildKit |
| network | internet | apt, pip, `snapshots.ros.org`, the dep clones |
| disk / RAM | ~15 GB under `/var/lib/docker`; ≥ 8 GB RAM | layer cache; colcon uses every core (`cc1plus ... Killed` = out of memory) |
| SSH → gitlab.com | access to `mit-acl/ugv/ugv_control/mpc` | the only private dependency |

```bash
ssh-add -l || { eval "$(ssh-agent -s)"; ssh-add ~/.ssh/id_ed25519; }   # key for the mpc clone
make -C docker hw-build                                   # -> mighty-hw:local
SSH_KEY=$HOME/.ssh/id_ed25519 make -C docker hw-build     # no agent: a passphrase-free key
make -C docker hw-test                                    # gtests (mighty-test stage)
```

The `mpc` clone runs under `--mount=type=ssh`, so the key never lands in a layer. The
dependency stages are keyed on `mighty.repos` alone; a source edit re-runs only the mighty
colcon layer.

Verify an image before it goes near a vehicle:

```bash
docker run --rm mighty-hw:local bash -c 'source /opt/ros/humble/setup.bash &&
  source $MIGHTY_WS/install/setup.bash &&
  ros2 pkg list | grep -E "^(mighty|mpc|dynus_interfaces|decomp)" &&
  dpkg-query -W ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor &&
  ros2 launch mighty mighty_hw.launch.py --show-args | grep -c "platform"'
```

Expect four packages (`decomp_ros_msgs`, `dynus_interfaces`, `mighty`, `mpc`; `decomp_util`
is plain CMake and never listed), both zenoh packages at the pinned versions, and a
non-zero count. Then `docker/mighty_hw.sh start --dev` on the build machine.

### Publish

The fleet branch is **`main`**. It was named `dev-rr-mad-eth-elevation` until 2026-09-26,
when it replaced this fork's `main`; the abandoned `dev-rr-mad-eth-elevation-v2` is kept as
tag `archive/dev-rr-mad-eth-elevation-v2` (`a2182ab`), and `dev-rr-mad` as
`archive/dev-rr-mad` (`b6e3cd9`). Registry tags are `<branch>-<shortsha>` of this repo's
HEAD, plus `latest` once validated on a vehicle; tag from a clean tree.

```bash
REG=registry.gitlab.com/mit-acl/ugv/redrover/rover/mighty-hw
TAG=$(git rev-parse --abbrev-ref HEAD | tr / -)-$(git rev-parse --short HEAD)
docker tag mighty-hw:local $REG:$TAG && docker push $REG:$TAG
```

Vehicles normally **pull, not build**: on the RR fleet `swarm` is locked out of the git
forges, and the rovers hold no registry credentials, so either `docker login
registry.gitlab.com` as a user with a PAT, or stream the image from the build machine
(`docker save | gzip -1`, check it with `gzip -t` before fanning out).

### The RMW pin

`Dockerfile.hw` pins `ros-humble-rmw-zenoh-cpp` and `ros-humble-zenoh-cpp-vendor` to one
exact apt build, `apt-mark hold`s them and asserts the versions. Two builds that both call
themselves `0.1.9` are not wire-compatible for TRANSIENT_LOCAL topics (`/tf_static` is
one): a mismatched image plans normally and commands exactly zero (RR08, 2026-09-17). Every
container on a vehicle must carry the same build (`mighty_hw.sh check` compares them); to
move a fleet forward, bump the ARGs together and rebuild every image in the same rollout.

### When the build fails

| symptom | cause | fix |
|---|---|---|
| `Permission denied (publickey)` / `Host key verification failed` in `deps-src` | no key forwarded, or no access to `mpc` | `ssh-add -l`; `ssh -T git@gitlab.com`; or `SSH_KEY=<key> make hw-build` |
| `HW_DEPS not in mighty.repos` | an `HW_DEPS` name does not match a `mighty.repos` key | fix the ARG or the entry |
| `unknown flag: --ssh` | compose v1 (`docker-compose`) | install the compose v2 plugin |
| build ends at the `dpkg-query` test | the pinned rmw build no longer resolves | re-pin (see the Dockerfile comment) |
| `cc1plus: fatal error: Killed` | out of memory in the mighty colcon layer | more RAM/swap |
| `rosdep ... cannot resolve` a key | a dependency without a rosdistro key | skip it and install it explicitly, as `libeigen3-dev`/`libboost-dev` are |

## Testing without the vehicle

**`--dev` (laptop).** Identity from `dev/rover.env` (`RR99`), settings from
`dev/mighty.env` (ground robot, DLIO-named odometry), zenoh configs from `dev/`, and an
isolated router on `127.0.0.1:7448` started from the same image (it listens on loopback
only and dials nothing, so a laptop stack never joins a fleet graph). With no inputs the
planner and MPC panes wait out the 60 s TF gate, then start. For a UAV:
`MIGHTY_ENV_FILE=$PWD/docker/dev/mighty.uav.env docker/mighty_hw.sh start --dev`.

**Bag replay (`dev/replay/`)** — the ground-robot regression test. `restamp_relay.py`
republishes bag topics with one constant offset onto the wall clock (headers, every `/tf`
transform, the Livox per-point `timestamp`), so no sim time is involved.

1. `pass1_rover.sh <bag> <seconds> <out>` — on the rover, DISARMED, lidar off: plays the
   bag's `livox/{lidar,imu}` into the live graph; the real DLIO, mapper and stack do the
   rest, and everything the planner needs (plus what it produced) is recorded. Restart
   `dlio.service` and the Orin mapper afterwards.
2. `pass2_laptop.sh <augmented bag> [env]` — on a laptop: replays only the planner inputs
   (TF, DLIO odometry, the three `*_2d_topic` grids) into `--dev` under the recording
   rover's name and counts what the laptop's planner and MPC emit. Enable exploration for
   it with a vehicle overlay (`config/vehicles/RR08.yaml`, `exploration.enabled: true`).

Reference: `mad_summer_2026/planner_test_cases/bag_20260806_151753_RR08_scene5_planner_replay/`
on the NAS (its `description.txt` lists inputs, reference outputs and the recipe).

**Click goals on a replayed bag (`dev/replay/goal_session.sh`).** Runs MIGHTY `--dev`
under the bag's vehicle name, RViz from the containerized-rviz image bound to the dev
router only (never to a `:7447` router on the machine), and the bag's planner inputs
(TF, odometry, the three 2D grids) through `restamp_relay.py --hold`: after the bag
ends the vehicle stays parked at its last pose with its last map, and every 2D Goal
click makes the local planner replan. With `--loop` the bag repeats instead: the relay
keeps time moving forward, and the vehicle jumps back to its start pose each loop.

```bash
docker/dev/replay/goal_session.sh up /path/to/bag_dir [--loop]   # opens RViz; tmux attach -t hw_mighty
docker/dev/replay/goal_session.sh down
```

To run the same thing step by step, see
[*Plan on a replayed bag, step by step*](#plan-on-a-replayed-bag-step-by-step).

**Simulator (`dev/sim/`)** — a dev-only fake_sim harness for the UAV mode; never part of
the hardware image. See `dev/sim/README.md`.

## Plan on a replayed bag, step by step

Replay a recorded bag into the hardware MIGHTY on your laptop, watch it in RViz and send
it goals. Only the bag's planner **inputs** are replayed (TF, odometry, the three 2D
grids, the lidar cloud for context); every path, trajectory and command you see comes
from the MIGHTY running on your machine. Nothing here connects to a robot. The example
is `debug_bag04`, recorded on RR08.

**Before you start**

- Linux with Docker, and a desktop session (RViz opens a window).
- This repository, checked out on the branch with this section (`feature/hw-standalone`
  until it merges). Run every command below from the top of the checkout.
- [containerized-rviz](https://github.com/blakete/containerized-rviz) cloned to
  `~/repos/containerized-rviz` and its image downloaded (`crviz pull`); its README
  covers the GitHub login and `docker login ghcr.io`.
- The MIGHTY image as `mighty-hw:local`. It is on ghcr.io next to the RViz image (ask
  for access to the `mighty-hw` package); with the same `docker login ghcr.io`:
  ```bash
  MIGHTY_REGISTRY=ghcr.io/blakete/mighty-hw docker/mighty_hw.sh pull feature-hw-standalone-5cd6017
  ```
  Or build it: `make -C docker hw-build` (needs SSH access to the private `mpc` repo).
- A folder holding the bag folders (each has a `metadata.yaml`).

**Rules**

- Keep the `ZENOH_SESSION_CONFIG_URI` line in step 2. Without it, on a machine that
  runs the C2 stack, RViz joins the fleet's router on `:7447`. With it, every container
  talks only to MIGHTY's private router on `127.0.0.1:7448`.
- For a bag from another vehicle, replace `RR08` everywhere (and `rr08.rviz` with a
  layout for it), and use an env file with that `ROBOT_NAME`
  (copy `docker/dev/replay/rover.RR08.env`).

**Steps**

1. MIGHTY under the bag's vehicle name:
   ```bash
   ROVER_ENV_FILE=$PWD/docker/dev/replay/rover.RR08.env docker/mighty_hw.sh start --dev
   ```
2. RViz, bound to the dev router (CPU rendering; on NVIDIA add
   `--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all`):
   ```bash
   BAGS=/path/to/folder/holding/bags
   CRVIZ=$HOME/repos/containerized-rviz
   docker run -d --name mighty-rviz --network host --ipc host --init \
     --user "$(id -u):$(id -g)" --env-file docker/dev/replay/rover.RR08.env \
     -e ZENOH_SESSION_CONFIG_URI=/zenoh/session.json5 \
     -e DISPLAY -e XAUTHORITY=/tmp/.xauth -e QT_X11_NO_MITSHM=1 \
     -v /tmp/.X11-unix:/tmp/.X11-unix -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.xauth:ro" \
     -v "$PWD/docker/dev/zenoh_session_config.json5:/zenoh/session.json5:ro" \
     -v "$PWD/docker/dev/replay/restamp_relay.py:/relay.py:ro" \
     -v "$BAGS:/bags:ro" -v "$CRVIZ/config:/configs:ro" \
     --entrypoint bash ghcr.io/blakete/containerized-rviz:latest \
     -c 'source /opt/ros/humble/setup.bash && source /opt/crviz/setup.bash && rviz2 -d /configs/rr08.rviz'
   ```
3. Second terminal: the relay, which re-stamps the inputs onto the wall clock (MIGHTY
   runs on wall time) and, after the bag ends, keeps the last pose, TF and maps coming:
   ```bash
   docker exec -it mighty-rviz bash -c 'source /opt/ros/humble/setup.bash && python3 /relay.py --hold 10 \
     /tf=tf2_msgs/msg/TFMessage /tf_static=tf2_msgs/msg/TFMessage \
     /RR08/dlio/odom_node/odom=nav_msgs/msg/Odometry \
     /RR08/occ_2d_topic=nav_msgs/msg/OccupancyGrid \
     /RR08/esdf_2d_topic=nav_msgs/msg/OccupancyGrid \
     /RR08/planning_occ_2d_topic=nav_msgs/msg/OccupancyGrid \
     /RR08/dlio/odom_node/pointcloud/deskewed=sensor_msgs/msg/PointCloud2'
   ```
4. Third terminal: play only the planner inputs, each renamed under `/replay` so it
   reaches MIGHTY through the relay only. The bag's recorded planner outputs are never
   played. Drop `--loop` to play once and park.
   ```bash
   docker exec -it mighty-rviz bash -c 'source /opt/ros/humble/setup.bash && ros2 bag play /bags/debug_bag04 --loop \
     --topics /tf /tf_static /RR08/dlio/odom_node/odom /RR08/occ_2d_topic /RR08/esdf_2d_topic \
              /RR08/planning_occ_2d_topic /RR08/dlio/odom_node/pointcloud/deskewed \
     --remap /tf:=/replay/tf /tf_static:=/replay/tf_static \
             /RR08/dlio/odom_node/odom:=/replay/RR08/dlio/odom_node/odom \
             /RR08/occ_2d_topic:=/replay/RR08/occ_2d_topic \
             /RR08/esdf_2d_topic:=/replay/RR08/esdf_2d_topic \
             /RR08/planning_occ_2d_topic:=/replay/RR08/planning_occ_2d_topic \
             /RR08/dlio/odom_node/pointcloud/deskewed:=/replay/RR08/dlio/odom_node/pointcloud/deskewed'
   ```
5. RViz's 2D Goal Pose tool publishes `/RR08/term_goal`; watch MIGHTY replan with
   `tmux attach -t hw_mighty` (goals and replans are also logged to
   `/tmp/mighty_debug.log` in `hw-mighty`). Stop: Ctrl-C in terminals 3 and 4, then
   `docker rm -f mighty-rviz && docker/mighty_hw.sh stop`.

`docker/dev/replay/goal_session.sh up <bag> [--loop]` does steps 1–4 in one command.

## The RR fleet

| | owner | MIGHTY consumes |
|---|---|---|
| DLIO odometry, `map -> odom` | `dlio.service` (`dlio_ws`) | `<ns>/dlio/odom_node/odom`, TF |
| Livox MID-360, D455 | `sensors.service` | (mapper input) |
| 2D occupancy / ESDF | `elevation_mapping_cupy` on the rover's Orin | `<ns>/*_2d_topic` |
| zenoh router `:7447`, `cmd_vel_auto` consumer | `drive.service` | everything above |

```bash
sudo docker/install_service.sh --user swarm --platform ground_robot \
     --odom-topic dlio/odom_node/odom --enable
printf 'mighty_node:\n  ros__parameters:\n    exploration.enabled: true\n' > config/vehicles/$ROBOT_NAME.yaml
```
