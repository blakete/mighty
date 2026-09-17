# hw-mighty container (hardware stack)

The MIGHTY hardware autonomy stack — **planner + MPC, nothing else** — packaged as
**one idling container** driven by **one script**: `mighty_hw.sh` builds a HOST tmux
session `hw_mighty` in which every pane `docker exec`s its node into the container.
Per-pane Ctrl-C / Up / Enter restarts a single node; Ctrl-b d detaches and the stack
keeps running.

## What ships, and where it comes from

The image is a pure function of **this checkout + `mighty.repos`**. The build context is
the mighty repo root; every dependency is cloned *inside* the build at its
`mighty.repos` pin, filtered to the hardware closure (`HW_DEPS` in `Dockerfile.hw`):

| in the image | source |
|---|---|
| `mighty` | this checkout (`COPY .`, filtered by `Dockerfile.hw.dockerignore`) |
| `mpc` | `git@gitlab.com:mit-acl/ugv/ugv_control/mpc.git` @ pin (**private** — needs SSH at build) |
| `dynus_interfaces` | github @ pin |
| `DecompROS2` (`decomp_util`, `decomp_ros_msgs`, `decomp_rviz_plugins`, `decomp_test_node`) | github @ pin |

Nothing else on the host is an input: no sibling workspaces, no prebuilt tarballs.
Change a dependency by bumping its pin in `mighty.repos` and rebuilding.

**Not in the image, on purpose** (each is owned by another service or host):

| | owner | what MIGHTY consumes |
|---|---|---|
| DLIO odometry, seed pose, `map->odom` | `dlio.service` (`dlio_ws`) | `<ns>/dlio/odom_node/odom`, TF |
| Livox MID-360, D455, `base_link->lidar` | `sensors.service` | `<ns>/livox/lidar` |
| occupancy / ESDF | `elevation_mapping_cupy` on the OX08 Orin | `<ns>/occ_2d_topic`, `<ns>/esdf_2d_topic` |
| zenoh router `:7447` | `drive.service` | everything above, over zenoh |

Nothing in mighty, mpc or dynus_interfaces links the Livox SDK or driver; the only
"livox" in mighty is a topic name and a frame-id string.

## Usage

```bash
./mighty_hw.sh start           # container up + 4-pane host session (auto-attaches on a tty)
./mighty_hw.sh attach          # or: tmux attach -t hw_mighty
./mighty_hw.sh status
./mighty_hw.sh stop
./mighty_hw.sh pull [<tag>]    # fleet registry -> mighty-hw:local (default: latest)
./mighty_hw.sh rebuild         # stop + build + start
```

## Panes

| pane | node | `only_nodes:=` |
|---|---|---|
| `MIGHTY planner` | the planner | `mighty_node` |
| `convert_odom_to_state` | odom → `state` relay | `convert_odom_to_state` |
| `MPC` | tracks `mpc_waypoints`, publishes `cmd_vel_auto` | `mpc` |
| `RViz 2D goal` | `repub_rviz_2Dgoal.py` | — |

The first three all run `onboard_mighty.launch.py`; `only_nodes:=` selects which of
that file's nodes a pane starts, so the launch file stays the single source of truth for
parameters (`mighty_node`'s are a computed merge of `mighty.yaml` ←
`hw_mighty_ground_robot.yaml` plus overrides — never hand-roll them as `ros2 run`).

Only the planner and MPC panes gate on `wait_for_tf.py <ns>/map <ns>/odom` (the
`/tf_static` startup-race fix); the state converter never touches TF. `wait_for_tf.py`
warns and continues on timeout, so the stack never deadlocks. Restarting the MPC pane is
**not instant** — `MPCNode` builds an IPOPT/collocation NLP first.

### Rollback of the per-node split is asymmetric

Revert **this host script only** (`git checkout -- docker/mighty_hw.sh`), no rebuild.
Do **not** revert the launch file while keeping the script: `ros2 launch` silently
ignores an argument it does not declare, so every pane would start the FULL node set —
three publishers on `cmd_vel_auto`. `start` refuses to build the session against an
image whose launch file has no `only_nodes:=`.

## Building (alienware-02, or any box with GitLab SSH access)

```bash
cd docker
SSH_KEY=$HOME/.ssh/id_ed25519 make hw-build     # no ssh-agent: point at a passphrase-free key
make hw-build                                   # with an agent
```

Cloning `mpc` is the only step that needs credentials. On a rover, `swarm` is
deliberately locked out of the git forges — build as `blakete`, or just `pull`.

Layer order keeps rebuilds cheap: the dependency clone + build re-run only when
`mighty.repos` changes; a mighty edit re-runs the mighty colcon layer and the final copy.
Build context is ~80 MB (the repo minus `.git`, `docker/`, benchmark results).

### Publishing

Tag convention on the fleet registry: `<branch>-<shortsha>` of this repo's HEAD, plus
`latest` once validated on a rover.

```bash
REG=registry.gitlab.com/mit-acl/ugv/redrover/rover/mighty-hw
TAG=$(git rev-parse --abbrev-ref HEAD | tr / -)-$(git rev-parse --short HEAD)
docker tag mighty-hw:local $REG:$TAG && docker push $REG:$TAG
# on the rover:
./mighty_hw.sh pull $TAG
```

## Tuning parameters on a rover without rebuilding

```bash
MIGHTY_HW_TUNE=1 ./mighty_hw.sh start
```

`compose.hw.tune.yaml` bind-mounts this checkout's `config/`, `launch/`, `rviz/` over the
installed share tree: edit a YAML, Ctrl-C / Up / Enter the pane. C++ edits still need a
rebuild. `mpc`'s config lives in the mpc repo (cloned in-image), so it is not covered —
commit there and bump the pin.

## Running on a laptop (`--dev`)

```bash
./mighty_hw.sh start --dev
```

No `/etc/rover/rover.env`, no `/home/swarm/config`, no `drive.service`: identity comes
from `dev/rover.env` (`ROBOT_NAME=RR99`), zenoh configs from `dev/`, and an **isolated**
router is started from the same image on `127.0.0.1:7448` (compose profile `dev`).
It listens on loopback only, scouts nothing and dials nothing, so a laptop stack can
never join the fleet graph — including the C2 router on the same machine.

With no sensors or odometry the planner and MPC panes sit in `wait_for_tf.py` until
its 60 s timeout, then start; that is the expected shape of a dev run. `stop` removes
the router too.

## Functional test by bag replay (`dev/replay/`)

A rover on wall power (lidar off) proves only the plumbing. `dev/replay/` turns a recorded
Livox bag into a planner test in two passes, with **no mapper on the laptop and no sim
time**: `restamp_relay.py` republishes bag topics with one constant offset so they land on
the wall clock (headers, every `/tf` transform, and the Livox per-point `timestamp`, which
DLIO deskews with; `/tf_static` goes out transient-local).

1. **`pass1_rover.sh <bag> <seconds> <out>`** — on the rover, DISARMED, lidar off. Plays the
   bag's `livox/{lidar,imu}` into the live graph; the real `dlio.service`, the real Orin
   mapper and the deployed stack do the rest, and everything the planner needs (plus what it
   produced) is recorded into an *augmented* bag. Afterwards `dlio.service` and the Orin
   mapper hold replay state — restart them before real driving.
2. **`pass2_laptop.sh <augmented bag> [env]`** — on a laptop. Replays only the planner
   inputs (TF, DLIO odom **and pose**, the three `*_2d_topic` grids) into the `--dev` stack
   running under the recording rover's name, and counts what the laptop's own planner/MPC
   emit. Expect thousands of goals/trajectories, non-zero `cmd_vel_auto`, the same
   exploration goals as the rover run.

Reference run (2026-09-17, `mad_summer_2026/traversability_test_cases/bag_20260806_151753_RR08`
— "scene 5", 117 s, ~20 s stationary then a clean drive): rover — 27 exploration goals, 8845
trajectories, 4162 `cmd_vel_auto`; laptop replay of the recording — 24 distinct exploration
goals, 5712 trajectories, MPC to `v_max`, 0 TF/solver failures. The augmented bag is the
one to test an image against, archived next to its source as
`bag_20260806_151753_RR08_pass1_augmented_20260917/` (its `description.txt` has the recipe).

Gotcha: the MPC reads its pose from `dlio/odom_node/pose` (`mpc.yaml: pose_topic`), not
from `/state` or TF — leave that topic out of the replay and every command is exactly 0.

## Env & networking

- Identity/transport from **`/etc/rover/rover.env`** via compose `env_file`
  (`ROBOT_NAME`, `VEHTYPE`/`VEHNUM`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`) — same as
  the drive/sensors/dlio siblings; `ROVER_ENV_FILE` / `ROVER_CONFIG_DIR` override the
  paths (that is all `--dev` does).
- `network_mode: host`; the nodes attach to the host router at `localhost:7447`.
- `/home/swarm/config` is mounted read-only and `ZENOH_SESSION_CONFIG_URI` points at the
  fleet `zenoh_session_config.json5` there (5 s non-droppable stall bound plus the
  drop-toward-router QoS rule).
- In-image workspace path is the rover's host path (`/home/swarm/code/mighty_ws`), so
  the pane setup line is what you would type on the rover.
