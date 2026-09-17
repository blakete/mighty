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

## Building on a fresh machine

The image is a pure function of *(this checkout, `mighty.repos`)*, so a fresh box needs
only Docker, an SSH key with access to the private `mpc` repo, and the repo itself. No
ROS, no colcon, no sibling workspaces on the host.

### 0. Host prerequisites

| | requirement | why |
|---|---|---|
| arch | **linux/amd64** | the rovers are amd64 NUCs; a native arm64 build (Orin, Apple silicon) produces an image they cannot run |
| Docker | Engine ≥ 23 with the **compose v2** plugin (`docker compose`, not `docker-compose`) | `build --ssh` + BuildKit; RR08 runs Engine 29.7.2 / compose v5.4.0 |
| network | internet on the first build | `# syntax=docker/dockerfile:1.7` pulls the BuildKit frontend; apt, pip, `snapshots.ros.org` and the dep clones all fetch |
| disk | ~20 GB free under `/var/lib/docker` | the image is ~3.9 GB and the layer cache is several times that |
| RAM | ≥ 8 GB | colcon builds mighty's C++ with every core; a memory-tight box shows up as `cc1plus ... Killed` |
| SSH → gitlab.com | an account with access to `mit-acl/ugv/ugv_control/mpc` | the **only** private dependency. `dynus_interfaces` and `DecompROS2` clone over https, unauthenticated |
| SSH → github.com | to clone this repo | not needed by the build itself |

To `pull` or `push` images you also need `docker login registry.gitlab.com` with a
GitLab PAT (`read_registry`, plus `write_registry` to publish).

### 1. Sources

```bash
git clone git@github.com:blakete/mighty.git ~/code/mighty
cd ~/code/mighty
git checkout feature/hw-image-self-contained   # not merged as of 2026-09-17
```

Nothing else is cloned by hand: `mighty.repos` pins are resolved *inside* the build.

### 2. Make the GitLab key reachable to the build

The `mpc` clone happens in the `deps-src` stage under `--mount=type=ssh`, so the key is
forwarded for that one RUN and never lands in a layer.

```bash
ssh-add -l || { eval "$(ssh-agent -s)"; ssh-add ~/.ssh/id_ed25519; }
ssh -T git@gitlab.com     # expect: Welcome to GitLab, @<user>!
```

No agent (headless build box, cron): skip the agent and point `SSH_KEY` at a
**passphrase-free** key in step 3 — `--ssh default=<path>` cannot prompt.

### 3. Build

```bash
cd docker
make hw-build                                   # with an ssh-agent
SSH_KEY=$HOME/.ssh/id_ed25519 make hw-build     # without one
```

Either is `docker compose -f compose.hw.yaml build --ssh default[=$SSH_KEY]`, producing
`mighty-hw:local`. Build context is ~80 MB (the repo minus `.git`, `docker/`, benchmark
results — see `Dockerfile.hw.dockerignore`).

Cold, the two colcon stages dominate the wall clock. Layer order keeps repeat builds
cheap: `deps-src`/`deps-build` are keyed on `mighty.repos` alone and re-run only when a
pin moves; a mighty source edit re-runs just the mighty colcon layer and the final copy;
a `config/*.yaml` edit needs no rebuild at all (see *Parameters* below).

### 4. Verify the image before it goes near a rover

```bash
docker run --rm mighty-hw:local bash -c '
  source /opt/ros/humble/setup.bash &&
  source /home/swarm/code/mighty_ws/install/setup.bash &&
  ros2 pkg list | grep -E "^(mighty|mpc|dynus_interfaces|decomp)" &&
  dpkg-query -W ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor &&
  ros2 launch mighty onboard_mighty.launch.py --show-args 2>/dev/null | grep -c only_nodes'
```

Expect, in order:

1. six packages — `mighty`, `mpc`, `dynus_interfaces`, `decomp_ros_msgs`,
   `decomp_rviz_plugins`, `decomp_test_node`. `decomp_util` is a plain CMake package and
   never appears in `ros2 pkg list` — run
   `ls /home/swarm/code/mighty_ws/install/decomp_util` inside the image instead.
2. both zenoh packages at exactly the pinned versions (see *The RMW pin* below). The
   build already fails on a mismatch; this catches a hand-patched image.
3. `1` — the `only_nodes:=` argument exists. `0` or an error means `mighty_hw.sh start`
   will refuse this image (it would otherwise run three full stacks, one per pane).

Then a no-hardware smoke test on the build machine itself:

```bash
./mighty_hw.sh start --dev     # 4 panes + an isolated router on 127.0.0.1:7448
./mighty_hw.sh stop
```

With no odometry the planner and MPC panes sit in `wait_for_tf.py` for its 60 s timeout
and then start — that is the expected shape of a dev run, and it proves the binaries,
the launch file and the zenoh session config load. It proves nothing about planning:
for that, run the bag replay in *Functional test* below.

### 5. Publish

Tag convention on the fleet registry: `<branch>-<shortsha>` of this repo's HEAD, plus
`latest` once validated on a rover.

```bash
REG=registry.gitlab.com/mit-acl/ugv/redrover/rover/mighty-hw
TAG=$(git rev-parse --abbrev-ref HEAD | tr / -)-$(git rev-parse --short HEAD)
docker tag mighty-hw:local $REG:$TAG && docker push $REG:$TAG
# on the rover:
./mighty_hw.sh pull $TAG
```

Tag from a **clean** tree — the sha is the only record of what went in, and `config/` is
bind-mounted from the checkout anyway, so a dirty config does not even reach the image.

### On a rover: pull, do not build

`swarm` is deliberately locked out of the git forges fleet-wide, so the `mpc` clone
cannot succeed as that user. Either build as `blakete` (who has the keys) or — the normal
path — `./mighty_hw.sh pull <tag>` against the registry. The rovers have no registry
credentials of their own by default; `docker login registry.gitlab.com` first, or
`docker save | ssh ... docker load` from the build box.

### The RMW pin is fleet-wide state

`Dockerfile.hw` pins `ros-humble-rmw-zenoh-cpp` and `ros-humble-zenoh-cpp-vendor` to one
exact apt build from a dated `snapshots.ros.org` repo, `apt-mark hold`s them and asserts
the installed versions at the end of the layer. This is not tidiness: two builds that both
call themselves `0.1.9` are not wire-compatible for TRANSIENT_LOCAL topics, and `/tf_static`
is one — a mismatched image plans normally and commands exactly zero (RR08, 2026-09-17).

Every container on a rover (drive, sensors, dlio, this one) must carry the same build. To
move the fleet forward, bump all three ARGs together and rebuild **every** rover image in
the same rollout. Candidate versions in a given snapshot:

```bash
curl -s http://snapshots.ros.org/humble/<date>/ubuntu/dists/jammy/main/binary-amd64/Packages.gz \
  | gunzip | awk '/^Package: ros-humble-(rmw-zenoh-cpp|zenoh-cpp-vendor)$/{p=1;print} p&&/^Version:/{print;p=0}'
```

`packages.ros.org` keeps only the newest build per version, which is why the pin cannot
come from there.

### When the build fails

| symptom | cause | fix |
|---|---|---|
| `Permission denied (publickey)` or `Host key verification failed` in the `deps-src` stage | no key forwarded, or the key has no access to `mpc` | `ssh-add -l`; `ssh -T git@gitlab.com`; or `SSH_KEY=<passphrase-free key> make hw-build` |
| `AssertionError: HW_DEPS not in mighty.repos: [...]` | a `HW_DEPS` name does not match a `mighty.repos` key | fix the ARG or the repos entry — the filter fails loudly on purpose |
| `unknown flag: --ssh` | compose v1 (`docker-compose`) | install the compose v2 plugin |
| build ends at the `dpkg-query` test | the snapshot no longer carries the pinned build, or the three ARGs drifted apart | re-derive them with the `curl`/`awk` above |
| `cc1plus: fatal error: Killed` | OOM during the mighty colcon layer | more RAM/swap, or build on a bigger box |
| `rosdep ... cannot resolve` a new key | a dependency was added without a rosdistro key | add it to the `--skip-keys` list *and* install it explicitly, as `libeigen3-dev` and `nlohmann-json3-dev` already are |
| `start` says `no zenoh router on 127.0.0.1:7447` | not a build problem | `drive.service` hosts the router; start it, or use `--dev` |

## Parameters: from the checkout, not the image

`compose.hw.yaml` bind-mounts **this checkout's `config/`** over the installed
`share/mighty/config`, always. So on a rover:

```bash
vim config/hw_mighty_ground_robot.yaml     # edit
# in the planner pane: Ctrl-C, Up, Enter   # restart that one node
git commit -am "..."                        # the change is a commit here, never a rebuild
```

What that covers and what it does not:

| | source | change needs |
|---|---|---|
| `config/*.yaml` (planner params, zenoh session) | this checkout | pane restart |
| `launch/`, `rviz/`, C++ | image | `make hw-build` + `pull` |
| `mpc.yaml` | mpc repo @ its `mighty.repos` pin, baked | commit in mpc + pin bump + rebuild |

`launch/` is deliberately not mounted: a launch edit can reference things the image does not
contain, which is the drift `start`'s `only_nodes` guard exists to catch.

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
one to test an image against, on the NAS at
`mad_summer_2026/planner_test_cases/bag_20260806_151753_RR08_scene5_planner_replay/`
(its `description.txt` lists inputs vs reference outputs and the recipe).

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
