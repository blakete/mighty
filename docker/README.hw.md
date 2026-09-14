# hw-mighty container (hardware stack)

The MIGHTY hardware autonomy stack (planner + MPC, and DLIO unless something else
owns it) packaged as **one idling container** driven by **one script**:
`mighty_hw.sh` builds a HOST tmux session `hw_mighty` in which every pane
`docker exec`s its node into the container. Per-pane Ctrl-C / Up / Enter restarts
a single node; Ctrl-b d detaches and the stack keeps running.

Sources are **baked from the host checkouts** at their host paths
(`/home/swarm/code/...`): the build context is `/home/swarm/code` because the
image bakes four sibling trees — `mighty_ws/src`, `decomp_ws/src`,
`livox_ws/install`, `Livox-SDK2`. See `Dockerfile.hw.dockerignore` for exactly
what ships. The session builder lives on the host, so pane/flag changes need **no
image rebuild** — only source changes do (`make hw-build` or
`./mighty_hw.sh rebuild`). Budget a few minutes even when every layer is cached:
the build context is large and uploading it dominates.

## Usage

```bash
./mighty_hw.sh start [--odom-type dlio|dlio_in_mocap|mocap|external] [--two-d-only|--no-two-d]
./mighty_hw.sh attach      # or: tmux attach -t hw_mighty
./mighty_hw.sh status
./mighty_hw.sh stop
./mighty_hw.sh rebuild     # stop + image rebuild + start (start flags forwarded)
```

## Panes

Every mode opens these four first:

| pane | node | `only_nodes:=` |
|---|---|---|
| `MIGHTY planner` | the planner | `mighty_node` |
| `convert_odom_to_state` (`convert_vicon_to_state` in mocap) | odom/pose → `state` relay | same as the pane title |
| `MPC` | tracks `mpc_waypoints`, publishes `cmd_vel_auto` | `mpc` |
| `RViz 2D goal` | `repub_rviz_2Dgoal.py` | — |

The first three panes all run `onboard_mighty.launch.py`; its `only_nodes:=`
argument selects **which of that launch file's nodes** a given pane starts. They
were one combined `Onboard MIGHTY` pane until per-node panes landed — splitting
them means Ctrl-C / Up / Enter restarts a single node rather than all three,
while the launch file stays the single source of truth for parameters
(`mighty_node`'s are a computed merge of `mighty.yaml` <- `hw_mighty_ground_robot.yaml`
plus programmatic overrides, so never hand-roll them as `ros2 run`). Keys are the
nodes' real ROS names, so they can be copy-pasted out of `ros2 node list`, and a
comma-separated list collapses several nodes back into one pane
(`only_nodes:=convert_odom_to_state,mpc`).

`only_nodes:=` defaults to empty, meaning "everything this mode selects", so
every other caller of the launch file is unaffected. It is **hardware only** —
in sim it would silently drop `fake_sim`/`pcl_render`, so the launch file
rejects it when `use_hardware:=false`. A key that this mode does not start is
also rejected, because `ros2 launch` with an empty action list exits 0 in
silence, which in a tmux pane is indistinguishable from a healthy node.

Only the planner and MPC panes gate on `wait_for_tf.py`; the state converter is
a pure sub→pub relay that never touches TF, so gating it too would stall a third
pane for the 60 s timeout whenever the TF is missing.

Restarting the MPC pane is **not instant** — `MPCNode` builds an
IPOPT/collocation NLP before its first control tick.

**No mapper pane.** `mighty_node` subscribes to `<ns>/occ_2d_topic` and
`<ns>/esdf_2d_topic` and does not care who publishes them. They come from
`elevation_mapping_cupy` on the OX08 Orin; the `acl-mapping` build is commented
out of `Dockerfile.hw`.

### Rollback is asymmetric

To undo the per-node split, revert **this host script only**:

```bash
git checkout -- docker/mighty_hw.sh && ./mighty_hw.sh start
```

No rebuild. Do **not** revert the launch file while keeping the new script:
`ros2 launch` silently ignores an argument it does not declare, so every pane
would start the FULL node set — three publishers on `cmd_vel_auto`, the exact
duplicate-stack failure `compose.hw.yaml`'s header warns about. `only_nodes` is
inert at its empty default, so leaving the new launch file baked in is the safe
state. `start()` guards this explicitly: it refuses to build the session against
an image whose launch file has no `only_nodes:=` argument.

## Prerequisites (every mode)

- **`drive.service`** — hosts the zenoh router on `:7447`; every pane spin-waits
  on it, and `start` fails fast if it is absent.
- **`sensors.service`** — livox MID-360 + D455. This stack launches **no sensors,
  ever** (the old `--no-sensors` flag is gone because it is the only behavior).
- **`elevation_mapping_cupy` on the OX08 Orin** — without it MIGHTY runs but
  never plans: no occupancy grid.
- **`dlio.service` and the DLIO panes here are the same nodes** — never run both.
  `start` detects a live `dlio.service` (or a container named `dlio`) and
  auto-selects `--odom-type external`, which opens no odometry panes; an explicit
  `--odom-type dlio`/`dlio_in_mocap` is then refused rather than double-published.

## Odom types

| `--odom-type` | Extra panes beyond the four above | Notes |
|---|---|---|
| `external` | *(none)* | **Auto-selected whenever `dlio.service` or a `dlio` container is up.** That stack owns DLIO, the seed pose and `tf map->odom`; MIGHTY consumes `<ns>/dlio/odom_node/odom` plus the map->odom TF over zenoh and does not care which container publishes them. `--two-d-only` is ignored here — `two_d_only` belongs to `dlio.service`. |
| `dlio` | DLIO, seed pose, `tf map->odom` | The default when nothing else owns DLIO. Replicates what `dlio.service` ran: DLIO is told `initial_pose_topic:=world` and a 2 Hz constant `PoseStamped` spoof on `/<ns>/world` (z = `DLIO_SEED_Z`, default 0.4) anchors it. Works with no external infrastructure. |
| `dlio_in_mocap` | DLIO, `tf map->odom` | A **real** mocap publishes `/<ns>/world`. No spoof pane — DLIO anchors to the **first** pose it receives, so a spoof would race the mocap and win. |
| `mocap` | `tf mocap->base_link`, `tf world->map`, `tf map->odom` | No DLIO. Mighty runs `use_onboard_localization:=false` with `twist_topic:=mocap/twist`, and the converter pane becomes `convert_vicon_to_state`. |

DLIO runs a ~3 s stationary IMU calibration whenever its pane (re)starts — keep
the rover still. `--two-d-only` (default **OFF**) pins DLIO's published z to the
seed; it applies only to the `dlio` / `dlio_in_mocap` panes.

## Env & networking

- Identity/transport come from **`/etc/rover/rover.env`** via compose
  `env_file` (`ROBOT_NAME`, `VEHTYPE`/`VEHNUM`, `ROS_DOMAIN_ID`,
  `RMW_IMPLEMENTATION`, optional `DLIO_SEED_Z`) — same as the
  drive/sensors/dlio siblings, so this directory is rover-agnostic.
- The container runs the mighty **nodes only** over `network_mode: host`,
  attaching to the host zenoh router at `localhost:7447` (owned by
  `drive.service`).
- `/home/swarm/config` is mounted read-only and `ZENOH_SESSION_CONFIG_URI` points
  at the fleet `zenoh_session_config.json5` there (5 s non-droppable stall bound
  plus the drop-toward-router QoS rule). `ZENOH_ROUTER_CONFIG_URI` from rover.env
  rides along harmlessly — no router runs in this container, the nodes are plain
  zenoh sessions.

## Startup ordering (/tf_static race)

The static TF panes latch one transient-local sample; over rmw_zenoh a
subscriber that forms before the publisher exists never receives it. Consumers
therefore gate themselves with `ros2 run mighty wait_for_tf.py <pairs>` before
launching — the MIGHTY planner and MPC panes wait on `<ns>/map -> <ns>/odom`.
`wait_for_tf.py` warns and continues on timeout, so the stack never deadlocks.
