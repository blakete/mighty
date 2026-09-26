# CLAUDE.md

Guidance for Claude Code (claude.ai/code) in this repository.

## What this repo is

The hardware deployment of the MIGHTY trajectory planner (ROS 2 Humble, C++): one
ament package `mighty` plus a container image and host tooling in `docker/`. It
targets ground robots and UAVs on real hardware. Simulation, benchmarks and paper
material were removed on purpose (they live upstream in mit-acl/mighty); the only
simulator is the dev-only harness in `docker/dev/sim/`, which never enters the
hardware image. Deployment guide: `docker/README.hw.md`.

## Build and test

There is usually no native ROS install on the dev machine: build and test in Docker.

```bash
make -C docker hw-build                 # -> mighty-hw:local (SSH_KEY=~/.ssh/<key> without an agent)
make -C docker hw-test                  # gtests: the mighty-test stage (BUILD_TESTING=ON)
docker/mighty_hw.sh start --dev         # laptop smoke test (RR99, isolated zenoh router :7448)
docker/mighty_hw.sh check --dev         # read-only preflight
docker/dev/replay/pass2_laptop.sh <bag> # ground-robot regression test (bag replay)
```

Native: `vcs import src < src/mighty/mighty.repos`, rosdep, `colcon build
--packages-up-to mighty` (see README.md). The private `mpc` dependency needs SSH to
gitlab.com.

## Layout

- `src/mighty/`, `include/mighty/`: `mighty_node` (ROS interface, ~4k lines),
  `mighty.cpp` (planning pipeline), `lbfgs_solver*` (trajectory optimization),
  `frontier_*` (exploration), `convert_odom_to_state` / `convert_vicon_to_state`
  (localization -> `dynus_interfaces/State`), `decomp_ros_utils.hpp` (vendored).
- `src/hgp/`, `include/hgp/`: global planner (graph search, `map_util.hpp` voxel and
  tri-state 2D maps, convex decomposition via `decomp_util`).
- `launch/mighty_hw.launch.py`: the only launch file. Parameters are layered
  `config/mighty.yaml` -> `config/platforms/<platform>.yaml` ->
  `config/vehicles/<ns>.yaml`; `only_nodes:=` picks one node per tmux pane.
- `docker/`: `Dockerfile.hw` (4 stages + optional `mighty-test`), `compose.hw.yaml`,
  `mighty_hw.sh` (host tmux session, `check`), `check_graph.py`, `mighty.service.in` +
  `install_service.sh`, `dev/` (laptop env, replay rig, sim harness).

## Rules that are easy to break

- Pane commands in `mighty_hw.sh` are sent single-quoted into `docker exec`: no single
  quotes inside them; settings are validated by `safe_value`.
- Never hand-roll node parameters as `ros2 run`; they come from the launch file's YAML
  layering.
- `config/` is bind-mounted from the checkout at run time; `launch/` and code are baked
  into the image. The script refuses an image whose launch file lacks `platform:=`.
- Every container on a vehicle must run the same pinned rmw_zenoh build
  (`Dockerfile.hw` ARGs).
- Keep effective parameters unchanged unless that is the point of the change: compare
  `ros2 param dump` (or the YAML layering) before and after.
- `mighty_node` derives its agent id from the namespace's trailing digits (or the
  `agent_id` parameter); `use_frame_alignment` makes it ignore peer trajectories until
  `/frame_align/<ns>/<peer>` arrives.
