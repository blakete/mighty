# MIGHTY: Hermite Spline-based Efficient Trajectory Planning — hardware deployment

This is the hardware deployment of [MIGHTY](https://github.com/mit-acl/mighty), a
trajectory planner for ground robots and UAVs, packaged so that a new vehicle can
clone it, build or pull one container image, install it as a systemd service, and start
receiving maps and goals and driving. Simulation, benchmarks and paper material live
upstream in [mit-acl/mighty](https://github.com/mit-acl/mighty).

- **Deploy on a vehicle:** [`docker/README.hw.md`](docker/README.hw.md) — six steps, the
  interface a vehicle has to provide, configuration, operation.
- **Per-vehicle parameters:** [`config/vehicles/README.md`](config/vehicles/README.md).
- **Test without hardware:** the laptop `--dev` mode and bag replay in `docker/README.hw.md`,
  and the dev-only simulator in [`docker/dev/sim/`](docker/dev/sim/README.md).

## What is in here

| path | contents |
|---|---|
| `src/mighty/`, `include/mighty/` | the planner node (`mighty`), L-BFGS trajectory optimization, frontier exploration, the state adapters (`convert_odom_to_state`, `convert_vicon_to_state`) |
| `src/hgp/`, `include/hgp/` | the global planner (graph search over the voxel / 2D map) and convex decomposition |
| `src/test/` | gtests (`make -C docker hw-test`) |
| `launch/mighty_hw.launch.py` | one vehicle: planner + state adapter (+ MPC on ground robots) |
| `config/` | `mighty.yaml` (base), `platforms/{ground_robot,uav}.yaml`, optional `vehicles/<name>.yaml` |
| `scripts/` | `wait_for_tf.py` (startup TF gate), `map_odom_tf.py` (optional identity map→odom) |
| `docker/` | the image (`Dockerfile.hw`), `mighty_hw.sh`, the systemd unit and its installer, dev tooling |
| `mighty.repos` | the three dependencies: `dynus_interfaces`, `mpc` (private), `DecompROS2` |

Platforms: `ground_robot` (2D planning on occupancy grids; an MPC turns the plan into
`cmd_vel_auto`) and `uav` (3D planning on occupancy point clouds; outputs `goal`
setpoints for a flight controller).

## Native build (no Docker)

Ubuntu 22.04 + ROS 2 Humble; also builds on arm64 (e.g. Jetson), since nothing here
depends on a simulator:

```bash
mkdir -p ~/mighty_ws/src && cd ~/mighty_ws/src
git clone git@github.com:blakete/mighty.git
vcs import . < mighty/mighty.repos                # mpc needs SSH access to gitlab.com
rm -rf DecompROS2/decomp_rviz_plugins DecompROS2/decomp_test_node   # optional: skips rviz
cd ~/mighty_ws
rosdep install -y --ignore-src --from-paths src --skip-keys "catkin ament_python"
sudo apt install -y libboost-dev
colcon build --packages-up-to mighty --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch mighty mighty_hw.launch.py namespace:=RR99 platform:=ground_robot odom_topic:=odom
```

The container image (`docker/README.hw.md`) is the supported deployment: it pins the
dependencies and the RMW build, and `mighty_hw.sh` manages the per-node panes.

## Paper

MIGHTY: Hermite Spline-based Efficient Trajectory Planning is available at
[https://arxiv.org/abs/2511.10822](https://arxiv.org/abs/2511.10822); the video is at
[https://youtu.be/Pvb-VPUdLvg](https://youtu.be/Pvb-VPUdLvg).

```bibtex
@ARTICLE{kondo2026mighty,
  author={Kondo, Kota and Wu, Yuwei and Kumar, Vijay and How, Jonathan P.},
  journal={IEEE Robotics and Automation Letters},
  title={MIGHTY: Hermite Spline-based Efficient Trajectory Planning},
  year={2026},
  volume={},
  number={},
  pages={1-8},
  keywords={Anisotropic;Central Processing Unit;Filters;Radio access networks;Regional area networks;Location awareness;Mobile communication;Communication systems;High frequency;Indoor environment;Aerial Systems: Perception and Autonomy;Motion and Path Planning;Trajectory Optimization;Hermite Splines;Unmanned Aerial Vehicles},
  doi={10.1109/LRA.2026.3681187}
}
```

## Acknowledgments

The L-BFGS solver implementation in this repository (`src/mighty/lbfgs_solver.cpp`,
`include/mighty/lbfgs_solver.hpp`) is adapted from
[ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/gcopter/include/gcopter/lbfgs.hpp).
We thank the authors for making their implementation publicly available.
`include/mighty/decomp_ros_utils.hpp` is adapted from DecompROS (BSD-3-Clause, notice in
the file), and `include/third_party/exprtk.hpp` is [ExprTk](https://www.partow.net/programming/exprtk/) (MIT).
