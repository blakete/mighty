# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MIGHTY (Hermite Spline-based Efficient Trajectory Planning) is a ROS 2 (Jazzy on Ubuntu 24.04 on this branch; `main` stays Humble) C++ package for real-time UAV trajectory planning with obstacle avoidance. It supports multi-agent simulations, Gazebo-based single-agent simulations, and hardware deployment.

## Build Commands

The project uses `ament_cmake` (colcon) as its build system. The workspace root is `~/code/mighty_ws`.

```bash
# Build the entire workspace (from mighty_ws root)
cd ~/code/mighty_ws
source /opt/ros/jazzy/setup.bash
source ~/code/decomp_ws/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build only the mighty package
colcon build --packages-select mighty --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source after building
source install/setup.bash
```

On this branch, `BUILD_SIMULATION` defaults to OFF because Gazebo Classic is not available on Jazzy. DecompROS2's `decomp_test_node` is skipped: build `decomp_util` first, then `colcon build --packages-skip decomp_util decomp_test_node`.

Initial setup (installs all dependencies, clones repos, builds everything):
```bash
./setup.sh        # uses all CPUs
./setup.sh -j 4   # limit parallel jobs
```

## Running Simulations

```bash
# Multi-agent simulation (default 10 agents)
python3 src/mighty/scripts/run_sim.py --mode multiagent -s ~/code/mighty_ws/install/setup.bash

# Single-agent Gazebo simulation
python3 src/mighty/scripts/run_sim.py --mode gazebo -s ~/code/mighty_ws/install/setup.bash

# Docker alternative (from docker/ directory)
make run           # multiagent
make run-gazebo    # gazebo
make shell         # interactive debug shell
```

## Architecture

### Core Planning Pipeline (`src/mighty/`, `include/mighty/`)

- **`mighty_node.hpp/cpp`** — ROS 2 node: subscribes to state, point clouds, goal, and dynamic trajectories; publishes planned trajectories and visualization markers.
- **`mighty.hpp/cpp`** — Core planner class: Hermite spline trajectory generation, obstacle constraint formulation, corridor-based planning.
- **`lbfgs_solver.hpp/cpp`** — L-BFGS optimization solver for trajectory refinement. `lbfgs_solver_utils` provides helper functions.
- **`mighty_type.hpp`** — Central type definitions and parameter struct (`parameters`) used throughout.
- **`initial_guess.hpp`** — Initial trajectory guess generation.
- **`utils.hpp/cpp`** — Shared utilities (coordinate transforms, geometry helpers).

### Dynamic Graph Planner (`src/dgp/`, `include/dgp/`)

- **`dgp_manager.hpp/cpp`** — Manages the occupancy map (voxel grid), convex decomposition, and interfaces with the planner. Handles both static and dynamic obstacles.
- **`dgp_planner.hpp/cpp`** — Graph-based global path planner over the voxel grid.
- **`graph_search.hpp/cpp`** — A* graph search implementation.
- **`map_util.hpp`** / **`read_map.hpp`** — Voxel map data structure and I/O.

### Simulation & Utilities (`src/sim/`, `src/mighty/`)

- **`fake_sim.cpp`** — Lightweight simulation node (no Gazebo) that integrates planned trajectories.
- **`obstacle_tracker_node.cpp`** — Tracks dynamic obstacles from sensor data.
- **`pure_pursuit.cpp`** — Ground robot controller.
- **`move_model.cpp`** — Gazebo plugin for moving dynamic obstacles.
- **`convert_odom_to_state.cpp`** / **`convert_vicon_to_state.cpp`** — Sensor-to-state converters for hardware.

### Key Dependencies

- **Eigen3** — Linear algebra (matrices, vectors throughout)
- **PCL** — Point cloud processing for obstacle detection
- **DecompROS2** (`decomp_util`) — Convex decomposition for safe flight corridors (built in separate `decomp_ws`)
- **dynus_interfaces** — Custom ROS 2 message types (State, Goal, Trajectory, DynTraj, etc.)
- **OpenMP** — Parallel computation in the planner

### Configuration

- `config/mighty.yaml` — Main planner parameters (simulation)
- `config/hw_mighty.yaml` — Hardware deployment parameters
- `config/mighty_ground_robot.yaml` — Ground robot parameters
- Launch files in `launch/` orchestrate multi-node setups

### Data Flow

1. `mighty_node` receives state (odometry), point cloud, and goal
2. `dgp_manager` updates the voxel map and computes convex free-space corridors
3. `mighty` generates a Hermite spline trajectory within corridors using L-BFGS optimization
4. Trajectory is published for the controller (or `fake_sim` in simulation)
