# Dev-only UAV simulator

Runs the real `mighty` planner binary (from `mighty-hw:local`) in a small simulated
world, so the UAV mode can be exercised without a vehicle or a recorded bag. It is
**never part of the hardware image**: it lives under `docker/`, outside the hardware
build context, and builds its own image on top of `mighty-hw:local`.

| node | package | does |
|---|---|---|
| `random_forest` | `map_generator` (uav_simulator) | random cylinders → `/map_generator/global_cloud` |
| `fake_sim` | `mighty_dev_sim` | integrates `<ns>/goal` into `<ns>/state`, TF `map -> <ns>/base_link`, `<ns>/odom` |
| `pcl_render_node` | `local_sensing` (uav_simulator) | the obstacles around the UAV → `<ns>/sensor_point_cloud` |
| `mighty_node` | `mighty` | the planner: `config/mighty.yaml` + `mighty_dev_sim/config/uav_sim.yaml` |

```bash
make -C docker hw-build            # or pull: the harness layers on mighty-hw:local
docker/dev/sim/dev_sim.sh build
docker/dev/sim/dev_sim.sh up [--rviz]
docker/dev/sim/dev_sim.sh goal 8 0 2      # /NX01/term_goal
docker/dev/sim/dev_sim.sh attach          # tmux: the launch | a goal shell
docker/dev/sim/dev_sim.sh down
```

Everything runs in one container on FastDDS restricted to localhost, domain 77: no zenoh
router, and no way to reach a vehicle or fleet graph. The planner's parameters are
bind-mounted from this checkout's `config/`, as on hardware.

Only the UAV mode is simulated. The ground-robot mode plans on 2D occupancy grids from an
elevation mapper that is not part of this harness; test it with the bag replay
(`docker/dev/replay/`, see `docker/README.hw.md`).
