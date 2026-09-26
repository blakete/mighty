# Per-vehicle parameter overlays

`launch/mighty_hw.launch.py` layers the planner parameters, later files winning:

1. `config/mighty.yaml` (shared base)
2. `config/platforms/<platform>.yaml` (`ground_robot` or `uav`)
3. `config/vehicles/<ROBOT_NAME>.yaml` (this directory, optional)
4. `map_frame_id` = `<ROBOT_NAME>/map`

An overlay holds only the keys that differ on that vehicle. A `mighty_node:`
section overrides planner parameters; an optional `mpc:` section overrides the
MPC's `mpc.yaml` (ground robots):

```yaml
# config/vehicles/RR08.yaml
mighty_node:
  ros__parameters:
    exploration.enabled: true   # drive to frontiers autonomously
mpc:
  ros__parameters:
    v_max: 0.4
```

`config/` is bind-mounted from the checkout into the container, so an edit
takes effect when that node's pane restarts (Ctrl-C, Up, Enter); no image
rebuild. A different file can be named with `MIGHTY_VEHICLE_CONFIG` in
`/etc/mighty/mighty.env`.

Overlays are git-ignored so field tuning never dirties a vehicle's checkout.
To share one, `git add -f config/vehicles/<ROBOT_NAME>.yaml`.

A vehicle whose name does not end in digits must set its agent id here:
`mighty_node: ros__parameters: agent_id: 7`.
