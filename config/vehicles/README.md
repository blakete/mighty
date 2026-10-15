# Per-vehicle parameter overlays

On hardware, `launch/onboard_mighty.launch.py` layers each node's parameters, later winning:

| Node | Layers |
|---|---|
| `mighty_node` | `mighty_ground_robot.yaml` → `hw_mighty_ground_robot.yaml` → `vehicles/<ns>.yaml` `mighty_node:` → launch arguments |
| `goal_selector` | the merged `mighty_node` dict → `hw_goal_selector.yaml` → `vehicles/<ns>.yaml` `goal_selector:` |
| `mpc` | `mpc.yaml` → `vehicles/<ns>.yaml` `mpc:` → launch-derived topics |

`<ns>` is the robot namespace (`ROBOT_NAME`, e.g. `JK02`). A vehicle without a file runs the fleet
files unchanged. An overlay holds only the keys that should differ on (or stay pinned for) that
vehicle; spell them like the fleet files (`exploration.enabled: true`). Any other top-level section
is an error.

```yaml
# config/vehicles/JK02.yaml
mighty_node:
  ros__parameters:
    inflation_2d_m: 0.1
goal_selector:
  ros__parameters:
    exploration.enabled: true
mpc:
  ros__parameters:
    v_max: 0.5
```

`config/` is bind-mounted from the checkout into the container, so an edit takes effect when the
MIGHTY panes restart (Ctrl-C, Up, Enter); the loader itself is in the launch file, which is baked
into the image. The planner pane prints `[onboard_mighty] vehicle overlay ...` (or `no vehicle
overlay for <ns>`) at start: an image older than this loader prints neither and ignores the file.
