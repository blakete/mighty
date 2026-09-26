"""MIGHTY on one vehicle: the planner, a state adapter, and (ground robots) the MPC.

    ros2 launch mighty mighty_hw.launch.py namespace:=RR08 platform:=ground_robot \
        odom_topic:=dlio/odom_node/odom

docker/mighty_hw.sh runs this once per node (only_nodes:=...), one tmux pane
each, with the arguments taken from /etc/mighty/mighty.env.

Planner parameters are layered, later files winning:
    config/mighty.yaml                      shared base
    config/platforms/<platform>.yaml        ground_robot | uav
    config/vehicles/<namespace>.yaml        optional per-vehicle overlay
    map_frame_id                            <namespace>/map unless overridden
A vehicle overlay's `mighty_node:` section overrides planner parameters and its
optional `mpc:` section overrides the MPC's mpc.yaml (e.g. v_max). The config
directory is bind-mounted from the checkout, so a parameter change is an edit
plus a restart of that node's pane — no image rebuild.

Interface (all topics relative to the namespace):
    in   state source   odom_topic (nav_msgs/Odometry), or pose_topic +
                        twist_topic (PoseStamped + TwistStamped) for mocap
    in   term_goal      geometry_msgs/PoseStamped (frame_id is ignored:
                        coordinates are read in <namespace>/map)
    in   maps           ground_robot: occ_2d_topic, esdf_2d_topic,
                        planning_occ_2d_topic (nav_msgs/OccupancyGrid)
                        uav: occupancy_grid, unknown_grid (PointCloud2)
    out  ground_robot   cmd_vel_auto (geometry_msgs/Twist, from the MPC)
    out  uav            goal (dynus_interfaces/Goal), trajectory
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORMS = ('ground_robot', 'uav')
STATE_SOURCES = ('odom', 'mocap')


def _section(path, key):
    """ros__parameters of one node section of a parameter file ({} if absent)."""
    with open(path, 'r') as f:
        doc = yaml.safe_load(f) or {}
    return dict((doc.get(key) or {}).get('ros__parameters') or {})


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    namespace = arg('namespace')
    platform = arg('platform')
    state_source = arg('state_source')
    if not namespace:
        raise RuntimeError('namespace:= is required (the vehicle name, e.g. RR08)')
    if platform not in PLATFORMS:
        raise RuntimeError(f'platform:={platform!r} must be one of {PLATFORMS}')
    if state_source not in STATE_SOURCES:
        raise RuntimeError(f'state_source:={state_source!r} must be one of {STATE_SOURCES}')

    config_dir = os.path.join(get_package_share_directory('mighty'), 'config')

    # Planner parameters: base -> platform -> vehicle -> launch-derived.
    parameters = _section(os.path.join(config_dir, 'mighty.yaml'), 'mighty_node')
    parameters.update(_section(os.path.join(config_dir, 'platforms', f'{platform}.yaml'),
                               'mighty_node'))
    vehicle_config = arg('vehicle_config')
    if vehicle_config and not os.path.isabs(vehicle_config):
        vehicle_config = os.path.join(config_dir, 'vehicles', vehicle_config)
    if vehicle_config and not os.path.isfile(vehicle_config):
        raise RuntimeError(f'vehicle_config:={vehicle_config} does not exist')
    if not vehicle_config:
        default_vehicle = os.path.join(config_dir, 'vehicles', f'{namespace}.yaml')
        vehicle_config = default_vehicle if os.path.isfile(default_vehicle) else ''
    mpc_overrides = {}
    if vehicle_config:
        parameters.update(_section(vehicle_config, 'mighty_node'))
        mpc_overrides.update(_section(vehicle_config, 'mpc'))
    parameters['map_frame_id'] = arg('map_frame_id') or f'{namespace}/map'
    # rclpy cannot infer the type of an empty list in dict-form parameters; drop
    # them so the node falls back to its declared default.
    parameters = {k: v for k, v in parameters.items()
                  if not (isinstance(v, list) and len(v) == 0)}

    log_level = ['--ros-args', '--log-level', arg('log_level') or 'error']
    pose_topic = arg('pose_topic') or ('pose' if state_source == 'odom' else 'world')

    mighty_node = Node(
        package='mighty', executable='mighty', name='mighty_node', namespace=namespace,
        output='screen', emulate_tty=True, parameters=[parameters], arguments=log_level)

    # State adapters: whatever the localization publishes -> <ns>/state (+ <ns>/pose).
    odom_to_state = Node(
        package='mighty', executable='convert_odom_to_state', name='convert_odom_to_state',
        namespace=namespace, output='screen', emulate_tty=True,
        remappings=[('odom', arg('odom_topic') or 'odom')])
    mocap_to_state = Node(
        package='mighty', executable='convert_vicon_to_state', name='convert_vicon_to_state',
        namespace=namespace, output='screen', emulate_tty=True,
        remappings=[('world', pose_topic), ('twist', arg('twist_topic') or 'twist')])

    nodes = {
        'mighty_node': mighty_node,
        'convert_odom_to_state': odom_to_state,
        'convert_vicon_to_state': mocap_to_state,
    }
    started = ['mighty_node',
               'convert_odom_to_state' if state_source == 'odom' else 'convert_vicon_to_state']

    if platform == 'ground_robot':
        # MPC tracks the planner's mpc_waypoints and publishes cmd_vel_auto. Its
        # tunables live in the mpc package's mpc.yaml; the vehicle overlay's mpc:
        # section and these two topics override it.
        mpc_overrides.update({'cmd_vel_topic': 'cmd_vel_auto', 'pose_topic': pose_topic})
        nodes['mpc'] = Node(
            package='mpc', executable='mpc_node', name='mpc', namespace=namespace,
            output='screen',
            parameters=[os.path.join(get_package_share_directory('mpc'), 'config', 'mpc.yaml'),
                        mpc_overrides])
        started.append('mpc')

    # Optional node subset, used by docker/mighty_hw.sh to give each node its own
    # tmux pane and process. Keys are the nodes' ROS names.
    only_nodes = [k.strip() for k in arg('only_nodes').split(',') if k.strip()]
    if only_nodes:
        unknown = [k for k in only_nodes if k not in nodes]
        if unknown:
            raise RuntimeError(f'only_nodes:= unknown key(s) {unknown}; valid keys: {sorted(nodes)}')
        picked = [k for k in only_nodes if k in started]
        if not picked:
            # An empty launch exits 0 silently, which in a tmux pane looks exactly
            # like a healthy node. Fail loudly instead.
            raise RuntimeError(f'only_nodes:={",".join(only_nodes)} selects nothing this '
                               f'configuration starts (platform={platform}, '
                               f'state_source={state_source}); available: {started}')
        started = picked

    print(f'[mighty_hw.launch] {namespace}: platform={platform} state_source={state_source} '
          f'vehicle_config={vehicle_config or "(none)"} nodes={started}', flush=True)
    return [nodes[k] for k in started]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='Vehicle name, e.g. RR08 (required; must end in '
                                          'digits unless the agent_id parameter is set)'),
        DeclareLaunchArgument('platform', default_value='',
                              description=f'Required: one of {PLATFORMS}'),
        DeclareLaunchArgument('state_source', default_value='odom',
                              description='odom: nav_msgs/Odometry on odom_topic; '
                                          'mocap: PoseStamped + TwistStamped'),
        DeclareLaunchArgument('odom_topic', default_value='odom',
                              description='Odometry topic (state_source:=odom), relative to '
                                          'the namespace unless it starts with /'),
        DeclareLaunchArgument('pose_topic', default_value='',
                              description='MPC pose input and mocap pose input; default '
                                          '"pose" (published by convert_odom_to_state) for '
                                          'odom, "world" for mocap'),
        DeclareLaunchArgument('twist_topic', default_value='twist',
                              description='Mocap twist topic (state_source:=mocap)'),
        DeclareLaunchArgument('vehicle_config', default_value='',
                              description='Per-vehicle parameter overlay: a file name in '
                                          'config/vehicles/ or an absolute path; default '
                                          'config/vehicles/<namespace>.yaml if it exists'),
        DeclareLaunchArgument('map_frame_id', default_value='',
                              description='Planner map frame; default <namespace>/map'),
        DeclareLaunchArgument('log_level', default_value='error',
                              description='mighty_node log level'),
        DeclareLaunchArgument('only_nodes', default_value='',
                              description='Comma-separated subset of: mighty_node, '
                                          'convert_odom_to_state, convert_vicon_to_state, mpc'),
        OpaqueFunction(function=launch_setup),
    ])
