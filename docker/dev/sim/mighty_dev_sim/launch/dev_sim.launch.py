"""Dev-only UAV simulator for MIGHTY: a random forest, a perfect-tracking UAV and a
rendered sensor cloud around it, with the real planner binary in the loop.

    ros2 launch mighty_dev_sim dev_sim.launch.py [namespace:=NX01] [rviz:=true]

Nodes:
    random_forest   (map_generator)   /map_generator/global_cloud: random obstacles
    fake_sim        (this package)    integrates <ns>/goal into <ns>/state, TF
                                      map -> <ns>/base_link and <ns>/odom
    pcl_render_node (local_sensing)   crops the global cloud around the UAV into
                                      <ns>/sensor_point_cloud (the planner's sim input)
    mighty_node     (mighty)          the planner, parameters config/mighty.yaml +
                                      this package's config/uav_sim.yaml
    rviz2                             optional

Send a goal: the rviz 2D Goal tool, or
    ros2 topic pub --once /NX01/term_goal geometry_msgs/msg/PoseStamped \
      '{pose: {position: {x: 8.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}'
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _params(path):
    with open(path, 'r') as f:
        return dict(yaml.safe_load(f)['mighty_node']['ros__parameters'])


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = arg('namespace')
    map_size = [float(arg('map_size_x')), float(arg('map_size_y')), float(arg('map_size_z'))]
    start = [float(v) for v in arg('start').split(',')]
    share = get_package_share_directory('mighty_dev_sim')

    parameters = _params(os.path.join(get_package_share_directory('mighty'), 'config', 'mighty.yaml'))
    parameters.update(_params(os.path.join(share, 'config', 'uav_sim.yaml')))
    parameters = {k: v for k, v in parameters.items()
                  if not (isinstance(v, list) and len(v) == 0)}

    nodes = [
        Node(package='map_generator', executable='random_forest', name='random_forest',
             output='screen', remappings=[('odometry', f'/{ns}/odom')],
             parameters=[{
                 'map/x_size': map_size[0], 'map/y_size': map_size[1],
                 'map/z_size': map_size[2], 'map/resolution': 0.1,
                 'map/obs_num': int(arg('obstacles')), 'map/circle_num': 0,
                 'ObstacleShape/seed': int(arg('seed')),
                 'ObstacleShape/lower_rad': 0.8, 'ObstacleShape/upper_rad': 0.8,
                 'ObstacleShape/lower_hei': 3.0, 'ObstacleShape/upper_hei': 3.0,
                 'ObstacleShape/radius_l': 0.8, 'ObstacleShape/radius_h': 0.8,
                 'ObstacleShape/z_l': 1.0, 'ObstacleShape/z_h': 1.0,
                 'ObstacleShape/theta': 0.5, 'sensing/radius': 10.0, 'sensing/rate': 50.0,
                 'min_distance': 2.0,
             }]),
        Node(package='mighty_dev_sim', executable='fake_sim', name='fake_sim', namespace=ns,
             output='screen', emulate_tty=True,
             parameters=[{
                 'start_pos': start, 'start_yaw': 0.0, 'publish_tf': True,
                 'publish_state': True, 'use_ground_robot': False, 'publish_odom': True,
                 'odom_topic': 'odom', 'odom_frame_id': 'map',
                 'base_frame_id': f'{ns}/base_link', 'map_frame_id': 'map',
                 'visual_level': 1,
             }]),
        Node(package='local_sensing', executable='pcl_render_node', name='pcl_render_node',
             namespace=ns, output='screen',
             parameters=[
                 {'sensing_horizon': 5.0, 'sensing_rate': 30.0, 'estimation_rate': 30.0,
                  'map/x_size': map_size[0], 'map/y_size': map_size[1],
                  'map/z_size': map_size[2]},
                 os.path.join(get_package_share_directory('local_sensing'), 'config', 'camera.yaml'),
             ],
             remappings=[('global_map', '/map_generator/global_cloud'),
                         ('odometry', 'odom'), ('depth', 'pcl_render_node/depth')]),
        Node(package='mighty', executable='mighty', name='mighty_node', namespace=ns,
             output='screen', emulate_tty=True, parameters=[parameters],
             arguments=['--ros-args', '--log-level', arg('log_level')]),
    ]
    if arg('rviz').lower() in ('1', 'true', 'yes'):
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'dev_sim.rviz')]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='NX01'),
        DeclareLaunchArgument('start', default_value='0.0,0.0,2.0', description='x,y,z'),
        DeclareLaunchArgument('map_size_x', default_value='20.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_size_z', default_value='6.0'),
        DeclareLaunchArgument('obstacles', default_value='30'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='error'),
        OpaqueFunction(function=launch_setup),
    ])
