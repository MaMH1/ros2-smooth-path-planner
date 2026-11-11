from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('smooth_path_planner'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(share / 'maps/mymap.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map'),
                          'frame_id': 'map', 'use_sim_time': False}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'costmap_demo_base']),
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d',
             parameters=[str(share / 'config/costmap.yaml')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='map_lifecycle_manager',
             parameters=[{'autostart': True, 'bond_timeout': 0.0, 'node_names': ['/map_server', '/costmap/costmap']}], output='screen'),
        Node(package='smooth_path_planner', executable='planner_node', name='smooth_planner',
             parameters=[str(share / 'config/planner.yaml')], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/planner.rviz')], output='screen'),
    ])
