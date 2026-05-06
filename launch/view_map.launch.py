import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    base_path = os.path.expanduser('~/ship_ws/maps/')

    map_subpath_arg = DeclareLaunchArgument(
        'map',
        default_value='map.yaml',
        description='final part of the map path (e.g. maps/map.yaml)'
    )

    full_map_path = PathJoinSubstitution([
        base_path,
        LaunchConfiguration('map')
    ])

    return LaunchDescription([
        map_subpath_arg,

        # Nodo Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': full_map_path}]
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        )
    ])
