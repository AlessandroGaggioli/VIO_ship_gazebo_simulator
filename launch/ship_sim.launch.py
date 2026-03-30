"""
Launch file for the ship simulation in Gazebo. 
It sets up the environment, launches the Gazebo simulator with the specified world, 
and starts the necessary ROS 2 nodes for camera processing and IMU compensation.

it includes:
- Environment variable setup for Gazebo plugins and resources
- Cleanup of old Gazebo GUI config files to prevent issues with the simulator
- Execution of the Gazebo simulator with the specified SDF world
- Launching the ROS-Gazebo bridge to connect Gazebo topics to ROS 2
- Static transforms for the cameras and IMU
- Image processing nodes for rectifying camera images and generating disparity maps
- IMU compensator node to adjust IMU readings based on the robot's spawn position in the world
"""


import os
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, TimerAction, LogInfo
from launch_ros.actions import Node
import xml.etree.ElementTree as ET

def generate_launch_description():

    for folder in [os.path.expanduser('~/.gz/sim'), os.path.expanduser('~/.ignition/sim')]:
        if os.path.exists(folder):
            for root, dirs, files in os.walk(folder):
                if 'gui.config' in files:
                    try:
                        os.remove(os.path.join(root, 'gui.config'))
                        print(f"cleaning: deleted {root}/gui.config")
                    except Exception as e:
                        print(f"error cleaning up: {e}")

    pkg_share = get_package_share_directory('ship_gazebo')
    pkg_prefix = get_package_prefix('ship_gazebo') 
    plugin_path = os.path.join(pkg_prefix, 'lib', 'ship_gazebo')
    
    sdf_em = os.path.expanduser('~/ship_ws/src/ship_gazebo/worlds/ship_world_dynamic.sdf.em')
    sdf_out = os.path.expanduser('~/ship_ws/src/ship_gazebo/worlds/ship_world_dynamic.sdf')
    bridge_config = os.path.expanduser('~/ship_ws/src/ship_gazebo/config/bridge_config.yaml')
    
    subprocess.run(
        ['python3', '-c', 'import em; em.main()', '--', sdf_em],
        stdout=open(sdf_out, 'w'),
        check=True
    )

    def get_robot_spawn_from_sdf(sdf_path):
        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()

            for include_tag in root.findall(".//include"):
                name_tag = include_tag.find("name")
                if name_tag is not None and "turtlebot3_waffle" in name_tag.text:
                    pose_text = include_tag.find("pose").text
                    coords = [float(x) for x in pose_text.split()]
                    return coords[0], coords[1], coords[2]
        except Exception as e: 
            print(f"error reading sdf: {e}")
        return 0.0,0.0,0.0
    
    sx,sy,sz = get_robot_spawn_from_sdf(sdf_out)
    print(f"DEBUG: Robot spawn in the SDF -> X:{sx}, Y:{sy}, Z:{sz}")

    left_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_left_tf',
        arguments=['0.07', '0.06', '0.10', '-1.570796', '0', '-1.570796', 'base_footprint', 'turtlebot3_waffle/base_link/stereo_camera_left']
    )

    right_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_right_tf',
        arguments=['0.07', '-0.06', '0.10', '-1.570796', '0', '-1.570796', 'base_footprint', 'turtlebot3_waffle/base_link/stereo_camera_right']
    )
    
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['-0.032', '0', '0.068', '0', '0', '0', 'base_footprint', 'imu_link']
    )

    left_camera_rect = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_left',
        remappings=[
            ('image','/camera/left/image_raw'), # topic published by Gazebo
            ('camera_info','/camera/left/camera_info'),
            ('image_rect','/camera/left/image_rect') # rectified image topic
        ],
        parameters=[{
            'approximate_sync':True,
            'queue_size':50,
            'use_sim_time':True
        }],
        output='screen'
    )

    right_camera_rect = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_right',
        remappings=[
            ('image','/camera/right/image_raw'),
            ('camera_info','/camera/right/camera_info'),
            ('image_rect','/camera/right/image_rect')
        ],
        parameters=[{
            'approximate_sync':True,
            'queue_size': 50,
            'use_sim_time': True
        }],
        output='screen'
    )

    disparity_map = Node (
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_map',
        remappings=[
            ('left/image_rect','/camera/left/image_rect'),
            ('right/image_rect','/camera/right/image_rect'),
            ('left/camera_info','/camera/left/camera_info'),
            ('right/camera_info','/camera/right/camera_info'),
            ('disparity','/camera/disparity')
        ],
        parameters=[{
            'approximate_sync':True,
            'queue_size':20,
            'stereo_algorithm': 1, # 0 = BM, 1 = SGBM (more accurate)
            'correlation_window_size': 15, 
            'min_disparity': 0,
            'num_disparities': 64 # multiple of 16
        }],
        on_exit=LogInfo(msg='Disparity node closed.'),
        respawn=False,
        output='screen'
    )

    stereo_view = Node(
        package='image_view',
        executable='stereo_view',
        name='stereo_view_node',
        remappings=[
            ('stereo/left/image', '/camera/left/image_rect'),
            ('stereo/right/image', '/camera/right/image_rect'),
            ('stereo/disparity', '/camera/disparity'),
        ],
        parameters=[{
            'approximate_sync': True,
            'queue_size': 10
        }],
        output='screen'
    )

    compensate_imu = Node(
        package='ship_gazebo',
        executable='imu_compensator.py',
        name='imu_compensator',
        parameters=[{
            'enable':True,
            'spawn_x':sx,
            'spawn_y':sy,
            'spawn_z':sz,
            'use_sim_time':True
        }]
    )
    
    return LaunchDescription([

        AppendEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path,
        ),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/opt/ros/jazzy/share/turtlebot3_gazebo/models',
        ),
        
        ExecuteProcess(
            cmd=['gz', 'sim','-r', sdf_out],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '--ros-args', '-p',
                f'config_file:={bridge_config}'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2','run','rqt_image_view','rqt_image_view'],
            output='screen' 
        ),

        ExecuteProcess(
            cmd=[
                'ros2','run','rqt_image_view','rqt_image_view'],
                output='screen' 
        ),
        left_camera_tf,
        right_camera_tf,
        imu_tf,
        left_camera_rect,
        right_camera_rect,
        disparity_map,
        stereo_view,
        compensate_imu,

    ])