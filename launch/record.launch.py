"""
Launch file for recording a rosbag from the ship simulation.

Starts Gazebo, the ROS-Gazebo bridge, and the sensor preprocessing pipeline
(camera rectification, stereo synchronization), then begins recording all
topics to a rosbag.

IMU compensation is NOT applied here — it runs at replay time so it can be
re-tuned without re-recording.

After launching, start the robot navigation separately:
  ros2 run ship_gazebo cmd_vel_pub.py
  or use teleop_twist_keyboard.

Args:
  motion : true | false  (default: true)  — enable ship roll/pitch/heave motion
  bag    : output bag name (default: bag_motion_<timestamp> or bag_no_motion_<timestamp>)
"""

import os
import sys
import subprocess
from datetime import datetime
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    motion   = True
    bag_name = None

    subprocess.run(['pkill', '-f', 'gz sim'], check=False)
    subprocess.run(['pkill', '-f', 'ros_gz_bridge.*parameter_bridge'], check=False)

    for arg in sys.argv:
        if 'motion:=' in arg:
            motion = arg.split(':=')[1].lower() not in ('false', '0', 'no')
        if 'bag:=' in arg:
            bag_name = arg.split(':=')[1]

    roll_amplitude  = 0.2 if motion else 0.0
    pitch_amplitude = 0.1 if motion else 0.0
    heave_amplitude = 0.1 if motion else 0.0
    mode_label      = 'motion' if motion else 'no_motion'

    BAGS_DIR = os.path.join(os.path.expanduser('~/ship_ws'), 'bags')
    os.makedirs(BAGS_DIR, exist_ok=True)

    if bag_name is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f'bag_{mode_label}_{timestamp}'

    bag_output = os.path.join(BAGS_DIR, bag_name)

    for folder in [os.path.expanduser('~/.gz/sim'), os.path.expanduser('~/.ignition/sim')]:
        if os.path.exists(folder):
            for root, dirs, files in os.walk(folder):
                if 'gui.config' in files:
                    try:
                        os.remove(os.path.join(root, 'gui.config'))
                    except Exception as e:
                        print(f"error cleaning up: {e}")

    pkg_share  = get_package_share_directory('ship_gazebo')
    pkg_prefix = get_package_prefix('ship_gazebo')
    plugin_path        = os.path.join(pkg_prefix, 'lib', 'ship_gazebo')
    sdf_dir            = os.path.join(pkg_share, 'worlds')
    sdf_em             = os.path.join(sdf_dir, 'ship_world_dynamic.sdf.em')
    sdf_out            = os.path.join(sdf_dir, 'ship_world_dynamic.sdf')
    bridge_config      = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    camera_params_path = os.path.join(pkg_share, 'config', 'stereo_params.yaml')

    subprocess.run(
        ['python3', '-m', 'em',
         '-D', f'roll_amplitude={roll_amplitude}',
         '-D', f'pitch_amplitude={pitch_amplitude}',
         '-D', f'heave_amplitude={heave_amplitude}',
         '-o', sdf_out, sdf_em],
        check=True
    )

    def get_robot_spawn_from_sdf(sdf_path):
        try:
            with open(sdf_path, 'r') as f:
                lines = f.readlines()
            xml_content = ''
            for line in lines:
                if '<?xml' in line or xml_content:
                    xml_content += line
            root = ET.fromstring(xml_content)
            for include_tag in root.findall('.//include'):
                name_tag = include_tag.find('name')
                if name_tag is not None and 'turtlebot3_waffle' in name_tag.text:
                    coords = [float(x) for x in include_tag.find('pose').text.split()]
                    return coords[0], coords[1], coords[2]
        except Exception as e:
            print(f"error reading sdf: {e}")
        return 0.0, 0.0, 0.0

    sx, sy, sz = get_robot_spawn_from_sdf(sdf_out)

    print(f"[record] motion={motion} | bag={bag_output} | spawn=({sx},{sy},{sz})")
    print("[record] Start navigation in another terminal when ready.")

    #===========================================================================
    # Static TFs
    #===========================================================================

    base_link_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_tf',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '0', '--y', '0', '--z', '0.010',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    )

    camera_link_left_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_link_left_base_tf',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '0.15', '--y', '0.06', '--z', '0.1',
                   '--roll', '0', '--pitch', '0.2', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link_left']
    )

    camera_link_right_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_link_right_base_tf',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '0.15', '--y', '-0.06', '--z', '0.1',
                   '--roll', '0', '--pitch', '0.2', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link_right']
    )

    left_camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_left_tf',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.570796', '--pitch', '0', '--yaw', '-1.570796',
                   '--frame-id', 'camera_link_left',
                   '--child-frame-id', 'ship_corridor_dynamic/turtlebot3_waffle/camera_link_left/stereo_camera_left']
    )

    right_camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_right_tf',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.570796', '--pitch', '0', '--yaw', '-1.570796',
                   '--frame-id', 'camera_link_right',
                   '--child-frame-id', 'ship_corridor_dynamic/turtlebot3_waffle/camera_link_right/stereo_camera_right']
    )

    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='imu_tf',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '-0.032', '--y', '0', '--z', '0.078',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'imu_link']
    )

    #===========================================================================
    # Camera rectification
    #===========================================================================

    left_camera_rect = Node(
        package='image_proc', executable='rectify_node', name='rectify_left',
        remappings=[
            ('image',      '/camera/left/image_raw'),
            ('camera_info','/camera/left/camera_info'),
            ('image_rect', '/camera/left/image_rect'),
        ],
        parameters=[camera_params_path, {'use_sim_time': True}],
        output='screen'
    )

    right_camera_rect = Node(
        package='image_proc', executable='rectify_node', name='rectify_right',
        remappings=[
            ('image',      '/camera/right/image_raw'),
            ('camera_info','/camera/right/camera_info'),
            ('image_rect', '/camera/right/image_rect'),
        ],
        parameters=[camera_params_path, {'use_sim_time': True}],
        output='screen'
    )

    #===========================================================================
    # Stereo synchronization
    #===========================================================================

    stereo_sync_node = Node(
        package='rtabmap_sync', executable='stereo_sync', name='stereo_sync',
        output='screen',
        parameters=[{
            'use_sim_time':            True,
            'approx_sync':             True,
            'approx_sync_max_interval': 0.01,
            'sync_queue_size':         10,
        }],
        remappings=[
            ('left/image_rect',  '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info','/camera/right/camera_info'),
            ('rgbd_image',       '/stereo_camera/rgbd_image'),
        ]
    )

    #===========================================================================
    # Bag recording — started after all preprocessing nodes are up
    #===========================================================================

    CAMERA_STARTUP = 2.5
    SYNC_STARTUP   = CAMERA_STARTUP + 1.0
    BAG_STARTUP    = SYNC_STARTUP + 1.0

    RECORD_TOPICS = [
        '/clock',
        '/tf_static',
        '/stereo_camera/rgbd_image',    # pre-synchronized stereo output — only input needed for replay
        '/robot/imu/raw',               # compensated at replay time by imu_compensator
        '/ship/imu/raw',
        '/ship/joint_states',
        '/robot/odom',
        '/robot/ground_truth/odom',     # also used to auto-detect spawn position at replay
        '/cmd_vel',
    ]

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--topics'] + RECORD_TOPICS + ['-o', bag_output],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('motion', default_value='true',
                              description='Enable ship roll/pitch/heave motion: true | false'),
        DeclareLaunchArgument('bag',    default_value='',
                              description='Output bag name (default: bag_motion_<timestamp>)'),

        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', '/opt/ros/jazzy/share/turtlebot3_gazebo/models'),

        ExecuteProcess(cmd=['gz', 'sim', '-r', sdf_out], name='gazebo_simulator', output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '--ros-args', '-p', f'config_file:={bridge_config}', '-p', 'use_sim_time:=true'],
            output='screen'
        ),

        TimerAction(period=CAMERA_STARTUP, actions=[
            base_link_tf, camera_link_left_tf, camera_link_right_tf,
            left_camera_tf, right_camera_tf, imu_tf,
            left_camera_rect, right_camera_rect,
        ]),
        TimerAction(period=SYNC_STARTUP, actions=[stereo_sync_node]),
        TimerAction(period=BAG_STARTUP,  actions=[bag_record]),
    ])
