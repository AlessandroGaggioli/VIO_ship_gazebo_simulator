"""
Launch file for replaying a recorded rosbag and running stereo odometry + SLAM.

Raw sensor data (raw IMU, rectified images, stereo sync output, ground truth)
is sourced from the bag — no Gazebo needed. The /tf_static transforms are also
restored from the bag. IMU compensation is applied here at replay time, so bags
always store raw IMU and compensation is re-applied on every replay.

The bag play starts immediately so /clock is published. Processing nodes are started
via TimerActions (wall time). The idle section at the beginning of the bag (before
navigation starts) gives nodes time to initialize.

Args:
  bag       : path to the rosbag directory (required)
  odom_type : loosely | ekf  (default: loosely)
  comp      : true | false   (default: true)  — enable IMU compensation

The robot spawn position for IMU compensation is read automatically from the
bag's first /robot/ground_truth/odom message, so it always matches the recording.

Example:
  ros2 launch ship_gazebo replay.launch.py bag:=motion_line odom_type:=loosely
"""

import os
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node


def get_spawn_from_bag(bag_path):
    """Read the robot's initial position from the bag's ground truth odometry."""
    try:
        with open(os.path.join(bag_path, 'metadata.yaml')) as f:
            meta = yaml.safe_load(f)
        storage_id = meta['rosbag2_bagfile_information']['storage_identifier']

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
            rosbag2_py.ConverterOptions('', ''),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/robot/ground_truth/odom']))
        if reader.has_next():
            _, data, _ = reader.read_next()
            pos = deserialize_message(data, Odometry).pose.pose.position
            return pos.x, pos.y, pos.z
    except Exception as e:
        print(f"[replay] Warning: could not read spawn from bag: {e}")
    return 0.0, 0.0, 0.0


def launch_setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag').perform(context)
    odom_type = LaunchConfiguration('odom_type').perform(context)
    comp = LaunchConfiguration('comp').perform(context).lower() not in ('false', '0', 'no')
    _3dof = LaunchConfiguration('3dof').perform(context).lower() not in ('false', '0', 'no')

    if not bag_path:
        raise ValueError(
            "bag:= argument is required.\n"
            "Example: ros2 launch ship_gazebo replay.launch.py bag:=bag_navigation_20250507_120000"
        )

    # Resolve bag name against ~/ship_ws/bags if not an absolute/existing path
    if not os.path.isabs(bag_path) and not os.path.exists(bag_path):
        bag_path = os.path.join(os.path.expanduser('~/ship_ws'), 'bags', bag_path)
    if odom_type not in ('loosely', 'ekf'):
        raise ValueError(f"Unknown odom_type: {odom_type}. Allowed: loosely, ekf")

    spawn_x, spawn_y, spawn_z = get_spawn_from_bag(bag_path)

    pkg_share              = get_package_share_directory('ship_gazebo')
    rtabmap_params_path    = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    ekf_odom_params_path   = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')
    rviz2_config           = os.path.join(pkg_share, 'config', 'rtabmap.rviz')

    print(f"[replay] bag={bag_path} | odom_type={odom_type} | comp={comp} | spawn=({spawn_x},{spawn_y},{spawn_z})")

    #===========================================================================
    # Static TFs — replicated here because tf_static QoS replay from bags is
    # unreliable (TRANSIENT_LOCAL durability not guaranteed by ros2 bag play)
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
    # RGB relay — extracts sensor_msgs/Image from RGBDImage for rqt_image_view
    #===========================================================================

    rgbd_relay = Node(
        package='ship_gazebo', executable='rgbd_image_relay.py', name='rgbd_image_relay',
        parameters=[{'use_sim_time': True}]
    )

    #===========================================================================
    # IMU compensator — applied at replay time so bags store raw IMU and
    # compensation can be re-tuned without re-recording.
    #===========================================================================

    imu_compensator = Node(
        package='ship_gazebo', executable='imu_compensator.py', name='imu_compensator',
        parameters=[{
            'enable':       comp,
            'spawn_x':      spawn_x,
            'spawn_y':      spawn_y,
            'spawn_z':      spawn_z,
            'use_sim_time': True,
        }]
    )

    #===========================================================================
    # Bag playback — replays the recorded /clock (Gazebo sim time) so all
    # nodes with use_sim_time:=True get the correct sim-time clock.
    # Do NOT use --clock here: that flag generates a NEW /clock from bag
    # metadata (wall-time values), which causes RTAB-Map to stamp TF at
    # wall time instead of sim time.
    #===========================================================================

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen'
    )

    #===========================================================================
    # Stereo odometry
    #===========================================================================

    stereo_odom_remappings = [('rgbd_image', '/stereo_camera/rgbd_image')]

    if odom_type == 'loosely':
        stereo_odom_remappings += [
            ('odom',          '/stereo_odom'),
            ('odom_local_map','/stereo_odom_local_map'),
            ('imu',           '/robot/imu/compensated'),
        ]
        stereo_odom_params = {
            'subscribe_imu':    True,
            'wait_imu_to_init': True,
            'guess_from_tf':    False,
            'publish_tf':       True,
            'odom_frame_id':    'stereo_odom',
            'Reg/Force3DoF':     'false' if not _3dof else 'true',
        }
    else:  # ekf
        stereo_odom_remappings += [
            ('odom',          '/stereo_odom'),
            ('odom_local_map','/stereo_odom_local_map'),
        ]
        stereo_odom_params = {
            'subscribe_imu':    False,
            'wait_imu_to_init': False,
            'guess_frame_id':   '',
            'publish_tf':       False,
            'odom_frame_id':    'stereo_odom',
            'Reg/Force3DoF':     'false' if not _3dof else 'true',
            'RGBD/ForceOdom3DoF': 'false' if not _3dof else 'true',
        }

    stereo_odometry_node = Node(
        package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry',
        output='screen',
        parameters=[rtabmap_params_path, {'use_sim_time': True}, stereo_odom_params],
        remappings=stereo_odom_remappings
    )

    #===========================================================================
    # EKF (only for odom_type == 'ekf')
    #===========================================================================

    ekf_odom_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[ekf_odom_params_path, {'use_sim_time': True}]
    )

    #===========================================================================
    # RTAB-Map SLAM
    #===========================================================================

    rtabmap_remappings = [
        ('rgbd_image', '/stereo_camera/rgbd_image'),
        ('imu',        '/robot/imu/compensated'),
        ('odom',       '/odometry/filtered' if odom_type == 'ekf' else '/stereo_odom'),
    ]

    rtabmap_slam_node = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=[rtabmap_params_path, {'use_sim_time': True}, {
            'Reg/Force3DoF': 'false' if not _3dof else 'true',
            'RGBD/ForceOdom3DoF': 'false' if not _3dof else 'true',
        }],
        remappings=rtabmap_remappings,
        arguments=['-d']
    )

    #===========================================================================
    # RViz2
    #===========================================================================

    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': True}]
    )

    #===========================================================================
    # Timing: bag plays immediately, nodes start via wall-time TimerActions.
    # The bag's idle preamble (Gazebo startup before navigation) covers warmup.
    #===========================================================================

    TF_STARTUP   = 1.0
    ODOM_STARTUP = 3.0
    SLAM_STARTUP = ODOM_STARTUP + 2.0

    nodes_list = [
        bag_play,
        rviz2,
        TimerAction(period=TF_STARTUP, actions=[
            base_link_tf, camera_link_left_tf, camera_link_right_tf,
            left_camera_tf, right_camera_tf, imu_tf,
            imu_compensator, rgbd_relay,
        ]),
        TimerAction(period=ODOM_STARTUP, actions=[stereo_odometry_node]),
        TimerAction(period=SLAM_STARTUP, actions=[rtabmap_slam_node]),
    ]

    if odom_type == 'ekf':
        nodes_list.append(TimerAction(period=ODOM_STARTUP - 1.0, actions=[ekf_odom_node]))

    return nodes_list

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag',       default_value='',
                              description='Path to the rosbag directory to replay'),
        DeclareLaunchArgument('odom_type', default_value='loosely',
                              description='Odometry type: loosely | ekf'),
        DeclareLaunchArgument('comp',      default_value='true',
                              description='Enable IMU compensation: true | false'),
        DeclareLaunchArgument('3dof',   default_value='false',
                              description='Enable 3DOF mode (planar movement only): true | false'),
        
        OpaqueFunction(function=launch_setup)
    ])
