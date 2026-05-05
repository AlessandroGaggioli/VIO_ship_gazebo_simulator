import os
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, LogInfo, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xml.etree.ElementTree as ET

def generate_launch_description():

    odom_type = 'loosely'
    disparity_viewer = False 
    camera_viewer = False

    for arg in sys.argv:
        if 'odom_type' in arg:
            odom_type = arg.split(':=')[1]
        elif 'disparity_viewer' in arg:
            disparity_viewer = arg.split(':=')[1].lower() == 'true'
        elif 'camera_viewer' in arg:
            camera_viewer = arg.split(':=')[1].lower() == 'true'

    odom_comparator_node = Node(
        package='ship_gazebo',
        executable='odom_comparator.py',
        name='odom_comparator',
        parameters=[{
            'gt_robot_topic': '/robot/ground_truth/odom',
            'ship_joints_topic': '/ship/joint_states',
            'est_topic': '/stereo_odom' if odom_type == 'loosely' else '/stereo_odom_sync',
            'sync_slop': 0.05
        }],
        output='screen'
    )

    imu_comparator_node = Node(
        package='ship_gazebo',
        executable='imu_comparator.py',
        name='imu_comparator',
        output='screen'
    )

    pkg_share = get_package_share_directory('ship_gazebo') # get the share directory of the ship_gazebo package
    camera_params_path = os.path.join(pkg_share, 'config', 'stereo_params.yaml') # path to the camera parameters file

    stereo_view = Node(
        package='image_view',
        executable='stereo_view',
        name='stereo_view_node',
        remappings=[
            ('stereo/left/image', '/camera/left/image_rect'),
            ('stereo/right/image', '/camera/right/image_rect'),
            ('stereo/disparity', '/debug/stereo/disparity'),
        ],
        parameters=[camera_params_path, {'use_sim_time': True}],
        output='screen'
    )

    rqt_image_view1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/camera/left/image_rect'
        ],
        output='screen'
    )

    rqt_image_view2 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/camera/first_int_corridor_camera/image'
        ],
        output='screen'
    )

    nodes_list = [
        DeclareLaunchArgument('odom_type', default_value='loosely'),
        DeclareLaunchArgument('disparity_viewer', default_value='false'),
        DeclareLaunchArgument('camera_viewer', default_value='false'),

        odom_comparator_node,
        imu_comparator_node
    ]

    if disparity_viewer:
        nodes_list.append(stereo_view)

    if camera_viewer:
        nodes_list.append(rqt_image_view1)
        nodes_list.append(rqt_image_view2)

    return LaunchDescription(nodes_list)




