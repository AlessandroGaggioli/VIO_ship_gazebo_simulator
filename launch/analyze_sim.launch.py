"""
Launch file for analyzing odometry and IMU data from a rosbag simulation.

This launch file:
1. Launches odom_comparator to compare ground truth and estimated odometry
2. Optionally launches imu_comparator to compare raw and compensated IMU data
3. Saves results (PDF and CSV) to a specified directory

Parameters:
    - save_path: Directory path where output files will be saved
    - filename: Base filename for output files (e.g., 'report')
    - save_odom: Save odometry results (true/false, default: true)
    - save_imu: Save IMU results (true/false, default: false)
    - save_map: Save SLAM map + compute map metrics (true/false, default: false)

Generated files:
    - If save_odom=true: {filename}_odom.pdf, {filename}_odom.csv
    - If save_imu=true: {filename}_imu.pdf, {filename}_imu.csv
    - If save_map=true: ~/ship_ws/maps/{filename}.pgm, {filename}.yaml,
                       {filename}_cropped.pgm, and a row appended to
                       {save_path}/map_metrics.csv

Usage:
    ros2 launch ship_gazebo analyze_sim.launch.py \\
        save_path:=/home/user/results \\
        filename:=report \\
        save_odom:=true \\
        save_imu:=true \\
        save_map:=true
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes_and_processes(context, *args, **kwargs):
    """
    Callback function to build nodes and processes with resolved paths.
    Generates filenames automatically from the base filename and save flags.
    """
    save_path = context.launch_configurations.get('save_path', './analysis_results')
    filename = context.launch_configurations.get('filename', 'report')
    save_odom_str = context.launch_configurations.get('save_odom', 'true').lower()
    save_imu_str = context.launch_configurations.get('save_imu', 'false').lower()
    save_map_str = context.launch_configurations.get('save_map', 'false').lower()
    est_topic = context.launch_configurations.get('est_topic', 'stereo_odom')

    save_odom = save_odom_str in ['true', 'True', '1', 'yes']
    save_imu = save_imu_str in ['true', 'True', '1', 'yes']
    save_map = save_map_str in ['true', 'True', '1', 'yes']

    # Build full paths with generated filenames
    odom_pdf_path = os.path.join(save_path, f'{filename}_odom.pdf') if save_odom else ''
    odom_csv_path = os.path.join(save_path, f'{filename}_odom.csv') if save_odom else ''
    imu_pdf_path = os.path.join(save_path, f'{filename}_imu.pdf') if save_imu else ''
    imu_csv_path = os.path.join(save_path, f'{filename}_imu.csv') if save_imu else ''

    entities = []

    # Ensure output directory exists
    Path(save_path).mkdir(parents=True, exist_ok=True)

    # Odom Comparator Node (always launch, but only save if save_odom is true)
    odom_comparator_node = Node(
        package='ship_gazebo',
        executable='odom_comparator.py',
        name='odom_comparator',
        output='screen',
        parameters=[
            {'gt_robot_topic': '/robot/ground_truth/odom'},
            {'est_topic': est_topic},
            {'csv_output_path': odom_csv_path},
            {'pdf_output_path': odom_pdf_path},
            {'use_sim_time': True},
        ]
    )
    entities.append(odom_comparator_node)

    # IMU Comparator Node (always launch, but only save if save_imu is true)
    imu_comparator_node = Node(
        package='ship_gazebo',
        executable='imu_comparator.py',
        name='imu_comparator',
        output='screen',
        parameters=[
            {'csv_output_path': imu_csv_path},
            {'pdf_output_path': imu_pdf_path},
            {'use_sim_time': True},
        ]
    )
    entities.append(imu_comparator_node)

    # Map Analyzer Node (only launch if save_map is true)
    if save_map:
        map_analyzer_node = Node(
            package='ship_gazebo',
            executable='map_analyzer.py',
            name='map_analyzer',
            output='screen',
            parameters=[
                {'map_name': filename},
                {'maps_dir': os.path.expanduser('~/ship_ws/maps')},
                {'save_path': save_path},
                {'use_sim_time': True},
            ]
        )
        entities.append(map_analyzer_node)

    rqt_image_viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_viewer',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ],
        arguments=['/stereo_camera/image_rgb']
    )
    entities.append(rqt_image_viewer_node)

    return entities


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ship_gazebo')

    # Declare launch arguments
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='./analysis_results',
        description='Directory path where output files will be saved'
    )
    
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value='report',
        description='Base filename for output files (e.g., "report")'
    )
    
    save_odom_arg = DeclareLaunchArgument(
        'save_odom',
        default_value='false',
        description='Save odometry results (true/false)'
    )
    
    save_imu_arg = DeclareLaunchArgument(
        'save_imu',
        default_value='false',
        description='Save IMU results (true/false)'
    )

    save_map_arg = DeclareLaunchArgument(
        'save_map',
        default_value='false',
        description='Save SLAM map and compute map metrics (true/false)'
    )

    est_topic_arg = DeclareLaunchArgument(
        'est_topic',
        default_value='stereo_odom',
        description='Estimated odometry topic name (default: stereo_odom)'
    )

    # Create launch description with the arguments
    ld = LaunchDescription([
        save_path_arg,
        filename_arg,
        save_odom_arg,
        save_imu_arg,
        save_map_arg,
        est_topic_arg,
    ])

    # Add launch info message
    launch_info = LogInfo(
        msg=[
            "\n",
            "============================================================\n",
            "Odometry & IMU Analysis Launch\n",
            "============================================================\n",
            "Save path: ",
            LaunchConfiguration('save_path'),
            "\n",
            "Base filename: ",
            LaunchConfiguration('filename'),
            "\n",
            "Estimated odometry topic: ",
            LaunchConfiguration('est_topic'),
            "\n",
            "Save odometry results: ",
            LaunchConfiguration('save_odom'),
            "\n",
            "Save IMU results: ",
            LaunchConfiguration('save_imu'),
            "\n",
            "Save map results: ",
            LaunchConfiguration('save_map'),
            "\n",
            "============================================================\n",
            "Output files will be generated as:\n",
            "  - Odometry: {filename}_odom.pdf, {filename}_odom.csv\n",
            "  - IMU: {filename}_imu.pdf, {filename}_imu.csv\n",
            "  - Map: ~/ship_ws/maps/{filename}.pgm + .yaml,\n",
            "         row appended to {save_path}/map_metrics.csv\n",
            "============================================================\n",
            "Now please launch the rosbag replay in another terminal:\n",
            "ros2 launch ship_gazebo replay.launch bag:=<bag_name>\n"
            "odom_type:=<loosely/ekf> [default: loosely] \n"
            "comp:=<true/false> [default: true] \n"
            "3dof:=<true/false> [default: false]\n"
            "============================================================\n",
        ]
    )
    ld.add_action(launch_info)

    # Add the opaque function that will create nodes/processes based on config
    ld.add_action(OpaqueFunction(function=_build_nodes_and_processes))

    return ld

