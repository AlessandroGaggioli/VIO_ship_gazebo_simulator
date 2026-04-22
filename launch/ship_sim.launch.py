"""
Launch file for the ship simulation in Gazebo. 
It sets up the environment, launches the Gazebo simulator with the specified world, 
and starts the necessary ROS 2 nodes for camera processing, IMU compensation, SLAM, 
and navigation based on the selected mode of operation (mapping or navigation).
"""


import os
import sys
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, LogInfo, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xml.etree.ElementTree as ET

#=======================================================================
# Function to get the configuration parameters based on the selected mode
#========================================================================
def get_mode_config(mode_str):
    if mode_str == 'mapping':
        return {
            'roll_amplitude': 0.0,
            'pitch_amplitude': 0.0,
            'heave_amplitude': 0.0,
            'imu_enable': True
        }
    elif mode_str == 'navigation_no_comp':
        return {
            'roll_amplitude': 0.3,
            'pitch_amplitude': 0.0,
            'heave_amplitude': 0.0,
            'imu_enable': False
        }
    elif mode_str == 'navigation':
        return {
            'roll_amplitude': 0.3,
            'pitch_amplitude': 0.0,
            'heave_amplitude': 0.0,
            'imu_enable': True
        }
    else:
        raise ValueError(f"Unknown mode: {mode_str}")
    

#=======================================================================
# Main function to generate the launch description
#=======================================================================
def generate_launch_description():

    mode_str = 'navigation'
    debug_camera = False
    odom_type = 'ekf'

    # Ensure there is a single simulation/clock source.
    # Stale gz/bridge processes from previous runs can cause /clock jumps,
    # which then break odometry, RTAB-Map and Nav2 behavior.
    subprocess.run(['pkill', '-f', 'gz sim'], check=False)
    subprocess.run(['pkill', '-f', 'ros_gz_bridge.*parameter_bridge'], check=False)

    # Parse command line arguments to get the mode settings
    for arg in sys.argv:
        if 'mode:=' in arg:
            mode_str = arg.split(':=')[1]  # get the value after ':=' and set it to mode_str
        if 'debug_camera:=' in arg:
            debug_camera = arg.split(':=')[1].lower() == 'true' # convert to boolean and set to debug_camera
        if 'odom_type:=' in arg:
            odom_type = arg.split(':=')[1] # set the odometry type based on the argument
    
    # get the configuration parameters based on the selected mode
    config = get_mode_config(mode_str) 

    # Clean up old Gazebo GUI config files to prevent issues with the simulator
    # Gazebo can sometimes crash if old GUI config files are present, so we remove them before launching the simulator
    for folder in [os.path.expanduser('~/.gz/sim'), os.path.expanduser('~/.ignition/sim')]:
        if os.path.exists(folder):
            for root, dirs, files in os.walk(folder):
                if 'gui.config' in files:
                    try:
                        os.remove(os.path.join(root, 'gui.config'))
                        print(f"cleaning: deleted {root}/gui.config")
                    except Exception as e:
                        print(f"error cleaning up: {e}")

    # Set up paths for Gazebo plugins and resources
    pkg_share = get_package_share_directory('ship_gazebo') # get the share directory of the ship_gazebo package
    pkg_prefix = get_package_prefix('ship_gazebo') # get the prefix (install) directory of the ship_gazebo package, where the compiled plugins are located
    plugin_path = os.path.join(pkg_prefix, 'lib', 'ship_gazebo') # path to the Gazebo plugins compiled from the ship_gazebo package
    sdf_dir = os.path.join(pkg_share, 'worlds') # directory where the SDF world templates are located.

    sdf_em = os.path.join(sdf_dir, 'ship_world_dynamic.sdf.em')
    sdf_out = os.path.join(sdf_dir, 'ship_world_dynamic.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    # run the EmPy command to process the SDF template and generate the final SDF file 
    # with the specified parameters for roll, pitch, and heave amplitudes
    subprocess.run(
        ['python3', '-m', 'em', 
         '-D', f'roll_amplitude={config["roll_amplitude"]}',
         '-D', f'pitch_amplitude={config["pitch_amplitude"]}',
         '-D', f'heave_amplitude={config["heave_amplitude"]}',
         '-o', sdf_out, 
         sdf_em],
        check=True
    )

    # Function to extract the robot's spawn position from the SDF file, 
    # which is needed for the IMU compensator node to adjust the IMU readings 
    # based on the robot's initial position in the world
    def get_robot_spawn_from_sdf(sdf_path):

        try:
            # Reads the file line by line and ignores initial empty lines
            with open(sdf_path, 'r') as f:
                lines = f.readlines()
            
            # Finds where the XML actually starts and joins the rest
            xml_content = ""
            for line in lines:
                if "<?xml" in line or xml_content:
                    xml_content += line
                    
            root = ET.fromstring(xml_content)

            for include_tag in root.findall(".//include"):
                name_tag = include_tag.find("name")
                if name_tag is not None and "turtlebot3_waffle" in name_tag.text:
                    pose_text = include_tag.find("pose").text
                    coords = [float(x) for x in pose_text.split()]
                    return coords[0], coords[1], coords[2]
                
        except Exception as e: 
            print(f"error reading sdf: {e}")
            
        return 0.0, 0.0, 0.0
    
    #Extract the robot's spawn position. 
    sx, sy, sz = get_robot_spawn_from_sdf(sdf_out)
    print(f"DEBUG: Robot spawn detected from SDF -> X:{sx}, Y:{sy}, Z:{sz}")

    #=======================================================
    # ROS 2 Nodes for Camera Processing and IMU Compensation
    #=======================================================

    # Camera position parameters relative to the robot's base_footprint frame.
    # These parameters define the static transforms for the left and right cameras
    # They need to be set equal to the position of the cameras in the SDF file.
    cam_x = '0.09'
    cam_y_offset = 0.06
    cam_z = '0.11'
    camera_params_path = os.path.join(pkg_share, 'config', 'stereo_params.yaml') # path to the camera parameters file

    #==============================================================================
    # Static transform from base_footprint to the left and right cameras and the IMU
    #==============================================================================
    left_camera_tf = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_left_tf',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '--x', cam_x, '--y', str(cam_y_offset), '--z', cam_z,
            '--roll', '-1.570796', '--pitch', '0', '--yaw', '-1.570796',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'ship_corridor_dynamic/turtlebot3_waffle/base_link/stereo_camera_left'
        ]
    )

    right_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_right_tf',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '--x', cam_x, '--y', str(-cam_y_offset), '--z', cam_z,
            '--roll', '-1.570796', '--pitch', '0', '--yaw', '-1.570796',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'ship_corridor_dynamic/turtlebot3_waffle/base_link/stereo_camera_right'
        ]
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'imu_link'
        ]
    )
    #==============================================================================
    #=============================================================================

    #==============================================================================
    # Nodes for rectifying the left and right camera images using the image_proc package
    #==============================================================================

    left_camera_rect = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_left',
        remappings=[
            ('image','/camera/left/image_raw'), # topic published by Gazebo
            ('camera_info','/camera/left/camera_info'),
            ('image_rect','/camera/left/image_rect') # rectified image topic
        ],
        parameters=[camera_params_path],
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
        parameters=[camera_params_path],
        output='screen'
    )

    #================================================================================
    # Node for generating the disparity map from the rectified left and right camera images using the stereo_image_proc package
    #==============================================================================
    disparity_map = Node (
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_map',
        remappings=[
            ('left/image_rect','/camera/left/image_rect'),
            ('right/image_rect','/camera/right/image_rect'),
            ('left/camera_info','/camera/left/camera_info'),
            ('right/camera_info','/camera/right/camera_info'),
            ('disparity','/debug/stereo/disparity')
        ],
        parameters=[camera_params_path],
        on_exit=LogInfo(msg='Disparity node closed.'),
        respawn=False,
        output='screen'
    )

    #==============================================================================
    # Node for visualizing the rectified stereo images and the disparity map using the stereo_view node from the image_view package
    #==============================================================================

    stereo_view = Node(
        package='image_view',
        executable='stereo_view',
        name='stereo_view_node',
        remappings=[
            ('stereo/left/image', '/camera/left/image_rect'),
            ('stereo/right/image', '/camera/right/image_rect'),
            ('stereo/disparity', '/debug/stereo/disparity'),
        ],
        parameters=[camera_params_path],
        output='screen'
    )

    rqt_image_view = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_image_view', 'rqt_image_view'
        ],
        output='screen'
    )

    #========================================================================================
    # Node for compensating the IMU readings based on the robot's spawn position in the world
    #=========================================================================================

    compensate_imu = Node(
        package='ship_gazebo',
        executable='imu_compensator.py',
        name='imu_compensator',
        parameters=[{
            'enable': config['imu_enable'],
            'spawn_x': sx,
            'spawn_y': sy,
            'spawn_z': sz,
            'use_sim_time': True
        }]
    )

    #=======================================
    # Odometry Nodes 
    #=======================================

    #=============================================
    # Stereo Synchronization Node by RTAB-Map (rtabmap_sync)
    #==============================================

    stereo_sync_node = Node(
        package='rtabmap_sync',
        executable='stereo_sync',
        name='stereo_sync',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'approx_sync': True,
            'queue_size': 10
        }],
        remappings=[
            ('left/image_rect', '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('rgbd_image', '/stereo_camera/rgbd_image') # Il nuovo topic "impacchettato"
        ]
    )

    #=========================================================================
    # EKF node for fusing wheel odometry, IMU data, and pure stereo odometry
    #=========================================================================
    ekf_odom_params_path = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')

    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_odom_params_path]
    )

    #===========================================================================
    # Node for generating odometry from the stereo camera data using the rtabmap_odom package
    #===========================================================================
    rtabmap_parameters_path = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    stereo_odom_remappings = [ # map topic to enable imu data for stereo odometry
                # ('left/image_rect', '/camera/left/image_rect'),
                # ('right/image_rect', '/camera/right/image_rect'),
                # ('left/camera_info', '/camera/left/camera_info'),
                # ('right/camera_info', '/camera/right/camera_info'),
                ('rgbd_image', '/stereo_camera/rgbd_image') # use the synchronized stereo image topic from rtabmap_sync
        ]

    if odom_type in ('loosely'):
        stereo_odom_remappings.append(('odom', '/stereo_odom'))
        stereo_odom_remappings.append(('odom_local_map', '/stereo_odom_local_map'))
        stereo_odom_remappings.append(('imu','/robot/imu/compensated'))
        stereo_odom_params = { # use imu data for stereo odometry
            'subscribe_imu': True,
            'wait_imu_to_init': True,
            'guess_frame_id': 'odom',
        }

    elif odom_type == 'ekf': # use pure stereo odometry as input to the EKF, without IMU data
        stereo_odom_remappings.append(('odom', '/stereo_odom'))
        stereo_odom_remappings.append(('odom_local_map', '/stereo_odom_local_map'))
        stereo_odom_params = { # do not use imu data for stereo odometry
            'subscribe_imu': False,
            'wait_imu_to_init': False,
            'guess_frame_id': '',
            'publish_tf': True, 
        }
    else:
        raise ValueError(f"Unknown odom_type: {odom_type}. Allowed values: loosely, tight, ekf")
    
    stereo_odometry_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[rtabmap_parameters_path, {'use_sim_time': True}, stereo_odom_params],
        remappings=stereo_odom_remappings
    )

    # Node for publishing the TF from the stereo odometry pose 
    # stereo_odom_tf_publisher_node = Node(
    #     package='ship_gazebo',
    #     executable='stereo_odom_tf_publisher.py',
    #     name='stereo_odom_tf_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'odom_topic': '/stereo_odom',
    #         'parent_frame_id': 'stereo_odom',
    #         'child_frame_id': 'base_footprint_stereo'
    #     }]
    # )

    delayed_stereo_odometry_node = TimerAction(period=4.0, actions=[stereo_odometry_node])
    #delayed_stereo_odom_tf_publisher_node = TimerAction(period=5.0, actions=[stereo_odom_tf_publisher_node])

    #==============================================================================
    # Node for SLAM and Navigation (RTAB-Map and Nav2)
    #==============================================================================

    rtabmap_slam_remappings = [
            # ('left/image_rect', '/camera/left/image_rect'),
            # ('right/image_rect', '/camera/right/image_rect'),
            # ('left/camera_info', '/camera/left/camera_info'),
            # ('right/camera_info', '/camera/right/camera_info'),
            ('rgbd_image', '/stereo_camera/rgbd_image'), # use the synchronized stereo image topic from rtabmap_sync
            ('imu','/robot/imu/compensated'),
    ]

    rtabmap_slam_parameters = [rtabmap_parameters_path,{'use_sim_time': True}]

    if odom_type == 'ekf': 
        rtabmap_slam_remappings.append(('odom', '/odometry/filtered'))
        rtabmap_slam_parameters.append({'odom_frame_id': 'odom_ekf'})
    elif odom_type == 'loosely':
        rtabmap_slam_remappings.append(('odom', '/stereo_odom'))
        rtabmap_slam_parameters.append({'odom_frame_id': 'stereo_odom'}) # set the odom frame id to match the stereo odometry output
    else:
        raise ValueError(f"Unknown odom_type: {odom_type}. Allowed values: loosely, tight, ekf")
    
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_slam_parameters,
        remappings=rtabmap_slam_remappings,
        arguments=['-d']
    )

    delayed_rtabmap_slam_node = TimerAction(period=7.0, actions=[rtabmap_slam_node])

    # # Node for resetting localization after the ship stabilizes
    # # Waits for ShipMotionPlugin to stabilize joints (when robot gravity settles),
    # # then calls Nav2's /reinitialize_global_localization service
    # # This ensures clean SLAM mapping without noise from initial gravity/spawn vibrations
    # reset_localization_node = Node(
    #     package='ship_gazebo',
    #     executable='reset_localization.py',
    #     name='reset_localization',
    #     parameters=[rtabmap_parameters_path],
    #     output='screen'
    # )

    # #=====================================
    # # Nodes for Exploration and Navigation
    # # #===================================

    # # Node for the explore_lite package, which will perform autonomous exploration of the environment using the occupancy grid generated by RTAB-Map.
    # # This node will be launched only in the mapping mode to allow the robot to explore and
    # # build a map of the environment. It will use the occupancy grid data from RTAB-Map to plan exploration paths and navigate to unexplored areas.
    # explore_lite_params_path = os.path.join(pkg_share, 'config', 'explore_lite_params.yaml')
    # explore_lite_node = Node(
    #     package='explore_lite',
    #     executable='explore',
    #     name='explore_lite',
    #     output='screen',
    #     parameters=[explore_lite_params_path]
    # )

    # # Start exploration early so the first map publication is not missed.
    # delayed_explore_lite_node = TimerAction(period=10.0, actions=[explore_lite_node])


    # # Node for launching the Nav2 stack, which will provide navigation capabilities for the robot.
    # # This node will be launched in both the mapping and navigation modes, 
    # # but in the mapping mode it will be used primarily for navigation during exploration, 
    # # while in the navigation mode it will be used for navigating to specific goals based on the map built by RTAB-Map. 

    # nav2_params_path = os.path.join(pkg_share,'config','nav2_params.yaml') # path to the Nav2 parameters file, which will be used for navigation in the navigation mode
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'),
    #     'launch', 'navigation_launch.py')]),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'params_file': nav2_params_path,
    #         'autostart': 'true',
    #     }.items()
    # )

    # # # Delay Nav2 so bridge + odom TF are available before activation.
    # delayed_nav2_launch = TimerAction(period=4.0, actions=[nav2_launch])

    #===================================
    # Nodes for Map Saving and Loading
    #===================================

    # # Map saver service - saves the occupancy grid to a file
    # # Used in mapping mode to persist the generated map
    # map_save_dir = os.path.expanduser('~/.local/share/ship_gazebo/maps')
    # os.makedirs(map_save_dir, exist_ok=True)

    # map_saver_server = Node(
    #     package='nav2_map_server',
    #     executable='map_saver_server',
    #     name='map_saver_server',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'save_map_timeout': 5.0
    #     }]
    # )

    # # Map server - loads a pre-saved occupancy grid map
    # # Used in navigation mode to provide the static map for localization and planning
    # map_save_dir = os.path.expanduser('~/.local/share/ship_gazebo/maps')
    # map_yaml_file = os.path.join(map_save_dir, 'ship_gazebo.yaml')

    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'yaml_filename': map_yaml_file
    #     }]
    # )

    #=======================================================
    # Rviz Node for Visualization
    #=======================================================

    # Rviz node for visualizing the robot, the map, and the navigation process. 
    # rviz_config_path = os.path.join(get_package_share_directory('nav2_bringup'),'rviz','nav2_default_view.rviz')
    # rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     parameters=[{
    #         'use_sim_time': True
    #     }],
    #     output='screen'
    # )

    #=============================
    # Nodes List
    #=============================

    nodes_list = [

        DeclareLaunchArgument('mode', default_value='navigation',description='Select the mode of operation: mapping, navigation_no_comp, navigation'),
        DeclareLaunchArgument('debug_camera', default_value='false', description='Enable or disable debug camera nodes (stereo_view and disparity_map)'),
        DeclareLaunchArgument('odom_type',default_value='ekf', description='Select the odometry type to use: loosely (stereo+imu+guess) or ekf (wheel+imu+stereo in EKF). tight is alias of loosely.'),

        AppendEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path,
        ),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/opt/ros/jazzy/share/turtlebot3_gazebo/models',
        ),

        #Execute the Gazebo simulator with the generated SDF file <sdf_out>
        ExecuteProcess(
            cmd=[
                'gz', 'sim','-r', sdf_out
            ],
            output='screen'
        ),

        #Execute the ros_gz_bridge parameter_bridge to bridge the topics between Gazebo and ROS 2
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '--ros-args', '-p',
                f'config_file:={bridge_config}',
                '-p','use_sim_time:=true',
            ],
            output='screen'
        ),

        left_camera_tf, # static transform from base_footprint to left camera
        right_camera_tf, # static transform from base_footprint to right camera
        imu_tf, # static transform from base_footprint to IMU
        left_camera_rect, # node for rectifying the left camera image
        right_camera_rect, # node for rectifying the right camera image
        compensate_imu, # node for compensating the IMU readings w.r.t. the ship motion
        stereo_sync_node, # node for synchronizing the left and right camera images and packaging them into a single RGBD image topic
        delayed_stereo_odometry_node, # node for generating odometry from the stereo camera data, delayed to ensure cameras and bridge are up
        #delayed_stereo_odom_tf_publisher_node, # node for publishing the TF from the stereo odometry pose, delayed to ensure stereo odometry is up
        delayed_rtabmap_slam_node,
        #reset_localization_node,
        #rviz_cmd
    ]

    if odom_type == 'ekf':
        nodes_list.append(ekf_odom_node) # run EKF only when selected as odometry architecture
   
    if debug_camera:  # append the stereo view and disparity map nodes only if debug_camera is enabled
        nodes_list.append(rqt_image_view)
        nodes_list.append(stereo_view)
        nodes_list.append(disparity_map)

    # if(mode_str == 'mapping'):
    #     nodes_list.append(delayed_explore_lite_node)
    #     nodes_list.append(delayed_nav2_launch)
    #     nodes_list.append(map_saver_server)
    # elif(mode_str == 'navigation' or mode_str == 'navigation_no_comp'):
    #     nodes_list.append(delayed_nav2_launch)
    #     nodes_list.append(map_server_node)

    #=============================
    # Launch Description
    #=============================

    return LaunchDescription(nodes_list)
