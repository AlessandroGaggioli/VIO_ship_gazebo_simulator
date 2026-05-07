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
import sys
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, LogInfo, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xml.etree.ElementTree as ET

# Function to get the configuration parameters based on the selected mode
def get_mode_config(mode_str):
    if mode_str == 'no_motion':
        return {
            'roll_amplitude': 0.0,
            'pitch_amplitude': 0.0,
            'heave_amplitude': 0.0,
            'imu_enable': False
        }
    elif mode_str == 'navigation_no_comp':
        return {
            'roll_amplitude': 0.2,
            'pitch_amplitude': 0.1,
            'heave_amplitude': 0.1,
            'imu_enable': False
        }
    elif mode_str == 'navigation':
        return {
            'roll_amplitude': 0.2,
            'pitch_amplitude': 0.1,
            'heave_amplitude': 0.1,
            'imu_enable': True
        }
    else:
        raise ValueError(f"Unknown mode: {mode_str}")

# Main function to generate the launch description
def generate_launch_description():

    mode_str = 'navigation'
    debug_camera = False
    odom_type = 'loosely'

    # Ensure there is a single simulation/clock source.
    # Stale gz/bridge processes from previous runs can cause /clock jumps,
    # which then break odometry, RTAB-Map behavior.
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
    
    config = get_mode_config(mode_str) # get the configuration parameters based on the selected mode

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
            # Legge il file riga per riga e ignora le righe vuote iniziali
            with open(sdf_path, 'r') as f:
                lines = f.readlines()
            
            # Trova dove inizia veramente l'XML e unisce il resto
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
    
    sx, sy, sz = get_robot_spawn_from_sdf(sdf_out) # extract the robot's spawn position from the generated SDF file to use in the IMU compensator node
    print(f"DEBUG: Robot spawn rilevato dall'SDF -> X:{sx}, Y:{sy}, Z:{sz}")

    #=========================================================
    # ROS 2 Nodes for Camera Processing and IMU Compensation
    #=========================================================

    # Camera position parameters relative to the robot's base_footprint frame.
    # These parameters define the static transforms for the left and right cameras
    # They need to be set equal to the position of the cameras in the SDF file.
    camera_params_path = os.path.join(pkg_share, 'config', 'stereo_params.yaml') # path to the camera parameters file

    # Define the ROS 2 nodes to be launched, including 
    #  - static transforms for the cameras and IMU
    #  - image processing nodes for rectifying camera images and generating disparity maps
    #  - the IMU compensator node to adjust IMU readings based on the robot's spawn position
    # - the RTAB-Map nodes for stereo odometry and SLAM, which will use the rectified camera images and compensated IMU data for localization and mapping

    #================================================================================
    # Static transform from base_footprint to the left and right cameras and the IMU
    #================================================================================

    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.010',  # da base_joint nel SDF
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ]
    )

    camera_link_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_left_base_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.15', '--y', '0.06', '--z', '0.1',  # da camera_left_joint nel SDF
            '--roll', '0', '--pitch', '0.2', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link_left'
        ]
    )

    camera_link_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_right_base_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.15', '--y', '-0.06', '--z', '0.1',  # da camera_right_joint nel SDF
            '--roll', '0', '--pitch', '0.2', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link_right'
        ]
    )

    left_camera_tf = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_left_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # '--x', cam_x, '--y', str(cam_y_offset), '--z', cam_z,
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.570796', '--pitch', '0', '--yaw', '-1.570796',
            '--frame-id', 'camera_link_left', # set the parent frame to the camera link, which is the frame where the camera images are published in Gazebo, to ensure correct TF tree structure for image processing nodes
            '--child-frame-id', 'ship_corridor_dynamic/turtlebot3_waffle/camera_link_left/stereo_camera_left'
        ]
    )

    right_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_right_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.570796', '--pitch', '0', '--yaw', '-1.570796',
            '--frame-id', 'camera_link_right', # set the parent frame to the camera link, which is the frame where the camera images are published in Gazebo, to ensure correct TF tree structure for image processing nodes
            '--child-frame-id', 'ship_corridor_dynamic/turtlebot3_waffle/camera_link_right/stereo_camera_right'
        ]
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '-0.032', '--y', '0', '--z', '0.078',  # 0.068 + 0.010
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
        parameters=[camera_params_path, {'use_sim_time': True}],
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
        parameters=[camera_params_path, {'use_sim_time': True}],
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
        parameters=[camera_params_path, {'use_sim_time': True}],
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
        parameters=[camera_params_path, {'use_sim_time': True}],
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

    #======================================================
    # Stereo Synchronization Node by RTAB-Map (rtabmap_sync)
    #======================================================

    stereo_sync_node = Node(
        package='rtabmap_sync',
        executable='stereo_sync',
        name='stereo_sync',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.01, # maximum time difference between left and right images to be considered synchronized
            'sync_queue_size': 10
        }],
        remappings=[
            ('left/image_rect', '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('rgbd_image', '/stereo_camera/rgbd_image') 
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
        parameters=[ekf_odom_params_path,{'use_sim_time':True}]
    )

    #===========================================================================
    # Node for generating odometry from the stereo camera data using the rtabmap_odom package
    #===========================================================================
    rtabmap_parameters_path = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    stereo_odom_remappings = [ # map topic to enable imu data for stereo odometry
                ('rgbd_image', '/stereo_camera/rgbd_image') # use the synchronized stereo image topic from rtabmap_sync
        ]


    if odom_type in ('loosely'):
        # wheel_tf_bridge = Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='wheel_tf_bridge',
        #     arguments=['/model/turtlebot3_waffle/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        #     remappings=[
        #         ('/model/turtlebot3_waffle/tf', '/tf')
        #     ],
        #     output='screen'
        # )

        stereo_odom_remappings.append(('odom', '/stereo_odom'))
        stereo_odom_remappings.append(('odom_local_map', '/stereo_odom_local_map'))
        stereo_odom_remappings.append(('imu','/robot/imu/compensated'))
        stereo_odom_params = { # use imu data for stereo odometry
            'subscribe_imu': True,
            'wait_imu_to_init': True,
            'guess_from_tf': False,
            #'guess_frame_id': 'odom',
            'publish_tf': True,
            'odom_frame_id': 'stereo_odom',
        }

    elif odom_type == 'ekf': # use pure stereo odometry as input to the EKF, without IMU data
        stereo_odom_remappings.append(('odom', '/stereo_odom'))
        stereo_odom_remappings.append(('odom_local_map', '/stereo_odom_local_map'))
        stereo_odom_params = { # do not use imu data for stereo odometry
            'subscribe_imu': False,
            'wait_imu_to_init': False,
            'guess_frame_id': '',
            'publish_tf': False,
            'odom_frame_id': 'stereo_odom', 
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
    #==============================================================================
    # Node for SLAM and Navigation (RTAB-Map and Nav2)
    #==============================================================================

    rtabmap_slam_remappings = [
            ('rgbd_image', '/stereo_camera/rgbd_image'), # use the synchronized stereo image topic from rtabmap_sync
            ('imu','/robot/imu/compensated'),
    ]

    rtabmap_slam_parameters = [rtabmap_parameters_path,{'use_sim_time': True}]

    if odom_type == 'ekf': 
        #if odom_frame_id is set, RTAB-Map will not subscribe to /odometry/filtered 
        rtabmap_slam_remappings.append(('odom', '/odometry/filtered'))
        #rtabmap_slam_parameters.append({'odom_frame_id': 'odom_ekf'})
    elif odom_type == 'loosely':
        rtabmap_slam_remappings.append(('odom', '/stereo_odom'))
        #rtabmap_slam_parameters.append({'odom_frame_id': 'stereo_odom'}) # set the odom frame id to match the stereo odometry output
    else:
        raise ValueError(f"Unknown odom_type: {odom_type}. Allowed values: loosely, ekf")
    
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_slam_parameters,
        remappings=rtabmap_slam_remappings,
        arguments=['-d']
    )

    #==========
    # RVIZ
    #===========

    rviz2_config = os.path.join(pkg_share,'config','rtabmap.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': True}]
    )

    #=============================
    # TIMING CONFIGURATION
    #=============================
       
    CAMERA_STARTUP = 2.5      
    SYNC_STARTUP = CAMERA_STARTUP + 1.0    
    ODOM_STARTUP = SYNC_STARTUP + 2.5      
    SLAM_STARTUP = ODOM_STARTUP + 2.0   

    #=============================
    # Nodes List
    #=============================   

    nodes_list = [

        DeclareLaunchArgument('mode', default_value='navigation',description='Select the mode of operation: no_motion, navigation_no_comp, navigation'),
        DeclareLaunchArgument('debug_camera', default_value='false', description='Enable or disable debug camera nodes (stereo_view and disparity_map)'),
        DeclareLaunchArgument('odom_type',default_value='loosely', description='Select the odometry type to use: loosely (stereo+imu+guess) or ekf (wheel+imu+stereo in EKF). tight is alias of loosely.'),

        AppendEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path,
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_share, 'models'),  # ← il tuo prima
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
            name='gazebo_simulator',
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

        TimerAction(period=CAMERA_STARTUP, actions=[
            left_camera_tf,
            right_camera_tf,
            imu_tf,
            base_link_tf,
            camera_link_left_tf,
            camera_link_right_tf,
            left_camera_rect,
            right_camera_rect,
            compensate_imu,  
        ]),

        TimerAction(period=SYNC_STARTUP, actions=[stereo_sync_node]),

        TimerAction(period=ODOM_STARTUP, actions=[stereo_odometry_node]),

        TimerAction(period=SLAM_STARTUP, actions=[rtabmap_slam_node]),

        rviz2,
    ]

    if odom_type == 'ekf':
                nodes_list.append(
            TimerAction(period=ODOM_STARTUP - 1.0, actions=[ekf_odom_node])
        )
    elif odom_type == 'loosely':
        # nodes_list.append(
        #     TimerAction(period=ODOM_STARTUP - 1.0, actions=[wheeel_tf_bridge])
        # )
        print("loosely")
   
    if debug_camera:  # append the stereo view and disparity map nodes only if debug_camera is enabled
        nodes_list.append(
            TimerAction(period=CAMERA_STARTUP + 0.5, actions=[rqt_image_view])
        )
        nodes_list.append(
            TimerAction(period=SYNC_STARTUP + 0.5, actions=[stereo_view, disparity_map])
        )


    #=============================
    # Launch Description
    #=============================

    return LaunchDescription(nodes_list)
