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

from logging import config
import os
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node
import xml.etree.ElementTree as ET
# Function to get the configuration parameters based on the selected mode
def get_mode_config(mode_str):
    if mode_str == 'mapping':
        return {
            'roll_amplitude': 0.0,
            'pitch_amplitude': 0.0,
            'heave_amplitude': 0.0,
            'imu_enable': False
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

# Main function to generate the launch description
def generate_launch_description():

    mode_str = 'navigation'
    enable_obstacles = True

    # Parse command line arguments to get the mode and obstacles settings
    for arg in sys.argv:
        if 'mode:=' in arg:
            mode_str = arg.split(':=')[1]  # get the value after ':=' and set it to mode_str
        if 'obstacles:=' in arg:
            enable_obstacles = arg.split(':=')[1].lower() == 'true' # convert to boolean and set to enable_obstacles
    
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

    if enable_obstacles: # choose the SDF template with obstacles if enabled, otherwise use the one without obstacles
        sdf_em = os.path.join(sdf_dir, 'ship_world_dynamic.sdf.em')
    else:
        sdf_em = os.path.join(sdf_dir, 'ship_world_dynamic_NoObs.sdf.em')
    
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

    # Define the ROS 2 nodes to be launched, including 
    #  - static transforms for the cameras and IMU
    #  - image processing nodes for rectifying camera images and generating disparity maps
    #  - the IMU compensator node to adjust IMU readings based on the robot's spawn position
    # - the RTAB-Map nodes for stereo odometry and SLAM, which will use the rectified camera images and compensated IMU data for localization and mapping

    # Static transform from base_footprint to the left and right cameras and the IMU
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

    # Nodes for rectifying the left and right camera images using the image_proc package
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

    # Node for generating the disparity map from the rectified left and right camera images using the stereo_image_proc package
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

    # Node for visualizing the rectified stereo images and the disparity map using the stereo_view node from the image_view package
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

    # Node for compensating the IMU readings based on the robot's spawn position in the world
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

    # Parameters for the RTAB-Map nodes, which will use the rectified camera images and compensated IMU data for localization and mapping.
    rtabmap_parameters = {
        'frame_id': 'base_footprint',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_stereo': True,
        'approx_sync': False,
        'use_sim_time': True,
        'wait_imu_to_init': True,
        # Parametri di Memoria e Mapping (Stringhe!)
        'Mem/IncrementalMemory': 'true', 
        'Mem/InitWMWithAllNodes': 'true', 
        'Grid/Sensor': '1', 
        'Grid/FromDepth': 'true',
        'RGBD/CreateOccupancyGrid': 'true', # Corretto RGBD
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MinObstacleHeight': '2.0',
        'Grid/RangeMax': '5.0',
    }

    # Node for stereo odometry using the rtabmap_odom package, which will provide odometry estimates based on the stereo camera images and IMU data. 
    # This node will be used in the navigation mode to provide odometry information for the robot.
    stereo_odometry_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('left/image_rect', '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('imu', '/robot/imu/compensated') # topic published by imu_compensator node
        ]
    )

    # Node for RTAB-Map SLAM, which will perform simultaneous localization and mapping using the stereo camera images and IMU data.
    # This node will build a map of the environment and provide localization estimates for the robot based on the visual and inertial data.
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('left/image_rect', '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('imu', '/robot/imu/compensated')
        ],
        arguments=['-d'] # start with an empty database
    )
    
    return LaunchDescription([

        DeclareLaunchArgument('mode', default_value='navigation'),
        DeclareLaunchArgument('obstacles', default_value='true'),

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
                'ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen' 
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
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
        stereo_odometry_node,
        rtabmap_slam_node
    ])

