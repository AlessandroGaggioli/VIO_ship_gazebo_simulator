import os
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, TimerAction
def generate_launch_description():
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
            cmd=['gz', 'sim', sdf_out],
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
    ])