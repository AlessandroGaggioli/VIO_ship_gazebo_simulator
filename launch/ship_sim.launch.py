import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('ship_gazebo')
    
    # legge direttamente dalla cartella src (no build necessario)
    sdf_em = os.path.expanduser(
        '~/ship_ws/src/ship_gazebo/worlds/ship_world_dynamic.sdf.em'
    )
    sdf_out = os.path.expanduser(
        '~/ship_ws/src/ship_gazebo/worlds/ship_world_dynamic.sdf'
    )
    
    # genera il file SDF dal template EmPy
    subprocess.run(
        ['python3', '-c', 'import em; em.main()', '--', sdf_em],
        stdout=open(sdf_out, 'w'),
        check=True
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', sdf_out],
            output='screen'
        )
    ])
