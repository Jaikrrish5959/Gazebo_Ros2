import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_gazebo = get_package_share_directory('my_robot_gazebo')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_my_gazebo, 'worlds', 'minimal.sdf')}.items(),
    )

    return LaunchDescription([gz_sim])
