import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('my_robot_gazebo'), 'launch', 'gazebo.launch.py'])
            )
        ),
        
        # 2. Spawn Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('my_robot_description'), 'launch', 'spawn_robot.launch.py'])
            )
        ),
        
        # 3. Mission Logic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('my_robot_control'), 'launch', 'control.launch.py'])
            )
        ),
    ])
