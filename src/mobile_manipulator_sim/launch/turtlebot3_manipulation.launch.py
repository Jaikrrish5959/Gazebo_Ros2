import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Set TurtleBot3 model to Waffle
    turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_mobile_manipulator = get_package_share_directory('mobile_manipulator_sim')
    
    # TurtleBot3 Gazebo launch (spawns robot and starts Gazebo)
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
    )
    
    # Nav2 Bringup (Localization + Navigation)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml',
            'params_file': '/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle.yaml',
        }.items(),
    )
    
    # Goal sender node
    goal_sender = Node(
        package='mobile_manipulator_sim',
        executable='goal_sender',
        name='goal_sender_node',
        output='screen',
        parameters=[{
            'target_x': 2.0, # Adjusted for TurtleBot3 World size
            'target_y': 0.0,
            'use_sim_time': True
        }]
    )

    # Manipulation node (simulated arm control)
    manipulation_node = Node(
        package='mobile_manipulator_sim',
        executable='manipulation',
        name='manipulation_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        turtlebot3_model,
        turtlebot3_gazebo,
        nav2_bringup,
        goal_sender,
        manipulation_node
    ])
