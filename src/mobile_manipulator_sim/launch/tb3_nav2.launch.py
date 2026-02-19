import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Set TurtleBot3 model to Waffle
    turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_mobile_manipulator = get_package_share_directory('mobile_manipulator_sim')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Custom world file
    world_file = os.path.join(pkg_mobile_manipulator, 'worlds', 'tb3_pick_world.sdf')
    
    # TurtleBot3 Gazebo launch (spawns robot and starts Gazebo)
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
    )
    
    # Nav2 Navigation launch
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    
    # Goal sender node - sends navigation goal to target box location
    goal_sender = Node(
        package='mobile_manipulator_sim',
        executable='goal_sender',
        name='goal_sender_node',
        output='screen',
        parameters=[{
            'target_x': 5.0,
            'target_y': 0.0,
        }]
    )

    return LaunchDescription([
        turtlebot3_model,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        turtlebot3_gazebo,
        nav2_bringup,
        goal_sender,
    ])
