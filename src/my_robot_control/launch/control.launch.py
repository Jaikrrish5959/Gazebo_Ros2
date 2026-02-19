from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_control',
            executable='decision_node',
            name='decision_node',
            output='screen'
        )
    ])
