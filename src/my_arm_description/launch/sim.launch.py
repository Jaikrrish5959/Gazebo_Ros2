from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('my_arm_description')
    urdf = os.path.join(pkg, 'urdf', 'simple_arm.urdf.xacro')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf])
        }]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'simple_arm', '-topic', 'robot_description'],
        output='screen'
    )

    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        TimerAction(period=5.0, actions=[load_jsb]),
        TimerAction(period=6.0, actions=[load_arm]),
    ])
