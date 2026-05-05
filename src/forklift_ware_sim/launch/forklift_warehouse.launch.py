#!/usr/bin/env python3
"""
forklift_warehouse.launch.py  –  self-contained, no internet required.

Usage:
  ros2 launch forklift_ware_sim forklift_warehouse.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, LogInfo, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg            = get_package_share_directory('forklift_ware_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg, 'worlds', 'forklift_warehouse.sdf')
    urdf_file  = os.path.join(pkg, 'urdf', 'forklift.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Plugin .so files live at install/forklift_ware_sim/lib/
    install_lib = os.path.join(pkg, '..', '..', 'lib')
    set_gz_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=install_lib,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/fork_lift_shift/lift_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/fork_lift_shift/shift_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    info = LogInfo(
        msg='\n========================================\n'
            '  FORKLIFT WAREHOUSE (local geometry)\n'
            '  Controls:\n'
            '    Drive : ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...\n'
            '    Lift  : ros2 topic pub /fork_lift_shift/lift_cmd std_msgs/msg/Float64 ...\n'
            '    Script: ros2 run forklift_ware_sim forklift_ops.py\n'
            '========================================\n'
    )

    return LaunchDescription([
        info,
        set_gz_plugin_path,
        gz_sim,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[robot_state_publisher]),
    ])
