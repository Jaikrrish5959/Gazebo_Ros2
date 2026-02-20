#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory('forklift_gazebo')

    world_file = os.path.join(pkg_share, 'worlds', 'forklift.world')
    models_dir = os.path.join(pkg_share, 'models')

    resource_path = models_dir + ':' + pkg_share + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    plugin_path = os.path.join(pkg_share, '..', '..', 'lib') + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': resource_path,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path,
        }
    )

    auto_drive = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'forklift_gazebo', 'auto_drive.py'],
                output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'}
            )
        ]
    )

    info = LogInfo(
        msg='\n\n========================================\n'
            '  AUTOMATED FORKLIFT DEMO\n'
            '  The forklift will:\n'
            '    1. Drive to the pallet stand\n'
            '    2. Insert forks under cargo\n'
            '    3. Lift the cargo\n'
            '    4. Reverse with cargo\n'
            '\n'
            '  Watch the Gazebo window!\n'
            '========================================\n'
    )

    return LaunchDescription([
        info,
        gz_sim,
        auto_drive,
    ])
