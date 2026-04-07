import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Freddy robot in the warehouse world with controllers."""

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Package directories
    pkg_warehouse = get_package_share_directory('forklift_warehouse')
    pkg_freddy_description = get_package_share_directory('freddy_description')
    pkg_freddy_gazebo = get_package_share_directory('freddy_gazebo')

    # World file
    world_file = os.path.join(pkg_warehouse, 'worlds', 'warehouse.sdf')

    # Freddy resource paths for GZ_SIM_RESOURCE_PATH
    resource_paths = [
        pkg_freddy_description,
        os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes'),
        os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes', 'sensors'),
        os.path.join(pkg_freddy_description, 'freddy_torso_description', 'meshes'),
        os.path.join(pkg_warehouse, 'models'),
    ]
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths),
    )

    # Robot description from Freddy xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('freddy_description'),
            'robots', 'freddy_gz.urdf.xacro'
        ]),
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True},
        ],
    )

    # Launch Gazebo with warehouse world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments=[('gz_args', [f' -r -v 4 {world_file}'])],
    )

    # Spawn Freddy into the world
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'freddy',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
    )

    # ROS-GZ Bridge for clock
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # Controller loading — use TimerAction to wait for gz_ros2_control
    # to initialize the controller_manager service after spawn.
    # The controller_manager needs several seconds after spawn to start.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_left_jtc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_left_joint_trajectory_controller'],
        output='screen'
    )

    load_arm_right_jtc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_right_joint_trajectory_controller'],
        output='screen'
    )

    load_base_velocity = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'base_velocity_controller'],
        output='screen'
    )

    # Wait 15 seconds after spawn for gz_ros2_control to initialize,
    # then load joint_state_broadcaster first
    delayed_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=15.0,
                    actions=[load_joint_state_broadcaster],
                )
            ],
        )
    )

    # After JSB loads, load arm and base controllers
    after_jsb_load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                load_arm_left_jtc,
                load_arm_right_jtc,
                load_base_velocity,
            ],
        )
    )

    return LaunchDescription([
        # Environment
        set_gz_resource_path,

        # Simulation
        gz_sim,
        ros_gz_bridge,

        # Robot
        robot_state_publisher,
        gz_spawn_entity,

        # Controllers (delayed sequential loading)
        delayed_jsb,
        after_jsb_load_controllers,

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulated clock'),
    ])
