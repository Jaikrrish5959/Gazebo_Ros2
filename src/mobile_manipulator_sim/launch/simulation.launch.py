import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_share = get_package_share_directory('mobile_manipulator_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf')

    # Robot Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mobile_manipulator',
                   '-z', '0.1'],
        output='screen',
    )

    # Bridge arguments
    bridge_args = [
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
    ]

    # Add Arm Joint Bridges
    joints_map = {
        'arm_base_joint': 'arm_base_controller', 
        'shoulder_joint': 'shoulder_controller', 
        'elbow_joint': 'elbow_controller', 
        'wrist_joint': 'wrist_controller', 
        'left_finger_joint': 'left_finger_controller', 
        'right_finger_joint': 'right_finger_controller'
    }
    
    for joint, controller in joints_map.items():
        # Gz Topic: {controller}/cmd_pos
        # ROS Topic: {controller}/cmd_pos
        bridge_args.append(
            f'/{controller}/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double'
        )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen'
    )

    # Mission Controller
    mission_controller = Node(
        package='mobile_manipulator_sim',
        executable='mission_controller',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        TimerAction(
            period=5.0, # Wait for robot to spawn
            actions=[mission_controller]
        )
    ])
