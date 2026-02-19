import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_hydraulic_bot = get_package_share_directory('hydraulic_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process URDF
    xacro_file = os.path.join(pkg_hydraulic_bot, 'urdf', 'hydraulic_mobile_robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': robot_description}]
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'hydraulic_bot',
                   '-x', '0.0',
                   '-y', '0.0', 
                   '-z', '0.2'], # Spawn slightly above ground
        output='screen'
    )

    # ROS GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Cmd Vel
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odom
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint States
            '/world/empty/model/hydraulic_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # Arm Joint Controllers (ROS -> GZ)
            # Note: The topic structure depends on the gazebo version and plugin config. 
            # Usually /model/<model_name>/joint/<joint_name>/cmd_pos if using standard plugins.
            '/model/hydraulic_bot/joint/shoulder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/hydraulic_bot/joint/lift_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/hydraulic_bot/joint/elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/hydraulic_bot/joint/wrist_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/hydraulic_bot/joint/finger_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/hydraulic_bot/joint/finger_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
