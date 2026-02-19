import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_mobile_manipulator = get_package_share_directory('mobile_manipulator_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless')
    world_file = os.path.join(pkg_mobile_manipulator, 'worlds', 'task_world.sdf')
    
    # Process URDF
    xacro_file = os.path.join(pkg_mobile_manipulator, 'urdf', 'robot.urdf.xacro')
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': robot_description_config}]
    )

    # Gazebo Sim
    gz_args = PythonExpression([
        "'-r ' + ('-s ' if '", headless, "' == 'true' else '') + '", world_file, "'"
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mobile_manipulator',
                   '-x', '0.0',
                   '-y', '0.0', 
                   '-z', '0.05'],
        output='screen'
    )

    # ROS GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Arm Joint Controllers (ROS -> GZ)
            '/model/mobile_manipulator/joint/shoulder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/mobile_manipulator/joint/elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/mobile_manipulator/joint/wrist_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/mobile_manipulator/joint/finger_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/mobile_manipulator/joint/finger_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # Target box pose (GZ -> ROS)
            '/model/target_box/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Logic Nodes - don't use sim_time for timers to work
    perception_node = Node(
        package='mobile_manipulator_sim',
        executable='perception',
        name='perception_node',
        output='screen'
    )

    navigation_node = Node(
        package='mobile_manipulator_sim',
        executable='navigation',
        name='navigation_node',
        output='screen'
    )

    manipulation_node = Node(
        package='mobile_manipulator_sim',
        executable='manipulation',
        name='manipulation_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo in headless mode'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        perception_node,
        navigation_node,
        manipulation_node
    ])
