import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ware = get_package_share_directory('ware')
    
    # Path to the bridge configuration file
    bridge_config = os.path.join(pkg_ware, 'config', 'bridge.yaml')

    # Set the resource path for Gazebo to find the models
    # The models are installed to share/ware/models
    # We need to add share/ware/models to GZ_SIM_RESOURCE_PATH
    gz_resource_path = os.path.join(pkg_ware, 'models')
    
    # We use AppendEnvironmentVariable to add to the existing path if any
    set_env = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_ware, 'urdf', 'tugbot.urdf')
    world_file = os.path.join(pkg_ware, 'models', 'tugbot_warehouse', 'model.sdf')

    # Bridge using configuration file
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )
    
    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': open(urdf_file).read()},
        ]
    )
    
    
    rviz_config = os.path.join(pkg_ware, 'rviz', 'warehouse.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        set_env,
        gz_sim,
        bridge,
        robot_state_publisher,
        rviz_node
    ])
