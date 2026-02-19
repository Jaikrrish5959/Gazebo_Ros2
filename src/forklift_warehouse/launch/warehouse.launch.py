import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_forklift_warehouse = get_package_share_directory('forklift_warehouse')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_forklift_warehouse, 'worlds', 'warehouse_with_forklift.sdf')
    urdf_file = os.path.join(pkg_forklift_warehouse, 'urdf', 'forklift.urdf')

    # Gazebo resource path (for model:// URI)
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_forklift_warehouse, 'models')
    )

    # Gazebo plugin path (for libForkLiftPlugin.so and libForkLiftWorld.so)
    # Plugins are installed to INSTALL_PREFIX/lib/
    set_gz_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(get_package_share_directory('forklift_warehouse'), '..', '..', 'lib')
    )

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
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

    return LaunchDescription([
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher,
    ])
