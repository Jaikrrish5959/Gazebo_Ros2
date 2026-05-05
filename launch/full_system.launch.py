import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the full warehouse autonomy stack.

    Includes warehouse.launch.py (which already starts Gazebo, RViz,
    controllers, lidar_box_detector, and reactive_navigator), then adds
    the adaptive_box_transport and ontology_reasoner nodes on top.
    """
    pkg_dir = get_package_share_directory('forklift_warehouse')

    # Core simulation: Gazebo + RViz + controllers + LiDAR detector + navigator
    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_dir, 'launch', 'warehouse.launch.py')]
        )
    )

    # Additional autonomy nodes (not included in warehouse.launch.py)
    # Delayed 30s to allow controllers to fully initialize first
    adaptive_box_transport = Node(
        package='forklift_warehouse',
        executable='adaptive_box_transport.py',
        name='adaptive_box_transport',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ontology_reasoner = Node(
        package='forklift_warehouse',
        executable='ontology_reasoner.py',
        name='ontology_reasoner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_extra_nodes = TimerAction(
        period=10.0,
        actions=[
            adaptive_box_transport,
            ontology_reasoner,
        ],
    )

    return LaunchDescription([
        warehouse_launch,
        delayed_extra_nodes,
    ])
