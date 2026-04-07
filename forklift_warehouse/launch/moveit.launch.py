import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    """Launch MoveIt 2 move_group for Freddy robot."""

    pkg_warehouse = get_package_share_directory('forklift_warehouse')

    # Robot description from Freddy xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('freddy_description'),
            'robots', 'freddy_gz.urdf.xacro'
        ]),
    ])
    robot_description = {'robot_description': robot_description_content}

    # SRDF
    srdf_file = os.path.join(pkg_warehouse, 'config', 'moveit', 'freddy.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Kinematics
    kinematics_yaml_path = os.path.join(pkg_warehouse, 'config', 'moveit', 'kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(kinematics_yaml_path)}

    # OMPL Planning
    ompl_planning_yaml_path = os.path.join(pkg_warehouse, 'config', 'moveit', 'ompl_planning.yaml')
    ompl_planning_pipeline_config = {'ompl': load_yaml(ompl_planning_yaml_path)}

    # MoveIt Controllers
    moveit_controllers_yaml_path = os.path.join(
        pkg_warehouse, 'config', 'moveit', 'moveit_controllers.yaml')
    moveit_controllers_config = load_yaml(moveit_controllers_yaml_path)

    # MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            moveit_controllers_config,
            {'use_sim_time': True},
            {'planning_scene_monitor_options': {
                'robot_description': 'robot_description',
                'joint_state_topic': '/joint_states',
            }},
        ],
    )

    # RViz for MoveIt visualization
    rviz_config = os.path.join(pkg_warehouse, 'config', 'moveit', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
