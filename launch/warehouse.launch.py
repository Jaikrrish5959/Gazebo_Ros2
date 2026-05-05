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


# ── LiDAR URDF snippet (Gazebo Harmonic gpu_lidar) ──────────────────────────
LIDAR_URDF_SNIPPET = """
  <!-- LiDAR sensor link (added by forklift_warehouse) -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="lidar_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0"
               izz="0.0001"/>
    </inertial>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.0 0.0 0.45" rpy="0 0 0"/>
  </joint>
  <gazebo reference="lidar_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <gz_frame_id>lidar_link</gz_frame_id>
      <visualize>true</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
    </sensor>
  </gazebo>
"""


def generate_launch_description():
    """Launch Freddy robot in a randomized warehouse world with controllers."""

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Package directories
    pkg_warehouse = get_package_share_directory('forklift_warehouse')
    pkg_freddy_description = get_package_share_directory('freddy_description')

    # ── Static World (fixed condensed warehouse — no random generation) ───
    world_file = os.path.join(pkg_warehouse, 'worlds', 'warehouse.sdf')

    # Freddy resource paths
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

    # ── Robot description: upstream xacro + LiDAR injection ───────────────
    # Process the upstream xacro (no overlay needed)
    freddy_xacro = os.path.join(
        pkg_freddy_description, 'robots', 'freddy_gz.urdf.xacro')
    robot_urdf = xacro.process_file(
        freddy_xacro,
        mappings={'use_gpu_2d_laser': 'false'}
    ).toxml()

    # Inject our Gazebo-Harmonic-compatible LiDAR sensor before </robot>
    robot_urdf = robot_urdf.replace('</robot>', LIDAR_URDF_SNIPPET + '</robot>')

    robot_description = {'robot_description': robot_urdf}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
    )

    # Launch Gazebo with the generated world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments=[('gz_args', [f' -r -v 4 {world_file}'])],
    )

    # Spawn Freddy into the world (exactly ONE robot)
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'freddy',
            '-allow_renaming', 'false',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
    )

    # ── ROS-GZ Bridge ─────────────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # ── Controller loading ────────────────────────────────────────────────
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

    # Wait 15 seconds after spawn for gz_ros2_control to initialize
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

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_warehouse, 'config', 'rviz', 'warehouse.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # ── Autonomy Nodes (all launched automatically) ───────────────────────
    # These start 5s after spawn to let sensors initialize.

    lidar_box_detector_node = Node(
        package='forklift_warehouse',
        executable='lidar_box_detector.py',
        name='lidar_box_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    reactive_navigator_node = Node(
        package='forklift_warehouse',
        executable='reactive_navigator.py',
        name='reactive_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Delay autonomy nodes until after robot spawn completes
    delayed_autonomy = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[
                        lidar_box_detector_node,
                        reactive_navigator_node,
                    ],
                )
            ],
        )
    )

    # ── Static TF: odom → base_link ─────────────────────────────────────
    # Gazebo Harmonic + Freddy doesn't publish an odom frame.
    # Provide a static identity transform so RViz / Nav2 have a valid TF tree.
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated clock'),

        # Environment
        set_gz_resource_path,

        # Simulation
        gz_sim,
        ros_gz_bridge,

        # Robot
        robot_state_publisher,
        gz_spawn_entity,

        # TF (odom frame)
        static_odom_tf,

        # Controllers (delayed sequential loading)
        delayed_jsb,
        after_jsb_load_controllers,

        # Autonomy (delayed after spawn)
        delayed_autonomy,

        # Visualization
        rviz_node,
    ])
