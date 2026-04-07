# Freddy Warehouse — Dual-Arm Box Transport Simulation

A ROS 2 Jazzy + Gazebo Harmonic simulation featuring the **Freddy** dual-arm mobile manipulator performing shelf-to-shelf box transport in a warehouse environment.

## Robot: Freddy

- **Base**: KELO omnidirectional platform (4 drives, 8 hub wheels, 500kg payload)
- **Arms**: 2× Kinova Gen3 7-DOF manipulators
- **Control**: `gz_ros2_control` with `joint_trajectory_controller` (arms) + `velocity_controller` (base)

## Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-ros-gz ros-jazzy-moveit

# Verify
echo $ROS_DISTRO  # Should print: jazzy
```

## Workspace Setup

```bash
# Create workspace
mkdir -p ~/freddy_ws/src
cd ~/freddy_ws/src

# Clone this package
cp -r /path/to/forklift_warehouse .

# Clone Freddy dependencies
git clone https://github.com/a2s-institute/freddy-gazebo-simulation.git
git clone --branch simulation https://github.com/a2s-institute/freddy_description.git
git clone --branch gz-devel https://github.com/a2s-institute/ros2_kortex.git
git clone --branch sim-dev https://github.com/a2s-institute/gz_ros2_control.git

# Set Gazebo version
export GZ_VERSION=harmonic

# Build
cd ~/freddy_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

### 1. Warehouse Simulation (Freddy + Gazebo)

```bash
cd ~/freddy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch forklift_warehouse warehouse.launch.py
```

This launches:
- Gazebo Harmonic with the warehouse world
- Freddy robot spawned at origin
- Controllers: `joint_state_broadcaster`, `arm_left/right_joint_trajectory_controller`, `base_velocity_controller`

### 2. Keyboard Teleop (Manual Control)

In a separate terminal:
```bash
cd ~/freddy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run freddy_gazebo freddy_control
```

Controls:
| Key | Action |
|-----|--------|
| `q` / `a` / `z` | Switch to left arm / right arm / base |
| `w e r t y u i o` | Increment joint 1-8 |
| `s d f g h j k l` | Decrement joint 1-8 |
| `+` / `-` | Increase / decrease step size |

### 3. MoveIt 2 (Motion Planning)

In a separate terminal:
```bash
cd ~/freddy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch forklift_warehouse moveit.launch.py
```

### 4. Box Transport (Automated Pipeline)

In a separate terminal:
```bash
cd ~/freddy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run forklift_warehouse box_transport.py
```

**Pipeline phases:**
1. Move arm to home
2. Drive to source shelf (shelf_0)
3. Pre-grasp position
4. Grasp box (contact-based attach)
5. Lift to carry position
6. Drive to destination shelf (shelf_3)
7. Place box
8. Return arm to home

## Architecture

```
forklift_warehouse/
├── config/
│   └── moveit/
│       ├── freddy.srdf          # Planning groups, named poses
│       ├── kinematics.yaml      # KDL solver config
│       ├── ompl_planning.yaml   # OMPL planners
│       └── moveit_controllers.yaml  # Controller mapping
├── launch/
│   ├── warehouse.launch.py      # Main launch (Gazebo + Freddy + controllers)
│   └── moveit.launch.py         # MoveIt 2 launch
├── src/
│   └── box_transport.py         # Automated transport pipeline
├── worlds/
│   └── warehouse.sdf            # Warehouse with shelves, box, destination marker
├── CMakeLists.txt
├── package.xml
└── README.md
```

## World Objects

| Object | Position | Description |
|--------|----------|-------------|
| `transport_box` | (-3.5, 2.3, 0.85) | Red 20cm cube, 1kg — **pick this up** |
| Multiple shelves | Various | Standard warehouse shelving |

## Troubleshooting

- **Controllers not loading**: Ensure `freddy_gazebo` is built and the URDF has `gz_ros2_control` tags
- **Robot falls through floor**: Check world physics `max_step_size` is ≤ 0.01
- **MoveIt can't find joints**: Verify `joint_state_broadcaster` is active via `ros2 control list_controllers`
