# Forklift Gazebo - ROS2 Jazzy & Gazebo Harmonic

Forklift simulation package for ROS2 Jazzy with Gazebo Harmonic support.

## Prerequisites

- ROS2 Jazzy
- Gazebo Harmonic (gz-sim8)
- Python 3
- colcon build tools

## Installation

### Install ROS2 Jazzy Dependencies

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces
sudo apt install ros-jazzy-robot-state-publisher
```

### Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic
```

## Building

```bash
cd /home/jaikrrishs/gazebo_ros2_ws
colcon build --symlink-install
```

## Running

### Source the workspace

```bash
source install/setup.bash
```

### Launch the simulation

**Option 1: Basic forklift simulation**
```bash
ros2 launch forklift_gazebo forklift.launch.py
```

**Option 1.5: Forklift simulation with Auto-Teleop (Recommended)**
Launches simulation and opens a new terminal for keyboard control automatically.
```bash
ros2 launch forklift_gazebo forklift_teleop.launch.py
```

**Option 2: Fork lift with shift mechanism**
```bash
ros2 launch forklift_gazebo fork_lift_shift.launch.py
```

**Option 3: Forklift freebody simulation**
```bash
ros2 launch forklift_gazebo forklift_freebody.launch.py
```

### Control the forklift joints

In a new terminal:

```bash
source install/setup.bash
ros2 run forklift_gazebo keyboard_teleop.py
```

Or use the helper script:
```bash
./run_teleop.sh
```

Or for freebody version:

```bash
ros2 run forklift_gazebo keyboard_teleop_freebody.py
```

**Keyboard Controls:**
- **↑/↓ Arrow Keys**: Lift up/down (vertical movement)
- **←/→ Arrow Keys**: Shift left/right (horizontal movement)
- **R**: Reset to default position
- **Q**: Quit

## Package Structure

```
forklift_gazebo/
├── launch/              # Python launch files for ROS2
├── models/              # Gazebo model definitions
├── plugins/             # Gazebo Harmonic plugins (C++)
├── src/                 # Python scripts
├── stl/                 # 3D mesh files
├── urdf/                # Robot description files
└── worlds/              # Gazebo world files
```

## Migration from ROS1

This package has been migrated from ROS1 (Catkin/Gazebo Classic) to ROS2 Jazzy with Gazebo Harmonic. Key changes:

- Package format upgraded to format 3
- Build system changed from catkin to ament_cmake
- Launch files converted from XML to Python
- Gazebo plugins updated to Harmonic API with ECS pattern
- Python scripts migrated from Python 2/rospy to Python 3/rclpy
- World files updated to SDF 1.9

## License

TODO
