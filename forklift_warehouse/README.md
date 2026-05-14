# Adaptive Warehouse Robot — LiDAR + Semantic Reasoning

A ROS 2 Jazzy + Gazebo Harmonic simulation featuring the **Freddy** dual-arm mobile manipulator performing **autonomous**, sensor-driven box transport in a dynamically generated warehouse. The robot uses LiDAR perception to discover boxes, avoids obstacles ethically, and validates actions through an OWL ontology reasoning layer. update

## Features

- **Single Command Launch** — the entire autonomy stack runs automatically via `warehouse.launch.py`.
- **Random Exploration** — navigator drives robot randomly through the warehouse to search for the pickup table.
- **LiDAR-based Table Detection** — filters specific cluster widths to identify tables while isolating from walls/shelves.
- **Dynamic warehouse** — randomized shelf, table, box, and obstacle positions each run.
- **Autonomous Pickup** - non-blocking transport pipeline orchestration via action server async calls.
- **OWL ontology reasoning** — CORA-aligned semantic validation (Robot ⊆ ∃hasSensor.LiDAR, ∀avoids.(Obstacle ⊔ Human))
- **Real-time RViz** — LaserScan, TF, detected table markers, robot model.

## Robot: Freddy

| Property | Value |
|----------|-------|
| **Base** | KELO omnidirectional platform (4 drives, 8 hub wheels, 500kg payload) |
| **Arms** | 2× Kinova Gen3 7-DOF manipulators |
| **Sensor** | GPU LiDAR — 720 samples, 360°, 10m range, 10Hz |
| **Control** | `gz_ros2_control` with trajectory controllers (arms) + velocity controller (base) |

## Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-ros-gz ros-jazzy-moveit

# Python (OWL reasoning)
pip install owlready2

# Verify
echo $ROS_DISTRO  # Should print: jazzy
```

## Workspace Setup

```bash
mkdir -p ~/freddy_ws/src
cd ~/freddy_ws/src

# Clone this package
cp -r /path/to/forklift_warehouse .

# Clone Freddy dependencies
git clone https://github.com/a2s-institute/freddy-gazebo-simulation.git
git clone --branch simulation https://github.com/a2s-institute/freddy_description.git
git clone --branch gz-devel https://github.com/a2s-institute/ros2_kortex.git
git clone --branch sim-dev https://github.com/a2s-institute/gz_ros2_control.git

export GZ_VERSION=harmonic

```bash
cd ~/freddy_ws
colcon build --symlink-install
source install/setup.bash
```

## How It Works
│  Spawns shelves + pickup table + target box + obstacles  │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  Gazebo Harmonic  ←→  ros_gz_bridge                      │
│  /lidar → /scan (LaserScan)                              │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  LiDAR Table Detector (lidar_box_detector.py)            │
│  Clusters scan → filters table size → publishes:        │
│    /detected_objects (MarkerArray)                        │
│    /detected_box_poses (PoseArray)                       │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  Wall-Follow Navigator (reactive_navigator.py)           │
│  States: RANDOM_EXPLORE → TABLE_DETECTED → APPROACH → ARRIVED │
│  Publishes: /nav_status                                  │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  Adaptive Box Transport (adaptive_box_transport.py)      │
│  Orchestrates: 25s wait → search → reach → pick → place  │
└──────────────────────────────────────────────────────────┘
         ↕
┌──────────────────────────────────────────────────────────┐
│  Ontology Reasoner (ontology_reasoner.py)                │
│  OWL validation: LiDAR check, box-shelf, ethics          │
│  Service: /query_ontology   Status: /ontology_status     │
└──────────────────────────────────────────────────────────┘
```

## Launch

### 1. Full Simulation (Gazebo + LiDAR + RViz + Controllers)

```bash
cd ~/freddy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch forklift_warehouse warehouse.launch.py
```

Each launch generates a fresh randomized warehouse layout.

### 2. LiDAR Box Detector

```bash
ros2 run forklift_warehouse lidar_box_detector.py
```

### 3. Reactive Navigator

```bash
ros2 run forklift_warehouse reactive_navigator.py
```

### 4. Autonomous Box Transport

```bash
ros2 run forklift_warehouse adaptive_box_transport.py
```

### 5. Ontology Reasoner

```bash
ros2 run forklift_warehouse ontology_reasoner.py
# Query it:
ros2 service call /query_ontology std_srvs/srv/Trigger
```

### 6. MoveIt 2 (Optional — Motion Planning UI)

```bash
ros2 launch forklift_warehouse moveit.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 360° LiDAR scan |
| `/detected_objects` | `visualization_msgs/MarkerArray` | Detected table markers for RViz |
| `/detected_box_poses` | `geometry_msgs/PoseArray` | Detected table positions |
| `/transport_status` | `std_msgs/String` | Adaptive transport state |
| `/nav_status` | `std_msgs/String` | Navigator state machine status |
| `/base_velocity_controller/commands` | `std_msgs/Float64MultiArray` | Base wheel velocities |
| `/ontology_status` | `std_msgs/String` | Ontology system health |
| `/tf` | `tf2_msgs/TFMessage` | Robot transform tree |

## Architecture

```
forklift_warehouse/
├── config/
│   ├── moveit/
│   │   ├── freddy.srdf
│   │   ├── kinematics.yaml
│   │   ├── ompl_planning.yaml
│   │   └── moveit_controllers.yaml
│   └── rviz/
│       └── warehouse.rviz
├── launch/
│   ├── warehouse.launch.py        # Main launch (world gen + Gazebo + RViz)
│   └── moveit.launch.py           # MoveIt 2 launch
├── ontology/
│   └── warehouse_robot.owl        # CORA-aligned OWL ontology
├── src/
│   ├── random_world_generator.py  # Randomized SDF generator
│   ├── lidar_box_detector.py      # LiDAR → box detection
│   ├── reactive_navigator.py      # Obstacle avoidance + navigation
│   ├── adaptive_box_transport.py  # Autonomous pickup pipeline
│   └── ontology_reasoner.py       # OWL semantic reasoning
├── urdf/
│   └── freddy_lidar.urdf.xacro   # LiDAR sensor overlay
├── worlds/
│   └── warehouse.sdf              # Base world template
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Ontology (warehouse_robot.owl)

| Class | Constraint |
|-------|-----------|
| `Robot` | `∃hasSensor.LiDAR` — must have LiDAR sensor |
| `Box` | `∃locatedOn.Shelf` — must be on a shelf |
| `Robot` | `∀avoids.(Obstacle ⊔ Human)` — ethical avoidance |
| `Human` | subclass of `Obstacle` |

## Troubleshooting

- **No `/scan` data**: Check that `freddy_lidar.urdf.xacro` is installed and the bridge is running
- **Controllers not loading**: Ensure `freddy_gazebo` is built and URDF has `gz_ros2_control` tags
- **Robot falls through floor**: Check world physics `max_step_size` ≤ 0.01
- **Owlready2 error**: Run `pip install owlready2`
- **Same world each run**: Check `/tmp/warehouse_dynamic.sdf` is being regenerated
