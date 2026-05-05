# Adaptive Warehouse Robot — LiDAR + Semantic Reasoning

A ROS 2 Jazzy + Gazebo Harmonic simulation featuring the **Freddy** dual-arm mobile manipulator performing **autonomous**, sensor-driven box transport in a warehouse environment. The robot uses LiDAR perception to discover tables and boxes, avoids obstacles ethically, and validates actions through an OWL ontology reasoning layer.

## Features

- **Single Command Launch** — `full_system.launch.py` brings up Gazebo, RViz, all autonomy nodes, and the ontology reasoner in one shot.
- **Core Simulation** — `warehouse.launch.py` launches Gazebo Harmonic, RViz, the robot, controllers, LiDAR detector, and reactive navigator.
- **LiDAR Multi-Class Detection** — clusters laser scan data to classify objects as `BOX`, `TABLE`, `SHELF`, `CONVEYOR`, or `WALL`.
- **Reactive Navigation** — wall-following explorer with safety stop and collision avoidance via LiDAR.
- **Autonomous Pickup Pipeline** — 25s controller warmup → search → reach → pick → place orchestration via `adaptive_box_transport.py`.
- **OWL Ontology Reasoning** — CORA-aligned semantic validation: `Robot ⊆ ∃hasSensor.LiDAR`, `∀avoids.(Obstacle ⊔ Human)`.
- **Static odom→base_link TF** — ensures RViz and Nav2 have a valid transform tree.
- **Real-time RViz** — LaserScan, TF, detected table/box markers, robot model visualization.

## Robot: Freddy

| Property    | Value |
|-------------|-------|
| **Base**    | KELO omnidirectional platform (4 drives, 8 hub wheels, 500 kg payload) |
| **Arms**    | 2× Kinova Gen3 7-DOF manipulators |
| **Sensor**  | GPU LiDAR — 720 samples, 360°, 10 m range, 10 Hz |
| **Control** | `gz_ros2_control` with trajectory controllers (arms) + velocity controller (base) |

## Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-ros-gz ros-jazzy-moveit \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros

# Python (OWL reasoning — HermiT reasoner is bundled, no external Java needed)
pip install owlready2

# Verify
echo $ROS_DISTRO   # Should print: jazzy
python3 -c "import owlready2; print('owlready2 OK')"
```

## Workspace Setup

The Freddy robot depends on several upstream packages. They must be built together in a single colcon workspace.

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
```

### Build

```bash
export GZ_VERSION=harmonic
cd ~/freddy_ws
colcon build --symlink-install
source install/setup.bash
```

> **Note:** If you are developing `forklift_warehouse` outside the Freddy workspace (e.g. on your Desktop), you must source the Freddy workspace **first**, then rebuild and source locally:
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/freddy_ws/install/setup.bash    # Freddy dependencies
> cd ~/Desktop/forklift_warehouse
> colcon build --symlink-install
> source install/setup.bash
> ```

## How It Works

```
┌──────────────────────────────────────────────────────────┐
│  warehouse.launch.py                                      │
│  Spawns Freddy + warehouse world (shelves, tables, boxes) │
│  Publishes odom→base_link TF                             │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  Gazebo Harmonic  ←→  ros_gz_bridge                      │
│  GPU LiDAR → /scan (sensor_msgs/LaserScan)               │
│  /clock (rosgraph_msgs/Clock)                            │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  LiDAR Multi-Class Detector (lidar_box_detector.py)      │
│  Clusters scan → classifies BOX/TABLE/SHELF/WALL         │
│  Publishes:                                              │
│    /detected_objects  (MarkerArray)                       │
│    /detected_box_poses (PoseArray)                        │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  Reactive Navigator (reactive_navigator.py)              │
│  Safety teleop: /cmd_vel → /base_velocity_controller     │
│  LiDAR collision check + safety stop + blink marker      │
│  Publishes: /nav_status, /safety_light                   │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────┐
│  Adaptive Box Transport (adaptive_box_transport.py)      │
│  Pipeline: 25s warmup → search → reach → pick → place   │
│  Publishes: /transport_status                            │
└──────────────────────────────────────────────────────────┘
         ↕
┌──────────────────────────────────────────────────────────┐
│  Ontology Reasoner (ontology_reasoner.py)                │
│  HermiT OWL reasoner (bundled, no external Java)         │
│  Service: /query_ontology   Status: /ontology_status     │
└──────────────────────────────────────────────────────────┘
```

## Launch

### Option 1: Full System (Recommended)

Launches **everything** — Gazebo, RViz, controllers, LiDAR detector, navigator, box transport, and ontology reasoner:

```bash
source /opt/ros/jazzy/setup.bash
source ~/freddy_ws/install/setup.bash
cd ~/Desktop/forklift_warehouse
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_warehouse full_system.launch.py
```

### Option 2: Core Simulation Only

Launches Gazebo + RViz + controllers + LiDAR detector + reactive navigator (no transport or reasoner):

```bash
source /opt/ros/jazzy/setup.bash
source ~/freddy_ws/install/setup.bash
cd ~/Desktop/forklift_warehouse
source install/setup.bash
ros2 launch forklift_warehouse warehouse.launch.py
```

### Running Individual Nodes

Each autonomy node can also be run independently in a separate terminal (after sourcing the workspace):

```bash
# LiDAR Box Detector
ros2 run forklift_warehouse lidar_box_detector.py

# Reactive Navigator (safety teleop)
ros2 run forklift_warehouse reactive_navigator.py

# Autonomous Box Transport
ros2 run forklift_warehouse adaptive_box_transport.py

# Ontology Reasoner
ros2 run forklift_warehouse ontology_reasoner.py

# Manual Keyboard Teleop (requires reactive_navigator running)
ros2 run forklift_warehouse teleop_arrow_keys.py
```

### Querying the Ontology

```bash
ros2 service call /query_ontology std_srvs/srv/Trigger
```

Expected response:

```
success=True
message="Freddy has LiDAR: YES | Capabilities: ['Pick Capability', 'Place Capability',
'Navigate Capability'] | Box_1 on shelf: ['Shelf A'] | Box_2 on shelf: ['Shelf B'] |
Box_3 on shelf: ['Shelf C'] | Box_4 on shelf: ['Shelf D'] |
Ethical constraint (avoids Obstacle ⊔ Human): ACTIVE"
```

### MoveIt 2 (Optional — Motion Planning UI)

```bash
ros2 launch forklift_warehouse moveit.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 360° LiDAR scan from GPU lidar sensor |
| `/detected_objects` | `visualization_msgs/MarkerArray` | Detected object markers for RViz |
| `/detected_box_poses` | `geometry_msgs/PoseArray` | Detected table/box positions |
| `/transport_status` | `std_msgs/String` | Adaptive transport pipeline state |
| `/nav_status` | `std_msgs/String` | Navigator state machine status |
| `/safety_light` | `visualization_msgs/Marker` | Safety strobe light (blinks red when blocked) |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot base velocity commands (from teleop) |
| `/base_velocity_controller/commands` | `std_msgs/Float64MultiArray` | Base wheel velocities (8 wheels) |
| `/ontology_status` | `std_msgs/String` | Ontology system health (published every 5s) |
| `/tf` | `tf2_msgs/TFMessage` | Robot transform tree (incl. odom→base_link) |
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock (bridged from Gazebo) |

## Architecture

```
forklift_warehouse/
├── config/
│   ├── moveit/
│   │   ├── freddy.srdf                 # SRDF (verified against Freddy URDF)
│   │   ├── kinematics.yaml             # KDL solver config for both arms
│   │   ├── ompl_planning.yaml          # OMPL planner config
│   │   └── moveit_controllers.yaml     # MoveIt → ros2_control mapping
│   └── rviz/
│       └── warehouse.rviz              # RViz config (LaserScan, TF, MarkerArray)
├── launch/
│   ├── warehouse.launch.py             # Core simulation (Gazebo + RViz + controllers + autonomy)
│   ├── full_system.launch.py           # Full stack (warehouse + transport + ontology)
│   └── moveit.launch.py                # MoveIt 2 launch (optional)
├── ontology/
│   └── warehouse_robot.owl             # CORA-aligned OWL ontology (10 classes, 7 properties)
├── src/
│   ├── random_world_generator.py       # Randomized SDF warehouse generator (utility)
│   ├── lidar_box_detector.py           # LiDAR → multi-class object detection
│   ├── reactive_navigator.py           # Safety teleop with LiDAR collision avoidance
│   ├── adaptive_box_transport.py       # Autonomous pickup/place pipeline
│   ├── ontology_reasoner.py            # OWL HermiT reasoner + /query_ontology service
│   └── teleop_arrow_keys.py            # Manual keyboard teleoperation
├── urdf/
│   └── freddy_lidar.urdf.xacro        # LiDAR sensor overlay (topic: scan, gz_frame_id set)
├── worlds/
│   └── warehouse.sdf                   # SDF 1.9 warehouse world with shelves, boxes, conveyor
├── models/
│   └── warehouse/                      # Warehouse wall meshes (model.sdf + model.config)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Ontology (warehouse_robot.owl)

The ontology follows the **CORA (Core Ontologies for Robotics and Automation)** alignment and defines 10 classes, 7 object properties, and 4 individuals.

| Class | Constraint |
|-------|-----------|
| `Robot` | `∃hasSensor.LiDAR` — must have a LiDAR sensor |
| `Box` | `∃locatedOn.Shelf` — must be located on a shelf |
| `Robot` | `∀avoids.(Obstacle ⊔ Human)` — ethical avoidance of obstacles and humans |
| `Human` | subclass of `Obstacle` |

### Individuals

| Individual | Type | Properties |
|-----------|------|------------|
| `Freddy` | `Robot` | `hasSensor: LiDAR_1`, `hasCapability: Pick, Place, Navigate`, `performs: BoxTransportTask` |
| `Box_1..4` | `Box` | `locatedOn: Shelf A..D` |

## Issues Fixed

| Issue | Category | Resolution |
|-------|----------|------------|
| ROS 1 plugins in `forklift_gazebo` | Build | Legacy dirs (`.git_publish*`) now have `COLCON_IGNORE` — never compiled |
| `freddy_description` not found | Build | Documented workspace sourcing order in README |
| Pellet reasoner Java 21+ failure | Runtime | Switched to **HermiT** reasoner (bundled with owlready2, no Java needed) |
| SDF version 1.4/1.7 outdated | Build | Updated `warehouse.sdf` to SDF **1.9** |
| Dead code (`box_transport.py`) | Structural | Removed (superseded by `adaptive_box_transport.py`) |
| `mission_controller.py`, `obstacle_avoidance.py` unregistered | Structural | Legacy code only in `.git_publish*` dirs — isolated via `COLCON_IGNORE` |
| LiDAR topic mismatch (`/lidar` vs `/scan`) | Integration | Fixed `freddy_lidar.urdf.xacro`: topic `scan` + `gz_frame_id` added |
| No TF odom→base_link | Runtime | Added `static_transform_publisher` in `warehouse.launch.py` |
| Duplicate LiDAR/navigator in `full_system.launch.py` | Integration | Removed duplicates — now only adds `adaptive_box_transport` + `ontology_reasoner` |
| Empty `plugins/` and `include/` dirs | Structural | Removed |
| Duplicate `forklift_warehouse` packages | Build | `.git_publish*` dirs have `COLCON_IGNORE` — no name collision |
| URDF/Xacro not used in launch | Integration | `warehouse.launch.py` injects LiDAR inline; `freddy_lidar.urdf.xacro` now consistent |
| Bridge syntax | Integration | `[` is correct unidirectional Gazebo→ROS syntax — verified, no change needed |
| `.python-version` misleading | Structural | Only in legacy `.git_publish_2` dir — isolated via `COLCON_IGNORE` |
| MoveIt SRDF joint mismatch | Integration | **Verified**: SRDF joints match actual Freddy URDF exactly |
| Missing `tf2_ros` dependency | Build | Added to `package.xml` |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **`package 'freddy_description' not found`** | Source the Freddy workspace first: `source ~/freddy_ws/install/setup.bash` |
| **No `/scan` data** | Check that `ros_gz_bridge` is running (`ros2 node list \| grep bridge`) and Gazebo is unpaused |
| **Controllers fail silently** | Wait at least 20s after launch — controllers load sequentially with built-in delays |
| **Robot falls through floor** | Check `max_step_size` ≤ 0.01 in `warehouse.sdf` |
| **Owlready2 import error** | Run `pip install owlready2` |
| **RobotModel error in RViz** | Cosmetic — URDF loads but some mesh paths may not resolve in RViz |
| **`gz_ros2_control` warning** | Harmless warning from missing optional file in Freddy workspace |
| **Duplicate node instances** | Use `full_system.launch.py` (not `warehouse.launch.py` + manual node starts) |

## License

Apache-2.0
