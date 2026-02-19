# Gazebo ROS 2 Simulations

This repository contains various ROS 2 simulations for Gazebo, including a forklift, a mobile manipulator, a hydraulic bot, and a custom robot arm.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
    - [Forklift Simulation](#forklift-simulation)
    - [Forklift Warehouse](#forklift-warehouse)
    - [Hydraulic Bot](#hydraulic-bot)
    - [Mobile Manipulator](#mobile-manipulator)
    - [Custom Robot](#custom-robot)
    - [Warehouse Simulation](#warehouse-simulation)

## Prerequisites

- **ROS 2** (Humble Hawksbill or newer recommended)
- **Gazebo**
- **Python 3**
- **Colcon** build tool

## Installation

1.  **Clone the repository**:

    ```bash
    mkdir -p ~/gazebo_ros2_ws/src
    cd ~/gazebo_ros2_ws/src
    git clone https://github.com/Jaikrrish5959/Gazebo_Ros2.git .
    ```

2.  **Install dependencies**:

    ```bash
    cd ~/gazebo_ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the workspace**:

    ```bash
    colcon build
    ```

4.  **Source the setup script**:

    ```bash
    source install/setup.bash
    ```

    *Tip: Add `source ~/gazebo_ros2_ws/install/setup.bash` to your `~/.bashrc` to source it automatically.*

## Usage

Run the following commands to launch the simulations. Ensure you have sourced the workspace first.

### Forklift Simulation

Launch the standalone forklift simulation:

```bash
ros2 launch forklift_gazebo forklift.launch
```

Or with a shifted start position:

```bash
ros2 launch forklift_gazebo fork_lift_shift.launch
```

### Forklift Warehouse

Launch the forklift in a warehouse environment:

```bash
ros2 launch forklift_warehouse warehouse.launch.py
```

### Hydraulic Bot

Spawn the hydraulic robot:

```bash
ros2 launch hydraulic_bot spawn_robot.launch.py
```

### Mobile Manipulator

Launch the mobile manipulator simulation:

```bash
ros2 launch mobile_manipulator_sim simulation.launch.py
```

### Custom Robot

Launch the custom robot in Gazebo:

```bash
ros2 launch my_robot_gazebo gazebo.launch.py
```

Spawn the robot description:

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

### Warehouse Simulation

Launch the generic warehouse simulation:

```bash
ros2 launch ware warehouse.launch.py
```
