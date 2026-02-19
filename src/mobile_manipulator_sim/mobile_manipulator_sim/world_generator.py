#!/usr/bin/env python3
"""
Script to generate a randomized SDF world file for the mobile manipulator simulation.
"""

import random
import os

def generate_world(output_path):
    # Random seed based on time for variety
    random.seed()
    
    # Target box position: 3-7m away from origin, random angle
    target_distance = random.uniform(3.0, 6.0)
    target_angle = random.uniform(-0.8, 0.8)  # radians, roughly ±45 degrees
    target_x = target_distance * (1.0 + 0.3 * target_angle)  # mostly forward
    target_y = target_distance * target_angle * 0.5
    
    # Ensure target is at least 3m away
    while (target_x**2 + target_y**2)**0.5 < 3.0:
        target_x += 0.5
    
    # Generate obstacles - avoid blocking direct path too much
    obstacles = []
    num_obstacles = random.randint(3, 5)
    
    for i in range(num_obstacles):
        # Place obstacles between robot and target, but offset to sides
        obs_x = random.uniform(1.5, target_x - 1.0)
        obs_y = random.uniform(-2.5, 2.5)
        
        # Don't place obstacle too close to origin or target
        if obs_x < 1.0 or ((obs_x - target_x)**2 + (obs_y - target_y)**2)**0.5 < 1.0:
            continue
            
        # Don't place obstacles on top of each other
        too_close = False
        for ox, oy in obstacles:
            if ((obs_x - ox)**2 + (obs_y - oy)**2)**0.5 < 0.8:
                too_close = True
                break
        
        if not too_close:
            obstacles.append((obs_x, obs_y))
    
    # Build SDF content
    obstacle_models = ""
    for i, (ox, oy) in enumerate(obstacles):
        obstacle_models += f'''
    <!-- Obstacle {i+1} -->
    <model name="obstacle_{i+1}">
      <pose>{ox:.2f} {oy:.2f} 0.25 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
    
    sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="task_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>100</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{obstacle_models}

    <!-- Target Box -->
    <model name="target_box">
      <pose>{target_x:.2f} {target_y:.2f} 0.15 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.002</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.002</iyy><iyz>0</iyz>
            <izz>0.002</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box><size>0.15 0.15 0.15</size></box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.15 0.15 0.15</size></box>
          </geometry>
          <material>
            <ambient>0.1 0.9 0.1 1</ambient>
            <diffuse>0.1 0.9 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      <!-- Pose Publisher for ground truth -->
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>10</update_frequency>
      </plugin>
    </model>

  </world>
</sdf>
'''
    
    with open(output_path, 'w') as f:
        f.write(sdf_content)
    
    print(f"Generated world with target at ({target_x:.2f}, {target_y:.2f}) and {len(obstacles)} obstacles")
    return target_x, target_y

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        output = sys.argv[1]
    else:
        output = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'task_world.sdf')
    generate_world(output)
