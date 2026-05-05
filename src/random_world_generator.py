#!/usr/bin/env python3
"""
Random Warehouse World Generator.

Generates a Gazebo SDF world with randomized shelf positions, a pickup table
with a box, and static obstacles. Each run produces a unique layout.

Usage:
  python3 random_world_generator.py [--output /tmp/warehouse_dynamic.sdf]
"""

import argparse
import random
import math
import sys
import os


# --- World constants ---
WAREHOUSE_X_MIN, WAREHOUSE_X_MAX = -8.0, 12.0
WAREHOUSE_Y_MIN, WAREHOUSE_Y_MAX = -10.0, 12.0
SHELF_KEEP_OUT = 3.0            # min distance between shelves
ROBOT_SPAWN_CLEARANCE = 3.0     # keep area around origin clear for Freddy
BOX_SIZE = 0.2
BOX_MASS = 1.0
BOX_Z = 0.85                    # box sits on shelf surface
TABLE_HEIGHT = 0.75
TABLE_TOP_THICKNESS = 0.05
TABLE_LEG_RADIUS = 0.03
TABLE_SIZE_X = 0.8
TABLE_SIZE_Y = 0.6


# ── SDF Templates ─────────────────────────────────────────────────────────────

SDF_HEADER = """\
<?xml version='1.0' encoding='ASCII'?>
<sdf version='1.7'>
  <world name='warehouse_world'>
    <gravity>0 0 -9.8</gravity>
    <physics type='ode'>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <max_contacts>20</max_contacts>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>

    <gui fullscreen="0">
      <plugin filename="GzScene3D" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>13.4 -6.1 2.23 0 0.4 -1.83</camera_pose>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <ignition-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>true</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <ignition-gui>
          <property key="state" type="string">docked_collapsed</property>
        </ignition-gui>
      </plugin>
    </gui>

    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>0</cast_shadows>
      <pose>-5 -3 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>1 1 1 1</specular>
      <direction>0 0 -1</direction>
    </light>

    <!-- Ground plane -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0.0 0.0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
        </collision>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>

    <!-- Base warehouse structure (walls, floor) -->
    <include>
      <uri>
        https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Warehouse
      </uri>
      <name>warehouse</name>
      <pose>0 0 -0.09 0 0 0</pose>
    </include>
"""

SDF_FOOTER = """\
  </world>
</sdf>
"""

SHELF_TEMPLATE = """\
    <!-- Shelf: {name} -->
    <include>
      <uri>
        https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf
      </uri>
      <name>{name}</name>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>
    </include>
"""

BOX_TEMPLATE = """\
    <!-- Box: {name} on {shelf_name} -->
    <model name="{name}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>0.004</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.004</iyy><iyz>0</iyz>
            <izz>0.004</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box><size>{s} {s} {s}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{s} {s} {s}</size></box>
          </geometry>
          <material>
            <ambient>{cr} {cg} {cb} 1</ambient>
            <diffuse>{cr} {cg} {cb} 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

TABLE_TEMPLATE = """\
    <!-- Pickup Table: {name} -->
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <!-- Table top -->
      <link name="tabletop">
        <pose>0 0 {top_z:.4f} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box><size>{sx:.3f} {sy:.3f} {thick:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx:.3f} {sy:.3f} {thick:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.55 0.27 0.07 1</ambient>
            <diffuse>0.55 0.27 0.07 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
      <!-- Leg 1 -->
      <link name="leg1">
        <pose>{lx:.3f} {ly:.3f} {leg_z:.4f} 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
      </link>
      <!-- Leg 2 -->
      <link name="leg2">
        <pose>{lx:.3f} {nly:.3f} {leg_z:.4f} 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
      </link>
      <!-- Leg 3 -->
      <link name="leg3">
        <pose>{nlx:.3f} {ly:.3f} {leg_z:.4f} 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
      </link>
      <!-- Leg 4 -->
      <link name="leg4">
        <pose>{nlx:.3f} {nly:.3f} {leg_z:.4f} 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{lr}</radius><length>{lh:.3f}</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
      </link>
    </model>
"""

OBSTACLE_TEMPLATE = """\
    <!-- Static obstacle: {name} -->
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {hz:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

HUMAN_TEMPLATE = """\
    <!-- Dynamic obstacle: human -->
    <model name="human_obstacle">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.9 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.3</radius><length>1.8</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.3</radius><length>1.8</length></cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.7 0.5 1</ambient>
            <diffuse>0.9 0.7 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


# ── Helpers ────────────────────────────────────────────────────────────────────

def _too_close(x, y, placed, min_dist):
    """Return True if (x,y) is within min_dist of any placed point."""
    for px, py in placed:
        if math.hypot(x - px, y - py) < min_dist:
            return True
    return False


def _rand_pos(placed, min_dist, x_range, y_range, max_tries=200):
    """Return a random (x, y) that is min_dist away from all placed points."""
    for _ in range(max_tries):
        x = random.uniform(*x_range)
        y = random.uniform(*y_range)
        if not _too_close(x, y, placed, min_dist):
            return x, y
    # fallback: return last attempt
    return x, y


BOX_COLORS = [
    (0.9, 0.2, 0.1),   # red
    (0.1, 0.2, 0.9),   # blue
    (0.1, 0.8, 0.2),   # green
    (0.9, 0.8, 0.1),   # yellow
    (0.7, 0.1, 0.8),   # purple
]


# ── Generator ─────────────────────────────────────────────────────────────────

def generate_world(output_path: str):
    """Write a randomized warehouse SDF to *output_path*."""

    placed = [(0.0, 0.0)]  # reserve robot spawn area

    lines = [SDF_HEADER]

    # --- Shelves + Boxes on shelves ---
    num_shelves = random.randint(4, 8)
    for i in range(num_shelves):
        x, y = _rand_pos(
            placed, SHELF_KEEP_OUT,
            (WAREHOUSE_X_MIN, WAREHOUSE_X_MAX),
            (WAREHOUSE_Y_MIN, WAREHOUSE_Y_MAX),
        )
        placed.append((x, y))
        yaw = random.choice([0.0, math.pi / 2])
        shelf_name = f'shelf_rnd_{i}'
        lines.append(SHELF_TEMPLATE.format(name=shelf_name, x=x, y=y, yaw=yaw))

        # Place a box on this shelf (offset slightly in front)
        bx = x + 0.9 * math.cos(yaw)
        by = y + 0.9 * math.sin(yaw)
        color = random.choice(BOX_COLORS)
        lines.append(BOX_TEMPLATE.format(
            name=f'box_shelf_{i}', shelf_name=shelf_name,
            x=bx, y=by, z=BOX_Z,
            mass=BOX_MASS, s=BOX_SIZE,
            cr=color[0], cg=color[1], cb=color[2],
        ))

    # --- Pickup Table + Target Box (THE box the robot must find) ---
    table_x, table_y = _rand_pos(
        placed, 3.5,
        (WAREHOUSE_X_MIN + 2, WAREHOUSE_X_MAX - 2),
        (WAREHOUSE_Y_MIN + 2, WAREHOUSE_Y_MAX - 2),
    )
    placed.append((table_x, table_y))

    lx = TABLE_SIZE_X / 2 - 0.05
    ly = TABLE_SIZE_Y / 2 - 0.05
    lines.append(TABLE_TEMPLATE.format(
        name='pickup_table',
        x=table_x, y=table_y,
        sx=TABLE_SIZE_X, sy=TABLE_SIZE_Y,
        thick=TABLE_TOP_THICKNESS,
        top_z=TABLE_HEIGHT,
        lx=lx, ly=ly, nlx=-lx, nly=-ly,
        lr=TABLE_LEG_RADIUS,
        lh=TABLE_HEIGHT,
        leg_z=TABLE_HEIGHT / 2,
    ))

    # Target box on the table
    target_box_z = TABLE_HEIGHT + TABLE_TOP_THICKNESS / 2 + BOX_SIZE / 2
    target_color = (0.9, 0.1, 0.1)  # bright red — the target
    lines.append(BOX_TEMPLATE.format(
        name='target_box', shelf_name='pickup_table',
        x=table_x, y=table_y, z=target_box_z,
        mass=BOX_MASS, s=BOX_SIZE,
        cr=target_color[0], cg=target_color[1], cb=target_color[2],
    ))

    # --- Static obstacles ---
    num_obstacles = random.randint(2, 4)
    for i in range(num_obstacles):
        x, y = _rand_pos(
            placed, 2.0,
            (WAREHOUSE_X_MIN, WAREHOUSE_X_MAX),
            (WAREHOUSE_Y_MIN, WAREHOUSE_Y_MAX),
        )
        placed.append((x, y))
        sx = random.uniform(0.4, 1.5)
        sy = random.uniform(0.4, 1.5)
        sz = random.uniform(0.5, 2.0)
        lines.append(OBSTACLE_TEMPLATE.format(
            name=f'obstacle_{i}', x=x, y=y,
            sx=sx, sy=sy, sz=sz, hz=sz / 2.0,
        ))

    # --- Optional human obstacle ---
    if random.random() < 0.5:
        x, y = _rand_pos(
            placed, 2.0,
            (WAREHOUSE_X_MIN + 2, WAREHOUSE_X_MAX - 2),
            (WAREHOUSE_Y_MIN + 2, WAREHOUSE_Y_MAX - 2),
        )
        placed.append((x, y))
        lines.append(HUMAN_TEMPLATE.format(x=x, y=y))

    lines.append(SDF_FOOTER)

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f'[WorldGen] Wrote randomized warehouse → {output_path}')
    print(f'[WorldGen]   Shelves: {num_shelves}  Table: ({table_x:.1f}, {table_y:.1f})'
          f'  Obstacles: {num_obstacles}')


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Generate a randomized warehouse SDF')
    parser.add_argument('--output', '-o', default='/tmp/warehouse_dynamic.sdf',
                        help='Output SDF path (default: /tmp/warehouse_dynamic.sdf)')
    args = parser.parse_args()
    generate_world(args.output)


if __name__ == '__main__':
    main()
