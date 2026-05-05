#!/usr/bin/env python3
"""
Autonomous Wall-Follow + Random Explore Navigator.

The robot explores the warehouse randomly while avoiding obstacles,
searching for the pickup table. Once the table is detected by the
LiDAR table detector, it switches to direct approach.

State machine:
    RANDOM_EXPLORE → TABLE_DETECTED → APPROACH → ARRIVED

Topics:
  Subscribed:
    /scan                        — sensor_msgs/LaserScan
    /detected_box_poses          — geometry_msgs/PoseArray  (table positions)
  Published:
    /base_velocity_controller/commands — std_msgs/Float64MultiArray
    /nav_status                  — std_msgs/String
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray, String


class WallFollowNavigator(Node):

    # ── Tuning parameters ─────────────────────────────────────────────────
    SAFETY_RADIUS = 0.5          # hard stop if obstacle inside this (m)
    MAX_SPEED = 5.0              # max forward wheel velocity (rad/s)
    TURN_SPEED = 3.5             # rotation speed (rad/s)
    NUM_WHEELS = 8               # Freddy KELO wheels

    ARRIVAL_DIST = 0.8           # close enough to declare ARRIVED (m)
    TABLE_DETECT_DIST = 4.0      # switch to APPROACH when table < this dist

    # Random exploration parameters
    EXPLORE_FORWARD_TIME = 2.0   # drive forward for this long before checking
    EXPLORE_TURN_TIME = 1.0      # turn duration when blocked

    def __init__(self):
        super().__init__('reactive_navigator')

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_be)
        self.pose_sub = self.create_subscription(
            PoseArray, '/detected_box_poses', self._pose_cb, 10)

        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/base_velocity_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/nav_status', 10)

        self.target = None          # (x, y) in base_link frame
        self.latest_scan = None
        self.state = 'RANDOM_EXPLORE'
        self.explore_action = 'FORWARD'
        self.action_start = time.time()
        self.turn_direction = random.choice(['left', 'right'])
        self.explore_turn_count = 0

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('Navigator ready — RANDOM EXPLORE mode')
        self.get_logger().info('Searching for the pickup table...')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _pose_cb(self, msg: PoseArray):
        if not msg.poses:
            self.target = None
            return
        # Pick nearest table
        best = None
        best_dist = float('inf')
        for p in msg.poses:
            d = math.hypot(p.position.x, p.position.y)
            if d < best_dist:
                best_dist = d
                best = (p.position.x, p.position.y)
        self.target = best

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        if self.latest_scan is None:
            return

        scan = self.latest_scan
        front_min = self._sector_min(scan, -30, 30)
        left_min = self._sector_min(scan, 60, 120)
        right_min = self._sector_min(scan, -120, -60)

        # ── Check if table is detected and close enough ───────────────────
        if self.target is not None:
            tx, ty = self.target
            target_dist = math.hypot(tx, ty)

            if target_dist < self.ARRIVAL_DIST:
                self._stop()
                self.state = 'ARRIVED'
                self._publish_status('ARRIVED')
                self.get_logger().info('✓ ARRIVED at pickup table!')
                return

            if target_dist < self.TABLE_DETECT_DIST:
                if self.state != 'APPROACH':
                    self.state = 'APPROACH'
                    self.get_logger().info(
                        f'TABLE DETECTED at {target_dist:.1f}m — approaching!')
                self._approach_target(tx, ty, target_dist, front_min)
                self._publish_status(
                    f'APPROACH — table at {target_dist:.1f}m')
                return

        # ── Random exploration with obstacle avoidance ────────────────────
        self.state = 'RANDOM_EXPLORE'
        elapsed = time.time() - self.action_start

        # Emergency: frontal obstacle
        if front_min < self.SAFETY_RADIUS:
            self._stop()
            # Pick a random turn direction
            self.explore_action = 'TURN'
            self.turn_direction = 'left' if left_min > right_min else 'right'
            self.action_start = time.time()
            self._publish_status(
                f'EXPLORE — obstacle ahead, turning {self.turn_direction}')
            self._rotate(self.turn_direction)
            return

        if self.explore_action == 'FORWARD':
            # Drive forward with slight random drift
            speed = self.MAX_SPEED * 0.6

            # Add slight drift every few seconds for more random coverage
            if elapsed > self.EXPLORE_FORWARD_TIME:
                # Randomly decide: keep going, turn, or drift
                choice = random.random()
                if choice < 0.3:
                    # Random turn
                    self.explore_action = 'TURN'
                    self.turn_direction = random.choice(['left', 'right'])
                    self.action_start = time.time()
                    self.explore_turn_count += 1
                elif choice < 0.5:
                    # Drift (slight curve)
                    self.explore_action = 'DRIFT'
                    self.turn_direction = random.choice(['left', 'right'])
                    self.action_start = time.time()
                else:
                    # Keep going forward (reset timer)
                    self.action_start = time.time()
            else:
                self._drive_straight(speed)

            self._publish_status(
                f'EXPLORE — forward (L={left_min:.1f} F={front_min:.1f} '
                f'R={right_min:.1f})')

        elif self.explore_action == 'TURN':
            # Execute a random turn
            if elapsed < self.EXPLORE_TURN_TIME:
                self._rotate(self.turn_direction)
                self._publish_status(
                    f'EXPLORE — turning {self.turn_direction}')
            else:
                self.explore_action = 'FORWARD'
                self.action_start = time.time()

        elif self.explore_action == 'DRIFT':
            # Drive with a slight curve (wall-like following)
            if elapsed < 1.5:
                speed = self.MAX_SPEED * 0.5
                if self.turn_direction == 'left':
                    self._drive_differential(speed * 0.6, speed)
                else:
                    self._drive_differential(speed, speed * 0.6)
                self._publish_status(
                    f'EXPLORE — drifting {self.turn_direction}')
            else:
                self.explore_action = 'FORWARD'
                self.action_start = time.time()

    # ── Approach target directly ──────────────────────────────────────────

    def _approach_target(self, tx, ty, dist, front_min):
        """Drive directly toward the detected table."""
        angle_to_target = math.atan2(ty, tx)

        if front_min < self.SAFETY_RADIUS:
            self._stop()
            return

        # Slow down as we get closer
        speed_scale = max(0.3, min(1.0, dist / self.TABLE_DETECT_DIST))
        speed = self.MAX_SPEED * speed_scale

        if abs(angle_to_target) > 0.15:
            # Rotate toward target
            if angle_to_target > 0:
                self._drive_differential(speed * 0.2, speed * 0.8)
            else:
                self._drive_differential(speed * 0.8, speed * 0.2)
        else:
            self._drive_straight(speed)

    # ── Motor helpers ─────────────────────────────────────────────────────

    def _sector_min(self, scan: LaserScan, deg_start: float, deg_end: float) -> float:
        n = len(scan.ranges)
        if n == 0:
            return float('inf')
        idx_start = int((math.radians(deg_start) - scan.angle_min) / scan.angle_increment)
        idx_end = int((math.radians(deg_end) - scan.angle_min) / scan.angle_increment)
        idx_start = max(0, min(idx_start, n - 1))
        idx_end = max(0, min(idx_end, n - 1))
        if idx_start > idx_end:
            idx_start, idx_end = idx_end, idx_start
        min_r = float('inf')
        for i in range(idx_start, idx_end + 1):
            r = scan.ranges[i]
            if not (math.isinf(r) or math.isnan(r)) and r > scan.range_min:
                min_r = min(min_r, r)
        return min_r

    def _drive_straight(self, speed: float):
        msg = Float64MultiArray()
        msg.data = [speed] * self.NUM_WHEELS
        self.vel_pub.publish(msg)

    def _drive_differential(self, left: float, right: float):
        msg = Float64MultiArray()
        msg.data = [left] * 4 + [right] * 4
        self.vel_pub.publish(msg)

    def _rotate(self, direction: str):
        s = self.TURN_SPEED
        if direction == 'left':
            self._drive_differential(-s, s)
        else:
            self._drive_differential(s, -s)

    def _stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * self.NUM_WHEELS
        self.vel_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
