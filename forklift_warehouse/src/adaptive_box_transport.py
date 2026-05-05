#!/usr/bin/env python3
"""
Adaptive Box Transport — Fully Autonomous Pipeline.

Orchestrates the full autonomous box pickup sequence:
  1. Wait for controllers & sensors (startup delay)
  2. Let wall-following navigator search the warehouse
  3. Wait for navigator to reach the target box/table
  4. Execute arm pickup sequence
  5. Reverse from table
  6. Place box and return home

Runs entirely via timer callbacks — no blocking spin.

Usage (launched automatically via warehouse.launch.py):
  ros2 run forklift_warehouse adaptive_box_transport.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray, String


class AdaptiveBoxTransport(Node):

    # Left arm joint names (Kinova Gen3 7-DOF)
    LEFT_ARM_JOINTS = [
        'kinova_left_joint_1',
        'kinova_left_joint_2',
        'kinova_left_joint_3',
        'kinova_left_joint_4',
        'kinova_left_joint_5',
        'kinova_left_joint_6',
        'kinova_left_joint_7',
    ]

    # Named arm poses (radians)
    POSE_HOME  = [0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57]
    POSE_REACH = [0.0, 0.0,  0.0,  -1.57, 0.0, -1.0, 0.0]
    POSE_GRASP = [0.0, 0.3,  0.0,  -1.2,  0.0, -1.2, 0.0]
    POSE_CARRY = [0.0, 0.5,  3.14, -2.0,  0.0,  0.7, 1.57]
    POSE_PLACE = [0.0, 0.0,  0.0,  -1.57, 0.0, -1.0, 0.0]

    # Drive parameters
    REVERSE_SPEED = -5.0
    REVERSE_DURATION = 3.0

    # Pipeline stages
    STAGES = [
        'STARTUP_WAIT',
        'ARM_HOME',
        'WALL_FOLLOW_SEARCH',
        'WAITING_ARRIVAL',
        'ARM_REACH',
        'ARM_GRASP',
        'ARM_CARRY',
        'REVERSE',
        'ARM_PLACE',
        'ARM_HOME_FINAL',
        'COMPLETE',
    ]

    def __init__(self):
        super().__init__('adaptive_box_transport')

        self.left_arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_left_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.base_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/base_velocity_controller/commands',
            10
        )

        # Subscribe to detected box poses and nav status
        self.detected_boxes = []
        self.nav_status = ''

        self.create_subscription(
            PoseArray, '/detected_box_poses', self._box_cb, 10)
        self.create_subscription(
            String, '/nav_status', self._nav_cb, 10)

        # Pipeline state
        self.stage = 'STARTUP_WAIT'
        self.stage_start_time = time.time()
        self.arm_busy = False

        # Status publisher
        self.status_pub = self.create_publisher(String, '/transport_status', 10)

        # Main loop timer (2 Hz — non-blocking)
        self.timer = self.create_timer(0.5, self._pipeline_tick)

        self.get_logger().info('=== Adaptive Box Transport Node Ready ===')
        self.get_logger().info('Pipeline will start after 25s controller warmup')

    # ── Subscription callbacks ────────────────────────────────────────────

    def _box_cb(self, msg: PoseArray):
        self.detected_boxes = [
            (p.position.x, p.position.y) for p in msg.poses
        ]

    def _nav_cb(self, msg: String):
        self.nav_status = msg.data

    # ── Pipeline tick (non-blocking) ──────────────────────────────────────

    def _pipeline_tick(self):
        """State machine tick — runs at 2 Hz."""
        elapsed = time.time() - self.stage_start_time

        if self.arm_busy:
            return  # wait for arm action to complete

        status = f'{self.stage} ({elapsed:.0f}s)'
        self._pub_status(status)

        if self.stage == 'STARTUP_WAIT':
            # Wait 25 seconds for controllers to load
            if elapsed > 25.0:
                self._advance('ARM_HOME')
                self.get_logger().info('')
                self.get_logger().info('╔══════════════════════════════════════════╗')
                self.get_logger().info('║  AUTONOMOUS BOX TRANSPORT — STARTING    ║')
                self.get_logger().info('╚══════════════════════════════════════════╝')

        elif self.stage == 'ARM_HOME':
            self._send_arm(self.POSE_HOME, 'HOME', 3.0, 'WALL_FOLLOW_SEARCH')

        elif self.stage == 'WALL_FOLLOW_SEARCH':
            # The navigator is doing wall-following autonomously.
            # We just wait and log what it's doing.
            if elapsed > 5.0 and int(elapsed) % 10 == 0:
                n_boxes = len(self.detected_boxes)
                self.get_logger().info(
                    f'[SEARCH] Wall-following... nav={self.nav_status} '
                    f'boxes_detected={n_boxes}')

            # Check if navigator has arrived at target
            if 'ARRIVED' in self.nav_status:
                self.get_logger().info('Navigator reports ARRIVED at target!')
                self._advance('ARM_REACH')
            elif elapsed > 180.0:
                self.get_logger().warn('Search timeout (180s) — attempting pickup anyway')
                self._advance('ARM_REACH')

        elif self.stage == 'WAITING_ARRIVAL':
            if 'ARRIVED' in self.nav_status or elapsed > 60.0:
                self._advance('ARM_REACH')

        elif self.stage == 'ARM_REACH':
            self._send_arm(self.POSE_REACH, 'REACH', 3.0, 'ARM_GRASP')

        elif self.stage == 'ARM_GRASP':
            self._send_arm(self.POSE_GRASP, 'GRASP', 2.0, 'ARM_CARRY')

        elif self.stage == 'ARM_CARRY':
            self._send_arm(self.POSE_CARRY, 'CARRY', 3.0, 'REVERSE')

        elif self.stage == 'REVERSE':
            self.get_logger().info('Reversing from table...')
            self._drive_timed(self.REVERSE_SPEED, self.REVERSE_DURATION)
            self._advance('ARM_PLACE')

        elif self.stage == 'ARM_PLACE':
            self._send_arm(self.POSE_PLACE, 'PLACE', 3.0, 'ARM_HOME_FINAL')

        elif self.stage == 'ARM_HOME_FINAL':
            self._send_arm(self.POSE_HOME, 'HOME (final)', 3.0, 'COMPLETE')

        elif self.stage == 'COMPLETE':
            if elapsed < 2.0:
                self.get_logger().info('')
                self.get_logger().info('╔══════════════════════════════════════════╗')
                self.get_logger().info('║  BOX TRANSPORT COMPLETE!                ║')
                self.get_logger().info('╚══════════════════════════════════════════╝')

    # ── Stage management ──────────────────────────────────────────────────

    def _advance(self, next_stage: str):
        self.get_logger().info(f'[Pipeline] {self.stage} → {next_stage}')
        self.stage = next_stage
        self.stage_start_time = time.time()

    def _pub_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # ── Arm control (async) ───────────────────────────────────────────────

    def _send_arm(self, pose, label, secs, next_stage):
        """Send arm goal asynchronously and advance on completion."""
        self.get_logger().info(f'Arm → {label}')

        if not self.left_arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'Arm action server unavailable — skipping {label}')
            self._advance(next_stage)
            return

        traj = JointTrajectory()
        traj.joint_names = self.LEFT_ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = pose
        pt.velocities = [0.0] * 7
        pt.time_from_start = Duration(seconds=secs).to_msg()
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.arm_busy = True
        future = self.left_arm_action.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._arm_goal_response(f, label, next_stage))

    def _arm_goal_response(self, future, label, next_stage):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn(f'Arm goal {label} rejected — skipping')
            self.arm_busy = False
            self._advance(next_stage)
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._arm_result(f, label, next_stage))

    def _arm_result(self, future, label, next_stage):
        self.get_logger().info(f'Arm → {label} ✓')
        self.arm_busy = False
        self._advance(next_stage)

    # ── Base drive ────────────────────────────────────────────────────────

    def _drive_timed(self, speed, duration):
        """Drive all wheels at the same velocity for duration (blocking)."""
        msg = Float64MultiArray()
        msg.data = [speed] * 8
        start = time.time()
        while time.time() - start < duration:
            self.base_vel_pub.publish(msg)
            time.sleep(0.02)
        msg.data = [0.0] * 8
        self.base_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveBoxTransport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cancelled.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
