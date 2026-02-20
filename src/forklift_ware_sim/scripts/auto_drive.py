#!/usr/bin/env python3
"""
Fully automated forklift demo:
  1. Drive to pallet stand
  2. Insert forks under cargo
  3. Lift cargo
  4. Reverse with cargo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
import time
from enum import Enum, auto


class Phase(Enum):
    WAITING = auto()
    DRIVING = auto()
    APPROACHING = auto()
    LIFTING = auto()
    REVERSING = auto()
    DONE = auto()


class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')

        # Target positions
        self.stand_x = 3.0       # Stand is at x=3.0
        self.stop_x = 2.3        # Stop here before inserting forks
        self.insert_x = 2.85     # Drive forward to here to insert forks

        # State
        self.phase = Phase.WAITING
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.start_time = time.time()
        self.phase_start_time = 0.0
        self.lift_pos = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('='*50)
        self.get_logger().info('  AUTOMATED FORKLIFT DEMO')
        self.get_logger().info('  Sequence: Drive → Insert → Lift → Reverse')
        self.get_logger().info('='*50)

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def publish_drive(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def publish_lift(self, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lift']
        msg.position = [position]
        self.joint_pub.publish(msg)

    def enter_phase(self, phase):
        self.phase = phase
        self.phase_start_time = time.time()
        self.get_logger().info(f'>>> Phase: {phase.name}')

    def phase_elapsed(self):
        return time.time() - self.phase_start_time

    def control_loop(self):
        elapsed = time.time() - self.start_time

        # ---- PHASE: WAITING ----
        if self.phase == Phase.WAITING:
            if elapsed > 5.0:
                self.enter_phase(Phase.DRIVING)
            return

        # ---- PHASE: DRIVING to stand ----
        elif self.phase == Phase.DRIVING:
            if self.odom_received:
                # Odom-based: drive until close to stop_x
                dist = self.stop_x - self.current_x
                if dist < 0.1:
                    self.publish_drive(0.0)
                    self.enter_phase(Phase.APPROACHING)
                    return

                # P-controller toward stop position
                target_yaw = math.atan2(-self.current_y, dist)
                yaw_err = target_yaw - self.current_yaw
                while yaw_err > math.pi: yaw_err -= 2*math.pi
                while yaw_err < -math.pi: yaw_err += 2*math.pi

                speed = min(0.5, max(0.15, dist * 0.3))
                self.publish_drive(speed, yaw_err * 1.0)
            else:
                # Timer fallback: drive forward for ~7.5s at 0.3 m/s ≈ 2.25m
                if self.phase_elapsed() > 7.5:
                    self.publish_drive(0.0)
                    self.enter_phase(Phase.APPROACHING)
                else:
                    self.publish_drive(0.3, 0.0)

        # ---- PHASE: APPROACHING (inserting forks under cargo) ----
        elif self.phase == Phase.APPROACHING:
            if self.odom_received:
                dist = self.insert_x - self.current_x
                if dist < 0.05:
                    self.publish_drive(0.0)
                    self.enter_phase(Phase.LIFTING)
                    return
                self.publish_drive(min(0.15, dist * 0.3), 0.0)
            else:
                # Timer fallback: creep for ~4s at 0.15 m/s ≈ 0.6m
                if self.phase_elapsed() > 4.0:
                    self.publish_drive(0.0)
                    self.enter_phase(Phase.LIFTING)
                else:
                    self.publish_drive(0.15, 0.0)

        # ---- PHASE: LIFTING the cargo ----
        elif self.phase == Phase.LIFTING:
            self.publish_drive(0.0)  # Stay still

            # Gradually raise lift from 0 to 0.2 over 4 seconds
            t = self.phase_elapsed()
            if t < 4.0:
                self.lift_pos = min(0.2, t * 0.05)
                self.publish_lift(self.lift_pos)
            else:
                self.publish_lift(0.2)
                self.get_logger().info('  Cargo lifted!')
                self.enter_phase(Phase.REVERSING)

        # ---- PHASE: REVERSING with cargo ----
        elif self.phase == Phase.REVERSING:
            self.publish_lift(0.2)  # Keep cargo lifted

            if self.phase_elapsed() < 5.0:
                self.publish_drive(-0.3, 0.0)
            else:
                self.publish_drive(0.0)
                self.enter_phase(Phase.DONE)

        # ---- PHASE: DONE ----
        elif self.phase == Phase.DONE:
            self.publish_drive(0.0)
            self.publish_lift(0.2)
            if self.phase_elapsed() < 1.0:
                self.get_logger().info('='*50)
                self.get_logger().info('  DEMO COMPLETE!')
                self.get_logger().info('  Cargo has been picked up and moved.')
                self.get_logger().info('='*50)
            # Keep running to hold position
            return


def main():
    rclpy.init()
    node = AutoDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
