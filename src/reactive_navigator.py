#!/usr/bin/env python3
"""
Manual Teleop Navigator with Safety Stop.

Reads manual /cmd_vel from the keyboard script, checks LiDAR for collisions,
and translates safe commands to /base_velocity_controller/commands.

If an obstacle is too close while trying to move, it forces a stop
and blinks a warning light marker in RViz.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker


# Tuning
SAFETY_DIST = 0.70    # Hard stop if trying to move blocked
NUM_WHEELS  = 8       # KELO 8-wheel drive
SPEED_GAIN  = 8.0     # scalar to convert cmd_vel (Twist) to wheel rad/s

class SafetyTeleop(Node):
    def __init__(self):
        super().__init__('reactive_navigator')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self.vel_pub     = self.create_publisher(Float64MultiArray, '/base_velocity_controller/commands', 10)
        self.status_pub  = self.create_publisher(String, '/nav_status', 10)
        self.light_pub   = self.create_publisher(Marker, '/safety_light', 10)

        self.latest_scan = None
        self.cmd_linear  = 0.0
        self.cmd_angular = 0.0

        self.is_blocked  = False
        self.blink_state = False

        self.timer = self.create_timer(0.05, self._loop) # 20 Hz
        self.blink_timer = self.create_timer(0.25, self._blink_loop) # 4 Hz blink

        self.get_logger().info("🛡️ Manual Control Active — Waiting for /cmd_vel")

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _cmd_cb(self, msg: Twist):
        self.cmd_linear  = msg.linear.x
        self.cmd_angular = msg.angular.z

    def _loop(self):
        if self.latest_scan is None:
            return

        # Check front and rear distances
        front = self._sector(self.latest_scan, -40, 40)
        rear  = self._sector(self.latest_scan, 140, 220)

        # Check if the user is trying to drive into an obstacle
        moving_forward  = self.cmd_linear > 0.01
        moving_backward = self.cmd_linear < -0.01

        blocked_front = (front < SAFETY_DIST) and moving_forward
        blocked_rear  = (rear  < SAFETY_DIST) and moving_backward

        if blocked_front or blocked_rear:
            self.is_blocked = True
            self._stop()
            self._status("⚠ BLOCKED! Safety Stop Engaged.")
        else:
            self.is_blocked = False
            self._drive_diff(self.cmd_linear, self.cmd_angular)
            if self.cmd_linear != 0 or self.cmd_angular != 0:
                self._status(f"Moving: lin={self.cmd_linear:.2f}, ang={self.cmd_angular:.2f}")

    def _blink_loop(self):
        """Blinks a bright red light marker if blocked, else turns off."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns = 'safety_strobe'
        marker.id = 0
        marker.type = Marker.CYLINDER
        
        # Position slightly above robot base
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.1

        if self.is_blocked:
            self.blink_state = not self.blink_state
            marker.action = Marker.ADD
            # Red light flashing
            if self.blink_state:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.3
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
        else:
            # Turn light off (delete)
            marker.action = Marker.DELETE

        self.light_pub.publish(marker)

    def _drive_diff(self, lin: float, ang: float):
        # Differential kinematics approximation for 8-wheel velocity input
        left_vel  = (lin - ang) * SPEED_GAIN
        right_vel = (lin + ang) * SPEED_GAIN

        msg = Float64MultiArray()
        # [left]*4 + [right]*4  assuming wheels are mapped this way
        msg.data = [left_vel] * 4 + [right_vel] * 4
        self.vel_pub.publish(msg)

    def _stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * NUM_WHEELS
        self.vel_pub.publish(msg)

    def _status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _sector(self, scan: LaserScan, deg_start, deg_end) -> float:
        n = len(scan.ranges)
        if n == 0:
            return float('inf')
        a_start = math.radians(deg_start)
        a_end   = math.radians(deg_end)
        i0 = int((a_start - scan.angle_min) / scan.angle_increment)
        i1 = int((a_end   - scan.angle_min) / scan.angle_increment)
        i0 = max(0, min(i0, n - 1))
        i1 = max(0, min(i1, n - 1))
        if i0 > i1:
            i0, i1 = i1, i0
        best = float('inf')
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if not (math.isinf(r) or math.isnan(r)) and r > scan.range_min:
                best = min(best, r)
        return best


def main(args=None):
    rclpy.init(args=args)
    node = SafetyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
