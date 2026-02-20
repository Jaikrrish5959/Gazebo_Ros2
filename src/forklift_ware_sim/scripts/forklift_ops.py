#!/usr/bin/env python3
"""
forklift_ops.py  –  Forklift Warehouse Operations Demo
=======================================================
Runs a scripted sequence of operations to demonstrate the forklift
working inside the warehouse:

  Phase 1 – Drive forward along the main aisle toward the target pallet.
  Phase 2 – Lower forks to insertion height, slide under the pallet.
  Phase 3 – Lift forks to raise the pallet off the floor.
  Phase 4 – Reverse with pallet to a drop-off location.
  Phase 5 – Lower forks to set the pallet down.
  Phase 6 – Back away and stop.

Run after launching the simulation:
  ros2 run forklift_ware_sim forklift_ops.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time


class ForkliftOps(Node):
    """Performs a pre-scripted sequence of warehouse operations."""

    def __init__(self):
        super().__init__('forklift_ops')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub    = self.create_publisher(Float64, '/fork_lift_shift/lift_cmd', 10)
        self.shift_pub   = self.create_publisher(Float64, '/fork_lift_shift/shift_cmd', 10)

        self.get_logger().info('ForkliftOps node started. Beginning warehouse demo...')

    # ------------------------------------------------------------------ #
    # Helper methods                                                       #
    # ------------------------------------------------------------------ #

    def drive(self, linear_x: float, angular_z: float, duration: float):
        """Publish a Twist command for `duration` seconds."""
        msg = Twist()
        msg.linear.x  = linear_x
        msg.angular.z = angular_z
        end = time.time() + duration
        rate = self.create_rate(10)  # 10 Hz
        while time.time() < end:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def stop(self):
        """Send a zero Twist to halt the forklift."""
        self.cmd_vel_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def set_lift(self, position: float, hold: float = 1.5):
        """Set lift joint target position (0.0 = down, 0.18 = max up) and hold."""
        msg      = Float64()
        msg.data = position
        end      = time.time() + hold
        rate     = self.create_rate(10)
        while time.time() < end:
            self.lift_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_shift(self, position: float, hold: float = 1.0):
        """Shift the fork carriage laterally (-0.1 … +0.1 m)."""
        msg      = Float64()
        msg.data = position
        end      = time.time() + hold
        while time.time() < end:
            self.shift_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def pause(self, seconds: float):
        """Spin without moving for `seconds`."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ------------------------------------------------------------------ #
    # Main operation sequence                                              #
    # ------------------------------------------------------------------ #

    def run_demo(self):
        self.get_logger().info('── Phase 0: Waiting for simulation to stabilise ──')
        self.pause(2.0)

        # ---------- Phase 1: Drive up the aisle toward the pallet ----------
        self.get_logger().info('── Phase 1: Driving forward to pallet ──')
        self.drive(linear_x=0.5, angular_z=0.0, duration=6.0)
        self.pause(0.5)

        # ---------- Phase 2: Lower forks to floor level (insertion) ---------
        self.get_logger().info('── Phase 2: Lowering forks for insertion ──')
        self.set_lift(0.0, hold=2.0)
        self.pause(0.5)

        # ---------- Phase 3: Slowly slide forks under pallet ----------------
        self.get_logger().info('── Phase 3: Inserting forks under pallet ──')
        self.drive(linear_x=0.2, angular_z=0.0, duration=2.5)
        self.pause(0.5)

        # ---------- Phase 4: Lift forks to raise pallet ----------------------
        self.get_logger().info('── Phase 4: Lifting pallet ──')
        self.set_lift(0.15, hold=3.0)
        self.pause(1.0)

        # ---------- Phase 5: Reverse to drop-off zone ------------------------
        self.get_logger().info('── Phase 5: Reversing to drop-off zone ──')
        self.drive(linear_x=-0.4, angular_z=0.0, duration=5.0)
        self.pause(0.5)

        # ---------- Optional: Turn to face drop-off shelf -------------------
        self.get_logger().info('── Phase 5b: Turning to storage area ──')
        self.drive(linear_x=0.0, angular_z=0.4, duration=2.5)
        self.pause(0.5)

        # ---------- Phase 6: Lower pallet at drop-off location ---------------
        self.get_logger().info('── Phase 6: Setting pallet down ──')
        self.set_lift(0.0, hold=2.0)
        self.pause(1.0)

        # ---------- Phase 7: Back away from pallet ---------------------------
        self.get_logger().info('── Phase 7: Backing away ──')
        self.drive(linear_x=-0.3, angular_z=0.0, duration=2.0)
        self.stop()

        self.get_logger().info('✔  Warehouse operations demo complete!')


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftOps()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
