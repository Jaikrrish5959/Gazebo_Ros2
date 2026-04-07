#!/usr/bin/env python3
"""
Box Transport: Pick from shelf and reverse back.

Freddy drives forward to shelf_0, extends left arm to pick a box,
then reverses straight back 2 steps to clear the shelf aisle.

Usage:
  ros2 run forklift_warehouse box_transport.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray

import time


class BoxTransport(Node):

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
    POSE_HOME = [0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57]
    POSE_REACH = [0.0, 0.0, 0.0, -1.57, 0.0, -1.0, 0.0]
    POSE_GRASP = [0.0, 0.3, 0.0, -1.2, 0.0, -1.2, 0.0]
    POSE_CARRY = [0.0, 0.5, 3.14, -2.0, 0.0, 0.7, 1.57]

    def __init__(self):
        super().__init__('box_transport')

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

        self.get_logger().info('=== Box Transport Node Ready ===')

    def move_arm(self, pose, label, secs=4.0):
        """Move the left arm to a named pose."""
        self.get_logger().info(f'Arm → {label}')

        if not self.left_arm_action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm action server unavailable!')
            return False

        traj = JointTrajectory()
        traj.joint_names = self.LEFT_ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = pose
        pt.velocities = [0.0] * 7
        pt.time_from_start = Duration(seconds=secs).to_msg()
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.left_arm_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'{label} goal rejected!')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=secs + 10.0)
        self.get_logger().info(f'Arm → {label} ✓')
        return True

    def drive(self, speed, duration, label):
        """Drive all wheels at the same velocity for straight-line motion."""
        self.get_logger().info(f'Base: {label} ({speed:.1f} rad/s, {duration:.1f}s)')

        msg = Float64MultiArray()
        msg.data = [speed] * 8

        start = time.time()
        rate = self.create_rate(50)
        while time.time() - start < duration:
            self.base_vel_pub.publish(msg)
            rate.sleep()

        # Stop
        msg.data = [0.0] * 8
        self.base_vel_pub.publish(msg)
        time.sleep(0.5)
        self.get_logger().info(f'Base: {label} ✓')

    def run(self):
        """Execute: home → drive to shelf → reach → grasp → carry → reverse back."""

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║   SHELF BOX PICKUP — STARTING       ║')
        self.get_logger().info('╚══════════════════════════════════════╝')

        # Step 1: Arm to home
        self.get_logger().info('\n[1/6] Arm → HOME')
        self.move_arm(self.POSE_HOME, 'HOME', secs=3.0)
        time.sleep(1.0)

        # Step 2: Drive forward to the shelf
        self.get_logger().info('\n[2/6] Driving FORWARD to shelf')
        self.drive(speed=2.0, duration=3.0, label='FORWARD to shelf')
        time.sleep(1.0)

        # Step 3: Extend arm to reach position
        self.get_logger().info('\n[3/6] Arm → REACH (extending toward box)')
        self.move_arm(self.POSE_REACH, 'REACH', secs=3.0)
        time.sleep(1.0)

        # Step 4: Move arm to grasp position (contact box)
        self.get_logger().info('\n[4/6] Arm → GRASP (contacting box)')
        self.move_arm(self.POSE_GRASP, 'GRASP', secs=2.0)
        self.get_logger().info('>>> BOX GRASPED <<<')
        time.sleep(1.0)

        # Step 5: Lift and tuck arm into carry position
        self.get_logger().info('\n[5/6] Arm → CARRY (lifting box)')
        self.move_arm(self.POSE_CARRY, 'CARRY', secs=3.0)
        time.sleep(1.0)

        # Step 6: Reverse straight back (2 steps = ~2m)
        self.get_logger().info('\n[6/6] Driving REVERSE (backing away from shelf)')
        self.drive(speed=-2.0, duration=3.0, label='REVERSE 2 steps')

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║   BOX PICKUP COMPLETE!               ║')
        self.get_logger().info('╚══════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = BoxTransport()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Cancelled.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
