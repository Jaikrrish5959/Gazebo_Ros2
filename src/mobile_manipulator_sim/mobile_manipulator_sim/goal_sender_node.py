#!/usr/bin/env python3
"""
Goal Sender Node - Sends a navigation goal to Nav2 to reach the target box.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
import math

class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender_node')
        
        # Parameters for target position
        self.declare_parameter('target_x', 5.0)
        self.declare_parameter('target_y', 0.0)
        
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Sequence: Initialize -> Wait -> Send Goal
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = "INIT" # INIT, WAIT, SEND, DONE
        self.wait_count = 0
        
        self.get_logger().info(f'Goal Sender initialized. Target: ({self.target_x}, {self.target_y})')

    def timer_callback(self):
        if self.state == "INIT":
            self.publish_initial_pose()
            self.state = "WAIT"
            self.get_logger().info('Sent initial pose. Waiting for AMCL...')
            
        elif self.state == "WAIT":
            self.wait_count += 1
            if self.wait_count > 5: # Wait 5 seconds
                self.state = "SEND"
                
        elif self.state == "SEND":
            self.send_goal()
            self.state = "DONE"
            self.timer.cancel()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -2.0 # Default spawn location for TurtleBot3 World
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0
        # Covariance is auto-initialized to 0, which is fine for exact start
        self.initial_pose_pub.publish(msg)

    def send_goal(self):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available, will retry...')
            return
        
        self.goal_sent = True
        self.timer.cancel()
        
        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.target_x
        goal_msg.pose.pose.position.y = self.target_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # Face forward
        
        self.get_logger().info(f'Sending goal to ({self.target_x}, {self.target_y})')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        current_pose = feedback_msg.feedback.current_pose.pose.position
        distance = math.sqrt(
            (current_pose.x - self.target_x)**2 + 
            (current_pose.y - self.target_y)**2
        )
        self.get_logger().info(f'Distance remaining: {distance:.2f}m', throttle_duration_sec=2.0)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached! Robot has arrived at target location.')
        else:
            self.get_logger().warn(f'Goal failed with status: {result.status}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
