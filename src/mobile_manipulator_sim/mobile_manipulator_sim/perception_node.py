import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import math

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Pose, '/model/target_box/pose', self.target_callback, 10)
        
        # Publishers
        self.obstacle_pub = self.create_publisher(Float32MultiArray, '/robot/obstacle_info', 10)
        self.target_pub = self.create_publisher(Pose, '/robot/target_pose', 10)
        
        self.get_logger().info('Perception Node Started')

    def scan_callback(self, msg):
        # Divide scan into 3 sectors: Left, Center, Right
        # Scan is -3.14 to 3.14 (360 deg). Front is 0.
        # Right: -90 to -30 (-1.57 to -0.52)
        # Center: -30 to 30 (-0.52 to 0.52)
        # Left: 30 to 90 (0.52 to 1.57)
        
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        def get_min_dist(min_angle, max_angle):
            min_idx = int((min_angle - angle_min) / angle_inc)
            max_idx = int((max_angle - angle_min) / angle_inc)
            
            # Clamp indices
            min_idx = max(0, min(min_idx, len(ranges)-1))
            max_idx = max(0, min(max_idx, len(ranges)-1))
            
            if min_idx >= max_idx:
                return float('inf')
                
            # Filter out inf/nan
            valid_ranges = [r for r in ranges[min_idx:max_idx] if not math.isinf(r) and not math.isnan(r) and r > 0.1]
            return min(valid_ranges) if valid_ranges else float('inf')

        right_dist = get_min_dist(-1.0, -0.3)
        center_dist = get_min_dist(-0.3, 0.3)
        left_dist = get_min_dist(0.3, 1.0)
        
        # Publish [left, center, right]
        msg_out = Float32MultiArray()
        msg_out.data = [left_dist, center_dist, right_dist]
        self.obstacle_pub.publish(msg_out)

    def target_callback(self, msg):
        # Pass through the target pose
        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
