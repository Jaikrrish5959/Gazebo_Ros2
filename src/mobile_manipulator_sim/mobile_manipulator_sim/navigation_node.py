import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Subscribe directly to Gazebo topics via bridge
        self.create_subscription(Pose, '/model/target_box/pose', self.target_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot/nav_status', 10)
        
        self.target_pose = None
        self.current_pose = None
        self.obstacle_dists = [float('inf'), float('inf'), float('inf')]
        
        self.grasp_distance = 0.45
        self.state = "NAVIGATE"
        
        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Navigation parameters
        self.max_linear_vel = 0.5
        self.max_angular_vel = 2.0
        self.heading_p_gain = 2.5
        
        self.loop_count = 0
        
        self.get_logger().info('Navigation Node Started - Direct subscriptions')

    def target_callback(self, msg):
        self.target_pose = msg

    def scan_callback(self, msg):
        """Process LiDAR scan to get obstacle distances in 3 sectors."""
        ranges = list(msg.ranges)
        n = len(ranges)
        
        if n == 0:
            return
            
        # Filter out invalid readings and self-detection
        min_valid_range = 0.3
        
        def get_sector_min(start_idx, end_idx):
            if start_idx < 0:
                start_idx = 0
            if end_idx > n:
                end_idx = n
            sector = ranges[start_idx:end_idx]
            valid = [r for r in sector if r > min_valid_range and r < msg.range_max and not math.isinf(r) and not math.isnan(r)]
            return min(valid) if valid else float('inf')
        
        # Front is at the middle of the array for -pi to +pi scan
        samples_per_30deg = n // 12
        center = n // 2
        
        sector_width = samples_per_30deg
        
        right_start = max(0, center - 2*sector_width)
        right_end = center - sector_width//2
        center_start = center - sector_width//2
        center_end = center + sector_width//2
        left_start = center + sector_width//2
        left_end = min(n, center + 2*sector_width)
        
        left = get_sector_min(left_start, left_end)
        center_dist = get_sector_min(center_start, center_end)
        right = get_sector_min(right_start, right_end)
        
        self.obstacle_dists = [left, center_dist, right]

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        self.loop_count += 1
        twist = Twist()
        
        # Log every 20 iterations (2 seconds)
        if self.loop_count % 20 == 0:
            has_target = self.target_pose is not None
            has_odom = self.current_pose is not None
            self.get_logger().info(f'Loop {self.loop_count}: target={has_target}, odom={has_odom}')
        
        # Check if we have the required data
        if self.target_pose is None:
            if self.loop_count % 50 == 0:
                self.get_logger().warn('Waiting for target pose...')
            # Still publish zero velocity
            self.cmd_vel_pub.publish(twist)
            return
            
        if self.current_pose is None:
            if self.loop_count % 50 == 0:
                self.get_logger().warn('Waiting for odometry...')
            self.cmd_vel_pub.publish(twist)
            return
        
        # Calculate distance and angle to target
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        # Log progress every 2 seconds
        if self.loop_count % 20 == 0:
            self.get_logger().info(f'Distance to target: {dist:.2f}m')
        
        # Check if arrived
        if dist < self.grasp_distance:
            self.state = "ARRIVED"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.status_pub.publish(String(data="ARRIVED"))
            if self.loop_count % 20 == 0:
                self.get_logger().info('ARRIVED at target!')
            return
        else:
            self.state = "NAVIGATE"
            self.status_pub.publish(String(data="MOVING"))

        # Calculate heading to target
        target_heading = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        heading_error = self.normalize_angle(target_heading - current_yaw)
        
        # Obstacle avoidance
        left, center, right = self.obstacle_dists
        obstacle_threshold = 1.2
        critical_threshold = 0.6
        
        angular_correction = 0.0
        speed_factor = 1.0
        
        # Check for obstacles
        if center < critical_threshold:
            speed_factor = 0.0
            if left > right:
                angular_correction = self.max_angular_vel
            else:
                angular_correction = -self.max_angular_vel
        elif center < obstacle_threshold:
            speed_factor = 0.5
            if left > right:
                angular_correction = 1.0
            else:
                angular_correction = -1.0
        elif left < 0.8:
            angular_correction = -0.5
        elif right < 0.8:
            angular_correction = 0.5
        
        # Motion control
        if abs(heading_error) > 0.4:
            # Large heading error: rotate in place with slight forward motion
            twist.linear.x = 0.05
            twist.angular.z = self.heading_p_gain * heading_error
        else:
            # Move towards target with heading correction
            twist.linear.x = self.max_linear_vel * speed_factor
            twist.angular.z = self.heading_p_gain * heading_error + angular_correction
        
        # Clamp velocities
        twist.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, twist.angular.z))
        twist.linear.x = max(0.0, min(self.max_linear_vel, twist.linear.x))
        
        # ALWAYS publish velocity
        self.cmd_vel_pub.publish(twist)
        
        if self.loop_count % 20 == 0:
            self.get_logger().info(f'cmd_vel: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
