import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import subprocess
import random
import math
import time

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.target_spawned = False
        self.latest_scan = None
        self.target_name = "target_object"
        self.world_name = "minimal_world"
        
        # Initial spawn
        self.spawn_target(3.0, 0.0)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Target Chaser Node Started")

    def spawn_target(self, x, y):
        sdf_xml = f"""
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="{self.target_name}">
            <static>true</static>
            <pose>{x} {y} 0.5 0 0 0</pose>
            <link name="link">
              <visual name="visual">
                <geometry><cylinder radius="0.3" length="1.0"/></geometry>
                <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
              </visual>
              <collision name="collision">
                <geometry><cylinder radius="0.3" length="1.0"/></geometry>
              </collision>
            </link>
          </model>
        </sdf>
        """
        # Remove if exists (best effort)
        # We rely on create handling name collisions or just ignoring errors, 
        # but for simplicity we assume it's fresh or we use 'create' once.
        # Actually, if we restart the node, the object might persist. 
        # Let's try to remove it first.
        subprocess.run(["gz", "service", "-s", f"/world/{self.world_name}/remove", 
                        "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", 
                        "--timeout", "1000", "--req", f'name: "{self.target_name}" type: MODEL'], 
                       capture_output=True)

        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", self.world_name,
            "-string", sdf_xml,
            "-name", self.target_name,
            "-x", str(x), "-y", str(y), "-z", "0.5"
        ]
        subprocess.Popen(cmd)
        self.target_spawned = True
        self.get_logger().info(f"Spawning target at {x}, {y}")

    def move_target(self):
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)
        
        req = f'name: "{self.target_name}", position: {{x: {x}, y: {y}, z: 0.5}}'
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000", "--req", req
        ]
        
        self.get_logger().info(f"Moving target to {x:.2f}, {y:.2f}")
        subprocess.run(cmd, capture_output=True)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def control_loop(self):
        if not self.latest_scan:
            return

        # Find closest object in scan
        ranges = self.latest_scan.ranges
        min_dist = float('inf')
        min_idx = -1
        
        for i, r in enumerate(ranges):
            if not math.isinf(r) and r > 0.1: # Filter valid ranges
                if r < min_dist:
                    min_dist = r
                    min_idx = i
        
        twist = Twist()
        
        if min_idx != -1:
            # Calculate angle to target
            angle = self.latest_scan.angle_min + (min_idx * self.latest_scan.angle_increment)
            
            # Check if reached
            if min_dist < 1.0:
                self.get_logger().info("Target Reached!")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_vel.publish(twist)
                self.move_target()
                # Pause briefly to let scan update/target move
                time.sleep(0.5) 
                return

            # Proportional Control
            twist.linear.x = 0.5
            twist.angular.z = 1.0 * angle  # P-gain for rotation
            
            # Limit angular speed
            twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)
        
        else:
            # Search mode: Rotate to find something
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        self.pub_vel.publish(twist)

def main():
    rclpy.init()
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()