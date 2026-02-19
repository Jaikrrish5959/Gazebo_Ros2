import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TugbotController(Node):
    def __init__(self):
        super().__init__('tugbot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Tugbot Controller has been started')
        self.count = 0

    def timer_callback(self):
        msg = Twist()
        # Move forward
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        
        # Turn after some time
        if self.count > 50:
             msg.linear.x = 0.0
             msg.angular.z = 0.5
        
        self.publisher_.publish(msg)
        self.count += 1
        # Reset cycle
        if self.count > 100:
            self.count = 0

def main(args=None):
    rclpy.init(args=args)
    node = TugbotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
