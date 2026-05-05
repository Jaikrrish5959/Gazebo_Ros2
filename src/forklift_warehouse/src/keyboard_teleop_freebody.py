#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select

class KeyboardTeleopFreeBody(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_freebody')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint positions
        self.shift_pos = 0.0
        self.lift_pos = 0.0
        
        # Joint limits
        self.shift_min = -0.1
        self.shift_max = 0.15
        self.lift_min = 0.0
        self.lift_max = 0.5
        
        # Step size for each key press
        self.shift_step = 0.01
        self.lift_step = 0.02
        
        self.get_logger().info('Keyboard Teleop FreeBody Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  ↑/↓ : Lift up/down')
        self.get_logger().info('  ←/→ : Shift left/right')
        self.get_logger().info('  r   : Reset to default position')
        self.get_logger().info('  q   : Quit')
        
    def get_key(self):
        """Get a single keypress from terminal"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def publish_shift(self):
        """Publish shift joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shift']
        msg.position = [self.shift_pos]
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Shift: {self.shift_pos:.3f}')
    
    def publish_lift(self):
        """Publish lift joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lift']
        msg.position = [self.lift_pos]
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Lift: {self.lift_pos:.3f}')
    
    def reset_position(self):
        """Reset to default position"""
        self.shift_pos = 0.0
        self.lift_pos = 0.0
        self.publish_shift()
        self.publish_lift()
        self.get_logger().info('Reset to default position')
    
    def run(self):
        """Main control loop"""
        # Save terminal settings
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            self.get_logger().info('Ready for keyboard input...')
            
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    # Arrow keys send escape sequences
                    if key == '\x1b':  # ESC
                        next1 = sys.stdin.read(1)
                        next2 = sys.stdin.read(1)
                        
                        if next1 == '[':
                            if next2 == 'A':  # Up arrow
                                self.lift_pos = min(self.lift_pos + self.lift_step, self.lift_max)
                                self.publish_lift()
                            elif next2 == 'B':  # Down arrow
                                self.lift_pos = max(self.lift_pos - self.lift_step, self.lift_min)
                                self.publish_lift()
                            elif next2 == 'C':  # Right arrow
                                self.shift_pos = min(self.shift_pos + self.shift_step, self.shift_max)
                                self.publish_shift()
                            elif next2 == 'D':  # Left arrow
                                self.shift_pos = max(self.shift_pos - self.shift_step, self.shift_min)
                                self.publish_shift()
                    
                    elif key == 'r' or key == 'R':
                        self.reset_position()
                    
                    elif key == 'q' or key == 'Q':
                        self.get_logger().info('Quitting...')
                        break
                    
                    elif key == '\x03':  # Ctrl+C
                        break
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    
    teleop = KeyboardTeleopFreeBody()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
