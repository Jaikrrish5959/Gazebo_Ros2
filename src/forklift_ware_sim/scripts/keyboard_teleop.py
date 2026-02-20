#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        # Drive publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Lift/Shift State
        self.shift_pos = 0.0
        self.lift_pos = 0.0
        self.shift_step = 0.01
        self.lift_step = 0.02
        self.shift_min = -0.1
        self.shift_max = 0.1
        self.lift_min = 0.0
        self.lift_max = 0.25
        
        # Drive State
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.linear_step = 0.5
        self.angular_step = 1.0
        
        self.print_instructions()
        
    def print_instructions(self):
        print("\n========================================")
        print("  Forklift Controls")
        print("----------------------------------------")
        print("  DRIVING (WASD):")
        print("    W / S : Forward / Backward")
        print("    A / D : Turn Left / Right")
        print("    Space : Stop Driving")
        print("")
        print("  FORK CONTROL (Arrows):")
        print("    ↑ / ↓ : Lift Up / Down")
        print("    ← / → : Shift Left / Right")
        print("")
        print("  r : Reset Fork Position")
        print("  q : Quit")
        print("========================================\n")

    def get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lift', 'shift']
        msg.position = [self.lift_pos, self.shift_pos]
        self.joint_pub.publish(msg)
        
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(msg)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    # Arrow Keys (Lift/Shift)
                    if key == '\x1b':
                        next1 = sys.stdin.read(1)
                        next2 = sys.stdin.read(1)
                        if next1 == '[':
                            if next2 == 'A': # Up - Lift Up
                                self.lift_pos = min(self.lift_pos + self.lift_step, self.lift_max)
                            elif next2 == 'B': # Down - Lift Down
                                self.lift_pos = max(self.lift_pos - self.lift_step, self.lift_min)
                            elif next2 == 'C': # Right - Shift Right
                                self.shift_pos = min(self.shift_pos + self.shift_step, self.shift_max)
                            elif next2 == 'D': # Left - Shift Left
                                self.shift_pos = max(self.shift_pos - self.shift_step, self.shift_min)
                            self.publish_joints()
                            
                    # WASD (Drive)
                    elif key in ['w', 'W']:
                        self.linear_vel = self.linear_step
                        self.publish_cmd_vel()
                    elif key in ['s', 'S']:
                        self.linear_vel = -self.linear_step
                        self.publish_cmd_vel()
                    elif key in ['a', 'A']:
                        self.angular_vel = self.angular_step
                        self.publish_cmd_vel()
                    elif key in ['d', 'D']:
                        self.angular_vel = -self.angular_step
                        self.publish_cmd_vel()
                    elif key == ' ': # Stop
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        self.publish_cmd_vel()
                        
                    # Reset
                    elif key in ['r', 'R']:
                        self.shift_pos = 0.0
                        self.lift_pos = 0.0
                        self.publish_joints()
                        
                    # Quit
                    elif key in ['q', 'Q', '\x03']:
                        break
                
                # Continuously publish velocity to prevent timeout
                self.publish_cmd_vel()
                
                # Spin gently
                rclpy.spin_once(self, timeout_sec=0.1)
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # Pub stop cmd before exiting
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.publish_cmd_vel()

def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
