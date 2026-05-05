#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Freddy Arrow Key Teleop
---------------------------
Use Arrow Keys to move:
   ↑ : Move Forward
   ↓ : Move Backward
   ← : Turn Left
   → : Turn Right

Space or 's' to force stop.
Ctrl-C to quit.
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':  # Escape sequence
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 1.0
        self.turn = 1.0

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(msg)
        try:
            while rclpy.ok():
                key = get_key(settings)
                twist = Twist()

                if key == '\x1b[A':   # Up
                    twist.linear.x = self.speed
                elif key == '\x1b[B': # Down
                    twist.linear.x = -self.speed
                elif key == '\x1b[C': # Right
                    twist.angular.z = -self.turn
                elif key == '\x1b[D': # Left
                    twist.angular.z = self.turn
                elif key == ' ' or key == 's':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':   # Ctrl-C
                    break
                
                self.pub.publish(twist)
        finally:
            twist = Twist()
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
