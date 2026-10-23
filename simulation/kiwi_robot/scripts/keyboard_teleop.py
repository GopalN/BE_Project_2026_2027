#!/usr/bin/env python3
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

INSTRUCTIONS = """
Kiwi robot keyboard teleop
---------------------------
w : move forward
s : move backward
a : strafe left
d : strafe right
r : rotate clockwise
q : quit

Any other key stops the robot.
CTRL-C to exit.
"""

LINEAR_SPEED = 0.5   # m/s
ANGULAR_SPEED = 1.0  # rad/s

# key -> (linear_x, linear_y, angular_z)
KEY_BINDINGS = {
    'w': (LINEAR_SPEED, 0.0, 0.0),
    's': (-LINEAR_SPEED, 0.0, 0.0),
    'a': (0.0, LINEAR_SPEED, 0.0),
    'd': (0.0, -LINEAR_SPEED, 0.0),
    'r': (0.0, 0.0, -ANGULAR_SPEED),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', LINEAR_SPEED)
        self.declare_parameter('angular_speed', ANGULAR_SPEED)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.publisher.publish(msg)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = KeyboardTeleop()

    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == 'q' or key == '\x03':
                break
            if key in KEY_BINDINGS:
                lx, ly, az = KEY_BINDINGS[key]
                vx = lx * (node.linear_speed / LINEAR_SPEED) if lx else 0.0
                vy = ly * (node.linear_speed / LINEAR_SPEED) if ly else 0.0
                wz = az * (node.angular_speed / ANGULAR_SPEED) if az else 0.0
                node.publish_twist(vx, vy, wz)
            elif key == '':
                continue
            else:
                node.publish_twist(0.0, 0.0, 0.0)
    except Exception as exc:
        node.get_logger().error(f'Teleop loop error: {exc}')
    finally:
        node.publish_twist(0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
